"""Internal 500 Hz transition engine composed by :class:`MotionController`."""
from __future__ import annotations

import threading
import uuid
from collections.abc import Sequence
from dataclasses import replace

from robot_contracts.action_chunks import ChunkAnchor
from robot_contracts.models import ContractError
from robot_contracts.trajectory.spline import ReferenceSample
from robot_trajectory.follower import PreparedFollowerPlan
from robot_trajectory.sampling import sample_reference_interval

from robot_backend_sim.config.motion_config import ControlConfig
from robot_backend_sim.config.stream_config import StreamConfig
from robot_backend_sim.control.braking import BrakingController
from robot_backend_sim.motion.motion_math import (
    deadline_decision,
    normalize_reference,
    select_stream_reference,
    sim_time_ns,
    validate_controller_inputs,
    validate_first_chunk,
    validate_wall,
    zero_sample,
)
from robot_backend_sim.motion.motion_snapshot import SnapshotContext, Snapshotter, TrackingState
from robot_backend_sim.motion.motion_transitions import (
    begin_stop,
    failed_terminal,
    resolve_chunk_anchor,
    snapshot_for_call,
    stop_terminal,
    stream_boundary,
)
from robot_backend_sim.motion.motion_types import (
    TERMINAL_PHASES,
    AppliedStep,
    BoundarySelection,
    FailedReason,
    MotionResult,
    MotionSnapshot,
    Phase,
    StopReason,
)
from robot_backend_sim.motion.stream_schedule import (
    ChunkEvent,
    ChunkSubmission,
    StreamSchedule,
    boundary_rejection,
)
from robot_backend_sim.physics.contacts import (
    GripperCompletion,
    GripperContactEvidence,
    resolve_contact_binding,
)
from robot_backend_sim.physics.gripper_contact import GripperContactTracker
from robot_backend_sim.physics.robot_io import Measurement, RobotIO
from robot_backend_sim.physics.scene_container import CompiledScene

_invalid = ContractError

class MotionRuntime:
    def __init__(
        self,
        scene: CompiledScene,
        io: RobotIO,
        config: ControlConfig,
        initial_reference: Sequence[float] | None = None,
        *,
        stream_config: StreamConfig | None = None,
    ) -> None:
        self._owner_thread = threading.current_thread()
        self._scene, self._io, self._config = scene, io, config
        self._binding = resolve_contact_binding(
            scene.model, scene.contact_object_names, io.finger_bodies
        )
        self._step_ns, self._hold_positions = validate_controller_inputs(
            scene, io, config, stream_config, initial_reference
        )
        self._stream_config = stream_config
        unload_ns = (
            0 if stream_config is None else round(stream_config.contact_unload_timeout_s * 1e9)
        )
        self._braking = BrakingController(config)
        self._gripper = GripperContactTracker(
            scene, self._binding, config, io.joint_names,
            contact_unload_timeout_ns=unload_ns,
        )
        self._snapshots = Snapshotter(self._gripper)
        self._tracking = TrackingState()
        self._schedule: StreamSchedule | None = None
        self._phase, self._motion_seq = Phase.IDLE, 0
        self._accepted_wall_ns = self._started_wall_ns = self._started_sim_ns = None
        self._last_wall_ns = self._result = self._terminal_snapshot = self._last_snapshot = None
        self._applied = self._stop_requested_wall_ns = self._stop_reason = None
        self._latched_failure = self._stable_since_ns = self._stable_signature = None
        self._instance_id = f"stream-{uuid.uuid4().hex}"
        self._stream_initial_positions = self._stream_final_positions = None
        self._stream_final_targets = self._stream_final_end_ns = None
        self._events: list[ChunkEvent] = []
        self._last_progress_wall_ns = self._last_progress_sim_ns = None

    @property
    def phase(self) -> Phase:
        self._check_thread()
        return self._phase

    @property
    def hold_positions(self) -> tuple[float, ...]:
        self._check_thread()
        return self._hold_positions

    @property
    def result(self) -> MotionResult | None:
        self._check_thread()
        return self._result

    @property
    def terminal_snapshot(self) -> MotionSnapshot | None:
        self._check_thread()
        return self._terminal_snapshot

    def chunk_anchor(self, start_sim_time_ns: int | None = None) -> ChunkAnchor:
        """Return an immutable preview/successor hand-off anchor."""
        self._check_thread()
        return resolve_chunk_anchor(
            phase=self._phase, schedule=self._schedule, stream_config=self._stream_config,
            instance_id=self._instance_id, motion_seq=self._motion_seq,
            hold_positions=self._hold_positions, profile_digest=self._io.profile.digest,
            now_sim_ns=sim_time_ns(self._scene.data), step_ns=self._step_ns,
            start_sim_time_ns=start_sim_time_ns,
        )

    def start_stream(self, first: PreparedFollowerPlan, wall_time_ns: int) -> MotionSnapshot:
        """Admit a first bounded-follower plan and hold until its aligned start."""
        self._check_thread()
        self._events = []
        wall = validate_wall(wall_time_ns, self._last_wall_ns)
        if self._stream_config is None:
            raise _invalid("STREAM_NOT_CONFIGURED", "MotionController has no stream config")
        now_sim_ns = sim_time_ns(self._scene.data)
        validate_first_chunk(
            first, phase=self._phase, instance_id=self._instance_id,
            motion_seq=self._motion_seq, stream_config=self._stream_config,
            step_ns=self._step_ns, now_sim_ns=now_sim_ns,
            hold_positions=self._hold_positions, profile_digest=self._io.profile.digest,
            profile_joints=self._io.profile.joints,
            max_accelerations=self._config.max_accelerations,
        )
        self._last_wall_ns = wall
        self._motion_seq += 1
        self._result = self._terminal_snapshot = self._applied = None
        self._braking.reset()
        self._gripper.reset()
        self._tracking.reset()
        self._tracking.violation_observed_sim_ns = now_sim_ns
        self._stop_requested_wall_ns = self._stop_reason = self._latched_failure = None
        self._stable_since_ns = self._stable_signature = None
        self._stream_initial_positions = first.anchor.reference.positions
        self._stream_final_positions = self._stream_final_targets = self._stream_final_end_ns = None
        schedule = StreamSchedule(first.anchor.stream_id, self._motion_seq)
        schedule.install_start(first)
        self._schedule, self._phase = schedule, Phase.ACCEPTED
        self._accepted_wall_ns = self._last_progress_wall_ns = wall
        self._started_wall_ns = self._started_sim_ns = None
        self._last_progress_sim_ns = now_sim_ns
        self._event("ACCEPTED", first, now_sim_ns)
        measurement = self._io.measure()
        snapshot = self._snapshots.build(
            self._context(), phase=Phase.ACCEPTED, reason="",
            reference=zero_sample(self._hold_positions), measurement=measurement,
            reference_application_sim_time_ns=measurement.sim_time_ns,
            elapsed_ns=0, stable_for_ns=0, contacts=self._gripper.empty_contacts(),
            reference_kind="HOLD",
        )
        self._last_snapshot = snapshot
        return snapshot

    def queue_chunk(self, successor: PreparedFollowerPlan, wall_time_ns: int) -> ChunkSubmission:
        """Queue at most one validated future successor at the active chunk end."""
        self._check_thread()
        self._events = []
        wall = validate_wall(wall_time_ns, self._last_wall_ns)
        if self._stream_config is None:
            raise _invalid("STREAM_NOT_CONFIGURED", "MotionController has no stream config")
        if not isinstance(successor, PreparedFollowerPlan):
            raise _invalid("INVALID_PREPARED_CHUNK", "successor must be a PreparedFollowerPlan")
        self._last_wall_ns = wall
        self._advance_deadlines(wall)
        schedule = self._schedule
        if self._phase in (Phase.STOPPING, *TERMINAL_PHASES):
            return self._reject(schedule, successor, self._terminal_reason())
        if schedule is None or self._phase not in (Phase.ACCEPTED, Phase.RUNNING, Phase.SETTLING):
            return ChunkSubmission(False, "STALE_STREAM", self._snapshot_for_call())
        now_sim_ns = sim_time_ns(self._scene.data)
        checks = (
            (schedule.active is None, "STREAM_NOT_STARTED"),
            (schedule.pending is not None, "QUEUE_FULL"),
            (schedule.active is not None and schedule.active.settle_at_end, "FINAL_CHUNK"),
            (self._phase is Phase.SETTLING, "FINAL_CHUNK"),
            (successor.duration_grid_ns != self._step_ns, "FOLLOWER_GRID_REQUIRED"),
            (successor.plan.duration_ns % self._step_ns != 0, "INVALID_CHUNK_GRID"),
        )
        for failed, reason in checks:
            if failed:
                return self._reject(schedule, successor, reason, now_sim_ns)
        reason, latches = boundary_rejection(
            schedule.active, successor, stream_id=schedule.stream_id,
            profile_digest=self._io.profile.digest, joints=tuple(self._io.profile.joints),
            max_accelerations=self._config.max_accelerations, now_sim_ns=now_sim_ns,
            min_lead_ns=self._stream_config.min_lead_ns,
            max_observation_age_ns=self._stream_config.max_observation_age_ns,
            step_ns=self._step_ns,
        )
        if reason is not None:
            if latches:
                self._begin_stop(wall, reason, reason)
            return self._reject(schedule, successor, reason, now_sim_ns)
        schedule.install_pending(successor)
        self._event("ACCEPTED", successor, now_sim_ns, "PENDING")
        return ChunkSubmission(True, "ACCEPTED", self._snapshot_for_call())

    def step(self, wall_time_ns: int) -> MotionSnapshot:
        """Apply one reference/hold at simulation time t, then step once to t+dt."""
        self._check_thread()
        self._events = []
        wall = validate_wall(wall_time_ns, self._last_wall_ns)
        self._last_wall_ns = wall
        if self._phase is Phase.IDLE or self._phase in TERMINAL_PHASES:
            return self._hold_step_live()
        if self._advance_deadlines(wall):
            assert self._terminal_snapshot is not None
            return self._terminal_snapshot
        application_sim_ns = sim_time_ns(self._scene.data)
        if self._phase is Phase.STOPPING:
            return self._step_stop(wall, application_sim_ns)
        if self._schedule is not None and self._phase in (
            Phase.ACCEPTED, Phase.RUNNING, Phase.SETTLING
        ):
            return self._step_stream(wall, application_sim_ns)
        raise AssertionError("active motion has no stream schedule")

    def _step_stop(self, wall: int, application_sim_ns: int) -> MotionSnapshot:
        if self._braking.plan is None or self._braking.stop_sim_start_ns is None:
            raise AssertionError("STOPPING without a braking plan and origin")
        if self._started_sim_ns is None:
            self._started_sim_ns, self._started_wall_ns = application_sim_ns, wall
        elapsed_ns = application_sim_ns - self._braking.stop_sim_start_ns
        reference = self._braking.sample(elapsed_ns)
        applied = self._apply_and_measure(reference, application_sim_ns, elapsed_ns, "BRAKE")
        self._record_progress(applied, wall)
        settled, stable_for = self._braking.update_stability(applied.measurement, elapsed_ns)
        if settled:
            self._finish_stop(applied.measurement, elapsed_ns, stable_for)
        if self._phase in TERMINAL_PHASES:
            assert self._terminal_snapshot is not None
            return self._terminal_snapshot
        snapshot = self._applied_snapshot(
            applied, phase=self._phase, reason=self._stop_reason or "", reference_kind="BRAKE",
            stable_for_ns=self._braking.stable_for_ns(applied.measurement),
        )
        self._last_snapshot = snapshot
        return snapshot

    def _step_stream(self, wall: int, application_sim_ns: int) -> MotionSnapshot:
        if self._phase is Phase.ACCEPTED:
            return self._step_stream_start(wall, application_sim_ns)
        schedule = self._schedule
        assert schedule is not None and schedule.active is not None
        choice = select_stream_reference(
            schedule, self._phase, application_sim_ns,
            self._hold_positions, self._stream_final_positions,
        )
        if choice.reference is None:  # underrun: brake from the last real application
            self._begin_stop(wall, FailedReason.UNDERRUN.value, FailedReason.UNDERRUN.value)
            if self._phase is Phase.FAILED:
                assert self._terminal_snapshot is not None
                return self._terminal_snapshot
            assert self._braking.stop_sim_start_ns is not None
            elapsed_ns = application_sim_ns - self._braking.stop_sim_start_ns
            reference = self._braking.sample(elapsed_ns)
        else:
            reference, elapsed_ns = choice.reference, choice.elapsed_ns
        applied = self._apply_and_measure(
            reference, application_sim_ns, elapsed_ns, choice.reference_kind
        )
        if choice.boundary is not None:
            self._apply_stream_boundary(wall, application_sim_ns, schedule.active, choice.boundary)
        return self._finish_stream_step(wall, applied, choice.reference_kind, choice.chunk_seq)

    def _step_stream_start(self, wall: int, application_sim_ns: int) -> MotionSnapshot:
        schedule = self._schedule
        assert schedule is not None and schedule.active is not None
        first = schedule.active
        at_start = application_sim_ns == first.start_sim_time_ns
        reference = first.sample_at(application_sim_ns) if at_start else zero_sample(
            self._hold_positions
        )
        kind = "CHUNK" if at_start else "HOLD"
        applied = self._apply_and_measure(reference, application_sim_ns, 0, kind)
        if at_start:
            self._started_sim_ns, self._started_wall_ns = application_sim_ns, wall
            self._phase, schedule.started = Phase.RUNNING, True
            self._event("STARTED", first, application_sim_ns)
            if first.settle_at_end:
                self._remember_final_chunk(first)
        elif application_sim_ns > first.start_sim_time_ns:
            self._begin_stop(
                wall, FailedReason.STREAM_START_MISSED.value,
                FailedReason.STREAM_START_MISSED.value,
            )
        return self._finish_stream_step(wall, applied, kind, first.seq if at_start else None)

    def _apply_stream_boundary(
        self, wall: int, application_sim_ns: int, active: PreparedFollowerPlan,
        boundary: BoundarySelection,
    ) -> None:
        assert self._schedule is not None
        effect = stream_boundary(self._schedule, active, boundary, application_sim_ns)
        if effect.underrun:
            self._begin_stop(wall, FailedReason.UNDERRUN.value, FailedReason.UNDERRUN.value)
        if effect.hold_positions is not None:
            self._hold_positions = effect.hold_positions
        if effect.final_positions is not None:
            self._stream_final_positions = effect.final_positions
        if effect.final_targets is not None:
            self._stream_final_targets = effect.final_targets
        if effect.final_end_ns is not None:
            self._stream_final_end_ns = effect.final_end_ns
        if effect.phase is Phase.SETTLING:
            self._phase = Phase.SETTLING
        for event in effect.events:
            self._event(*event)  # type: ignore[arg-type]

    def _finish_stream_step(
        self, wall: int, applied: AppliedStep, reference_kind: str, chunk_seq: int | None
    ) -> MotionSnapshot:
        self._record_progress(applied, wall)
        measurement = applied.measurement
        if self._phase is Phase.STOPPING:
            settled, stable_for = self._braking.update_stability(measurement, applied.elapsed_ns)
            if settled:
                self._finish_stop(measurement, applied.elapsed_ns, stable_for)
        elif self._phase in (Phase.RUNNING, Phase.SETTLING):
            assert self._stream_config is not None
            self._update_stream_tracking(applied)
            if (
                self._tracking.violation_since_ns is not None
                and measurement.sim_time_ns - self._tracking.violation_since_ns
                >= self._stream_config.tracking_timeout_ns
            ):
                self._begin_stop(
                    wall, FailedReason.TRACKING_TIMEOUT.value,
                    FailedReason.TRACKING_TIMEOUT.value,
                )
            elif self._phase is Phase.SETTLING:
                self._update_normal_stability(applied)
                assert self._stream_final_end_ns is not None
                if (
                    self._phase is Phase.SETTLING
                    and measurement.sim_time_ns
                    >= self._stream_final_end_ns + self._config.settle_timeout_ns
                ):
                    self._begin_stop(
                        wall, StopReason.TRACKING_TIMEOUT, FailedReason.TRACKING_TIMEOUT
                    )
        if self._phase in TERMINAL_PHASES:
            assert self._terminal_snapshot is not None
            return self._terminal_snapshot
        snapshot = self._applied_snapshot(
            applied, phase=self._phase,
            reason=self._stop_reason if self._phase is Phase.STOPPING else "",
            reference_kind=reference_kind, chunk_seq=chunk_seq,
            stable_for_ns=self._stable_for_ns(measurement),
        )
        self._last_snapshot = snapshot
        return snapshot

    def _remember_final_chunk(self, chunk: PreparedFollowerPlan) -> None:
        self._stream_final_positions = sample_reference_interval(
            chunk.plan, chunk.plan.duration_ns
        ).positions
        self._stream_final_targets = tuple(chunk.window.targets[-1].positions)
        self._stream_final_end_ns = chunk.end_sim_time_ns

    def poll(self, wall_time_ns: int) -> MotionSnapshot:
        """Return state and apply wall-clock deadline transitions without stepping."""
        self._check_thread()
        self._events = []
        wall = validate_wall(wall_time_ns, self._last_wall_ns)
        self._last_wall_ns = wall
        if self._phase is Phase.IDLE:
            measurement = self._io.measure()
            contacts = self._gripper.empty_contacts()
            snapshot = self._snapshots.build(
                self._context(), phase=Phase.IDLE, reason="",
                reference=zero_sample(self._hold_positions), measurement=measurement,
                reference_application_sim_time_ns=measurement.sim_time_ns,
                elapsed_ns=0, stable_for_ns=0, contacts=contacts,
                completions=self._gripper.hold_completions(
                    measurement, contacts, self._hold_positions
                ),
            )
            self._last_snapshot = snapshot
            return snapshot
        if self._phase in TERMINAL_PHASES:
            assert self._terminal_snapshot is not None
            return self._terminal_snapshot
        if self._advance_deadlines(wall):
            assert self._terminal_snapshot is not None
            return self._terminal_snapshot
        if self._phase is Phase.STOPPING:
            return self._sync_stopping_snapshot()
        assert self._last_snapshot is not None
        return self._last_snapshot

    def request_stop(self, reason: str, wall_time_ns: int) -> MotionSnapshot:
        """Request a bounded brake; repeated requests never extend the deadline."""
        self._check_thread()
        self._events = []
        wall = validate_wall(wall_time_ns, self._last_wall_ns)
        if not isinstance(reason, str) or not reason:
            raise _invalid("INVALID_REASON", "stop reason must be a non-empty string")
        if self._phase is Phase.IDLE:
            raise _invalid("MOTION_NOT_STARTED", "call start before request_stop")
        self._last_wall_ns = wall
        if self._phase in TERMINAL_PHASES:
            assert self._terminal_snapshot is not None
            return self._terminal_snapshot
        if self._advance_deadlines(wall):
            assert self._terminal_snapshot is not None
            return self._terminal_snapshot
        if self._phase is Phase.STOPPING:
            return self._sync_stopping_snapshot()
        self._begin_stop(wall, reason, None)
        if self._phase is Phase.FAILED:
            assert self._terminal_snapshot is not None
            return self._terminal_snapshot
        return self._sync_stopping_snapshot()

    def _sync_stopping_snapshot(self) -> MotionSnapshot:
        assert self._last_snapshot is not None
        if self._last_snapshot.phase is Phase.STOPPING:
            return self._last_snapshot
        changes: dict[str, object] = {"phase": Phase.STOPPING, "reason": self._stop_reason or ""}
        if self._events:
            changes["chunk_events"] = tuple(self._events)
        self._last_snapshot = replace(self._last_snapshot, **changes)  # type: ignore[arg-type]
        return self._last_snapshot

    def _check_thread(self) -> None:
        if threading.current_thread() is not self._owner_thread:
            raise _invalid("WRONG_THREAD", "MotionController must be used on its creating thread")


    def _event(self, phase: str, chunk: PreparedFollowerPlan, sim_ns: int, reason: str = "") -> None:  # noqa: E501
        assert self._schedule is not None
        self._events.append(self._schedule.make_event(phase, chunk, sim_ns, reason))

    def _reject(
        self, schedule: StreamSchedule | None, successor: PreparedFollowerPlan,
        reason: str, sim_ns: int | None = None,
    ) -> ChunkSubmission:
        if schedule is not None:
            self._events.append(
                schedule.make_event(
                    "REJECTED", successor,
                    sim_time_ns(self._scene.data) if sim_ns is None else sim_ns, reason,
                )
            )
        return ChunkSubmission(False, reason, self._snapshot_for_call())

    def _terminal_reason(self) -> str:
        if self._phase is Phase.COMPLETED:
            return "STALE_STREAM"
        return (
            self._stop_reason
            or (self._result.reason if self._result is not None else "")
            or self._latched_failure
            or "STOPPING"
        )

    def _context(self) -> SnapshotContext:
        schedule = self._schedule
        return SnapshotContext(
            phase=self._phase, motion_seq=self._motion_seq,
            accepted_wall_ns=self._accepted_wall_ns,
            started_wall_ns=self._started_wall_ns, started_sim_ns=self._started_sim_ns,
            joint_names=tuple(self._io.joint_names),
            stream_id=None if schedule is None else schedule.stream_id,
            hold_positions=self._hold_positions,
            stream_initial_positions=self._stream_initial_positions,
            stream_final_targets=self._stream_final_targets,
            active_plan=None if schedule is None else schedule.active,
            tracking_quality=self._tracking.quality,
            tracking_violation_ns=self._tracking.violation_ns,
            events=tuple(self._events),
        )

    def _active_chunk_seq(self) -> int | None:
        if self._schedule is not None and self._schedule.active is not None:
            return self._schedule.active.seq
        return None

    def _completion_bounds(self) -> tuple[tuple[float, ...], tuple[float, ...]]:
        return self._gripper.completion_bounds(
            self._stream_initial_positions, self._stream_final_targets,
            None if self._schedule is None else self._schedule.active, self._hold_positions,
        )

    def _completions(
        self, measurement: Measurement, contacts: tuple[GripperContactEvidence, ...]
    ) -> tuple[GripperCompletion, ...]:
        return self._gripper.completions(measurement, contacts, *self._completion_bounds())

    def _update_normal_stability(self, applied: AppliedStep) -> None:
        settled, since, signature, stable_for = self._gripper.update_normal_stability(
            applied.measurement, applied.reference,
            self._completions(applied.measurement, applied.contacts),
            self._stream_final_end_ns or 0, self._stable_since_ns, self._stable_signature,
        )
        self._stable_since_ns, self._stable_signature = since, signature
        if not settled:
            return
        self._hold_positions = self._completion_bounds()[1]
        self._phase = Phase.COMPLETED
        snapshot = self._applied_snapshot(
            applied, phase=Phase.COMPLETED, reason="COMPLETED",
            chunk_seq=self._active_chunk_seq(),
            elapsed_ns=applied.application_sim_time_ns - (self._started_sim_ns or 0),
            stable_for_ns=stable_for,
        )
        self._freeze(snapshot, wall=self._last_wall_ns or 0, stop_reason=None)

    def _finish_stop(self, measurement: Measurement, elapsed_ns: int, stable_for: int) -> None:
        assert self._applied is not None
        self._hold_positions = self._braking.hold_positions
        self._phase = Phase.FAILED if self._latched_failure else Phase.STOPPED
        reason = self._latched_failure or self._stop_reason or StopReason.STOP_REQUESTED.value
        snapshot, result = stop_terminal(
            self._snapshots, context=self._context(), applied=self._applied, phase=self._phase,
            reason=reason, elapsed_ns=elapsed_ns, stable_for_ns=stable_for,
            completions=self._completions(measurement, self._applied.contacts),
            wall=self._last_wall_ns or 0, result_stop_reason=self._stop_reason,
        )
        if self._schedule is not None:
            self._schedule.close()
        self._terminal_snapshot, self._result = snapshot, result

    def _stable_for_ns(self, measurement: Measurement) -> int:
        if self._phase is Phase.STOPPING:
            return self._braking.stable_for_ns(measurement)
        if self._stable_since_ns is None:
            return 0
        return max(0, measurement.sim_time_ns - self._stable_since_ns)

    def _begin_stop(self, wall: int, reason: str, latched: str | None) -> None:
        if self._phase is Phase.STOPPING or self._phase in TERMINAL_PHASES:
            return
        base = (
            zero_sample(self._hold_positions) if self._applied is None else self._applied.reference
        )
        assert self._schedule is not None
        try:
            canceled, _base_positions, reason_text, latched_text = begin_stop(
                self._braking, schedule=self._schedule, applied=self._applied,
                hold_positions=self._hold_positions,
                profile_joints=tuple(self._io.profile.joints),
                sim_time_ns=sim_time_ns(self._scene.data), reason=reason, latched=latched,
            )
        except ContractError:
            self._hold_positions = base.positions
            self._stop_reason = reason
            self._latched_failure = FailedReason.STOP_PLAN_REJECTED.value
            self._fail(FailedReason.STOP_PLAN_REJECTED.value, wall)
            return
        if canceled is not None:
            self._event("CANCELED", canceled, sim_time_ns(self._scene.data), reason_text)
        self._stop_requested_wall_ns = wall
        self._stop_reason = reason_text
        self._latched_failure = latched_text
        self._phase = Phase.STOPPING

    def _advance_deadlines(self, wall: int) -> bool:
        if self._phase in TERMINAL_PHASES:
            return True
        decision = deadline_decision(
            phase=self._phase, wall=wall, config=self._config,
            stream_config=self._stream_config, accepted_wall_ns=self._accepted_wall_ns,
            stop_requested_wall_ns=self._stop_requested_wall_ns,
            last_progress_wall_ns=self._last_progress_wall_ns,
            last_progress_sim_ns=self._last_progress_sim_ns,
            now_sim_ns=sim_time_ns(self._scene.data),
            applied_measurement_sim_ns=(
                None if self._applied is None else self._applied.measurement.sim_time_ns
            ),
            final_end_ns=self._stream_final_end_ns,
        )
        if decision is None:
            return False
        action, reason, latched, exact_failure = decision
        if action == "fail":
            self._fail(reason, wall)
            return True
        self._begin_stop(wall, reason, latched)
        return exact_failure and self._phase is Phase.FAILED

    def _fail(self, reason: str, wall: int) -> None:
        self._phase = Phase.FAILED
        if self._applied is not None:
            self._hold_positions = self._applied.reference.positions
            measurement = self._applied.measurement
        else:
            measurement = self._io.measure()
        contacts = self._gripper.empty_contacts()
        snapshot, result = failed_terminal(
            self._snapshots, context=self._context(), applied=self._applied,
            reason=str(reason), stable_for_ns=self._stable_for_ns(measurement),
            wall=wall, stop_reason=self._stop_reason, measurement=measurement,
            hold_positions=self._hold_positions, contacts=contacts,
            hold_completions=self._gripper.hold_completions(
                measurement, contacts, self._hold_positions
            ),
        )
        self._last_snapshot = snapshot
        if self._schedule is not None:
            self._schedule.close()
        self._terminal_snapshot, self._result = snapshot, result

    def _freeze(self, snapshot: MotionSnapshot, wall: int, stop_reason: str | None) -> None:
        if self._schedule is not None:
            self._schedule.close()
        self._terminal_snapshot = snapshot
        self._result = Snapshotter.freeze(snapshot, wall, stop_reason)

    def _apply_and_measure(
        self, reference: ReferenceSample, application_sim_ns: int,
        elapsed_ns: int, reference_kind: str,
    ) -> AppliedStep:
        """The one real application/step/forward/measure sequence in the module."""
        applied_reference, corrections = (
            normalize_reference(reference, tuple(self._io.profile.joints))
            if self._schedule is not None else (reference, ())
        )
        self._io.apply_reference(applied_reference.positions, applied_reference.velocities)
        import mujoco

        mujoco.mj_step(self._scene.model, self._scene.data)
        mujoco.mj_forward(self._scene.model, self._scene.data)
        self._applied = AppliedStep(
            applied_reference, self._io.measure(), self._gripper.measure_contacts(),
            application_sim_ns, elapsed_ns, reference_kind, reference, corrections,
        )
        return self._applied

    def _record_progress(self, applied: AppliedStep, wall: int) -> None:
        if self._last_progress_sim_ns != applied.measurement.sim_time_ns:
            self._last_progress_sim_ns = applied.measurement.sim_time_ns
            self._last_progress_wall_ns = wall

    def _update_stream_tracking(self, applied: AppliedStep) -> None:
        measurement = applied.measurement
        constrained = self._gripper.constrained_axes(
            applied.reference, measurement, applied.contacts
        )
        self._tracking.update(
            applied.reference, measurement, constrained, self._gripper.unloading_axes,
            self._config.position_tolerances, self._config.velocity_tolerances,
        )

    def _snapshot_for_call(self) -> MotionSnapshot:
        measurement = self._io.measure()
        contacts = self._gripper.empty_contacts()
        return snapshot_for_call(
            self._snapshots, context=self._context(), last_snapshot=self._last_snapshot,
            phase=self._phase, stop_reason=self._stop_reason, events=tuple(self._events),
            measurement=measurement, hold_positions=self._hold_positions, contacts=contacts,
            hold_completions=self._gripper.hold_completions(
                measurement, contacts, self._hold_positions
            ),
        )

    def _applied_snapshot(
        self, applied: AppliedStep, *, phase: Phase, reason: str,
        reference_kind: str | None = None, chunk_seq: int | None = None,
        stable_for_ns: int | None = None,
        completions: tuple[GripperCompletion, ...] | None = None,
        elapsed_ns: int | None = None,
    ) -> MotionSnapshot:
        return self._snapshots.build_applied(
            self._context(), applied, phase=phase, reason=reason,
            reference_kind=reference_kind, chunk_seq=chunk_seq,
            stable_for_ns=(
                self._stable_for_ns(applied.measurement)
                if stable_for_ns is None else stable_for_ns
            ),
            completions=completions, elapsed_ns=elapsed_ns,
        )

    def _hold_step_live(self) -> MotionSnapshot:
        application_sim_ns = sim_time_ns(self._scene.data)
        reference = zero_sample(self._hold_positions)
        elapsed_ns = (
            0 if self._started_sim_ns is None else max(0, application_sim_ns - self._started_sim_ns)
        )
        applied = self._apply_and_measure(reference, application_sim_ns, elapsed_ns, "HOLD")
        reason = self._terminal_snapshot.reason if self._phase in TERMINAL_PHASES else ""
        snapshot = self._snapshots.build(
            self._context(), phase=self._phase, reason=reason, reference=reference,
            measurement=applied.measurement,
            reference_application_sim_time_ns=application_sim_ns,
            elapsed_ns=elapsed_ns, stable_for_ns=0, contacts=applied.contacts,
            completions=self._gripper.hold_completions(
                applied.measurement, applied.contacts, self._hold_positions
            ),
            reference_kind="HOLD",
        )
        self._last_snapshot = snapshot
        return snapshot
