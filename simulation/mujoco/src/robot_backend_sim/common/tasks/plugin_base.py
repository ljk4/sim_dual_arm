"""Shared lifecycle, idempotency, and Teacher-chunk cache base for task plugins."""

from __future__ import annotations

from collections import OrderedDict
from collections.abc import Sequence
from typing import Any

from robot_contracts.io import canonical_sha256
from robot_contracts.models import ContractError, RobotProfile, RunContext
from robot_contracts.task_state import TaskState

from robot_backend_sim.common.tasks.base import (
    TaskChunk,
    TaskChunkView,
    TaskPlan,
    TaskPrepareResult,
    TaskPrepareSnapshot,
    TeacherPrepareSnapshot,
)

__all__ = [
    "BaseTeacherTaskPlugin",
    "pose_moved",
    "sample_step_chunk",
    "teacher_joint_limits",
]

_COMMAND_HISTORY = 64


def pose_moved(current: Sequence[float], original: Sequence[float]) -> bool:
    """Return True if an object's (x, y, z, qw, qx, qy, qz) pose drifted materially."""
    if any(abs(a - b) > 1e-4 for a, b in zip(current[:3], original[:3], strict=True)):
        return True
    dot = abs(sum(a * b for a, b in zip(current[3:7], original[3:7], strict=True)))
    return abs(dot - 1.0) > 1e-6


class BaseTeacherTaskPlugin:
    """Shared task plugin lifecycle, runtime delegation, and Teacher chunk cache."""

    task_type: str = ""
    teacher_capability: str = ""

    def __init__(self, config: Any, *, task_mode: str | None) -> None:
        self.config = config
        self._task_mode = task_mode
        self._scene_config: Any = None
        self._binding: Any = None
        self._runtime: Any = None
        self._teacher_feedback: OrderedDict[int, Any] = OrderedDict()
        self._step_motion_start: dict[int, int] = {}
        self._prepared_command_id = ""
        self._chunk_cache: dict[str, tuple[str, TaskChunk]] = {}

    @property
    def config_digest(self) -> str:
        return self.config.config_digest

    @property
    def plan_timeout_s(self) -> float:
        return float(self.config.teacher.plan_wall_timeout_s)

    @property
    def teacher_enabled(self) -> bool:
        return self._task_mode == "teacher" and bool(self.config.teacher.enabled)

    @property
    def state(self) -> TaskState:
        return self._require_runtime().state

    @property
    def binding(self) -> Any:
        return self._binding

    def _require_runtime(self) -> Any:
        if self._runtime is None:
            raise ContractError("TASK_NOT_READY", "task plugin has no bound scene")
        return self._runtime

    def capacity_requirements(self) -> dict[str, int]:
        capacity = self.config.capacity
        return {
            "chunk_history": int(capacity.max_episode_chunks),
            "command_history": _COMMAND_HISTORY,
            "event_queue": int(capacity.expected_events),
            "follower_reference_queue": int(capacity.expected_references),
        }

    def _cached_prepare_duration_s(self, fact: Any) -> float:
        return float(fact.plan_duration_ns / 1e9)

    def _build_prepare_snapshot(
        self,
        *,
        context: RunContext,
        task_epoch: int,
        sim_time_ns: int,
        capture_seq: int,
        qpos: tuple[float, ...],
        seed: int,
        controller_command_id: str,
    ) -> TeacherPrepareSnapshot:
        return TeacherPrepareSnapshot(
            context=context,
            task_epoch=task_epoch,
            sim_time_ns=sim_time_ns,
            capture_seq=capture_seq,
            qpos=qpos,
            seed=seed,
            controller_command_id=controller_command_id,
        )

    def _pose_check_addresses(self) -> tuple[int, ...]:
        return ()

    def _validate_plan_for_commit(self, plan_obj: Any) -> tuple[int, float, float]:
        steps = getattr(plan_obj, "steps", None)
        if steps is None:
            return (0, 0.0, 0.0)
        return (
            len(steps),
            float(plan_obj.ik_max_position_error_m),
            float(plan_obj.ik_max_orientation_error_rad),
        )

    def commit_prepare(
        self,
        *,
        scene: object,
        controller: object,
        snapshot: TaskPrepareSnapshot,
        plan: TaskPlan,
        request_id: str,
        content_digest: str,
        context: Any,
        lease_id: str,
        now: int,
    ) -> TaskPrepareResult:
        runtime = self._require_runtime()
        self._validate_commit_prepare(
            runtime=runtime,
            controller=controller,
            snapshot=snapshot,
            context=context,
            lease_id=lease_id,
            now=now,
        )
        for addr in self._pose_check_addresses():
            current_obj = tuple(float(v) for v in scene.data.qpos[addr : addr + 7])  # type: ignore[attr-defined]
            snap_obj = tuple(snapshot.qpos[addr : addr + 7])
            if pose_moved(current_obj, snap_obj):
                raise ContractError(
                    "STALE_TASK_PREPARE", "scene objects moved between snapshot and commit"
                )
        step_count, ik_pos_err, ik_ori_err = self._validate_plan_for_commit(plan.plan)
        self._reset_teacher_caches()
        outcome = runtime.commit_prepare(
            request_id=request_id,
            content_digest=content_digest,
            context=context,
            plan=plan.plan,
            plan_digest=plan.plan_digest,
            plan_duration_ns=plan.duration_ns,
            step_count=step_count,
            ik_max_position_error_m=ik_pos_err,
            ik_max_orientation_error_rad=ik_ori_err,
            prepare_kind=plan.prepare_kind,
        )
        self._prepared_command_id = snapshot.controller_command_id
        return TaskPrepareResult(
            task_type=self.task_type,
            task_epoch=outcome.fact.state.task_epoch,
            task_config_digest=self.config.config_digest,
            plan_digest=plan.plan_digest,
            idempotent=outcome.idempotent,
            state=outcome.fact.state,
            prepare_kind=plan.prepare_kind,
            metrics={
                "step_count": outcome.fact.step_count,
                "ik_max_position_error_m": outcome.fact.ik_max_position_error_m,
                "ik_max_orientation_error_rad": outcome.fact.ik_max_orientation_error_rad,
                "planned_duration_s": plan.duration_ns / 1e9,
            },
        )

    def prepare_snapshot(
        self,
        *,
        scene: object,
        controller: object,
        context: RunContext,
        lease_id: str,
        request_id: str,
        content_digest: str,
        episode_id: str,
        seed: int,
        capture_seq: int,
        sim_time_ns: int,
        now: int,
        reset_seed: int,
    ) -> TaskPrepareSnapshot:
        runtime = self._require_runtime()
        if not isinstance(request_id, str) or not request_id:
            raise ContractError("INVALID_REQUEST_ID", "request_id must be non-empty")
        if not isinstance(content_digest, str) or not content_digest:
            raise ContractError("INVALID_REQUEST_ID", "content digest must be non-empty")
        if not isinstance(seed, int) or isinstance(seed, bool) or seed < 0:
            raise ContractError("INVALID_SEED", "seed must be a non-negative integer")
        if not controller.lease_held(context, lease_id, now):  # type: ignore[attr-defined]
            raise ContractError("LEASE_EXPIRED", "prepare does not own the current lease")
        if context != controller.context or episode_id != context.episode_id:  # type: ignore[attr-defined]
            raise ContractError("STALE_CONTEXT", "prepare context/episode differs")
        if runtime.state.phase == "RUNNING":
            raise ContractError("TASK_BUSY", f"a {self.task_type} task is already running")
        if not controller.is_quiescent():  # type: ignore[attr-defined]
            raise ContractError("TASK_NOT_QUIESCENT", "prepare requires a still robot")
        if seed != reset_seed:
            raise ContractError(
                "SEED_MISMATCH",
                f"prepare seed {seed} differs from the last Scene.reset seed {reset_seed}",
            )
        cached = runtime.lookup_prepare(request_id, content_digest)
        if cached is not None:
            fact = cached.fact
            return TaskPrepareResult(  # type: ignore[return-value]
                task_type=self.task_type,
                task_epoch=fact.state.task_epoch,
                task_config_digest=self.config.config_digest,
                plan_digest=fact.plan_digest,
                idempotent=True,
                state=fact.state,
                prepare_kind=fact.prepare_kind,
                metrics={
                    "step_count": fact.step_count,
                    "ik_max_position_error_m": fact.ik_max_position_error_m,
                    "ik_max_orientation_error_rad": fact.ik_max_orientation_error_rad,
                    "planned_duration_s": self._cached_prepare_duration_s(fact),
                },
            )
        runtime.require_prepare_slot()
        status = controller.current_status()  # type: ignore[attr-defined]
        return self._build_prepare_snapshot(
            context=context,
            task_epoch=runtime.task_epoch + 1,
            sim_time_ns=sim_time_ns,
            capture_seq=capture_seq,
            qpos=tuple(float(value) for value in scene.data.qpos),  # type: ignore[attr-defined]
            seed=seed,
            controller_command_id="" if status is None else status.command_id,
        )

    def _validate_commit_prepare(
        self,
        *,
        runtime: Any,
        controller: object,
        snapshot: Any,
        context: RunContext,
        lease_id: str,
        now: int,
    ) -> None:
        if not controller.lease_held(context, lease_id, now):  # type: ignore[attr-defined]
            raise ContractError("LEASE_EXPIRED", "prepare lease expired before commit")
        if context != controller.context:  # type: ignore[attr-defined]
            raise ContractError("STALE_CONTEXT", "prepare context changed before commit")
        if runtime.state.phase != "IDLE" or runtime.task_epoch != snapshot.task_epoch - 1:
            raise ContractError("STALE_TASK_PREPARE", "task generation changed before commit")
        status = controller.current_status()  # type: ignore[attr-defined]
        if ("" if status is None else status.command_id) != snapshot.controller_command_id:
            raise ContractError(
                "STALE_TASK_PREPARE", "another motion ran while task planning was pending"
            )
        if not controller.is_quiescent():  # type: ignore[attr-defined]
            raise ContractError("TASK_NOT_QUIESCENT", "robot moved before prepare commit")

    def start(
        self,
        *,
        controller: object,
        context: RunContext,
        lease_id: str,
        request_id: str,
        content_digest: str,
        plan_digest: str,
        task_epoch: int,
        episode_id: str,
        sim_time_ns: int,
        wall_ns: int,
        command_id: str,
    ) -> tuple[TaskState, bool]:
        runtime = self._require_runtime()
        if not controller.lease_held(context, lease_id, wall_ns):  # type: ignore[attr-defined]
            raise ContractError("LEASE_EXPIRED", "start does not own the current lease")
        if context != controller.context or episode_id != context.episode_id:  # type: ignore[attr-defined]
            raise ContractError("STALE_CONTEXT", "start context/episode differs")
        if runtime.state.phase == "PREPARED" and command_id != self._prepared_command_id:
            raise ContractError("STALE_TASK_PREPARE", "robot command changed after prepare")
        state, idempotent = runtime.start(
            request_id=request_id,
            content_digest=content_digest,
            context=context,
            plan_digest=plan_digest,
            task_epoch=task_epoch,
            sim_time_ns=sim_time_ns,
            wall_ns=wall_ns,
        )
        if not idempotent and self.teacher_enabled:
            self._reset_teacher_caches()
        return state, idempotent

    def tick(self, observation: object, wall_ns: int) -> TaskState:
        return self._require_runtime().tick(observation, wall_ns)  # type: ignore[arg-type]

    def reset(self, reason: str) -> TaskState:
        runtime = self._require_runtime()
        self._reset_teacher_caches()
        return runtime.reset(reason)

    def fail_control(self, reason: str) -> TaskState:
        return self._require_runtime().fail_control(reason)

    def _reset_teacher_caches(self) -> None:
        self._teacher_feedback.clear()
        self._step_motion_start.clear()
        self._chunk_cache.clear()

    def _retained_feedback_and_gate(self, capture_seq: int) -> tuple[Any, Any] | None:
        return self._teacher_feedback.get(capture_seq)

    def _validate_prepared_plan(self, plan: Any) -> None:
        if plan is None:
            raise ContractError("TASK_NOT_PREPARED", "task has no immutable plan")

    def chunk_view(
        self,
        *,
        context: RunContext,
        task_epoch: int,
        capture_seq: int,
        observation_time_ns: int,
        latest_capture_seq: int,
        capture_times: object,
        current_sim_time_ns: int,
        handoff_time_ns: int = 0,
    ) -> TaskChunkView:
        runtime = self._require_runtime()
        state = runtime.state
        if state.phase != "RUNNING":
            raise ContractError("TASK_NOT_RUNNING", "teacher chunks require a running task")
        if state.context != context or task_epoch != state.task_epoch:
            raise ContractError("STALE_TASK_CONTEXT", "teacher chunk task epoch differs")
        if isinstance(capture_seq, bool) or not isinstance(capture_seq, int) or capture_seq < 1:
            raise ContractError("INVALID_CAPTURE", "capture_seq must be positive")
        if capture_seq > latest_capture_seq:
            raise ContractError("CAPTURE_NOT_RETAINED", "capture_seq is not retained yet")
        if observation_time_ns > current_sim_time_ns:
            raise ContractError(
                "TEACHER_FUTURE_OBSERVATION",
                "observation time is later than the current simulation time",
            )
        retained_time = next(
            (sim_ns for seq, sim_ns in reversed(capture_times) if seq == capture_seq),  # type: ignore[attr-defined]
            None,
        )
        if retained_time is None:
            raise ContractError("CAPTURE_NOT_RETAINED", "capture_seq is no longer retained")
        if observation_time_ns != retained_time:
            raise ContractError(
                "TEACHER_OBSERVATION_MISMATCH",
                "observation time does not belong to the retained capture",
            )
        start_ns = int(state.metric("task_start_sim_time_ns", 0))
        if observation_time_ns < start_ns:
            raise ContractError("TEACHER_BEFORE_START", "observation precedes task start")
        plan = runtime.plan
        self._validate_prepared_plan(plan)
        pair = self._retained_feedback_and_gate(capture_seq)
        if pair is None:
            raise ContractError(
                "CAPTURE_FEEDBACK_MISSING",
                "no immutable teacher feedback exists for this capture",
            )
        feedback, gate = pair
        return TaskChunkView(
            plan=plan,
            task_epoch=state.task_epoch,
            task_start_sim_time_ns=start_ns,
            current_sim_time_ns=current_sim_time_ns,
            latest_capture_seq=latest_capture_seq,
            state=state,
            feedback=feedback,
            progress=gate,
            handoff_time_ns=handoff_time_ns,
        )

    def _on_step_motion_start(self, step_index: int, motion_start: int) -> None:
        del step_index, motion_start

    def _sample_step_chunk(
        self,
        view: TaskChunkView,
        *,
        request_id: str,
        capture_seq: int,
        observation_time_ns: int,
        profile: RobotProfile,
        control_config: object,
        motion_start: int,
    ) -> TaskChunk:
        raise NotImplementedError

    def sample_chunk(
        self,
        view: TaskChunkView,
        *,
        request_id: str,
        capture_seq: int,
        observation_time_ns: int,
        profile: RobotProfile,
        control_config: object,
    ) -> TaskChunk:
        content_digest = canonical_sha256(
            {
                "kind": "chunk",
                "context": view.state.context.__dict__
                if view.state.context is not None
                else None,
                "task_epoch": view.task_epoch,
                "request_id": request_id,
                "capture_seq": capture_seq,
                "observation_time_ns": observation_time_ns,
                "handoff_time_ns": view.handoff_time_ns,
                "plan_digest": view.plan.plan_digest,
            }
        )
        cached = self._chunk_cache.get(request_id)
        if cached is not None:
            if cached[0] != content_digest:
                raise ContractError(
                    "REQUEST_ID_CONFLICT", "same teacher request_id has different content"
                )
            return cached[1]
        if len(self._chunk_cache) >= self.config.teacher.chunk_cache_entries:
            raise ContractError(
                "TEACHER_CACHE_FULL",
                "teacher idempotency cache is full; refusing to drop an audited chunk",
            )
        step_index = view.progress.step_index
        motion_start = self._step_motion_start.get(step_index)
        if motion_start is None:
            motion_start = view.handoff_time_ns or view.progress.active_start_sim_time_ns
            self._step_motion_start[step_index] = motion_start
            self._on_step_motion_start(step_index, motion_start)
        chunk = self._sample_step_chunk(
            view,
            request_id=request_id,
            capture_seq=capture_seq,
            observation_time_ns=observation_time_ns,
            profile=profile,
            control_config=control_config,
            motion_start=motion_start,
        )
        self._chunk_cache[request_id] = (content_digest, chunk)
        return chunk

    def fill_chunk_response(self, response: object, chunk: TaskChunk) -> object:
        from robot_backend_sim.common.tasks.ros_conv import fill_teacher_chunk_response

        return fill_teacher_chunk_response(response, chunk)

    def clear_chunk_cache(self) -> None:
        self._chunk_cache.clear()


def teacher_joint_limits(
    profile: RobotProfile,
    control: Any,
    teacher: Any,
    gripper_indices: tuple[int, ...] = (7, 15),
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """Compute velocity and acceleration limits for Teacher path planning."""
    velocities = [
        float(joint.max_velocity) * teacher.max_joint_velocity_fraction
        for joint in profile.joints
    ]
    accelerations = [
        float(value) * teacher.max_joint_acceleration_fraction
        for value in control.max_accelerations
    ]
    for index in gripper_indices:
        velocities[index] = teacher.gripper_velocity_m_s
        accelerations[index] = teacher.gripper_acceleration_m_s2
    return tuple(velocities), tuple(accelerations)


def sample_step_chunk(
    plan: Any,
    *,
    steps: tuple[Any, ...],
    behaviour_phases: tuple[str, ...],
    gate: Any,
    feedback: Any,
    request_id: str,
    task_epoch: int,
    capture_seq: int,
    observation_time_ns: int,
    chunk_size: int,
    period_ns: int,
    profile: RobotProfile,
    control_config: Any,
    start_sim_time_ns: int,
    error_cls: type[ContractError] = ContractError,
) -> TaskChunk:
    """Sample one 60-point Teacher chunk along the currently gated step."""
    import hashlib
    import json
    import math

    from robot_contracts.models import Target
    from robot_trajectory.spline_math import validate_reference_inputs

    from robot_backend_sim.common.tasks.math import scalar_acceleration, trapezoid_progress

    if chunk_size != 60 or period_ns != 50_000_000:
        raise error_cls(
            "INVALID_TEACHER_CLOCK", "teacher chunks are exactly 60 points at 50 ms"
        )
    if feedback.capture_seq != capture_seq or feedback.sim_time_ns != observation_time_ns:
        raise error_cls(
            "TEACHER_FEEDBACK_MISMATCH",
            "feedback does not belong to the retained capture/observation",
        )
    if gate.failure or gate.status == "FAILED":
        raise error_cls("TEACHER_GATE_FAILED", gate.failure or "task gate failed")
    if gate.step_index >= len(steps):
        raise error_cls("TEACHER_PLAN_COMPLETE", "all gated steps have passed")
    if gate.active_start_sim_time_ns < 0:
        raise error_cls("TEACHER_NOT_STARTED", "no measured step start exists")

    step = plan.step_at(gate.step_index)
    duration_ns = round(step.duration_s * 1e9)
    acceleration = scalar_acceleration(
        step.start_positions, step.end_positions, plan.accelerations
    )
    positions: list[tuple[float, ...]] = []
    phases: list[str] = []
    times: list[int] = []
    for index in range(chunk_size):
        target_time_ns = observation_time_ns + (index + 1) * period_ns
        elapsed = target_time_ns - start_sim_time_ns
        if step.final or duration_ns <= 0 or elapsed >= duration_ns:
            position = step.end_positions
        else:
            progress = trapezoid_progress(
                max(0.0, elapsed / 1e9), step.duration_s, acceleration
            )
            position = tuple(
                first + (last - first) * progress
                for first, last in zip(step.start_positions, step.end_positions, strict=True)
            )
        positions.append(position)
        phases.append(step.phase)
        times.append((index + 1) * period_ns)

    if not all(math.isfinite(value) for pos in positions for value in pos):
        raise error_cls("INVALID_TEACHER_TARGET", "teacher produced a non-finite target")

    targets = tuple(
        Target(time_ns, pos) for time_ns, pos in zip(times, positions, strict=True)
    )
    validate_reference_inputs(
        positions[0], targets, profile.joints, control_config.max_accelerations
    )

    digest_payload = {
        "plan_digest": plan.plan_digest,
        "step": step.name,
        "capture_seq": capture_seq,
        "observation_time_ns": observation_time_ns,
        "positions": positions,
        "phase": step.phase,
        "gate": gate.status,
    }
    chunk_digest = hashlib.sha256(
        json.dumps(
            digest_payload, sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode()
    ).hexdigest()
    wait_s = max(0.0, (observation_time_ns - start_sim_time_ns - duration_ns) / 1e9)
    return TaskChunk(
        request_id=request_id,
        task_epoch=task_epoch,
        capture_seq=capture_seq,
        observation_time_ns=observation_time_ns,
        source_time_ns=feedback.sim_time_ns,
        robot_profile_hash=plan.context.robot_profile_hash,
        plan_digest=plan.plan_digest,
        chunk_digest=chunk_digest,
        phase=step.phase,
        phases=tuple(phases),
        phase_codes=tuple(behaviour_phases.index(p) for p in phases),
        time_from_observation_ns=tuple(times),
        positions=tuple(positions),
        ik_max_position_error_m=plan.ik_max_position_error_m,
        ik_max_orientation_error_rad=plan.ik_max_orientation_error_rad,
        gate_state=gate.status,
        gate_failure=gate.failure,
        phase_start_sim_time_ns=start_sim_time_ns,
        phase_wait_s=wait_s,
    )
