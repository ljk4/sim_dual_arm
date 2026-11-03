"""Single-threaded no-ROS motion state machine over PandaIO and real MuJoCo.

The module composes an already validated cubic reference plan, the accepted
named I/O layer and the actual physics data.  It does not implement ROS,
lease/reset, command history, object success or a worker thread.  A caller must
drive ``step`` with an explicit monotonic wall time and the scene's configured
2 ms physics step; the machine writes only the sixteen ctrl targets through
``PandaIO``, steps MuJoCo and refreshes contact evidence at the same
measurement time.
"""

from __future__ import annotations

import math
import threading
import uuid
from collections.abc import Sequence
from dataclasses import dataclass, replace
from enum import StrEnum
from numbers import Integral, Real

from robot_contracts.action_chunks import ChunkAnchor
from robot_contracts.follower import PreparedFollowerPlan
from robot_contracts.follower_math import normalize_follower_positions
from robot_contracts.models import ContractError, JointSpec, RobotProfile, Target
from robot_contracts.reference import (
    MAX_HORIZON_NS,
    ReferencePlan,
    ReferenceSample,
    verify_reference_plan,
)

from .braking import BrakingPlan, plan_braking
from .chunk_plan import PreparedChunk
from .chunk_types import PreparedStreamChunk, chunk_kind
from .contact_unload import REFERENCE_VELOCITY_ATOL_M_S, ContactUnload
from .contacts import (
    GripperCompletion,
    GripperContactEvidence,
    empty_contact_evidence,
    evaluate_gripper_completion,
    measure_contact_evidence,
    resolve_contact_binding,
)
from .motion_config import CONTROL_JOINT_NAMES, PandaControlConfig
from .panda_io import Measurement, PandaIO
from .reference_compiler import compile_reference_plan
from .scene import Scene
from .stream_config import StreamConfig
from .stream_schedule import (
    ChunkEvent,
    ChunkSubmission,
    StreamSchedule,
    boundary_rejection,
    grid_aligned,
    next_grid_time,
    observation_rejection,
)

__all__ = [
    "ChunkEvent",
    "ChunkSubmission",
    "FailedReason",
    "MotionResult",
    "MotionSnapshot",
    "NativeMotion",
    "Phase",
    "PreparedMotion",
    "StopReason",
    "prepare_motion",
]

_MAX_NS = 2**63 - 1
_ARM_INDICES = tuple(index for index in range(16) if index not in (7, 15))
_GRIPPER_INDEX = {"left": 7, "right": 15}


class Phase(StrEnum):
    """Local motion phase; this is not a ROS command acknowledgement."""

    IDLE = "IDLE"
    ACCEPTED = "ACCEPTED"
    RUNNING = "RUNNING"
    SETTLING = "SETTLING"
    STOPPING = "STOPPING"
    COMPLETED = "COMPLETED"
    STOPPED = "STOPPED"
    FAILED = "FAILED"


class StopReason(StrEnum):
    STOP_REQUESTED = "STOP_REQUESTED"
    TRACKING_TIMEOUT = "TRACKING_TIMEOUT"
    WALL_TIMEOUT = "WALL_TIMEOUT"


class FailedReason(StrEnum):
    TRACKING_TIMEOUT = "TRACKING_TIMEOUT"
    WALL_TIMEOUT = "WALL_TIMEOUT"
    STOP_TIMEOUT = "STOP_TIMEOUT"
    STOP_PLAN_REJECTED = "STOP_PLAN_REJECTED"
    PROGRESS_TIMEOUT = "PROGRESS_TIMEOUT"
    UNDERRUN = "UNDERRUN"
    CHUNK_LATE = "CHUNK_LATE"
    OBSERVATION_STALE = "OBSERVATION_STALE"
    OBSERVATION_FROM_FUTURE = "OBSERVATION_FROM_FUTURE"
    STREAM_START_MISSED = "STREAM_START_MISSED"


_ACTIVE_PHASES = frozenset({Phase.ACCEPTED, Phase.RUNNING, Phase.SETTLING, Phase.STOPPING})
_TERMINAL_PHASES = frozenset({Phase.COMPLETED, Phase.STOPPED, Phase.FAILED})


def _invalid(reason: str, detail: str) -> ContractError:
    return ContractError(reason, detail)


def _finite_vector(value: object, location: str, length: int) -> tuple[float, ...]:
    if isinstance(value, str | bytes):
        raise _invalid("INVALID_REFERENCE", f"{location}: must be an array of numbers")
    try:
        items = tuple(value)  # type: ignore[arg-type]
    except TypeError as error:
        raise _invalid("INVALID_REFERENCE", f"{location}: must be an array of numbers") from error
    if len(items) != length:
        raise _invalid(
            "INVALID_REFERENCE", f"{location}: expected {length} values, got {len(items)}"
        )
    result: list[float] = []
    for index, item in enumerate(items):
        if isinstance(item, bool) or not isinstance(item, Real):
            raise _invalid(
                "INVALID_REFERENCE",
                f"{location}[{index}]: expected a finite real number",
            )
        try:
            number = float(item)
        except (OverflowError, TypeError, ValueError) as error:
            raise _invalid("INVALID_REFERENCE", f"{location}[{index}]: not finite") from error
        if not math.isfinite(number):
            raise _invalid("INVALID_REFERENCE", f"{location}[{index}]: must be finite")
        result.append(number)
    return tuple(result)


def _zero_sample(positions: tuple[float, ...]) -> ReferenceSample:
    zero = (0.0,) * len(positions)
    return ReferenceSample(positions, zero, zero)


def _sim_time_ns(data: object) -> int:
    value = data.time
    if isinstance(value, bool) or not isinstance(value, Real):
        raise _invalid("INVALID_MEASUREMENT", "simulation time must be a real number")
    seconds = float(value)
    if not math.isfinite(seconds) or seconds < 0.0 or seconds > _MAX_NS / 1e9:
        raise _invalid("INVALID_MEASUREMENT", "simulation time is not a finite non-negative time")
    return round(seconds * 1_000_000_000)


@dataclass(frozen=True, slots=True)
class PreparedMotion:
    """Deeply immutable, fully verified motion ready for ``start``.

    Constructing this object directly re-runs the shared mathematical plan
    verification outside the physics loop.  ``prepare_motion`` additionally
    compiles the original waypoints once with the accepted SciPy compiler.
    """

    joint_names: tuple[str, ...]
    joints: tuple[JointSpec, ...]
    initial_reference: tuple[float, ...]
    targets: tuple[Target, ...]
    plan: ReferencePlan
    max_accelerations: tuple[float, ...]
    profile_digest: str

    def __post_init__(self) -> None:
        try:
            names = tuple(self.joint_names)
            joints = tuple(self.joints)
            targets = tuple(self.targets)
        except TypeError as error:
            raise _invalid(
                "INVALID_PREPARED_MOTION", "names/joints/targets must be arrays"
            ) from error
        if not names or len(set(names)) != len(names):
            raise _invalid("INVALID_PREPARED_MOTION", "joint_names must be unique and non-empty")
        if not isinstance(self.plan, ReferencePlan):
            raise _invalid("INVALID_PREPARED_MOTION", "plan must be a ReferencePlan")
        if not isinstance(self.profile_digest, str) or not self.profile_digest:
            raise _invalid("INVALID_PREPARED_MOTION", "profile_digest must be a non-empty string")
        initial = _finite_vector(self.initial_reference, "initial_reference", len(names))
        accelerations = _finite_vector(self.max_accelerations, "max_accelerations", len(names))
        for index, acceleration in enumerate(accelerations):
            if acceleration <= 0.0:
                raise _invalid(
                    "INVALID_PREPARED_MOTION",
                    f"max_accelerations[{index}] must be positive",
                )
        # Validate raw targets (including time/position types) against the plan.
        verify_reference_plan(
            self.plan,
            initial,
            targets,
            joints,
            accelerations,
        )
        normalised_targets: list[Target] = []
        for index, target in enumerate(targets):
            if not isinstance(target, Target):
                raise _invalid("INVALID_PREPARED_MOTION", f"targets[{index}] must be a Target")
            positions = _finite_vector(target.positions, f"targets[{index}].positions", len(names))
            normalised_targets.append(Target(target.time_ns, positions))
        if tuple(joint.name for joint in joints) != names:
            raise _invalid("INVALID_PREPARED_MOTION", "joint_names must match the joints array")
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "joints", joints)
        object.__setattr__(self, "initial_reference", initial)
        object.__setattr__(self, "targets", tuple(normalised_targets))
        object.__setattr__(self, "max_accelerations", accelerations)
        object.__setattr__(self, "profile_digest", str(self.profile_digest))

    @property
    def final_positions(self) -> tuple[float, ...]:
        """Original final waypoint; never replaced by a measured opening."""
        return self.targets[-1].positions

    @property
    def duration_ns(self) -> int:
        return self.plan.duration_ns


def prepare_motion(
    initial_reference: Sequence[float],
    targets: Sequence[Target],
    profile: RobotProfile,
    config: PandaControlConfig,
) -> PreparedMotion:
    """Compile and verify one motion once, before the physical loop starts."""
    if not isinstance(profile, RobotProfile):
        raise _invalid("INVALID_PREPARED_MOTION", "profile must be a RobotProfile")
    if not isinstance(config, PandaControlConfig):
        raise _invalid("INVALID_PREPARED_MOTION", "config must be a PandaControlConfig")
    if tuple(profile.joint_names) != config.joint_names:
        raise _invalid("PROFILE_MISMATCH", "profile joint order differs from control config")
    initial = _finite_vector(initial_reference, "initial_reference", len(CONTROL_JOINT_NAMES))
    target_items = tuple(targets)
    plan = compile_reference_plan(
        initial,
        target_items,
        profile.joints,
        config.max_accelerations,
    )
    if plan.duration_ns > MAX_HORIZON_NS:
        raise _invalid("INVALID_PREPARED_MOTION", "plan horizon exceeds the supported maximum")
    return PreparedMotion(
        joint_names=tuple(profile.joint_names),
        joints=tuple(profile.joints),
        initial_reference=initial,
        targets=target_items,
        plan=plan,
        max_accelerations=config.max_accelerations,
        profile_digest=profile.digest,
    )


@dataclass(frozen=True, slots=True)
class MotionSnapshot:
    """One immutable live snapshot of reference, measurement and evidence."""

    phase: Phase
    motion_seq: int
    reason: str
    accepted_wall_ns: int | None
    started_wall_ns: int | None
    started_sim_ns: int | None
    reference_application_sim_time_ns: int
    reference_elapsed_ns: int
    measurement_sim_time_ns: int
    stable_for_ns: int
    joint_names: tuple[str, ...]
    reference_positions: tuple[float, ...]
    reference_velocities: tuple[float, ...]
    reference_accelerations: tuple[float, ...]
    measured_positions: tuple[float, ...]
    measured_velocities: tuple[float, ...]
    tracking_errors: tuple[float, ...]
    grippers: tuple[GripperCompletion, ...]
    contacts: tuple[GripperContactEvidence, ...]
    stream_id: str | None = None
    chunk_seq: int | None = None
    reference_kind: str = "HOLD"
    tracking_velocity_errors: tuple[float, ...] = ()
    tracking_quality: str = "HOLD"
    tracking_violation_ns: int = 0
    chunk_events: tuple[ChunkEvent, ...] = ()
    sampled_reference_positions: tuple[float, ...] = ()
    sampled_reference_velocities: tuple[float, ...] = ()
    reference_boundary_corrections: tuple[float, ...] = ()


@dataclass(frozen=True, slots=True)
class MotionResult:
    """Terminal result frozen at the first COMPLETED/STOPPED/FAILED transition."""

    motion_seq: int
    phase: Phase
    reason: str
    stop_reason: str | None
    accepted_wall_ns: int | None
    started_wall_ns: int | None
    started_sim_ns: int | None
    finished_wall_ns: int
    snapshot: MotionSnapshot

    @property
    def terminal_snapshot(self) -> MotionSnapshot:
        return self.snapshot


@dataclass(frozen=True, slots=True)
class _BoundarySelection:
    """Pure reference choice for one active-chunk endpoint."""

    reference: ReferenceSample
    elapsed_ns: int
    reference_kind: str
    chunk_seq: int
    action: str


@dataclass(frozen=True, slots=True)
class _AppliedStep:
    """One coherent physical application, used as the source for a later brake.

    ``reference`` is what was actually written to ``PandaIO``.  For a follower,
    ``sampled_reference`` keeps the untouched coefficient sample and
    ``reference_corrections`` keeps ``sampled - applied`` per axis so the shared
    boundary normalization is independently auditable.  Strict chunks keep the
    two references identical and corrections empty.
    """

    reference: ReferenceSample
    measurement: Measurement
    contacts: tuple[GripperContactEvidence, ...]
    application_sim_time_ns: int
    elapsed_ns: int
    reference_kind: str
    sampled_reference: ReferenceSample
    reference_corrections: tuple[float, ...]


class NativeMotion:
    """One motion owner bound to the creating thread and one ``PandaIO``.

    ``step`` applies the reference at simulation time ``t`` through
    ``PandaIO.apply_targets``, calls ``mj_step`` once to ``t + dt`` and then
    ``mj_forward`` so contact evidence belongs to the post-step measurement.
    Reference and braking elapsed come only from ``Scene.data.time``; callers'
    wall time is used only for motion/stop deadlines and never advances a
    reference plan.  ``poll`` never steps or reads a hidden wall clock.
    """

    def __init__(
        self,
        scene: Scene,
        io: PandaIO,
        config: PandaControlConfig,
        initial_reference: Sequence[float] | None = None,
        *,
        stream_config: StreamConfig | None = None,
    ) -> None:
        if not isinstance(io, PandaIO):
            raise _invalid("INVALID_MOTION", "io must be a PandaIO")
        if not isinstance(config, PandaControlConfig):
            raise _invalid("INVALID_MOTION", "config must be a PandaControlConfig")
        if not isinstance(scene, Scene):
            raise _invalid("INVALID_MOTION", "scene must be a Scene")
        self._owner_thread = threading.current_thread()
        io.require_scene(scene)
        self._scene = scene
        self._io = io
        self._config = config
        self._binding = resolve_contact_binding(
            scene.model, tuple(obj.name for obj in scene.config.objects)
        )
        if tuple(io.joint_names) != config.joint_names:
            raise _invalid("PROFILE_MISMATCH", "PandaIO joint order differs from control config")
        if tuple(io.joint_names) != CONTROL_JOINT_NAMES:
            raise _invalid(
                "PROFILE_MISMATCH", "PandaIO joint order is not the fixed 16-joint order"
            )
        if tuple(io.profile.joint_names) != config.joint_names:
            raise _invalid(
                "PROFILE_MISMATCH", "robot profile joint order differs from control config"
            )
        if not isinstance(io.profile.digest, str) or not io.profile.digest:
            raise _invalid("PROFILE_MISMATCH", "robot profile digest is missing")
        timestep = float(scene.model.opt.timestep)
        if not math.isfinite(timestep) or timestep <= 0.0:
            raise _invalid("MODEL_MISMATCH", "scene timestep must be finite and positive")
        step_ns = round(timestep * 1_000_000_000)
        if step_ns <= 0:
            raise _invalid("MODEL_MISMATCH", "scene timestep must resolve to a positive grid")
        self._step_ns = step_ns
        if stream_config is not None and not isinstance(stream_config, StreamConfig):
            raise _invalid("INVALID_STREAM_CONFIG", "stream_config must be a StreamConfig")
        if stream_config is not None and stream_config.min_lead_ns < step_ns:
            raise _invalid(
                "INVALID_STREAM_CONFIG",
                "min_lead_s must be at least one physics step",
            )
        self._stream_config = stream_config

        default = io.profile.initial_positions
        hold = default if initial_reference is None else initial_reference
        self._hold_positions = _finite_vector(hold, "initial_reference", len(CONTROL_JOINT_NAMES))
        self._check_positions_within_profile(self._hold_positions)

        self._phase = Phase.IDLE
        self._motion_seq = 0
        self._accepted_wall_ns: int | None = None
        self._started_wall_ns: int | None = None
        self._started_sim_ns: int | None = None
        self._last_wall_ns: int | None = None
        self._prepared: PreparedMotion | None = None
        self._result: MotionResult | None = None
        self._terminal_snapshot: MotionSnapshot | None = None
        self._last_snapshot: MotionSnapshot | None = None
        self._applied: _AppliedStep | None = None
        self._stop_plan: BrakingPlan | None = None
        self._stop_requested_wall_ns: int | None = None
        self._stop_sim_start_ns: int | None = None
        self._stop_reason: str | None = None
        self._latched_failure: str | None = None
        self._stable_since_ns: int | None = None
        self._stable_signature: tuple[tuple[str, str | None], ...] | None = None
        self._stop_stable_since_ns: int | None = None
        self._instance_id = f"native-{uuid.uuid4().hex}"
        self._stream: StreamSchedule | None = None
        self._stream_initial_positions: tuple[float, ...] | None = None
        self._stream_final_positions: tuple[float, ...] | None = None
        self._stream_final_targets: tuple[float, ...] | None = None
        self._stream_final_end_ns: int | None = None
        self._tracking_violation_since_ns: int | None = None
        self._tracking_violation_observed_sim_ns: int = 0
        self._tracking_quality = "HOLD"
        self._contact_unload = {side: ContactUnload() for side in ("left", "right")}
        self._unloading_axes = frozenset()
        self._events: list[ChunkEvent] = []
        self._last_progress_wall_ns: int | None = None
        self._last_progress_sim_ns: int | None = None

    # ------------------------------------------------------------------ API

    @property
    def phase(self) -> Phase:
        self._check_thread()
        return self._phase

    @property
    def hold_positions(self) -> tuple[float, ...]:
        """The immutable reference the state machine holds when no plan runs.

        It changes only when a motion is accepted (the last hold/target) and is
        therefore the only position a caller may use to precompile a new MOTION.
        """
        self._check_thread()
        return self._hold_positions

    @property
    def result(self) -> MotionResult | None:
        """Frozen terminal result, or ``None`` while no terminal transition exists."""
        self._check_thread()
        return self._result

    @property
    def terminal_snapshot(self) -> MotionSnapshot | None:
        self._check_thread()
        return self._terminal_snapshot

    def start(self, prepared: PreparedMotion, wall_time_ns: int) -> MotionSnapshot:
        """Accept one already verified plan; the first step, not start, is started."""
        self._check_thread()
        self._events = []
        wall = self._validate_wall(wall_time_ns)
        if not isinstance(prepared, PreparedMotion):
            raise _invalid("INVALID_PREPARED_MOTION", "prepared must be a PreparedMotion")
        if self._phase in _ACTIVE_PHASES:
            raise _invalid("MOTION_ACTIVE", "cannot overlap active/settling/stopping motion")
        if self._phase is Phase.FAILED:
            raise _invalid("FAILED_LATCHED", "a failed motion must be reset before a new start")
        if prepared.joint_names != tuple(self._io.joint_names):
            raise _invalid("PROFILE_MISMATCH", "prepared joint order differs from PandaIO")
        if prepared.joint_names != self._config.joint_names:
            raise _invalid("PROFILE_MISMATCH", "prepared joint order differs from control config")
        if prepared.joints != tuple(self._io.profile.joints):
            raise _invalid("PROFILE_MISMATCH", "prepared joints differ from the current profile")
        if prepared.profile_digest != self._io.profile.digest:
            raise _invalid(
                "PROFILE_MISMATCH",
                "prepared profile digest differs from the current profile",
            )
        if prepared.max_accelerations != self._config.max_accelerations:
            raise _invalid(
                "PROFILE_MISMATCH", "prepared accelerations differ from the current config"
            )
        for index, (initial, hold) in enumerate(
            zip(prepared.initial_reference, self._hold_positions, strict=True)
        ):
            if not math.isclose(initial, hold, rel_tol=0.0, abs_tol=1e-9):
                raise _invalid(
                    "INITIAL_REFERENCE_MISMATCH",
                    f"initial_reference[{index}] {initial!r} does not match hold {hold!r}",
                )
        self._accept_wall(wall)
        # A legacy single-motion start retires any private stream scheduling
        # state.  The old stream id is retained only to reject stale packages.
        self._stream = None
        self._stream_initial_positions = None
        self._stream_final_positions = None
        self._stream_final_targets = None
        self._stream_final_end_ns = None
        self._tracking_violation_since_ns = None
        self._tracking_quality = "HOLD"
        self._contact_unload = {side: ContactUnload() for side in ("left", "right")}
        self._unloading_axes = frozenset()
        self._motion_seq += 1
        self._prepared = prepared
        self._accepted_wall_ns = wall
        self._started_wall_ns = None
        self._started_sim_ns = None
        self._result = None
        self._terminal_snapshot = None
        self._applied = None
        self._stop_plan = None
        self._stop_requested_wall_ns = None
        self._stop_sim_start_ns = None
        self._stop_reason = None
        self._latched_failure = None
        self._stable_since_ns = None
        self._stable_signature = None
        self._stop_stable_since_ns = None
        self._phase = Phase.ACCEPTED
        measurement = self._io.measure()
        contacts = self._empty_contacts()
        reference = _zero_sample(prepared.initial_reference)
        snapshot = self._build_snapshot(
            phase=Phase.ACCEPTED,
            reason="",
            reference=reference,
            measurement=measurement,
            reference_application_sim_time_ns=measurement.sim_time_ns,
            elapsed_ns=0,
            stable_for_ns=0,
            contacts=contacts,
            reference_kind="HOLD",
        )
        self._last_snapshot = snapshot
        return snapshot

    def chunk_anchor(self, start_sim_time_ns: int | None = None) -> ChunkAnchor:
        """Return an immutable preview/successor hand-off anchor.

        With no active stream the anchor previews the next motion's first chunk
        from the last hold (zero velocity/acceleration).  While a non-final
        chunk is running it previews that chunk's endpoint for one successor.
        This method never changes phase, deadlines, sequence numbers or physics.
        """
        self._check_thread()
        if self._stream_config is None:
            raise _invalid(
                "STREAM_NOT_CONFIGURED", "NativeMotion was created without a stream config"
            )
        now_sim_ns = _sim_time_ns(self._scene.data)
        if self._phase is Phase.FAILED:
            raise _invalid("STREAM_ANCHOR_UNAVAILABLE", "a failed motion has no successor")
        if self._phase is Phase.STOPPING:
            raise _invalid("STREAM_ANCHOR_UNAVAILABLE", "stopping motion has no successor")
        schedule = self._stream
        if schedule is None and self._phase in _ACTIVE_PHASES:
            raise _invalid("MOTION_ACTIVE", "legacy motion is active")
        if schedule is not None and self._phase in (
            Phase.ACCEPTED,
            Phase.RUNNING,
            Phase.SETTLING,
        ):
            if schedule.active is None:
                raise _invalid("STREAM_NOT_STARTED", "stream has not applied its first chunk")
            if self._phase is Phase.SETTLING or schedule.active.settle_at_end:
                raise _invalid("STREAM_FINAL_CHUNK", "final settling chunk has no successor")
            if schedule.pending is not None:
                raise _invalid("STREAM_PENDING_EXISTS", "one successor is already pending")
            active = schedule.active
            end_ns = active.end_sim_time_ns
            if start_sim_time_ns is not None:
                if isinstance(start_sim_time_ns, bool) or not isinstance(
                    start_sim_time_ns, Integral
                ):
                    raise _invalid("INVALID_CHUNK_START", "start_sim_time_ns must be an integer")
                if int(start_sim_time_ns) != end_ns:
                    raise _invalid(
                        "INVALID_CHUNK_START",
                        "explicit start does not equal the fixed active chunk end",
                    )
            return ChunkAnchor(
                schedule.stream_id,
                active.seq,
                end_ns,
                active.sample_at(end_ns),
                self._io.profile.digest,
            )

        if start_sim_time_ns is None:
            start_ns = next_grid_time(
                now_sim_ns, self._stream_config.initial_lead_ns, self._step_ns
            )
        else:
            if isinstance(start_sim_time_ns, bool) or not isinstance(start_sim_time_ns, Integral):
                raise _invalid("INVALID_CHUNK_START", "start_sim_time_ns must be an integer")
            start_ns = int(start_sim_time_ns)
            if start_ns < now_sim_ns:
                raise _invalid("INVALID_CHUNK_START", "start_sim_time_ns is in the past")
            if not grid_aligned(start_ns, self._step_ns):
                raise _invalid(
                    "INVALID_CHUNK_START", "start_sim_time_ns is not on the physics grid"
                )
        stream_id = f"{self._instance_id}:motion-{self._motion_seq + 1}"
        return ChunkAnchor(
            stream_id,
            0,
            start_ns,
            _zero_sample(self._hold_positions),
            self._io.profile.digest,
        )

    def start_stream(self, first: PreparedStreamChunk, wall_time_ns: int) -> MotionSnapshot:
        """Admit a first chunk; hold until its aligned start is actually stepped."""
        self._check_thread()
        self._events = []
        wall = self._validate_wall(wall_time_ns)
        if self._stream_config is None:
            raise _invalid(
                "STREAM_NOT_CONFIGURED", "NativeMotion was created without a stream config"
            )
        kind = chunk_kind(first)
        if kind == "CHUNK" and not isinstance(first, PreparedChunk):
            raise _invalid("INVALID_PREPARED_CHUNK", "first must be a PreparedChunk")
        if self._phase in _ACTIVE_PHASES:
            raise _invalid("MOTION_ACTIVE", "cannot overlap active/settling/stopping motion")
        if self._phase is Phase.FAILED:
            raise _invalid("FAILED_LATCHED", "a failed motion must be reset before a new start")
        expected_stream_id = f"{self._instance_id}:motion-{self._motion_seq + 1}"
        if first.anchor.stream_id != expected_stream_id:
            raise _invalid(
                "STALE_STREAM",
                "stream id is not the next generation of this motion instance",
            )
        if first.anchor.predecessor_seq != 0:
            raise _invalid("INVALID_CHUNK_ORDER", "first chunk predecessor_seq must be zero")
        if first.anchor.robot_profile_hash != self._io.profile.digest:
            raise _invalid(
                "PROFILE_MISMATCH", "chunk profile digest differs from the current profile"
            )
        if tuple(first.joints) != tuple(self._io.profile.joints):
            raise _invalid("PROFILE_MISMATCH", "chunk joints differ from the current profile")
        if tuple(first.max_accelerations) != self._config.max_accelerations:
            raise _invalid("PROFILE_MISMATCH", "chunk accelerations differ from the current config")
        if not grid_aligned(first.start_sim_time_ns, self._step_ns):
            raise _invalid("INVALID_CHUNK_START", "chunk start is not on the physics grid")
        if not grid_aligned(first.end_sim_time_ns, self._step_ns):
            raise _invalid("INVALID_CHUNK_GRID", "chunk end is not on the physics grid")
        if kind == "FOLLOWER":
            assert isinstance(first, PreparedFollowerPlan)
            if first.duration_grid_ns != self._step_ns:
                raise _invalid(
                    "FOLLOWER_GRID_REQUIRED",
                    "physical follower chunks must be prepared on the scene physics grid",
                )
            if first.plan.duration_ns % self._step_ns != 0:
                raise _invalid(
                    "INVALID_CHUNK_GRID",
                    "follower plan duration is not a multiple of the physics step",
                )
        now_sim_ns = _sim_time_ns(self._scene.data)
        if first.start_sim_time_ns < now_sim_ns:
            raise _invalid("INVALID_CHUNK_START", "chunk start is in the past")
        observation = observation_rejection(
            first.window, now_sim_ns, self._stream_config.max_observation_age_ns
        )
        if observation is not None:
            raise _invalid(observation, "first chunk observation is not admissible")
        if now_sim_ns > first.start_sim_time_ns - self._stream_config.min_lead_ns:
            raise _invalid("CHUNK_LATE", "first chunk missed the admission lead deadline")
        for index, (initial, hold) in enumerate(
            zip(first.anchor.reference.positions, self._hold_positions, strict=True)
        ):
            if not math.isclose(initial, hold, rel_tol=0.0, abs_tol=1e-9):
                raise _invalid(
                    "INITIAL_REFERENCE_MISMATCH",
                    f"chunk initial reference[{index}] does not match the current hold",
                )
        for value in first.anchor.reference.velocities:
            if not math.isclose(value, 0.0, rel_tol=0.0, abs_tol=1e-9):
                raise _invalid(
                    "INITIAL_REFERENCE_MISMATCH",
                    "first chunk must start from a zero-velocity hold",
                )

        self._accept_wall(wall)
        self._motion_seq += 1
        self._prepared = None
        self._result = None
        self._terminal_snapshot = None
        self._applied = None
        self._stop_plan = None
        self._stop_requested_wall_ns = None
        self._stop_sim_start_ns = None
        self._stop_reason = None
        self._latched_failure = None
        self._stable_since_ns = None
        self._stable_signature = None
        self._stop_stable_since_ns = None
        self._tracking_violation_since_ns = None
        self._tracking_violation_observed_sim_ns = now_sim_ns
        self._tracking_quality = "HOLD"
        self._contact_unload = {side: ContactUnload() for side in ("left", "right")}
        self._unloading_axes = frozenset()
        self._stream_initial_positions = first.anchor.reference.positions
        self._stream_final_positions = None
        self._stream_final_targets = None
        self._stream_final_end_ns = None
        schedule = StreamSchedule(first.anchor.stream_id, self._motion_seq)
        schedule.install_start(first)
        self._stream = schedule
        self._phase = Phase.ACCEPTED
        self._accepted_wall_ns = wall
        self._started_wall_ns = None
        self._started_sim_ns = None
        self._last_progress_wall_ns = wall
        self._last_progress_sim_ns = now_sim_ns
        self._events.append(schedule.make_event("ACCEPTED", first, now_sim_ns))
        measurement = self._io.measure()
        snapshot = self._build_snapshot(
            phase=Phase.ACCEPTED,
            reason="",
            reference=_zero_sample(self._hold_positions),
            measurement=measurement,
            reference_application_sim_time_ns=measurement.sim_time_ns,
            elapsed_ns=0,
            stable_for_ns=0,
            contacts=self._empty_contacts(),
            reference_kind="HOLD",
            chunk_seq=None,
        )
        self._last_snapshot = snapshot
        return snapshot

    def queue_chunk(self, successor: PreparedStreamChunk, wall_time_ns: int) -> ChunkSubmission:
        """Queue at most one validated future successor at the active chunk end."""
        self._check_thread()
        self._events = []
        wall = self._validate_wall(wall_time_ns)
        if self._stream_config is None:
            raise _invalid(
                "STREAM_NOT_CONFIGURED", "NativeMotion was created without a stream config"
            )
        successor_kind = chunk_kind(successor)
        if successor_kind == "CHUNK" and not isinstance(successor, PreparedChunk):
            raise _invalid("INVALID_PREPARED_CHUNK", "successor must be a PreparedChunk")
        self._accept_wall(wall)
        self._advance_deadlines(wall)
        schedule = self._stream
        if schedule is not None and schedule.kind != successor_kind:
            self._events.append(
                schedule.make_event(
                    "REJECTED",
                    successor,
                    _sim_time_ns(self._scene.data),
                    "STREAM_MODE_MISMATCH",
                )
            )
            return ChunkSubmission(False, "STREAM_MODE_MISMATCH", self._snapshot_for_call())
        if self._phase in (Phase.STOPPING, *_TERMINAL_PHASES):
            if self._phase is Phase.COMPLETED:
                reason = "STALE_STREAM"
            else:
                reason = (
                    self._stop_reason
                    or (self._result.reason if self._result is not None else "")
                    or self._latched_failure
                    or "STOPPING"
                )
            if schedule is not None:
                self._events.append(
                    schedule.make_event(
                        "REJECTED",
                        successor,
                        _sim_time_ns(self._scene.data),
                        reason,
                    )
                )
            return ChunkSubmission(False, reason, self._snapshot_for_call())
        if schedule is None or self._phase not in (
            Phase.ACCEPTED,
            Phase.RUNNING,
            Phase.SETTLING,
        ):
            return ChunkSubmission(False, "STALE_STREAM", self._snapshot_for_call())
        now_sim_ns = _sim_time_ns(self._scene.data)
        if schedule.active is None:
            reason = "STREAM_NOT_STARTED"
            self._events.append(schedule.make_event("REJECTED", successor, now_sim_ns, reason))
            return ChunkSubmission(False, reason, self._snapshot_for_call())
        if schedule.pending is not None:
            reason = "QUEUE_FULL"
            self._events.append(schedule.make_event("REJECTED", successor, now_sim_ns, reason))
            return ChunkSubmission(False, reason, self._snapshot_for_call())
        if schedule.active.settle_at_end or self._phase is Phase.SETTLING:
            reason = "FINAL_CHUNK"
            self._events.append(schedule.make_event("REJECTED", successor, now_sim_ns, reason))
            return ChunkSubmission(False, reason, self._snapshot_for_call())
        if successor_kind == "FOLLOWER":
            assert isinstance(successor, PreparedFollowerPlan)
            if successor.duration_grid_ns != self._step_ns:
                reason = "FOLLOWER_GRID_REQUIRED"
                self._events.append(schedule.make_event("REJECTED", successor, now_sim_ns, reason))
                return ChunkSubmission(False, reason, self._snapshot_for_call())
            if successor.plan.duration_ns % self._step_ns != 0:
                reason = "INVALID_CHUNK_GRID"
                self._events.append(schedule.make_event("REJECTED", successor, now_sim_ns, reason))
                return ChunkSubmission(False, reason, self._snapshot_for_call())

        reason, latches = boundary_rejection(
            schedule.active,
            successor,
            stream_id=schedule.stream_id,
            profile_digest=self._io.profile.digest,
            joints=tuple(self._io.profile.joints),
            max_accelerations=self._config.max_accelerations,
            now_sim_ns=now_sim_ns,
            min_lead_ns=self._stream_config.min_lead_ns,
            max_observation_age_ns=self._stream_config.max_observation_age_ns,
            step_ns=self._step_ns,
        )
        if reason is not None:
            self._events.append(schedule.make_event("REJECTED", successor, now_sim_ns, reason))
            if latches:
                self._begin_stop(wall, reason, reason)
            return ChunkSubmission(False, reason, self._snapshot_for_call())
        schedule.install_pending(successor)
        self._events.append(schedule.make_event("ACCEPTED", successor, now_sim_ns, "PENDING"))
        return ChunkSubmission(True, "ACCEPTED", self._snapshot_for_call())

    def step(self, wall_time_ns: int) -> MotionSnapshot:
        """Apply one reference/hold at simulation time t, then step once to t+dt."""
        self._check_thread()
        self._events = []
        wall = self._validate_wall(wall_time_ns)
        self._accept_wall(wall)
        if self._phase is Phase.IDLE or self._phase in _TERMINAL_PHASES:
            return self._hold_step_live()
        if self._advance_deadlines(wall):
            assert self._terminal_snapshot is not None
            return self._terminal_snapshot
        application_sim_ns = _sim_time_ns(self._scene.data)
        if self._stream is not None and self._phase in (
            Phase.ACCEPTED,
            Phase.RUNNING,
            Phase.SETTLING,
        ):
            return self._step_stream(wall, application_sim_ns)
        return self._step_legacy(wall, application_sim_ns)

    def _step_legacy(self, wall: int, application_sim_ns: int) -> MotionSnapshot:
        if self._phase is Phase.STOPPING:
            if self._stop_plan is None or self._stop_sim_start_ns is None:
                raise AssertionError("STOPPING without a braking plan and origin")
            if self._started_sim_ns is None:
                self._started_sim_ns = application_sim_ns
                self._started_wall_ns = wall
            elapsed_ns = application_sim_ns - self._stop_sim_start_ns
            reference = self._stop_plan.sample(elapsed_ns)
            reference_kind = "BRAKE"
        else:
            if self._prepared is None:
                raise AssertionError("active motion without a prepared plan")
            if self._started_sim_ns is None:
                self._started_sim_ns = application_sim_ns
                self._started_wall_ns = wall
            elapsed_ns = application_sim_ns - self._started_sim_ns
            reference = self._prepared.plan.sample(elapsed_ns)
            reference_kind = "HOLD" if elapsed_ns > self._prepared.duration_ns else "TRAJECTORY"

        applied = self._apply_and_measure(reference, application_sim_ns, elapsed_ns, reference_kind)
        self._record_progress(applied, wall)
        measurement = applied.measurement
        contacts = applied.contacts

        if self._phase is Phase.ACCEPTED:
            self._phase = Phase.RUNNING
        if self._phase in (Phase.RUNNING, Phase.SETTLING):
            assert self._prepared is not None and self._started_sim_ns is not None
            horizon_sim_ns = self._started_sim_ns + self._prepared.duration_ns
            if measurement.sim_time_ns >= horizon_sim_ns:
                self._phase = Phase.SETTLING
            completions = self._completions(measurement, contacts)
            self._update_normal_stability(measurement, completions, horizon_sim_ns)
            if self._phase in (Phase.RUNNING, Phase.SETTLING):
                settle_deadline_ns = horizon_sim_ns + self._config.settle_timeout_ns
                if measurement.sim_time_ns >= settle_deadline_ns:
                    self._begin_stop(
                        wall,
                        StopReason.TRACKING_TIMEOUT,
                        FailedReason.TRACKING_TIMEOUT,
                    )
                    if self._phase is Phase.FAILED:
                        assert self._terminal_snapshot is not None
                        return self._terminal_snapshot
        elif self._phase is Phase.STOPPING:
            completions = self._completions(measurement, contacts)
            self._update_stop_stability(measurement, elapsed_ns)
        else:
            completions = self._completions(measurement, contacts)

        if self._phase in _TERMINAL_PHASES:
            assert self._terminal_snapshot is not None
            return self._terminal_snapshot
        assert self._last_snapshot is not None
        snapshot = self._build_snapshot(
            phase=self._phase,
            reason=self._stop_reason if self._phase is Phase.STOPPING else "",
            reference=reference,
            measurement=measurement,
            reference_application_sim_time_ns=application_sim_ns,
            elapsed_ns=elapsed_ns,
            stable_for_ns=self._stable_for_ns(measurement),
            contacts=contacts,
            completions=completions,
            reference_kind=reference_kind,
        )
        self._last_snapshot = snapshot
        return snapshot

    def _step_stream(self, wall: int, application_sim_ns: int) -> MotionSnapshot:
        """Choose one reference, advance once, then process the measured result."""
        if self._phase is Phase.ACCEPTED:
            return self._step_stream_start(wall, application_sim_ns)
        schedule = self._stream
        assert schedule is not None and schedule.active is not None
        active = schedule.active
        boundary = None
        if self._phase is Phase.SETTLING:
            assert self._stream_final_positions is not None
            reference = _zero_sample(self._stream_final_positions)
            elapsed_ns = application_sim_ns - active.start_sim_time_ns
            kind, chunk_seq = "HOLD", active.seq
        elif application_sim_ns > active.end_sim_time_ns:
            # A skipped endpoint cannot be sampled or retrospectively applied.
            # Brake from the last reference that actually reached the actuator.
            self._begin_stop(wall, FailedReason.UNDERRUN.value, FailedReason.UNDERRUN.value)
            if self._phase is Phase.FAILED:
                assert self._terminal_snapshot is not None
                return self._terminal_snapshot
            assert self._stop_plan is not None and self._stop_sim_start_ns is not None
            elapsed_ns = application_sim_ns - self._stop_sim_start_ns
            reference = self._stop_plan.sample(elapsed_ns)
            kind, chunk_seq = "BRAKE", active.seq
        elif application_sim_ns == active.end_sim_time_ns:
            boundary = self._select_stream_boundary(active, application_sim_ns)
            reference, elapsed_ns = boundary.reference, boundary.elapsed_ns
            kind, chunk_seq = boundary.reference_kind, boundary.chunk_seq
        elif application_sim_ns < active.start_sim_time_ns:
            reference, elapsed_ns = _zero_sample(self._hold_positions), 0
            kind, chunk_seq = "HOLD", active.seq
        else:
            reference = active.sample_at(application_sim_ns)
            elapsed_ns = application_sim_ns - active.start_sim_time_ns
            kind, chunk_seq = "CHUNK", active.seq
        applied = self._apply_and_measure(reference, application_sim_ns, elapsed_ns, kind)
        if boundary is not None:
            self._apply_stream_boundary(wall, application_sim_ns, active, boundary)
        return self._finish_stream_step(wall, applied, kind, chunk_seq)

    def _step_stream_start(self, wall: int, application_sim_ns: int) -> MotionSnapshot:
        """Hold until the planned first application, then emit one STARTED."""
        schedule = self._stream
        assert schedule is not None and schedule.active is not None
        first = schedule.active
        at_start = application_sim_ns == first.start_sim_time_ns
        reference = (
            first.sample_at(application_sim_ns) if at_start else _zero_sample(self._hold_positions)
        )
        kind = "CHUNK" if at_start else "HOLD"
        applied = self._apply_and_measure(reference, application_sim_ns, 0, kind)
        if at_start:
            self._started_sim_ns = application_sim_ns
            self._started_wall_ns = wall
            self._phase = Phase.RUNNING
            schedule.started = True
            self._events.append(schedule.make_event("STARTED", first, application_sim_ns))
            if first.settle_at_end:
                self._remember_final_chunk(first)
        elif application_sim_ns > first.start_sim_time_ns:
            self._begin_stop(
                wall, FailedReason.STREAM_START_MISSED.value, FailedReason.STREAM_START_MISSED.value
            )
        return self._finish_stream_step(wall, applied, kind, first.seq if at_start else None)

    def _select_stream_boundary(
        self, active: PreparedStreamChunk, application_sim_ns: int
    ) -> _BoundarySelection:
        """Pure choice of endpoint, successor start, or underrun endpoint."""
        schedule = self._stream
        assert schedule is not None
        end_reference = active.sample_at(application_sim_ns)
        if active.settle_at_end:
            return _BoundarySelection(
                end_reference, active.duration_ns, "CHUNK", active.seq, "settle"
            )
        if schedule.pending is not None:
            successor = schedule.pending
            return _BoundarySelection(
                successor.sample_at(application_sim_ns),
                0,
                "CHUNK",
                successor.seq,
                "handoff",
            )
        return _BoundarySelection(
            end_reference, active.duration_ns, "CHUNK", active.seq, "underrun"
        )

    def _apply_stream_boundary(
        self,
        wall: int,
        application_sim_ns: int,
        active: PreparedStreamChunk,
        boundary: _BoundarySelection,
    ) -> None:
        """Apply post-step boundary state/events for the one chosen reference."""
        schedule = self._stream
        assert schedule is not None
        if boundary.action == "settle":
            self._phase = Phase.SETTLING
            self._hold_positions = boundary.reference.positions
            self._stream_final_positions = boundary.reference.positions
            if schedule.kind == "FOLLOWER":
                self._stream_final_targets = tuple(active.window.targets[-1].positions)
            else:
                self._stream_final_targets = boundary.reference.positions
            self._stream_final_end_ns = application_sim_ns
            self._events.append(
                schedule.make_event("RETIRED", active, application_sim_ns, "FINAL_HOLD")
            )
        elif boundary.action == "handoff":
            retired, started = schedule.handoff()
            self._events.append(
                schedule.make_event("RETIRED", retired, application_sim_ns, "HANDOFF")
            )
            self._events.append(
                schedule.make_event("STARTED", started, application_sim_ns, "HANDOFF")
            )
            if started.settle_at_end:
                self._remember_final_chunk(started)
        elif boundary.action == "underrun":
            self._begin_stop(
                wall,
                FailedReason.UNDERRUN.value,
                FailedReason.UNDERRUN.value,
            )
            self._events.append(
                schedule.make_event("RETIRED", active, application_sim_ns, "UNDERRUN")
            )

    def _finish_stream_step(
        self,
        wall: int,
        applied: _AppliedStep,
        reference_kind: str,
        chunk_seq: int | None,
    ) -> MotionSnapshot:
        """Update tracking and goal/stop stability after one real physical step."""
        self._record_progress(applied, wall)
        measurement, contacts = applied.measurement, applied.contacts
        if self._phase is Phase.STOPPING:
            self._update_stop_stability(measurement, applied.elapsed_ns)
        elif self._phase in (Phase.RUNNING, Phase.SETTLING):
            assert self._stream_config is not None
            self._update_stream_tracking(applied.reference, applied, contacts)
            if (
                self._tracking_violation_since_ns is not None
                and measurement.sim_time_ns - self._tracking_violation_since_ns
                >= self._stream_config.tracking_timeout_ns
            ):
                self._begin_stop(
                    wall, FailedReason.TRACKING_TIMEOUT.value, FailedReason.TRACKING_TIMEOUT.value
                )
            elif self._phase is Phase.SETTLING:
                assert self._stream_final_end_ns is not None
                completions = self._completions(measurement, contacts)
                self._update_normal_stability(measurement, completions, self._stream_final_end_ns)
                if (
                    self._phase is Phase.SETTLING
                    and measurement.sim_time_ns
                    >= self._stream_final_end_ns + self._config.settle_timeout_ns
                ):
                    self._begin_stop(
                        wall, StopReason.TRACKING_TIMEOUT, FailedReason.TRACKING_TIMEOUT
                    )
        if self._phase in _TERMINAL_PHASES:
            assert self._terminal_snapshot is not None
            return self._terminal_snapshot
        return self._stream_snapshot(
            self._phase,
            self._stop_reason if self._phase is Phase.STOPPING else "",
            applied.reference,
            applied,
            reference_kind,
            chunk_seq,
        )

    def _remember_final_chunk(self, chunk: PreparedStreamChunk) -> None:
        end = chunk.plan.sample_interval(chunk.plan.duration_ns)
        self._stream_final_positions = end.positions
        if chunk_kind(chunk) == "FOLLOWER":
            self._stream_final_targets = tuple(chunk.window.targets[-1].positions)
        else:
            self._stream_final_targets = end.positions
        self._stream_final_end_ns = chunk.end_sim_time_ns

    def _stream_snapshot(
        self,
        phase: Phase,
        reason: str,
        reference: ReferenceSample,
        applied: _AppliedStep,
        reference_kind: str,
        chunk_seq: int | None,
    ) -> MotionSnapshot:
        completions = self._completions(applied.measurement, applied.contacts)
        snapshot = self._build_snapshot(
            phase=phase,
            reason=reason,
            reference=reference,
            measurement=applied.measurement,
            reference_application_sim_time_ns=applied.application_sim_time_ns,
            elapsed_ns=applied.elapsed_ns,
            stable_for_ns=self._stable_for_ns(applied.measurement),
            contacts=applied.contacts,
            completions=completions,
            reference_kind=reference_kind,
            chunk_seq=chunk_seq,
            sampled_reference_positions=applied.sampled_reference.positions,
            sampled_reference_velocities=applied.sampled_reference.velocities,
            reference_boundary_corrections=applied.reference_corrections,
        )
        self._last_snapshot = snapshot
        return snapshot

    def poll(self, wall_time_ns: int) -> MotionSnapshot:
        """Return state and apply wall-clock deadline transitions without stepping.

        In a terminal phase this returns the frozen terminal snapshot; only
        ``step`` returns a fresh physical snapshot after the plant advances.
        """
        self._check_thread()
        self._events = []
        wall = self._validate_wall(wall_time_ns)
        self._accept_wall(wall)
        if self._phase is Phase.IDLE:
            measurement = self._io.measure()
            contacts = self._empty_contacts()
            reference = _zero_sample(self._hold_positions)
            snapshot = self._build_snapshot(
                phase=Phase.IDLE,
                reason="",
                reference=reference,
                measurement=measurement,
                reference_application_sim_time_ns=measurement.sim_time_ns,
                elapsed_ns=0,
                stable_for_ns=0,
                contacts=contacts,
            )
            self._last_snapshot = snapshot
            return snapshot
        if self._phase in _TERMINAL_PHASES:
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
        """Request a bounded brake; repeated requests never extend the deadline.

        Wall deadlines are checked before a late manual stop can replace a
        timeout-triggered stop with a clean STOPPED result.
        """
        self._check_thread()
        self._events = []
        wall = self._validate_wall(wall_time_ns)
        if not isinstance(reason, str) or not reason:
            raise _invalid("INVALID_REASON", "stop reason must be a non-empty string")
        if self._phase is Phase.IDLE:
            raise _invalid("MOTION_NOT_STARTED", "call start before request_stop")
        self._accept_wall(wall)
        if self._phase in _TERMINAL_PHASES:
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
        if self._events:
            snapshot = replace(
                self._last_snapshot,
                phase=Phase.STOPPING,
                reason=self._stop_reason or "",
                chunk_events=tuple(self._events),
            )
        else:
            snapshot = replace(
                self._last_snapshot,
                phase=Phase.STOPPING,
                reason=self._stop_reason or "",
            )
        self._last_snapshot = snapshot
        return snapshot

    # ------------------------------------------------------------- internals

    def _check_thread(self) -> None:
        if threading.current_thread() is not self._owner_thread:
            raise _invalid("WRONG_THREAD", "NativeMotion must be used on its creating thread")

    def _validate_wall(self, value: object) -> int:
        if isinstance(value, bool) or not isinstance(value, Integral):
            raise _invalid("INVALID_WALL_TIME", "wall_time_ns must be an integer")
        result = int(value)
        if result < 0:
            raise _invalid("INVALID_WALL_TIME", "wall_time_ns must not be negative")
        if result > _MAX_NS:
            raise _invalid("INVALID_WALL_TIME", "wall_time_ns exceeds the supported range")
        if self._last_wall_ns is not None and result < self._last_wall_ns:
            raise _invalid("WALL_TIME_REGRESSION", "wall_time_ns must not move backwards")
        return result

    def _accept_wall(self, value: int) -> None:
        self._last_wall_ns = value

    def _check_positions_within_profile(self, positions: tuple[float, ...]) -> None:
        for index, (position, joint) in enumerate(
            zip(positions, self._io.profile.joints, strict=True)
        ):
            if position < joint.minimum or position > joint.maximum:
                raise _invalid(
                    "JOINT_LIMIT",
                    f"initial_reference[{index}] ({joint.name}) is outside profile limits",
                )

    def _empty_contacts(self) -> tuple[GripperContactEvidence, ...]:
        return tuple(
            empty_contact_evidence(self._scene.data, self._binding, side)
            for side in ("left", "right")
        )

    def _measure_contacts(self) -> tuple[GripperContactEvidence, ...]:
        return tuple(
            measure_contact_evidence(
                self._scene.model,
                self._scene.data,
                self._binding,
                side,
            )
            for side in ("left", "right")
        )

    def _active_chunk_seq(self) -> int | None:
        if self._stream is not None and self._stream.active is not None:
            return self._stream.active.seq
        return None

    def _completion_bounds(self) -> tuple[tuple[float, ...], tuple[float, ...]]:
        """Initial and final/target positions for gripper completion evidence."""
        if self._stream is not None and self._stream_initial_positions is not None:
            final = self._stream_final_targets
            if final is None and self._stream.active is not None:
                final = self._stream.active.plan.sample_interval(
                    self._stream.active.plan.duration_ns
                ).positions
            if final is None:
                final = self._hold_positions
            return self._stream_initial_positions, final
        if self._prepared is not None:
            return self._prepared.initial_reference, self._prepared.final_positions
        return self._hold_positions, self._hold_positions

    def _completions(
        self, measurement: Measurement, contacts: tuple[GripperContactEvidence, ...]
    ) -> tuple[GripperCompletion, ...]:
        initial, final = self._completion_bounds()
        result: list[GripperCompletion] = []
        for side in ("left", "right"):
            index = _GRIPPER_INDEX[side]
            contact = next(item for item in contacts if item.side == side)
            result.append(
                evaluate_gripper_completion(
                    side=side,
                    initial_reference_m=initial[index],
                    final_reference_m=final[index],
                    target_position_m=final[index],
                    measured_position_m=measurement.positions[index],
                    measured_velocity_m_s=measurement.velocities[index],
                    position_tolerance_m=self._config.position_tolerances[index],
                    velocity_tolerance_m_s=self._config.velocity_tolerances[index],
                    force_min_n=self._config.contact_force_min_n,
                    finger_speed_max_m_s=self._config.finger_velocity_max_m_s,
                    contact=contact,
                )
            )
        return tuple(result)

    def _arm_stable(self, measurement: Measurement) -> bool:
        assert self._applied is not None
        return all(
            abs(measurement.positions[index] - self._applied.reference.positions[index])
            <= self._config.position_tolerances[index]
            and abs(measurement.velocities[index]) <= self._config.velocity_tolerances[index]
            for index in _ARM_INDICES
        )

    def _update_normal_stability(
        self,
        measurement: Measurement,
        completions: tuple[GripperCompletion, ...],
        horizon_sim_ns: int,
    ) -> None:
        assert self._applied is not None
        overall = self._arm_stable(measurement) and all(
            completion.mode != "NONE" for completion in completions
        )
        signature = tuple(
            (
                completion.mode,
                completion.contact_object if completion.mode == "CONTACT" else None,
            )
            for completion in completions
        )
        if measurement.sim_time_ns < horizon_sim_ns:
            overall = False
        if not overall:
            self._stable_since_ns = None
            self._stable_signature = None
            return
        if self._stable_since_ns is None or signature != self._stable_signature:
            self._stable_since_ns = measurement.sim_time_ns
            self._stable_signature = signature
        stable_for = measurement.sim_time_ns - self._stable_since_ns
        if stable_for >= self._config.stable_duration_ns:
            _initial, final = self._completion_bounds()
            self._hold_positions = final
            self._phase = Phase.COMPLETED
            snapshot = self._build_snapshot(
                phase=Phase.COMPLETED,
                reason="COMPLETED",
                reference=self._applied.reference,
                measurement=measurement,
                reference_application_sim_time_ns=self._applied.application_sim_time_ns,
                elapsed_ns=self._applied.application_sim_time_ns - (self._started_sim_ns or 0),
                stable_for_ns=stable_for,
                contacts=self._applied.contacts,
                completions=completions,
                reference_kind=self._applied.reference_kind,
                chunk_seq=self._active_chunk_seq(),
                sampled_reference_positions=self._applied.sampled_reference.positions,
                sampled_reference_velocities=self._applied.sampled_reference.velocities,
                reference_boundary_corrections=self._applied.reference_corrections,
            )
            self._freeze(snapshot, wall=self._last_wall_ns or 0, stop_reason=None)

    def _update_stop_stability(self, measurement: Measurement, elapsed_ns: int) -> None:
        assert self._stop_plan is not None and self._applied is not None
        speeds_ok = all(
            abs(velocity) <= tolerance
            for velocity, tolerance in zip(
                measurement.velocities, self._config.velocity_tolerances, strict=True
            )
        )
        if elapsed_ns >= self._stop_plan.duration_ns and speeds_ok:
            if self._stop_stable_since_ns is None:
                self._stop_stable_since_ns = measurement.sim_time_ns
            stable_for = measurement.sim_time_ns - self._stop_stable_since_ns
            if stable_for >= self._config.stable_duration_ns:
                self._hold_positions = self._stop_plan.hold_positions
                phase = Phase.FAILED if self._latched_failure else Phase.STOPPED
                reason = (
                    self._latched_failure or self._stop_reason or StopReason.STOP_REQUESTED.value
                )
                self._phase = phase
                snapshot = self._build_snapshot(
                    phase=phase,
                    reason=reason,
                    reference=self._applied.reference,
                    measurement=measurement,
                    reference_application_sim_time_ns=self._applied.application_sim_time_ns,
                    elapsed_ns=elapsed_ns,
                    stable_for_ns=stable_for,
                    contacts=self._applied.contacts,
                    completions=self._completions(measurement, self._applied.contacts),
                    reference_kind=self._applied.reference_kind,
                    chunk_seq=self._active_chunk_seq(),
                    sampled_reference_positions=self._applied.sampled_reference.positions,
                    sampled_reference_velocities=self._applied.sampled_reference.velocities,
                    reference_boundary_corrections=self._applied.reference_corrections,
                )
                self._freeze(snapshot, wall=self._last_wall_ns or 0, stop_reason=self._stop_reason)
        else:
            self._stop_stable_since_ns = None

    def _stable_for_ns(self, measurement: Measurement) -> int:
        if self._phase is Phase.STOPPING:
            if self._stop_stable_since_ns is None:
                return 0
            return max(0, measurement.sim_time_ns - self._stop_stable_since_ns)
        if self._stable_since_ns is None:
            return 0
        return max(0, measurement.sim_time_ns - self._stable_since_ns)

    def _begin_stop(self, wall: int, reason: str, latched: str | None) -> None:
        if self._phase is Phase.STOPPING or self._phase in _TERMINAL_PHASES:
            return
        if self._applied is None:
            base = _zero_sample(self._hold_positions)
        else:
            base = self._applied.reference
        if self._stream is not None:
            canceled = self._stream.cancel_pending()
            if canceled is not None:
                self._events.append(
                    self._stream.make_event(
                        "CANCELED", canceled, _sim_time_ns(self._scene.data), str(reason)
                    )
                )
            joints = (
                self._stream.active.joints
                if self._stream.active is not None
                else tuple(self._io.profile.joints)
            )
        else:
            assert self._prepared is not None
            joints = self._prepared.joints
        try:
            stop_plan = plan_braking(
                base.positions,
                base.velocities,
                joints,
                self._config.max_accelerations,
            )
        except ContractError:
            self._hold_positions = base.positions
            self._stop_reason = reason
            self._latched_failure = FailedReason.STOP_PLAN_REJECTED.value
            self._fail(FailedReason.STOP_PLAN_REJECTED.value, wall)
            return
        self._stop_plan = stop_plan
        self._stop_requested_wall_ns = wall
        # The base p/v belong to this application time, one physics step before
        # the next call. Sampling the brake at dt preserves the reference's
        # position/velocity continuation instead of repeating a moving target.
        self._stop_sim_start_ns = (
            _sim_time_ns(self._scene.data)
            if self._applied is None
            else self._applied.application_sim_time_ns
        )
        self._stop_reason = str(reason)
        self._latched_failure = str(latched) if latched is not None else None
        self._stop_stable_since_ns = None
        self._phase = Phase.STOPPING

    def _advance_deadlines(self, wall: int) -> bool:
        if self._phase in _TERMINAL_PHASES:
            return True
        if self._phase in (Phase.ACCEPTED, Phase.RUNNING, Phase.SETTLING):
            assert self._accepted_wall_ns is not None
            if self._stream is not None and self._stream_config is not None:
                if wall - self._accepted_wall_ns >= self._stream_config.stream_wall_timeout_ns:
                    self._begin_stop(wall, StopReason.WALL_TIMEOUT, FailedReason.WALL_TIMEOUT)
                elif (
                    self._last_progress_wall_ns is not None
                    and self._last_progress_sim_ns is not None
                    and wall - self._last_progress_wall_ns
                    >= self._stream_config.progress_wall_timeout_ns
                    and _sim_time_ns(self._scene.data) == self._last_progress_sim_ns
                ):
                    self._begin_stop(
                        wall,
                        FailedReason.PROGRESS_TIMEOUT.value,
                        FailedReason.PROGRESS_TIMEOUT.value,
                    )
                if (
                    self._phase in (Phase.RUNNING, Phase.SETTLING)
                    and self._applied is not None
                    and self._stream_final_end_ns is not None
                    and self._applied.measurement.sim_time_ns
                    >= self._stream_final_end_ns + self._config.settle_timeout_ns
                ):
                    self._begin_stop(
                        wall,
                        StopReason.TRACKING_TIMEOUT,
                        FailedReason.TRACKING_TIMEOUT,
                    )
                    if self._phase is Phase.FAILED:
                        return True
            else:
                if wall - self._accepted_wall_ns >= self._config.motion_wall_timeout_ns:
                    self._begin_stop(wall, StopReason.WALL_TIMEOUT, FailedReason.WALL_TIMEOUT)
                    if self._phase is Phase.FAILED:
                        return True
                if (
                    self._phase in (Phase.RUNNING, Phase.SETTLING)
                    and self._applied is not None
                    and self._prepared is not None
                    and self._started_sim_ns is not None
                    and self._applied.measurement.sim_time_ns
                    >= self._started_sim_ns
                    + self._prepared.duration_ns
                    + self._config.settle_timeout_ns
                ):
                    self._begin_stop(
                        wall,
                        StopReason.TRACKING_TIMEOUT,
                        FailedReason.TRACKING_TIMEOUT,
                    )
                    if self._phase is Phase.FAILED:
                        return True
        if self._phase is Phase.STOPPING:
            assert self._stop_requested_wall_ns is not None
            if wall - self._stop_requested_wall_ns >= self._config.stop_wall_timeout_ns:
                self._fail(FailedReason.STOP_TIMEOUT.value, wall)
                return True
        return False

    def _fail(self, reason: str, wall: int) -> None:
        self._phase = Phase.FAILED
        if self._applied is not None:
            self._hold_positions = self._applied.reference.positions
        if self._applied is None:
            measurement = self._io.measure()
            contacts = self._empty_contacts()
            reference = _zero_sample(self._hold_positions)
            reference_application_sim_time_ns = measurement.sim_time_ns
            elapsed_ns = 0
        else:
            measurement = self._applied.measurement
            contacts = self._applied.contacts
            reference = self._applied.reference
            reference_application_sim_time_ns = self._applied.application_sim_time_ns
            elapsed_ns = self._applied.elapsed_ns
        snapshot = self._build_snapshot(
            phase=Phase.FAILED,
            reason=str(reason),
            reference=reference,
            measurement=measurement,
            reference_application_sim_time_ns=reference_application_sim_time_ns,
            elapsed_ns=elapsed_ns,
            stable_for_ns=self._stable_for_ns(measurement),
            contacts=contacts,
            reference_kind=(self._applied.reference_kind if self._applied is not None else "HOLD"),
            chunk_seq=self._active_chunk_seq(),
            sampled_reference_positions=(
                None if self._applied is None else self._applied.sampled_reference.positions
            ),
            sampled_reference_velocities=(
                None if self._applied is None else self._applied.sampled_reference.velocities
            ),
            reference_boundary_corrections=(
                () if self._applied is None else self._applied.reference_corrections
            ),
        )
        self._last_snapshot = snapshot
        self._freeze(snapshot, wall=wall, stop_reason=self._stop_reason)

    def _freeze(self, snapshot: MotionSnapshot, wall: int, stop_reason: str | None) -> None:
        if self._stream is not None:
            self._stream.close()
        result = MotionResult(
            motion_seq=snapshot.motion_seq,
            phase=snapshot.phase,
            reason=snapshot.reason,
            stop_reason=stop_reason,
            accepted_wall_ns=snapshot.accepted_wall_ns,
            started_wall_ns=snapshot.started_wall_ns,
            started_sim_ns=snapshot.started_sim_ns,
            finished_wall_ns=wall,
            snapshot=snapshot,
        )
        self._terminal_snapshot = snapshot
        self._result = result

    def _apply_and_measure(
        self,
        reference: ReferenceSample,
        application_sim_ns: int,
        elapsed_ns: int,
        reference_kind: str,
    ) -> _AppliedStep:
        """The one real application/step/forward/measure sequence in the module."""
        data = self._scene.data
        sampled_reference = reference
        applied_reference = reference
        corrections: tuple[float, ...] = ()
        if self._stream is not None and self._stream.kind == "FOLLOWER":
            normalized = normalize_follower_positions(
                reference.positions, tuple(self._io.profile.joints)
            )
            corrections = normalized.corrections
            if normalized.has_correction:
                applied_reference = ReferenceSample(
                    normalized.applied_positions,
                    reference.velocities,
                    reference.accelerations,
                )
        self._io.apply_reference(applied_reference.positions, applied_reference.velocities)
        import mujoco

        mujoco.mj_step(self._scene.model, data)
        mujoco.mj_forward(self._scene.model, data)
        measurement = self._io.measure()
        contacts = self._measure_contacts()
        applied = _AppliedStep(
            applied_reference,
            measurement,
            contacts,
            application_sim_ns,
            elapsed_ns,
            reference_kind,
            sampled_reference,
            corrections,
        )
        self._applied = applied
        return applied

    def _record_progress(self, applied: _AppliedStep, wall: int) -> None:
        if self._last_progress_sim_ns != applied.measurement.sim_time_ns:
            self._last_progress_sim_ns = applied.measurement.sim_time_ns
            self._last_progress_wall_ns = wall

    def _constrained_axes(
        self,
        reference: ReferenceSample,
        measurement: Measurement,
        contacts: tuple[GripperContactEvidence, ...],
    ) -> frozenset[int]:
        """Exempt only a slow, same-object contact opposing a closing reference."""
        constrained: set[int] = set()
        unloading: set[int] = set()
        for side in ("left", "right"):
            index = _GRIPPER_INDEX[side]
            target = reference.positions[index]
            contact = next(item for item in contacts if item.side == side)
            completion = evaluate_gripper_completion(
                side=side,
                initial_reference_m=target,
                final_reference_m=target,
                target_position_m=target,
                measured_position_m=measurement.positions[index],
                measured_velocity_m_s=measurement.velocities[index],
                position_tolerance_m=self._config.position_tolerances[index],
                velocity_tolerance_m_s=self._config.velocity_tolerances[index],
                force_min_n=self._config.contact_force_min_n,
                finger_speed_max_m_s=self._config.finger_velocity_max_m_s,
                contact=contact,
            )
            closing_contact = (
                completion.contact_ok
                and reference.velocities[index] <= REFERENCE_VELOCITY_ATOL_M_S
            )
            if closing_contact:
                constrained.add(index)
            timeout_ns = (
                0
                if self._stream_config is None
                else round(self._stream_config.contact_unload_timeout_s * 1_000_000_000)
            )
            if self._contact_unload[side].update(
                now_ns=measurement.sim_time_ns,
                timeout_ns=timeout_ns,
                reference_position=target,
                reference_velocity=reference.velocities[index],
                measured_position=measurement.positions[index],
                position_tolerance=self._config.position_tolerances[index],
                closing_contact_ok=closing_contact,
                contact=contact,
            ):
                constrained.add(index)
                unloading.add(index)
        self._unloading_axes = frozenset(unloading)
        return frozenset(constrained)

    def _update_stream_tracking(
        self,
        reference: ReferenceSample,
        applied: _AppliedStep,
        contacts: tuple[GripperContactEvidence, ...],
    ) -> None:
        measurement = applied.measurement
        constrained = self._constrained_axes(reference, measurement, contacts)
        violating = False
        for index in range(len(reference.positions)):
            if index in constrained:
                continue
            if (
                abs(reference.positions[index] - measurement.positions[index])
                > (self._config.position_tolerances[index])
                or abs(reference.velocities[index] - measurement.velocities[index])
                > (self._config.velocity_tolerances[index])
            ):
                violating = True
                break
        if violating:
            if self._tracking_violation_since_ns is None:
                self._tracking_violation_since_ns = measurement.sim_time_ns
            self._tracking_violation_observed_sim_ns = measurement.sim_time_ns
            self._tracking_quality = "OUTSIDE_TOLERANCE"
        else:
            self._tracking_violation_since_ns = None
            self._tracking_quality = (
                "CONTACT_UNLOADING"
                if self._unloading_axes
                else "CONSTRAINED"
                if constrained
                else "FOLLOWING"
            )

    def _snapshot_for_call(self) -> MotionSnapshot:
        if self._last_snapshot is not None:
            reason = (
                self._stop_reason
                if self._phase is Phase.STOPPING
                else (self._last_snapshot.reason if self._phase in _TERMINAL_PHASES else "")
            )
            return replace(
                self._last_snapshot,
                phase=self._phase,
                reason=reason or "",
                chunk_events=tuple(self._events),
            )
        measurement = self._io.measure()
        return self._build_snapshot(
            phase=self._phase,
            reason="",
            reference=_zero_sample(self._hold_positions),
            measurement=measurement,
            reference_application_sim_time_ns=measurement.sim_time_ns,
            elapsed_ns=0,
            stable_for_ns=0,
            contacts=self._empty_contacts(),
            reference_kind="HOLD",
        )

    def _build_snapshot(
        self,
        *,
        phase: Phase,
        reason: str,
        reference: ReferenceSample,
        measurement: Measurement,
        reference_application_sim_time_ns: int,
        elapsed_ns: int,
        stable_for_ns: int,
        contacts: tuple[GripperContactEvidence, ...],
        completions: tuple[GripperCompletion, ...] | None = None,
        reference_kind: str = "TRAJECTORY",
        chunk_seq: int | None = None,
        sampled_reference_positions: tuple[float, ...] | None = None,
        sampled_reference_velocities: tuple[float, ...] | None = None,
        reference_boundary_corrections: tuple[float, ...] = (),
    ) -> MotionSnapshot:
        if completions is None:
            if self._stream is not None or self._prepared is not None:
                completions = self._completions(measurement, contacts)
            else:
                completions = self._hold_completions(measurement, contacts)
        errors = tuple(
            commanded - measured
            for commanded, measured in zip(reference.positions, measurement.positions, strict=True)
        )
        velocity_errors = tuple(
            commanded - measured
            for commanded, measured in zip(
                reference.velocities, measurement.velocities, strict=True
            )
        )
        violation_ns = 0
        if self._tracking_violation_since_ns is not None:
            violation_ns = max(
                0, self._tracking_violation_observed_sim_ns - self._tracking_violation_since_ns
            )
        return MotionSnapshot(
            phase=phase,
            motion_seq=self._motion_seq,
            reason=reason,
            accepted_wall_ns=self._accepted_wall_ns,
            started_wall_ns=self._started_wall_ns,
            started_sim_ns=self._started_sim_ns,
            reference_application_sim_time_ns=reference_application_sim_time_ns,
            reference_elapsed_ns=elapsed_ns,
            measurement_sim_time_ns=measurement.sim_time_ns,
            stable_for_ns=stable_for_ns,
            joint_names=tuple(self._io.joint_names),
            reference_positions=reference.positions,
            reference_velocities=reference.velocities,
            reference_accelerations=reference.accelerations,
            measured_positions=measurement.positions,
            measured_velocities=measurement.velocities,
            tracking_errors=errors,
            grippers=completions,
            contacts=contacts,
            stream_id=self._stream.stream_id if self._stream is not None else None,
            chunk_seq=chunk_seq,
            reference_kind=reference_kind,
            tracking_velocity_errors=velocity_errors,
            tracking_quality=self._tracking_quality,
            tracking_violation_ns=violation_ns,
            chunk_events=tuple(self._events),
            sampled_reference_positions=(
                reference.positions
                if sampled_reference_positions is None
                else sampled_reference_positions
            ),
            sampled_reference_velocities=(
                reference.velocities
                if sampled_reference_velocities is None
                else sampled_reference_velocities
            ),
            reference_boundary_corrections=reference_boundary_corrections,
        )

    def _hold_completions(
        self, measurement: Measurement, contacts: tuple[GripperContactEvidence, ...]
    ) -> tuple[GripperCompletion, ...]:
        result: list[GripperCompletion] = []
        for side in ("left", "right"):
            index = _GRIPPER_INDEX[side]
            contact = next(item for item in contacts if item.side == side)
            target = self._hold_positions[index]
            result.append(
                evaluate_gripper_completion(
                    side=side,
                    initial_reference_m=target,
                    final_reference_m=target,
                    target_position_m=target,
                    measured_position_m=measurement.positions[index],
                    measured_velocity_m_s=measurement.velocities[index],
                    position_tolerance_m=self._config.position_tolerances[index],
                    velocity_tolerance_m_s=self._config.velocity_tolerances[index],
                    force_min_n=self._config.contact_force_min_n,
                    finger_speed_max_m_s=self._config.finger_velocity_max_m_s,
                    contact=contact,
                )
            )
        return tuple(result)

    def _hold_step_live(self) -> MotionSnapshot:
        application_sim_ns = _sim_time_ns(self._scene.data)
        reference = _zero_sample(self._hold_positions)
        elapsed_ns = (
            0 if self._started_sim_ns is None else max(0, application_sim_ns - self._started_sim_ns)
        )
        applied = self._apply_and_measure(reference, application_sim_ns, elapsed_ns, "HOLD")
        reason = self._terminal_snapshot.reason if self._phase in _TERMINAL_PHASES else ""
        snapshot = self._build_snapshot(
            phase=self._phase,
            reason=reason,
            reference=reference,
            measurement=applied.measurement,
            reference_application_sim_time_ns=application_sim_ns,
            elapsed_ns=elapsed_ns,
            stable_for_ns=0,
            contacts=applied.contacts,
            completions=self._hold_completions(applied.measurement, applied.contacts),
            reference_kind="HOLD",
        )
        self._last_snapshot = snapshot
        return snapshot
