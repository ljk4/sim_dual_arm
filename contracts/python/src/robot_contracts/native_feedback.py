"""Pure immutable native feedback values shared by backend and ROS adapters.

Only the standard library and existing contracts are used here.  Feedback keeps
the real command/stream identity, optional started/finished flags, actual
reference kind, measurement time and per-finger contact evidence.  It never
recomputes a terminal snapshot from a later robot state and never reads a
clock, scene or hidden physical truth.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Integral, Real

from .control import INT64_MAX, MAX_ID_LENGTH, UINT64_MAX, validate_context
from .models import ContractError, RunContext

MAX_FINGER_OBJECTS = 8
MAX_GRIPPER_FINGERS = 2
COMMAND_PHASES = frozenset({"ACCEPTED", "STARTED", "STOPPING", "COMPLETED", "CANCELED", "FAILED"})
TERMINAL_COMMAND_PHASES = frozenset({"COMPLETED", "CANCELED", "FAILED"})
CHUNK_PHASES = frozenset({"ACCEPTED", "STARTED", "RETIRED", "CANCELED", "REJECTED"})
TERMINAL_CHUNK_PHASES = frozenset({"RETIRED", "CANCELED", "REJECTED"})
MOTION_PHASES = frozenset(
    {"", "IDLE", "ACCEPTED", "RUNNING", "SETTLING", "STOPPING", "COMPLETED", "STOPPED", "FAILED"}
)
REFERENCE_KINDS = frozenset({"HOLD", "TRAJECTORY", "CHUNK", "BRAKE"})
TRACKING_QUALITIES = frozenset(
    {"HOLD", "FOLLOWING", "CONSTRAINED", "CONTACT_UNLOADING", "OUTSIDE_TOLERANCE"}
)
GRIPPER_MODES = frozenset({"NONE", "POSITION", "CONTACT"})
EVENT_PHASES = COMMAND_PHASES | CHUNK_PHASES | frozenset({"PENDING", "HANDOFF"})


def _invalid(reason: str, location: str, message: str) -> ContractError:
    return ContractError(reason, f"{location}: {message}")


def _integer(
    value: object,
    location: str,
    minimum: int,
    maximum: int,
    *,
    reason: str = "INVALID_NATIVE_FEEDBACK",
) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise _invalid(reason, location, "must be an integer")
    result = int(value)
    if result < minimum or result > maximum:
        raise _invalid(reason, location, f"must be in {minimum}..{maximum}")
    return result


def _optional_integer(
    value: object,
    location: str,
    maximum: int,
    *,
    reason: str,
) -> int | None:
    if value is None:
        return None
    return _integer(value, location, 0, maximum, reason=reason)


def _finite(value: object, location: str, reason: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise _invalid(reason, location, "must be a finite real number")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise _invalid(reason, location, "cannot be represented as a finite float") from error
    if not math.isfinite(result):
        raise _invalid(reason, location, "must be finite")
    return result


def _vector(value: object, location: str, reason: str) -> tuple[float, ...]:
    if isinstance(value, str | bytes):
        raise _invalid(reason, location, "must be an array of real numbers")
    try:
        items = tuple(value)  # type: ignore[arg-type]
    except TypeError as error:
        raise _invalid(reason, location, "must be an array of real numbers") from error
    return tuple(_finite(item, f"{location}[{index}]", reason) for index, item in enumerate(items))


def _names(value: object, location: str, reason: str, maximum: int) -> tuple[str, ...]:
    if isinstance(value, str | bytes):
        raise _invalid(reason, location, "must be an array of names")
    try:
        items = tuple(value)  # type: ignore[arg-type]
    except TypeError as error:
        raise _invalid(reason, location, "must be an array of names") from error
    if not 1 <= len(items) <= maximum:
        raise _invalid(reason, location, f"must contain 1..{maximum} names")
    result: list[str] = []
    for index, item in enumerate(items):
        if not isinstance(item, str) or not item or len(item) > MAX_ID_LENGTH:
            raise _invalid(reason, f"{location}[{index}]", "must be a non-empty string")
        if item in result:
            raise _invalid(reason, f"{location}[{index}]", "must be unique")
        result.append(item)
    return tuple(result)


def _text(
    value: object,
    location: str,
    reason: str,
    *,
    allow_empty: bool = False,
    maximum: int = MAX_ID_LENGTH,
) -> str:
    if not isinstance(value, str):
        raise _invalid(reason, location, "must be a string")
    if not value and not allow_empty:
        raise _invalid(reason, location, "must not be empty")
    if len(value) > maximum:
        raise _invalid(reason, location, f"must be at most {maximum} characters")
    return value


@dataclass(frozen=True, slots=True)
class NativeFinger:
    """One finger's summed normal forces/counts for each allowed object."""

    side: str
    finger_index: int
    body_name: str
    joint_speed_m_s: float
    object_names: tuple[str, ...]
    normal_forces: tuple[float, ...]
    contact_counts: tuple[int, ...]

    def __post_init__(self) -> None:
        reason = "INVALID_NATIVE_GRIPPER"
        if self.side not in ("left", "right"):
            raise _invalid(reason, "side", "must be 'left' or 'right'")
        if isinstance(self.finger_index, bool) or self.finger_index not in (1, 2):
            raise _invalid(reason, "finger_index", "must be 1 or 2")
        body_name = _text(self.body_name, "body_name", reason)
        speed = _finite(self.joint_speed_m_s, "joint_speed_m_s", reason)
        if speed < 0.0:
            raise _invalid(reason, "joint_speed_m_s", "must not be negative")
        if isinstance(self.object_names, str | bytes):
            raise _invalid(reason, "object_names", "must be an array of names")
        try:
            names = tuple(self.object_names)
        except TypeError as error:
            raise _invalid(reason, "object_names", "must be an array of names") from error
        if len(names) > MAX_FINGER_OBJECTS:
            raise _invalid(
                reason, "object_names", f"must contain at most {MAX_FINGER_OBJECTS} names"
            )
        seen: set[str] = set()
        for index, name in enumerate(names):
            if not isinstance(name, str) or not name:
                raise _invalid(reason, f"object_names[{index}]", "must be a non-empty string")
            if name in seen:
                raise _invalid(reason, "object_names", "must be unique")
            seen.add(name)
        try:
            force_items = tuple(self.normal_forces)
            count_items = tuple(self.contact_counts)
        except TypeError as error:
            raise _invalid(reason, "normal_forces/contact_counts", "must be arrays") from error
        if len(force_items) != len(names) or len(count_items) != len(names):
            raise _invalid(
                reason,
                "normal_forces/contact_counts",
                "must be the same length as object_names",
            )
        forces: list[float] = []
        counts: list[int] = []
        for index, (force, count) in enumerate(zip(force_items, count_items, strict=True)):
            value = _finite(force, f"normal_forces[{index}]", reason)
            if value < 0.0:
                raise _invalid(reason, f"normal_forces[{index}]", "must not be negative")
            forces.append(value)
            counts.append(
                _integer(
                    count,
                    f"contact_counts[{index}]",
                    0,
                    UINT64_MAX,
                    reason=reason,
                )
            )
        object.__setattr__(self, "body_name", body_name)
        object.__setattr__(self, "joint_speed_m_s", speed)
        object.__setattr__(self, "object_names", names)
        object.__setattr__(self, "normal_forces", tuple(forces))
        object.__setattr__(self, "contact_counts", tuple(counts))


@dataclass(frozen=True, slots=True)
class NativeGripper:
    """One gripper's original target, measured state and per-finger evidence."""

    side: str
    mode: str
    position_ok: bool
    contact_ok: bool
    contact_object: str | None
    ambiguous: bool
    target_position_m: float
    initial_reference_m: float
    final_reference_m: float
    measured_position_m: float
    measured_velocity_m_s: float
    target_error_m: float
    fingers: tuple[NativeFinger, ...]

    def __post_init__(self) -> None:
        reason = "INVALID_NATIVE_GRIPPER"
        if self.side not in ("left", "right"):
            raise _invalid(reason, "side", "must be 'left' or 'right'")
        if self.mode not in GRIPPER_MODES:
            raise _invalid(reason, "mode", f"must be one of {sorted(GRIPPER_MODES)}")
        if not isinstance(self.position_ok, bool) or not isinstance(self.contact_ok, bool):
            raise _invalid(reason, "position_ok/contact_ok", "must be bool")
        if not isinstance(self.ambiguous, bool):
            raise _invalid(reason, "ambiguous", "must be bool")
        contact_object = self.contact_object
        if contact_object is not None:
            contact_object = _text(contact_object, "contact_object", reason)
        if self.mode == "POSITION" and not self.position_ok:
            raise _invalid(reason, "mode", "POSITION requires position_ok")
        if self.mode == "CONTACT":
            if not self.contact_ok:
                raise _invalid(reason, "mode", "CONTACT requires contact_ok")
            if contact_object is None:
                raise _invalid(reason, "mode", "CONTACT requires a contact object")
        if self.mode == "NONE" and (self.position_ok or self.contact_ok):
            raise _invalid(reason, "mode", "NONE cannot carry a successful flag")
        if self.contact_ok:
            if contact_object is None:
                raise _invalid(reason, "contact_ok", "requires a contact object")
            if self.ambiguous:
                raise _invalid(reason, "contact_ok", "cannot be true when ambiguous")

        target = _finite(self.target_position_m, "target_position_m", reason)
        initial = _finite(self.initial_reference_m, "initial_reference_m", reason)
        final = _finite(self.final_reference_m, "final_reference_m", reason)
        measured = _finite(self.measured_position_m, "measured_position_m", reason)
        velocity = _finite(self.measured_velocity_m_s, "measured_velocity_m_s", reason)
        error = _finite(self.target_error_m, "target_error_m", reason)
        if not math.isclose(error, target - measured, rel_tol=1e-9, abs_tol=1e-12):
            raise _invalid(reason, "target_error_m", "must equal target - measured")

        try:
            finger_items = tuple(self.fingers)
        except TypeError as error_object:
            raise _invalid(reason, "fingers", "must be an array") from error_object
        if len(finger_items) != MAX_GRIPPER_FINGERS:
            raise _invalid(reason, "fingers", "must contain exactly two fingers")
        for index, finger in enumerate(finger_items):
            if not isinstance(finger, NativeFinger):
                raise _invalid(reason, f"fingers[{index}]", "must be a NativeFinger")
            if finger.side != self.side:
                raise _invalid(reason, f"fingers[{index}].side", "must match the gripper")
        if tuple(finger.finger_index for finger in finger_items) != (1, 2):
            raise _invalid(reason, "fingers", "must be ordered finger_index 1 then 2")

        object.__setattr__(self, "contact_object", contact_object)
        object.__setattr__(self, "target_position_m", target)
        object.__setattr__(self, "initial_reference_m", initial)
        object.__setattr__(self, "final_reference_m", final)
        object.__setattr__(self, "measured_position_m", measured)
        object.__setattr__(self, "measured_velocity_m_s", velocity)
        object.__setattr__(self, "target_error_m", error)
        object.__setattr__(self, "fingers", finger_items)


def _command_identity(
    context: object,
    command_id: object,
    command_seq: object,
    observation_id: object,
    reason: str,
) -> tuple[RunContext, str, int, int]:
    if not isinstance(context, RunContext):
        raise _invalid(reason, "context", "must be a RunContext")
    validate_context(context)
    return (
        context,
        _text(command_id, "command_id", reason),
        _integer(command_seq, "command_seq", 1, UINT64_MAX, reason=reason),
        _integer(observation_id, "observation_id", 1, UINT64_MAX, reason=reason),
    )


@dataclass(frozen=True, slots=True)
class NativeExecution:
    """Frozen overall execution feedback derived from a ControlStatus snapshot."""

    context: RunContext
    command_id: str
    command_seq: int
    observation_id: int
    phase: str
    reason: str
    stop_reason: str
    motion_seq: int | None
    stream_id: str
    chunk_seq: int
    accepted_sim_time_ns: int
    accepted_wall_time_ns: int
    has_started: bool
    started_sim_time_ns: int | None
    started_wall_time_ns: int | None
    has_finished: bool
    finished_sim_time_ns: int | None
    finished_wall_time_ns: int | None
    motion_phase: str
    reference_application_sim_time_ns: int
    measurement_sim_time_ns: int
    reference_elapsed_ns: int
    stable_for_ns: int
    joint_names: tuple[str, ...]
    reference_positions: tuple[float, ...]
    reference_velocities: tuple[float, ...]
    reference_accelerations: tuple[float, ...]
    measured_positions: tuple[float, ...]
    measured_velocities: tuple[float, ...]
    position_errors: tuple[float, ...]
    velocity_errors: tuple[float, ...]
    reference_kind: str
    tracking_quality: str
    tracking_violation_ns: int
    grippers: tuple[NativeGripper, ...]

    def __post_init__(self) -> None:
        reason = "INVALID_NATIVE_EXECUTION"
        context, command_id, command_seq, observation_id = _command_identity(
            self.context,
            self.command_id,
            self.command_seq,
            self.observation_id,
            reason,
        )
        if self.phase not in COMMAND_PHASES:
            raise _invalid(reason, "phase", f"must be one of {sorted(COMMAND_PHASES)}")
        reason_text = _text(self.reason, "reason", reason, allow_empty=True)
        stop_reason = _text(self.stop_reason, "stop_reason", reason, allow_empty=True)
        motion_seq = _optional_integer(self.motion_seq, "motion_seq", UINT64_MAX, reason=reason)
        if motion_seq == 0:
            raise _invalid(reason, "motion_seq", "zero is not a motion sequence")
        stream_id = _text(self.stream_id, "stream_id", reason, allow_empty=True)
        chunk_seq = _integer(self.chunk_seq, "chunk_seq", 0, UINT64_MAX, reason=reason)
        accepted_sim = _integer(
            self.accepted_sim_time_ns, "accepted_sim_time_ns", 0, INT64_MAX, reason=reason
        )
        accepted_wall = _integer(
            self.accepted_wall_time_ns, "accepted_wall_time_ns", 0, INT64_MAX, reason=reason
        )
        if not isinstance(self.has_started, bool) or not isinstance(self.has_finished, bool):
            raise _invalid(reason, "has_started/has_finished", "must be bool")
        started_sim = _optional_integer(
            self.started_sim_time_ns, "started_sim_time_ns", INT64_MAX, reason=reason
        )
        started_wall = _optional_integer(
            self.started_wall_time_ns, "started_wall_time_ns", INT64_MAX, reason=reason
        )
        finished_sim = _optional_integer(
            self.finished_sim_time_ns, "finished_sim_time_ns", INT64_MAX, reason=reason
        )
        finished_wall = _optional_integer(
            self.finished_wall_time_ns, "finished_wall_time_ns", INT64_MAX, reason=reason
        )
        if self.has_started != (started_sim is not None and started_wall is not None):
            raise _invalid(
                reason,
                "has_started",
                "must match the presence of both started sim/wall times",
            )
        if self.has_finished != (finished_sim is not None and finished_wall is not None):
            raise _invalid(
                reason,
                "has_finished",
                "must match the presence of both finished sim/wall times",
            )
        if started_sim is not None and started_sim < accepted_sim:
            raise _invalid(reason, "started_sim_time_ns", "must not precede acceptance")
        if finished_sim is not None and finished_sim < accepted_sim:
            raise _invalid(reason, "finished_sim_time_ns", "must not precede acceptance")
        if started_sim is not None and finished_sim is not None and finished_sim < started_sim:
            raise _invalid(reason, "finished_sim_time_ns", "must not precede start")
        terminal = self.phase in TERMINAL_COMMAND_PHASES
        if terminal != self.has_finished:
            raise _invalid(
                reason,
                "has_finished",
                "must be true exactly for terminal command phases",
            )
        if self.motion_phase not in MOTION_PHASES:
            raise _invalid(reason, "motion_phase", f"must be one of {sorted(MOTION_PHASES)}")
        if (self.motion_phase in ("COMPLETED", "STOPPED", "FAILED")) != self.has_finished:
            raise _invalid(
                reason,
                "motion_phase",
                "terminal motion phases require has_finished and vice versa",
            )
        application = _integer(
            self.reference_application_sim_time_ns,
            "reference_application_sim_time_ns",
            0,
            INT64_MAX,
            reason=reason,
        )
        measurement = _integer(
            self.measurement_sim_time_ns,
            "measurement_sim_time_ns",
            0,
            INT64_MAX,
            reason=reason,
        )
        if measurement < application:
            raise _invalid(reason, "measurement_sim_time_ns", "must not precede application")
        elapsed = _integer(
            self.reference_elapsed_ns, "reference_elapsed_ns", 0, INT64_MAX, reason=reason
        )
        stable = _integer(self.stable_for_ns, "stable_for_ns", 0, INT64_MAX, reason=reason)
        names = _names(self.joint_names, "joint_names", reason, 64)
        vectors = {
            name: _vector(getattr(self, name), name, reason)
            for name in (
                "reference_positions",
                "reference_velocities",
                "reference_accelerations",
                "measured_positions",
                "measured_velocities",
                "position_errors",
                "velocity_errors",
            )
        }
        for name, values in vectors.items():
            if len(values) != len(names):
                raise _invalid(reason, name, f"has {len(values)} values for {len(names)} joints")
        for reference, measured, error in zip(
            vectors["reference_positions"],
            vectors["measured_positions"],
            vectors["position_errors"],
            strict=True,
        ):
            if not math.isclose(reference - measured, error, rel_tol=1e-9, abs_tol=1e-12):
                raise _invalid(reason, "position_errors", "must equal reference - measured")
        for reference, measured, error in zip(
            vectors["reference_velocities"],
            vectors["measured_velocities"],
            vectors["velocity_errors"],
            strict=True,
        ):
            if not math.isclose(reference - measured, error, rel_tol=1e-9, abs_tol=1e-12):
                raise _invalid(reason, "velocity_errors", "must equal reference - measured")
        if self.reference_kind not in REFERENCE_KINDS:
            raise _invalid(reason, "reference_kind", f"must be one of {sorted(REFERENCE_KINDS)}")
        if self.tracking_quality not in TRACKING_QUALITIES:
            raise _invalid(
                reason, "tracking_quality", f"must be one of {sorted(TRACKING_QUALITIES)}"
            )
        violation = _integer(
            self.tracking_violation_ns, "tracking_violation_ns", 0, INT64_MAX, reason=reason
        )
        try:
            grippers = tuple(self.grippers)
        except TypeError as error_object:
            raise _invalid(reason, "grippers", "must be an array") from error_object
        if len(grippers) != 2:
            raise _invalid(reason, "grippers", "must contain exactly left and right")
        if tuple(item.side for item in grippers) != ("left", "right"):
            raise _invalid(reason, "grippers", "must be ordered left then right")
        for index, item in enumerate(grippers):
            if not isinstance(item, NativeGripper):
                raise _invalid(reason, f"grippers[{index}]", "must be a NativeGripper")

        object.__setattr__(self, "context", context)
        object.__setattr__(self, "command_id", command_id)
        object.__setattr__(self, "command_seq", command_seq)
        object.__setattr__(self, "observation_id", observation_id)
        object.__setattr__(self, "reason", reason_text)
        object.__setattr__(self, "stop_reason", stop_reason)
        object.__setattr__(self, "motion_seq", motion_seq)
        object.__setattr__(self, "stream_id", stream_id)
        object.__setattr__(self, "chunk_seq", chunk_seq)
        object.__setattr__(self, "accepted_sim_time_ns", accepted_sim)
        object.__setattr__(self, "accepted_wall_time_ns", accepted_wall)
        object.__setattr__(self, "started_sim_time_ns", started_sim)
        object.__setattr__(self, "started_wall_time_ns", started_wall)
        object.__setattr__(self, "finished_sim_time_ns", finished_sim)
        object.__setattr__(self, "finished_wall_time_ns", finished_wall)
        object.__setattr__(self, "reference_application_sim_time_ns", application)
        object.__setattr__(self, "measurement_sim_time_ns", measurement)
        object.__setattr__(self, "reference_elapsed_ns", elapsed)
        object.__setattr__(self, "stable_for_ns", stable)
        object.__setattr__(self, "joint_names", names)
        for name, values in vectors.items():
            object.__setattr__(self, name, values)
        object.__setattr__(self, "tracking_violation_ns", violation)
        object.__setattr__(self, "grippers", grippers)


@dataclass(frozen=True, slots=True)
class NativeChunkStatus:
    """One chunk's real phase; RETIRED is never renamed to motion COMPLETED."""

    context: RunContext
    command_id: str
    command_seq: int
    chunk_seq: int
    observation_id: int
    phase: str
    reason: str
    accepted_sim_time_ns: int
    accepted_wall_time_ns: int
    has_started: bool
    started_sim_time_ns: int | None
    started_wall_time_ns: int | None
    has_finished: bool
    finished_sim_time_ns: int | None
    finished_wall_time_ns: int | None

    def __post_init__(self) -> None:
        reason = "INVALID_NATIVE_CHUNK"
        context, command_id, command_seq, observation_id = _command_identity(
            self.context,
            self.command_id,
            self.command_seq,
            self.observation_id,
            reason,
        )
        chunk_seq = _integer(self.chunk_seq, "chunk_seq", 1, UINT64_MAX, reason=reason)
        if self.phase not in CHUNK_PHASES:
            raise _invalid(reason, "phase", f"must be one of {sorted(CHUNK_PHASES)}")
        reason_text = _text(self.reason, "reason", reason, allow_empty=True)
        accepted_sim = _integer(
            self.accepted_sim_time_ns, "accepted_sim_time_ns", 0, INT64_MAX, reason=reason
        )
        accepted_wall = _integer(
            self.accepted_wall_time_ns, "accepted_wall_time_ns", 0, INT64_MAX, reason=reason
        )
        if not isinstance(self.has_started, bool) or not isinstance(self.has_finished, bool):
            raise _invalid(reason, "has_started/has_finished", "must be bool")
        started_sim = _optional_integer(
            self.started_sim_time_ns, "started_sim_time_ns", INT64_MAX, reason=reason
        )
        started_wall = _optional_integer(
            self.started_wall_time_ns, "started_wall_time_ns", INT64_MAX, reason=reason
        )
        finished_sim = _optional_integer(
            self.finished_sim_time_ns, "finished_sim_time_ns", INT64_MAX, reason=reason
        )
        finished_wall = _optional_integer(
            self.finished_wall_time_ns, "finished_wall_time_ns", INT64_MAX, reason=reason
        )
        if self.has_started != (started_sim is not None and started_wall is not None):
            raise _invalid(reason, "has_started", "must match started sim/wall presence")
        if self.has_finished != (finished_sim is not None and finished_wall is not None):
            raise _invalid(reason, "has_finished", "must match finished sim/wall presence")
        if started_sim is not None and started_sim < accepted_sim:
            raise _invalid(reason, "started_sim_time_ns", "must not precede acceptance")
        if finished_sim is not None and finished_sim < accepted_sim:
            raise _invalid(reason, "finished_sim_time_ns", "must not precede acceptance")
        if started_sim is not None and finished_sim is not None and finished_sim < started_sim:
            raise _invalid(reason, "finished_sim_time_ns", "must not precede start")
        if (self.phase in TERMINAL_CHUNK_PHASES) != self.has_finished:
            raise _invalid(
                reason,
                "has_finished",
                "must be true exactly for RETIRED/CANCELED/REJECTED",
            )
        object.__setattr__(self, "context", context)
        object.__setattr__(self, "command_id", command_id)
        object.__setattr__(self, "command_seq", command_seq)
        object.__setattr__(self, "chunk_seq", chunk_seq)
        object.__setattr__(self, "observation_id", observation_id)
        object.__setattr__(self, "reason", reason_text)
        object.__setattr__(self, "accepted_sim_time_ns", accepted_sim)
        object.__setattr__(self, "accepted_wall_time_ns", accepted_wall)
        object.__setattr__(self, "started_sim_time_ns", started_sim)
        object.__setattr__(self, "started_wall_time_ns", started_wall)
        object.__setattr__(self, "finished_sim_time_ns", finished_sim)
        object.__setattr__(self, "finished_wall_time_ns", finished_wall)


@dataclass(frozen=True, slots=True)
class NativeControlEvent:
    """One controller event with original identity and time, no new values."""

    context: RunContext
    event_seq: int
    command_id: str
    command_seq: int
    observation_id: int
    is_chunk: bool
    chunk_seq: int | None
    phase: str
    reason: str
    sim_time_ns: int

    def __post_init__(self) -> None:
        reason = "INVALID_NATIVE_EVENT"
        context, command_id, command_seq, observation_id = _command_identity(
            self.context,
            self.command_id,
            self.command_seq,
            self.observation_id,
            reason,
        )
        event_seq = _integer(self.event_seq, "event_seq", 1, UINT64_MAX, reason=reason)
        if not isinstance(self.is_chunk, bool):
            raise _invalid(reason, "is_chunk", "must be bool")
        chunk_seq = _optional_integer(self.chunk_seq, "chunk_seq", UINT64_MAX, reason=reason)
        if chunk_seq == 0:
            raise _invalid(reason, "chunk_seq", "must be positive when present")
        if self.is_chunk != (chunk_seq is not None):
            raise _invalid(reason, "is_chunk", "must match chunk_seq presence")
        if self.phase not in EVENT_PHASES:
            raise _invalid(reason, "phase", f"must be one of {sorted(EVENT_PHASES)}")
        reason_text = _text(self.reason, "reason", reason, allow_empty=True)
        sim_time = _integer(self.sim_time_ns, "sim_time_ns", 0, INT64_MAX, reason=reason)
        object.__setattr__(self, "context", context)
        object.__setattr__(self, "event_seq", event_seq)
        object.__setattr__(self, "command_id", command_id)
        object.__setattr__(self, "command_seq", command_seq)
        object.__setattr__(self, "observation_id", observation_id)
        object.__setattr__(self, "chunk_seq", chunk_seq)
        object.__setattr__(self, "reason", reason_text)
        object.__setattr__(self, "sim_time_ns", sim_time)


__all__ = [
    "CHUNK_PHASES",
    "COMMAND_PHASES",
    "EVENT_PHASES",
    "GRIPPER_MODES",
    "MAX_FINGER_OBJECTS",
    "MOTION_PHASES",
    "REFERENCE_KINDS",
    "TERMINAL_CHUNK_PHASES",
    "TERMINAL_COMMAND_PHASES",
    "TRACKING_QUALITIES",
    "NativeChunkStatus",
    "NativeControlEvent",
    "NativeExecution",
    "NativeFinger",
    "NativeGripper",
]
