"""Immutable execution feedback and contact evidence shared by adapters."""

from __future__ import annotations

import math
from typing import Annotated, ClassVar, Literal, Self

from pydantic import BeforeValidator, Field, model_validator

from .control import INT64_MAX, MAX_ID_LENGTH, UINT64_MAX
from .models import FloatVector, RunContext
from .validation import (
    NonEmptyStr,
    NonNegativeInt,
    PositiveInt,
    StrictContractModel,
    StrictFloat,
    as_tuple,
    contract_error,
)

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

OptionalInt64 = Annotated[int, Field(strict=True, ge=0, le=INT64_MAX)] | None
Names = Annotated[tuple[NonEmptyStr, ...], BeforeValidator(as_tuple),
                  Field(min_length=1, max_length=64)]


def _same_length(name: str, values: tuple[float, ...], expected: int) -> None:
    if len(values) != expected:
        raise contract_error(name, f"has {len(values)} values for {expected} joints")


class FingerFeedback(StrictContractModel):
    """One finger's summed normal forces/counts for each allowed object."""

    _error_reason: ClassVar[str] = "INVALID_GRIPPER_FEEDBACK"

    side: Literal["left", "right"]
    finger_index: Annotated[int, Field(strict=True, ge=1, le=2)]
    body_name: Annotated[NonEmptyStr, Field(max_length=MAX_ID_LENGTH)]
    joint_speed_m_s: Annotated[StrictFloat, Field(ge=0.0)]
    object_names: Annotated[tuple[NonEmptyStr, ...], BeforeValidator(as_tuple),
                            Field(max_length=MAX_FINGER_OBJECTS)]
    normal_forces: Annotated[tuple[float, ...], BeforeValidator(as_tuple),
                             Field(max_length=MAX_FINGER_OBJECTS)]
    contact_counts: Annotated[tuple[int, ...], BeforeValidator(as_tuple),
                              Field(max_length=MAX_FINGER_OBJECTS)]

    @model_validator(mode="after")
    def _evidence(self) -> Self:
        if len(set(self.object_names)) != len(self.object_names):
            raise contract_error("object_names", "must be unique")
        for name in ("normal_forces", "contact_counts"):
            if len(getattr(self, name)) != len(self.object_names):
                raise contract_error(name, "must match object_names")
        if any(isinstance(value, bool) or not math.isfinite(value)
               for value in self.normal_forces):
            raise contract_error("normal_forces", "must be finite real numbers")
        if any(value < 0.0 for value in self.normal_forces):
            raise contract_error("normal_forces", "must not be negative")
        if any(isinstance(value, bool) or not isinstance(value, int)
               for value in self.contact_counts):
            raise contract_error("contact_counts", "must be integers")
        if any(value < 0 for value in self.contact_counts):
            raise contract_error("contact_counts", "must not be negative")
        return self


class GripperFeedback(StrictContractModel):
    """One gripper's original target, measured state and per-finger evidence."""

    _error_reason: ClassVar[str] = "INVALID_GRIPPER_FEEDBACK"

    side: Literal["left", "right"]
    mode: Literal["NONE", "POSITION", "CONTACT"]
    position_ok: bool
    contact_ok: bool
    contact_object: Annotated[NonEmptyStr, Field(max_length=MAX_ID_LENGTH)] | None
    ambiguous: bool
    target_position_m: StrictFloat
    initial_reference_m: StrictFloat
    final_reference_m: StrictFloat
    measured_position_m: StrictFloat
    measured_velocity_m_s: StrictFloat
    target_error_m: StrictFloat
    fingers: Annotated[tuple[FingerFeedback, ...], BeforeValidator(as_tuple),
                       Field(min_length=MAX_GRIPPER_FINGERS,
                             max_length=MAX_GRIPPER_FINGERS)]

    @model_validator(mode="after")
    def _evidence(self) -> Self:
        if self.mode == "POSITION" and not self.position_ok:
            raise contract_error("mode", "POSITION requires position_ok")
        if self.mode == "CONTACT":
            if not self.contact_ok:
                raise contract_error("mode", "CONTACT requires contact_ok")
            if self.contact_object is None:
                raise contract_error("mode", "CONTACT requires a contact object")
        if self.mode == "NONE" and (self.position_ok or self.contact_ok):
            raise contract_error("mode", "NONE cannot carry a successful flag")
        if self.contact_ok and self.contact_object is None:
            raise contract_error("contact_ok", "requires a contact object")
        if self.contact_ok and self.ambiguous:
            raise contract_error("contact_ok", "cannot be true when ambiguous")
        if not math.isclose(self.target_error_m,
                            self.target_position_m - self.measured_position_m,
                            rel_tol=1e-9, abs_tol=1e-12):
            raise contract_error("target_error_m", "must equal target - measured")
        if any(finger.side != self.side for finger in self.fingers):
            raise contract_error("fingers", "must match the gripper side")
        if tuple(finger.finger_index for finger in self.fingers) != (1, 2):
            raise contract_error("fingers", "must be ordered finger_index 1 then 2")
        return self


class ExecutionFeedback(StrictContractModel):
    """Frozen overall execution feedback derived from a ControlStatus snapshot."""

    _error_reason: ClassVar[str] = "INVALID_EXECUTION_FEEDBACK"

    context: RunContext
    command_id: Annotated[NonEmptyStr, Field(max_length=MAX_ID_LENGTH)]
    command_seq: Annotated[PositiveInt, Field(le=UINT64_MAX)]
    observation_id: Annotated[PositiveInt, Field(le=UINT64_MAX)]
    phase: str
    reason: str
    stop_reason: str
    motion_seq: Annotated[int, Field(strict=True, ge=0, le=UINT64_MAX)] | None
    stream_id: Annotated[str, Field(max_length=MAX_ID_LENGTH)]
    chunk_seq: Annotated[NonNegativeInt, Field(le=UINT64_MAX)]
    accepted_sim_time_ns: Annotated[NonNegativeInt, Field(le=INT64_MAX)]
    accepted_wall_time_ns: Annotated[NonNegativeInt, Field(le=INT64_MAX)]
    has_started: bool
    started_sim_time_ns: OptionalInt64
    started_wall_time_ns: OptionalInt64
    has_finished: bool
    finished_sim_time_ns: OptionalInt64
    finished_wall_time_ns: OptionalInt64
    motion_phase: str
    reference_application_sim_time_ns: Annotated[NonNegativeInt, Field(le=INT64_MAX)]
    measurement_sim_time_ns: Annotated[NonNegativeInt, Field(le=INT64_MAX)]
    reference_elapsed_ns: Annotated[NonNegativeInt, Field(le=INT64_MAX)]
    stable_for_ns: Annotated[NonNegativeInt, Field(le=INT64_MAX)]
    joint_names: Names
    reference_positions: FloatVector
    reference_velocities: FloatVector
    reference_accelerations: FloatVector
    measured_positions: FloatVector
    measured_velocities: FloatVector
    position_errors: FloatVector
    velocity_errors: FloatVector
    reference_kind: str
    tracking_quality: str
    tracking_violation_ns: Annotated[NonNegativeInt, Field(le=INT64_MAX)]
    grippers: Annotated[tuple[GripperFeedback, ...], BeforeValidator(as_tuple),
                        Field(min_length=2, max_length=2)]

    @model_validator(mode="after")
    def _execution(self) -> Self:
        if self.phase not in COMMAND_PHASES:
            raise contract_error("phase", "unknown command phase")
        if self.motion_seq == 0:
            raise contract_error("motion_seq", "zero is not a motion sequence")
        if self.has_started != (self.started_sim_time_ns is not None
                                and self.started_wall_time_ns is not None):
            raise contract_error("has_started", "must match started sim/wall presence")
        if self.has_finished != (self.finished_sim_time_ns is not None
                                 and self.finished_wall_time_ns is not None):
            raise contract_error("has_finished", "must match finished sim/wall presence")
        if (self.started_sim_time_ns is not None
                and self.started_sim_time_ns < self.accepted_sim_time_ns):
            raise contract_error("started_sim_time_ns", "must not precede acceptance")
        if (self.finished_sim_time_ns is not None
                and self.finished_sim_time_ns < self.accepted_sim_time_ns):
            raise contract_error("finished_sim_time_ns", "must not precede acceptance")
        if (self.started_sim_time_ns is not None and self.finished_sim_time_ns is not None
                and self.finished_sim_time_ns < self.started_sim_time_ns):
            raise contract_error("finished_sim_time_ns", "must not precede start")
        if (self.phase in TERMINAL_COMMAND_PHASES) != self.has_finished:
            raise contract_error("has_finished", "must match terminal command phases")
        if self.motion_phase not in MOTION_PHASES:
            raise contract_error("motion_phase", "unknown motion phase")
        if (self.motion_phase in ("COMPLETED", "STOPPED", "FAILED")) != self.has_finished:
            raise contract_error("motion_phase", "terminal phases require has_finished")
        if self.measurement_sim_time_ns < self.reference_application_sim_time_ns:
            raise contract_error("measurement_sim_time_ns", "must not precede application")
        count = len(self.joint_names)
        if len(set(self.joint_names)) != count:
            raise contract_error("joint_names", "must be unique")
        vectors = {name: getattr(self, name) for name in (
            "reference_positions", "reference_velocities", "reference_accelerations",
            "measured_positions", "measured_velocities", "position_errors",
            "velocity_errors")}
        for name, values in vectors.items():
            _same_length(name, tuple(values), count)
        for reference, measured, error in zip(
            vectors["reference_positions"], vectors["measured_positions"],
            vectors["position_errors"], strict=True,
        ):
            if not math.isclose(reference - measured, error, rel_tol=1e-9, abs_tol=1e-12):
                raise contract_error("position_errors", "must equal reference - measured")
        for reference, measured, error in zip(
            vectors["reference_velocities"], vectors["measured_velocities"],
            vectors["velocity_errors"], strict=True,
        ):
            if not math.isclose(reference - measured, error, rel_tol=1e-9, abs_tol=1e-12):
                raise contract_error("velocity_errors", "must equal reference - measured")
        if self.reference_kind not in REFERENCE_KINDS:
            raise contract_error("reference_kind", "unknown reference kind")
        if self.tracking_quality not in TRACKING_QUALITIES:
            raise contract_error("tracking_quality", "unknown tracking quality")
        if tuple(item.side for item in self.grippers) != ("left", "right"):
            raise contract_error("grippers", "must be ordered left then right")
        return self


class StreamChunkStatus(StrictContractModel):
    """One chunk's real phase; RETIRED is never renamed to motion COMPLETED."""

    _error_reason: ClassVar[str] = "INVALID_STREAM_CHUNK"

    context: RunContext
    command_id: Annotated[NonEmptyStr, Field(max_length=MAX_ID_LENGTH)]
    command_seq: Annotated[PositiveInt, Field(le=UINT64_MAX)]
    chunk_seq: Annotated[PositiveInt, Field(le=UINT64_MAX)]
    observation_id: Annotated[PositiveInt, Field(le=UINT64_MAX)]
    phase: str
    reason: str
    accepted_sim_time_ns: Annotated[NonNegativeInt, Field(le=INT64_MAX)]
    accepted_wall_time_ns: Annotated[NonNegativeInt, Field(le=INT64_MAX)]
    has_started: bool
    started_sim_time_ns: OptionalInt64
    started_wall_time_ns: OptionalInt64
    has_finished: bool
    finished_sim_time_ns: OptionalInt64
    finished_wall_time_ns: OptionalInt64

    @model_validator(mode="after")
    def _chunk(self) -> Self:
        if self.phase not in CHUNK_PHASES:
            raise contract_error("phase", "unknown chunk phase")
        if self.has_started != (self.started_sim_time_ns is not None
                                and self.started_wall_time_ns is not None):
            raise contract_error("has_started", "must match started presence")
        if self.has_finished != (self.finished_sim_time_ns is not None
                                 and self.finished_wall_time_ns is not None):
            raise contract_error("has_finished", "must match finished presence")
        if (self.started_sim_time_ns is not None
                and self.started_sim_time_ns < self.accepted_sim_time_ns):
            raise contract_error("started_sim_time_ns", "must not precede acceptance")
        if (self.finished_sim_time_ns is not None
                and self.finished_sim_time_ns < self.accepted_sim_time_ns):
            raise contract_error("finished_sim_time_ns", "must not precede acceptance")
        if (self.started_sim_time_ns is not None and self.finished_sim_time_ns is not None
                and self.finished_sim_time_ns < self.started_sim_time_ns):
            raise contract_error("finished_sim_time_ns", "must not precede start")
        if (self.phase in TERMINAL_CHUNK_PHASES) != self.has_finished:
            raise contract_error("has_finished", "must match terminal chunk phases")
        return self


class ControlEvent(StrictContractModel):
    """One controller event with original identity and time, no new values."""

    _error_reason: ClassVar[str] = "INVALID_CONTROL_EVENT"

    context: RunContext
    event_seq: Annotated[PositiveInt, Field(le=UINT64_MAX)]
    command_id: Annotated[NonEmptyStr, Field(max_length=MAX_ID_LENGTH)]
    command_seq: Annotated[PositiveInt, Field(le=UINT64_MAX)]
    observation_id: Annotated[PositiveInt, Field(le=UINT64_MAX)]
    is_chunk: bool
    chunk_seq: Annotated[PositiveInt, Field(le=UINT64_MAX)] | None
    phase: str
    reason: str
    sim_time_ns: Annotated[NonNegativeInt, Field(le=INT64_MAX)]

    @model_validator(mode="after")
    def _event(self) -> Self:
        if self.is_chunk != (self.chunk_seq is not None):
            raise contract_error("is_chunk", "must match chunk_seq presence")
        if self.phase not in EVENT_PHASES:
            raise contract_error("phase", "unknown event phase")
        return self


__all__ = [
    "CHUNK_PHASES", "COMMAND_PHASES", "EVENT_PHASES", "GRIPPER_MODES", "MAX_FINGER_OBJECTS",
    "MOTION_PHASES", "REFERENCE_KINDS", "TERMINAL_CHUNK_PHASES", "TERMINAL_COMMAND_PHASES",
    "TRACKING_QUALITIES", "ControlEvent", "ExecutionFeedback", "FingerFeedback",
    "GripperFeedback", "StreamChunkStatus",
]
