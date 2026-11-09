"""Native request DTOs, accepted-reference records, validation and sampling."""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Sequence
from typing import Annotated, ClassVar, Self

from pydantic import BeforeValidator, Field, model_validator

from ..action_chunks import (
    INT64_MAX,
    MAX_TARGET_COUNT,
    UINT64_MAX,
    ActionWindow,
    ChunkAnchor,
)
from ..control import MAX_ID_LENGTH, ControlCommand
from ..models import ContractError, FloatVector, JointSpec, RobotProfile, RunContext, Target
from ..trajectory.spline import (
    MAX_HORIZON_NS,
    MAX_JOINTS,
    MAX_SEGMENTS,
    VELOCITY_ATOL,
    CubicSegment,
    ReferencePlan,
    ReferenceSample,
    validate_reference_inputs,
    verify_reference_plan,
)
from ..validation import (
    NonEmptyStr,
    PositiveInt,
    StrictContractModel,
    as_tuple,
    contract_error,
)

NATIVE_MOTION = 1
NATIVE_STREAM_FIRST = 2
NATIVE_STREAM_NEXT = 3
NATIVE_KINDS = (NATIVE_MOTION, NATIVE_STREAM_FIRST, NATIVE_STREAM_NEXT)
NATIVE_KIND_NAMES = {
    NATIVE_MOTION: "MOTION",
    NATIVE_STREAM_FIRST: "STREAM_FIRST",
    NATIVE_STREAM_NEXT: "STREAM_NEXT",
}
INTERPOLATION_VERSION = "explicit_cubic_v1"
REFERENCE_DIGEST_VERSION = "native-reference-v1"
DIGEST_HEX_LENGTH = 64
_SHA256_PATTERN = r"^(?:[0-9a-f]{64})?$"


def _invalid(location: str, message: str) -> ContractError:
    return ContractError("INVALID_NATIVE_REQUEST", f"{location}: {message}")


def _record_invalid(location: str, message: str) -> ContractError:
    return ContractError("INVALID_REFERENCE_RECORD", f"{location}: {message}")


class Trajectory(StrictContractModel):
    """One frozen command trajectory: identity, joint order and timed targets."""

    _error_reason: ClassVar[str] = "INVALID_NATIVE_REQUEST"

    context: RunContext
    lease_id: Annotated[NonEmptyStr, Field(max_length=MAX_ID_LENGTH)]
    command_id: Annotated[NonEmptyStr, Field(max_length=MAX_ID_LENGTH)]
    command_seq: Annotated[PositiveInt, Field(le=UINT64_MAX)]
    observation_id: Annotated[PositiveInt, Field(le=UINT64_MAX)]
    accept_before_ns: Annotated[int, Field(strict=True, ge=0, le=INT64_MAX)]
    joint_names: Annotated[tuple[NonEmptyStr, ...], BeforeValidator(as_tuple),
                           Field(min_length=1, max_length=MAX_JOINTS)]
    targets: Annotated[tuple[Target, ...], BeforeValidator(as_tuple),
                       Field(min_length=1, max_length=MAX_SEGMENTS)]

    @model_validator(mode="after")
    def _trajectory(self) -> Self:
        if len(set(self.joint_names)) != len(self.joint_names):
            raise contract_error("joint_names", "must be unique")
        previous = 0
        for index, target in enumerate(self.targets):
            if target.time_ns <= previous or target.time_ns > MAX_HORIZON_NS:
                raise contract_error(f"targets[{index}].time_ns",
                                     "must be strictly increasing inside the horizon")
            if len(target.positions) != len(self.joint_names):
                raise contract_error(f"targets[{index}].positions",
                                     "must match the joint dimension")
            previous = target.time_ns
        return self


class NativeRequest(StrictContractModel):
    """One frozen native command request in exactly one of the three modes."""

    _error_reason: ClassVar[str] = "INVALID_NATIVE_REQUEST"

    trajectory: Trajectory
    kind: int
    anchor: ChunkAnchor | None = None
    observation_time_ns: Annotated[int, Field(strict=True, ge=0, le=INT64_MAX)] = 0
    target_count: Annotated[int, Field(strict=True, ge=0, le=MAX_TARGET_COUNT)] = 0
    settle_at_end: bool = True
    final_velocities: FloatVector | None = None

    @model_validator(mode="after")
    def _mode(self) -> Self:
        if self.kind not in NATIVE_KINDS:
            raise contract_error("kind", f"must be one of {NATIVE_KINDS}")
        joint_count = len(self.trajectory.joint_names)
        if self.final_velocities is not None:
            if len(self.final_velocities) != joint_count:
                raise contract_error("final_velocities", "must match the joint dimension")
            if self.settle_at_end and any(
                abs(velocity) > VELOCITY_ATOL for velocity in self.final_velocities
            ):
                raise contract_error("final_velocities",
                                     "must be zero when settle_at_end is true")
        if self.kind == NATIVE_MOTION:
            if self.anchor is not None:
                raise contract_error("anchor", "MOTION must not carry a ChunkAnchor")
            if self.observation_time_ns != 0:
                raise contract_error("observation_time_ns", "MOTION must use zero")
            if self.target_count != 0:
                raise contract_error("target_count", "MOTION must use zero")
            if self.settle_at_end is not True:
                raise contract_error("settle_at_end", "MOTION must settle at the end")
            if self.final_velocities is not None:
                raise contract_error("final_velocities", "MOTION uses zero boundaries")
            return self
        if self.anchor is None:
            raise contract_error("anchor", "stream modes require a ChunkAnchor")
        if self.anchor.robot_profile_hash != self.trajectory.context.robot_profile_hash:
            raise contract_error("anchor.robot_profile_hash",
                                 "must equal the trajectory profile hash")
        if len(self.anchor.reference.positions) != joint_count:
            raise contract_error("anchor.reference.positions",
                                 "must match the joint dimension")
        if self.kind == NATIVE_STREAM_FIRST and self.anchor.predecessor_seq != 0:
            raise contract_error("anchor.predecessor_seq", "STREAM_FIRST requires zero")
        if self.kind == NATIVE_STREAM_NEXT and self.anchor.predecessor_seq <= 0:
            raise contract_error("anchor.predecessor_seq", "STREAM_NEXT requires positive")
        if not 2 <= self.target_count <= MAX_TARGET_COUNT:
            raise contract_error("target_count", "must be in 2..MAX_TARGET_COUNT")
        if self.target_count > len(self.trajectory.targets):
            raise contract_error("target_count", "must not exceed the source target count")
        if self.observation_time_ns > self.anchor.start_sim_time_ns:
            raise contract_error("observation_time_ns", "must not follow the hand-off")
        if self.observation_time_ns + self.trajectory.targets[-1].time_ns > INT64_MAX:
            raise contract_error("observation_time_ns", "plus target time exceeds int64")
        return self

    @property
    def command(self) -> ControlCommand:
        return ControlCommand(
            self.trajectory.context,
            self.trajectory.lease_id,
            self.trajectory.command_id,
            self.trajectory.command_seq,
            self.trajectory.observation_id,
            self.trajectory.accept_before_ns,
        )

    @property
    def mode(self) -> str:
        return NATIVE_KIND_NAMES[self.kind]

    @property
    def window(self) -> ActionWindow | None:
        """Return the recomputed stream window; MOTION has none."""
        if self.kind == NATIVE_MOTION:
            return None
        assert self.anchor is not None
        return ActionWindow(
            self.trajectory.observation_id,
            self.observation_time_ns,
            self.anchor.start_sim_time_ns,
            self.trajectory.targets,
            self.target_count,
        )


def reference_digest_payload(record: AcceptedReference) -> dict[str, object]:
    """Return the versioned canonical document hashed by the record digest."""
    document = record.model_dump(mode="json", exclude={"reference_digest"})
    document["digest_version"] = REFERENCE_DIGEST_VERSION
    return document


def compute_reference_digest(record: AcceptedReference) -> str:
    """SHA-256 of stable canonical JSON over every public reference field."""
    encoded = json.dumps(
        reference_digest_payload(record),
        sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False,
    ).encode("ascii")
    return hashlib.sha256(encoded).hexdigest()


class AcceptedReference(StrictContractModel):
    """One independently auditable accepted reference record."""

    _error_reason: ClassVar[str] = "INVALID_REFERENCE_RECORD"

    request: NativeRequest
    motion_seq: Annotated[PositiveInt, Field(le=UINT64_MAX)]
    accepted_sim_time_ns: Annotated[int, Field(strict=True, ge=0, le=INT64_MAX)]
    joint_names: Annotated[tuple[NonEmptyStr, ...], BeforeValidator(as_tuple),
                           Field(min_length=1, max_length=MAX_JOINTS)]
    plan: ReferencePlan
    initial_positions: FloatVector
    initial_velocities: FloatVector
    final_velocities: FloatVector
    max_accelerations: FloatVector
    selected_indices: Annotated[tuple[int, ...], BeforeValidator(as_tuple),
                                Field(max_length=MAX_SEGMENTS)] = ()
    interpolation_version: str = INTERPOLATION_VERSION
    reference_digest: Annotated[str, Field(pattern=_SHA256_PATTERN)] | None = None

    @model_validator(mode="after")
    def _record(self) -> Self:
        if len(set(self.joint_names)) != len(self.joint_names):
            raise contract_error("joint_names", "must be unique")
        if self.plan.joint_names != self.joint_names:
            raise contract_error("plan.joint_names", "must match record joint_names")
        for name in ("initial_positions", "initial_velocities", "final_velocities",
                     "max_accelerations"):
            if len(getattr(self, name)) != len(self.joint_names):
                raise contract_error(name, "must match the joint dimension")
        if any(value < 0.0 for value in self.max_accelerations):
            raise contract_error("max_accelerations", "must not be negative")
        previous = -1
        for index, value in enumerate(self.selected_indices):
            if isinstance(value, bool) or not isinstance(value, int) or value <= previous:
                raise contract_error(f"selected_indices[{index}]",
                                     "must be strictly increasing integers")
            previous = value
        if self.interpolation_version != INTERPOLATION_VERSION:
            raise contract_error("interpolation_version",
                                 f"must be {INTERPOLATION_VERSION!r}")
        expected = compute_reference_digest(self)
        if self.reference_digest is None:
            object.__setattr__(self, "reference_digest", expected)
        elif self.reference_digest != expected:
            raise ContractError(
                "REFERENCE_DIGEST_MISMATCH",
                "reference_digest does not cover the retained request/plan/times",
            )
        return self


def _backward_final_velocities(window: ActionWindow) -> tuple[float, ...]:
    last, previous = window.targets[-1], window.targets[-2]
    delta_ns = last.time_ns - previous.time_ns
    if delta_ns <= 0:
        raise ContractError("INVALID_REFERENCE_RECORD",
                            "selected times are not strictly increasing")
    delta_s = delta_ns / 1_000_000_000
    return tuple((right - left) / delta_s
                 for left, right in zip(previous.positions, last.positions, strict=True))


def expected_final_velocities(request: NativeRequest) -> tuple[float, ...]:
    """Derive the stream final velocity from the last two source targets."""
    if request.kind == NATIVE_MOTION:
        return (0.0,) * len(request.trajectory.joint_names)
    assert request.anchor is not None
    if request.settle_at_end:
        return (0.0,) * len(request.trajectory.joint_names)
    if request.final_velocities is not None:
        return request.final_velocities
    window = request.window
    assert window is not None
    return _backward_final_velocities(window)


def plan_from_segments(
    joint_names: Sequence[str],
    segments: Sequence[CubicSegment],
) -> ReferencePlan:
    """Explicit helper used by transports to rebuild the public plan."""
    return ReferencePlan(tuple(joint_names), tuple(segments))

def _vector_close(left: Sequence[float], right: Sequence[float], *, atol: float) -> bool:
    return len(left) == len(right) and all(
        math.isclose(a, b, rel_tol=0.0, abs_tol=atol)
        for a, b in zip(left, right, strict=True)
    )


def validate_accepted_reference(
    record: AcceptedReference,
    profile: RobotProfile,
    max_accelerations: Sequence[float],
) -> None:
    """Independently verify one accepted reference against the profile."""
    if not isinstance(record, AcceptedReference):
        raise _record_invalid("record", "must be an AcceptedReference")
    if not isinstance(profile, RobotProfile):
        raise ContractError("PROFILE_MISMATCH", "profile must be a RobotProfile")
    if tuple(record.joint_names) != tuple(profile.joint_names):
        raise ContractError("PROFILE_MISMATCH", "record joint names/order differ from profile")
    if record.request.trajectory.joint_names != record.joint_names:
        raise _record_invalid("request.trajectory.joint_names", "must match record")
    if record.request.trajectory.context.robot_profile_hash != profile.digest:
        raise ContractError("PROFILE_MISMATCH", "request profile digest differs")
    limits = tuple(float(value) for value in max_accelerations)
    if len(limits) != len(record.joint_names):
        raise _record_invalid("max_accelerations", "must match the record joint dimension")
    if not _vector_close(limits, record.max_accelerations, atol=0.0):
        raise _record_invalid("max_accelerations", "differs from the retained limits")
    if record.reference_digest != compute_reference_digest(record):
        raise ContractError("REFERENCE_DIGEST_MISMATCH",
                            "accepted reference fields were modified")
    request = record.request
    joints: tuple[JointSpec, ...] = tuple(profile.joints)
    if request.kind == NATIVE_MOTION:
        if record.selected_indices:
            raise _record_invalid("selected_indices", "MOTION must not carry a selection")
        if any(abs(value) > VELOCITY_ATOL for value in record.initial_velocities):
            raise _record_invalid("initial_velocities", "MOTION must start at zero")
        if any(abs(value) > VELOCITY_ATOL for value in record.final_velocities):
            raise _record_invalid("final_velocities", "MOTION must end at zero")
        zero = (0.0,) * len(record.joint_names)
        validate_reference_inputs(record.initial_positions, request.trajectory.targets,
                                  joints, limits, initial_velocities=zero,
                                  final_velocities=zero)
        verify_reference_plan(record.plan, record.initial_positions,
                              request.trajectory.targets, joints, limits,
                              initial_velocities=zero, final_velocities=zero)
        return
    assert request.anchor is not None
    window = request.window
    assert window is not None
    selected = tuple(record.selected_indices)
    if selected != tuple(window.selected_indices):
        raise ContractError("REFERENCE_SELECTION_MISMATCH",
                            "selected_indices differ from the window recomputation")
    if len(selected) != request.target_count:
        raise _record_invalid("selected_indices", "must contain exactly target_count entries")
    anchor = request.anchor
    if not _vector_close(record.initial_positions, anchor.reference.positions, atol=0.0):
        raise _record_invalid("initial_positions", "must equal the anchor positions")
    if not _vector_close(record.initial_velocities, anchor.reference.velocities, atol=0.0):
        raise _record_invalid("initial_velocities", "must equal the anchor velocities")
    if not _vector_close(record.final_velocities, expected_final_velocities(request),
                         atol=VELOCITY_ATOL):
        raise _record_invalid("final_velocities", "differ from the declared mode")
    validate_reference_inputs(record.initial_positions, request.trajectory.targets,
                              joints, limits,
                              initial_velocities=record.initial_velocities,
                              final_velocities=record.final_velocities)
    verify_reference_plan(record.plan, record.initial_positions, window.targets,
                          joints, limits,
                          initial_velocities=record.initial_velocities,
                          final_velocities=record.final_velocities)


def sample_accepted_reference(
    record: AcceptedReference,
    sim_time_ns: int,
    *,
    actual_started_sim_ns: int | None = None,
) -> ReferenceSample:
    """Rebuild the reference at an absolute application time."""
    if not isinstance(record, AcceptedReference):
        raise _record_invalid("record", "must be an AcceptedReference")
    if isinstance(sim_time_ns, bool) or not isinstance(sim_time_ns, int):
        raise _record_invalid("sim_time_ns", "must be an integer")
    if sim_time_ns < 0 or sim_time_ns > INT64_MAX:
        raise _record_invalid("sim_time_ns", "must be a non-negative int64")
    request = record.request
    if request.kind == NATIVE_MOTION:
        if actual_started_sim_ns is None:
            raise _record_invalid("actual_started_sim_ns",
                                  "MOTION reconstruction requires the started time")
        if isinstance(actual_started_sim_ns, bool) or not isinstance(
            actual_started_sim_ns, int
        ) or not 0 <= actual_started_sim_ns <= INT64_MAX:
            raise _record_invalid("actual_started_sim_ns", "must be a non-negative int64")
        start, final_hold = actual_started_sim_ns, True
    else:
        if actual_started_sim_ns is not None:
            raise _record_invalid("actual_started_sim_ns",
                                  "STREAM reconstruction takes the anchor start")
        assert request.anchor is not None
        start, final_hold = request.anchor.start_sim_time_ns, request.settle_at_end
    elapsed = sim_time_ns - start
    if elapsed < 0:
        raise ContractError("REFERENCE_EXPIRED",
                            f"sim_time_ns {sim_time_ns} is before the plan origin {start}")
    if elapsed <= record.plan.duration_ns:
        return record.plan.sample_interval(elapsed)
    if final_hold:
        terminal = record.plan.sample_interval(record.plan.duration_ns)
        if any(abs(value) > VELOCITY_ATOL
               for value in (*record.final_velocities, *terminal.velocities)):
            raise ContractError("REFERENCE_EXPIRED",
                                "a declared final hold requires a zero-velocity endpoint")
        zero = (0.0,) * len(terminal.positions)
        return ReferenceSample(terminal.positions, zero, zero)
    raise ContractError("REFERENCE_EXPIRED",
                        f"elapsed_ns {elapsed} is outside [0, {record.plan.duration_ns}]")
