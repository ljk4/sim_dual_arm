"""Accepted bounded-follower reference records and their canonical digest.

The curve representation, sampling and hard-boundary normalization are shared
conventions; a record is accepted from any producer that supplies a valid
:class:`ReferencePlan`.  ``algorithm`` keeps the producer's real
identity for readers and logs, but is not compared with any shared constant.
"""

from __future__ import annotations

from typing import Annotated, ClassVar, Self

from pydantic import BeforeValidator, Field, model_validator

from robot_contracts.trajectory.constants import MAX_HORIZON_NS, MAX_JOINTS
from robot_contracts.trajectory.follower import FollowerLimitFlags
from robot_contracts.trajectory.spline import ReferencePlan

from ..action_chunks import MAX_TARGET_COUNT, ActionWindow, ChunkAnchor
from ..models import ContractError, FloatVector
from ..stream_request import (
    REQUEST_STREAM_FIRST,
    REQUEST_STREAM_NEXT,
    StreamRequest,
)
from ..validation import (
    NonEmptyStr,
    PositiveInt,
    StrictContractModel,
    as_tuple,
    contract_error,
)

FOLLOWER_REFERENCE_KINDS = (REQUEST_STREAM_FIRST, REQUEST_STREAM_NEXT)
_UINT64_MAX = 2**64 - 1
_INT64_MAX = 2**63 - 1
_DEVIATION_ATOL = 1e-12


def _invalid(location: str, message: str) -> ContractError:
    return ContractError("INVALID_FOLLOWER_REFERENCE", f"{location}: {message}")


class FollowerAppliedSample(StrictContractModel):
    """One raw coefficient sample and its auditable hard-boundary correction."""

    _error_reason: ClassVar[str] = "INVALID_FOLLOWER_REFERENCE"

    sampled_positions: FloatVector
    applied_positions: FloatVector
    corrections: FloatVector
    sampled_velocities: FloatVector
    sampled_accelerations: FloatVector

    @property
    def has_correction(self) -> bool:
        return any(value != 0.0 for value in self.corrections)


class FollowerReference(StrictContractModel):
    """One accepted bounded-follower block with explicit, protected evidence."""

    _error_reason: ClassVar[str] = "INVALID_FOLLOWER_REFERENCE"

    request: StreamRequest
    motion_seq: Annotated[PositiveInt, Field(le=_UINT64_MAX)]
    accepted_backend_time_ns: Annotated[int, Field(strict=True, ge=0, le=_INT64_MAX)]
    accepted_wall_time_ns: Annotated[int, Field(strict=True, ge=0, le=_INT64_MAX)]
    joint_names: Annotated[tuple[NonEmptyStr, ...], BeforeValidator(as_tuple),
                           Field(min_length=1, max_length=MAX_JOINTS)]
    plan: ReferencePlan
    anchor: ChunkAnchor
    window: ActionWindow
    max_accelerations: FloatVector
    max_target_deviation_limits: FloatVector
    raw_terminal_velocities: FloatVector
    restricted_terminal_velocities: FloatVector
    actual_terminal_positions: FloatVector
    actual_terminal_velocities: FloatVector
    actual_terminal_accelerations: FloatVector
    max_target_deviations: FloatVector
    is_final: bool
    requested_window_ns: Annotated[int, Field(strict=True, ge=1, le=MAX_HORIZON_NS)]
    curve_duration_ns: Annotated[int, Field(strict=True, ge=1, le=MAX_HORIZON_NS)]
    duration_grid_ns: Annotated[int, Field(strict=True, ge=1, le=MAX_HORIZON_NS)] | None
    velocity_limited: Annotated[tuple[bool, ...], BeforeValidator(as_tuple),
                                Field(min_length=1, max_length=MAX_JOINTS)]
    acceleration_limited: Annotated[tuple[bool, ...], BeforeValidator(as_tuple),
                                    Field(min_length=1, max_length=MAX_JOINTS)]
    boundary_preserved: Annotated[tuple[bool, ...], BeforeValidator(as_tuple),
                                  Field(min_length=1, max_length=MAX_JOINTS)]
    algorithm: NonEmptyStr

    @model_validator(mode="after")
    def _record(self) -> Self:
        request = self.request
        if request.kind not in FOLLOWER_REFERENCE_KINDS:
            raise contract_error("request.kind", "requires STREAM_FIRST or STREAM_NEXT")
        if request.anchor is None or request.window is None:
            raise contract_error("request", "requires an anchor and window")
        joint_count = len(self.joint_names)
        if self.plan.joint_names != self.joint_names:
            raise contract_error("joint_names", "must match the explicit plan")
        if tuple(request.trajectory.joint_names) != self.joint_names:
            raise contract_error("joint_names", "must match the raw request")
        if request.anchor != self.anchor:
            raise contract_error("anchor", "must equal the raw request anchor")
        if request.window != self.window:
            raise contract_error("window", "must equal the recomputed request window")
        if self.window.observation_id != request.trajectory.observation_id:
            raise contract_error("window.observation_id", "must equal request observation_id")
        if self.window.target_count != request.target_count:
            raise contract_error("window.target_count", "must equal request target_count")
        if self.window.handoff_time_ns != self.anchor.start_backend_time_ns:
            raise contract_error("window.handoff_time_ns", "must equal anchor start time")
        if self.is_final != request.settle_at_end:
            raise contract_error("is_final", "must equal request settle_at_end")
        if not (request.observation_time_ns <= self.accepted_backend_time_ns
                <= self.anchor.start_backend_time_ns
                and self.accepted_backend_time_ns <= request.trajectory.accept_before_ns):
            raise contract_error("accepted_backend_time_ns",
                                 "must follow the observation and precede both deadlines")
        for name in ("max_accelerations", "max_target_deviation_limits",
                     "raw_terminal_velocities", "restricted_terminal_velocities",
                     "actual_terminal_positions", "actual_terminal_velocities",
                     "actual_terminal_accelerations", "max_target_deviations"):
            values = getattr(self, name)
            if len(values) != joint_count:
                raise contract_error(name, "must contain one value per joint")
            if name in ("max_accelerations", "max_target_deviation_limits") and any(
                    value <= 0.0 for value in values):
                raise contract_error(name, "must be positive")
        if any(value < 0.0 for value in self.max_target_deviations):
            raise contract_error("max_target_deviations", "must not be negative")
        for name in ("velocity_limited", "acceleration_limited", "boundary_preserved"):
            if len(getattr(self, name)) != joint_count:
                raise contract_error(name, "must contain one value per joint")
        if self.requested_window_ns != self.window.end_time_ns:
            raise contract_error("requested_window_ns", "must equal the window end time")
        if self.curve_duration_ns < self.plan.duration_ns:
            raise contract_error("curve_duration_ns", "must cover the plan duration")
        if self.duration_grid_ns is not None:
            if self.plan.duration_ns % self.duration_grid_ns != 0:
                raise contract_error("duration_grid_ns", "plan duration must lie on the grid")
        if self.is_final:
            if self.plan.duration_ns < self.requested_window_ns:
                raise contract_error("plan.duration_ns", "a final plan must cover the window")
        elif self.plan.duration_ns != self.requested_window_ns:
            raise contract_error("plan.duration_ns", "a non-final plan must end at the hand-off")
        return self

    context = property(lambda self: self.request.trajectory.context)

    command_id = property(lambda self: self.request.trajectory.command_id)

    command_seq = property(lambda self: self.request.trajectory.command_seq)

    observation_id = property(lambda self: self.request.trajectory.observation_id)

    kind = property(lambda self: self.request.kind)

    mode = property(lambda self: self.request.mode)

    stream_id = property(lambda self: self.anchor.stream_id)

    chunk_seq = property(lambda self: self.anchor.seq)

    joint_count = property(lambda self: len(self.joint_names))

    duration_ns = property(lambda self: self.plan.duration_ns)

    start_backend_time_ns = property(lambda self: self.anchor.start_backend_time_ns)

    end_backend_time_ns = property(lambda self: self.start_backend_time_ns + self.plan.duration_ns)

    extension_ns = property(lambda self: max(0, self.plan.duration_ns - self.requested_window_ns))

    flags = property(lambda self: FollowerLimitFlags(
        self.velocity_limited, self.acceleration_limited, self.boundary_preserved))





    def to_document(self) -> dict[str, object]:
        document = self.model_dump(mode="json")
        document["window"]["selected_indices"] = list(self.window.selected_indices)
        return document






class FollowerValidationLimits(StrictContractModel):
    """Explicit pure-standard-library constraints for one follower record."""

    _error_reason: ClassVar[str] = "INVALID_FOLLOWER_REFERENCE"

    joint_names: Annotated[tuple[NonEmptyStr, ...], BeforeValidator(as_tuple),
                           Field(min_length=1, max_length=MAX_JOINTS)]
    max_accelerations: FloatVector
    max_target_deviations: FloatVector
    target_count: Annotated[int, Field(strict=True, ge=2, le=MAX_TARGET_COUNT)]
    max_terminal_tail_duration_ns: Annotated[int, Field(strict=True, ge=1, le=MAX_HORIZON_NS)]
    duration_grid_ns: Annotated[int, Field(strict=True, ge=1, le=MAX_HORIZON_NS)] | None = None

    @model_validator(mode="after")
    def _limits(self) -> Self:
        for name in ("max_accelerations", "max_target_deviations"):
            values = getattr(self, name)
            if len(values) != len(self.joint_names):
                raise contract_error(f"limits.{name}", "must match the joint dimension")
            if any(value <= 0.0 for value in values):
                raise contract_error(f"limits.{name}", "must be positive")
        return self
