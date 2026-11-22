"""Streaming control requests shared by the backend and ROS adapters."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Annotated, ClassVar, Self

from pydantic import BeforeValidator, Field, model_validator

from .action_chunks import (
    INT64_MAX,
    MAX_TARGET_COUNT,
    UINT64_MAX,
    ActionWindow,
    ChunkAnchor,
)
from .control import MAX_ID_LENGTH, ControlCommand
from .models import FloatVector, RunContext, Target
from .trajectory.constants import VELOCITY_ATOL
from .trajectory.spline import (
    MAX_HORIZON_NS,
    MAX_JOINTS,
    MAX_SEGMENTS,
    CubicSegment,
    ReferencePlan,
)
from .validation import (
    NonEmptyStr,
    PositiveInt,
    StrictContractModel,
    as_tuple,
    contract_error,
)

REQUEST_STREAM_FIRST = 2
REQUEST_STREAM_NEXT = 3
REQUEST_KINDS = (REQUEST_STREAM_FIRST, REQUEST_STREAM_NEXT)
REQUEST_KIND_NAMES = {
    REQUEST_STREAM_FIRST: "STREAM_FIRST",
    REQUEST_STREAM_NEXT: "STREAM_NEXT",
}


class Trajectory(StrictContractModel):
    """One frozen command trajectory: identity, joint order and timed targets."""

    _error_reason: ClassVar[str] = "INVALID_STREAM_REQUEST"

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


class StreamRequest(StrictContractModel):
    """One frozen stream command request in exactly one of the three modes."""

    _error_reason: ClassVar[str] = "INVALID_STREAM_REQUEST"

    trajectory: Trajectory
    kind: int
    anchor: ChunkAnchor | None = None
    observation_time_ns: Annotated[int, Field(strict=True, ge=0, le=INT64_MAX)] = 0
    target_count: Annotated[int, Field(strict=True, ge=0, le=MAX_TARGET_COUNT)] = 0
    settle_at_end: bool = True
    final_velocities: FloatVector | None = None

    @model_validator(mode="after")
    def _mode(self) -> Self:
        if self.kind not in REQUEST_KINDS:
            raise contract_error("kind", f"must be one of {REQUEST_KINDS}")
        joint_count = len(self.trajectory.joint_names)
        if self.final_velocities is not None:
            if len(self.final_velocities) != joint_count:
                raise contract_error("final_velocities", "must match the joint dimension")
            if self.settle_at_end and any(
                abs(velocity) > VELOCITY_ATOL for velocity in self.final_velocities
            ):
                raise contract_error("final_velocities",
                                     "must be zero when settle_at_end is true")
        if self.anchor is None:
            raise contract_error("anchor", "stream modes require a ChunkAnchor")
        if self.anchor.robot_profile_hash != self.trajectory.context.robot_profile_hash:
            raise contract_error("anchor.robot_profile_hash",
                                 "must equal the trajectory profile hash")
        if len(self.anchor.reference.positions) != joint_count:
            raise contract_error("anchor.reference.positions",
                                 "must match the joint dimension")
        if self.kind == REQUEST_STREAM_FIRST and self.anchor.predecessor_seq != 0:
            raise contract_error("anchor.predecessor_seq", "STREAM_FIRST requires zero")
        if self.kind == REQUEST_STREAM_NEXT and self.anchor.predecessor_seq <= 0:
            raise contract_error("anchor.predecessor_seq", "STREAM_NEXT requires positive")
        if not 2 <= self.target_count <= MAX_TARGET_COUNT:
            raise contract_error("target_count", "must be in 2..MAX_TARGET_COUNT")
        if self.target_count > len(self.trajectory.targets):
            raise contract_error("target_count", "must not exceed the source target count")
        if self.observation_time_ns > self.anchor.start_backend_time_ns:
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
        return REQUEST_KIND_NAMES[self.kind]

    @property
    def window(self) -> ActionWindow:
        """Return the recomputed stream window."""
        assert self.anchor is not None
        return ActionWindow(
            self.trajectory.observation_id,
            self.observation_time_ns,
            self.anchor.start_backend_time_ns,
            self.trajectory.targets,
            self.target_count,
        )


def plan_from_segments(
    joint_names: Sequence[str],
    segments: Sequence[CubicSegment],
) -> ReferencePlan:
    """Explicit helper used by transports to rebuild the public plan."""
    return ReferencePlan(tuple(joint_names), tuple(segments))


__all__ = [
    "REQUEST_KINDS", "REQUEST_KIND_NAMES", "REQUEST_STREAM_FIRST",
    "REQUEST_STREAM_NEXT", "StreamRequest", "Trajectory", "plan_from_segments",
]
