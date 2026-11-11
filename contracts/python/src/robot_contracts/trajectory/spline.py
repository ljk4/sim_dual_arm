"""Immutable cubic segments, samples, plans and extrema (no evaluator)."""
from __future__ import annotations

from typing import Annotated, ClassVar, Self

from pydantic import BeforeValidator, Field, model_validator

from robot_contracts.models import FloatVector
from robot_contracts.validation import NonEmptyStr, as_tuple, contract_error
from robot_contracts.validation import StrictContractModel as _Base

from .constants import MAX_HORIZON_NS, MAX_JOINTS, MAX_SEGMENTS


class CubicSegment(_Base):
    """One cubic segment in local seconds, with per-joint ascending coefficients."""

    _error_reason: ClassVar[str] = "INVALID_REFERENCE_PLAN"
    duration_ns: Annotated[int, Field(strict=True, ge=1, le=MAX_HORIZON_NS)]
    c0: FloatVector
    c1: FloatVector
    c2: FloatVector
    c3: FloatVector

    @model_validator(mode="after")
    def _same_joint_count(self) -> Self:
        for name in ("c1", "c2", "c3"):
            if len(getattr(self, name)) != len(self.c0):
                raise contract_error(name, "has a different length from c0")
        return self

    joint_count = property(lambda self: len(self.c0))
    duration_s = property(lambda self: self.duration_ns / 1_000_000_000)

class ReferenceSample(_Base):
    """One immutable sampled state, all arrays in joint order."""

    _error_reason: ClassVar[str] = "INVALID_REFERENCE_PLAN"
    positions: FloatVector
    velocities: FloatVector
    accelerations: FloatVector

    @model_validator(mode="after")
    def _same_joint_count(self) -> Self:
        for name in ("velocities", "accelerations"):
            if len(getattr(self, name)) != len(self.positions):
                raise contract_error(name, "has a different length from positions")
        return self

class ReferencePlan(_Base):
    """An immutable sequence of cubic segments starting at local elapsed zero."""

    _error_reason: ClassVar[str] = "INVALID_REFERENCE_PLAN"
    joint_names: Annotated[tuple[NonEmptyStr, ...], BeforeValidator(as_tuple),
                           Field(min_length=1, max_length=MAX_JOINTS)]
    segments: Annotated[tuple[CubicSegment, ...], BeforeValidator(as_tuple),
                        Field(min_length=1, max_length=MAX_SEGMENTS)]

    @model_validator(mode="after")
    def _plan(self) -> Self:
        if len(set(self.joint_names)) != len(self.joint_names):
            raise contract_error("joint_names", "must be unique")
        total = sum(segment.duration_ns for segment in self.segments)
        for index, segment in enumerate(self.segments):
            if segment.joint_count != len(self.joint_names):
                raise contract_error(f"segments[{index}]", "joint dimension differs from plan")
        if total > MAX_HORIZON_NS:
            raise contract_error("segments", f"horizon {total}ns exceeds {MAX_HORIZON_NS}ns")
        return self

    joint_count = property(lambda self: len(self.joint_names))
    duration_ns = property(lambda self: sum(segment.duration_ns for segment in self.segments))
    horizon_ns = property(lambda self: self.duration_ns)




class JointExtremes(_Base):
    """Continuous-interval extrema for one joint over the whole plan."""

    _error_reason: ClassVar[str] = "INVALID_REFERENCE_PLAN"
    joint_name: NonEmptyStr
    minimum_position: float
    maximum_position: float
    maximum_velocity: float
    maximum_acceleration: float

class ReferenceExtremes(_Base):
    """Readable per-joint extrema suitable for logging and independent checks."""

    _error_reason: ClassVar[str] = "INVALID_REFERENCE_PLAN"
    joints: tuple[JointExtremes, ...]

    joint_names = property(
        lambda self: tuple(extremes.joint_name for extremes in self.joints))
