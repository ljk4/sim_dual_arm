"""Immutable cubic reference plans and sampling.

Continuous-interval extrema and plan verification live in the private
``trajectory._math.spline_math`` implementation and are re-exported here.
"""

from __future__ import annotations

from typing import Annotated, ClassVar, Self

from pydantic import BeforeValidator, Field, model_validator

from ..models import ContractError, FloatVector
from ..validation import NonEmptyStr, as_tuple, contract_error
from ..validation import StrictContractModel as _Base
from ._math.spline_math import (
    ACCELERATION_ATOL,
    LIMIT_ATOL,
    LIMIT_RTOL,
    MAX_HORIZON_NS,
    MAX_JOINTS,
    MAX_SEGMENTS,
    POSITION_ATOL,
    RELATIVE_TOLERANCE,
    VELOCITY_ATOL,
    JointExtremes,
    ReferenceExtremes,
    ReferenceInputs,
    _finite_values,
    _in_interval,
    _invalid,
    _is_close,
    _limit_tolerance,
    _quadratic_real_roots,
    _state_at,
    check_continuous_limits,
    compute_reference_extremes,
    validate_reference_inputs,
    validate_velocity_boundaries,
    verify_reference_plan,
)

__all__ = [
    "ACCELERATION_ATOL", "LIMIT_ATOL", "LIMIT_RTOL", "MAX_HORIZON_NS", "MAX_JOINTS",
    "MAX_SEGMENTS", "POSITION_ATOL", "RELATIVE_TOLERANCE", "VELOCITY_ATOL",
    "CubicSegment", "JointExtremes", "ReferenceExtremes", "ReferenceInputs",
    "ReferencePlan", "ReferenceSample", "check_continuous_limits",
    "compute_reference_extremes", "validate_reference_inputs",
    "validate_velocity_boundaries", "verify_reference_plan",
    "_finite_values", "_in_interval", "_is_close", "_limit_tolerance",
    "_quadratic_real_roots", "_state_at",
]


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

    def sample(self, elapsed_ns: int) -> ReferenceSample:
        if isinstance(elapsed_ns, bool) or not isinstance(elapsed_ns, int):
            raise _invalid("elapsed_ns", "must be an integer")
        if elapsed_ns < 0:
            raise _invalid("elapsed_ns", "must not be negative")
        if elapsed_ns <= self.duration_ns:
            return self._sample_within_horizon(elapsed_ns)
        segment = self.segments[-1]
        positions, _, _ = _state_at(segment, segment.duration_s, "segments[-1]")
        zero = (0.0,) * self.joint_count
        return ReferenceSample(positions, zero, zero)

    def sample_interval(self, elapsed_ns: int) -> ReferenceSample:
        if isinstance(elapsed_ns, bool) or not isinstance(elapsed_ns, int):
            raise _invalid("elapsed_ns", "must be an integer")
        if elapsed_ns < 0 or elapsed_ns > self.duration_ns:
            raise ContractError("REFERENCE_EXPIRED",
                                f"elapsed_ns {elapsed_ns} outside [0, {self.duration_ns}]")
        return self._sample_within_horizon(elapsed_ns)

    def _sample_within_horizon(self, elapsed: int) -> ReferenceSample:
        for index, segment in enumerate(self.segments):
            if elapsed <= segment.duration_ns:
                return ReferenceSample(
                    *_state_at(segment, elapsed / 1_000_000_000, f"segments[{index}]"))
            elapsed -= segment.duration_ns
        raise AssertionError("validated plan did not contain the elapsed time")
