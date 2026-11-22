"""Pure cubic-spline evaluation, extrema and continuous-limit verification."""

from __future__ import annotations

import math
import sys
from collections.abc import Sequence
from numbers import Real
from typing import Annotated, ClassVar, Self

from pydantic import BeforeValidator, Field, model_validator
from robot_contracts.models import ContractError, FloatVector, JointSpec, Target
from robot_contracts.trajectory.constants import (
    LIMIT_ATOL,
    LIMIT_RTOL,
    MAX_HORIZON_NS,
    MAX_JOINTS,
    MAX_SEGMENTS,
    POSITION_ATOL,
    RELATIVE_TOLERANCE,
    VELOCITY_ATOL,
)
from robot_contracts.trajectory.spline import (
    JointExtremes,
    ReferenceExtremes,
)
from robot_contracts.validation import StrictContractModel, as_tuple, contract_error

_EPSILON = sys.float_info.epsilon


def _invalid(location: str, message: str) -> ContractError:
    return ContractError("INVALID_REFERENCE_PLAN", f"{location}: {message}")


def _is_close(left: float, right: float, *, atol: float) -> bool:
    return math.isclose(left, right, rel_tol=RELATIVE_TOLERANCE, abs_tol=atol)


def _limit_tolerance(bound: float) -> float:
    return LIMIT_ATOL + LIMIT_RTOL * abs(bound)


def _finite_values(value: object, location: str, minimum: int = 1, maximum: int = MAX_JOINTS):
    try:
        raw = tuple(value)  # type: ignore[arg-type]
    except TypeError as error:
        raise _invalid(location, "must be an array of real numbers") from error
    if any(isinstance(item, bool) or not isinstance(item, Real) for item in raw):
        raise _invalid(location, "must contain real numbers")
    try:
        values = tuple(float(item) for item in raw)
    except (TypeError, ValueError, OverflowError) as error:
        raise _invalid(location, "cannot be represented as a finite float") from error
    if not minimum <= len(values) <= maximum:
        raise _invalid(location, f"must contain {minimum}..{maximum} values")
    if not all(math.isfinite(item) for item in values):
        raise _invalid(location, "must be finite")
    return values


def validate_velocity_boundaries(
    velocities: Sequence[float] | None, joints: Sequence[JointSpec], location: str
) -> tuple[float, ...]:
    joint_items = tuple(joints)
    if not 1 <= len(joint_items) <= MAX_JOINTS:
        raise _invalid("joints", f"must contain 1..{MAX_JOINTS} joints")
    if velocities is None:
        return (0.0,) * len(joint_items)
    values = _finite_values(velocities, location)
    if len(values) != len(joint_items):
        raise _invalid(location, f"has {len(values)} values for {len(joint_items)} joints")
    for index, (velocity, joint) in enumerate(zip(values, joint_items, strict=True)):
        limit = float(joint.max_velocity)
        if abs(velocity) > limit + _limit_tolerance(limit):
            raise ContractError("VELOCITY_LIMIT",
                                f"{location}[{index}] ({joint.name}) exceeds {limit!r}")
    return values






class ReferenceInputs(StrictContractModel):
    """Normalized, immutable inputs shared by compilation and verification."""

    _error_reason: ClassVar[str] = "INVALID_REFERENCE_PLAN"
    joints: Annotated[tuple[JointSpec, ...], BeforeValidator(as_tuple),
                      Field(min_length=1, max_length=MAX_JOINTS)]
    initial_positions: FloatVector
    targets: Annotated[tuple[Target, ...], BeforeValidator(as_tuple),
                       Field(min_length=1, max_length=MAX_SEGMENTS)]
    max_accelerations: FloatVector
    initial_velocities: tuple[float, ...] = ()
    final_velocities: tuple[float, ...] = ()

    @model_validator(mode="after")
    def _inputs(self) -> Self:
        count = len(self.joints)
        names = [joint.name for joint in self.joints]
        if len(set(names)) != count:
            raise contract_error("joints", "names must be unique")
        for name, values in (("initial_positions", self.initial_positions),
                             ("max_accelerations", self.max_accelerations)):
            if len(values) != count:
                raise contract_error(name, "must match the joint dimension")
        _check_joint_range(self.initial_positions, self.joints, "initial_positions")
        previous = 0
        for index, target in enumerate(self.targets):
            if target.time_ns <= previous or target.time_ns > MAX_HORIZON_NS:
                raise contract_error(f"targets[{index}].time_ns",
                                     "must be strictly increasing inside the horizon")
            if len(target.positions) != count:
                raise contract_error(f"targets[{index}].positions",
                                     "must match the joint dimension")
            _check_joint_range(target.positions, self.joints, f"targets[{index}].positions")
            previous = target.time_ns
        if any(value < 0.0 for value in self.max_accelerations):
            raise contract_error("max_accelerations", "must not be negative")
        return self

    joint_names = property(lambda self: tuple(joint.name for joint in self.joints))
    target_times_ns = property(lambda self: tuple(target.time_ns for target in self.targets))
    target_positions = property(lambda self: tuple(target.positions for target in self.targets))


def validate_reference_inputs(
    initial_positions: Sequence[float], targets: Sequence[Target],
    joints: Sequence[JointSpec], max_accelerations: Sequence[float], *,
    initial_velocities: Sequence[float] | None = None,
    final_velocities: Sequence[float] | None = None,
) -> ReferenceInputs:
    joint_items = tuple(joints)
    return ReferenceInputs(
        joints=joint_items, initial_positions=initial_positions, targets=tuple(targets),
        max_accelerations=max_accelerations,
        initial_velocities=validate_velocity_boundaries(
            initial_velocities, joint_items, "initial_velocities"),
        final_velocities=validate_velocity_boundaries(
            final_velocities, joint_items, "final_velocities"))


def _check_joint_range(positions: Sequence[float], joints: Sequence[JointSpec], location: str):
    for index, (position, joint) in enumerate(zip(positions, joints, strict=True)):
        lower, upper = float(joint.minimum), float(joint.maximum)
        if not lower - _limit_tolerance(lower) <= position <= upper + _limit_tolerance(upper):
            raise ContractError("JOINT_LIMIT",
                                f"{location}[{index}] ({joint.name}) {position!r} out of range")


def evaluate_cubic_segment(segment: object, local_s: float, location: str = "segment"):
    """Evaluate position, velocity and acceleration on one cubic polynomial segment."""
    return _state_at(segment, local_s, location)


def _check_vector_close(left: Sequence[float], right: Sequence[float], *, atol: float,
                        location: str, quantity: str, joint_names: Sequence[str]) -> None:
    for index, (a, b, name) in enumerate(zip(left, right, joint_names, strict=True)):
        if not _is_close(a, b, atol=atol):
            raise _invalid(f"{location}[{index}]",
                           f"joint {name!r} {quantity} is discontinuous: {a!r} != {b!r}")


def check_c1_continuity(
    left_positions: Sequence[float],
    left_velocities: Sequence[float],
    right_positions: Sequence[float],
    right_velocities: Sequence[float],
    *,
    location: str = "boundary",
    joint_names: Sequence[str] | None = None,
) -> None:
    """Verify position and velocity continuity across a segment boundary."""
    names = (
        tuple(f"joint[{index}]" for index in range(len(left_positions)))
        if joint_names is None
        else joint_names
    )
    _check_vector_close(
        left_positions,
        right_positions,
        atol=POSITION_ATOL,
        location=f"{location}.positions",
        quantity="position",
        joint_names=names,
    )
    _check_vector_close(
        left_velocities,
        right_velocities,
        atol=VELOCITY_ATOL,
        location=f"{location}.velocities",
        quantity="velocity",
        joint_names=names,
    )


def _state_at(segment: object, local_s: float, location: str):
    positions: list[float] = []
    velocities: list[float] = []
    accelerations: list[float] = []
    for index, (c0, c1, c2, c3) in enumerate(
            zip(segment.c0, segment.c1, segment.c2, segment.c3, strict=True)):
        c3_s = c3 * local_s
        position = ((c3_s + c2) * local_s + c1) * local_s + c0
        velocity = (3.0 * c3_s + 2.0 * c2) * local_s + c1
        acceleration = 6.0 * c3_s + 2.0 * c2
        if not all(map(math.isfinite, (position, velocity, acceleration))):
            raise _invalid(f"{location}.c*[{index}]",
                           f"polynomial evaluation is not finite at {local_s!r}s")
        positions.append(position)
        velocities.append(velocity)
        accelerations.append(acceleration)
    return tuple(positions), tuple(velocities), tuple(accelerations)


def _quadratic_real_roots(a: float, b: float, c: float) -> tuple[float, ...]:
    if a == 0.0:
        return () if b == 0.0 else (-c / b,)
    discriminant = b * b - 4.0 * a * c
    magnitude = b * b + abs(4.0 * a * c)
    tolerance = 32.0 * _EPSILON * max(magnitude, sys.float_info.min)
    if discriminant < -tolerance:
        return ()
    discriminant = max(discriminant, 0.0)
    q = -0.5 * (b + math.copysign(math.sqrt(discriminant), b))
    return (0.0,) if q == 0.0 else (q / a, c / q)


def _in_interval(root: float, duration_s: float) -> float | None:
    if not math.isfinite(root):
        return None
    margin = 1e-12 * max(1.0, duration_s)
    if root < -margin or root > duration_s + margin:
        return None
    return min(max(root, 0.0), duration_s)


def _velocity_roots(c1: float, c2: float, c3: float, duration_s: float) -> tuple[float, ...]:
    scale = max(abs(c1), abs(c2), abs(c3))
    if scale == 0.0:
        return ()
    roots = _quadratic_real_roots(3.0 * (c3 / scale), 2.0 * (c2 / scale), c1 / scale)
    return tuple(root for root in (_in_interval(item, duration_s) for item in roots)
                 if root is not None)


def _acceleration_root(c2: float, c3: float, duration_s: float) -> float | None:
    return None if c3 == 0.0 else _in_interval(-(c2 / c3) / 3.0, duration_s)


def compute_reference_extremes(plan) -> ReferenceExtremes:
    count = plan.joint_count
    minimum = [math.inf] * count
    maximum = [-math.inf] * count
    velocity = [0.0] * count
    acceleration = [0.0] * count
    for segment_index, segment in enumerate(plan.segments):
        location = f"segments[{segment_index}]"
        duration_s = segment.duration_s
        for axis in range(count):
            at = f"{location}.c*[{axis}]"
            end_positions, end_velocities, end_accelerations = _state_at(
                segment, duration_s, at)
            _, _, start_accelerations = _state_at(segment, 0.0, at)
            candidates = [segment.c0[axis], end_positions[axis]]
            for root in _velocity_roots(segment.c1[axis], segment.c2[axis],
                                        segment.c3[axis], duration_s):
                candidates.append(_state_at(segment, root, at)[0][axis])
            minimum[axis] = min(minimum[axis], min(candidates))
            maximum[axis] = max(maximum[axis], max(candidates))
            speeds = [segment.c1[axis], end_velocities[axis]]
            root = _acceleration_root(segment.c2[axis], segment.c3[axis], duration_s)
            if root is not None:
                speeds.append(_state_at(segment, root, at)[1][axis])
            velocity[axis] = max(velocity[axis], max(abs(value) for value in speeds))
            acceleration[axis] = max(acceleration[axis], abs(start_accelerations[axis]),
                                     abs(end_accelerations[axis]))
    values = minimum + maximum + velocity + acceleration
    if not all(map(math.isfinite, values)):
        raise _invalid("plan", "extrema are not finite")
    return ReferenceExtremes(tuple(
        JointExtremes(plan.joint_names[index], minimum[index], maximum[index],
                      velocity[index], acceleration[index]) for index in range(count)))


def check_continuous_limits(extremes: ReferenceExtremes, joints: Sequence[JointSpec],
                            max_accelerations: Sequence[float]) -> None:
    joint_items = tuple(joints)
    extreme_items = tuple(extremes.joints)
    acceleration_items = tuple(max_accelerations)
    if len(extreme_items) != len(joint_items) or len(acceleration_items) != len(joint_items):
        raise _invalid("extremes", "joint/extrema/acceleration lengths must match")
    seen: set[str] = set()
    for index, (item, joint, acceleration) in enumerate(
            zip(extreme_items, joint_items, acceleration_items, strict=True)):
        location = f"extremes.joints[{index}]"
        if joint.name in seen:
            raise _invalid(f"joints[{index}].name", "must be unique")
        seen.add(joint.name)
        if item.joint_name != joint.name:
            raise _invalid(f"{location}.joint_name", "must match the declared joint order")
        lower, upper = float(joint.minimum), float(joint.maximum)
        velocity_limit = float(joint.max_velocity)
        if acceleration < 0.0:
            raise _invalid(f"max_accelerations[{index}]", "must not be negative")
        if item.minimum_position > item.maximum_position:
            raise _invalid(location, "position extrema must be ordered")
        if item.maximum_velocity < 0.0 or item.maximum_acceleration < 0.0:
            raise _invalid(location, "absolute peaks must be nonnegative")
        if item.minimum_position < lower - _limit_tolerance(lower):
            raise ContractError("JOINT_LIMIT", f"joint {joint.name!r} minimum below bound")
        if item.maximum_position > upper + _limit_tolerance(upper):
            raise ContractError("JOINT_LIMIT", f"joint {joint.name!r} maximum above bound")
        if item.maximum_velocity > velocity_limit + _limit_tolerance(velocity_limit):
            raise ContractError("VELOCITY_LIMIT", f"joint {joint.name!r} velocity exceeded")
        if item.maximum_acceleration > acceleration + _limit_tolerance(acceleration):
            raise ContractError("ACCELERATION_LIMIT",
                                f"joint {joint.name!r} acceleration exceeded")
