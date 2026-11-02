"""Immutable timed cubic reference plans, sampling, extrema and validation.

This module deliberately uses only the Python standard library. Producers fit
splines elsewhere and hand this module explicit per-segment coefficients; the
shared contract never imports ROS, NumPy, SciPy or MuJoCo.

A segment is expressed in local seconds ``s``::

    p(s) = c0 + c1*s + c2*s^2 + c3*s^3
    v(s) = c1 + 2*c2*s + 3*c3*s^2
    a(s) = 2*c2 + 6*c3*s

Durations and sample times are integer nanoseconds. Sampling never reads a
wall clock and never extrapolates beyond the plan horizon.
"""

from __future__ import annotations

import math
import sys
from collections.abc import Sequence
from dataclasses import dataclass
from numbers import Integral, Real

from .models import ContractError, JointSpec, Target

MAX_JOINTS = 64
MAX_SEGMENTS = 128
MAX_HORIZON_NS = 3_000_000_000

# Explicit comparison tolerances. They are tight enough to reject a real
# discontinuity while allowing normal double-precision spline round-off.
POSITION_ATOL = 1e-9
VELOCITY_ATOL = 1e-9
ACCELERATION_ATOL = 1e-9
RELATIVE_TOLERANCE = 1e-9

# Limits are accepted exactly at the configured boundary. The tiny absolute
# allowance only absorbs coefficient/evaluation round-off at that boundary.
LIMIT_ATOL = 1e-9
LIMIT_RTOL = 1e-9

_EPSILON = sys.float_info.epsilon


def _invalid(location: str, message: str) -> ContractError:
    return ContractError("INVALID_REFERENCE_PLAN", f"{location}: {message}")


def _positive_int(value: object, location: str, maximum: int | None = None) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise _invalid(location, "must be an integer")
    result = int(value)
    if result <= 0:
        raise _invalid(location, "must be positive")
    if maximum is not None and result > maximum:
        raise _invalid(location, f"must not exceed {maximum}")
    return result


def _nonnegative_int(value: object, location: str) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise _invalid(location, "must be an integer")
    result = int(value)
    if result < 0:
        raise _invalid(location, "must not be negative")
    return result


def _finite_float(value: object, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise _invalid(location, "must be a finite real number")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise _invalid(location, "cannot be represented as a finite float") from error
    if not math.isfinite(result):
        raise _invalid(location, "must be finite")
    return result


def _coefficient_tuple(value: object, location: str) -> tuple[float, ...]:
    if isinstance(value, str | bytes):
        raise _invalid(location, "must be an array of real numbers")
    try:
        items = tuple(value)  # type: ignore[arg-type]
    except TypeError as error:
        raise _invalid(location, "must be an array of real numbers") from error
    if not 1 <= len(items) <= MAX_JOINTS:
        raise _invalid(location, f"must contain 1..{MAX_JOINTS} values")
    return tuple(_finite_float(item, f"{location}[{index}]") for index, item in enumerate(items))


def _is_close(left: float, right: float, *, atol: float) -> bool:
    return math.isclose(left, right, rel_tol=RELATIVE_TOLERANCE, abs_tol=atol)


def _limit_tolerance(bound: float) -> float:
    return LIMIT_ATOL + LIMIT_RTOL * abs(bound)


def validate_velocity_boundaries(
    velocities: Sequence[float] | None,
    joints: Sequence[JointSpec],
    location: str,
) -> tuple[float, ...]:
    """Normalize one declared endpoint-velocity vector without clipping.

    ``None`` means the all-zero vector, preserving the historical clamped
    behaviour.  An explicit vector follows the same finite/real/1..64/dimension
    rules as every other coefficient vector, and each absolute axis value is
    checked against ``JointSpec.max_velocity`` exactly like the continuous
    plan limit.  The value is never silently reduced to the limit.
    """
    joint_items = tuple(joints)
    if not 1 <= len(joint_items) <= MAX_JOINTS:
        raise _invalid("joints", f"must contain 1..{MAX_JOINTS} joints")
    joint_count = len(joint_items)
    if velocities is None:
        return (0.0,) * joint_count
    values = _coefficient_tuple(velocities, location)
    if len(values) != joint_count:
        raise _invalid(location, f"has {len(values)} values for {joint_count} joints")
    for index, (velocity, joint) in enumerate(zip(values, joint_items, strict=True)):
        if not isinstance(joint, JointSpec):
            raise _invalid(f"joints[{index}]", "must be a JointSpec")
        limit = _finite_float(joint.max_velocity, f"joints[{index}].max_velocity")
        if limit <= 0.0:
            raise _invalid(f"joints[{index}].max_velocity", "must be positive")
        if abs(velocity) > limit + _limit_tolerance(limit):
            raise ContractError(
                "VELOCITY_LIMIT",
                f"{location}[{index}] ({joint.name}) magnitude {abs(velocity)!r} exceeds {limit!r}",
            )
    return values


@dataclass(frozen=True)
class CubicSegment:
    """One cubic segment in local seconds, with per-joint ascending coefficients."""

    duration_ns: int
    c0: tuple[float, ...]
    c1: tuple[float, ...]
    c2: tuple[float, ...]
    c3: tuple[float, ...]

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "duration_ns",
            _positive_int(self.duration_ns, "duration_ns", MAX_HORIZON_NS),
        )
        coefficients = tuple(
            _coefficient_tuple(getattr(self, name), name) for name in ("c0", "c1", "c2", "c3")
        )
        joint_count = len(coefficients[0])
        for name, values in zip(("c0", "c1", "c2", "c3"), coefficients, strict=True):
            if len(values) != joint_count:
                raise _invalid(
                    name,
                    f"has {len(values)} values but c0 has {joint_count}",
                )
            object.__setattr__(self, name, values)

    @property
    def joint_count(self) -> int:
        return len(self.c0)

    @property
    def duration_s(self) -> float:
        return self.duration_ns / 1_000_000_000


@dataclass(frozen=True)
class ReferencePlan:
    """An immutable sequence of cubic segments starting at local elapsed zero."""

    joint_names: tuple[str, ...]
    segments: tuple[CubicSegment, ...]

    def __post_init__(self) -> None:
        if isinstance(self.joint_names, str | bytes):
            raise _invalid("joint_names", "must be an array of names")
        try:
            names = tuple(self.joint_names)
        except TypeError as error:
            raise _invalid("joint_names", "must be an array of names") from error
        if not 1 <= len(names) <= MAX_JOINTS:
            raise _invalid("joint_names", f"must contain 1..{MAX_JOINTS} names")
        for index, name in enumerate(names):
            if not isinstance(name, str) or not name:
                raise _invalid(f"joint_names[{index}]", "must be a non-empty string")
        if len(set(names)) != len(names):
            raise _invalid("joint_names", "must be unique")
        object.__setattr__(self, "joint_names", names)

        try:
            segments = tuple(self.segments)
        except TypeError as error:
            raise _invalid("segments", "must be an array of CubicSegment values") from error
        if not 1 <= len(segments) <= MAX_SEGMENTS:
            raise _invalid("segments", f"must contain 1..{MAX_SEGMENTS} segments")
        total_ns = 0
        for index, segment in enumerate(segments):
            if not isinstance(segment, CubicSegment):
                raise _invalid(f"segments[{index}]", "must be a CubicSegment")
            if segment.joint_count != len(names):
                raise _invalid(
                    f"segments[{index}]",
                    f"has {segment.joint_count} joints but plan has {len(names)}",
                )
            total_ns += segment.duration_ns
        if total_ns > MAX_HORIZON_NS:
            raise _invalid("segments", f"horizon {total_ns}ns exceeds {MAX_HORIZON_NS}ns")
        object.__setattr__(self, "segments", segments)

    @property
    def joint_count(self) -> int:
        return len(self.joint_names)

    @property
    def duration_ns(self) -> int:
        return sum(segment.duration_ns for segment in self.segments)

    @property
    def horizon_ns(self) -> int:
        """Alias for the total plan duration; not an execution-complete signal."""
        return self.duration_ns

    def sample(self, elapsed_ns: int) -> ReferenceSample:
        """Sample the plan at local execution time, holding the end after the horizon.

        An elapsed time exactly on an internal boundary selects the segment that
        ends there. The horizon itself uses the final segment end. Only values
        strictly after the horizon are held with zero velocity and acceleration;
        reaching the horizon is a mathematical fact, not execution completion.
        """
        elapsed = _nonnegative_int(elapsed_ns, "elapsed_ns")
        if elapsed > self.duration_ns:
            segment = self.segments[-1]
            positions, _, _ = _state_at(
                segment,
                segment.duration_s,
                f"segments[{len(self.segments) - 1}]",
            )
            zero = (0.0,) * self.joint_count
            return ReferenceSample(positions, zero, zero)

        return self._sample_within_horizon(elapsed)

    def sample_interval(self, elapsed_ns: int) -> ReferenceSample:
        """Sample strictly inside ``[0, duration_ns]`` without holding or extrapolating.

        This is the entry point intended for continuous action chunks.  Values
        before zero or after the horizon raise ``REFERENCE_EXPIRED`` rather than
        silently returning a zero-velocity hold.
        """
        if isinstance(elapsed_ns, bool) or not isinstance(elapsed_ns, Integral):
            raise _invalid("elapsed_ns", "must be an integer")
        elapsed = int(elapsed_ns)
        if elapsed < 0 or elapsed > self.duration_ns:
            raise ContractError(
                "REFERENCE_EXPIRED",
                f"elapsed_ns {elapsed} is outside [0, {self.duration_ns}]",
            )
        return self._sample_within_horizon(elapsed)

    def _sample_within_horizon(self, elapsed: int) -> ReferenceSample:
        remaining = elapsed
        for index, segment in enumerate(self.segments):
            if remaining <= segment.duration_ns:
                positions, velocities, accelerations = _state_at(
                    segment,
                    remaining / 1_000_000_000,
                    f"segments[{index}]",
                )
                return ReferenceSample(positions, velocities, accelerations)
            remaining -= segment.duration_ns
        raise AssertionError("validated plan did not contain the elapsed time")


@dataclass(frozen=True)
class ReferenceSample:
    """One immutable sampled state, all arrays in joint order."""

    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    accelerations: tuple[float, ...]

    def __post_init__(self) -> None:
        arrays = tuple(
            _coefficient_tuple(getattr(self, name), name)
            for name in ("positions", "velocities", "accelerations")
        )
        joint_count = len(arrays[0])
        for name, values in zip(("positions", "velocities", "accelerations"), arrays, strict=True):
            if len(values) != joint_count:
                raise _invalid(name, f"has {len(values)} values but positions has {joint_count}")
            object.__setattr__(self, name, values)


@dataclass(frozen=True)
class JointExtremes:
    """Continuous-interval extrema for one joint over the whole plan."""

    joint_name: str
    minimum_position: float
    maximum_position: float
    maximum_velocity: float
    maximum_acceleration: float

    @property
    def min_position(self) -> float:
        return self.minimum_position

    @property
    def max_position(self) -> float:
        return self.maximum_position

    @property
    def max_velocity(self) -> float:
        return self.maximum_velocity

    @property
    def max_acceleration(self) -> float:
        return self.maximum_acceleration


@dataclass(frozen=True)
class ReferenceExtremes:
    """Readable per-joint extrema suitable for logging and independent checks."""

    joints: tuple[JointExtremes, ...]

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(extremes.joint_name for extremes in self.joints)


@dataclass(frozen=True)
class ReferenceInputs:
    """Normalized, immutable inputs shared by compilation and verification."""

    joint_names: tuple[str, ...]
    joints: tuple[JointSpec, ...]
    initial_positions: tuple[float, ...]
    target_times_ns: tuple[int, ...]
    target_positions: tuple[tuple[float, ...], ...]
    max_accelerations: tuple[float, ...]
    initial_velocities: tuple[float, ...] = ()
    final_velocities: tuple[float, ...] = ()


def validate_reference_inputs(
    initial_positions: Sequence[float],
    targets: Sequence[Target],
    joints: Sequence[JointSpec],
    max_accelerations: Sequence[float],
    *,
    initial_velocities: Sequence[float] | None = None,
    final_velocities: Sequence[float] | None = None,
) -> ReferenceInputs:
    """Validate dimensions, ordering, joint metadata and explicit thresholds.

    Continuous-interval limits are checked later against the compiled plan. Here
    the initial value and the original waypoints are also rejected if they
    already lie outside the declared joint range.
    """
    try:
        joint_items = tuple(joints)
    except TypeError as error:
        raise _invalid("joints", "must be an array of JointSpec values") from error
    if not 1 <= len(joint_items) <= MAX_JOINTS:
        raise _invalid("joints", f"must contain 1..{MAX_JOINTS} joints")

    names: list[str] = []
    for index, joint in enumerate(joint_items):
        if not isinstance(joint, JointSpec):
            raise _invalid(f"joints[{index}]", "must be a JointSpec")
        if not isinstance(joint.name, str) or not joint.name:
            raise _invalid(f"joints[{index}].name", "must be a non-empty string")
        if joint.name in names:
            raise _invalid(f"joints[{index}].name", "must be unique")
        minimum = _finite_float(joint.minimum, f"joints[{index}].minimum")
        maximum = _finite_float(joint.maximum, f"joints[{index}].maximum")
        max_velocity = _finite_float(joint.max_velocity, f"joints[{index}].max_velocity")
        if minimum > maximum:
            raise _invalid(f"joints[{index}]", "minimum must not exceed maximum")
        if max_velocity <= 0.0:
            raise _invalid(f"joints[{index}].max_velocity", "must be positive")
        names.append(joint.name)

    initial = _coefficient_tuple(initial_positions, "initial_positions")
    if len(initial) != len(names):
        raise _invalid(
            "initial_positions",
            f"has {len(initial)} values for {len(names)} joints",
        )
    _check_joint_range(initial, joint_items, "initial_positions")

    initial_velocities = validate_velocity_boundaries(
        initial_velocities,
        joint_items,
        "initial_velocities",
    )
    final_velocities = validate_velocity_boundaries(
        final_velocities,
        joint_items,
        "final_velocities",
    )

    try:
        target_items = tuple(targets)
    except TypeError as error:
        raise _invalid("targets", "must be an array of Target values") from error
    if not 1 <= len(target_items) <= MAX_SEGMENTS:
        raise _invalid("targets", f"must contain 1..{MAX_SEGMENTS} targets")

    times: list[int] = []
    positions: list[tuple[float, ...]] = []
    previous_time = 0
    for index, target in enumerate(target_items):
        if not isinstance(target, Target):
            raise _invalid(f"targets[{index}]", "must be a Target")
        time_ns = _positive_int(target.time_ns, f"targets[{index}].time_ns", MAX_HORIZON_NS)
        if time_ns <= previous_time:
            raise _invalid(
                f"targets[{index}].time_ns",
                "must be strictly increasing and after elapsed zero",
            )
        target_positions = _coefficient_tuple(target.positions, f"targets[{index}].positions")
        if len(target_positions) != len(names):
            raise _invalid(
                f"targets[{index}].positions",
                f"has {len(target_positions)} values for {len(names)} joints",
            )
        _check_joint_range(target_positions, joint_items, f"targets[{index}].positions")
        times.append(time_ns)
        positions.append(target_positions)
        previous_time = time_ns

    accelerations = _coefficient_tuple(max_accelerations, "max_accelerations")
    if len(accelerations) != len(names):
        raise _invalid(
            "max_accelerations",
            f"has {len(accelerations)} values for {len(names)} joints",
        )
    for index, acceleration in enumerate(accelerations):
        if acceleration < 0.0:
            raise _invalid(f"max_accelerations[{index}]", "must not be negative")

    return ReferenceInputs(
        joint_names=tuple(names),
        joints=joint_items,
        initial_positions=initial,
        initial_velocities=initial_velocities,
        target_times_ns=tuple(times),
        target_positions=tuple(positions),
        max_accelerations=accelerations,
        final_velocities=final_velocities,
    )


def _check_joint_range(
    positions: Sequence[float], joints: Sequence[JointSpec], location: str
) -> None:
    for index, (position, joint) in enumerate(zip(positions, joints, strict=True)):
        lower = float(joint.minimum)
        upper = float(joint.maximum)
        if position < lower - _limit_tolerance(lower):
            raise ContractError(
                "JOINT_LIMIT",
                f"{location}[{index}] ({joint.name}) {position!r} is below {lower!r}",
            )
        if position > upper + _limit_tolerance(upper):
            raise ContractError(
                "JOINT_LIMIT",
                f"{location}[{index}] ({joint.name}) {position!r} is above {upper!r}",
            )


def _state_at(
    segment: CubicSegment, local_s: float, location: str
) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
    """Evaluate position, velocity and acceleration for every joint."""
    positions: list[float] = []
    velocities: list[float] = []
    accelerations: list[float] = []
    for index, (c0, c1, c2, c3) in enumerate(
        zip(segment.c0, segment.c1, segment.c2, segment.c3, strict=True)
    ):
        c3_s = c3 * local_s
        position = ((c3_s + c2) * local_s + c1) * local_s + c0
        velocity = (3.0 * c3_s + 2.0 * c2) * local_s + c1
        acceleration = 6.0 * c3_s + 2.0 * c2
        if not (
            math.isfinite(position) and math.isfinite(velocity) and math.isfinite(acceleration)
        ):
            raise _invalid(
                f"{location}.c*[{index}]",
                f"polynomial evaluation is not finite at local time {local_s!r}s",
            )
        positions.append(position)
        velocities.append(velocity)
        accelerations.append(acceleration)
    return tuple(positions), tuple(velocities), tuple(accelerations)


def _quadratic_real_roots(a: float, b: float, c: float) -> tuple[float, ...]:
    """Stable real roots of ``a*x^2 + b*x + c``, treating degenerate cases.

    Coefficients are expected to be pre-scaled so squares cannot overflow. A
    genuinely negative discriminant (beyond rounding tolerance) means no real
    roots; it is never reported as an extremum.
    """
    if a == 0.0:
        if b == 0.0:
            return ()
        return (-c / b,)

    discriminant = b * b - 4.0 * a * c
    magnitude = b * b + abs(4.0 * a * c)
    tolerance = 32.0 * _EPSILON * max(magnitude, sys.float_info.min)
    if discriminant < -tolerance:
        return ()
    if discriminant < 0.0:
        discriminant = 0.0

    root_discriminant = math.sqrt(discriminant)
    q = -0.5 * (b + math.copysign(root_discriminant, b))
    if q == 0.0:
        return (0.0,)
    return (q / a, c / q)


def _in_interval(root: float, duration_s: float) -> float | None:
    if not math.isfinite(root):
        return None
    margin = 1e-12 * max(1.0, duration_s)
    if root < -margin or root > duration_s + margin:
        return None
    return min(max(root, 0.0), duration_s)


def _velocity_roots(c1: float, c2: float, c3: float, duration_s: float) -> tuple[float, ...]:
    """Interior roots of velocity ``c1 + 2*c2*s + 3*c3*s^2``."""
    scale = max(abs(c1), abs(c2), abs(c3))
    if scale == 0.0:
        return ()
    a = 3.0 * (c3 / scale)
    b = 2.0 * (c2 / scale)
    c = c1 / scale
    roots: list[float] = []
    for root in _quadratic_real_roots(a, b, c):
        bounded = _in_interval(root, duration_s)
        if bounded is not None:
            roots.append(bounded)
    return tuple(roots)


def _acceleration_root(c2: float, c3: float, duration_s: float) -> float | None:
    """Interior root of acceleration ``2*c2 + 6*c3*s``, if any."""
    if c3 == 0.0:
        return None
    ratio = c2 / c3
    root = -(ratio / 3.0)
    return _in_interval(root, duration_s)


def compute_reference_extremes(plan: ReferencePlan) -> ReferenceExtremes:
    """Compute continuous-interval extrema without applying joint limits.

    Position candidates are the velocity quadratic real roots plus endpoints.
    Velocity candidates are the acceleration linear root plus endpoints.
    Acceleration is linear per segment, so its extrema are its endpoints.
    """
    if not isinstance(plan, ReferencePlan):
        raise _invalid("plan", "must be a ReferencePlan")

    joint_count = plan.joint_count
    minimum = [math.inf] * joint_count
    maximum = [-math.inf] * joint_count
    maximum_velocity = [0.0] * joint_count
    maximum_acceleration = [0.0] * joint_count

    for segment_index, segment in enumerate(plan.segments):
        location = f"segments[{segment_index}]"
        duration_s = segment.duration_s
        for joint_index in range(joint_count):
            joint_location = f"{location}.c*[{joint_index}]"
            velocity_roots = _velocity_roots(
                segment.c1[joint_index],
                segment.c2[joint_index],
                segment.c3[joint_index],
                duration_s,
            )
            acceleration_root = _acceleration_root(
                segment.c2[joint_index],
                segment.c3[joint_index],
                duration_s,
            )
            candidates = [segment.c0[joint_index]]
            end_positions, _, _ = _state_at(segment, duration_s, joint_location)
            candidates.append(end_positions[joint_index])
            for root in velocity_roots:
                positions, _, _ = _state_at(segment, root, joint_location)
                candidates.append(positions[joint_index])
            segment_minimum = min(candidates)
            segment_maximum = max(candidates)
            minimum[joint_index] = min(minimum[joint_index], segment_minimum)
            maximum[joint_index] = max(maximum[joint_index], segment_maximum)

            velocity_candidates = [segment.c1[joint_index]]
            _, end_velocities, _ = _state_at(segment, duration_s, joint_location)
            velocity_candidates.append(end_velocities[joint_index])
            if acceleration_root is not None:
                _, velocities, _ = _state_at(segment, acceleration_root, joint_location)
                velocity_candidates.append(velocities[joint_index])
            segment_velocity = max(abs(value) for value in velocity_candidates)
            maximum_velocity[joint_index] = max(maximum_velocity[joint_index], segment_velocity)

            _, _, start_accelerations = _state_at(segment, 0.0, joint_location)
            _, _, end_accelerations = _state_at(segment, duration_s, joint_location)
            segment_acceleration = max(
                abs(start_accelerations[joint_index]),
                abs(end_accelerations[joint_index]),
            )
            maximum_acceleration[joint_index] = max(
                maximum_acceleration[joint_index], segment_acceleration
            )

    if not (
        all(math.isfinite(value) for value in minimum)
        and all(math.isfinite(value) for value in maximum)
        and all(math.isfinite(value) for value in maximum_velocity)
        and all(math.isfinite(value) for value in maximum_acceleration)
    ):
        raise _invalid("plan", "extrema are not finite")

    return ReferenceExtremes(
        tuple(
            JointExtremes(
                joint_name=plan.joint_names[index],
                minimum_position=minimum[index],
                maximum_position=maximum[index],
                maximum_velocity=maximum_velocity[index],
                maximum_acceleration=maximum_acceleration[index],
            )
            for index in range(joint_count)
        )
    )


def _check_vector_close(
    left: Sequence[float],
    right: Sequence[float],
    *,
    atol: float,
    location: str,
    quantity: str,
    joint_names: Sequence[str],
) -> None:
    for index, (left_value, right_value, name) in enumerate(
        zip(left, right, joint_names, strict=True)
    ):
        if not _is_close(left_value, right_value, atol=atol):
            raise _invalid(
                f"{location}[{index}]",
                f"joint {name!r} {quantity} is discontinuous: {left_value!r} != {right_value!r}",
            )


def verify_reference_plan(
    plan: ReferencePlan,
    initial_positions: Sequence[float],
    targets: Sequence[Target],
    joints: Sequence[JointSpec],
    max_accelerations: Sequence[float],
    *,
    initial_velocities: Sequence[float] | None = None,
    final_velocities: Sequence[float] | None = None,
) -> ReferenceExtremes:
    """Verify one explicit plan against inputs and all continuous limits.

    Checks joint dimensions/order, target ordering and horizon, declared
    position and velocity boundaries, internal position/velocity/acceleration
    continuity, and continuous position/velocity/acceleration limits.
    ``None`` endpoint velocities mean zero, preserving the old normal-plan
    contract; explicit values are compared with what the caller declared rather
    than with the plan's own derivative.  Returns the per-joint extrema used for
    the limit decisions.
    """
    if not isinstance(plan, ReferencePlan):
        raise _invalid("plan", "must be a ReferencePlan")
    inputs = validate_reference_inputs(
        initial_positions,
        targets,
        joints,
        max_accelerations,
        initial_velocities=initial_velocities,
        final_velocities=final_velocities,
    )
    if plan.joint_names != inputs.joint_names:
        raise _invalid("plan.joint_names", "must match the joint order passed to verification")
    if len(plan.segments) != len(inputs.target_times_ns):
        raise _invalid(
            "plan.segments",
            f"has {len(plan.segments)} segments for {len(inputs.target_times_ns)} targets",
        )
    if plan.duration_ns != inputs.target_times_ns[-1]:
        raise _invalid(
            "plan.segments",
            f"horizon {plan.duration_ns}ns differs from last target {inputs.target_times_ns[-1]}ns",
        )

    previous_end: tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]] | None = None
    elapsed_ns = 0
    for index, segment in enumerate(plan.segments):
        location = f"segments[{index}]"
        elapsed_ns += segment.duration_ns
        if elapsed_ns != inputs.target_times_ns[index]:
            raise _invalid(
                f"{location}.duration_ns",
                f"cumulative boundary {elapsed_ns}ns differs from target "
                f"{inputs.target_times_ns[index]}ns",
            )
        duration_s = segment.duration_s
        start_positions, start_velocities, start_accelerations = _state_at(segment, 0.0, location)
        end_positions, end_velocities, end_accelerations = _state_at(segment, duration_s, location)
        expected_start = (
            inputs.initial_positions if index == 0 else inputs.target_positions[index - 1]
        )
        expected_end = inputs.target_positions[index]
        _check_vector_close(
            start_positions,
            expected_start,
            atol=POSITION_ATOL,
            location=f"{location}.start_positions",
            quantity="position",
            joint_names=inputs.joint_names,
        )
        _check_vector_close(
            end_positions,
            expected_end,
            atol=POSITION_ATOL,
            location=f"{location}.end_positions",
            quantity="position",
            joint_names=inputs.joint_names,
        )
        if index == 0:
            _check_vector_close(
                start_velocities,
                inputs.initial_velocities,
                atol=VELOCITY_ATOL,
                location=f"{location}.start_velocities",
                quantity="velocity",
                joint_names=inputs.joint_names,
            )
        if index == len(plan.segments) - 1:
            _check_vector_close(
                end_velocities,
                inputs.final_velocities,
                atol=VELOCITY_ATOL,
                location=f"{location}.end_velocities",
                quantity="velocity",
                joint_names=inputs.joint_names,
            )
        if previous_end is not None:
            _check_vector_close(
                previous_end[0],
                start_positions,
                atol=POSITION_ATOL,
                location=f"{location}.start_positions",
                quantity="position",
                joint_names=inputs.joint_names,
            )
            _check_vector_close(
                previous_end[1],
                start_velocities,
                atol=VELOCITY_ATOL,
                location=f"{location}.start_velocities",
                quantity="velocity",
                joint_names=inputs.joint_names,
            )
            _check_vector_close(
                previous_end[2],
                start_accelerations,
                atol=ACCELERATION_ATOL,
                location=f"{location}.start_accelerations",
                quantity="acceleration",
                joint_names=inputs.joint_names,
            )
        previous_end = (end_positions, end_velocities, end_accelerations)

    extremes = compute_reference_extremes(plan)
    check_continuous_limits(extremes, inputs.joints, inputs.max_accelerations)
    return extremes


def check_continuous_limits(
    extremes: ReferenceExtremes,
    joints: Sequence[JointSpec],
    max_accelerations: Sequence[float],
) -> None:
    """Check continuous-interval extrema against per-joint declared limits.

    This is the shared limit decision used by both normal cubic plans and the
    later constant-acceleration braking plan.  It does not sample or refit a
    curve; callers must supply extrema over every continuous segment.  Limits
    are accepted exactly at the configured boundary with the same round-off
    allowance used by normal plan verification.
    """
    if not isinstance(extremes, ReferenceExtremes):
        raise _invalid("extremes", "must be a ReferenceExtremes")
    try:
        joint_items = tuple(joints)
        extreme_items = tuple(extremes.joints)
        acceleration_items = tuple(max_accelerations)
    except TypeError as error:
        raise _invalid("limits", "joints, extrema and accelerations must be arrays") from error
    if not 1 <= len(joint_items) <= MAX_JOINTS:
        raise _invalid("joints", f"must contain 1..{MAX_JOINTS} joints")
    if len(extreme_items) != len(joint_items) or len(acceleration_items) != len(joint_items):
        raise _invalid("extremes", "joint/extrema/acceleration lengths must match")

    seen_names: set[str] = set()
    for index, (joint_extremes, joint, max_acceleration) in enumerate(
        zip(extreme_items, joint_items, acceleration_items, strict=True)
    ):
        location = f"extremes.joints[{index}]"
        if not isinstance(joint, JointSpec):
            raise _invalid(f"joints[{index}]", "must be a JointSpec")
        if not isinstance(joint.name, str) or not joint.name or joint.name in seen_names:
            raise _invalid(f"joints[{index}].name", "must be a unique non-empty string")
        seen_names.add(joint.name)
        if not isinstance(joint_extremes, JointExtremes):
            raise _invalid(location, "must be a JointExtremes")
        if joint_extremes.joint_name != joint.name:
            raise _invalid(f"{location}.joint_name", "must match the declared joint order")
        lower = _finite_float(joint.minimum, f"joints[{index}].minimum")
        upper = _finite_float(joint.maximum, f"joints[{index}].maximum")
        velocity_limit = _finite_float(joint.max_velocity, f"joints[{index}].max_velocity")
        if lower > upper or velocity_limit <= 0.0:
            raise _invalid(f"joints[{index}]", "range must be ordered and velocity limit positive")
        acceleration = _finite_float(max_acceleration, f"max_accelerations[{index}]")
        if acceleration < 0.0:
            raise _invalid(f"max_accelerations[{index}]", "must not be negative")
        minimum = _finite_float(joint_extremes.minimum_position, f"{location}.minimum_position")
        maximum = _finite_float(joint_extremes.maximum_position, f"{location}.maximum_position")
        velocity = _finite_float(joint_extremes.maximum_velocity, f"{location}.maximum_velocity")
        peak_acceleration = _finite_float(
            joint_extremes.maximum_acceleration, f"{location}.maximum_acceleration"
        )
        if minimum > maximum or velocity < 0.0 or peak_acceleration < 0.0:
            raise _invalid(
                location, "position extrema must be ordered and absolute peaks nonnegative"
            )
        if minimum < lower - _limit_tolerance(lower):
            raise ContractError(
                "JOINT_LIMIT", f"joint {joint.name!r} minimum {minimum!r} is below {lower!r}"
            )
        if maximum > upper + _limit_tolerance(upper):
            raise ContractError(
                "JOINT_LIMIT", f"joint {joint.name!r} maximum {maximum!r} is above {upper!r}"
            )
        if velocity > velocity_limit + _limit_tolerance(velocity_limit):
            raise ContractError(
                "VELOCITY_LIMIT",
                f"joint {joint.name!r} velocity {velocity!r} exceeds {velocity_limit!r}",
            )
        if peak_acceleration > acceleration + _limit_tolerance(acceleration):
            raise ContractError(
                "ACCELERATION_LIMIT",
                f"joint {joint.name!r} acceleration {peak_acceleration!r} exceeds {acceleration!r}",
            )


__all__ = [
    "ACCELERATION_ATOL",
    "LIMIT_ATOL",
    "LIMIT_RTOL",
    "MAX_HORIZON_NS",
    "MAX_JOINTS",
    "MAX_SEGMENTS",
    "POSITION_ATOL",
    "RELATIVE_TOLERANCE",
    "VELOCITY_ATOL",
    "CubicSegment",
    "JointExtremes",
    "ReferenceExtremes",
    "ReferenceInputs",
    "ReferencePlan",
    "ReferenceSample",
    "check_continuous_limits",
    "compute_reference_extremes",
    "validate_reference_inputs",
    "validate_velocity_boundaries",
    "verify_reference_plan",
]
