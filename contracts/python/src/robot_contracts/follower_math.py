"""Pure bounded-follower terminal-speed rules and continuous deviation math.

The functions have no controller, ROS or numerical-library dependency.
Follower record validation and immutable DTOs live in follower.py.
"""

from __future__ import annotations

import math
import sys
from collections.abc import Sequence
from dataclasses import dataclass
from numbers import Integral, Real

from .models import ContractError, JointSpec, Target
from .reference import (
    LIMIT_ATOL,
    LIMIT_RTOL,
    MAX_HORIZON_NS,
    MAX_JOINTS,
    CubicSegment,
    ReferencePlan,
    validate_velocity_boundaries,
)

_EPSILON = sys.float_info.epsilon
_DEVIATION_ATOL = 1e-12


def _invalid(location: str, message: str) -> ContractError:
    return ContractError("INVALID_FOLLOWER_PLAN", f"{location}: {message}")


def _finite(value: object, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise _invalid(location, "must be a finite real number")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise _invalid(location, "cannot be represented as a finite float") from error
    if not math.isfinite(result):
        raise _invalid(location, "must be finite")
    return result


def _integer(value: object, location: str, *, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise _invalid(location, "must be an integer")
    result = int(value)
    if result < minimum or result > maximum:
        raise _invalid(location, f"must be in {minimum}..{maximum}")
    return result


def _vector(value: object, location: str, length: int) -> tuple[float, ...]:
    if isinstance(value, str | bytes):
        raise _invalid(location, "must be an array of real numbers")
    try:
        items = tuple(value)  # type: ignore[arg-type]
    except TypeError as error:
        raise _invalid(location, "must be an array of real numbers") from error
    if len(items) != length:
        raise _invalid(location, f"has {len(items)} values for {length} joints")
    return tuple(_finite(item, f"{location}[{index}]") for index, item in enumerate(items))


def _position_vector(value: object, location: str) -> tuple[float, ...]:
    if isinstance(value, str | bytes):
        raise _invalid(location, "must be an array of real numbers")
    try:
        items = tuple(value)  # type: ignore[arg-type]
    except TypeError as error:
        raise _invalid(location, "must be an array of real numbers") from error
    if not 1 <= len(items) <= MAX_JOINTS:
        raise _invalid(location, f"must contain 1..{MAX_JOINTS} values")
    return tuple(_finite(item, f"{location}[{index}]") for index, item in enumerate(items))


def _limit_tolerance(bound: float) -> float:
    return LIMIT_ATOL + LIMIT_RTOL * abs(bound)


@dataclass(frozen=True)
class NormalizedFollowerPositions:
    """One position sample with an explicit, auditable hard-boundary correction.

    ``sampled_positions`` are the untouched coefficient samples.
    ``applied_positions`` are always inside each joint's configured hard range.
    ``corrections`` satisfy ``applied = sampled - correction`` per axis.  A
    correction is nonzero only when the sampled value exceeded a hard boundary
    by at most the shared ``LIMIT_ATOL + LIMIT_RTOL * |bound|`` round-off
    allowance.  Velocity and acceleration samples are not altered: they are not
    written by ``PandaIO.apply_targets`` and remain governed by the shared
    continuous-limit checks.
    """

    sampled_positions: tuple[float, ...]
    applied_positions: tuple[float, ...]
    corrections: tuple[float, ...]

    @property
    def has_correction(self) -> bool:
        return any(value != 0.0 for value in self.corrections)


def normalize_follower_positions(
    positions: Sequence[float],
    joints: Sequence[JointSpec],
    location: str = "reference.positions",
) -> NormalizedFollowerPositions:
    """Move a follower position sample onto its hard boundary within round-off.

    This is the single shared follower/audit definition of the numeric
    normalization.  It never clips a meaningful prediction error: a sampled
    value outside the configured hard range by more than the accepted shared
    limit tolerance raises ``FOLLOWER_BOUNDARY_MISMATCH``.  A value already in
    range is returned unchanged with a zero correction.
    """
    try:
        joint_items = tuple(joints)
    except TypeError as error:
        raise _invalid("joints", "must be an array of JointSpec values") from error
    if not 1 <= len(joint_items) <= MAX_JOINTS:
        raise _invalid("joints", f"must contain 1..{MAX_JOINTS} joints")
    sampled = _vector(positions, location, len(joint_items))
    applied: list[float] = []
    corrections: list[float] = []
    for index, (position, joint) in enumerate(zip(sampled, joint_items, strict=True)):
        if not isinstance(joint, JointSpec):
            raise _invalid(f"joints[{index}]", "must be a JointSpec")
        minimum = _finite(joint.minimum, f"joints[{index}].minimum")
        maximum = _finite(joint.maximum, f"joints[{index}].maximum")
        if minimum > maximum:
            raise _invalid(f"joints[{index}]", "minimum must not exceed maximum")
        if minimum <= position <= maximum:
            corrected = position
        elif position < minimum:
            if position < minimum - _limit_tolerance(minimum):
                raise ContractError(
                    "FOLLOWER_BOUNDARY_MISMATCH",
                    f"{location}[{index}] ({joint.name}) {position!r} is below the "
                    f"hard minimum {minimum!r} by more than the shared round-off tolerance",
                )
            corrected = minimum
        else:
            if position > maximum + _limit_tolerance(maximum):
                raise ContractError(
                    "FOLLOWER_BOUNDARY_MISMATCH",
                    f"{location}[{index}] ({joint.name}) {position!r} is above the "
                    f"hard maximum {maximum!r} by more than the shared round-off tolerance",
                )
            corrected = maximum
        applied.append(corrected)
        corrections.append(position - corrected)
    return NormalizedFollowerPositions(
        sampled_positions=sampled,
        applied_positions=tuple(applied),
        corrections=tuple(corrections),
    )


def estimate_raw_terminal_velocities(targets: Sequence[Target]) -> tuple[float, ...]:
    """Backward difference of the last two selected raw targets, in per-second units.

    This is the single pure-standard-library definition used both by the backend
    adapter and by :class:`PreparedFollowerPlan` when it independently checks its
    stored metadata.
    """
    try:
        items = tuple(targets)
    except TypeError as error:
        raise _invalid("targets", "must be an array of Target values") from error
    if len(items) < 2:
        raise _invalid("targets", "must contain at least two points")
    last, previous = items[-1], items[-2]
    if not isinstance(last, Target):
        raise _invalid("targets[-1]", "must be a Target")
    if not isinstance(previous, Target):
        raise _invalid("targets[-2]", "must be a Target")
    last_time_ns = _integer(
        last.time_ns,
        "targets[-1].time_ns",
        minimum=1,
        maximum=MAX_HORIZON_NS,
    )
    previous_time_ns = _integer(
        previous.time_ns,
        "targets[-2].time_ns",
        minimum=1,
        maximum=MAX_HORIZON_NS,
    )
    delta_ns = last_time_ns - previous_time_ns
    if delta_ns <= 0:
        raise _invalid("targets", "selected times are not strictly increasing")
    last_positions = _position_vector(last.positions, "targets[-1].positions")
    previous_positions = _position_vector(previous.positions, "targets[-2].positions")
    if len(last_positions) != len(previous_positions):
        raise _invalid("targets", "last two target dimensions differ")
    delta_s = delta_ns / 1_000_000_000
    return tuple(
        (right - left) / delta_s
        for left, right in zip(previous_positions, last_positions, strict=True)
    )


def restrict_terminal_velocities(
    raw_terminal_velocities: Sequence[float],
    initial_positions: Sequence[float],
    initial_velocities: Sequence[float],
    target_positions: Sequence[float],
    joints: Sequence[JointSpec],
    max_accelerations: Sequence[float],
    *,
    is_final: bool,
) -> tuple[float, ...]:
    """Apply the one generic terminal-speed rule used by every adapter.

    The smaller of two safe magnitudes is used in the displacement direction:

    * target-approach braking feasibility
      ``sqrt(max(0, u0^2 + 2*a*d))`` for non-negative ``u0`` and
      ``sqrt(max(0, 2*a*d - u0^2))`` otherwise;
    * the actual joint position hard boundary at the goal
      ``sqrt(2*a*(max - goal))`` for a positive terminal velocity and
      ``sqrt(2*a*(goal - min))`` for a negative one.

    A final plan is always zero.  A terminal speed opposing the total
    displacement is reduced to zero.  Raw values are never overwritten; this
    function returns the separate restricted vector.
    """
    if not isinstance(is_final, bool):
        raise _invalid("is_final", "must be a bool")
    try:
        joint_items = tuple(joints)
    except TypeError as error:
        raise _invalid("joints", "must be an array of JointSpec values") from error
    if not 1 <= len(joint_items) <= MAX_JOINTS:
        raise _invalid("joints", f"must contain 1..{MAX_JOINTS} joints")
    count = len(joint_items)
    raw = _vector(raw_terminal_velocities, "raw_terminal_velocities", count)
    initial = _vector(initial_positions, "initial_positions", count)
    goal = _vector(target_positions, "target_positions", count)
    accelerations = _vector(max_accelerations, "max_accelerations", count)
    initial_velocities = validate_velocity_boundaries(
        initial_velocities,
        joint_items,
        "initial_velocities",
    )
    for index, (joint, acceleration) in enumerate(zip(joint_items, accelerations, strict=True)):
        if not isinstance(joint, JointSpec):
            raise _invalid(f"joints[{index}]", "must be a JointSpec")
        minimum = _finite(joint.minimum, f"joints[{index}].minimum")
        maximum = _finite(joint.maximum, f"joints[{index}].maximum")
        max_velocity = _finite(joint.max_velocity, f"joints[{index}].max_velocity")
        if minimum > maximum:
            raise _invalid(f"joints[{index}]", "minimum must not exceed maximum")
        if max_velocity <= 0.0:
            raise _invalid(f"joints[{index}].max_velocity", "must be positive")
        if acceleration <= 0.0:
            raise _invalid(f"max_accelerations[{index}]", "must be positive")
        if goal[index] < minimum - _limit_tolerance(minimum):
            raise ContractError(
                "JOINT_LIMIT",
                f"target_positions[{index}] ({joint.name}) {goal[index]!r} is below {minimum!r}",
            )
        if goal[index] > maximum + _limit_tolerance(maximum):
            raise ContractError(
                "JOINT_LIMIT",
                f"target_positions[{index}] ({joint.name}) {goal[index]!r} is above {maximum!r}",
            )

    if is_final:
        return (0.0,) * count

    restricted: list[float] = []
    for index, (joint, acceleration) in enumerate(zip(joint_items, accelerations, strict=True)):
        minimum = float(joint.minimum)
        maximum = float(joint.maximum)
        maximum_velocity = float(joint.max_velocity)
        displacement = goal[index] - initial[index]
        if displacement == 0.0:
            restricted.append(0.0)
            continue
        direction = 1.0 if displacement > 0.0 else -1.0
        distance = abs(displacement)
        u0 = direction * initial_velocities[index]
        u_raw = direction * raw[index]
        if u0 >= 0.0:
            approach_cap = math.sqrt(max(0.0, u0 * u0 + 2.0 * acceleration * distance))
        else:
            approach_cap = math.sqrt(max(0.0, 2.0 * acceleration * distance - u0 * u0))
        speed = min(max(u_raw, 0.0), approach_cap, maximum_velocity)
        signed = direction * speed
        if signed > 0.0:
            position_cap = math.sqrt(max(0.0, 2.0 * acceleration * (maximum - goal[index])))
            signed = min(signed, position_cap)
        elif signed < 0.0:
            position_cap = math.sqrt(max(0.0, 2.0 * acceleration * (goal[index] - minimum)))
            signed = max(signed, -position_cap)
        if not math.isfinite(signed):
            raise _invalid(
                f"restricted_terminal_velocities[{index}]",
                "generic terminal-speed cap is not finite",
            )
        restricted.append(signed)
    return validate_velocity_boundaries(
        tuple(restricted),
        joint_items,
        "restricted_terminal_velocities",
    )


def _segment_state(
    segment: object, local_s: float, location: str
) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
    if not isinstance(segment, CubicSegment):
        raise _invalid(location, "must be a CubicSegment")
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
            raise _invalid(f"{location}.c*[{index}]", "polynomial evaluation is not finite")
        positions.append(position)
        velocities.append(velocity)
        accelerations.append(acceleration)
    return tuple(positions), tuple(velocities), tuple(accelerations)


def _quadratic_real_roots(a: float, b: float, c: float) -> tuple[float, ...]:
    """Stable real roots of ``a*x^2 + b*x + c`` for the cubic derivative."""
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


def _in_closed_interval(value: float, start: float, end: float) -> float | None:
    if not math.isfinite(value):
        return None
    margin = 1e-12 * max(1.0, abs(start), abs(end))
    if value < start - margin or value > end + margin:
        return None
    return min(max(value, start), end)


@dataclass(frozen=True)
class _DesiredPiece:
    start_ns: int
    end_ns: int
    start_positions: tuple[float, ...]
    end_positions: tuple[float, ...]


def _desired_pieces(
    initial_positions: Sequence[float],
    targets: Sequence[Target],
    duration_ns: int,
    is_final: bool,
) -> tuple[_DesiredPiece, ...]:
    try:
        target_items = tuple(targets)
    except TypeError as error:
        raise _invalid("window.targets", "must be an array of Target values") from error
    if not target_items:
        raise _invalid("window.targets", "must not be empty")
    joint_count = len(initial_positions)
    points: list[tuple[int, tuple[float, ...]]] = [(0, tuple(initial_positions))]
    previous_time = 0
    for index, target in enumerate(target_items):
        if not isinstance(target, Target):
            raise _invalid(f"window.targets[{index}]", "must be a Target")
        time_ns = _integer(
            target.time_ns,
            f"window.targets[{index}].time_ns",
            minimum=1,
            maximum=MAX_HORIZON_NS,
        )
        if time_ns <= previous_time:
            raise _invalid(
                f"window.targets[{index}].time_ns",
                "must be strictly increasing",
            )
        positions = _vector(
            target.positions,
            f"window.targets[{index}].positions",
            joint_count,
        )
        points.append((time_ns, positions))
        previous_time = time_ns
    if duration_ns < previous_time:
        raise _invalid(
            "plan.duration_ns",
            f"horizon {duration_ns}ns ends before the last target {previous_time}ns",
        )
    if duration_ns > previous_time and not is_final:
        raise _invalid(
            "plan.duration_ns",
            "a non-final follower plan must end exactly at the fixed hand-off",
        )

    pieces: list[_DesiredPiece] = []
    for index in range(1, len(points)):
        start_ns, start_positions = points[index - 1]
        end_ns, end_positions = points[index]
        pieces.append(_DesiredPiece(start_ns, end_ns, start_positions, end_positions))
    if duration_ns > previous_time:
        pieces.append(_DesiredPiece(previous_time, duration_ns, points[-1][1], points[-1][1]))
    if not pieces:
        raise _invalid("window.targets", "produced no desired interval")
    return tuple(pieces)


def _desired_value(piece: _DesiredPiece, time_ns: int, axis: int) -> float:
    duration = piece.end_ns - piece.start_ns
    if duration <= 0:
        raise AssertionError("desired pieces must have positive duration")
    fraction = (time_ns - piece.start_ns) / duration
    return piece.start_positions[axis] + fraction * (
        piece.end_positions[axis] - piece.start_positions[axis]
    )


def compute_follower_deviations(
    plan: ReferencePlan,
    initial_positions: Sequence[float],
    targets: Sequence[Target],
    is_final: bool,
) -> tuple[float, ...]:
    """Return the exact continuous maximum ``|reference - desired|`` per axis.

    The desired reference is the piecewise-linear path from ``initial_positions``
    at elapsed zero through ``targets``.  For a final plan the last value is held
    on any extension.  On each intersection the difference is a cubic; its
    derivative is solved analytically, so the result is not a control-tick
    sample.
    """
    if not isinstance(plan, ReferencePlan):
        raise _invalid("plan", "must be a ReferencePlan")
    if not isinstance(is_final, bool):
        raise _invalid("is_final", "must be a bool")
    initial = _vector(initial_positions, "initial_positions", plan.joint_count)
    pieces = _desired_pieces(initial, targets, plan.duration_ns, is_final)
    deviations = [0.0] * plan.joint_count
    reference_start_ns = 0
    for segment_index, segment in enumerate(plan.segments):
        reference_end_ns = reference_start_ns + segment.duration_ns
        for piece in pieces:
            overlap_start_ns = max(reference_start_ns, piece.start_ns)
            overlap_end_ns = min(reference_end_ns, piece.end_ns)
            if overlap_end_ns <= overlap_start_ns:
                continue
            local_start = (overlap_start_ns - reference_start_ns) / 1_000_000_000
            local_end = (overlap_end_ns - reference_start_ns) / 1_000_000_000
            for axis in range(plan.joint_count):
                desired_start = _desired_value(piece, overlap_start_ns, axis)
                desired_end = _desired_value(piece, overlap_end_ns, axis)
                slope = (desired_end - desired_start) / (local_end - local_start)
                a0 = segment.c0[axis] - desired_start + slope * local_start
                a1 = segment.c1[axis] - slope
                a2 = segment.c2[axis]
                a3 = segment.c3[axis]
                candidates = [local_start, local_end]
                for root in _quadratic_real_roots(3.0 * a3, 2.0 * a2, a1):
                    bounded = _in_closed_interval(root, local_start, local_end)
                    if bounded is not None:
                        candidates.append(bounded)
                for local_s in candidates:
                    value = ((a3 * local_s + a2) * local_s + a1) * local_s + a0
                    if not math.isfinite(value):
                        raise _invalid(
                            f"segments[{segment_index}]",
                            "target-deviation polynomial is not finite",
                        )
                    deviations[axis] = max(deviations[axis], abs(value))
        reference_start_ns = reference_end_ns
    return tuple(deviations)
