"""Deterministic C1 follower curves from bounded one-axis velocity profiles."""

from __future__ import annotations

import math
from dataclasses import dataclass
from itertools import pairwise

from robot_contracts.models import ContractError, JointSpec, RobotProfile
from robot_contracts.stream_request import StreamRequest
from robot_contracts.trajectory.constants import MAX_SEGMENTS
from robot_contracts.trajectory.follower_reference import FollowerValidationLimits
from robot_contracts.trajectory.spline import CubicSegment, ReferencePlan

from .follower_math import estimate_raw_terminal_velocities, restrict_terminal_velocities
from .spline_math import _limit_tolerance, validate_reference_inputs

_NS_PER_S = 1_000_000_000
_DISTANCE_TOLERANCE = 1e-12


@dataclass(frozen=True, slots=True)
class _Axis:
    joint: JointSpec
    start_position: float
    start_speed: float
    target_position: float
    target_speed: float
    acceleration: float


# Times in nanoseconds and velocities at those breakpoints. Position is the
# integral of the linearly interpolated velocity.
_VelocityKnots = tuple[tuple[int, float], ...]


def _switches(axis: _Axis, duration: float, plateau: float) -> tuple[float, float, float]:
    """Return the start/end of cruise and its speed, or a single triangular peak."""
    start, end, acceleration = axis.start_speed, axis.target_speed, axis.acceleration
    ramp_up = abs(plateau - start) / acceleration
    ramp_down = abs(plateau - end) / acceleration
    if ramp_up + ramp_down <= duration:
        return ramp_up, duration - ramp_down, plateau
    # Overlapping ramps meet before reaching the requested cruise velocity.
    slope = math.copysign(acceleration, plateau - start)
    switch = min(duration, max(0.0, (duration + (end - start) / slope) * 0.5))
    return switch, switch, start + slope * switch


def _displacement(axis: _Axis, duration: float, plateau: float) -> float:
    left, right, speed = _switches(axis, duration, plateau)
    return (
        (axis.start_speed + speed) * left * 0.5
        + speed * (right - left)
        + (speed + axis.target_speed) * (duration - right) * 0.5
    )


def _knots(axis: _Axis, duration_ns: int, plateau: float) -> _VelocityKnots:
    duration = duration_ns / _NS_PER_S
    left, right, _ = _switches(axis, duration, plateau)
    times = sorted(
        {
            boundary
            for switch in (left, right)
            for boundary in (math.floor(switch * _NS_PER_S), math.ceil(switch * _NS_PER_S))
            if 0 < boundary < duration_ns
        }
    )
    knots = [(0, axis.start_speed)]
    # Include the grid points on both sides of each switch so approximating
    # the velocity kink only affects the enclosing one-nanosecond interval.
    # Chords of the velocity envelope preserve the acceleration limit.
    for time_ns in times:
        elapsed = time_ns / _NS_PER_S
        remaining = (duration_ns - time_ns) / _NS_PER_S
        lower = max(
            axis.start_speed - axis.acceleration * elapsed,
            axis.target_speed - axis.acceleration * remaining,
        )
        upper = min(
            axis.start_speed + axis.acceleration * elapsed,
            axis.target_speed + axis.acceleration * remaining,
        )
        knots.append((time_ns, min(max(plateau, lower), upper)))
    knots.append((duration_ns, axis.target_speed))
    return tuple(knots)


def _within_positions(axis: _Axis, knots: _VelocityKnots) -> bool:
    position = axis.start_position
    lower = float(axis.joint.minimum)
    upper = float(axis.joint.maximum)
    lower -= _limit_tolerance(lower)
    upper += _limit_tolerance(upper)
    for (start_ns, start), (end_ns, end) in pairwise(knots):
        duration = (end_ns - start_ns) / _NS_PER_S
        if start * end < 0.0:
            zero_at = duration * abs(start) / (abs(start) + abs(end))
            if not lower <= position + 0.5 * start * zero_at <= upper:
                return False
        position += 0.5 * (start + end) * duration
        if not lower <= position <= upper:
            return False
    return True


def _solve_axis(axis: _Axis, duration_ns: int) -> _VelocityKnots | None:
    duration = duration_ns / _NS_PER_S
    if abs(axis.target_speed - axis.start_speed) > (
        axis.acceleration * duration + _DISTANCE_TOLERANCE
    ):
        return None
    distance = axis.target_position - axis.start_position
    speed = float(axis.joint.max_velocity)
    low, high = -speed, speed
    if not (
        _displacement(axis, duration, low) - _DISTANCE_TOLERANCE
        <= distance
        <= _displacement(axis, duration, high) + _DISTANCE_TOLERANCE
    ):
        return None
    plateau = min(high, max(low, distance / duration))
    for _ in range(48):
        error = _displacement(axis, duration, plateau) - distance
        if abs(error) <= _DISTANCE_TOLERANCE:
            break
        if error < 0.0:
            low = plateau
        else:
            high = plateau
        plateau = 0.5 * (low + high)
    knots = _knots(axis, duration_ns, plateau)
    return knots if _within_positions(axis, knots) else None


def _find_axis_curve(axis: _Axis, minimum_ns: int, maximum_ns: int) -> _VelocityKnots:
    curve = _solve_axis(axis, minimum_ns)
    if curve is not None:
        return curve
    last_ns = minimum_ns
    step_ns = max(1_000_000, minimum_ns // 16)
    while last_ns < maximum_ns:
        candidate_ns = min(maximum_ns, last_ns + step_ns)
        curve = _solve_axis(axis, candidate_ns)
        if curve is not None:
            # Refine the first feasible interval to millisecond precision.
            while candidate_ns - last_ns > 1_000_000:
                middle_ns = (last_ns + candidate_ns) // 2
                middle = _solve_axis(axis, middle_ns)
                if middle is None:
                    last_ns = middle_ns
                else:
                    candidate_ns, curve = middle_ns, middle
            return curve
        last_ns = candidate_ns
        step_ns *= 2
    raise ContractError(
        "FOLLOWER_INFEASIBLE",
        f"joint {axis.joint.name!r} has no bounded curve within {maximum_ns}ns",
    )


def _emit_plan(
    axes: tuple[_Axis, ...], curves: tuple[_VelocityKnots, ...], duration_ns: int
) -> ReferencePlan:
    boundaries = sorted(
        {0, duration_ns}.union(
            time_ns for curve in curves for time_ns, _ in curve if time_ns < duration_ns
        )
    )
    if len(boundaries) > MAX_SEGMENTS + 1:
        raise ContractError("FOLLOWER_INFEASIBLE", "merged curve exceeds the segment budget")
    indices = [0] * len(axes)
    positions = tuple(axis.start_position for axis in axes)
    zero = (0.0,) * len(axes)
    segments = []
    for start_ns, end_ns in pairwise(boundaries):
        elapsed = (end_ns - start_ns) / _NS_PER_S
        speeds = []
        half_accelerations = []
        for axis_index, (axis, knots) in enumerate(zip(axes, curves, strict=True)):
            index = indices[axis_index]
            if index + 1 < len(knots) and start_ns == knots[index + 1][0]:
                index += 1
                indices[axis_index] = index
            knot_ns, speed = knots[index]
            if index + 1 < len(knots):
                next_ns, next_speed = knots[index + 1]
                slope = (next_speed - speed) * _NS_PER_S / (next_ns - knot_ns)
                # A chord slope can exceed the analytic bound by rounding only.
                slope = min(axis.acceleration, max(-axis.acceleration, slope))
            else:
                slope = 0.0
            speeds.append(speed + slope * (start_ns - knot_ns) / _NS_PER_S)
            half_accelerations.append(0.5 * slope)
        segment = CubicSegment(end_ns - start_ns, positions, speeds, half_accelerations, zero)
        segments.append(segment)
        positions = tuple(
            p + (v + a * elapsed) * elapsed
            for p, v, a in zip(positions, speeds, half_accelerations, strict=True)
        )
    return ReferencePlan(tuple(axis.joint.name for axis in axes), tuple(segments))


def generate_follower_plan(
    request: StreamRequest,
    profile: RobotProfile,
    limits: FollowerValidationLimits,
) -> ReferencePlan:
    """Generate an anchored, velocity/acceleration-bounded polynomial reference.

    Infeasible fixed-duration endpoints may use the prefix of a longer bounded
    curve. The exported non-final hand-off never moves. A final curve reaches
    the last target at rest, with shorter axes holding through the common tail.
    """
    anchor, window = request.anchor, request.window
    if anchor is None or window is None or request.target_count != limits.target_count:
        raise ContractError("INVALID_FOLLOWER_REQUEST", "expected a configured stream window")
    if (
        request.trajectory.joint_names != limits.joint_names
        or profile.joint_names != limits.joint_names
        or anchor.robot_profile_hash != profile.digest
    ):
        raise ContractError("PROFILE_MISMATCH", "follower request, profile and limits differ")
    is_final = request.settle_at_end
    duration_ns = window.end_time_ns
    grid_ns = limits.duration_grid_ns or 1
    maximum_ns = limits.max_terminal_tail_duration_ns
    if is_final:
        duration_ns = (duration_ns + grid_ns - 1) // grid_ns * grid_ns
        maximum_ns = maximum_ns // grid_ns * grid_ns
    elif duration_ns % grid_ns:
        raise ContractError("FOLLOWER_GRID_MISMATCH", "hand-off must lie on the duration grid")
    if duration_ns > maximum_ns:
        raise ContractError("FOLLOWER_HORIZON_EXCEEDED", "window exceeds follower horizon")

    targets = window.targets
    inputs = validate_reference_inputs(
        anchor.reference.positions,
        window.source_targets,
        profile.joints,
        limits.max_accelerations,
        initial_velocities=anchor.reference.velocities,
    )
    target_position = targets[-1].positions
    target_speed = restrict_terminal_velocities(
        estimate_raw_terminal_velocities(targets),
        inputs.initial_positions,
        inputs.initial_velocities,
        target_position,
        inputs.joints,
        inputs.max_accelerations,
        is_final=is_final,
    )
    axes = tuple(
        _Axis(*values)
        for values in zip(
            inputs.joints,
            inputs.initial_positions,
            inputs.initial_velocities,
            target_position,
            target_speed,
            inputs.max_accelerations,
            strict=True,
        )
    )
    curves = tuple(_find_axis_curve(axis, duration_ns, maximum_ns) for axis in axes)
    if is_final:
        duration_ns = max(curve[-1][0] for curve in curves)
        duration_ns = (duration_ns + grid_ns - 1) // grid_ns * grid_ns
    return _emit_plan(axes, curves, duration_ns)
