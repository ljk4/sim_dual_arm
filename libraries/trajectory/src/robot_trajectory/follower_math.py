"""Pure terminal-speed and follower-deviation mathematics."""

from __future__ import annotations

import math
from collections.abc import Sequence
from numbers import Real

from robot_contracts.models import ContractError, JointSpec, Target
from robot_contracts.trajectory.spline import ReferencePlan

from .spline_math import (
    _quadratic_real_roots,
    validate_velocity_boundaries,
)

MAX_JOINTS = 64
MAX_HORIZON_NS = 3_000_000_000
_DEVIATION_ATOL = 1e-12


def _invalid(location: str, message: str) -> ContractError:
    return ContractError("INVALID_FOLLOWER_PLAN", f"{location}: {message}")


def _vector(value: object, location: str, length: int) -> tuple[float, ...]:
    try:
        raw = tuple(value)  # type: ignore[arg-type]
    except TypeError as error:
        raise _invalid(location, "must be an array of real numbers") from error
    if any(isinstance(item, bool) or not isinstance(item, Real) for item in raw):
        raise _invalid(location, "must contain real numbers")
    try:
        items = tuple(float(item) for item in raw)
    except (TypeError, ValueError, OverflowError) as error:
        raise _invalid(location, "cannot be represented as a finite float") from error
    if len(items) != length:
        raise _invalid(location, f"has {len(items)} values for {length} joints")
    if not all(math.isfinite(item) for item in items):
        raise _invalid(location, "must contain finite real numbers")
    return items


def estimate_raw_terminal_velocities(targets: Sequence[Target]) -> tuple[float, ...]:
    items = tuple(targets)
    if len(items) < 2:
        raise _invalid("targets", "must contain at least two points")
    last, previous = items[-1], items[-2]
    delta_ns = last.time_ns - previous.time_ns
    if delta_ns <= 0:
        raise _invalid("targets", "selected times are not strictly increasing")
    last_positions = _vector(last.positions, "targets[-1].positions", len(last.positions))
    previous_positions = _vector(
        previous.positions, "targets[-2].positions", len(last_positions))
    delta_s = delta_ns / 1_000_000_000
    return tuple((right - left) / delta_s
                 for left, right in zip(previous_positions, last_positions, strict=True))


def restrict_terminal_velocities(
    raw_terminal_velocities: Sequence[float], initial_positions: Sequence[float],
    initial_velocities: Sequence[float], target_positions: Sequence[float],
    joints: Sequence[JointSpec], max_accelerations: Sequence[float], *, is_final: bool,
) -> tuple[float, ...]:
    if not isinstance(is_final, bool):
        raise _invalid("is_final", "must be a bool")
    joint_items = tuple(joints)
    if not 1 <= len(joint_items) <= MAX_JOINTS:
        raise _invalid("joints", f"must contain 1..{MAX_JOINTS} joints")
    count = len(joint_items)
    raw = _vector(raw_terminal_velocities, "raw_terminal_velocities", count)
    initial = _vector(initial_positions, "initial_positions", count)
    goal = _vector(target_positions, "target_positions", count)
    accelerations = _vector(max_accelerations, "max_accelerations", count)
    initial_velocities = validate_velocity_boundaries(
        initial_velocities, joint_items, "initial_velocities")
    for index, acceleration in enumerate(accelerations):
        if acceleration <= 0.0:
            raise _invalid(f"max_accelerations[{index}]", "must be positive")
    if is_final:
        return (0.0,) * count
    restricted: list[float] = []
    for index, (joint, acceleration) in enumerate(zip(joint_items, accelerations, strict=True)):
        minimum, maximum = float(joint.minimum), float(joint.maximum)
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
        signed = direction * min(max(u_raw, 0.0), approach_cap, float(joint.max_velocity))
        if signed > 0.0:
            signed = min(signed, math.sqrt(max(0.0, 2.0 * acceleration * (maximum - goal[index]))))
        elif signed < 0.0:
            signed = max(signed, -math.sqrt(max(0.0, 2.0 * acceleration * (goal[index] - minimum))))
        if not math.isfinite(signed):
            raise _invalid(f"restricted_terminal_velocities[{index}]",
                           "generic terminal-speed cap is not finite")
        restricted.append(signed)
    return validate_velocity_boundaries(
        tuple(restricted), joint_items, "restricted_terminal_velocities")


_Piece = tuple[int, int, tuple[float, ...], tuple[float, ...]]


def _desired_pieces(initial_positions: Sequence[float], targets: Sequence[Target],
                    duration_ns: int, is_final: bool) -> tuple[_Piece, ...]:
    target_items = tuple(targets)
    count = len(initial_positions)
    points: list[tuple[int, tuple[float, ...]]] = [(0, tuple(initial_positions))]
    previous_time = 0
    for index, target in enumerate(target_items):
        if target.time_ns <= previous_time or target.time_ns > MAX_HORIZON_NS:
            raise _invalid(f"window.targets[{index}].time_ns", "must be strictly increasing")
        points.append((target.time_ns, _vector(
            target.positions, f"window.targets[{index}].positions", count)))
        previous_time = target.time_ns
    if duration_ns < previous_time:
        raise _invalid("plan.duration_ns", "horizon ends before the last target")
    if duration_ns > previous_time and not is_final:
        raise _invalid("plan.duration_ns", "a non-final plan must end at the hand-off")
    pieces = [(points[index - 1][0], points[index][0], points[index - 1][1],
               points[index][1]) for index in range(1, len(points))]
    if duration_ns > previous_time:
        pieces.append((previous_time, duration_ns, points[-1][1], points[-1][1]))
    if not pieces:
        raise _invalid("window.targets", "produced no desired interval")
    return tuple(pieces)


def _desired_value(piece: _Piece, time_ns: int, axis: int) -> float:
    fraction = (time_ns - piece[0]) / (piece[1] - piece[0])
    return piece[2][axis] + fraction * (piece[3][axis] - piece[2][axis])


def _in_closed_interval(value: float, start: float, end: float) -> float | None:
    if not math.isfinite(value):
        return None
    margin = 1e-12 * max(1.0, abs(start), abs(end))
    if value < start - margin or value > end + margin:
        return None
    return min(max(value, start), end)


def compute_follower_deviations(plan: ReferencePlan, initial_positions: Sequence[float],
                                targets: Sequence[Target], is_final: bool) -> tuple[float, ...]:
    initial = _vector(initial_positions, "initial_positions", plan.joint_count)
    pieces = _desired_pieces(initial, targets, plan.duration_ns, is_final)
    deviations = [0.0] * plan.joint_count
    reference_start_ns = 0
    for segment_index, segment in enumerate(plan.segments):
        reference_end_ns = reference_start_ns + segment.duration_ns
        for piece in pieces:
            overlap_start_ns = max(reference_start_ns, piece[0])
            overlap_end_ns = min(reference_end_ns, piece[1])
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
                a2, a3 = segment.c2[axis], segment.c3[axis]
                candidates = [local_start, local_end]
                for root in _quadratic_real_roots(3.0 * a3, 2.0 * a2, a1):
                    bounded = _in_closed_interval(root, local_start, local_end)
                    if bounded is not None:
                        candidates.append(bounded)
                for local_s in candidates:
                    value = ((a3 * local_s + a2) * local_s + a1) * local_s + a0
                    if not math.isfinite(value):
                        raise _invalid(f"segments[{segment_index}]",
                                       "target-deviation polynomial is not finite")
                    deviations[axis] = max(deviations[axis], abs(value))
        reference_start_ns = reference_end_ns
    return tuple(deviations)
