"""Shared kinematics, timing, and physics contact math helpers for simulation tasks."""

from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Any

from robot_contracts.models import ContractError

__all__ = [
    "contact_normal_force",
    "duration_from_limits",
    "matrix_as_tuple",
    "quaternion_wxyz_to_matrix",
    "rotation_from_axes",
    "rotation_z",
    "scalar_acceleration",
    "trapezoid_progress",
]


def rotation_z(radians_value: float) -> Any:
    """Return a 3x3 float64 rotation matrix around world +Z."""
    import numpy as np

    cosine, sine = math.cos(radians_value), math.sin(radians_value)
    return np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def rotation_from_axes(
    approach: tuple[float, float, float], closing: tuple[float, float, float]
) -> Any:
    """Build gripper TCP rotation matrix where local -Y approaches and +Z closes."""
    import numpy as np

    first = np.asarray(approach, dtype=np.float64)
    first /= np.linalg.norm(first)
    second = np.asarray(closing, dtype=np.float64)
    second -= first * float(np.dot(first, second))
    second /= np.linalg.norm(second)
    local_y = -first
    local_z = second
    local_x = np.cross(local_y, local_z)
    return np.column_stack([local_x, local_y, local_z])


def matrix_as_tuple(matrix: Any) -> tuple[tuple[float, float, float], ...]:
    """Convert a 2D array-like matrix into nested Python float tuples."""
    return tuple(tuple(float(value) for value in row) for row in matrix)


def duration_from_limits(
    start: Sequence[float],
    end: Sequence[float],
    velocities: Sequence[float],
    accelerations: Sequence[float],
    *,
    minimum_s: float,
) -> float:
    """Return the minimum feasible segment duration bounded by velocity and acceleration limits."""
    duration = float(minimum_s)
    for index, (first, second) in enumerate(zip(start, end, strict=True)):
        delta = abs(second - first)
        if delta <= 0.0:
            continue
        by_velocity = 1.5 * delta / velocities[index]
        by_acceleration = math.sqrt(6.0 * delta / accelerations[index])
        duration = max(duration, by_velocity, by_acceleration)
    return duration


def trapezoid_progress(elapsed_s: float, duration_s: float, acceleration: float) -> float:
    """Return normalized [0, 1] scalar progress along a trapezoidal velocity profile."""
    if duration_s <= 0.0 or elapsed_s <= 0.0:
        return 0.0
    if elapsed_s >= duration_s:
        return 1.0
    if acceleration <= 0.0:
        return 1.0
    product = acceleration * duration_s * duration_s
    if product < 4.0:
        raise ContractError(
            "TEACHER_PLAN_INVALID",
            f"step duration {duration_s:.3f} s is below the acceleration-limited minimum",
        )
    discriminant = acceleration * acceleration * duration_s * duration_s - 4.0 * acceleration
    cruise = (acceleration * duration_s - math.sqrt(max(0.0, discriminant))) / 2.0
    if cruise <= 0.0:
        return min(1.0, elapsed_s / duration_s)
    accelerate_s = cruise / acceleration
    accelerate_distance = 0.5 * acceleration * accelerate_s * accelerate_s
    cruise_s = max(0.0, (1.0 - 2.0 * accelerate_distance) / cruise)
    if elapsed_s < accelerate_s:
        return 0.5 * acceleration * elapsed_s * elapsed_s
    if elapsed_s < accelerate_s + cruise_s:
        return accelerate_distance + cruise * (elapsed_s - accelerate_s)
    return 1.0 - 0.5 * acceleration * (duration_s - elapsed_s) ** 2


def scalar_acceleration(
    start_positions: Sequence[float],
    end_positions: Sequence[float],
    accelerations: Sequence[float],
) -> float:
    """Return the normalized scalar acceleration bound for a linear joint segment."""
    acceleration = math.inf
    for first, last, limit in zip(start_positions, end_positions, accelerations, strict=True):
        delta = abs(last - first)
        if delta <= 0.0:
            continue
        acceleration = min(acceleration, limit / delta)
    if not math.isfinite(acceleration) or acceleration <= 0.0:
        return 0.0
    return acceleration


def quaternion_wxyz_to_matrix(
    quat_wxyz: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]:
    """Convert a unit (w, x, y, z) quaternion to a 3x3 rotation matrix tuple."""
    w, x, y, z = quat_wxyz
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def contact_normal_force(model: Any, data: Any, contact_index: int) -> float:
    """Read the normal contact force magnitude for a single MuJoCo contact index."""
    import mujoco
    import numpy as np

    value = np.zeros(6, dtype=float)
    mujoco.mj_contactForce(model, data, contact_index, value)
    force = float(value[0])
    if not math.isfinite(force):
        raise ContractError("INVALID_CONTACT", "contact normal force is not finite")
    return abs(force)
