"""Robot- and task-agnostic key-pose and joint-path primitives.

These helpers only validate tuples and perform arithmetic; they import neither
MuJoCo nor a robot-specific configuration.  Panda/Marvin task planners and the
Panda key-pose checker share them so generic trajectory sampling never depends
on a robot assembly.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from robot_backend_sim.common.kinematics.kinematics_clearance import KinematicsError

__all__ = [
    "ObjectPose",
    "extract_object_pose",
    "finite_float",
    "rotation_down",
    "sample_joint_path",
    "vector",
]


@dataclass(frozen=True, slots=True)
class ObjectPose:
    name: str
    center: tuple[float, float, float]
    yaw_rad: float


def finite_float(value: object, field: str) -> float:
    """Return a finite float, rejecting booleans and non-numbers."""
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise KinematicsError("target_invalid", f"{field} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise KinematicsError("target_invalid", f"{field} must be finite")
    return result


def vector(value: object, length: int, field: str) -> tuple[float, ...]:
    """Return one fixed-length tuple of finite floats."""
    if not isinstance(value, tuple | list) or len(value) != length:
        raise KinematicsError("target_invalid", f"{field} must contain {length} values")
    return tuple(finite_float(item, f"{field}[{index}]") for index, item in enumerate(value))


def rotation_down(yaw_rad: float) -> tuple[tuple[float, float, float], ...]:
    """Return ``Rz(yaw) @ diag(1, -1, -1)`` as an explicit row-major matrix."""
    yaw = finite_float(yaw_rad, "yaw_rad")
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        (cosine, sine, 0.0),
        (sine, -cosine, 0.0),
        (0.0, 0.0, -1.0),
    )


def extract_object_pose(qpos: object, address: int, name: str) -> ObjectPose:
    """Read the initial free-joint centre and yaw from one qpos vector."""
    if isinstance(address, bool) or not isinstance(address, int) or address < 0:
        raise KinematicsError("initial_state_invalid", "object qpos address is invalid")
    try:
        values = [float(qpos[address + index]) for index in range(7)]  # type: ignore[index]
    except (IndexError, TypeError, ValueError) as error:
        raise KinematicsError(
            "initial_state_invalid", f"object {name!r} qpos slice is not readable"
        ) from error
    if not all(math.isfinite(value) for value in values):
        raise KinematicsError(
            "initial_state_invalid", f"object {name!r} qpos contains a non-finite value"
        )
    w, x, y, z = values[3], values[4], values[5], values[6]
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-9:
        raise KinematicsError("initial_state_invalid", f"object {name!r} quaternion has zero norm")
    if abs(norm - 1.0) > 1e-3:
        raise KinematicsError(
            "initial_state_invalid", f"object {name!r} quaternion is not unit length"
        )
    yaw = 2.0 * math.atan2(z, w)
    return ObjectPose(name=name, center=(values[0], values[1], values[2]), yaw_rad=yaw)


def sample_joint_path(
    start: object,
    end: object,
    *,
    max_step_rad: float,
    max_samples: int,
) -> tuple[tuple[float, ...], ...]:
    """Sample a linear joint-space path, including both endpoints."""
    step = finite_float(max_step_rad, "max_step_rad")
    if step <= 0.0:
        raise KinematicsError("path_sample_invalid", "max_step_rad must be positive")
    if isinstance(max_samples, bool) or not isinstance(max_samples, int) or max_samples < 2:
        raise KinematicsError("path_sample_invalid", "max_samples must be an integer >= 2")
    start_values = vector(start, len(start), "start")  # type: ignore[arg-type]
    end_values = vector(end, len(end), "end")  # type: ignore[arg-type]
    if len(start_values) != len(end_values) or not start_values:
        raise KinematicsError(
            "path_sample_invalid", "start and end must have the same non-zero length"
        )
    deltas = [abs(second - first) for first, second in zip(start_values, end_values, strict=True)]
    maximum = max(deltas)
    count = max(2, math.ceil(maximum / step - 1e-12) + 1)
    if count > max_samples:
        raise KinematicsError(
            "path_sample_limit_exceeded",
            f"path needs {count} samples for step {step} rad, limit is {max_samples}",
        )
    samples: list[tuple[float, ...]] = []
    for index in range(count):
        fraction = index / (count - 1)
        samples.append(
            tuple(
                first + (second - first) * fraction
                for first, second in zip(start_values, end_values, strict=True)
            )
        )
    return tuple(samples)
