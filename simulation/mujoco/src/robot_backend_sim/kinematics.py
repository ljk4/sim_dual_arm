"""Typed key poses, private mink IK and geometric clearance diagnostics.

Pure target and joint-path construction lives at the top of this module so it
can be validated before MuJoCo or an EGL context exists.  MuJoCo-facing functions use
a private ``mink.Configuration``/MjData and never write ``Scene.data``.  The
gripper stays open and both objects stay at their initial poses; lift/place are
key-pose previews, not object transport.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from .kinematics_config import (
    PHASES,
    ClearanceConfig,
    IKConfig,
    TargetConfig,
)
from .scene_config import PANDA_JOINT_RANGES

__all__ = [
    "ClearancePoint",
    "CollisionPair",
    "CollisionPairSet",
    "IKResult",
    "KeyPoseTarget",
    "KinematicsError",
    "ObjectPose",
    "PathResult",
    "build_collision_pairs",
    "build_targets",
    "check_initial_pose",
    "check_path",
    "extract_object_pose",
    "measure_clearance",
    "rotation_down",
    "sample_joint_path",
    "solve_key_poses",
]


class KinematicsError(RuntimeError):
    """A stable kinematics/clearance failure with a bounded reason."""

    def __init__(self, reason: str, message: str = "") -> None:
        super().__init__(message or reason)
        self.reason = reason
        self.message = message or reason


@dataclass(frozen=True, slots=True)
class ObjectPose:
    name: str
    center: tuple[float, float, float]
    yaw_rad: float


@dataclass(frozen=True, slots=True)
class KeyPoseTarget:
    phase: str
    position: tuple[float, float, float]
    rotation: tuple[tuple[float, float, float], ...]


@dataclass(frozen=True, slots=True)
class IKResult:
    phase: str
    target: KeyPoseTarget
    converged: bool
    iterations: int
    position_error_m: float
    orientation_error_rad: float
    qpos_right: tuple[float, ...]
    qpos_full: tuple[float, ...]
    frozen_max_deviation: float
    elapsed_s: float
    reason: str | None = None
    message: str | None = None


@dataclass(frozen=True, slots=True)
class ClearancePoint:
    distance_m: float
    geom1: int | None
    geom2: int | None
    body1: str | None
    body2: str | None
    geom1_name: str | None
    geom2_name: str | None
    sample_index: int
    fraction: float
    measured_pairs: int
    broadphase_skipped: int
    lower_bound: bool


@dataclass(frozen=True, slots=True)
class PathResult:
    segment: str
    sample_count: int
    requested_max_step_rad: float
    actual_max_step_rad: float
    minimum: ClearancePoint
    passed: bool
    elapsed_s: float
    failure_reason: str | None = None
    failure_message: str | None = None


@dataclass(frozen=True, slots=True)
class CollisionPair:
    geom1: int | None
    geom2: int | None
    body1: str | None
    body2: str | None
    geom1_name: str | None
    geom2_name: str | None
    category: str


@dataclass(frozen=True, slots=True)
class CollisionPairSet:
    pairs: tuple[CollisionPair, ...]
    categories: dict[str, int]
    exclusions: dict[str, int]
    total_physical_pairs: int
    explicit_pair_count: int


def _finite_float(value: object, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise KinematicsError("target_invalid", f"{field} must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise KinematicsError("target_invalid", f"{field} must be finite")
    return result


def _vector(value: object, length: int, field: str) -> tuple[float, ...]:
    if not isinstance(value, tuple | list) or len(value) != length:
        raise KinematicsError("target_invalid", f"{field} must contain {length} values")
    return tuple(_finite_float(item, f"{field}[{index}]") for index, item in enumerate(value))


def rotation_down(yaw_rad: float) -> tuple[tuple[float, float, float], ...]:
    """Return ``Rz(yaw) @ diag(1, -1, -1)`` as an explicit row-major matrix."""
    yaw = _finite_float(yaw_rad, "yaw_rad")
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


def build_targets(
    source: ObjectPose,
    target: ObjectPose,
    *,
    table_top_z: float,
    source_half_z: float,
    target_half_z: float,
    config: TargetConfig,
) -> tuple[KeyPoseTarget, ...]:
    """Build the six key poses from the complete initial object snapshot."""
    table_top = _finite_float(table_top_z, "table_top_z")
    source_half = _finite_float(source_half_z, "source_half_z")
    target_half = _finite_float(target_half_z, "target_half_z")
    if source_half <= 0.0 or target_half <= 0.0:
        raise KinematicsError("target_invalid", "object half extents must be positive")
    source_center = _vector(source.center, 3, "source.center")
    target_center = _vector(target.center, 3, "target.center")
    if not math.isfinite(source.yaw_rad) or not math.isfinite(target.yaw_rad):
        raise KinematicsError("target_invalid", "object yaw must be finite")
    rotation = rotation_down(source.yaw_rad)
    places = (
        (
            "pregrasp",
            (source_center[0], source_center[1], source_center[2] + config.pregrasp_height_m),
        ),
        (
            "grasp_pose",
            (source_center[0], source_center[1], source_center[2] + config.grasp_height_m),
        ),
        (
            "lift_pose",
            (source_center[0], source_center[1], source_center[2] + config.lift_height_m),
        ),
        (
            "place_hover",
            (target_center[0], target_center[1], target_center[2] + config.place_hover_height_m),
        ),
        (
            "place_pose",
            (
                target_center[0],
                target_center[1],
                table_top + 2.0 * target_half + source_half + config.place_clearance_m,
            ),
        ),
        (
            "retreat",
            (target_center[0], target_center[1], target_center[2] + config.retreat_height_m),
        ),
    )
    targets: list[KeyPoseTarget] = []
    for phase, position in places:
        if not all(math.isfinite(value) for value in position):
            raise KinematicsError("target_invalid", f"{phase} target is not finite")
        targets.append(KeyPoseTarget(phase=phase, position=position, rotation=rotation))
    if tuple(item.phase for item in targets) != PHASES:
        raise KinematicsError("target_invalid", "key-pose order does not match configured phases")
    return tuple(targets)


def sample_joint_path(
    start: object,
    end: object,
    *,
    max_step_rad: float,
    max_samples: int,
) -> tuple[tuple[float, ...], ...]:
    """Sample a linear joint-space path, including both endpoints."""
    step = _finite_float(max_step_rad, "max_step_rad")
    if step <= 0.0:
        raise KinematicsError("path_sample_invalid", "max_step_rad must be positive")
    if isinstance(max_samples, bool) or not isinstance(max_samples, int) or max_samples < 2:
        raise KinematicsError("path_sample_invalid", "max_samples must be an integer >= 2")
    start_values = _vector(start, len(start), "start")  # type: ignore[arg-type]
    end_values = _vector(end, len(end), "end")  # type: ignore[arg-type]
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


def _geometry_names(model: object) -> tuple[list[str], list[str]]:
    body_names = [str(model.body(index).name) or f"body[{index}]" for index in range(model.nbody)]
    geom_names = [str(model.geom(index).name) or f"geom[{index}]" for index in range(model.ngeom)]
    return body_names, geom_names


def _body_movable(model: object, body_id: int) -> bool:
    current = body_id
    while current > 0:
        if int(model.body_jntnum[current]) > 0:
            return True
        current = int(model.body_parentid[current])
    return False


def _robot_body(name: str) -> bool:
    return name.startswith("left_") or name.startswith("right_")


def build_collision_pairs(model: object) -> CollisionPairSet:
    """Enumerate effective robot-involving physical geom pairs natively.

    All native default exclusions are accounted for explicitly; no global
    "robot collision off" switch is used and no model margin is modified.
    """
    body_names, geom_names = _geometry_names(model)
    geom_body = [int(value) for value in model.geom_bodyid]
    contype = [int(value) for value in model.geom_contype]
    conaffinity = [int(value) for value in model.geom_conaffinity]
    welded = [int(value) for value in model.body_weldid]
    parent = [int(value) for value in model.body_parentid]
    movable = [_body_movable(model, body) for body in range(model.nbody)]
    exclude_signatures = {int(value) for value in model.exclude_signature}
    explicit: set[tuple[int, int]] = set()
    for index in range(model.npair):
        first = int(model.pair_geom1[index])
        second = int(model.pair_geom2[index])
        explicit.add((min(first, second), max(first, second)))

    categories: dict[str, int] = {}
    exclusions: dict[str, int] = {}
    pairs: list[CollisionPair] = []
    physical = 0
    explicit_count = 0
    for first in range(model.ngeom):
        for second in range(first + 1, model.ngeom):
            first_body = geom_body[first]
            second_body = geom_body[second]
            first_name = body_names[first_body]
            second_name = body_names[second_body]
            if first_body == second_body:
                exclusions["same_body"] = exclusions.get("same_body", 0) + 1
                continue
            if welded[first_body] == welded[second_body]:
                exclusions["welded_body"] = exclusions.get("welded_body", 0) + 1
                continue
            key = (first, second)
            is_explicit = key in explicit
            if is_explicit:
                explicit_count += 1
            elif not (contype[first] & conaffinity[second] or contype[second] & conaffinity[first]):
                exclusions["visual_or_no_affinity"] = exclusions.get("visual_or_no_affinity", 0) + 1
                continue
            if not is_explicit:
                # Match mink/MuJoCo's welded-body parent-child rule rather than
                # only the direct body parent link.
                first_weld = welded[first_body]
                second_weld = welded[second_body]
                first_parent_weld = welded[parent[first_weld]]
                second_parent_weld = welded[parent[second_weld]]
                if first_weld == second_parent_weld or second_weld == first_parent_weld:
                    exclusions["parent_child"] = exclusions.get("parent_child", 0) + 1
                    continue
            if not is_explicit:
                signature = (min(first_body, second_body) << 16) | max(first_body, second_body)
                if signature in exclude_signatures:
                    exclusions["explicit_exclude"] = exclusions.get("explicit_exclude", 0) + 1
                    continue
            physical += 1
            if not _robot_body(first_name) and not _robot_body(second_name):
                exclusions["non_robot_pair"] = exclusions.get("non_robot_pair", 0) + 1
                continue
            if not movable[first_body] and not movable[second_body]:
                exclusions["fixed_connection"] = exclusions.get("fixed_connection", 0) + 1
                continue
            if _robot_body(first_name) and _robot_body(second_name):
                category = (
                    "robot_self"
                    if first_name.split("_", 1)[0] == second_name.split("_", 1)[0]
                    else "robot_cross"
                )
            elif _robot_body(first_name) or _robot_body(second_name):
                other = second_name if _robot_body(first_name) else first_name
                category = "robot_object" if other in {"source", "target"} else "robot_scene"
            else:  # pragma: no cover - guarded above
                continue
            pairs.append(
                CollisionPair(
                    geom1=first,
                    geom2=second,
                    body1=first_name,
                    body2=second_name,
                    geom1_name=geom_names[first],
                    geom2_name=geom_names[second],
                    category=category,
                )
            )
            categories[category] = categories.get(category, 0) + 1
    if not pairs:
        raise KinematicsError("collision_pairs_empty", "no effective robot collision pairs found")
    return CollisionPairSet(
        pairs=tuple(pairs),
        categories=categories,
        exclusions=exclusions,
        total_physical_pairs=physical,
        explicit_pair_count=explicit_count,
    )


def _frozen_deviation(qpos: object, initial: object, free_addresses: set[int]) -> float:
    deviation = 0.0
    for index in range(len(initial)):  # type: ignore[arg-type]
        if index in free_addresses:
            continue
        difference = abs(float(qpos[index]) - float(initial[index]))  # type: ignore[index]
        if difference > deviation:
            deviation = difference
    return deviation


def _check_finite_and_frozen(
    model: object,
    qpos: object,
    initial_qpos: object,
    right_addresses: tuple[int, ...],
    right_joint_ids: tuple[int, ...],
) -> None:
    import numpy as np

    values = np.asarray(qpos, dtype=np.float64)
    if values.shape != (model.nq,) or not np.isfinite(values).all():
        raise KinematicsError(
            "ik_state_invalid", "solved qpos is not a finite vector of model.nq values"
        )
    free_addresses = set(right_addresses)
    deviation = _frozen_deviation(values, np.asarray(initial_qpos), free_addresses)
    if deviation > 1e-8:
        raise KinematicsError(
            "frozen_qpos_changed",
            f"non-right-arm qpos changed by {deviation:.3e} (quaternion rounding allowed)",
        )
    for joint_id in right_joint_ids:
        if int(model.jnt_limited[joint_id]) == 0:
            continue
        address = int(model.jnt_qposadr[joint_id])
        value = float(values[address])
        lower = float(model.jnt_range[joint_id, 0])
        upper = float(model.jnt_range[joint_id, 1])
        if value < lower - 1e-9 or value > upper + 1e-9:
            raise KinematicsError(
                "joint_limit_violation",
                f"joint {model.joint(joint_id).name!r} value {value:.6f} outside "
                f"[{lower:.6f}, {upper:.6f}]",
            )


def measure_clearance(
    model: object,
    data: object,
    pair_set: CollisionPairSet,
    *,
    detection_distance_m: float,
    sample_index: int,
    fraction: float,
) -> ClearancePoint:
    """Measure the smallest signed distance in a conservative broadphase."""
    import mujoco
    import numpy as np

    detection = float(detection_distance_m)
    if not math.isfinite(detection) or detection <= 0.0:
        raise KinematicsError("clearance_config_invalid", "detection distance must be positive")
    positions = np.asarray(data.geom_xpos)
    radii = np.asarray(model.geom_rbound)
    minimum = detection
    worst: CollisionPair | None = None
    measured = 0
    if not np.isfinite(positions).all() or not np.isfinite(radii).all():
        raise KinematicsError("clearance_non_finite", "geom positions and bounds must be finite")
    pair_ids = np.asarray([(pair.geom1, pair.geom2) for pair in pair_set.pairs], dtype=np.intp)
    first, second = pair_ids[:, 0], pair_ids[:, 1]
    bounded = (radii[first] > 0.0) & (radii[second] > 0.0)
    displacement = positions[first] - positions[second]
    distance_squared = np.einsum("ij,ij->i", displacement, displacement)
    maximum_radius = radii[first] + radii[second] + detection
    # Unbounded geoms are always queried; finite bounding spheres only cull
    # pairs that provably cannot be within the detection distance.
    candidates = np.flatnonzero(~bounded | (distance_squared <= maximum_radius**2))
    skipped = len(pair_set.pairs) - len(candidates)
    for index in candidates:
        pair = pair_set.pairs[int(index)]
        fromto = np.zeros(6, dtype=np.float64)
        distance = float(
            mujoco.mj_geomDistance(model, data, pair.geom1, pair.geom2, detection, fromto)
        )
        measured += 1
        if not math.isfinite(distance):
            raise KinematicsError(
                "clearance_non_finite",
                f"mj_geomDistance returned a non-finite value for "
                f"{pair.geom1_name}/{pair.geom2_name}",
            )
        if distance < minimum:
            minimum = distance
            worst = pair
    if worst is None:
        return ClearancePoint(
            distance_m=detection,
            geom1=None,
            geom2=None,
            body1=None,
            body2=None,
            geom1_name=None,
            geom2_name=None,
            sample_index=sample_index,
            fraction=fraction,
            measured_pairs=measured,
            broadphase_skipped=skipped,
            lower_bound=True,
        )
    return ClearancePoint(
        distance_m=minimum,
        geom1=worst.geom1,
        geom2=worst.geom2,
        body1=worst.body1,
        body2=worst.body2,
        geom1_name=worst.geom1_name,
        geom2_name=worst.geom2_name,
        sample_index=sample_index,
        fraction=fraction,
        measured_pairs=measured,
        broadphase_skipped=skipped,
        lower_bound=minimum >= detection - 1e-12,
    )


def _right_joint_ids(model: object) -> tuple[int, ...]:
    try:
        return tuple(int(model.joint(f"right_joint{index}").id) for index in range(1, 8))
    except KeyError as error:
        raise KinematicsError(
            "scene_model_missing_name", f"right arm joint is missing: {error}"
        ) from error


def _right_addresses(model: object, joint_ids: tuple[int, ...]) -> tuple[int, ...]:
    return tuple(int(model.jnt_qposadr[joint_id]) for joint_id in joint_ids)


def solve_key_poses(
    model: object,
    initial_qpos: object,
    targets: tuple[KeyPoseTarget, ...],
    ik_config: IKConfig,
    clearance: ClearanceConfig,
    pair_set: CollisionPairSet,
) -> tuple[IKResult, ...]:
    """Solve every configured target from one layout's initial snapshot."""
    import mink
    import numpy as np

    joint_ids = _right_joint_ids(model)
    right_dofs = {int(model.jnt_dofadr[joint_id]) for joint_id in joint_ids}
    frozen_dofs = sorted(set(range(int(model.nv))) - right_dofs)
    right_addresses = _right_addresses(model, joint_ids)
    initial = np.asarray(initial_qpos, dtype=np.float64)
    if initial.shape != (model.nq,) or not np.isfinite(initial).all():
        raise KinematicsError("initial_state_invalid", "initial qpos is not finite")

    try:
        limits = [
            mink.ConfigurationLimit(model),
            mink.CollisionAvoidanceLimit(
                model,
                [((pair.geom1,), (pair.geom2,)) for pair in pair_set.pairs],
                minimum_distance_from_collisions=clearance.minimum_clearance_m,
                collision_detection_distance=clearance.collision_detection_distance_m,
            ),
        ]
        # mink 1.3 implements DofFreezingTask as an equality Task.  Passing it
        # through ``constraints`` makes the freeze exact; as a normal task it
        # would only be a soft preference.
        constraints = [mink.DofFreezingTask(model, frozen_dofs)]
    except Exception as error:
        raise KinematicsError(
            "mink_constraint_api_mismatch",
            f"mink 1.3 constraint construction failed: {type(error).__name__}: {error}",
        ) from error

    # The supervisor's numeric reach seed is only an IK initial value.  The
    # actual path start remains the layout's Scene home qpos, which is still
    # used by check_path and by the frozen-qpos comparison baseline.
    solver_start = initial.copy()
    for address, value in zip(
        right_addresses, ik_config.initial_right_joint_positions, strict=True
    ):
        solver_start[address] = value
    posture_reference = solver_start.copy()
    results: list[IKResult] = []
    start = solver_start
    for target in targets:
        started = time.perf_counter()
        configuration = mink.Configuration(model)
        configuration.update(start)
        frame_task = mink.FrameTask(
            frame_name="right_tcp",
            frame_type="site",
            position_cost=ik_config.position_cost,
            orientation_cost=ik_config.orientation_cost,
            lm_damping=ik_config.lm_damping,
        )
        pose = np.eye(4, dtype=np.float64)
        pose[:3, :3] = np.asarray(target.rotation, dtype=np.float64)
        pose[:3, 3] = np.asarray(target.position, dtype=np.float64)
        frame_task.set_target(mink.SE3.from_matrix(pose))
        posture_task = mink.PostureTask(model, cost=ik_config.posture_cost)
        posture_task.set_target(posture_reference)
        tasks = [frame_task, posture_task]
        converged = False
        iterations = 0
        position_error = math.inf
        orientation_error = math.inf
        failure_reason: str | None = None
        failure_message: str | None = None
        try:
            for iteration in range(1, ik_config.max_iterations + 1):
                iterations = iteration
                velocity = np.asarray(
                    mink.solve_ik(
                        configuration,
                        tasks,
                        ik_config.dt_s,
                        solver=ik_config.solver,
                        limits=limits,
                        constraints=constraints,
                    ),
                    dtype=np.float64,
                )
                # The hard DofFreezingTask constraint already requires zero
                # velocity on these DoFs; zeroing the integrated vector makes
                # the "only seven right hinge qpos may change" invariant exact
                # despite QP equality tolerances.
                velocity[frozen_dofs] = 0.0
                configuration.integrate_inplace(velocity, ik_config.dt_s)
                error = np.asarray(frame_task.compute_error(configuration), dtype=np.float64)
                position_error = float(np.linalg.norm(error[:3]))
                orientation_error = float(np.linalg.norm(error[3:]))
                if (
                    position_error <= ik_config.position_tolerance_m
                    and orientation_error <= ik_config.orientation_tolerance_rad
                ):
                    converged = True
                    break
        except Exception as error:
            failure_reason = "ik_solver_failed"
            failure_message = f"{type(error).__name__}: {error}"
        elapsed = time.perf_counter() - started
        solved = np.asarray(configuration.q, dtype=np.float64)
        try:
            _check_finite_and_frozen(model, solved, initial, right_addresses, joint_ids)
            frozen_deviation = _frozen_deviation(solved, initial, set(right_addresses))
        except KinematicsError as error:
            failure_reason = error.reason
            failure_message = error.message
            frozen_deviation = math.inf
        if not converged and failure_reason is None:
            failure_reason = "ik_not_converged"
            failure_message = (
                f"target {target.phase!r} did not reach tolerances after "
                f"{iterations} iterations: position={position_error:.6g} m, "
                f"orientation={orientation_error:.6g} rad"
            )
        result = IKResult(
            phase=target.phase,
            target=target,
            converged=converged and failure_reason is None,
            iterations=iterations,
            position_error_m=position_error,
            orientation_error_rad=orientation_error,
            qpos_right=tuple(float(solved[address]) for address in right_addresses),
            qpos_full=tuple(float(value) for value in solved),
            frozen_max_deviation=frozen_deviation,
            elapsed_s=elapsed,
            reason=failure_reason,
            message=failure_message,
        )
        results.append(result)
        if result.converged:
            start = solved.copy()
        if failure_reason is not None:
            break
    return tuple(results)


def _set_right_qpos(
    qpos: object, right_addresses: tuple[int, ...], values: tuple[float, ...]
) -> object:
    import numpy as np

    updated = np.array(qpos, dtype=np.float64, copy=True)
    for address, value in zip(right_addresses, values, strict=True):
        updated[address] = value
    return updated


def _min_step(samples: tuple[tuple[float, ...], ...]) -> float:
    maximum = 0.0
    for first, second in zip(samples, samples[1:], strict=False):
        for left, right in zip(first, second, strict=True):
            maximum = max(maximum, abs(right - left))
    return maximum


def check_path(
    model: object,
    initial_qpos: object,
    start_right: object,
    end_right: object,
    pair_set: CollisionPairSet,
    clearance: ClearanceConfig,
    *,
    segment: str,
    deadline: float | None = None,
) -> PathResult:
    """Check one linear right-arm path on a private mink Configuration."""
    import mink

    started = time.perf_counter()
    joint_ids = _right_joint_ids(model)
    right_addresses = _right_addresses(model, joint_ids)
    initial = tuple(float(value) for value in initial_qpos)  # type: ignore[arg-type]
    samples = sample_joint_path(
        start_right,
        end_right,
        max_step_rad=clearance.sample_step_rad,
        max_samples=clearance.max_samples_per_segment,
    )
    actual_step = _min_step(samples)
    configuration = mink.Configuration(model)
    configuration.update(initial)
    minimum: ClearancePoint | None = None
    failure_reason: str | None = None
    failure_message: str | None = None
    for index, sample in enumerate(samples):
        if deadline is not None and time.monotonic() > deadline:
            failure_reason = "time_budget_exceeded"
            failure_message = f"path {segment!r} exceeded the elapsed-time budget"
            break
        qpos = _set_right_qpos(initial, right_addresses, sample)
        try:
            configuration.update(qpos)
            _check_finite_and_frozen(model, configuration.q, initial, right_addresses, joint_ids)
            point = measure_clearance(
                model,
                configuration.data,
                pair_set,
                detection_distance_m=clearance.collision_detection_distance_m,
                sample_index=index,
                fraction=index / (len(samples) - 1),
            )
        except KinematicsError as error:
            failure_reason = error.reason
            failure_message = error.message
            break
        if minimum is None or point.distance_m < minimum.distance_m:
            minimum = point
    if minimum is None:
        # Keep a real measured point in the result even for an immediate
        # failure; this is the initial pose, not a fabricated large distance.
        try:
            configuration.update(initial)
            minimum = measure_clearance(
                model,
                configuration.data,
                pair_set,
                detection_distance_m=clearance.collision_detection_distance_m,
                sample_index=0,
                fraction=0.0,
            )
        except KinematicsError as error:
            raise KinematicsError(error.reason, error.message) from error
    passed = failure_reason is None and minimum.distance_m >= clearance.minimum_clearance_m
    if not passed and failure_reason is None:
        failure_reason = "clearance_below_minimum"
        failure_message = (
            f"segment {segment!r} sample {minimum.sample_index} "
            f"({minimum.geom1_name}/{minimum.geom2_name}, "
            f"{minimum.body1}/{minimum.body2}) has clearance "
            f"{minimum.distance_m:.6f} m < {clearance.minimum_clearance_m:.6f} m"
        )
    return PathResult(
        segment=segment,
        sample_count=len(samples),
        requested_max_step_rad=clearance.sample_step_rad,
        actual_max_step_rad=actual_step,
        minimum=minimum,
        passed=passed,
        elapsed_s=time.perf_counter() - started,
        failure_reason=failure_reason,
        failure_message=failure_message,
    )


def check_initial_pose(
    model: object,
    initial_qpos: object,
    pair_set: CollisionPairSet,
    clearance: ClearanceConfig,
) -> PathResult:
    """Check the untouched initial pose as the first path endpoint."""
    import mink

    started = time.perf_counter()
    joint_ids = _right_joint_ids(model)
    right_addresses = _right_addresses(model, joint_ids)
    initial = tuple(float(value) for value in initial_qpos)  # type: ignore[arg-type]
    configuration = mink.Configuration(model)
    configuration.update(initial)
    _check_finite_and_frozen(model, configuration.q, initial, right_addresses, joint_ids)
    point = measure_clearance(
        model,
        configuration.data,
        pair_set,
        detection_distance_m=clearance.collision_detection_distance_m,
        sample_index=0,
        fraction=0.0,
    )
    passed = point.distance_m >= clearance.minimum_clearance_m
    return PathResult(
        segment="initial",
        sample_count=1,
        requested_max_step_rad=clearance.sample_step_rad,
        actual_max_step_rad=0.0,
        minimum=point,
        passed=passed,
        elapsed_s=time.perf_counter() - started,
        failure_reason=(None if passed else "clearance_below_minimum"),
        failure_message=(
            None
            if passed
            else (
                f"initial pose clearance {point.distance_m:.6f} m < "
                f"{clearance.minimum_clearance_m:.6f} m at "
                f"{point.geom1_name}/{point.geom2_name}"
            )
        ),
    )


def _panda_range_check(right_qpos: object) -> None:
    """Pure range check used by target/path tests without a MuJoCo model."""
    values = tuple(float(value) for value in right_qpos)  # type: ignore[arg-type]
    if len(values) != len(PANDA_JOINT_RANGES):
        raise KinematicsError("joint_limit_violation", "right arm must have seven joints")
    for index, (value, (lower, upper)) in enumerate(zip(values, PANDA_JOINT_RANGES, strict=True)):
        if not math.isfinite(value) or value < lower or value > upper:
            raise KinematicsError(
                "joint_limit_violation",
                f"right_joint{index + 1}={value!r} outside [{lower}, {upper}]",
            )
