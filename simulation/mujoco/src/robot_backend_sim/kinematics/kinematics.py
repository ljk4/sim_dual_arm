"""Panda right-arm key-pose IK and geometric clearance diagnostics.

This module backs the Panda stack target/key-pose pipeline: the stack teacher
and the ``tools/check_kinematics.py`` diagnostic both drive it with the Panda
``right_joint1..7`` chain and ``right_tcp`` site.  Robot-agnostic primitives
(``ObjectPose``, ``sample_joint_path``, ``rotation_down``) live in
:mod:`robot_backend_sim.kinematics.common`; the Marvin handover planner uses
:mod:`robot_backend_sim.kinematics.dual_arm` instead.

Pure target and joint-path construction stays at the top of this module so it
can be validated before MuJoCo or an EGL context exists.  MuJoCo-facing
functions use a private ``mink.Configuration``/MjData and never write
``Scene.data``.  The gripper stays open and both objects stay at their initial
poses; lift/place are key-pose previews, not object transport.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from robot_backend_sim.config.kinematics_config import (
    PHASES,
    ClearanceConfig,
    IKConfig,
    TargetConfig,
)
from robot_backend_sim.kinematics.common import (
    ObjectPose,
    finite_float,
    rotation_down,
    sample_joint_path,
    vector,
)
from robot_backend_sim.kinematics.kinematics_clearance import (
    ClearancePoint,
    CollisionPair,
    CollisionPairSet,
    KinematicsError,
    PathResult,
    _check_finite_and_frozen,
    _frozen_deviation,
    build_collision_pairs,
    measure_clearance,
)

__all__ = [
    "ClearancePoint",
    "CollisionPair",
    "CollisionPairSet",
    "IKResult",
    "KeyPoseTarget",
    "KinematicsError",
    "PathResult",
    "build_collision_pairs",
    "build_targets",
    "check_initial_pose",
    "check_path",
    "measure_clearance",
    "solve_key_poses",
]


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
    table_top = finite_float(table_top_z, "table_top_z")
    source_half = finite_float(source_half_z, "source_half_z")
    target_half = finite_float(target_half_z, "target_half_z")
    if source_half <= 0.0 or target_half <= 0.0:
        raise KinematicsError("target_invalid", "object half extents must be positive")
    source_center = vector(source.center, 3, "source.center")
    target_center = vector(target.center, 3, "target.center")
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


def _right_joint_ids(model: object) -> tuple[int, ...]:
    try:
        return tuple(int(model.joint(f"right_joint{index}").id) for index in range(1, 8))
    except KeyError as error:
        raise KinematicsError(
            "scene_model_missing_name", f"right arm joint is missing: {error}"
        ) from error


def _right_addresses(model: object, joint_ids: tuple[int, ...]) -> tuple[int, ...]:
    return tuple(int(model.jnt_qposadr[joint_id]) for joint_id in joint_ids)


def _configuration_limit(model: object, joint_ids: tuple[int, ...], margin_rad: float):
    """Configuration limit with an interior margin on the solved arm hinges.

    ``mink.ConfigurationLimit`` clamps at the exact hard limits.  Shrinking just
    the active arm joints keeps every solved key pose at least ``margin_rad``
    away from a hard limit while leaving free/finger joints untouched.
    """
    import mink

    limit = mink.ConfigurationLimit(model)
    if margin_rad <= 0.0:
        return limit
    for joint_id in joint_ids:
        if not int(model.jnt_limited[joint_id]):
            continue
        lower = float(model.jnt_range[joint_id][0]) + margin_rad
        upper = float(model.jnt_range[joint_id][1]) - margin_rad
        if lower >= upper:
            raise KinematicsError(
                "ik_limit_margin_invalid",
                f"joint margin {margin_rad:g} rad collapses joint {joint_id}",
            )
        limit.lower[int(model.jnt_qposadr[joint_id])] = lower
        limit.upper[int(model.jnt_qposadr[joint_id])] = upper
    return limit


def _mink_limits(
    model: object,
    joint_ids: tuple[int, ...],
    pair_set: CollisionPairSet,
    clearance: ClearanceConfig,
    limit_margin_rad: float,
):
    import mink

    try:
        limits = [
            _configuration_limit(model, joint_ids, limit_margin_rad),
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
        return limits
    except Exception as error:
        raise KinematicsError(
            "mink_constraint_api_mismatch",
            f"mink 1.3 constraint construction failed: {type(error).__name__}: {error}",
        ) from error


def _solver_start(initial, right_addresses, values) -> object:
    solver_start = initial.copy()
    for address, value in zip(right_addresses, values, strict=True):
        solver_start[address] = value
    return solver_start


def _solve_one(
    model: object,
    target: KeyPoseTarget,
    start: object,
    ik_config: IKConfig,
    limits: list,
    constraints: list,
    frozen_dofs: list[int],
    posture_reference: object,
):
    import mink
    import numpy as np

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
    return (
        configuration,
        converged,
        iterations,
        position_error,
        orientation_error,
        failure_reason,
        failure_message,
        time.perf_counter() - started,
    )


def _solve_result(
    model: object,
    target: KeyPoseTarget,
    initial: object,
    right_addresses: tuple[int, ...],
    joint_ids: tuple[int, ...],
    solved_state,
) -> IKResult:
    import numpy as np

    (
        configuration,
        converged,
        iterations,
        position_error,
        orientation_error,
        failure_reason,
        failure_message,
        elapsed,
    ) = solved_state
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
    return (
        IKResult(
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
        ),
        solved,
    )


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
    if ik_config.limit_margin_rad > 0.0:
        for index, joint_id in enumerate(joint_ids):
            angle = ik_config.initial_right_joint_positions[index]
            lower = float(model.jnt_range[joint_id][0]) + ik_config.limit_margin_rad
            upper = float(model.jnt_range[joint_id][1]) - ik_config.limit_margin_rad
            if not lower <= angle <= upper:
                raise KinematicsError(
                    "ik_seed_outside_margin",
                    f"initial_right_joint_positions[{index}] is outside the IK "
                    f"margin bounds [{lower:.6f}, {upper:.6f}]",
                )
    limits = _mink_limits(
        model, joint_ids, pair_set, clearance, ik_config.limit_margin_rad
    )
    constraints = [mink.DofFreezingTask(model, frozen_dofs)]
    # The supervisor's numeric reach seed is only an IK initial value.  The
    # actual path start remains the layout's Scene home qpos, which is still
    # used by check_path and by the frozen-qpos comparison baseline.
    start = _solver_start(
        initial, right_addresses, ik_config.initial_right_joint_positions
    )
    posture_reference = start.copy()
    results: list[IKResult] = []
    for target in targets:
        solved_state = _solve_one(
            model,
            target,
            start,
            ik_config,
            limits,
            constraints,
            frozen_dofs,
            posture_reference,
        )
        result, solved = _solve_result(
            model, target, initial, right_addresses, joint_ids, solved_state
        )
        results.append(result)
        if result.converged:
            start = solved.copy()
        if result.reason is not None:
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


def _scan_path(
    model: object,
    initial: tuple[float, ...],
    right_addresses: tuple[int, ...],
    joint_ids: tuple[int, ...],
    samples: tuple[tuple[float, ...], ...],
    pair_set: CollisionPairSet,
    clearance: ClearanceConfig,
    segment: str,
    deadline: float | None,
) -> tuple[ClearancePoint | None, str | None, str | None]:
    import mink

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
            _check_finite_and_frozen(
                model, configuration.q, initial, right_addresses, joint_ids
            )
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
    return minimum, failure_reason, failure_message


def _initial_point(
    model: object,
    initial: tuple[float, ...],
    pair_set: CollisionPairSet,
    clearance: ClearanceConfig,
) -> ClearancePoint:
    import mink

    configuration = mink.Configuration(model)
    configuration.update(initial)
    return measure_clearance(
        model,
        configuration.data,
        pair_set,
        detection_distance_m=clearance.collision_detection_distance_m,
        sample_index=0,
        fraction=0.0,
    )


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
    minimum, failure_reason, failure_message = _scan_path(
        model,
        initial,
        right_addresses,
        joint_ids,
        samples,
        pair_set,
        clearance,
        segment,
        deadline,
    )
    if minimum is None:
        # Keep a real measured point in the result even for an immediate
        # failure; this is the initial pose, not a fabricated large distance.
        try:
            minimum = _initial_point(model, initial, pair_set, clearance)
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
