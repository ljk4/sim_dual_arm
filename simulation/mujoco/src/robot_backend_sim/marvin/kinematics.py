"""Robot-agnostic multi-arm IK and carried-object path clearance.

The robot adapter supplies each arm's joint names and TCP site; the task
supplies the target poses and which object, if any, is assumed rigidly held.
Every MuJoCo forward computation runs on a private ``mink.Configuration`` or a
private ``MjData`` copy.  The live scene is never written, so planning cannot
move an object or a joint.

Sequential IK is the simplest scheme that completes the first handover task:
targets are solved in order, each one starting from the previous accepted
solution while every non-active DoF is held at the incoming state.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass

from robot_backend_sim.common.kinematics.common import sample_joint_path
from robot_backend_sim.common.kinematics.kinematics_clearance import (
    ClearancePoint,
    CollisionPairSet,
    KinematicsError,
    _check_finite_and_frozen,
    _frozen_deviation,
    measure_clearance,
)
from robot_backend_sim.common.physics.robot_geometry import ArmChain

__all__ = [
    "ArmChain",
    "BlockEnvelope",
    "CarriedBlock",
    "ClearanceParameters",
    "IKSolution",
    "IKParameters",
    "KeyTarget",
    "PathCheck",
    "check_joint_path",
    "object_pair_set",
    "relative_block_pose",
    "robot_pair_set",
    "solve_targets",
    "tcp_matrix",
]

_IDENTITY = (
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.0),
    (0.0, 0.0, 0.0, 1.0),
)


@dataclass(frozen=True, slots=True)
class IKParameters:
    dt_s: float
    max_iterations: int
    position_tolerance_m: float
    orientation_tolerance_rad: float
    posture_cost: float
    position_cost: float
    orientation_cost: float
    lm_damping: float
    solver: str = "daqp"
    # Interior margin (rad) reserved by the IK configuration limit on every
    # solved arm hinge, so no joint can land on a hard limit.
    limit_margin_rad: float = 0.0


@dataclass(frozen=True, slots=True)
class ClearanceParameters:
    minimum_clearance_m: float
    collision_detection_distance_m: float
    sample_step_rad: float
    max_samples_per_segment: int


@dataclass(frozen=True, slots=True)
class KeyTarget:
    name: str
    side: str
    position: tuple[float, float, float]
    rotation: tuple[tuple[float, float, float], ...]


@dataclass(frozen=True, slots=True)
class IKSolution:
    target: KeyTarget
    converged: bool
    iterations: int
    position_error_m: float
    orientation_error_rad: float
    qpos: tuple[float, ...]
    frozen_max_deviation: float
    elapsed_s: float
    reason: str | None = None
    message: str | None = None


@dataclass(frozen=True, slots=True)
class BlockEnvelope:
    """Rigid block pose expressed in the holding TCP frame."""

    object_name: str
    body_id: int
    geom_id: int
    qpos_address: int
    holder_tcp_site: str
    relative_pose: tuple[tuple[float, float, float, float], ...]


@dataclass(frozen=True, slots=True)
class CarriedBlock:
    """How the block follows a path sample.

    ``follow_sample`` attaches the block to the moving holder's TCP.  When it
    is false the holder stays at ``holder_qpos`` and only the other arm or the
    environment is being checked against the held block.
    """

    envelope: BlockEnvelope
    holder_qpos: tuple[float, ...]
    follow_sample: bool


@dataclass(frozen=True, slots=True)
class PathCheck:
    segment: str
    side: str
    sample_count: int
    requested_max_step_rad: float
    actual_max_step_rad: float
    duration_s: float
    robot_minimum: ClearancePoint
    object_minimum: ClearancePoint | None
    passed: bool
    elapsed_s: float
    failure_reason: str | None = None
    failure_message: str | None = None


def _finite(value: float, field: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise KinematicsError("target_invalid", f"{field} must be finite")
    return result


def _pose_matrix(
    position: tuple[float, float, float],
    rotation: tuple[tuple[float, float, float], ...],
) -> object:
    import numpy as np

    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = np.asarray(rotation, dtype=np.float64)
    matrix[:3, 3] = np.asarray(position, dtype=np.float64)
    if not np.isfinite(matrix).all():
        raise KinematicsError("target_invalid", "target pose is not finite")
    return matrix


def _chain(
    model: object, chains: dict[str, ArmChain], side: str
) -> tuple[ArmChain, tuple[int, ...]]:
    chain = chains.get(side)
    if chain is None:
        raise KinematicsError("target_invalid", f"unknown arm side {side!r}")
    joint_ids: list[int] = []
    for name in chain.joint_names:
        try:
            joint_ids.append(int(model.joint(name).id))
        except KeyError as error:
            raise KinematicsError(
                "scene_model_missing_name", f"joint {name!r} is missing"
            ) from error
    try:
        model.site(chain.tcp_site)
    except KeyError as error:
        raise KinematicsError(
            "scene_model_missing_name", f"TCP site {chain.tcp_site!r} is missing"
        ) from error
    return chain, tuple(joint_ids)


def _configuration_limit(
    model: object, arm_joint_ids: tuple[int, ...], margin_rad: float
):
    """Configuration limit with an interior margin on every arm hinge.

    ``mink.ConfigurationLimit`` clamps at the exact hard limits.  Shrinking only
    the arm joints keeps every solved target at least ``margin_rad`` away from a
    hard limit while leaving free joints and gripper slides untouched.
    """
    import mink

    limit = mink.ConfigurationLimit(model)
    if margin_rad <= 0.0:
        return limit
    for joint_id in arm_joint_ids:
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


def _limits(
    model: object,
    pair_set: CollisionPairSet | None,
    clearance: ClearanceParameters,
    arm_joint_ids: tuple[int, ...],
    limit_margin_rad: float,
):
    import mink

    limits: list[object] = [_configuration_limit(model, arm_joint_ids, limit_margin_rad)]
    if pair_set is not None and pair_set.pairs:
        try:
            limits.append(
                mink.CollisionAvoidanceLimit(
                    model,
                    [((pair.geom1,), (pair.geom2,)) for pair in pair_set.pairs],
                    minimum_distance_from_collisions=clearance.minimum_clearance_m,
                    collision_detection_distance=clearance.collision_detection_distance_m,
                )
            )
        except Exception as error:
            raise KinematicsError(
                "mink_constraint_api_mismatch",
                f"mink collision limit construction failed: {type(error).__name__}: {error}",
            ) from error
    return limits


def _posture_reference(
    model: object, start: object, joint_ids: tuple[int, ...], margin_rad: float
) -> object:
    """Incoming state clamped into the interior margin box.

    The previous accepted solution (or the reset pose for the first target) is
    already interior and non-singular, so using it as the posture reference
    resolves the redundant DoF toward a safe configuration instead of toward a
    raw joint midpoint, which for this wrist is the q5 = 0 singularity.
    """
    import numpy as np

    target = np.array(start, dtype=np.float64, copy=True)
    if margin_rad > 0.0:
        for joint_id in joint_ids:
            address = int(model.jnt_qposadr[joint_id])
            lower = float(model.jnt_range[joint_id][0]) + margin_rad
            upper = float(model.jnt_range[joint_id][1]) - margin_rad
            target[address] = min(max(float(target[address]), lower), upper)
    return target


def solve_targets(
    model: object,
    initial_qpos: object,
    targets: tuple[KeyTarget, ...],
    chains: tuple[ArmChain, ...],
    ik: IKParameters,
    clearance: ClearanceParameters,
    pair_set: CollisionPairSet | None = None,
    *,
    deadline: float | None = None,
) -> tuple[IKSolution, ...]:
    """Solve targets sequentially on private configurations."""
    import mink
    import numpy as np

    chain_by_side = {chain.side: chain for chain in chains}
    if len(chain_by_side) != len(chains):
        raise KinematicsError("target_invalid", "arm chain sides must be unique")
    initial = np.asarray(initial_qpos, dtype=np.float64)
    if initial.shape != (model.nq,) or not np.isfinite(initial).all():
        raise KinematicsError("initial_state_invalid", "initial qpos is not finite")
    start = initial.copy()
    results: list[IKSolution] = []
    for target in targets:
        if deadline is not None and time.monotonic() > deadline:
            results.append(
                IKSolution(
                    target=target,
                    converged=False,
                    iterations=0,
                    position_error_m=math.inf,
                    orientation_error_rad=math.inf,
                    qpos=tuple(float(value) for value in start),
                    frozen_max_deviation=0.0,
                    elapsed_s=0.0,
                    reason="time_budget_exceeded",
                    message=f"target {target.name!r} was not solved before the budget",
                )
            )
            break
        chain, joint_ids = _chain(model, chain_by_side, target.side)
        active_addresses = tuple(int(model.jnt_qposadr[joint_id]) for joint_id in joint_ids)
        active_dofs = {int(model.jnt_dofadr[joint_id]) for joint_id in joint_ids}
        frozen_dofs = sorted(set(range(int(model.nv))) - active_dofs)
        # Only the active chain gets the interior margin; the frozen arm keeps
        # the hard limits so its measured state can never make the QP infeasible.
        limits = _limits(model, pair_set, clearance, joint_ids, ik.limit_margin_rad)
        started = time.perf_counter()
        configuration = mink.Configuration(model)
        configuration.update(start)
        frame_task = mink.FrameTask(
            frame_name=chain.tcp_site,
            frame_type="site",
            position_cost=ik.position_cost,
            orientation_cost=ik.orientation_cost,
            lm_damping=ik.lm_damping,
        )
        frame_task.set_target(mink.SE3.from_matrix(_pose_matrix(target.position, target.rotation)))
        posture_task = mink.PostureTask(model, cost=ik.posture_cost)
        posture_task.set_target(
            _posture_reference(model, start, joint_ids, ik.limit_margin_rad)
        )
        tasks = [frame_task, posture_task]
        converged = False
        iterations = 0
        position_error = math.inf
        orientation_error = math.inf
        failure_reason: str | None = None
        failure_message: str | None = None
        try:
            for iteration in range(1, ik.max_iterations + 1):
                if deadline is not None and time.monotonic() > deadline:
                    failure_reason = "time_budget_exceeded"
                    failure_message = f"target {target.name!r} exceeded the elapsed-time budget"
                    break
                iterations = iteration
                velocity = np.asarray(
                    mink.solve_ik(
                        configuration, tasks, ik.dt_s, solver=ik.solver, limits=limits
                    ),
                    dtype=np.float64,
                )
                velocity[frozen_dofs] = 0.0
                configuration.integrate_inplace(velocity, ik.dt_s)
                error = np.asarray(frame_task.compute_error(configuration), dtype=np.float64)
                position_error = float(np.linalg.norm(error[:3]))
                orientation_error = float(np.linalg.norm(error[3:]))
                if (
                    position_error <= ik.position_tolerance_m
                    and orientation_error <= ik.orientation_tolerance_rad
                ):
                    converged = True
                    break
        except Exception as error:  # noqa: BLE001 - solver failures are reported facts
            failure_reason = "ik_solver_failed"
            failure_message = f"{type(error).__name__}: {error}"
        solved = np.asarray(configuration.q, dtype=np.float64)
        frozen_deviation = math.inf
        try:
            _check_finite_and_frozen(
                model,
                solved,
                start,
                active_addresses,
                joint_ids,
            )
            frozen_deviation = _frozen_deviation(solved, start, set(active_addresses))
        except KinematicsError as error:
            failure_reason = failure_reason or error.reason
            failure_message = failure_message or error.message
        if not converged and failure_reason is None:
            failure_reason = "ik_not_converged"
            failure_message = (
                f"target {target.name!r} did not reach tolerances after {iterations} "
                f"iterations: position={position_error:.6g} m, "
                f"orientation={orientation_error:.6g} rad"
            )
        result = IKSolution(
            target=target,
            converged=converged and failure_reason is None,
            iterations=iterations,
            position_error_m=position_error,
            orientation_error_rad=orientation_error,
            qpos=tuple(float(value) for value in solved),
            frozen_max_deviation=frozen_deviation,
            elapsed_s=time.perf_counter() - started,
            reason=failure_reason,
            message=failure_message,
        )
        results.append(result)
        if result.converged:
            start = solved.copy()
        else:
            break
    return tuple(results)


def _matmul(left: object, right: object) -> object:
    import numpy as np

    return np.asarray(left, dtype=np.float64) @ np.asarray(right, dtype=np.float64)


def _quaternion(matrix: object) -> tuple[float, float, float, float]:
    import mujoco
    import numpy as np

    value = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(value, np.asarray(matrix, dtype=np.float64).reshape(9))
    return tuple(float(item) for item in value)


def _site_matrix(configuration: object, site_name: str) -> object:
    import numpy as np

    matrix = np.eye(4, dtype=np.float64)
    site_id = int(configuration.model.site(site_name).id)
    matrix[:3, :3] = np.asarray(configuration.data.site_xmat[site_id]).reshape(3, 3)
    matrix[:3, 3] = configuration.data.site_xpos[site_id]
    return matrix



def tcp_matrix(model: object, qpos: object, site_name: str) -> object:
    """Return the world TCP pose as a 4x4 matrix for one joint vector."""
    import mink
    import numpy as np

    configuration = mink.Configuration(model)
    configuration.update(np.asarray(qpos, dtype=np.float64))
    return _site_matrix(configuration, site_name)


def _set_block(
    model: object, data: object, envelope: BlockEnvelope, matrix: object
) -> None:
    import mujoco
    import numpy as np

    matrix = np.asarray(matrix, dtype=np.float64)
    data.qpos[envelope.qpos_address : envelope.qpos_address + 3] = matrix[:3, 3]
    data.qpos[envelope.qpos_address + 3 : envelope.qpos_address + 7] = _quaternion(
        matrix[:3, :3]
    )
    mujoco.mj_kinematics(model, data)
    mujoco.mj_comPos(model, data)


def _block_matrix(
    configuration: object,
    carried: CarriedBlock,
    follow_qpos: object | None,
) -> object:
    if carried.follow_sample:
        tcp = _site_matrix(configuration, carried.envelope.holder_tcp_site)
    else:
        import mink
        import numpy as np

        holder = mink.Configuration(configuration.model)
        holder.update(np.asarray(follow_qpos, dtype=np.float64))
        tcp = _site_matrix(holder, carried.envelope.holder_tcp_site)
    return _matmul(tcp, carried.envelope.relative_pose)


def _filter_pairs(
    pair_set: CollisionPairSet, allowed_body_names: frozenset[str]
) -> CollisionPairSet:
    if not allowed_body_names:
        return pair_set
    pairs = tuple(
        pair
        for pair in pair_set.pairs
        if pair.body1 not in allowed_body_names and pair.body2 not in allowed_body_names
    )
    return CollisionPairSet(
        pairs=pairs,
        categories={"carried_object": len(pairs)},
        exclusions=dict(pair_set.exclusions),
        total_physical_pairs=pair_set.total_physical_pairs,
        explicit_pair_count=pair_set.explicit_pair_count,
    )


def check_joint_path(
    model: object,
    initial_qpos: object,
    start_qpos: object,
    end_qpos: object,
    *,
    active_joint_names: tuple[str, ...],
    robot_pairs: CollisionPairSet,
    object_pairs: CollisionPairSet | None,
    clearance: ClearanceParameters,
    segment: str,
    side: str,
    max_velocities: tuple[float, ...] = (),
    carried: CarriedBlock | None = None,
    allowed_contacts_at_start: tuple[str, ...] = (),
    allowed_contacts_at_end: tuple[str, ...] = (),
    duration_peak_factor: float = 1.5,
    minimum_duration_s: float = 0.4,
    deadline: float | None = None,
) -> PathCheck:
    """Sample one joint-space segment and measure continuous clearance."""
    import mink
    import numpy as np

    started = time.perf_counter()
    initial = np.asarray(initial_qpos, dtype=np.float64)
    start = np.asarray(start_qpos, dtype=np.float64)
    end = np.asarray(end_qpos, dtype=np.float64)
    if initial.shape != (model.nq,) or start.shape != (model.nq,) or end.shape != (model.nq,):
        raise KinematicsError(
            "path_sample_invalid", "path endpoints must be finite model.nq vectors"
        )
    active_addresses = []
    joint_ids = []
    for name in active_joint_names:
        try:
            joint_id = int(model.joint(name).id)
        except KeyError as error:
            raise KinematicsError(
                "scene_model_missing_name", f"joint {name!r} is missing"
            ) from error
        joint_ids.append(joint_id)
        active_addresses.append(int(model.jnt_qposadr[joint_id]))
    free_addresses = set(active_addresses)
    samples = sample_joint_path(
        tuple(float(value) for value in start[active_addresses]),
        tuple(float(value) for value in end[active_addresses]),
        max_step_rad=clearance.sample_step_rad,
        max_samples=clearance.max_samples_per_segment,
    )
    if len(samples) < 2:
        raise KinematicsError("path_sample_invalid", "path produced fewer than two samples")
    actual_step = 0.0
    for first, second in zip(samples, samples[1:], strict=False):
        actual_step = max(
            actual_step,
            max(abs(a - b) for a, b in zip(first, second, strict=True)),
        )
    start_allowed = frozenset(allowed_contacts_at_start)
    end_allowed = frozenset(allowed_contacts_at_end)
    total_delta = np.abs(end - start)
    duration = float(minimum_duration_s)
    if max_velocities and len(max_velocities) == len(active_addresses):
        ratio = max(
            float(total_delta[address]) / velocity
            for address, velocity in zip(active_addresses, max_velocities, strict=True)
        )
        duration = max(duration, float(duration_peak_factor) * ratio)

    configuration = mink.Configuration(model)
    configuration.update(start)
    robot_minimum: ClearancePoint | None = None
    object_minimum: ClearancePoint | None = None
    failure_reason: str | None = None
    failure_message: str | None = None
    last_index = len(samples) - 1
    try:
        for index, sample in enumerate(samples):
            if deadline is not None and time.monotonic() > deadline:
                failure_reason = "time_budget_exceeded"
                failure_message = f"path {segment!r} exceeded the elapsed-time budget"
                break
            qpos = np.array(start, copy=True)
            for address, value in zip(active_addresses, sample, strict=True):
                qpos[address] = value
            configuration.update(qpos)
            deviation = _frozen_deviation(configuration.q, start, free_addresses)
            if deviation > 1e-8:
                failure_reason = "frozen_qpos_changed"
                failure_message = (
                    f"segment {segment!r} sample {index} moved a non-active DoF by "
                    f"{deviation:.3e}"
                )
                break
            data = configuration.data
            if carried is not None and object_pairs is not None:
                matrix = _block_matrix(
                    configuration,
                    carried,
                    qpos if carried.follow_sample else carried.holder_qpos,
                )
                _set_block(model, data, carried.envelope, matrix)
            point = measure_clearance(
                model,
                data,
                robot_pairs,
                detection_distance_m=clearance.collision_detection_distance_m,
                sample_index=index,
                fraction=index / last_index,
            )
            if robot_minimum is None or point.distance_m < robot_minimum.distance_m:
                robot_minimum = point
            if carried is not None and object_pairs is not None:
                allowed = frozenset()
                if index == 0:
                    allowed = start_allowed
                elif index == last_index:
                    allowed = end_allowed
                pairs = _filter_pairs(object_pairs, allowed)
                object_point = measure_clearance(
                    model,
                    data,
                    pairs,
                    detection_distance_m=clearance.collision_detection_distance_m,
                    sample_index=index,
                    fraction=index / last_index,
                )
                if (
                    object_minimum is None
                    or object_point.distance_m < object_minimum.distance_m
                ):
                    object_minimum = object_point
    except KinematicsError as error:
        failure_reason = error.reason
        failure_message = error.message
    if robot_minimum is None:
        robot_minimum = measure_clearance(
            model,
            configuration.data,
            robot_pairs,
            detection_distance_m=clearance.collision_detection_distance_m,
            sample_index=0,
            fraction=0.0,
        )
    passed = (
        failure_reason is None
        and robot_minimum.distance_m >= clearance.minimum_clearance_m
        and (
            object_minimum is None
            or object_minimum.distance_m >= clearance.minimum_clearance_m
        )
    )
    if not passed and failure_reason is None:
        failure_reason = "clearance_below_minimum"
        if robot_minimum.distance_m < clearance.minimum_clearance_m:
            failure_message = (
                f"segment {segment!r} sample {robot_minimum.sample_index} "
                f"({robot_minimum.geom1_name}/{robot_minimum.geom2_name}, "
                f"{robot_minimum.body1}/{robot_minimum.body2}) has robot clearance "
                f"{robot_minimum.distance_m:.6f} m < "
                f"{clearance.minimum_clearance_m:.6f} m"
            )
        else:
            assert object_minimum is not None
            failure_message = (
                f"segment {segment!r} sample {object_minimum.sample_index} "
                f"({object_minimum.geom1_name}/{object_minimum.geom2_name}, "
                f"{object_minimum.body1}/{object_minimum.body2}) has carried-object "
                f"clearance {object_minimum.distance_m:.6f} m < "
                f"{clearance.minimum_clearance_m:.6f} m"
            )
    return PathCheck(
        segment=segment,
        side=side,
        sample_count=len(samples),
        requested_max_step_rad=clearance.sample_step_rad,
        actual_max_step_rad=actual_step,
        duration_s=duration,
        robot_minimum=robot_minimum,
        object_minimum=object_minimum,
        passed=passed,
        elapsed_s=time.perf_counter() - started,
        failure_reason=failure_reason,
        failure_message=failure_message,
    )


def relative_block_pose(
    tcp_position: tuple[float, float, float],
    tcp_rotation: tuple[tuple[float, float, float], ...],
    block_center: tuple[float, float, float],
    block_rotation: tuple[tuple[float, float, float], ...] | None = None,
) -> tuple[tuple[float, float, float, float], ...]:
    """Return the world block pose in the grasp TCP frame."""
    import numpy as np

    rotation = np.asarray(tcp_rotation, dtype=np.float64)
    position = np.asarray(tcp_position, dtype=np.float64)
    center = np.asarray(block_center, dtype=np.float64)
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ position
    block = np.eye(4, dtype=np.float64)
    if block_rotation is not None:
        block[:3, :3] = np.asarray(block_rotation, dtype=np.float64)
    block[:3, 3] = center
    return tuple(tuple(float(value) for value in row) for row in inverse @ block)


def object_pair_set(
    model: object,
    object_names: tuple[str, ...],
    *,
    excluded_body_names: tuple[str, ...],
) -> CollisionPairSet:
    from robot_backend_sim.common.kinematics.kinematics_clearance import (
        build_object_collision_pairs,
    )

    return build_object_collision_pairs(
        model, object_names, excluded_body_names=excluded_body_names
    )


def robot_pair_set(
    model: object,
    *,
    robot_prefixes: tuple[str, ...],
    robot_sides: dict[str, tuple[str, ...]],
    object_names: tuple[str, ...],
    excluded_object_bodies: tuple[str, ...] = (),
) -> CollisionPairSet:
    """Build robot pairs and drop the allowed holding-finger/object pairs."""
    from robot_backend_sim.common.kinematics.kinematics_clearance import (
        build_collision_pairs,
    )

    full = build_collision_pairs(
        model,
        robot_prefixes=robot_prefixes,
        robot_sides=robot_sides,
        object_names=object_names,
    )
    if not excluded_object_bodies:
        return full
    excluded = frozenset(excluded_object_bodies)
    pairs = tuple(
        pair
        for pair in full.pairs
        if not (
            pair.category == "robot_object"
            and (pair.body1 in excluded or pair.body2 in excluded)
        )
    )
    categories: dict[str, int] = {}
    for pair in pairs:
        categories[pair.category] = categories.get(pair.category, 0) + 1
    return CollisionPairSet(
        pairs=pairs,
        categories=categories,
        exclusions=dict(full.exclusions),
        total_physical_pairs=full.total_physical_pairs,
        explicit_pair_count=full.explicit_pair_count,
    )
