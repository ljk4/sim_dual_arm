"""Right-arm stack targets, sequential IK and path/carried-object clearance."""

from __future__ import annotations

import math
from dataclasses import dataclass

import mujoco
import numpy as np

from robot_backend_sim.common.kinematics.common import ObjectPose
from robot_backend_sim.common.kinematics.kinematics_clearance import (
    ClearancePoint,
    CollisionPairSet,
    KinematicsError,
    measure_clearance,
)
from robot_backend_sim.marvin_sim.kinematics import (
    BlockEnvelope,
    CarriedBlock,
    ClearanceParameters,
    IKParameters,
    IKSolution,
    KeyTarget,
    PathCheck,
    check_joint_path,
    object_pair_set,
    relative_block_pose,
    robot_pair_set,
    solve_targets,
)
from robot_backend_sim.marvin_sim.scene_base import marvin_robot_geometry
from robot_backend_sim.marvin_sim.stack.kinematics_config import (
    ClearanceConfig,
    IKConfig,
    TargetConfig,
)

__all__ = [
    "BlockEnvelope",
    "CarriedBlock",
    "ClearancePoint",
    "CollisionPairSet",
    "IKResult",
    "KeyPoseTarget",
    "KinematicsError",
    "PathResult",
    "build_targets",
    "check_gripper_pose_clearance",
    "check_initial_pose",
    "check_path",
    "solve_key_poses",
    "stack_object_pairs",
    "stack_pair_set",
]

_RIGHT = "right"


@dataclass(frozen=True, slots=True)
class KeyPoseTarget:
    phase: str
    position: tuple[float, float, float]
    rotation: tuple[tuple[float, float, float], ...]

    @property
    def name(self) -> str:
        return self.phase


@dataclass(frozen=True, slots=True)
class IKResult:
    phase: str
    target: KeyPoseTarget
    converged: bool
    iterations: int
    position_error_m: float
    orientation_error_rad: float
    qpos: tuple[float, ...]
    qpos_right: tuple[float, ...]
    frozen_max_deviation: float
    elapsed_s: float
    reason: str | None = None
    message: str | None = None


@dataclass(frozen=True, slots=True)
class PathResult:
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

    @property
    def minimum(self) -> ClearancePoint:
        if self.object_minimum is None:
            return self.robot_minimum
        if self.object_minimum.distance_m < self.robot_minimum.distance_m:
            return self.object_minimum
        return self.robot_minimum


def _rotation_z(radians_value: float) -> np.ndarray:
    cosine, sine = math.cos(radians_value), math.sin(radians_value)
    return np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def _rotation_from_axes(
    approach: tuple[float, float, float], closing: tuple[float, float, float]
) -> np.ndarray:
    """Build Arm_R_tcp rotation matrix where local -Y approaches and +Z closes."""
    first = np.asarray(approach, dtype=np.float64)
    first /= np.linalg.norm(first)
    second = np.asarray(closing, dtype=np.float64)
    second -= first * float(np.dot(first, second))
    second /= np.linalg.norm(second)
    local_y = -first
    local_z = second
    local_x = np.cross(local_y, local_z)
    return np.column_stack([local_x, local_y, local_z])


def _as_tuple(matrix: np.ndarray) -> tuple[tuple[float, float, float], ...]:
    return tuple(tuple(float(value) for value in row) for row in matrix)


def _stack_rotation(yaw_rad: float, tilt_deg: float, yaw_offset_deg: float) -> np.ndarray:
    tilt = math.radians(tilt_deg)
    approach = (0.0, math.sin(tilt), -math.cos(tilt))
    closing = (1.0, 0.0, 0.0)
    return _rotation_z(yaw_rad + math.radians(yaw_offset_deg)) @ _rotation_from_axes(
        approach, closing
    )


def _clearance_params(clearance: ClearanceConfig) -> ClearanceParameters:
    return ClearanceParameters(
        minimum_clearance_m=clearance.minimum_clearance_m,
        collision_detection_distance_m=clearance.collision_detection_distance_m,
        sample_step_rad=clearance.sample_step_rad,
        max_samples_per_segment=clearance.max_samples_per_segment,
    )


def build_targets(
    source: ObjectPose,
    target: ObjectPose,
    *,
    table_top_z: float,
    source_half_z: float,
    target_half_z: float,
    config: TargetConfig,
) -> tuple[KeyPoseTarget, ...]:
    """Build the 6 world Arm_R_tcp targets for pick-and-stack."""
    sx, sy, _ = source.center
    tx, ty, _ = target.center

    grasp_rotation = _stack_rotation(
        source.yaw_rad, config.approach_tilt_deg, config.approach_yaw_deg
    )
    place_rotation = _stack_rotation(
        target.yaw_rad, config.approach_tilt_deg, config.place_yaw_deg
    )

    grasp_source_center = np.array(
        [sx, sy, table_top_z + source_half_z], dtype=np.float64
    )
    grasp_pos = np.array(
        [sx, sy, table_top_z + source_half_z + config.grasp_height_m], dtype=np.float64
    )
    local_tcp_from_source = grasp_rotation.T @ (grasp_pos - grasp_source_center)
    place_source_center = np.array(
        [
            tx,
            ty,
            table_top_z + 2.0 * target_half_z + source_half_z + config.place_clearance_m,
        ],
        dtype=np.float64,
    )
    place_pos = place_source_center + place_rotation @ local_tcp_from_source

    grasp_rot_t = _as_tuple(grasp_rotation)
    place_rot_t = _as_tuple(place_rotation)
    return (
        KeyPoseTarget(
            "pregrasp",
            (float(grasp_pos[0]), float(grasp_pos[1]), float(grasp_pos[2] + config.pregrasp_height_m)),
            grasp_rot_t,
        ),
        KeyPoseTarget(
            "grasp",
            (float(grasp_pos[0]), float(grasp_pos[1]), float(grasp_pos[2])),
            grasp_rot_t,
        ),
        KeyPoseTarget(
            "lift",
            (float(grasp_pos[0]), float(grasp_pos[1]), float(grasp_pos[2] + config.lift_height_m)),
            grasp_rot_t,
        ),
        KeyPoseTarget(
            "place_hover",
            (float(place_pos[0]), float(place_pos[1]), float(place_pos[2] + config.place_hover_height_m)),
            place_rot_t,
        ),
        KeyPoseTarget(
            "place",
            (float(place_pos[0]), float(place_pos[1]), float(place_pos[2])),
            place_rot_t,
        ),
        KeyPoseTarget(
            "retreat",
            (float(place_pos[0]), float(place_pos[1]), float(place_pos[2] + config.retreat_height_m)),
            place_rot_t,
        ),
    )


def stack_pair_set(
    model: mujoco.MjModel, *, excluded_object_bodies: tuple[str, ...] = ()
) -> CollisionPairSet:
    geometry = marvin_robot_geometry()
    return robot_pair_set(
        model,
        robot_prefixes=geometry.body_prefixes,
        robot_sides=dict(geometry.side_prefixes),
        object_names=("source", "target"),
        excluded_object_bodies=excluded_object_bodies,
    )


def stack_object_pairs(
    model: mujoco.MjModel, *, excluded_body_names: tuple[str, ...] = ()
) -> CollisionPairSet:
    return object_pair_set(
        model, ("source", "target"), excluded_body_names=excluded_body_names
    )


def solve_key_poses(
    model: mujoco.MjModel,
    initial_qpos: object,
    targets: tuple[KeyPoseTarget, ...],
    ik: IKConfig,
    clearance: ClearanceConfig,
    pair_set: CollisionPairSet | None = None,
    *,
    deadline: float | None = None,
) -> tuple[IKResult, ...]:
    geometry = marvin_robot_geometry()
    chains = {chain.side: chain for chain in geometry.chains}
    right_chain = chains[_RIGHT]
    right_addresses = tuple(
        int(model.jnt_qposadr[int(model.joint(name).id)]) for name in right_chain.joint_names
    )
    seed_qpos = np.array(initial_qpos, dtype=np.float64, copy=True)
    for address, value in zip(right_addresses, ik.initial_right_joint_positions, strict=True):
        seed_qpos[address] = value

    key_targets = tuple(
        KeyTarget(target.phase, _RIGHT, target.position, target.rotation)
        for target in targets
    )
    solutions: tuple[IKSolution, ...] = solve_targets(
        model,
        seed_qpos,
        key_targets,
        geometry.chains,
        IKParameters(
            dt_s=ik.dt_s,
            max_iterations=ik.max_iterations,
            position_tolerance_m=ik.position_tolerance_m,
            orientation_tolerance_rad=ik.orientation_tolerance_rad,
            posture_cost=ik.posture_cost,
            position_cost=ik.position_cost,
            orientation_cost=ik.orientation_cost,
            lm_damping=ik.lm_damping,
            solver=ik.solver,
            limit_margin_rad=ik.limit_margin_rad,
        ),
        _clearance_params(clearance),
        pair_set,
        deadline=deadline,
    )
    by_phase = {target.phase: target for target in targets}
    results: list[IKResult] = []
    for solution in solutions:
        qpos_right = tuple(float(solution.qpos[address]) for address in right_addresses)
        results.append(
            IKResult(
                phase=solution.target.name,
                target=by_phase[solution.target.name],
                converged=solution.converged,
                iterations=solution.iterations,
                position_error_m=solution.position_error_m,
                orientation_error_rad=solution.orientation_error_rad,
                qpos=solution.qpos,
                qpos_right=qpos_right,
                frozen_max_deviation=solution.frozen_max_deviation,
                elapsed_s=solution.elapsed_s,
                reason=solution.reason,
                message=solution.message,
            )
        )
    return tuple(results)


def check_initial_pose(
    model: mujoco.MjModel,
    initial_qpos: object,
    pair_set: CollisionPairSet,
    clearance: ClearanceConfig,
) -> PathResult:
    data = mujoco.MjData(model)
    data.qpos[:] = np.asarray(initial_qpos, dtype=np.float64)
    mujoco.mj_forward(model, data)
    point = measure_clearance(
        model,
        data,
        pair_set,
        detection_distance_m=clearance.collision_detection_distance_m,
        sample_index=0,
        fraction=0.0,
    )
    passed = point.distance_m >= clearance.minimum_clearance_m
    return PathResult(
        segment="initial",
        side=_RIGHT,
        sample_count=1,
        requested_max_step_rad=clearance.sample_step_rad,
        actual_max_step_rad=0.0,
        duration_s=0.0,
        robot_minimum=point,
        object_minimum=None,
        passed=passed,
        elapsed_s=0.0,
        failure_reason=None if passed else "clearance_below_minimum",
        failure_message=None
        if passed
        else (
            f"initial pose ({point.geom1_name}/{point.geom2_name}) clearance "
            f"{point.distance_m:.6f} m < {clearance.minimum_clearance_m:.6f} m"
        ),
    )


def check_path(
    model: mujoco.MjModel,
    initial_qpos: object,
    start_right_qpos: tuple[float, ...],
    end_right_qpos: tuple[float, ...],
    pair_set: CollisionPairSet,
    clearance: ClearanceConfig,
    *,
    segment: str,
    object_pairs: CollisionPairSet | None = None,
    max_velocities: tuple[float, ...] = (),
    carried: CarriedBlock | None = None,
    allowed_contacts_at_start: tuple[str, ...] = (),
    allowed_contacts_at_end: tuple[str, ...] = (),
    duration_peak_factor: float = 1.5,
    minimum_duration_s: float = 0.4,
    deadline: float | None = None,
) -> PathResult:
    geometry = marvin_robot_geometry()
    right_chain = next(chain for chain in geometry.chains if chain.side == _RIGHT)
    addresses = tuple(
        int(model.jnt_qposadr[int(model.joint(name).id)]) for name in right_chain.joint_names
    )
    start = np.array(initial_qpos, dtype=np.float64, copy=True)
    end = np.array(initial_qpos, dtype=np.float64, copy=True)
    for address, s_val, e_val in zip(addresses, start_right_qpos, end_right_qpos, strict=True):
        start[address] = s_val
        end[address] = e_val
    result: PathCheck = check_joint_path(
        model,
        initial_qpos,
        start,
        end,
        active_joint_names=right_chain.joint_names,
        robot_pairs=pair_set,
        object_pairs=object_pairs,
        clearance=_clearance_params(clearance),
        segment=segment,
        side=_RIGHT,
        max_velocities=max_velocities,
        carried=carried,
        allowed_contacts_at_start=allowed_contacts_at_start,
        allowed_contacts_at_end=allowed_contacts_at_end,
        duration_peak_factor=duration_peak_factor,
        minimum_duration_s=minimum_duration_s,
        deadline=deadline,
    )
    return PathResult(
        segment=result.segment,
        side=result.side,
        sample_count=result.sample_count,
        requested_max_step_rad=result.requested_max_step_rad,
        actual_max_step_rad=result.actual_max_step_rad,
        duration_s=result.duration_s,
        robot_minimum=result.robot_minimum,
        object_minimum=result.object_minimum,
        passed=result.passed,
        elapsed_s=result.elapsed_s,
        failure_reason=result.failure_reason,
        failure_message=result.failure_message,
    )


def check_gripper_pose_clearance(
    model: mujoco.MjModel,
    qpos: object,
    *,
    opening_m: float,
    gripper_open_total: float,
    pair_set: CollisionPairSet,
    clearance: ClearanceConfig,
    segment: str,
) -> PathResult:
    """Measure clearance at a static right-arm qpos with a specific gripper opening.

    Marvin finger slides move from 0 (open) to ``gripper_open_total`` total closure,
    so each prismatic slide takes ``(gripper_open_total - opening_m) / 2``.
    """
    geometry = marvin_robot_geometry()
    per_finger = max(0.0, (float(gripper_open_total) - float(opening_m)) / 2.0)
    data = mujoco.MjData(model)
    data.qpos[:] = np.asarray(qpos, dtype=np.float64)
    for _body_name, joint_name in geometry.finger_bodies[_RIGHT]:
        address = int(model.jnt_qposadr[int(model.joint(joint_name).id)])
        data.qpos[address] = per_finger
    mujoco.mj_forward(model, data)
    point = measure_clearance(
        model,
        data,
        pair_set,
        detection_distance_m=clearance.collision_detection_distance_m,
        sample_index=0,
        fraction=0.0,
    )
    passed = point.distance_m >= clearance.minimum_clearance_m
    return PathResult(
        segment=segment,
        side=_RIGHT,
        sample_count=1,
        requested_max_step_rad=clearance.sample_step_rad,
        actual_max_step_rad=0.0,
        duration_s=0.0,
        robot_minimum=point,
        object_minimum=None,
        passed=passed,
        elapsed_s=0.0,
        failure_reason=None if passed else "clearance_below_minimum",
        failure_message=None
        if passed
        else (
            f"segment {segment!r} ({point.geom1_name}/{point.geom2_name}) clearance "
            f"{point.distance_m:.6f} m < {clearance.minimum_clearance_m:.6f} m"
        ),
    )
