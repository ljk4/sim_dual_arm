"""Measured-grasp lift steps with one checked command path.

The caller supplies immutable measurements and submits the returned joint
target.  IK, carried-object clearance and both grasp-frame error curves use
the same joint segment, on private kinematic configurations.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np

from robot_backend_sim.kinematics.common import sample_joint_path
from robot_backend_sim.kinematics.dual_arm import (
    BlockEnvelope,
    CarriedBlock,
    ClearanceParameters,
    IKParameters,
    KeyTarget,
    PathCheck,
    check_joint_path,
    object_pair_set,
    relative_block_pose,
    robot_pair_set,
    solve_targets,
)
from robot_backend_sim.kinematics.kinematics_clearance import KinematicsError
from robot_backend_sim.physics.marvin.scene import MarvinScene
from robot_backend_sim.tasks.handover.config import HandoverPlanConfig
from robot_backend_sim.tasks.handover.plan import CoordinationPoint, _coordination_errors

__all__ = ["CoholdReference", "CoholdStep", "plan_cohold_step"]


@dataclass(frozen=True, slots=True)
class CoholdReference:
    """One measured grasp, translated vertically during the lift."""

    block_center: tuple[float, float, float]
    block_rotation: tuple[tuple[float, float, float], ...]
    tcp_positions: Mapping[str, tuple[float, float, float]]
    tcp_rotations: Mapping[str, tuple[tuple[float, float, float], ...]]


@dataclass(frozen=True, slots=True)
class CoholdStep:
    qpos: tuple[float, ...]
    clearance: PathCheck
    coordination: tuple[CoordinationPoint, ...]
    failures: tuple[str, ...]

    @property
    def passed(self) -> bool:
        return self.clearance.passed and not self.failures


def _coordination_curve(
    scene: MarvinScene,
    config: HandoverPlanConfig,
    reference: CoholdReference,
    start: tuple[float, ...],
    end: tuple[float, ...],
    start_rise_m: float,
    end_rise_m: float,
) -> tuple[CoordinationPoint, ...]:
    chains = {chain.side: chain for chain in scene.robot_geometry.chains}
    relative = {
        side: np.asarray(
            relative_block_pose(
                reference.tcp_positions[side], reference.tcp_rotations[side],
                reference.block_center, reference.block_rotation,
            ),
            dtype=np.float64,
        )
        for side in ("left", "right")
    }
    samples = sample_joint_path(
        start, end, max_step_rad=config.clearance.sample_step_rad,
        max_samples=config.clearance.max_samples_per_segment,
    )
    curve = []
    for index, qpos in enumerate(samples):
        fraction = index / (len(samples) - 1)
        rise = start_rise_m + (end_rise_m - start_rise_m) * fraction
        center = np.asarray(reference.block_center) + np.array([0.0, 0.0, rise])
        errors = _coordination_errors(
            scene.model, tuple(qpos), left_site=chains["left"].tcp_site,
            right_site=chains["right"].tcp_site,
            block_center=tuple(float(value) for value in center), block_yaw_rad=0.0,
            left_relative=relative["left"], right_relative=relative["right"],
            block_rotation=reference.block_rotation,
        )
        curve.append(CoordinationPoint(fraction, *errors))
    return tuple(curve)


def plan_cohold_step(
    scene: MarvinScene,
    config: HandoverPlanConfig,
    reference: CoholdReference,
    *,
    measured_joints: Mapping[str, float],
    previous_command: tuple[float, ...],
    start_rise_m: float,
    end_rise_m: float,
    max_velocities: Mapping[str, float],
    first_step: bool,
) -> CoholdStep:
    """Plan from a measured IK seed and validate the commanded lift segment.

    Joint keys are compiled model names supplied by the robot geometry.  The
    measured state seeds IK; the segment starts at the previous command, which
    is the controller's reference.  No live scene data is read or written.
    """
    model, geometry = scene.model, scene.robot_geometry
    active_names = tuple(name for chain in geometry.chains for name in chain.joint_names)
    seed = np.array(previous_command, dtype=np.float64, copy=True)
    for name in active_names:
        seed[int(model.jnt_qposadr[model.joint(name).id])] = measured_joints[name]
    target_block = np.asarray(reference.block_center) + np.array([0.0, 0.0, end_rise_m])
    targets = tuple(
        KeyTarget(
            f"cohold_{chain.side}", chain.side,
            tuple(float(value) for value in target_block
                  + np.asarray(reference.tcp_positions[chain.side])
                  - np.asarray(reference.block_center)),
            reference.tcp_rotations[chain.side],
        )
        for chain in geometry.chains
    )
    ik = IKParameters(**config.ik.model_dump())
    clearance = ClearanceParameters(**config.clearance.model_dump())
    results = solve_targets(model, seed, targets, geometry.chains, ik, clearance, None)
    if not all(result.converged for result in results):
        details = [
            (result.target.name, result.reason)
            for result in results
            if not result.converged
        ]
        raise KinematicsError("cohold_ik_failed", f"cohold lift IK failed: {details}")
    qpos = results[-1].qpos
    fingers = tuple(body for chain in geometry.chains
                    for body, _joint in geometry.finger_bodies[chain.side])
    block_name = scene.config.task.block.name
    left_chain = next(chain for chain in geometry.chains if chain.side == "left")
    envelope = BlockEnvelope(
        block_name, int(model.body(block_name).id),
        int(model.geom(f"{block_name}_geom").id),
        int(scene.object_free_joint_qpos[block_name]), left_chain.tcp_site,
        relative_block_pose(reference.tcp_positions["left"],
                            reference.tcp_rotations["left"], reference.block_center,
                            reference.block_rotation),
    )
    check = check_joint_path(
        model, previous_command, previous_command, qpos,
        active_joint_names=active_names,
        robot_pairs=robot_pair_set(
            model, robot_prefixes=geometry.body_prefixes,
            robot_sides=dict(geometry.side_prefixes), object_names=(block_name,),
            excluded_object_bodies=fingers,
        ),
        object_pairs=object_pair_set(model, (block_name,), excluded_body_names=fingers),
        clearance=clearance, segment="cohold_lift_step", side="both",
        max_velocities=tuple(max_velocities[name] for name in active_names),
        carried=CarriedBlock(envelope, previous_command, True),
        allowed_contacts_at_start=scene.support_body_names[:1] if first_step else (),
        duration_peak_factor=config.execution.duration_peak_factor,
        minimum_duration_s=config.execution.minimum_segment_duration_s,
    )
    curve = _coordination_curve(
        scene, config, reference, previous_command, qpos, start_rise_m, end_rise_m,
    )
    failures = []
    for side in ("left", "right"):
        for metric, tolerance in (
            ("position_error_m", config.diagnostics.relative_position_tolerance_m),
            ("orientation_error_rad", config.diagnostics.relative_orientation_tolerance_rad),
        ):
            maximum = max(getattr(point, f"{side}_{metric}") for point in curve)
            if maximum > tolerance:
                failures.append(f"{side} {metric}: {maximum:.6f} > {tolerance:.6f}")
    return CoholdStep(qpos, check, curve, tuple(failures))
