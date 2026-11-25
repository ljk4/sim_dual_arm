"""Task goals and candidate paths for the Marvin handover first task.

The module builds named world TCP targets from one immutable layout snapshot,
solves them with the robot's own chains, and checks the connecting joint paths
against robot/environment and carried-block geometry.  It writes no scene state
and produces the same immutable plan for the diagnostics entry and the P03
teacher.
"""

from __future__ import annotations

import hashlib
import math
import time
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from robot_backend_sim.common.kinematics.common import sample_joint_path
from robot_backend_sim.common.kinematics.kinematics_clearance import ClearancePoint, KinematicsError
from robot_backend_sim.common.physics.robot_geometry import ArmChain
from robot_backend_sim.common.tasks.math import (
    matrix_as_tuple as _as_tuple,
    rotation_from_axes as _rotation_from_axes,
    rotation_z as _rotation_z,
)
from robot_backend_sim.marvin_sim.handover.config import HandoverPlanConfig
from robot_backend_sim.marvin_sim.handover.scene import MarvinScene
from robot_backend_sim.marvin_sim.handover.scene_config import MarvinSceneConfig
from robot_backend_sim.marvin_sim.kinematics import (
    BlockEnvelope,
    CarriedBlock,
    ClearanceParameters,
    IKParameters,
    IKSolution,
    KeyTarget,
    check_joint_path,
    object_pair_set,
    relative_block_pose,
    robot_pair_set,
    solve_targets,
    tcp_matrix,
)
from robot_backend_sim.marvin_sim.scene_base import base_world_point

__all__ = [
    "CoordinationPoint",
    "HandoverLayout",
    "HandoverPlan",
    "PlanSegment",
    "PlanTarget",
    "build_handover_targets",
    "layout_from_config",
    "layout_from_scene",
    "perturb_handover_plan_config",
    "plan_handover",
]

_LEFT = "left"
_RIGHT = "right"


@dataclass(frozen=True, slots=True)
class HandoverLayout:
    base_position: tuple[float, float, float]
    base_yaw_deg: float
    table_top_z: float
    block_name: str
    block_half_extents: tuple[float, float, float]
    block_initial_center: tuple[float, float, float]
    block_initial_yaw_rad: float
    box_name: str
    box_center: tuple[float, float, float]
    box_inner_xy: tuple[float, float]
    box_inner_bottom_z: float
    box_rim_z: float
    box_yaw_rad: float


@dataclass(frozen=True, slots=True)
class PlanTarget:
    name: str
    side: str
    position: tuple[float, float, float]
    rotation: tuple[tuple[float, float, float], ...]
    qpos: tuple[float, ...]
    converged: bool
    position_error_m: float
    orientation_error_rad: float
    iterations: int
    reason: str | None
    message: str | None


@dataclass(frozen=True, slots=True)
class PlanSegment:
    name: str
    side: str
    start_target: str
    end_target: str
    duration_s: float
    sample_count: int
    passed: bool
    robot_minimum: ClearancePoint
    object_minimum: ClearancePoint | None
    failure_reason: str | None
    failure_message: str | None


@dataclass(frozen=True, slots=True)
class CoordinationPoint:
    fraction: float
    left_position_error_m: float
    left_orientation_error_rad: float
    right_position_error_m: float
    right_orientation_error_rad: float


@dataclass(frozen=True, slots=True)
class HandoverPlan:
    layout: HandoverLayout
    targets: tuple[PlanTarget, ...]
    segments: tuple[PlanSegment, ...]
    placement_contained: bool
    placement_message: str
    planned_duration_s: float
    planning_elapsed_s: float
    coordination: tuple[CoordinationPoint, ...] = field(default_factory=tuple)
    coordinated_waypoints: tuple[tuple[float, ...], ...] = field(default_factory=tuple)
    coordination_summary: dict[str, object] = field(default_factory=dict)
    failures: tuple[str, ...] = field(default_factory=tuple)
    teacher_waypoints: dict[str, tuple[float, ...]] = field(default_factory=dict)

    @property
    def passed(self) -> bool:
        return (
            not self.failures
            and all(target.converged for target in self.targets)
            and all(segment.passed for segment in self.segments)
        )


def _joint_velocities(
    max_velocities: dict[str, tuple[float, ...]] | None, side: str
) -> tuple[float, ...]:
    if max_velocities is None or side not in max_velocities:
        return ()
    return tuple(max_velocities[side])


def layout_from_scene(scene: MarvinScene) -> HandoverLayout:
    """Read one immutable task layout from the compiled handover scene."""
    config = scene.config
    address = int(scene.object_free_joint_qpos[config.task.block.name])
    return layout_from_config(config, scene.data.qpos, address)


def layout_from_config(
    config: MarvinSceneConfig,
    qpos: object,
    block_qpos_address: int,
    *,
    center_tolerance_m: float = 0.002,
) -> HandoverLayout:
    """Read the same layout from a frozen snapshot instead of live ``MjData``.

    The declared geometry is the single source for targets.  The live block
    pose is only checked against it within a settle tolerance, because a reset
    followed by a few physics steps lets the free block seat on the table.
    """
    base = config.robot
    table = config.task.table
    block = config.task.block
    box = config.task.box
    table_top = base.base_position[2] + table.top_z
    block_center = base_world_point(
        (block.initial_xy[0], block.initial_xy[1], table.top_z + block.half_extents[2]),
        base.base_position,
        base.base_yaw_deg,
    )
    box_center = base_world_point(
        (box.center_xy[0], box.center_xy[1], table.top_z),
        base.base_position,
        base.base_yaw_deg,
    )
    actual = [
        float(value)
        for value in qpos[block_qpos_address : block_qpos_address + 7]  # type: ignore[index]
    ]
    if any(
        abs(a - b) > center_tolerance_m
        for a, b in zip(actual[:3], block_center, strict=True)
    ):
        raise KinematicsError(
            "layout_mismatch", "scene block qpos does not match the declared initial layout"
        )
    # Use the settled pose as truth for grasp targets; the declared box and
    # table geometry remain the single source for the environment.
    block_center = (actual[0], actual[1], actual[2])
    return HandoverLayout(
        base_position=tuple(float(value) for value in base.base_position),
        base_yaw_deg=float(base.base_yaw_deg),
        table_top_z=table_top,
        block_name=block.name,
        block_half_extents=tuple(float(value) for value in block.half_extents),
        block_initial_center=block_center,
        block_initial_yaw_rad=math.radians(base.base_yaw_deg + block.initial_yaw_deg),
        box_name=box.name,
        box_center=box_center,
        box_inner_xy=tuple(float(value) for value in box.inner_xy),
        box_inner_bottom_z=table_top + box.bottom_thickness,
        box_rim_z=table_top + box.height,
        box_yaw_rad=math.radians(base.base_yaw_deg + box.yaw_deg),
    )


def _handover_center(layout: HandoverLayout, config: HandoverPlanConfig) -> np.ndarray:
    offset = np.asarray(config.handover.block_center_offset_m, dtype=np.float64)
    return np.asarray(layout.block_initial_center, dtype=np.float64) + _rotation_z(
        math.radians(layout.base_yaw_deg)
    ) @ offset


def _side_grasp_rotation(
    layout: HandoverLayout, tilt_deg: float, yaw_deg: float
) -> np.ndarray:
    tilt = math.radians(tilt_deg)
    approach = (0.0, math.sin(tilt), -math.cos(tilt))
    closing = (1.0, 0.0, 0.0)
    return _rotation_z(layout.block_initial_yaw_rad + math.radians(yaw_deg)) @ (
        _rotation_from_axes(approach, closing)
    )


def build_handover_targets(
    layout: HandoverLayout,
    config: HandoverPlanConfig,
    chains: tuple[ArmChain, ...],
) -> tuple[tuple[KeyTarget, ...], dict[str, Any]]:
    """Return every named target plus the rigid poses behind them."""
    sides = {chain.side for chain in chains}
    if {_LEFT, _RIGHT} - sides:  # pragma: no cover - adapter guard
        raise KinematicsError("robot_geometry_invalid", "both arm chains are required")
    grasp = config.grasp
    above = grasp.tcp_above_block_center_m
    block_center = np.asarray(layout.block_initial_center, dtype=np.float64)
    handover_center = _handover_center(layout, config)
    left_rotation = _side_grasp_rotation(
        layout, grasp.left.approach_tilt_deg, grasp.left.approach_yaw_deg
    )
    right_center_rotation = _side_grasp_rotation(
        layout, grasp.right_center.approach_tilt_deg, grasp.right_center.approach_yaw_deg
    )
    left_cohold_rotation = _side_grasp_rotation(
        layout, grasp.left_cohold.approach_tilt_deg, grasp.left_cohold.approach_yaw_deg
    )
    right_cohold_rotation = _side_grasp_rotation(
        layout, grasp.right_cohold.approach_tilt_deg, grasp.right_cohold.approach_yaw_deg
    )
    block_offset = _rotation_z(layout.block_initial_yaw_rad)
    left_offset = block_offset @ np.array(
        [0.0, grasp.left.along_block_y_m, above], dtype=np.float64
    )
    right_center_offset = block_offset @ np.array(
        [0.0, grasp.right_center.along_block_y_m, above], dtype=np.float64
    )
    right_handover_offset = block_offset @ np.array(
        [0.0, grasp.right_cohold.along_block_y_m, above], dtype=np.float64
    )

    # Rigid block pose in the right TCP frame at any grasp with this
    # orientation.  The place pose rotates that hand about the block z axis.
    right_local_offset = right_cohold_rotation.T @ (-right_handover_offset)
    place_rotation = (
        _rotation_z(math.radians(config.transport.place_yaw_deg)) @ right_cohold_rotation
    )
    place_world_offset = place_rotation @ right_local_offset
    place_xy_delta = _rotation_z(math.radians(layout.base_yaw_deg)) @ np.array(
        [
            config.transport.place_xy_offset_m[0],
            config.transport.place_xy_offset_m[1],
            0.0,
        ],
        dtype=np.float64,
    )
    place_center = np.array(
        [
            layout.box_center[0] + place_xy_delta[0],
            layout.box_center[1] + place_xy_delta[1],
            layout.box_inner_bottom_z + layout.block_half_extents[2],
        ],
        dtype=np.float64,
    )
    preplace_center = place_center.copy()
    preplace_center[2] = (
        layout.box_rim_z + config.transport.preplace_block_center_above_rim_m
    )
    retreat = base_world_point(
        tuple(config.retreat.left_tcp_position_m),
        layout.base_position,
        layout.base_yaw_deg,
    )
    left_ready = base_world_point(
        tuple(config.approach.left_ready_tcp_m),
        layout.base_position,
        layout.base_yaw_deg,
    )
    right_wait = base_world_point(
        tuple(config.approach.right_wait_tcp_m),
        layout.base_position,
        layout.base_yaw_deg,
    )
    right_wait_rotation = _side_grasp_rotation(
        layout, config.approach.right_wait_tilt_deg, 0.0
    )

    def left_target(name: str, center: np.ndarray, lift: float) -> KeyTarget:
        position = center + block_offset @ np.array(
            [0.0, grasp.left.along_block_y_m, above + lift], dtype=np.float64
        )
        return KeyTarget(
            name, _LEFT, tuple(float(value) for value in position), _as_tuple(left_rotation)
        )

    def right_center_target(name: str, center: np.ndarray, lift: float) -> KeyTarget:
        position = center + block_offset @ np.array(
            [0.0, grasp.right_center.along_block_y_m, above + lift], dtype=np.float64
        )
        return KeyTarget(
            name,
            _RIGHT,
            tuple(float(value) for value in position),
            _as_tuple(right_center_rotation),
        )

    def left_cohold_target(name: str, lift: float) -> KeyTarget:
        position = block_center + block_offset @ np.array(
            [0.0, grasp.left_cohold.along_block_y_m, above + lift], dtype=np.float64
        )
        return KeyTarget(
            name,
            _LEFT,
            tuple(float(value) for value in position),
            _as_tuple(left_cohold_rotation),
        )

    def right_cohold_target(name: str, lift: float) -> KeyTarget:
        position = block_center + block_offset @ np.array(
            [0.0, grasp.right_cohold.along_block_y_m, above + lift], dtype=np.float64
        )
        return KeyTarget(
            name,
            _RIGHT,
            tuple(float(value) for value in position),
            _as_tuple(right_cohold_rotation),
        )

    def right_handover_target(name: str, lift: float) -> KeyTarget:
        position = handover_center + right_handover_offset + np.array([0.0, 0.0, lift])
        return KeyTarget(
            name, _RIGHT, tuple(float(value) for value in position),
            _as_tuple(right_cohold_rotation),
        )

    targets = (
        KeyTarget("left_ready", _LEFT, left_ready, _as_tuple(left_rotation)),
        left_target("left_pregrasp", block_center, grasp.pregrasp_height_m),
        left_target("left_grasp", block_center, 0.0),
        left_target("left_lift", block_center, grasp.lift_height_m),
        left_target("left_handover", handover_center, 0.0),
        KeyTarget("right_handover_wait", _RIGHT, right_wait, _as_tuple(right_wait_rotation)),
        right_handover_target("right_handover_pregrasp", grasp.pregrasp_height_m),
        right_handover_target("right_handover_grasp", 0.0),
        KeyTarget("left_retreat", _LEFT, retreat, _as_tuple(left_rotation)),
        KeyTarget(
            "right_preplace",
            _RIGHT,
            tuple(float(value) for value in preplace_center - place_world_offset),
            _as_tuple(place_rotation),
        ),
        KeyTarget(
            "right_place",
            _RIGHT,
            tuple(float(value) for value in place_center - place_world_offset),
            _as_tuple(place_rotation),
        ),
        KeyTarget("right_wait_initial", _RIGHT, right_wait, _as_tuple(right_wait_rotation)),
        right_center_target(
            "right_initial_pregrasp", block_center, grasp.right_initial_pregrasp_height_m
        ),
        right_center_target("right_center_initial", block_center, 0.0),
        right_center_target(
            "right_center_lift", block_center, grasp.right_initial_lift_height_m
        ),
        left_cohold_target("left_cohold_pregrasp", grasp.pregrasp_height_m),
        left_cohold_target("left_cohold_grasp", 0.0),
        KeyTarget("right_cohold_wait", _RIGHT, right_wait, _as_tuple(right_wait_rotation)),
        right_cohold_target("right_cohold_pregrasp", grasp.right_initial_pregrasp_height_m),
        right_cohold_target("right_cohold_grasp", 0.0),
        KeyTarget("left_cohold_retreat", _LEFT, retreat, _as_tuple(left_rotation)),
        left_cohold_target("left_cohold_lift", grasp.cohold_lift_height_m),
        right_cohold_target("right_cohold_lift", grasp.cohold_lift_height_m),
        KeyTarget("left_air_retreat", _LEFT, retreat, _as_tuple(left_rotation)),
    )
    context = {
        "initial_block_center": tuple(float(value) for value in block_center),
        "handover_block_center": tuple(float(value) for value in handover_center),
        "place_block_center": tuple(float(value) for value in place_center),
        "preplace_block_center": tuple(float(value) for value in preplace_center),
        "left_grasp_offset_world": tuple(float(value) for value in left_offset),
        "right_center_grasp_offset_world": tuple(float(value) for value in right_center_offset),
        "right_handover_grasp_offset_world": tuple(float(value) for value in right_handover_offset),
        "place_rotation": _as_tuple(place_rotation),
        "place_world_offset": tuple(float(value) for value in place_world_offset),
    }
    return targets, context


def _signed_uniform(seed: int, attempt: int, index: int) -> float:
    """Deterministic uniform sample in [-1, 1] for one seed/attempt slot."""
    digest = hashlib.sha256(
        f"handover_diversity:{seed}:{attempt}:{index}".encode()
    ).digest()
    return int.from_bytes(digest[:8], "big") / ((1 << 64) - 1) * 2.0 - 1.0


def perturb_handover_plan_config(
    config: HandoverPlanConfig, seed: int, attempt: int = 0
) -> tuple[HandoverPlanConfig, float]:
    """Return a seed-deterministic recipe variant and its speed scale.

    Spatial jitter decays with each retry so a failed attempt re-solves near the
    nominal recipe; the speed scale stays seed-specific even when the offsets
    have fully decayed, so every seed has distinct timing.
    """
    diversity = config.diversity
    if not diversity.enabled:
        return config, 1.0
    u = [_signed_uniform(int(seed), int(attempt), index) for index in range(20)]
    alpha = 0.0 if attempt >= diversity.max_attempts - 1 else 0.7 ** attempt

    handover_base = config.handover.block_center_offset_m
    handover_range = diversity.handover_offset_range_m
    dz = alpha * u[2] * handover_range[2]
    handover = config.handover.model_copy(
        update={
            "block_center_offset_m": (
                float(handover_base[0]) + alpha * u[0] * handover_range[0],
                float(handover_base[1])
                + alpha * (u[1] * 0.65 - 0.35 * abs(u[1])) * handover_range[1],
                float(handover_base[2]) + dz,
            )
        }
    )

    left_base = config.approach.left_ready_tcp_m
    right_base = config.approach.right_wait_tcp_m
    left_range = diversity.left_ready_offset_range_m
    right_range = diversity.right_wait_offset_range_m
    approach = config.approach.model_copy(
        update={
            "left_ready_tcp_m": (
                float(left_base[0]) + alpha * u[3] * left_range[0],
                float(left_base[1]) + alpha * u[4] * left_range[1],
                float(left_base[2]) + alpha * u[5] * left_range[2],
            ),
            "right_wait_tcp_m": (
                float(right_base[0]) + alpha * u[6] * right_range[0],
                float(right_base[1]) + alpha * u[7] * right_range[1],
                float(right_base[2]) + alpha * u[8] * right_range[2],
            ),
        }
    )

    grasp = config.grasp
    pregrasp = max(
        0.028,
        float(grasp.pregrasp_height_m)
        + alpha * u[9] * diversity.pregrasp_height_range_m,
    )
    left_grasp = grasp.left.model_copy(
        update={
            "along_block_y_m": min(
                0.072,
                max(
                    0.048,
                    float(grasp.left.along_block_y_m)
                    + alpha * u[10] * diversity.grasp_along_y_range_m,
                ),
            ),
            "approach_yaw_deg": float(grasp.left.approach_yaw_deg)
            + alpha * u[12] * diversity.wrist_yaw_range_deg,
        }
    )
    right_cohold = grasp.right_cohold.model_copy(
        update={
            "along_block_y_m": max(
                -0.072,
                min(
                    -0.048,
                    float(grasp.right_cohold.along_block_y_m)
                    + alpha * u[11] * diversity.grasp_along_y_range_m,
                ),
            ),
            "approach_tilt_deg": float(grasp.right_cohold.approach_tilt_deg)
            + alpha * u[13] * diversity.wrist_tilt_range_deg,
            "approach_yaw_deg": float(grasp.right_cohold.approach_yaw_deg)
            + alpha * u[14] * diversity.wrist_yaw_range_deg,
        }
    )
    grasp = grasp.model_copy(
        update={
            "pregrasp_height_m": pregrasp,
            "left": left_grasp,
            "right_cohold": right_cohold,
        }
    )

    transport = config.transport
    place_xy_range = diversity.place_xy_range_m
    transport = transport.model_copy(
        update={
            "place_xy_offset_m": (
                alpha * u[15] * place_xy_range[0],
                alpha * u[16] * place_xy_range[1],
            ),
            "place_yaw_deg": float(transport.place_yaw_deg)
            + alpha * u[17] * diversity.place_yaw_range_deg,
            "preplace_block_center_above_rim_m": max(
                0.055,
                float(transport.preplace_block_center_above_rim_m)
                + alpha * u[18] * diversity.preplace_height_range_m,
            ),
        }
    )

    low, high = diversity.speed_scale_range
    speed_scale = float(low) + (u[19] + 1.0) / 2.0 * (float(high) - float(low))
    perturbed = config.model_copy(
        update={
            "handover": handover,
            "approach": approach,
            "grasp": grasp,
            "transport": transport,
        }
    )
    return perturbed, speed_scale


def _plan_targets(results: tuple[IKSolution, ...]) -> dict[str, PlanTarget]:
    return {
        result.target.name: PlanTarget(
            name=result.target.name,
            side=result.target.side,
            position=result.target.position,
            rotation=result.target.rotation,
            qpos=result.qpos,
            converged=result.converged,
            position_error_m=result.position_error_m,
            orientation_error_rad=result.orientation_error_rad,
            iterations=result.iterations,
            reason=result.reason,
            message=result.message,
        )
        for result in results
    }


def _inverse_pose(matrix: np.ndarray) -> np.ndarray:
    rotation = matrix[:3, :3]
    inverse = np.eye(4, dtype=np.float64)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ matrix[:3, 3]
    return inverse


def _pose_error(actual: np.ndarray, expected: np.ndarray) -> tuple[float, float]:
    position = float(np.linalg.norm(actual[:3, 3] - expected[:3, 3]))
    rotation = (float(np.trace(actual[:3, :3].T @ expected[:3, :3])) - 1.0) / 2.0
    return position, math.acos(max(-1.0, min(1.0, rotation)))


def _coordination_errors(
    model: object,
    qpos: tuple[float, ...],
    *,
    left_site: str,
    right_site: str,
    block_center: tuple[float, float, float],
    block_yaw_rad: float,
    left_relative: np.ndarray,
    right_relative: np.ndarray,
    block_rotation: tuple[tuple[float, float, float], ...] | None = None,
) -> tuple[float, float, float, float]:
    block = np.eye(4, dtype=np.float64)
    block[:3, :3] = _rotation_z(block_yaw_rad) if block_rotation is None else block_rotation
    block[:3, 3] = np.asarray(block_center, dtype=np.float64)
    left_tcp = np.asarray(tcp_matrix(model, qpos, left_site), dtype=np.float64)
    right_tcp = np.asarray(tcp_matrix(model, qpos, right_site), dtype=np.float64)
    left_error = _pose_error(_inverse_pose(left_tcp) @ block, left_relative)
    right_error = _pose_error(_inverse_pose(right_tcp) @ block, right_relative)
    return left_error[0], left_error[1], right_error[0], right_error[1]


def _sample_path_qpos(
    start: tuple[float, ...],
    end: tuple[float, ...],
    step_rad: float,
    max_samples: int,
) -> tuple[tuple[float, ...], ...]:
    from robot_backend_sim.common.kinematics.common import sample_joint_path

    return sample_joint_path(start, end, max_step_rad=step_rad, max_samples=max_samples)


def _waypoint_lift(
    model: object,
    start_qpos: tuple[float, ...],
    *,
    chains: tuple[ArmChain, ...],
    ik: IKParameters,
    clearance: ClearanceParameters,
    block_center: np.ndarray,
    block_yaw_rad: float,
    left_grasp: object,
    right_grasp: object,
    lift_height: float,
    left_site: str,
    right_site: str,
    left_relative: np.ndarray,
    right_relative: np.ndarray,
    steps: int,
    sample_step_rad: float,
    max_samples: int,
    deadline: float | None,
) -> tuple[tuple[tuple[float, ...], ...], tuple[CoordinationPoint, ...]]:
    """Solve a coordinated lift through block-translated waypoints.

    Both TCP targets are derived from the same block pose at every waypoint;
    the left and right IK are solved sequentially from the previous accepted
    state.  The returned pieces are the waypoint joint states and the sampled
    relative-pose error curves for both TCPs against that common block pose.
    """
    from robot_backend_sim.common.kinematics.common import sample_joint_path

    left_offset = np.asarray(left_grasp.position) - block_center
    right_offset = np.asarray(right_grasp.position) - block_center
    waypoints: list[tuple[float, ...]] = [tuple(start_qpos)]
    curve: list[CoordinationPoint] = []
    current = tuple(start_qpos)
    for index in range(1, steps + 1):
        fraction = index / steps
        center = np.asarray(block_center, dtype=np.float64) + np.array(
            [0.0, 0.0, lift_height * fraction]
        )
        left_position = center + left_offset
        right_position = center + right_offset
        targets = (
            KeyTarget(
                "coord_left", _LEFT, tuple(float(v) for v in left_position), left_grasp.rotation
            ),
            KeyTarget(
                "coord_right", _RIGHT, tuple(float(v) for v in right_position), right_grasp.rotation
            ),
        )
        results = solve_targets(
            model, current, targets, chains, ik, clearance, None, deadline=deadline
        )
        if not all(result.converged for result in results):
            break
        current = results[-1].qpos
        waypoints.append(current)
        errors = _coordination_errors(
            model,
            current,
            left_site=left_site,
            right_site=right_site,
            block_center=tuple(float(v) for v in center),
            block_yaw_rad=block_yaw_rad,
            left_relative=left_relative,
            right_relative=right_relative,
        )
        curve.append(
            CoordinationPoint(
                fraction=fraction,
                left_position_error_m=errors[0],
                left_orientation_error_rad=errors[1],
                right_position_error_m=errors[2],
                right_orientation_error_rad=errors[3],
            )
        )
    # Sample the joint-linear pieces between waypoints as well; the execution
    # follows these pieces, so the curve must cover them, not only the knots.
    sampled_curve: list[CoordinationPoint] = []
    total = len(waypoints) - 1
    if total > 0:
        for index in range(total):
            samples = sample_joint_path(
                waypoints[index],
                waypoints[index + 1],
                max_step_rad=sample_step_rad,
                max_samples=max_samples,
            )
            for sub_index, qpos in enumerate(samples):
                fraction = (index + sub_index / (len(samples) - 1)) / total
                center = np.asarray(block_center, dtype=np.float64) + np.array(
                    [0.0, 0.0, lift_height * fraction]
                )
                errors = _coordination_errors(
                    model,
                    tuple(float(v) for v in qpos),
                    left_site=left_site,
                    right_site=right_site,
                    block_center=tuple(float(v) for v in center),
                    block_yaw_rad=block_yaw_rad,
                    left_relative=left_relative,
                    right_relative=right_relative,
                )
                sampled_curve.append(
                    CoordinationPoint(
                        fraction=fraction,
                        left_position_error_m=errors[0],
                        left_orientation_error_rad=errors[1],
                        right_position_error_m=errors[2],
                        right_orientation_error_rad=errors[3],
                    )
                )
    return tuple(waypoints), tuple(sampled_curve)


def _placement_containment(
    layout: HandoverLayout,
    block_center: np.ndarray,
    block_rotation: np.ndarray,
    half_extents: tuple[float, float, float],
) -> tuple[bool, str]:
    half = np.asarray(half_extents, dtype=np.float64)
    corners = np.array(
        [
            [sx * half[0], sy * half[1], sz * half[2]]
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
            for sz in (-1.0, 1.0)
        ],
        dtype=np.float64,
    )
    world = corners @ block_rotation.T + block_center
    relative = (world - np.asarray(layout.box_center, dtype=np.float64)) @ _rotation_z(
        -layout.box_yaw_rad
    ).T
    inner_x, inner_y = layout.box_inner_xy
    # A placed block rests on the inner bottom; allow 1 mm of contact-scale
    # penetration from IK/kinematic rounding, not a free vertical offset.
    vertical_tolerance = 1e-3
    above_bottom = world[:, 2] >= layout.box_inner_bottom_z - vertical_tolerance
    below_rim = world[:, 2] <= layout.box_rim_z - 1e-6
    inside_xy = (np.abs(relative[:, 0]) <= inner_x / 2.0 + 1e-6) & (
        np.abs(relative[:, 1]) <= inner_y / 2.0 + 1e-6
    )
    passed = bool(np.all(above_bottom & below_rim & inside_xy))
    margins = (
        inner_x / 2.0 - float(np.max(np.abs(relative[:, 0]))),
        inner_y / 2.0 - float(np.max(np.abs(relative[:, 1]))),
        float(np.min(world[:, 2])) - layout.box_inner_bottom_z,
        layout.box_rim_z - float(np.max(world[:, 2])),
    )
    message = (
        "block fully inside the box"
        if passed
        else "block is not fully inside the box"
    ) + (
        f" (x={margins[0]:.5f} m, y={margins[1]:.5f} m, "
        f"bottom={margins[2]:.5f} m, rim={margins[3]:.5f} m)"
    )
    return passed, message


def plan_handover(
    scene: MarvinScene,
    config: HandoverPlanConfig,
    *,
    max_velocities: dict[str, tuple[float, ...]] | None = None,
    deadline: float | None = None,
    teacher_only: bool = False,
) -> HandoverPlan:
    """Build and check every candidate target and connecting path."""
    started = time.perf_counter()
    address = int(scene.object_free_joint_qpos[scene.config.task.block.name])
    layout = layout_from_config(scene.config, scene.data.qpos, address)
    model = scene.model
    geometry = scene.robot_geometry
    chains = geometry.chains
    side_fingers = {
        side: tuple(body for body, _joint in geometry.finger_bodies[side])
        for side in (_LEFT, _RIGHT)
    }
    targets, context = build_handover_targets(layout, config, chains)
    ik = IKParameters(
        dt_s=config.ik.dt_s,
        max_iterations=config.ik.max_iterations,
        position_tolerance_m=config.ik.position_tolerance_m,
        orientation_tolerance_rad=config.ik.orientation_tolerance_rad,
        posture_cost=config.ik.posture_cost,
        position_cost=config.ik.position_cost,
        orientation_cost=config.ik.orientation_cost,
        lm_damping=config.ik.lm_damping,
        solver=config.ik.solver,
        limit_margin_rad=config.ik.limit_margin_rad,
    )
    clearance = ClearanceParameters(
        minimum_clearance_m=config.clearance.minimum_clearance_m,
        collision_detection_distance_m=config.clearance.collision_detection_distance_m,
        sample_step_rad=config.clearance.sample_step_rad,
        max_samples_per_segment=config.clearance.max_samples_per_segment,
    )
    initial_qpos = tuple(float(value) for value in scene.data.qpos)
    by_name = {target.name: target for target in targets}
    sequence_a = tuple(
        by_name[name]
        for name in (
            "left_ready",
            "right_handover_wait",
            "left_pregrasp",
            "left_grasp",
            "left_lift",
            "left_handover",
            "right_handover_pregrasp",
            "right_handover_grasp",
            "left_retreat",
            "right_preplace",
            "right_place",
        )
    )
    sequence_b = tuple(
        by_name[name]
        for name in (
            "right_wait_initial",
            "right_initial_pregrasp",
            "right_center_initial",
            "right_center_lift",
        )
    )
    sequence_c = tuple(
        by_name[name]
        for name in (
            "left_ready",
            "left_cohold_pregrasp",
            "left_cohold_grasp",
            "right_cohold_wait",
            "right_cohold_pregrasp",
            "right_cohold_grasp",
            "left_cohold_retreat",
            "left_cohold_lift",
            "right_cohold_lift",
            "left_air_retreat",
        )
    )
    solved = _plan_targets(
        solve_targets(
            model, initial_qpos, sequence_a, chains, ik, clearance, None, deadline=deadline
        )
    )
    solved_b: dict[str, PlanTarget] = {}
    solved_c: dict[str, PlanTarget] = {}
    if not teacher_only:
        solved_b = _plan_targets(
            solve_targets(
                model, initial_qpos, sequence_b, chains, ik, clearance, None, deadline=deadline
            )
        )
        solved_c = _plan_targets(
            solve_targets(
                model, initial_qpos, sequence_c, chains, ik, clearance, None, deadline=deadline
            )
        )
    all_names = [target.name for target in sequence_a]
    if not teacher_only:
        all_names.extend(target.name for target in sequence_b)
        all_names.extend(target.name for target in sequence_c)
    all_plans = {**solved_b, **solved_c, **solved}
    all_targets = tuple(
        all_plans[name] for name in dict.fromkeys(all_names) if name in all_plans
    )
    failures: list[str] = []
    for name in dict.fromkeys(all_names):
        target = all_plans.get(name)
        if target is None:
            failures.append(f"IK {name}: no solution was returned")
        elif not target.converged:
            failures.append(f"IK {target.name}: {target.reason}: {target.message}")
    if failures:
        return HandoverPlan(
            layout=layout,
            targets=all_targets,
            segments=(),
            placement_contained=False,
            placement_message="placement was not checked because IK failed",
            planned_duration_s=0.0,
            planning_elapsed_s=time.perf_counter() - started,
            failures=tuple(failures),
        )

    left_fingers = side_fingers[_LEFT]
    right_fingers = side_fingers[_RIGHT]
    object_names = (layout.block_name,)
    left_grasp = solved["left_grasp"]
    right_handover_grasp = solved["right_handover_grasp"]
    body_id = int(model.body(layout.block_name).id)
    geom_id = int(model.geom(f"{layout.block_name}_geom").id)
    qpos_address = int(scene.object_free_joint_qpos[layout.block_name])
    left_site = next(chain.tcp_site for chain in chains if chain.side == _LEFT)
    right_site = next(chain.tcp_site for chain in chains if chain.side == _RIGHT)
    block_rotation = _as_tuple(_rotation_z(layout.block_initial_yaw_rad))
    left_envelope = BlockEnvelope(
        object_name=layout.block_name,
        body_id=body_id,
        geom_id=geom_id,
        qpos_address=qpos_address,
        holder_tcp_site=left_site,
        relative_pose=relative_block_pose(
            left_grasp.position, left_grasp.rotation, context["initial_block_center"],
            block_rotation,
        ),
    )
    right_handover_envelope = BlockEnvelope(
        object_name=layout.block_name,
        body_id=body_id,
        geom_id=geom_id,
        qpos_address=qpos_address,
        holder_tcp_site=right_site,
        relative_pose=relative_block_pose(
            right_handover_grasp.position,
            right_handover_grasp.rotation,
            context["handover_block_center"],
            block_rotation,
        ),
    )
    right_center_envelope: BlockEnvelope | None = None
    left_cohold_envelope: BlockEnvelope | None = None
    right_cohold_envelope: BlockEnvelope | None = None
    if not teacher_only:
        right_center_initial = solved_b["right_center_initial"]
        right_center_envelope = BlockEnvelope(
            object_name=layout.block_name,
            body_id=body_id,
            geom_id=geom_id,
            qpos_address=qpos_address,
            holder_tcp_site=right_site,
            relative_pose=relative_block_pose(
                right_center_initial.position, right_center_initial.rotation,
                context["initial_block_center"],
                block_rotation,
            ),
        )
        left_cohold_envelope = BlockEnvelope(
            object_name=layout.block_name,
            body_id=body_id,
            geom_id=geom_id,
            qpos_address=qpos_address,
            holder_tcp_site=left_site,
            relative_pose=relative_block_pose(
                solved_c["left_cohold_grasp"].position,
                solved_c["left_cohold_grasp"].rotation,
                context["initial_block_center"],
                block_rotation,
            ),
        )
        right_cohold_envelope = BlockEnvelope(
            object_name=layout.block_name,
            body_id=body_id,
            geom_id=geom_id,
            qpos_address=qpos_address,
            holder_tcp_site=right_site,
            relative_pose=relative_block_pose(
                solved_c["right_cohold_grasp"].position,
                solved_c["right_cohold_grasp"].rotation,
                context["initial_block_center"],
                block_rotation,
            ),
        )

    def pairs(excluded: tuple[str, ...]) -> object:
        return robot_pair_set(
            model,
            robot_prefixes=geometry.body_prefixes,
            robot_sides=dict(geometry.side_prefixes),
            object_names=object_names,
            excluded_object_bodies=excluded,
        )

    def objects(excluded: tuple[str, ...]) -> object:
        return object_pair_set(model, object_names, excluded_body_names=excluded)

    def check(
        name: str,
        start_name: str,
        end_name: str,
        start_qpos: tuple[float, ...],
        end_qpos: tuple[float, ...],
        *,
        active_side: str,
        excluded: tuple[str, ...],
        carried: CarriedBlock | None,
        allowed_start: tuple[str, ...] = (),
        allowed_end: tuple[str, ...] = (),
    ) -> PlanSegment:
        if active_side == "both":
            active_names = tuple(
                name
                for chain in chains
                if chain.side in (_LEFT, _RIGHT)
                for name in chain.joint_names
            )
            velocities = tuple(
                value
                for side in (_LEFT, _RIGHT)
                for value in _joint_velocities(max_velocities, side)
            )
        else:
            chain = next(item for item in chains if item.side == active_side)
            active_names = chain.joint_names
            velocities = _joint_velocities(max_velocities, active_side)
        result = check_joint_path(
            model,
            initial_qpos,
            start_qpos,
            end_qpos,
            active_joint_names=active_names,
            robot_pairs=pairs(excluded),
            object_pairs=objects(excluded) if carried is not None else None,
            clearance=clearance,
            segment=name,
            side=active_side,
            max_velocities=velocities,
            carried=carried,
            allowed_contacts_at_start=allowed_start,
            allowed_contacts_at_end=allowed_end,
            duration_peak_factor=config.execution.duration_peak_factor,
            minimum_duration_s=config.execution.minimum_segment_duration_s,
            deadline=deadline,
        )
        return PlanSegment(
            name=name,
            side=active_side,
            start_target=start_name,
            end_target=end_name,
            duration_s=result.duration_s,
            sample_count=result.sample_count,
            passed=result.passed,
            robot_minimum=result.robot_minimum,
            object_minimum=result.object_minimum,
            failure_reason=result.failure_reason,
            failure_message=result.failure_message,
        )

    left_carry = CarriedBlock(left_envelope, left_grasp.qpos, True)
    left_static = CarriedBlock(left_envelope, solved["left_handover"].qpos, False)
    right_handover_carry = CarriedBlock(right_handover_envelope, right_handover_grasp.qpos, True)
    left_chain = next(chain for chain in chains if chain.side == _LEFT)
    right_chain = next(chain for chain in chains if chain.side == _RIGHT)

    def _arm_addresses(chain: ArmChain) -> tuple[int, ...]:
        return tuple(
            int(model.jnt_qposadr[int(model.joint(name).id)])
            for name in chain.joint_names
        )

    left_addresses = _arm_addresses(left_chain)
    right_addresses = _arm_addresses(right_chain)

    def _mix_arms(
        base: tuple[float, ...],
        *,
        left_source: tuple[float, ...] | None = None,
        left_fraction: float = 1.0,
        right_source: tuple[float, ...] | None = None,
        right_fraction: float = 1.0,
    ) -> tuple[float, ...]:
        values = [float(value) for value in base]
        if left_source is not None:
            for address in left_addresses:
                values[address] = float(base[address]) + left_fraction * (
                    float(left_source[address]) - float(base[address])
                )
        if right_source is not None:
            for address in right_addresses:
                values[address] = float(base[address]) + right_fraction * (
                    float(right_source[address]) - float(base[address])
                )
        return tuple(values)

    q_ready_wait70 = _mix_arms(
        initial_qpos,
        left_source=solved["left_ready"].qpos,
        right_source=solved["right_handover_wait"].qpos,
        right_fraction=0.70,
    )
    q_pregrasp_wait100 = solved["left_pregrasp"].qpos
    q_grasp_wait100 = solved["left_grasp"].qpos
    q_handover_wait100 = solved["left_handover"].qpos
    q_right_pregrasp = solved["right_handover_pregrasp"].qpos
    q_right_grasp = solved["right_handover_grasp"].qpos
    q_retreat35 = _mix_arms(
        q_right_grasp,
        left_source=solved["left_retreat"].qpos,
        left_fraction=0.35,
    )
    q_retreat100_preplace = solved["right_preplace"].qpos
    q_retreat100_place = solved["right_place"].qpos
    teacher_waypoints = {
        "ready_wait70": q_ready_wait70,
        "pregrasp_wait100": q_pregrasp_wait100,
        "grasp_wait100": q_grasp_wait100,
        "handover_wait100": q_handover_wait100,
        "right_pregrasp": q_right_pregrasp,
        "right_grasp": q_right_grasp,
        "retreat35": q_retreat35,
        "retreat100_preplace": q_retreat100_preplace,
        "retreat100_place": q_retreat100_place,
    }

    segments: list[PlanSegment] = [
        check(
            "left_approach",
            "initial",
            "left_ready",
            initial_qpos,
            q_ready_wait70,
            active_side="both",
            excluded=(),
            carried=None,
        ),
        check(
            "left_pregrasp_path",
            "left_ready",
            "left_pregrasp",
            q_ready_wait70,
            q_pregrasp_wait100,
            active_side="both",
            excluded=(),
            carried=None,
        ),
        check(
            "left_grasp_path",
            "left_pregrasp",
            "left_grasp",
            q_pregrasp_wait100,
            q_grasp_wait100,
            active_side=_LEFT,
            excluded=(),
            carried=None,
        ),
        check(
            "left_handover",
            "left_grasp",
            "left_handover",
            q_grasp_wait100,
            q_handover_wait100,
            active_side=_LEFT,
            excluded=left_fingers,
            carried=left_carry,
            allowed_start=("table",),
        ),
        check(
            "right_handover_pregrasp_path",
            "left_handover",
            "right_handover_pregrasp",
            q_handover_wait100,
            q_right_pregrasp,
            active_side=_RIGHT,
            excluded=left_fingers,
            carried=left_static,
        ),
        check(
            "right_handover_grasp_path",
            "right_handover_pregrasp",
            "right_handover_grasp",
            q_right_pregrasp,
            q_right_grasp,
            active_side=_RIGHT,
            excluded=left_fingers,
            carried=left_static,
        ),
        check(
            "left_retreat",
            "right_handover_grasp",
            "left_retreat",
            q_right_grasp,
            q_retreat35,
            active_side=_LEFT,
            excluded=right_fingers,
            carried=CarriedBlock(right_handover_envelope, q_right_grasp, False),
        ),
        check(
            "right_transport",
            "left_retreat",
            "right_preplace",
            q_retreat35,
            q_retreat100_preplace,
            active_side="both",
            excluded=right_fingers,
            carried=right_handover_carry,
        ),
        check(
            "right_place",
            "right_preplace",
            "right_place",
            q_retreat100_preplace,
            q_retreat100_place,
            active_side=_RIGHT,
            excluded=right_fingers,
            carried=right_handover_carry,
            allowed_end=(layout.box_name,),
        ),
    ]
    if not teacher_only:
        right_initial_carry = CarriedBlock(
            right_center_envelope, right_center_initial.qpos, True
        )
        segments.extend(
            [
                check(
                    "right_initial_wait",
                    "initial",
                    "right_wait_initial",
                    initial_qpos,
                    solved_b["right_wait_initial"].qpos,
                    active_side=_RIGHT,
                    excluded=(),
                    carried=None,
                ),
                check(
                    "right_initial_pregrasp_path",
                    "right_wait_initial",
                    "right_initial_pregrasp",
                    solved_b["right_wait_initial"].qpos,
                    solved_b["right_initial_pregrasp"].qpos,
                    active_side=_RIGHT,
                    excluded=(),
                    carried=None,
                ),
                check(
                    "right_initial_grasp_path",
                    "right_initial_pregrasp",
                    "right_center_initial",
                    solved_b["right_initial_pregrasp"].qpos,
                    solved_b["right_center_initial"].qpos,
                    active_side=_RIGHT,
                    excluded=(),
                    carried=None,
                ),
                check(
                    "right_initial_lift",
                    "right_center_initial",
                    "right_center_lift",
                    solved_b["right_center_initial"].qpos,
                    solved_b["right_center_lift"].qpos,
                    active_side=_RIGHT,
                    excluded=right_fingers,
                    carried=right_initial_carry,
                    allowed_start=("table",),
                ),
                check(
                    "left_cohold_approach",
                    "initial",
                    "left_ready",
                    initial_qpos,
                    solved_c["left_ready"].qpos,
                    active_side=_LEFT,
                    excluded=(),
                    carried=None,
                ),
                check(
                    "left_cohold_pregrasp_path",
                    "left_ready",
                    "left_cohold_pregrasp",
                    solved_c["left_ready"].qpos,
                    solved_c["left_cohold_pregrasp"].qpos,
                    active_side=_LEFT,
                    excluded=(),
                    carried=None,
                ),
                check(
                    "left_cohold_grasp_path",
                    "left_cohold_pregrasp",
                    "left_cohold_grasp",
                    solved_c["left_cohold_pregrasp"].qpos,
                    solved_c["left_cohold_grasp"].qpos,
                    active_side=_LEFT,
                    excluded=(),
                    carried=None,
                ),
                check(
                    "right_cohold_wait_path",
                    "left_cohold_grasp",
                    "right_cohold_wait",
                    solved_c["left_cohold_grasp"].qpos,
                    solved_c["right_cohold_wait"].qpos,
                    active_side=_RIGHT,
                    excluded=left_fingers,
                    carried=CarriedBlock(
                        left_cohold_envelope, solved_c["left_cohold_grasp"].qpos, False
                    ),
                ),
                check(
                    "right_cohold_pregrasp_path",
                    "right_cohold_wait",
                    "right_cohold_pregrasp",
                    solved_c["right_cohold_wait"].qpos,
                    solved_c["right_cohold_pregrasp"].qpos,
                    active_side=_RIGHT,
                    excluded=left_fingers,
                    carried=CarriedBlock(
                        left_cohold_envelope, solved_c["left_cohold_grasp"].qpos, False
                    ),
                ),
                check(
                    "right_cohold_grasp_path",
                    "right_cohold_pregrasp",
                    "right_cohold_grasp",
                    solved_c["right_cohold_pregrasp"].qpos,
                    solved_c["right_cohold_grasp"].qpos,
                    active_side=_RIGHT,
                    excluded=left_fingers,
                    carried=CarriedBlock(
                        left_cohold_envelope, solved_c["left_cohold_grasp"].qpos, False
                    ),
                ),
                check(
                    "left_cohold_retreat",
                    "right_cohold_grasp",
                    "left_cohold_retreat",
                    solved_c["right_cohold_grasp"].qpos,
                    solved_c["left_cohold_retreat"].qpos,
                    active_side=_LEFT,
                    excluded=right_fingers,
                    carried=CarriedBlock(
                        BlockEnvelope(
                            object_name=layout.block_name,
                            body_id=body_id,
                            geom_id=geom_id,
                            qpos_address=qpos_address,
                            holder_tcp_site=right_site,
                            relative_pose=relative_block_pose(
                                solved_c["right_cohold_grasp"].position,
                                solved_c["right_cohold_grasp"].rotation,
                                context["initial_block_center"],
                                block_rotation,
                            ),
                        ),
                        solved_c["right_cohold_grasp"].qpos,
                        False,
                    ),
                ),
                check(
                    "dual_hold_lift",
                    "right_cohold_grasp",
                    "right_cohold_lift",
                    solved_c["right_cohold_grasp"].qpos,
                    solved_c["right_cohold_lift"].qpos,
                    active_side="both",
                    excluded=left_fingers + right_fingers,
                    carried=CarriedBlock(
                        left_cohold_envelope, solved_c["right_cohold_grasp"].qpos, True
                    ),
                ),
                check(
                    "left_air_retreat",
                    "right_cohold_lift",
                    "left_air_retreat",
                    solved_c["right_cohold_lift"].qpos,
                    solved_c["left_air_retreat"].qpos,
                    active_side=_LEFT,
                    excluded=right_fingers,
                    carried=CarriedBlock(
                        right_cohold_envelope, solved_c["right_cohold_lift"].qpos, False
                    ),
                ),
            ]
        )

    place_tcp = tcp_matrix(model, solved["right_place"].qpos, right_site)
    place_block_matrix = np.asarray(place_tcp, dtype=np.float64) @ np.asarray(
        right_handover_envelope.relative_pose, dtype=np.float64
    )
    contained, containment_message = _placement_containment(
        layout,
        place_block_matrix[:3, 3],
        place_block_matrix[:3, :3],
        layout.block_half_extents,
    )
    if teacher_only:
        for segment in segments:
            if not segment.passed:
                failures.append(
                    f"path {segment.name}: {segment.failure_reason}: {segment.failure_message}"
                )
        if not contained:
            failures.append(f"placement containment: {containment_message}")
        return HandoverPlan(
            layout=layout,
            targets=all_targets,
            segments=tuple(segments),
            placement_contained=contained,
            placement_message=containment_message,
            planned_duration_s=sum(segment.duration_s for segment in segments),
            planning_elapsed_s=time.perf_counter() - started,
            failures=tuple(failures),
            teacher_waypoints=teacher_waypoints,
        )

    # Dual-hold coordination: both TCPs are compared with the same nominal
    # block pose along the coordinated lift.  The single joint-linear segment
    # is reported for contrast; the waypoint path is the path the execution
    # follows.
    cohold_grasp = solved_c["right_cohold_grasp"]
    cohold_lift = solved_c["right_cohold_lift"]
    left_relative = np.asarray(
        relative_block_pose(
            solved_c["left_cohold_grasp"].position,
            solved_c["left_cohold_grasp"].rotation,
            context["initial_block_center"],
            block_rotation,
        ),
        dtype=np.float64,
    )
    right_relative = np.asarray(
        relative_block_pose(
            cohold_grasp.position,
            cohold_grasp.rotation,
            context["initial_block_center"],
            block_rotation,
        ),
        dtype=np.float64,
    )
    single_samples = sample_joint_path(
        cohold_grasp.qpos,
        cohold_lift.qpos,
        max_step_rad=clearance.sample_step_rad,
        max_samples=clearance.max_samples_per_segment,
    )
    single_curve: list[CoordinationPoint] = []
    denominator = max(1, len(single_samples) - 1)
    for index, qpos in enumerate(single_samples):
        fraction = index / denominator
        center = np.asarray(context["initial_block_center"], dtype=np.float64) + np.array(
            [0.0, 0.0, config.grasp.cohold_lift_height_m * fraction]
        )
        errors = _coordination_errors(
            model,
            tuple(float(value) for value in qpos),
            left_site=left_site,
            right_site=right_site,
            block_center=tuple(float(value) for value in center),
            block_yaw_rad=layout.block_initial_yaw_rad,
            left_relative=left_relative,
            right_relative=right_relative,
        )
        single_curve.append(
            CoordinationPoint(fraction, errors[0], errors[1], errors[2], errors[3])
        )
    waypoints, waypoint_curve = _waypoint_lift(
        model,
        cohold_grasp.qpos,
        chains=chains,
        ik=ik,
        clearance=clearance,
        block_center=np.asarray(context["initial_block_center"], dtype=np.float64),
        block_yaw_rad=layout.block_initial_yaw_rad,
        left_grasp=solved_c["left_cohold_grasp"],
        right_grasp=cohold_grasp,
        lift_height=config.grasp.cohold_lift_height_m,
        left_site=left_site,
        right_site=right_site,
        left_relative=left_relative,
        right_relative=right_relative,
        steps=10,
        sample_step_rad=clearance.sample_step_rad,
        max_samples=clearance.max_samples_per_segment,
        deadline=deadline,
    )

    def _curve_max(curve: tuple[CoordinationPoint, ...]) -> dict[str, float]:
        if not curve:
            return {
                "left_position_error_m": math.inf,
                "left_orientation_error_rad": math.inf,
                "right_position_error_m": math.inf,
                "right_orientation_error_rad": math.inf,
            }
        return {
            "left_position_error_m": max(item.left_position_error_m for item in curve),
            "left_orientation_error_rad": max(
                item.left_orientation_error_rad for item in curve
            ),
            "right_position_error_m": max(item.right_position_error_m for item in curve),
            "right_orientation_error_rad": max(
                item.right_orientation_error_rad for item in curve
            ),
        }

    coordination_summary = {
        "single_segment": _curve_max(tuple(single_curve)),
        "waypoint_path": _curve_max(waypoint_curve),
        "waypoint_count": len(waypoints),
        "waypoint_samples": len(waypoint_curve),
    }

    # The execution follows the waypoint pieces, so clearance is measured on
    # exactly those pieces, with the carried block following the left hand.
    waypoint_active = tuple(
        name
        for chain in chains
        if chain.side in (_LEFT, _RIGHT)
        for name in chain.joint_names
    )
    waypoint_velocities = tuple(
        value
        for side in (_LEFT, _RIGHT)
        for value in _joint_velocities(max_velocities, side)
    )
    waypoint_pieces = []
    for index in range(len(waypoints) - 1):
        waypoint_pieces.append(
            check_joint_path(
                model,
                initial_qpos,
                waypoints[index],
                waypoints[index + 1],
                active_joint_names=waypoint_active,
                robot_pairs=pairs(left_fingers + right_fingers),
                object_pairs=objects(left_fingers + right_fingers),
                clearance=clearance,
                segment=f"dual_hold_lift_waypoint_{index + 1}",
                side="both",
                max_velocities=waypoint_velocities,
                carried=CarriedBlock(
                    left_cohold_envelope, cohold_grasp.qpos, True
                ),
                allowed_contacts_at_start=("table",) if index == 0 else (),
                duration_peak_factor=config.execution.duration_peak_factor,
                minimum_duration_s=config.execution.minimum_segment_duration_s,
                deadline=deadline,
            )
        )
    if waypoint_pieces:
        worst_robot = min(waypoint_pieces, key=lambda item: item.robot_minimum.distance_m)
        worst_object = min(
            (
                item
                for item in waypoint_pieces
                if item.object_minimum is not None
            ),
            key=lambda item: item.object_minimum.distance_m,
            default=None,
        )
        waypoint_segment = PlanSegment(
            name="dual_hold_lift_waypoints",
            side="both",
            start_target="right_cohold_grasp",
            end_target="right_cohold_lift",
            duration_s=sum(item.duration_s for item in waypoint_pieces),
            sample_count=sum(item.sample_count for item in waypoint_pieces),
            passed=all(item.passed for item in waypoint_pieces),
            robot_minimum=worst_robot.robot_minimum,
            object_minimum=None if worst_object is None else worst_object.object_minimum,
            failure_reason=(
                None
                if all(item.passed for item in waypoint_pieces)
                else worst_robot.failure_reason
            ),
            failure_message=(
                None
                if all(item.passed for item in waypoint_pieces)
                else worst_robot.failure_message
            ),
        )
        segments.append(waypoint_segment)
    else:
        waypoint_segment = None
    if waypoint_segment is not None and not waypoint_segment.passed:
        failures.append(
            f"path {waypoint_segment.name}: {waypoint_segment.failure_reason}: "
            f"{waypoint_segment.failure_message}"
        )
    waypoint_max = coordination_summary["waypoint_path"]
    if waypoint_max["left_position_error_m"] > config.diagnostics.relative_position_tolerance_m:
        failures.append(
            "dual-hold left position coordination "
            f"{waypoint_max['left_position_error_m']:.6f} m exceeds "
            f"{config.diagnostics.relative_position_tolerance_m:.6f} m"
        )
    if waypoint_max["right_position_error_m"] > config.diagnostics.relative_position_tolerance_m:
        failures.append(
            "dual-hold right position coordination "
            f"{waypoint_max['right_position_error_m']:.6f} m exceeds "
            f"{config.diagnostics.relative_position_tolerance_m:.6f} m"
        )
    if (
        waypoint_max["left_orientation_error_rad"]
        > config.diagnostics.relative_orientation_tolerance_rad
    ):
        failures.append("dual-hold left orientation coordination exceeds the tolerance")
    if (
        waypoint_max["right_orientation_error_rad"]
        > config.diagnostics.relative_orientation_tolerance_rad
    ):
        failures.append("dual-hold right orientation coordination exceeds the tolerance")
    for segment in segments:
        if not segment.passed:
            failures.append(
                f"path {segment.name}: {segment.failure_reason}: {segment.failure_message}"
            )
    if not contained:
        failures.append(f"placement containment: {containment_message}")
    return HandoverPlan(
        layout=layout,
        targets=all_targets,
        segments=tuple(segments),
        placement_contained=contained,
        placement_message=containment_message,
        planned_duration_s=sum(segment.duration_s for segment in segments),
        planning_elapsed_s=time.perf_counter() - started,
        coordination=waypoint_curve,
        coordinated_waypoints=waypoints,
        coordination_summary=coordination_summary,
        failures=tuple(failures),
        teacher_waypoints=teacher_waypoints,
    )
