"""Airborne transfer and placement preserve one right-hand grasp transform."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

pytest.importorskip("mujoco")
np = pytest.importorskip("numpy")

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "simulation/mujoco/src"))

from robot_backend_sim.config.marvin_scene_config import load_marvin_scene_config  # noqa: E402
from robot_backend_sim.kinematics.dual_arm import relative_block_pose  # noqa: E402
from robot_backend_sim.physics.marvin.scene import build_marvin_scene  # noqa: E402
from robot_backend_sim.physics.robot_geometry import ArmChain  # noqa: E402
from robot_backend_sim.tasks.handover.config import load_handover_plan_config  # noqa: E402
from robot_backend_sim.tasks.handover.plan import (  # noqa: E402
    HandoverLayout,
    build_handover_targets,
    layout_from_scene,
)


def _pose(position, rotation):
    value = np.eye(4)
    value[:3, :3] = rotation
    value[:3, 3] = position
    return value


def _yaw(angle):
    cosine, sine = math.cos(angle), math.sin(angle)
    return np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])


def test_layout_block_yaw_includes_the_robot_base_yaw():
    config = load_marvin_scene_config(
        ROOT / "shared/scenarios/marvin_handover/configs/simulation/scene/marvin_handover.yaml"
    )
    config = config.model_copy(update={
        "robot": config.robot.model_copy(update={"base_yaw_deg": 45.0}),
        "task": config.task.model_copy(update={
            "block": config.task.block.model_copy(update={"initial_yaw_deg": -15.0})
        }),
    })
    scene = build_marvin_scene(config)
    layout = layout_from_scene(scene)
    address = scene.object_free_joint_qpos[config.task.block.name]
    quaternion = scene.data.qpos[address + 3:address + 7]
    compiled_yaw = 2.0 * math.atan2(float(quaternion[3]), float(quaternion[0]))
    assert layout.block_initial_yaw_rad == pytest.approx(math.radians(30.0))
    assert layout.block_initial_yaw_rad == pytest.approx(compiled_yaw)


def test_relative_block_pose_preserves_full_world_rotation():
    base_yaw, block_yaw, roll = math.radians(45.0), math.radians(-15.0), 0.2
    cosine, sine = math.cos(roll), math.sin(roll)
    block_rotation = _yaw(base_yaw + block_yaw) @ np.array(
        [[1.0, 0.0, 0.0], [0.0, cosine, -sine], [0.0, sine, cosine]]
    )
    tcp_position, block_center = (0.5, 0.2, 0.95), (0.55, 0.12, 0.91)
    tcp_rotation = _yaw(-0.4)
    relative = relative_block_pose(
        tcp_position, tuple(map(tuple, tcp_rotation)), block_center,
        tuple(map(tuple, block_rotation)),
    )
    np.testing.assert_allclose(
        _pose(tcp_position, tcp_rotation) @ relative,
        _pose(block_center, block_rotation), atol=1e-12,
    )
    default_relative = relative_block_pose(
        tcp_position, tuple(map(tuple, tcp_rotation)), block_center,
    )
    np.testing.assert_allclose(
        _pose(tcp_position, tcp_rotation) @ default_relative,
        _pose(block_center, np.eye(3)), atol=1e-12,
    )


def test_handover_and_place_keep_the_edge_grasp_and_distinct_lift_height():
    config = load_handover_plan_config(
        ROOT / "shared/scenarios/marvin_handover/configs/simulation/planning/handover_paths.yaml"
    )
    config = config.model_copy(update={"grasp": config.grasp.model_copy(update={
        "right_cohold": config.grasp.right_cohold.model_copy(update={"approach_yaw_deg": 12.0})
    })})
    layout = HandoverLayout(
        base_position=(0.0, 0.0, 0.0), base_yaw_deg=0.0, table_top_z=0.78,
        block_name="block", block_half_extents=(0.03, 0.10, 0.03),
        block_initial_center=(0.55, 0.12, 0.81), block_initial_yaw_rad=0.25,
        box_name="box", box_center=(0.64, -0.27, 0.78), box_inner_xy=(0.296, 0.296),
        box_inner_bottom_z=0.792, box_rim_z=0.88, box_yaw_rad=0.0,
    )
    targets, context = build_handover_targets(
        layout, config, (ArmChain("left", (), "left_tcp"), ArmChain("right", (), "right_tcp"))
    )
    targets = {target.name: target for target in targets}
    assert targets["left_lift"].position[2] - targets["left_grasp"].position[2] == (
        pytest.approx(config.grasp.lift_height_m)
    )
    assert targets["left_handover"].position[2] - targets["left_grasp"].position[2] == (
        pytest.approx(config.handover.block_center_offset_m[2])
    )
    grasp = targets["right_handover_grasp"]
    expected_offset = _yaw(layout.block_initial_yaw_rad) @ np.array(
        [0.0, config.grasp.right_cohold.along_block_y_m, config.grasp.tcp_above_block_center_m]
    )
    np.testing.assert_allclose(
        np.asarray(grasp.position) - context["handover_block_center"], expected_offset
    )
    block_in_tcp = np.linalg.inv(_pose(grasp.position, grasp.rotation)) @ _pose(
        context["handover_block_center"], _yaw(layout.block_initial_yaw_rad)
    )
    place_yaw = layout.block_initial_yaw_rad + math.radians(config.transport.place_yaw_deg)
    for name, center in (
        ("right_preplace", "preplace_block_center"), ("right_place", "place_block_center")
    ):
        target = targets[name]
        np.testing.assert_allclose(
            _pose(target.position, target.rotation) @ block_in_tcp,
            _pose(context[center], _yaw(place_yaw)), atol=1e-12,
        )
