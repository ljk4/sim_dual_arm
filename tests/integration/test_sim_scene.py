from __future__ import annotations

import sys
from pathlib import Path

import pytest

mujoco = pytest.importorskip("mujoco")
np = pytest.importorskip("numpy")

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.marvin_sim.assembly import gripper_opening  # noqa: E402
from robot_backend_sim.marvin_sim.scene_base import (  # noqa: E402
    base_world_point,
    combined_yaw_quaternion,
)
from robot_backend_sim.marvin_sim.stack.kinematics_config import (  # noqa: E402
    MARVIN_ARM_JOINT_RANGES,
)
from robot_backend_sim.marvin_sim.stack.scene import (  # noqa: E402
    build_marvin_stack_scene,
)
from robot_backend_sim.marvin_sim.stack.scene_config import (  # noqa: E402
    load_marvin_stack_scene_config,
)

CONFIG_PATH = ROOT / "shared/scenarios/marvin_sim/stack/configs/simulation/scene/marvin_stack.yaml"


@pytest.fixture(scope="module")
def scene():
    config = load_marvin_stack_scene_config(CONFIG_PATH)
    return build_marvin_stack_scene(
        config,
    )


def object_qpos(scene, name: str):
    address = scene.indices.environment.object_free_joint_qpos[name]
    return scene.data.qpos[address : address + 7]


def test_model_has_dual_marvin_names_dof_and_cameras(scene):
    model = scene.model
    assert (model.nq, model.nv, model.nu, model.ncam, model.nkey) == (32, 30, 16, 3, 0)
    assert model.ntendon == 2
    assert model.neq == 2

    for side, prefix in (("left", "L"), ("right", "R")):
        for joint in range(1, 8):
            name = f"Arm_{prefix}{joint}_Joint"
            assert model.joint(name).name == name
            actuator = f"Arm_{prefix}{joint}_Actuator"
            assert model.actuator(actuator).name == actuator
        for finger in (10, 11):
            name = f"Arm_{prefix}{finger}_Joint"
            assert model.joint(name).name == name
            link = f"Arm_{prefix}{finger}_Link"
            assert model.body(link).name == link
        gripper_actuator = f"Arm_{prefix}_gripper_actuator"
        assert model.actuator(gripper_actuator).name == gripper_actuator
        assert model.body(f"Arm_{prefix}9_Link").name == f"Arm_{prefix}9_Link"
        assert model.site(f"Arm_{prefix}_tcp").name == f"Arm_{prefix}_tcp"
        assert len(scene.indices.robot.arm_joint_qpos[side]) == 7
        assert len(scene.indices.robot.finger_joint_qpos[side]) == 2

    assert np.all(model.cam_targetbodyid == -1)
    assert np.all(model.cam_mode == mujoco.mjtCamLight.mjCAMLIGHT_FIXED)
    assert model.cam_bodyid[scene.indices.camera_ids["global"]] == model.body("camera_mount").id
    assert model.cam_bodyid[scene.indices.camera_ids["left_wrist"]] == model.body("Arm_L9_Link").id
    assert model.cam_bodyid[scene.indices.camera_ids["right_wrist"]] == model.body("Arm_R9_Link").id
    assert np.all(model.eq_type != mujoco.mjtEq.mjEQ_WELD)
    np.testing.assert_allclose(
        model.body("ZJ_Robot_link").pos, list(scene.config.robot.base_position)
    )
    assert [camera.name for camera in scene.config.cameras] == [
        "global",
        "left_wrist",
        "right_wrist",
    ]
    assert all(camera.width == 320 and camera.height == 240 for camera in scene.config.cameras)


def test_joint_ranges_and_initial_values_match_fixed_upstream(scene):
    scene.reset(0)
    model = scene.model
    for side, prefix in (("left", "L"), ("right", "R")):
        home = scene.config.robot.home_joint_positions.for_side(side)
        for index, (lower, upper) in enumerate(MARVIN_ARM_JOINT_RANGES, start=1):
            joint = model.joint(f"Arm_{prefix}{index}_Joint")
            np.testing.assert_allclose(model.jnt_range[joint.id], [lower, upper], atol=1e-4)
            assert model.jnt_limited[joint.id] == 1
            position = home[index - 1]
            assert lower <= position <= upper
            assert scene.data.qpos[scene.indices.robot.arm_joint_qpos[side][index - 1]] == (
                pytest.approx(position)
            )

    for left_name, right_name in (
        ("Arm_L3_Link", "Arm_R3_Link"),
        ("Arm_L5_Link", "Arm_R5_Link"),
    ):
        left_pos = scene.data.body(left_name).xpos
        right_pos = scene.data.body(right_name).xpos
        np.testing.assert_allclose(
            right_pos, [left_pos[0], -left_pos[1], left_pos[2]], atol=2e-3
        )
    left_tcp = scene.data.site(scene.indices.robot.tcp_site_ids["left"]).xpos
    right_tcp = scene.data.site(scene.indices.robot.tcp_site_ids["right"]).xpos
    np.testing.assert_allclose(
        right_tcp, [left_tcp[0], -left_tcp[1], left_tcp[2]], atol=2e-3
    )


def test_objects_are_free_colliding_bodies_under_gravity(scene):
    model = scene.model
    assert model.opt.gravity[2] == pytest.approx(-9.81)

    for obj in scene.config.objects:
        joint = model.joint(f"{obj.name}_free")
        body_id = model.body(obj.name).id
        geom_id = model.geom(f"{obj.name}_geom").id
        assert model.jnt_type[joint.id] == mujoco.mjtJoint.mjJNT_FREE
        assert model.body_mocapid[body_id] == -1
        assert model.body_mass[body_id] == pytest.approx(obj.mass)
        assert model.geom_contype[geom_id] != 0
        assert model.geom_conaffinity[geom_id] != 0
        assert model.jnt_bodyid[joint.id] == body_id


def test_same_seed_reset_restores_complete_state(scene):
    scene.reset(11)
    expected_qpos = scene.data.qpos.copy()
    expected_ctrl = scene.data.ctrl.copy()
    expected_sites = {
        side: scene.data.site(scene.indices.robot.tcp_site_ids[side]).xpos.copy()
        for side in ("left", "right")
    }
    expected_bodies = {
        obj.name: scene.data.body(obj.name).xpos.copy() for obj in scene.config.objects
    }

    for _ in range(25):
        mujoco.mj_step(scene.model, scene.data)
    scene.data.qvel[:] = 3.5
    scene.data.ctrl[:] = -7.0
    scene.data.act[:] = 2.0
    scene.data.qfrc_applied[:] = 4.0
    scene.data.xfrc_applied[:] = 4.0
    scene.data.qacc_warmstart[:] = 1.0
    scene.data.time = 99.0

    scene.reset(11)
    np.testing.assert_array_equal(scene.data.qpos, expected_qpos)
    np.testing.assert_array_equal(scene.data.ctrl, expected_ctrl)
    assert scene.data.time == 0.0
    assert not np.any(scene.data.qvel)
    assert not np.any(scene.data.act)
    assert not np.any(scene.data.qfrc_applied)
    assert not np.any(scene.data.xfrc_applied)
    assert not np.any(scene.data.qacc_warmstart)
    for side, expected in expected_sites.items():
        np.testing.assert_array_equal(
            scene.data.site(scene.indices.robot.tcp_site_ids[side]).xpos, expected
        )
    for name, expected in expected_bodies.items():
        np.testing.assert_array_equal(scene.data.body(name).xpos, expected)


def test_seed_does_not_change_nominal_object_poses(scene):
    robot = scene.config.robot
    expected = {}
    for obj in scene.config.objects:
        center = base_world_point(
            (*obj.nominal_xy, scene.config.table.top_z + obj.half_extents[2]),
            robot.base_position,
            robot.base_yaw_deg,
        )
        orientation = combined_yaw_quaternion(robot.base_yaw_deg, obj.nominal_yaw_deg)
        expected[obj.name] = (*center, *orientation)

    for seed in (0, 11, 1000, 11):
        scene.reset(seed)
        for name, pose in expected.items():
            np.testing.assert_allclose(object_qpos(scene, name), pose, rtol=0.0, atol=1e-12)


def test_default_objects_rest_on_table_for_short_run(scene):
    scene.reset(3)
    initial = {obj.name: object_qpos(scene, obj.name).copy() for obj in scene.config.objects}

    for _ in range(500):
        mujoco.mj_step(scene.model, scene.data)

    assert np.all(np.isfinite(scene.data.qpos))
    assert np.all(np.isfinite(scene.data.qvel))
    for obj in scene.config.objects:
        qpos = object_qpos(scene, obj.name)
        start = initial[obj.name]
        expected_z = (
            scene.config.robot.base_position[2]
            + scene.config.table.top_z
            + obj.half_extents[2]
        )
        assert abs(qpos[2] - expected_z) <= 0.01
        assert abs(qpos[0] - start[0]) <= 0.03
        assert abs(qpos[1] - start[1]) <= 0.03
        dof = int(scene.model.joint(f"{obj.name}_free").dofadr[0])
        speed = scene.data.qvel[dof : dof + 6]
        assert np.linalg.norm(speed[:3]) <= 0.2


def test_gripper_initialization_uses_tendon_closure_command(scene):
    scene.reset(0)
    assert scene.gripper_open_total == pytest.approx(0.0814)
    assert scene.gripper_closed_total == pytest.approx(0.0014)
    assert scene.gripper_ctrl(scene.gripper_open_total) == pytest.approx(0.0)
    assert scene.gripper_ctrl(scene.gripper_closed_total) == pytest.approx(0.08)
    assert scene.gripper_ctrl(0.0414) == pytest.approx(0.04)
    for side in ("left", "right"):
        gripper = scene.indices.robot.gripper_actuator_ids[side]
        assert scene.data.ctrl[gripper] == pytest.approx(0.0)
        for address in scene.indices.robot.finger_joint_qpos[side]:
            assert scene.data.qpos[address] == pytest.approx(0.0)
        assert gripper_opening(scene.model, scene.data, scene.indices.robot, side) == pytest.approx(
            scene.gripper_open_total, abs=1e-4
        )
