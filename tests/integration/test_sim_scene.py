from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest
from model_helpers import replace

mujoco = pytest.importorskip("mujoco")
np = pytest.importorskip("numpy")

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.panda_sim.robot_config import PANDA_JOINT_RANGES  # noqa: E402
from robot_backend_sim.panda_sim.stack.scene_config import load_scene_config  # noqa: E402
from robot_backend_sim.panda_sim.stack.scene import Scene, build_scene  # noqa: E402

CONFIG_PATH = ROOT / "shared/scenarios/panda_sim/stack/configs/simulation/scene/panda_stack.yaml"


@pytest.fixture(scope="module")
def scene():
    config = load_scene_config(CONFIG_PATH)
    return build_scene(
        config,
    )


def object_qpos(scene, name: str):
    address = scene.indices.environment.object_free_joint_qpos[name]
    return scene.data.qpos[address : address + 7]


def test_model_has_dual_panda_names_dof_and_cameras(scene):
    model = scene.model
    assert (model.nq, model.nv, model.nu, model.ncam, model.nkey) == (32, 30, 16, 3, 0)
    assert model.ntendon == 2
    assert model.neq == 2

    for side in ("left", "right"):
        for joint in range(1, 8):
            assert model.joint(f"{side}_joint{joint}").name == f"{side}_joint{joint}"
        for finger in (1, 2):
            assert (
                model.joint(f"{side}_finger_joint{finger}").name == f"{side}_finger_joint{finger}"
            )
        for actuator in range(1, 9):
            assert model.actuator(f"{side}_actuator{actuator}").name == f"{side}_actuator{actuator}"
        assert model.body(f"{side}_hand").name == f"{side}_hand"
        assert model.site(f"{side}_tcp").name == f"{side}_tcp"
        assert len(scene.indices.robot.arm_joint_qpos[side]) == 7
        assert len(scene.indices.robot.finger_joint_qpos[side]) == 2

    assert np.all(model.cam_targetbodyid == -1)
    assert np.all(model.cam_mode == mujoco.mjtCamLight.mjCAMLIGHT_FIXED)
    assert model.cam_bodyid[scene.indices.camera_ids["global"]] == model.body("column").id
    assert model.cam_bodyid[scene.indices.camera_ids["left_wrist"]] == model.body("left_hand").id
    assert model.cam_bodyid[scene.indices.camera_ids["right_wrist"]] == model.body("right_hand").id
    assert np.all(model.eq_type != mujoco.mjtEq.mjEQ_WELD)
    for side, sign in (("left", -1.0), ("right", 1.0)):
        np.testing.assert_allclose(model.body(f"{side}_link0").pos, [0.25 * sign, 0.0, 1.0])
    assert [camera.name for camera in scene.config.cameras] == [
        "global",
        "left_wrist",
        "right_wrist",
    ]
    assert all(camera.width == 320 and camera.height == 240 for camera in scene.config.cameras)


def test_joint_ranges_and_initial_values_match_fixed_upstream(scene):
    scene.reset(0)
    model = scene.model
    home = scene.config.reset.home_joint_positions
    for side in ("left", "right"):
        for index, (lower, upper) in enumerate(PANDA_JOINT_RANGES, start=1):
            joint = model.joint(f"{side}_joint{index}")
            np.testing.assert_allclose(model.jnt_range[joint.id], [lower, upper])
            assert model.jnt_limited[joint.id] == 1
            position = home[index - 1]
            assert lower <= position <= upper
            assert scene.data.qpos[scene.indices.robot.arm_joint_qpos[side][index - 1]] == (
                pytest.approx(position)
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
        side: scene.data.site(scene.indices.robot.tcp_site_ids[f"{side}_tcp"]).xpos.copy()
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
            scene.data.site(scene.indices.robot.tcp_site_ids[f"{side}_tcp"]).xpos, expected
        )
    for name, expected in expected_bodies.items():
        np.testing.assert_array_equal(scene.data.body(name).xpos, expected)


def test_different_seeds_vary_objects_within_configured_domain(scene):
    samples = []
    for seed in (1, 2, 3, 4, 5):
        scene.reset(seed)
        seed_sample = []
        for obj in scene.config.objects:
            qpos = object_qpos(scene, obj.name)
            x, y, z = qpos[:3]
            quaternion = qpos[3:7]
            yaw = 2.0 * math.atan2(quaternion[3], quaternion[0])
            assert abs(x - obj.nominal_xy[0]) <= obj.randomization.xy_m + 1e-9
            assert abs(y - obj.nominal_xy[1]) <= obj.randomization.xy_m + 1e-9
            assert abs(yaw) <= math.radians(obj.randomization.yaw_deg) + 1e-9
            assert z == pytest.approx(scene.config.table.top_z + obj.half_extents[2])
            assert math.sqrt(float(np.dot(quaternion, quaternion))) == pytest.approx(1.0)
            seed_sample.append((x, y, yaw))
        samples.append(seed_sample)

    assert not np.allclose(samples[0], samples[1])
    assert not np.allclose(samples[0], samples[2])


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
        expected_z = scene.config.table.top_z + obj.half_extents[2]
        assert abs(qpos[2] - expected_z) <= 0.01
        assert abs(qpos[0] - start[0]) <= 0.03
        assert abs(qpos[1] - start[1]) <= 0.03
        dof = int(scene.model.joint(f"{obj.name}_free").dofadr[0])
        speed = scene.data.qvel[dof : dof + 6]
        assert np.linalg.norm(speed[:3]) <= 0.2


def test_gripper_initialization_uses_continuous_vendor_command(scene):
    half_open = replace(scene.config, reset=replace(scene.config.reset, gripper_total_opening=0.04))
    half_data = mujoco.MjData(scene.model)
    half_scene = Scene(scene.model, half_data, half_open, scene.indices)
    half_scene.reset(0)

    scene.reset(0)
    for side in ("left", "right"):
        gripper = scene.indices.robot.gripper_actuator_ids[side]
        assert scene.data.ctrl[gripper] == pytest.approx(255.0)
        assert half_data.ctrl[gripper] == pytest.approx(127.5)
        for address in scene.indices.robot.finger_joint_qpos[side]:
            assert half_data.qpos[address] == pytest.approx(0.02)
