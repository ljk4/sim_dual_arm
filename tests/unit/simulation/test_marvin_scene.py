"""Marvin scene/robot/task separation and compiled-model contract."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

mujoco = pytest.importorskip("mujoco")
np = pytest.importorskip("numpy")

ROOT = Path(__file__).resolve().parents[3]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.common.config.camera_config import camera_declared_description  # noqa: E402
from robot_backend_sim.marvin_sim.handover.scene_config import load_marvin_scene_config  # noqa: E402
from robot_backend_sim.marvin_sim.handover.scene import build_marvin_scene  # noqa: E402
from robot_backend_sim.marvin_sim.assembly import gripper_opening  # noqa: E402
from robot_contracts.models import ContractError  # noqa: E402
from robot_contracts.profiles import (  # noqa: E402
    load_document,
    load_profile,
    semantic_hash,
    validate_bindings,
)

SCENARIO = ROOT / "shared/scenarios/marvin_sim/handover"
CONFIG_PATH = SCENARIO / "configs/simulation/scene/marvin_handover.yaml"
PROFILE_PATH = SCENARIO / "profiles/robot/profile.json"
SCHEMAS = ROOT / "shared/common/interfaces/schemas"


@pytest.fixture(scope="module")
def config():
    return load_marvin_scene_config(CONFIG_PATH)


@pytest.fixture(scope="module")
def scene(config):
    return build_marvin_scene(config)


def test_robot_and_task_are_selected_independently(config):
    assert config.robot.assembly == "marvin"
    assert config.task.environment == "handover"
    assert config.urdf_path == (
        ROOT
        / "simulation/mujoco/assets/marvin/urdf/Marvin_Stand_M6_2026.4.8.urdf"
    )
    assert config.urdf_path.is_file()
    assert config.robot.gripper_closed_total < config.robot.gripper_open_total


def test_profile_order_cameras_and_calibrations_match_the_scene(config):
    profile = load_profile(PROFILE_PATH, SCHEMAS)
    shared_profiles = ROOT / "shared/scenarios/marvin_sim/profiles"
    observations = load_document(shared_profiles / "observations/marvin_rgb_joint.json")
    actions = load_document(shared_profiles / "actions/marvin_joint_position.json")
    validate_bindings(profile, observations, actions, SCHEMAS)
    for camera, profile_camera in zip(config.cameras, profile.cameras, strict=True):
        declaration = camera_declared_description(camera)
        assert semantic_hash(declaration) == profile_camera.calibration_digest


def test_scene_has_a_fixed_root_and_named_bindings(scene):
    model = scene.model
    assert (model.nq, model.nv, model.nu, model.ncam, model.neq, model.ntendon) == (
        25,
        24,
        16,
        3,
        2,
        2,
    )
    robot = scene.indices.robot
    assert int(model.body_jntnum[robot.base_body_id]) == 0
    for side in ("left", "right"):
        assert len(robot.arm_joint_qpos[side]) == 7
        assert len(robot.finger_joint_qpos[side]) == 2
        assert len(robot.arm_actuator_ids[side]) == 7
        assert robot.tcp_site_ids[side] >= 0
    assert model.body("Arm_L9_Link").name == "Arm_L9_Link"
    assert model.body("Arm_R9_Link").name == "Arm_R9_Link"


def test_robot_weight_compensation_does_not_compensate_the_free_object(config):
    scene = build_marvin_scene(config)
    model, data = scene.model, scene.data
    block = data.joint(f"{config.task.block.name}_free")
    block.qpos[2] += 0.10
    initial_height = float(block.qpos[2])
    mujoco.mj_forward(model, data)
    initial_joints = {
        name: data.joint(name).qpos.copy()
        for chain in scene.robot_geometry.chains
        for name in chain.joint_names
    }
    mujoco.mj_step(model, data, nstep=20)
    assert initial_height - float(block.qpos[2]) > 0.005
    for name, initial in initial_joints.items():
        np.testing.assert_allclose(data.joint(name).qpos, initial, atol=1e-5)


def test_only_the_structurally_overlapping_self_pair_is_excluded(scene):
    model = scene.model
    assert model.nexclude == 2
    pairs = set()
    for index in range(model.nexclude):
        signature = int(model.exclude_signature[index])
        pairs.add(
            frozenset(
                (
                    model.body(signature >> 16).name,
                    model.body(signature & 0xFFFF).name,
                )
            )
        )
    assert pairs == {
        frozenset(("Arm_L5_Link", "Arm_L7_Link")),
        frozenset(("Arm_R5_Link", "Arm_R7_Link")),
    }


def test_finger_coupling_comes_from_the_source_mimic_once(scene):
    model = scene.model
    assert model.neq == 2
    for prefix in ("L", "R"):
        equality = model.equality(f"Arm_{prefix}_finger_coupling")
        assert int(model.eq_type[equality.id]) == int(mujoco.mjtEq.mjEQ_JOINT)
        assert {
            model.joint(int(model.eq_obj1id[equality.id])).name,
            model.joint(int(model.eq_obj2id[equality.id])).name,
        } == {f"Arm_{prefix}10_Joint", f"Arm_{prefix}11_Joint"}
        assert float(model.eq_data[equality.id, 0]) == pytest.approx(0.0)
        assert float(model.eq_data[equality.id, 1]) == pytest.approx(1.0)


def test_reset_is_deterministic_and_gripper_opening_is_measured(scene):
    scene.reset(0)
    first = scene.data.qpos.copy()
    scene.reset(0)
    np.testing.assert_array_equal(scene.data.qpos, first)
    for side in ("left", "right"):
        assert gripper_opening(scene.model, scene.data, scene.indices.robot, side) == (
            pytest.approx(scene.gripper_open_total, abs=5e-4)
        )
    scene.data.ctrl[scene.indices.robot.gripper_actuator_ids["left"]] = (
        scene.gripper_open_total - scene.gripper_closed_total
    )
    for _ in range(1000):
        mujoco.mj_step(scene.model, scene.data)
    assert gripper_opening(scene.model, scene.data, scene.indices.robot, "left") == (
        pytest.approx(scene.gripper_closed_total, abs=5e-4)
    )


def test_box_is_open_at_the_centre_and_contains_the_block(scene, config):
    model = scene.model
    data = scene.data
    scene.reset(0)
    box = config.task.box
    origin = scene.data.xpos[scene.indices.environment.box_body_id]
    top = float(origin[2]) + box.height
    bottom = float(origin[2]) + box.bottom_thickness
    geomid = np.array([-1], dtype=np.int32)
    distance = mujoco.mj_ray(
        model,
        data,
        np.array([origin[0], origin[1], top + 0.1]),
        np.array([0.0, 0.0, -1.0]),
        None,
        True,
        -1,
        geomid,
    )
    assert int(geomid[0]) == scene.indices.environment.box_geom_ids["box_bottom"]
    assert distance == pytest.approx(top + 0.1 - bottom, abs=1e-6)
    assert len(scene.indices.environment.box_geom_ids) == 5


def test_profile_and_scene_reject_mismatched_selection(config):
    assert config.robot.assembly == "marvin"
    shared_profiles = ROOT / "shared/scenarios/marvin_sim/profiles"
    with pytest.raises(ContractError, match="Joint ordering"):
        profile = load_profile(PROFILE_PATH, SCHEMAS)
        observations = load_document(
            shared_profiles / "observations/marvin_rgb_joint.json"
        )
        observations["joint_names"] = list(reversed(observations["joint_names"]))
        validate_bindings(profile, observations, load_document(
            shared_profiles / "actions/marvin_joint_position.json"
        ), SCHEMAS)


def test_marvin_visual_geoms_have_white_body_and_black_grippers(scene):
    white = (0.92, 0.92, 0.92, 1.0)
    black = (0.12, 0.12, 0.12, 1.0)
    model = scene.model
    arm_bodies = {
        "ZJ_Robot_link",
        *(f"Arm_{side}{idx}_Link" for side in ("L", "R") for idx in range(1, 8)),
    }
    gripper_bodies = {
        f"Arm_{side}{idx}_Link" for side in ("L", "R") for idx in range(8, 12)
    }
    seen_arm = 0
    seen_gripper = 0
    for geom_id in range(model.ngeom):
        body_name = model.body(int(model.geom_bodyid[geom_id])).name
        rgba = tuple(float(v) for v in model.geom_rgba[geom_id])
        if body_name in arm_bodies:
            assert rgba == pytest.approx(white, abs=1e-5)
            seen_arm += 1
        elif body_name in gripper_bodies:
            assert rgba == pytest.approx(black, abs=1e-5)
            seen_gripper += 1
    assert seen_arm > 0
    assert seen_gripper > 0
