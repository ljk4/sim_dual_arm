from __future__ import annotations

import dataclasses
import sys
from pathlib import Path

import pytest

mujoco = pytest.importorskip("mujoco")
np = pytest.importorskip("numpy")
pytest.importorskip("mink")

ROOT = Path(__file__).resolve().parents[2]
SCENARIO = ROOT / "shared/scenarios/marvin_sim/stack"
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.common.kinematics import common, kinematics_clearance  # noqa: E402
from robot_backend_sim.marvin_sim.stack import plan as stack_plan  # noqa: E402
from robot_backend_sim.marvin_sim.stack.kinematics_config import load_kinematics_config  # noqa: E402
from robot_backend_sim.marvin_sim.stack.scene import build_marvin_stack_scene  # noqa: E402
from robot_backend_sim.marvin_sim.stack.scene_config import load_marvin_stack_scene_config  # noqa: E402

CONFIG_PATH = SCENARIO / "configs/simulation/scene/marvin_stack.yaml"
KINEMATICS_PATH = SCENARIO / "configs/simulation/scene/stack_paths.yaml"


@pytest.fixture(scope="module")
def scene_config():
    return load_marvin_stack_scene_config(CONFIG_PATH)


@pytest.fixture(scope="module")
def kinematics_config(scene_config):
    return load_kinematics_config(KINEMATICS_PATH, scene_config)


@pytest.fixture(scope="module")
def scene(scene_config):
    return build_marvin_stack_scene(
        scene_config,
    )


def initial_state(scene, scene_config, seed: int = 11):
    scene.reset(seed)
    qpos = scene.data.qpos.copy()
    object_addresses = {
        obj.name: scene.indices.environment.object_free_joint_qpos[obj.name]
        for obj in scene_config.objects
    }
    poses = {
        name: common.extract_object_pose(qpos, address, name)
        for name, address in object_addresses.items()
    }
    return qpos, poses


def make_targets(scene_config, kinematics_config, poses):
    source = scene_config.objects[0]
    target = scene_config.objects[1]
    return stack_plan.build_targets(
        poses["source"],
        poses["target"],
        table_top_z=scene_config.robot.base_position[2] + scene_config.table.top_z,
        source_half_z=source.half_extents[2],
        target_half_z=target.half_extents[2],
        config=kinematics_config.targets,
    )


def test_real_mink_solves_six_targets_and_freezes_every_other_dof(
    scene, scene_config, kinematics_config
):
    assert kinematics_config.targets.approach_tilt_deg == 20.0
    assert kinematics_config.targets.approach_yaw_deg == -90.0
    assert kinematics_config.targets.place_yaw_deg == -90.0
    assert kinematics_config.clearance.minimum_clearance_m == 0.003

    qpos, poses = initial_state(scene, scene_config)
    scene_data_before = {
        "qpos": scene.data.qpos.copy(),
        "qvel": scene.data.qvel.copy(),
        "time": float(scene.data.time),
    }
    pair_set = stack_plan.stack_pair_set(scene.model)
    targets = make_targets(scene_config, kinematics_config, poses)
    results = stack_plan.solve_key_poses(
        scene.model,
        qpos,
        targets,
        kinematics_config.ik,
        kinematics_config.clearance,
        pair_set,
    )
    assert len(results) == 6
    assert all(result.converged for result in results)
    for result in results:
        assert result.position_error_m <= kinematics_config.ik.position_tolerance_m
        assert result.orientation_error_rad <= kinematics_config.ik.orientation_tolerance_rad
        assert result.frozen_max_deviation <= 1e-8
        assert np.isfinite(np.asarray(result.qpos_right)).all()
    # Only the seven right hinge qpos addresses may differ from the snapshot.
    right_addresses = set(scene.indices.robot.arm_joint_qpos["right"])
    final = np.asarray(results[-1].qpos)
    for address in range(scene.model.nq):
        if address not in right_addresses:
            assert final[address] == pytest.approx(qpos[address], abs=1e-8)
    np.testing.assert_array_equal(scene.data.qpos, scene_data_before["qpos"])
    np.testing.assert_array_equal(scene.data.qvel, scene_data_before["qvel"])
    assert scene.data.time == scene_data_before["time"]


def test_unreachable_target_fails_instead_of_reporting_false_success(
    scene, scene_config, kinematics_config
):
    qpos, poses = initial_state(scene, scene_config)
    pair_set = stack_plan.stack_pair_set(scene.model)
    target = dataclasses.replace(
        make_targets(scene_config, kinematics_config, poses)[0],
        phase="unreachable",
        position=(5.0, 5.0, 5.0),
    )
    results = stack_plan.solve_key_poses(
        scene.model,
        qpos,
        (target,),
        kinematics_config.ik,
        kinematics_config.clearance,
        pair_set,
    )
    assert len(results) == 1
    assert results[0].converged is False
    assert results[0].reason in {"ik_not_converged", "ik_solver_failed"}


def test_real_penetrating_endpoint_is_rejected_by_path_clearance(
    scene, scene_config, kinematics_config
):
    qpos, _poses = initial_state(scene, scene_config)
    pair_set = stack_plan.stack_pair_set(scene.model)
    addresses = tuple(scene.indices.robot.arm_joint_qpos["right"])
    home = tuple(float(qpos[address]) for address in addresses)
    penetrating = None
    for joint2 in (-1.5, -1.0, -0.5, 0.0):
        for joint4 in (-2.5, -2.0, -1.5, -1.0):
            for joint6 in (-1.0, 0.0, 1.0):
                candidate = (0.0, joint2, 0.0, joint4, 0.0, joint6, 0.0)
                sample = np.array(qpos, copy=True)
                for address, value in zip(addresses, candidate, strict=True):
                    sample[address] = value
                check = stack_plan.check_initial_pose(
                    scene.model,
                    sample,
                    pair_set,
                    kinematics_config.clearance,
                )
                if not check.passed:
                    penetrating = candidate
                    break
            if penetrating is not None:
                break
        if penetrating is not None:
            break
    assert penetrating is not None, "deterministic grid found no penetrating pose"
    path = stack_plan.check_path(
        scene.model,
        qpos,
        home,
        penetrating,
        pair_set,
        kinematics_config.clearance,
        segment="real_penetration_probe",
    )
    assert path.passed is False
    assert path.failure_reason == "clearance_below_minimum"
    assert path.minimum.distance_m < kinematics_config.clearance.minimum_clearance_m


def test_plane_with_zero_rbound_is_not_culled_and_reports_penetration():
    max_model = mujoco.MjModel.from_xml_string(
        """
        <mujoco>
          <worldbody>
            <geom name="floor" type="plane" size="1 1 0.1"/>
            <body name="right_probe" pos="0 0 0.05">
              <joint name="right_joint1" type="hinge" axis="0 1 0" range="-1 1"/>
              <geom name="right_ball" type="sphere" size="0.1"/>
            </body>
          </worldbody>
        </mujoco>
        """
    )
    data = mujoco.MjData(max_model)
    mujoco.mj_forward(max_model, data)
    pair_set = kinematics_clearance.CollisionPairSet(
        pairs=(
            kinematics_clearance.CollisionPair(
                geom1=0,
                geom2=1,
                body1="world",
                body2="right_probe",
                geom1_name="floor",
                geom2_name="right_ball",
                category="robot_scene",
            ),
        ),
        categories={"robot_scene": 1},
        exclusions={},
        total_physical_pairs=1,
        explicit_pair_count=0,
    )
    point = kinematics_clearance.measure_clearance(
        max_model,
        data,
        pair_set,
        detection_distance_m=0.03,
        sample_index=0,
        fraction=0.0,
    )
    assert max_model.geom_rbound[0] == 0.0
    assert point.measured_pairs == 1
    assert point.lower_bound is False
    assert point.distance_m < 0.0
    assert point.distance_m < 0.002


def test_collision_pair_enumeration_records_mujoco_exclusions(scene):
    pair_set = stack_plan.stack_pair_set(scene.model)
    assert pair_set.pairs
    assert len(pair_set.pairs) == 259
    assert pair_set.categories.get("robot_self", 0) > 0
    assert pair_set.categories.get("robot_cross", 0) > 0
    assert pair_set.categories.get("robot_scene", 0) > 0
    assert pair_set.exclusions.get("visual_or_no_affinity", 0) > 0
    assert pair_set.exclusions.get("parent_child", 0) > 0
    assert pair_set.exclusions.get("welded_body", 0) > 0
    assert pair_set.exclusions.get("explicit_exclude", 0) > 0
