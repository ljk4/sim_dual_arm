from __future__ import annotations

import copy
import math
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
SCENARIO = ROOT / "shared/scenarios/marvin_sim/stack"
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.common.kinematics import common  # noqa: E402
from robot_backend_sim.marvin_sim.stack import plan  # noqa: E402
from robot_backend_sim.marvin_sim.stack.kinematics_config import (  # noqa: E402
    STACK_KEY_POSES,
    KinematicsConfigError,
    load_kinematics_config,
)
from robot_backend_sim.marvin_sim.stack.scene_config import (  # noqa: E402
    load_marvin_stack_scene_config,
)

SCENE_PATH = SCENARIO / "configs/simulation/scene/marvin_stack.yaml"
KINEMATICS_PATH = SCENARIO / "configs/simulation/scene/stack_paths.yaml"


@pytest.fixture(scope="module")
def scene_config():
    return load_marvin_stack_scene_config(SCENE_PATH)


@pytest.fixture(scope="module")
def config(scene_config):
    return load_kinematics_config(KINEMATICS_PATH, scene_config)


def document() -> dict[str, object]:
    return yaml.safe_load(KINEMATICS_PATH.read_text(encoding="utf-8"))


def write_document(tmp_path: Path, value: object, name: str = "kinematics.yaml") -> Path:
    path = tmp_path / name
    path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")
    return path


def test_config_records_all_mandated_initial_values(config):
    assert config.ik.dt_s == 0.02
    assert config.ik.max_iterations == 1200
    assert config.ik.position_tolerance_m == 0.001
    assert config.ik.orientation_tolerance_rad == 0.005
    assert config.ik.posture_cost == 0.01
    assert config.ik.position_cost == 10.0
    assert config.ik.orientation_cost == 10.0
    assert config.ik.lm_damping == 0.1
    assert config.ik.limit_margin_rad == 0.05
    assert config.ik.solver == "daqp"
    assert config.ik.initial_right_joint_positions == pytest.approx(
        (0.74, -1.815, 1.778, -1.264, 0.635, -0.413, -0.88)
    )
    assert config.clearance.minimum_clearance_m == 0.003
    assert config.clearance.collision_detection_distance_m == 0.03
    assert config.clearance.sample_step_rad == 0.02
    assert config.clearance.max_samples_per_segment == 1000
    assert config.targets.pregrasp_height_m == 0.09
    assert config.targets.grasp_height_m == 0.032
    assert config.targets.lift_height_m == 0.14
    assert config.targets.place_hover_height_m == 0.11
    assert config.targets.place_clearance_m == 0.006
    assert config.targets.retreat_height_m == 0.15
    assert config.targets.approach_tilt_deg == 20.0
    assert config.targets.approach_yaw_deg == -90.0
    assert config.targets.place_yaw_deg == -90.0


@pytest.mark.parametrize(
    "mutate, field",
    [
        (lambda d: d.__setitem__("unknown", 1), "unknown"),
        (lambda d: d["ik"].__setitem__("extra", 1), "ik.extra"),
        (lambda d: d["ik"].__setitem__("dt_s", True), "ik.dt_s"),
        (lambda d: d["ik"].__setitem__("dt_s", float("nan")), "ik.dt_s"),
        (lambda d: d["ik"].__setitem__("dt_s", float("inf")), "ik.dt_s"),
        (lambda d: d["ik"].__setitem__("max_iterations", 5001), "ik.max_iterations"),
        (lambda d: d["ik"].__setitem__("max_iterations", 10**80), "ik.max_iterations"),
        (
            lambda d: d["clearance"].__setitem__("sample_step_rad", 0.06),
            "clearance.sample_step_rad",
        ),
        (
            lambda d: d["clearance"].__setitem__("max_samples_per_segment", 5001),
            "clearance.max_samples_per_segment",
        ),
        (lambda d: d["ik"].__setitem__("solver", "quadprog"), "ik.solver"),
        (
            lambda d: d["ik"]["initial_right_joint_positions"].__setitem__(0, 3.1),
            "ik.initial_right_joint_positions[0]",
        ),
        (
            lambda d: d["ik"].__setitem__("initial_right_joint_positions", [0.0] * 6),
            "ik.initial_right_joint_positions",
        ),
    ],
)
def test_config_rejects_invalid_values_with_file_and_field(tmp_path, scene_config, mutate, field):
    value = copy.deepcopy(document())
    mutate(value)
    path = write_document(tmp_path, value)
    with pytest.raises(KinematicsConfigError) as error:
        load_kinematics_config(path, scene_config)
    assert str(path) in str(error.value)
    assert error.value.field == field


def test_config_rejects_duplicate_yaml_keys(tmp_path, scene_config):
    path = tmp_path / "duplicate.yaml"
    text = KINEMATICS_PATH.read_text(encoding="utf-8")
    path.write_text(text + "\nclearance: {}\n", encoding="utf-8")
    with pytest.raises(KinematicsConfigError, match="duplicate key"):
        load_kinematics_config(path, scene_config)


def test_targets_use_snapshot_centres_and_configured_heights(config):
    source = common.ObjectPose("source", (0.55, -0.08, 0.80), math.radians(10.0))
    target = common.ObjectPose("target", (0.55, -0.30, 0.81), math.radians(-4.0))
    targets = plan.build_targets(
        source,
        target,
        table_top_z=0.78,
        source_half_z=0.02,
        target_half_z=0.03,
        config=config.targets,
    )
    assert tuple(item.phase for item in targets) == STACK_KEY_POSES
    positions = {item.phase: item.position for item in targets}
    assert positions["grasp"] == pytest.approx((0.55, -0.08, 0.832))
    assert positions["pregrasp"] == pytest.approx((0.55, -0.08, 0.922))
    assert positions["lift"] == pytest.approx((0.55, -0.08, 0.972))
    assert positions["place_hover"][:2] == pytest.approx(positions["place"][:2])
    assert positions["place_hover"][2] - positions["place"][2] == pytest.approx(
        config.targets.place_hover_height_m
    )
    assert positions["retreat"][:2] == pytest.approx(positions["place"][:2])
    assert positions["retreat"][2] - positions["place"][2] == pytest.approx(
        config.targets.retreat_height_m
    )
    # Arm_R_tcp local -Y approaches at tilt=20 deg downward (-cos(20) along world Z).
    rotation = targets[0].rotation
    tilt = math.radians(config.targets.approach_tilt_deg)
    assert -rotation[2][1] == pytest.approx(-math.cos(tilt))
    determinant = (
        rotation[0][0] * (rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1])
        - rotation[0][1] * (rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0])
        + rotation[0][2] * (rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0])
    )
    assert determinant == pytest.approx(1.0)


def test_path_sampling_includes_endpoints_and_bounds_step():
    samples = common.sample_joint_path(
        (0.0,) * 7,
        (0.05,) + (0.0,) * 6,
        max_step_rad=0.01,
        max_samples=2000,
    )
    assert samples[0] == (0.0,) * 7
    assert samples[-1] == (0.05,) + (0.0,) * 6
    assert len(samples) == 6
    steps = [
        abs(a - b)
        for first, second in zip(samples, samples[1:], strict=False)
        for a, b in zip(first, second, strict=True)
    ]
    assert max(steps) <= 0.01 + 1e-12


def test_path_sampling_rejects_excessive_or_invalid_input():
    with pytest.raises(plan.KinematicsError, match="limit"):
        common.sample_joint_path(
            (0.0,) * 7,
            (1.0,) * 7,
            max_step_rad=0.01,
            max_samples=10,
        )
    with pytest.raises(plan.KinematicsError, match="finite"):
        common.sample_joint_path(
            (math.nan,) + (0.0,) * 6,
            (0.0,) * 7,
            max_step_rad=0.01,
            max_samples=10,
        )
    with pytest.raises(plan.KinematicsError, match="same non-zero length"):
        common.sample_joint_path((0.0,), (), max_step_rad=0.01, max_samples=10)
