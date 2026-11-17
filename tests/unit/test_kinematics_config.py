from __future__ import annotations

import copy
import math
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
SCENARIO = ROOT / "shared/scenarios/panda_sim/stack"
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.config.kinematics_config import (  # noqa: E402
    KinematicsConfigError,
    load_kinematics_config,
)
from robot_backend_sim.config.panda_scene_config import load_scene_config  # noqa: E402
from robot_backend_sim.kinematics import common, kinematics  # noqa: E402
from robot_backend_sim.render import visibility  # noqa: E402

SCENE_PATH = SCENARIO / "configs/simulation/scene/panda_stack.yaml"
CHECK_PATH = SCENARIO / "configs/simulation/checks/scene_check.yaml"
KINEMATICS_PATH = SCENARIO / "configs/simulation/checks/kinematics_check.yaml"


@pytest.fixture(scope="module")
def scene_config():
    return load_scene_config(SCENE_PATH)


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
    assert config.ik.dt_s == 0.01
    assert config.ik.max_iterations == 250
    assert config.ik.position_tolerance_m == 0.0005
    assert config.ik.orientation_tolerance_rad == 0.005
    assert config.ik.posture_cost == 0.01
    assert config.ik.position_cost == 1.0
    assert config.ik.orientation_cost == 1.0
    assert config.ik.lm_damping == 0.001
    assert config.ik.solver == "daqp"
    assert config.ik.initial_right_joint_positions == pytest.approx(
        (
            0.6611657875566201,
            1.053952789173405,
            -0.47077198599796316,
            -1.8999275750261582,
            1.0656643185801171,
            2.6750836691327793,
            1.7547643964342687,
        )
    )
    assert config.clearance.minimum_clearance_m == 0.002
    assert config.clearance.collision_detection_distance_m == 0.03
    assert config.clearance.sample_step_rad == 0.01
    assert config.clearance.max_samples_per_segment == 2000
    assert config.targets.pregrasp_height_m == 0.12
    assert config.targets.grasp_height_m == 0.004
    assert config.targets.lift_height_m == 0.20
    assert config.targets.place_hover_height_m == 0.16
    assert config.targets.place_clearance_m == 0.006
    assert config.targets.retreat_height_m == 0.20


def test_config_reuses_the_67_layout_boundary_domain(config, scene_config):
    check = visibility.load_check_config(CHECK_PATH, scene_config)
    layouts = visibility.build_layouts(check, scene_config)
    assert len(layouts) == 67
    assert [layout.kind for layout in layouts[:3]] == ["seed", "seed", "seed"]
    assert all(layout.kind == "boundary" for layout in layouts[3:])
    assert config.budgets.max_layouts == 100


def test_visibility_rules_cover_every_camera_object_phase_once(config):
    seen = set()
    for rule in config.visibility_rules:
        if rule.record_only:
            assert rule.min_pixels is None
        else:
            assert rule.min_pixels is not None and rule.min_pixels >= 1
        for phase in rule.phases:
            key = (rule.camera, rule.object, phase)
            assert key not in seen
            seen.add(key)
    for camera in config.camera_names:
        objects = list(config.object_names)
        if camera == "left_wrist":
            objects.append("table")
        for obj in objects:
            for phase in (
                "pregrasp",
                "grasp_pose",
                "lift_pose",
                "place_hover",
                "place_pose",
                "retreat",
            ):
                assert (camera, obj, phase) in seen


@pytest.mark.parametrize(
    "mutate, field",
    [
        (lambda d: d.__setitem__("unknown", 1), "unknown"),
        (lambda d: d["ik"].__setitem__("extra", 1), "ik.extra"),
        (lambda d: d["ik"].__setitem__("dt_s", True), "ik.dt_s"),
        (lambda d: d["ik"].__setitem__("dt_s", float("nan")), "ik.dt_s"),
        (lambda d: d["ik"].__setitem__("dt_s", float("inf")), "ik.dt_s"),
        (lambda d: d["ik"].__setitem__("max_iterations", 251), "ik.max_iterations"),
        (lambda d: d["ik"].__setitem__("max_iterations", 10**80), "ik.max_iterations"),
        (
            lambda d: d["clearance"].__setitem__("sample_step_rad", 0.02),
            "clearance.sample_step_rad",
        ),
        (
            lambda d: d["clearance"].__setitem__("max_samples_per_segment", 2001),
            "clearance.max_samples_per_segment",
        ),
        (lambda d: d["budgets"].__setitem__("max_layouts", 101), "budgets.max_layouts"),
        (
            lambda d: d["budgets"].__setitem__("max_png_bytes", 128 * 1024 * 1024 + 1),
            "budgets.max_png_bytes",
        ),
        (lambda d: d["ik"].__setitem__("solver", "quadprog"), "ik.solver"),
        (
            lambda d: d["ik"]["initial_right_joint_positions"].__setitem__(0, 3.0),
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
    path.write_text(text + "\nschema_version: 1\n", encoding="utf-8")
    with pytest.raises(KinematicsConfigError, match="duplicate key"):
        load_kinematics_config(path, scene_config)


def test_config_rejects_missing_or_duplicate_visibility_coverage(tmp_path, scene_config):
    missing = copy.deepcopy(document())
    missing["visibility"]["rules"].pop(0)
    missing_path = write_document(tmp_path, missing, "missing.yaml")
    with pytest.raises(KinematicsConfigError, match="missing coverage"):
        load_kinematics_config(missing_path, scene_config)

    duplicate = copy.deepcopy(document())
    duplicate["visibility"]["rules"].append(copy.deepcopy(duplicate["visibility"]["rules"][0]))
    duplicate_path = write_document(tmp_path, duplicate, "duplicate-coverage.yaml")
    with pytest.raises(KinematicsConfigError, match="duplicate rule coverage"):
        load_kinematics_config(duplicate_path, scene_config)


def test_config_rejects_record_only_threshold_and_unknown_camera(tmp_path, scene_config):
    record_threshold = copy.deepcopy(document())
    rule = next(
        item for item in record_threshold["visibility"]["rules"] if item["mode"] == "record_only"
    )
    rule["min_pixels"] = 1
    path = write_document(tmp_path, record_threshold, "record-threshold.yaml")
    with pytest.raises(KinematicsConfigError, match="must not set a threshold"):
        load_kinematics_config(path, scene_config)

    unknown_camera = copy.deepcopy(document())
    unknown_camera["visibility"]["rules"][0]["camera"] = "overhead"
    path = write_document(tmp_path, unknown_camera, "unknown-camera.yaml")
    with pytest.raises(KinematicsConfigError, match="camera"):
        load_kinematics_config(path, scene_config)


def test_targets_use_snapshot_centres_and_configured_heights(config):
    source = common.ObjectPose("source", (0.20, 0.55, 0.77), math.radians(10.0))
    target = common.ObjectPose("target", (0.36, 0.50, 0.78), math.radians(-4.0))
    targets = kinematics.build_targets(
        source,
        target,
        table_top_z=0.75,
        source_half_z=0.02,
        target_half_z=0.03,
        config=config.targets,
    )
    assert tuple(item.phase for item in targets) == (
        "pregrasp",
        "grasp_pose",
        "lift_pose",
        "place_hover",
        "place_pose",
        "retreat",
    )
    positions = {item.phase: item.position for item in targets}
    assert positions["pregrasp"] == pytest.approx((0.20, 0.55, 0.89))
    assert positions["grasp_pose"] == pytest.approx((0.20, 0.55, 0.774))
    assert positions["lift_pose"] == pytest.approx((0.20, 0.55, 0.97))
    assert positions["place_hover"] == pytest.approx((0.36, 0.50, 0.94))
    assert positions["place_pose"] == pytest.approx((0.36, 0.50, 0.836))
    assert positions["retreat"] == pytest.approx((0.36, 0.50, 0.98))
    rotation = targets[0].rotation
    expected = (
        (math.cos(math.radians(10.0)), math.sin(math.radians(10.0)), 0.0),
        (math.sin(math.radians(10.0)), -math.cos(math.radians(10.0)), 0.0),
        (0.0, 0.0, -1.0),
    )
    for actual_row, expected_row in zip(rotation, expected, strict=True):
        assert actual_row == pytest.approx(expected_row)
    assert rotation[2][2] == -1.0
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
    with pytest.raises(kinematics.KinematicsError, match="limit"):
        common.sample_joint_path(
            (0.0,) * 7,
            (1.0,) * 7,
            max_step_rad=0.01,
            max_samples=10,
        )
    with pytest.raises(kinematics.KinematicsError, match="finite"):
        common.sample_joint_path(
            (math.nan,) + (0.0,) * 6,
            (0.0,) * 7,
            max_step_rad=0.01,
            max_samples=10,
        )
    with pytest.raises(kinematics.KinematicsError, match="same non-zero length"):
        common.sample_joint_path((0.0,), (), max_step_rad=0.01, max_samples=10)


def test_png_budget_is_checked_before_creating_a_file(tmp_path):
    from PIL import Image
    from robot_backend_sim.tools import check_kinematics

    image = Image.new("RGB", (8, 8), (120, 60, 20))
    budget = check_kinematics.ArtifactBudget(limit_bytes=1)
    rejected = tmp_path / "rejected.png"
    with pytest.raises(check_kinematics.KinematicsCheckFailure) as caught:
        check_kinematics._save_png(rejected, image, budget)
    assert caught.value.reason == "png_budget_exceeded"
    assert not rejected.exists()
    assert budget.total_bytes == budget.count == 0

    budget = check_kinematics.ArtifactBudget(limit_bytes=10000)
    accepted = tmp_path / "accepted.png"
    check_kinematics._save_png(accepted, image, budget)
    actual = accepted.read_bytes()
    assert budget.total_bytes == len(actual) and budget.count == 1
    budget.limit_bytes = budget.total_bytes
    with pytest.raises(check_kinematics.KinematicsCheckFailure):
        check_kinematics._save_png(rejected, image, budget)
    assert not rejected.exists() and accepted.read_bytes() == actual
