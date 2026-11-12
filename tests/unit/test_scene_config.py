from __future__ import annotations

import copy
import math
import sys
from pathlib import Path

import pytest
import yaml
from model_helpers import replace
from pydantic import ValidationError

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.config.scene_config import (  # noqa: E402
    SceneConfigError,
    camera_declared_description,
    load_scene_config,
)
from robot_contracts.profiles import load_profile, semantic_hash  # noqa: E402

CONFIG_PATH = ROOT / "shared/scenarios/panda_stack/configs/simulation/scene/panda_stack.yaml"
PROFILE_PATH = ROOT / "shared/scenarios/panda_stack/profiles/robots/panda_stack_v1/profile.json"
SCHEMAS = ROOT / "shared/common/interfaces/schemas"
DEFAULT_DOCUMENT = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))


def write_document(tmp_path: Path, document: dict) -> Path:
    path = tmp_path / "panda_stack.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    return path


def config_with(tmp_path: Path, mutate) -> Path:
    document = copy.deepcopy(DEFAULT_DOCUMENT)
    mutate(document)
    return write_document(tmp_path, document)


def set_value(*path, value):
    def mutate(document):
        target = document
        for key in path[:-1]:
            target = target[key]
        target[path[-1]] = value

    return mutate


def test_default_config_loads_frozen_typed_values():
    config = load_scene_config(CONFIG_PATH)

    assert config.physics.timestep == pytest.approx(0.002)
    assert config.physics.gravity == (0.0, 0.0, -9.81)
    assert config.column.radius == pytest.approx(0.12)
    assert config.mounts.base_x == pytest.approx(0.25)
    assert config.table.top_z == pytest.approx(0.75)
    assert [obj.name for obj in config.objects] == ["source", "target"]
    assert [camera.name for camera in config.cameras] == [
        "global",
        "left_wrist",
        "right_wrist",
    ]
    assert all(camera.width == 320 and camera.height == 240 for camera in config.cameras)
    assert config.reset.home_joint_positions == (
        0.0,
        0.0,
        0.0,
        -1.57079,
        0.0,
        1.57079,
        -0.7853,
    )

    with pytest.raises(ValidationError):
        config.physics.timestep = 0.01  # type: ignore[misc]
    with pytest.raises(ValidationError):
        config.objects[0].name = "other"  # type: ignore[misc]
    with pytest.raises(ValidationError):
        config.cameras[0].declared_description_json = "{}"  # type: ignore[misc]
    assert isinstance(config.objects, tuple)
    assert isinstance(config.cameras, tuple)
    assert all(isinstance(camera.declared_description_json, str) for camera in config.cameras)


def test_camera_declarations_match_accepted_profile_digest():
    config = load_scene_config(CONFIG_PATH)
    profile = load_profile(PROFILE_PATH, SCHEMAS)

    assert [camera.name for camera in config.cameras] == list(profile.camera_names)
    for camera, profile_camera in zip(config.cameras, profile.cameras, strict=True):
        declaration = camera_declared_description(camera)
        assert declaration["name"] == profile_camera.name
        assert declaration["width"] == profile_camera.width
        assert declaration["height"] == profile_camera.height
        assert semantic_hash(declaration) == profile_camera.calibration_digest


def test_camera_declared_description_returns_fresh_mutable_values():
    camera = load_scene_config(CONFIG_PATH).cameras[1]
    original_json = camera.declared_description_json
    first = camera_declared_description(camera)
    second = camera_declared_description(camera)

    assert first == second
    assert first is not second
    assert first["position"] is not second["position"]
    first["name"] = "changed"
    first["position"][0] = 99.0
    first["width"] = 1
    assert camera.declared_description_json == original_json
    assert camera_declared_description(camera) == second


def test_camera_declaration_preserves_integer_and_float_source_values(tmp_path):
    def mutate(document):
        camera = document["cameras"]["global"]
        camera["position"] = [0, 0.03, 1]
        camera["quaternion_wxyz"] = [1, 0, 0, 0]
        camera["fovy"] = 60

    path = config_with(tmp_path, mutate)
    camera = load_scene_config(path).cameras[0]
    declaration = camera_declared_description(camera)

    assert declaration["position"] == [0, 0.03, 1]
    assert [type(value) for value in declaration["position"]] == [int, float, int]
    assert declaration["quaternion_wxyz"] == [1, 0, 0, 0]
    assert all(type(value) is int for value in declaration["quaternion_wxyz"])
    assert declaration["fovy"] == 60
    assert type(declaration["fovy"]) is int
    assert camera.position == (0.0, 0.03, 1.0)
    assert camera.quaternion_wxyz == (1.0, 0.0, 0.0, 0.0)
    assert camera.fovy == 60.0


def test_replaced_typed_camera_geometry_with_old_declaration_is_rejected():
    camera = load_scene_config(CONFIG_PATH).cameras[0]
    changed = replace(
        camera,
        position=(camera.position[0] + 0.1, *camera.position[1:]),
    )

    with pytest.raises(SceneConfigError) as caught:
        camera_declared_description(changed)
    assert caught.value.field == "cameras.global.position"


def test_error_reports_file_and_field_source(tmp_path):
    path = config_with(tmp_path, set_value("physics", "timestep", value=True))

    with pytest.raises(SceneConfigError) as caught:
        load_scene_config(path)

    assert caught.value.source == str(path)
    assert caught.value.field == "physics.timestep"
    assert str(path) in str(caught.value)
    assert "physics.timestep" in str(caught.value)


@pytest.mark.parametrize(
    ("mutate", "field"),
    [
        (set_value("physics", "timestep", value=math.nan), "physics.timestep"),
        (
            set_value("physics", "gravity", value=[0.0, math.inf, 0.0]),
            "physics.gravity[1]",
        ),
        (set_value("physics", "gravity", value=[0.0, 0.0, 0.0]), "physics.gravity[2]"),
        (set_value("column", "radius", value=0.0), "column.radius"),
        (set_value("column", "height", value=-1.0), "column.height"),
        (set_value("table", "size_xy", value=[1.2, 0.0]), "table.size_xy[1]"),
        (set_value("table", "thickness", value=0.0), "table.thickness"),
        (set_value("objects", 0, "mass", value=True), "objects[0].mass"),
        (
            set_value("objects", 0, "half_extents", value=[0.0, 0.02, 0.02]),
            "objects[0].half_extents[0]",
        ),
        (
            set_value("cameras", "global", "fovy", value=200.0),
            "cameras[0].fovy",
        ),
        (
            set_value("reset", "gripper_total_opening", value=0.09),
            "reset.gripper_total_opening",
        ),
    ],
)
def test_rejects_boolean_nonfinite_and_out_of_range_numbers(tmp_path, mutate, field):
    path = config_with(tmp_path, mutate)

    with pytest.raises(SceneConfigError) as caught:
        load_scene_config(path)

    assert caught.value.field == field


@pytest.mark.parametrize(
    ("mutate", "field"),
    [
        (set_value("physics", "timestep", value=math.nan), "physics.timestep"),
        (set_value("physics", "unexpected", value=1), "physics.unexpected"),
        (set_value("unexpected", value=1), "unexpected"),
    ],
)
def test_unknown_keys_and_nan_are_rejected_with_field(tmp_path, mutate, field):
    path = config_with(tmp_path, mutate)

    with pytest.raises(SceneConfigError) as caught:
        load_scene_config(path)

    assert caught.value.field == field


def test_missing_required_key_is_rejected(tmp_path):
    def mutate(document):
        del document["physics"]["timestep"]

    path = config_with(tmp_path, mutate)
    with pytest.raises(SceneConfigError) as caught:
        load_scene_config(path)
    assert caught.value.field == "physics.timestep"


def test_duplicate_yaml_key_is_rejected(tmp_path):
    text = CONFIG_PATH.read_text(encoding="utf-8").replace(
        "  timestep: 0.002", "  timestep: 0.002\n  timestep: 0.003", 1
    )
    path = tmp_path / "panda_stack.yaml"
    path.write_text(text, encoding="utf-8")

    with pytest.raises(SceneConfigError) as caught:
        load_scene_config(path)

    assert caught.value.field == "timestep"
    assert "duplicate key" in str(caught.value)


@pytest.mark.parametrize(
    "quaternion",
    [
        [0.0, 0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [math.nan, 0.0, 0.0, 0.0],
        [2.0, 0.0, 0.0, 0.0],
    ],
)
def test_invalid_camera_quaternion_is_rejected(tmp_path, quaternion):
    def mutate(document):
        document["cameras"]["left_wrist"]["quaternion_wxyz"] = quaternion

    path = config_with(tmp_path, mutate)
    with pytest.raises(SceneConfigError) as caught:
        load_scene_config(path)
    assert caught.value.field.startswith("cameras[1].quaternion_wxyz")


def test_camera_parent_and_shared_resolution_are_enforced(tmp_path):
    parent_path = config_with(
        tmp_path, lambda document: document["cameras"]["global"].__setitem__("parent", "world")
    )
    with pytest.raises(SceneConfigError) as caught:
        load_scene_config(parent_path)
    assert caught.value.field == "cameras[0].parent"

    resolution_path = config_with(
        tmp_path,
        lambda document: document["cameras"]["right_wrist"].__setitem__("width", 640),
    )
    with pytest.raises(SceneConfigError) as caught:
        load_scene_config(resolution_path)
    assert caught.value.field == "cameras"


def test_duplicate_object_name_is_rejected(tmp_path):
    def mutate(document):
        document["objects"][1]["name"] = document["objects"][0]["name"]

    path = config_with(tmp_path, mutate)
    with pytest.raises(SceneConfigError) as caught:
        load_scene_config(path)
    assert caught.value.field == "objects[1].name"
    assert "duplicate object name" in str(caught.value)


def test_randomized_object_may_not_cross_table_edge(tmp_path):
    def mutate(document):
        document["objects"][0]["nominal_xy"] = [0.57, 0.55]

    path = config_with(tmp_path, mutate)
    with pytest.raises(SceneConfigError) as caught:
        load_scene_config(path)
    assert caught.value.field == "objects[0].nominal_xy[0]"
    assert "table edge" in str(caught.value)


def test_randomized_objects_may_not_overlap(tmp_path):
    def mutate(document):
        document["objects"][1]["nominal_xy"] = [0.24, 0.55]

    path = config_with(tmp_path, mutate)
    with pytest.raises(SceneConfigError) as caught:
        load_scene_config(path)
    assert caught.value.field == "objects[1].nominal_xy"
    assert "overlap" in str(caught.value)


def test_object_count_and_randomization_limits_are_enforced(tmp_path):
    path = config_with(
        tmp_path, lambda document: document["objects"][0]["randomization"].__setitem__("xy_m", 0.04)
    )
    with pytest.raises(SceneConfigError) as caught:
        load_scene_config(path)
    assert caught.value.field == "objects[0].randomization.xy_m"

    def remove_object(document):
        document["objects"].pop()

    path = config_with(tmp_path, remove_object)
    with pytest.raises(SceneConfigError) as caught:
        load_scene_config(path)
    assert caught.value.field == "objects"


def test_non_mapping_and_missing_file_are_rejected(tmp_path):
    path = config_with(tmp_path, lambda document: document.__setitem__("physics", []))
    with pytest.raises(SceneConfigError) as caught:
        load_scene_config(path)
    assert caught.value.field == "physics"

    missing = tmp_path / "missing.yaml"
    with pytest.raises(SceneConfigError) as caught:
        load_scene_config(missing)
    assert caught.value.source == str(missing)
    assert caught.value.field == "<file>"


@pytest.mark.parametrize("index,value", [(3, 0.5), (1, 2.0)])
def test_home_uses_fixed_per_joint_limits(tmp_path, index, value):
    positions = list(DEFAULT_DOCUMENT["reset"]["home_joint_positions"])
    positions[index] = value
    path = config_with(tmp_path, set_value("reset", "home_joint_positions", value=positions))

    with pytest.raises(SceneConfigError) as caught:
        load_scene_config(path)
    assert caught.value.field == f"reset.home_joint_positions[{index}]"


def test_home_accepts_legal_joint4_and_joint6_values(tmp_path):
    positions = [0.0, 0.0, 0.0, -3.0, 0.0, 3.2, -0.7853]
    path = config_with(tmp_path, set_value("reset", "home_joint_positions", value=positions))

    config = load_scene_config(path)
    assert config.reset.home_joint_positions == tuple(positions)


def test_huge_integer_is_rejected_without_leaking_overflow(tmp_path):
    path = config_with(tmp_path, set_value("physics", "timestep", value=2**4096))

    with pytest.raises(SceneConfigError) as caught:
        load_scene_config(path)
    assert caught.value.field == "physics.timestep"


def test_diagonal_jitter_worst_case_overlap_is_rejected(tmp_path):
    def mutate(document):
        document["objects"][0]["nominal_xy"] = [0.20, 0.50]
        document["objects"][1]["nominal_xy"] = [0.30, 0.60]

    path = config_with(tmp_path, mutate)
    with pytest.raises(SceneConfigError) as caught:
        load_scene_config(path)
    assert caught.value.field == "objects[1].nominal_xy"
    assert "overlap" in str(caught.value)


def test_overlap_boundary_uses_worst_case_axis_separation(tmp_path):
    footprint = math.hypot(0.02, 0.02) + math.hypot(0.03, 0.03)
    boundary_x = 0.20 + 0.06 + footprint

    def mutate(target_x):
        def apply(document):
            document["objects"][1]["nominal_xy"] = [target_x, 0.55]

        return apply

    accepted = config_with(tmp_path, mutate(boundary_x + 1e-5))
    load_scene_config(accepted)

    rejected = config_with(tmp_path, mutate(boundary_x - 1e-5))
    with pytest.raises(SceneConfigError) as caught:
        load_scene_config(rejected)
    assert caught.value.field == "objects[1].nominal_xy"
