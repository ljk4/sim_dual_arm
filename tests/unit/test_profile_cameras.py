import json
from copy import deepcopy
from pathlib import Path

import pytest
from pydantic import ValidationError
from robot_contracts.models import CameraSpec, ContractError, RobotProfile
from robot_contracts.profiles import (
    load_document,
    load_profile,
    normalize_cameras,
    semantic_hash,
    validate_bindings,
)
from scenario_fixtures import MARVIN_SHARED_PROFILES

ROOT = Path(__file__).resolve().parents[2]
PROFILES = ROOT / "shared/scenarios/marvin_sim/stack/profiles"
SCHEMAS = ROOT / "shared/common/interfaces/schemas"
STACK_PROFILE = PROFILES / "robots/marvin_stack/profile.json"
STACK_OBSERVATION = MARVIN_SHARED_PROFILES / "observations/marvin_rgb_joint.json"
STACK_ACTION = MARVIN_SHARED_PROFILES / "actions/marvin_joint_position.json"


def _stack_document():
    return deepcopy(load_document(STACK_PROFILE))


def _write(tmp_path, document, name="profile.json"):
    path = tmp_path / name
    path.write_text(json.dumps(document))
    return path


def test_profile_supports_one_camera_and_preserves_the_source_hash(tmp_path):
    document = _stack_document()
    document["cameras"] = document["cameras"][:1]
    path = _write(tmp_path, document)
    profile = load_profile(path, SCHEMAS)

    assert profile.digest == semantic_hash(document)
    assert profile.camera_names == ("global",)


def test_profile_is_an_immutable_multi_camera_view():
    raw = _stack_document()
    profile = load_profile(STACK_PROFILE, SCHEMAS)
    cameras = normalize_cameras(raw)

    assert profile.test_only is False
    assert profile.cameras == cameras
    assert profile.camera_names == ("global", "left_wrist", "right_wrist")
    assert len({camera.frame_id for camera in cameras}) == 3
    assert all(len(camera.calibration_digest) == 64 for camera in cameras)

    raw["cameras"][0]["name"] = "mutated"
    raw["joints"][0]["initial"] = 9.0
    assert profile.cameras[0].name == "global"
    assert profile.joints[0].initial == 0.75

    with pytest.raises(ValidationError):
        profile.cameras[0].name = "changed"


def test_stack_bindings_have_si_units_and_exact_orders():
    profile = load_profile(STACK_PROFILE, SCHEMAS)
    observation = load_document(STACK_OBSERVATION)
    action = load_document(STACK_ACTION)

    validate_bindings(profile, observation, action, SCHEMAS)

    expected = (
        *[f"left_joint{index}" for index in range(1, 8)],
        "left_gripper_width",
        *[f"right_joint{index}" for index in range(1, 8)],
        "right_gripper_width",
    )
    assert profile.joint_names == expected
    assert tuple(observation["camera_names"]) == ("global", "left_wrist", "right_wrist")
    assert [joint.unit for joint in profile.joints] == ["rad"] * 7 + ["m"] + ["rad"] * 7 + ["m"]
    assert profile.joints[7].minimum == 0.0014
    assert profile.joints[7].maximum == 0.0814
    assert profile.joints[7].initial == 0.0814
    assert profile.joints[0].minimum == -3.1067
    assert profile.joints[3].maximum == 1.0472
    assert profile.joints[0].max_velocity == 0.8
    assert profile.joints[7].max_velocity == 0.04
    assert [(camera.width, camera.height) for camera in profile.cameras] == [
        (320, 240),
        (320, 240),
        (320, 240),
    ]


@pytest.mark.parametrize("change", ["name", "frame_id"])
def test_duplicate_camera_identity_is_rejected(tmp_path, change):
    document = _stack_document()
    if change == "name":
        document["cameras"][1]["name"] = document["cameras"][0]["name"]
        fragment = "duplicate camera names"
    else:
        document["cameras"][2]["frame_id"] = document["cameras"][0]["frame_id"]
        fragment = "duplicate camera frame_id"
    path = _write(tmp_path, document)

    with pytest.raises(ContractError, match=fragment):
        load_profile(path, SCHEMAS)


def test_duplicate_profile_field_is_rejected(tmp_path):
    raw = json.dumps(_stack_document())
    path = tmp_path / "duplicate.json"
    duplicate = raw.replace(
        '"robot_id": "marvin_stack"',
        '"robot_id": "marvin_stack", "robot_id": "other"',
    )
    path.write_text(duplicate)

    with pytest.raises(ContractError, match="duplicate key") as caught:
        load_profile(path, SCHEMAS)
    assert "robot_id" in str(caught.value)
    assert str(path) in str(caught.value)


def test_unknown_camera_field_and_missing_digest_are_rejected(tmp_path):
    with_unknown = _stack_document()
    with_unknown["cameras"][0]["lens"] = "wide"
    with pytest.raises(ContractError, match="lens"):
        load_profile(_write(tmp_path, with_unknown, "unknown.json"), SCHEMAS)

    without_digest = _stack_document()
    del without_digest["cameras"][0]["calibration_digest"]
    with pytest.raises(ContractError, match="calibration_digest"):
        load_profile(_write(tmp_path, without_digest, "missing.json"), SCHEMAS)


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("width", 0),
        ("width", 641),
        ("height", 0),
        ("height", 481),
        ("width", True),
        ("calibration_digest", "A" * 64),
        ("calibration_digest", "0" * 63),
    ],
)
def test_invalid_camera_values_are_rejected(tmp_path, field, value):
    document = _stack_document()
    document["cameras"][0][field] = value
    with pytest.raises(ContractError, match=field):
        load_profile(_write(tmp_path, document), SCHEMAS)


def test_more_than_eight_cameras_are_rejected(tmp_path):
    document = _stack_document()
    template = document["cameras"][0]
    document["cameras"] = [
        {**template, "name": f"camera{index}", "frame_id": f"camera{index}_frame"}
        for index in range(9)
    ]
    with pytest.raises(ContractError, match="cameras"):
        load_profile(_write(tmp_path, document), SCHEMAS)


@pytest.mark.parametrize("change", ["joint_order", "camera_order", "camera_missing"])
def test_binding_mismatch_is_rejected(change):
    profile = load_profile(STACK_PROFILE, SCHEMAS)
    observation = load_document(STACK_OBSERVATION)
    action = load_document(STACK_ACTION)
    if change == "joint_order":
        observation["joint_names"].reverse()
    elif change == "camera_order":
        observation["camera_names"].reverse()
    else:
        observation["camera_names"].pop()
    with pytest.raises(ContractError, match="SCHEMA_MISMATCH"):
        validate_bindings(profile, observation, action, SCHEMAS)


def test_camera_digest_is_the_scene_description_semantic_hash():
    scene_path = (
        ROOT / "shared/scenarios/marvin_sim/stack/configs/simulation/scene/marvin_stack.yaml"
    )
    raw_cameras = load_document(scene_path)["cameras"]
    profile = load_profile(STACK_PROFILE, SCHEMAS)
    for camera in profile.cameras:
        description = {"name": camera.name, **raw_cameras[camera.name]}
        assert camera.calibration_digest == semantic_hash(description)


@pytest.mark.parametrize("field", ["min", "max", "max_velocity", "initial"])
def test_unrepresentable_joint_number_is_a_contract_error(tmp_path, field):
    document = _stack_document()
    document["joints"][0][field] = 10**3500
    path = _write(tmp_path, document)

    with pytest.raises(ContractError) as caught:
        load_profile(path, SCHEMAS)
    assert caught.value.reason == "SCHEMA_MISMATCH"
    assert "joints[0]" in str(caught.value)
    assert field in str(caught.value)
    assert str(path) in str(caught.value)


@pytest.mark.parametrize(
    "name",
    ["../wrong", "bad/name", "bad name", ".hidden", "9camera", "camera-name", "x" * 65],
)
def test_v2_camera_name_must_be_a_ros_topic_basename(tmp_path, name):
    document = _stack_document()
    document["cameras"][0]["name"] = name
    path = _write(tmp_path, document)

    with pytest.raises(ContractError, match=r"cameras(\.0|\[0\])\.name") as caught:
        load_profile(path, SCHEMAS)
    assert caught.value.reason == "SCHEMA_MISMATCH"
    assert str(path) in str(caught.value)


@pytest.mark.parametrize("name", ["_camera", "camera_1", "Global", "x" * 64])
def test_v2_camera_name_accepts_safe_topic_basenames(tmp_path, name):
    document = _stack_document()
    document["cameras"][0]["name"] = name
    path = _write(tmp_path, document)

    assert load_profile(path, SCHEMAS).cameras[0].name == name


def test_legacy_camera_object_is_rejected_by_file_and_model_loaders(tmp_path):
    document = _stack_document()
    document["camera"] = document.pop("cameras")[0]
    path = _write(tmp_path, document)

    with pytest.raises(ContractError, match="camera"):
        load_profile(path, SCHEMAS)
    with pytest.raises(ContractError, match="cameras"):
        RobotProfile.validate_dict(document)
    with pytest.raises(ContractError, match="cameras"):
        normalize_cameras(document)


@pytest.mark.parametrize("digest", [None, "", "0" * 63])
def test_direct_camera_model_requires_a_calibration_digest(digest):
    document = _stack_document()["cameras"][0]
    if digest is None:
        del document["calibration_digest"]
    else:
        document["calibration_digest"] = digest
    with pytest.raises(ContractError, match="calibration_digest"):
        CameraSpec.validate_dict(document)
