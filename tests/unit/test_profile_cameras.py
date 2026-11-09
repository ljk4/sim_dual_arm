import json
from copy import deepcopy
from dataclasses import FrozenInstanceError
from pathlib import Path

import pytest
from robot_contracts.models import CameraSpec, ContractError
from robot_contracts.profiles import (
    load_document,
    load_profile,
    normalize_cameras,
    semantic_hash,
    validate_bindings,
)

ROOT = Path(__file__).resolve().parents[2]
SCHEMAS = ROOT / "contracts/schemas"
V1_PROFILE = ROOT / "profiles/robots/mock_dual_arm_v1/profile.json"
PANDA_PROFILE = ROOT / "profiles/robots/panda_stack_v1/profile.json"
PANDA_OBSERVATION = ROOT / "profiles/observations/panda_rgb_joint_v1.json"
PANDA_ACTION = ROOT / "profiles/actions/panda_joint_position_v1.json"


def _panda_document():
    return deepcopy(load_document(PANDA_PROFILE))


def _write(tmp_path, document, name="profile.json"):
    path = tmp_path / name
    path.write_text(json.dumps(document))
    return path


def test_v1_profile_keeps_original_hash_and_normalizes_one_camera():
    raw = load_document(V1_PROFILE)
    profile = load_profile(V1_PROFILE, SCHEMAS)

    assert profile.digest == semantic_hash(raw)
    assert profile.test_only is True
    assert profile.camera_names == ("front",)
    assert profile.cameras == (CameraSpec("front", 64, 48, "front_optical_frame", ""),)
    # A version-specific schema directory remains a valid call site.
    assert load_profile(V1_PROFILE, SCHEMAS / "v1").digest == profile.digest


def test_v2_profile_is_an_immutable_multi_camera_view():
    raw = _panda_document()
    profile = load_profile(PANDA_PROFILE, SCHEMAS)
    cameras = normalize_cameras(raw)

    assert profile.test_only is False
    assert profile.cameras == cameras
    assert profile.camera_names == ("global", "left_wrist", "right_wrist")
    assert len({camera.frame_id for camera in cameras}) == 3
    assert all(len(camera.calibration_digest) == 64 for camera in cameras)

    raw["cameras"][0]["name"] = "mutated"
    raw["joints"][0]["initial"] = 9.0
    assert profile.cameras[0].name == "global"
    assert profile.joints[0].initial == 0.0

    with pytest.raises(FrozenInstanceError):
        profile.cameras[0].name = "changed"


def test_panda_bindings_have_si_units_and_exact_orders():
    profile = load_profile(PANDA_PROFILE, SCHEMAS)
    observation = load_document(PANDA_OBSERVATION)
    action = load_document(PANDA_ACTION)

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
    assert profile.joints[7].minimum == 0.0
    assert profile.joints[7].maximum == 0.08
    assert profile.joints[7].initial == 0.08
    assert profile.joints[0].minimum == -2.8973
    assert profile.joints[3].maximum == -0.0698
    assert profile.joints[0].max_velocity == 0.8
    assert profile.joints[7].max_velocity == 0.04
    assert [(camera.width, camera.height) for camera in profile.cameras] == [
        (320, 240),
        (320, 240),
        (320, 240),
    ]


@pytest.mark.parametrize("version", [True, False, "2", 0, 3, None])
def test_unknown_or_boolean_profile_version_is_rejected(tmp_path, version):
    document = _panda_document()
    document["schema_version"] = version
    path = _write(tmp_path, document)

    with pytest.raises(ContractError, match="schema_version") as caught:
        load_profile(path, SCHEMAS)
    assert caught.value.reason == "SCHEMA_MISMATCH"
    assert str(path) in str(caught.value)


@pytest.mark.parametrize("change", ["name", "frame_id"])
def test_duplicate_camera_identity_is_rejected(tmp_path, change):
    document = _panda_document()
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
    raw = json.dumps(_panda_document())
    path = tmp_path / "duplicate.json"
    duplicate = raw.replace(
        '"robot_id": "panda_stack"',
        '"robot_id": "panda_stack", "robot_id": "other"',
    )
    path.write_text(duplicate)

    with pytest.raises(ContractError, match="duplicate key") as caught:
        load_profile(path, SCHEMAS)
    assert "robot_id" in str(caught.value)
    assert str(path) in str(caught.value)


def test_unknown_camera_field_and_missing_digest_are_rejected(tmp_path):
    with_unknown = _panda_document()
    with_unknown["cameras"][0]["lens"] = "wide"
    with pytest.raises(ContractError, match="lens"):
        load_profile(_write(tmp_path, with_unknown, "unknown.json"), SCHEMAS)

    without_digest = _panda_document()
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
    document = _panda_document()
    document["cameras"][0][field] = value
    with pytest.raises(ContractError, match=field):
        load_profile(_write(tmp_path, document), SCHEMAS)


def test_more_than_eight_cameras_are_rejected(tmp_path):
    document = _panda_document()
    template = document["cameras"][0]
    document["cameras"] = [
        {**template, "name": f"camera{index}", "frame_id": f"camera{index}_frame"}
        for index in range(9)
    ]
    with pytest.raises(ContractError, match="cameras"):
        load_profile(_write(tmp_path, document), SCHEMAS)


@pytest.mark.parametrize("change", ["joint_order", "camera_order", "camera_missing"])
def test_binding_mismatch_is_rejected(change):
    profile = load_profile(PANDA_PROFILE, SCHEMAS)
    observation = load_document(PANDA_OBSERVATION)
    action = load_document(PANDA_ACTION)
    if change == "joint_order":
        observation["joint_names"].reverse()
    elif change == "camera_order":
        observation["camera_names"].reverse()
    else:
        observation["camera_names"].pop()
    with pytest.raises(ContractError, match="SCHEMA_MISMATCH"):
        validate_bindings(profile, observation, action, SCHEMAS)


def test_camera_digest_is_the_scene_description_semantic_hash():
    scene_path = ROOT / "simulation/mujoco/configs/scene/panda_stack.yaml"
    raw_cameras = load_document(scene_path)["cameras"]
    profile = load_profile(PANDA_PROFILE, SCHEMAS)
    for camera in profile.cameras:
        description = {"name": camera.name, **raw_cameras[camera.name]}
        assert camera.calibration_digest == semantic_hash(description)


@pytest.mark.parametrize("field", ["min", "max", "max_velocity", "initial"])
def test_unrepresentable_joint_number_is_a_contract_error(tmp_path, field):
    document = _panda_document()
    document["joints"][0][field] = 10**3500
    path = _write(tmp_path, document)

    with pytest.raises(ContractError, match=f"joints\\[0\\]\\.{field}") as caught:
        load_profile(path, SCHEMAS)
    assert caught.value.reason == "SCHEMA_MISMATCH"
    assert str(path) in str(caught.value)


@pytest.mark.parametrize(
    "name",
    ["../wrong", "bad/name", "bad name", ".hidden", "9camera", "camera-name", "x" * 65],
)
def test_v2_camera_name_must_be_a_ros_topic_basename(tmp_path, name):
    document = _panda_document()
    document["cameras"][0]["name"] = name
    path = _write(tmp_path, document)

    with pytest.raises(ContractError, match=r"cameras(\.0|\[0\])\.name") as caught:
        load_profile(path, SCHEMAS)
    assert caught.value.reason == "SCHEMA_MISMATCH"
    assert str(path) in str(caught.value)


@pytest.mark.parametrize("name", ["_camera", "camera_1", "Global", "x" * 64])
def test_v2_camera_name_accepts_safe_topic_basenames(tmp_path, name):
    document = _panda_document()
    document["cameras"][0]["name"] = name
    path = _write(tmp_path, document)

    assert load_profile(path, SCHEMAS).cameras[0].name == name


def test_v1_legacy_camera_name_is_not_retroactively_restricted(tmp_path):
    document = deepcopy(load_document(V1_PROFILE))
    document["camera"]["name"] = "legacy/camera"
    path = _write(tmp_path, document, "legacy.json")

    assert load_profile(path, SCHEMAS).camera_names == ("legacy/camera",)
