"""Keep the learning camera variant from silently changing robot mechanics."""

import json
from pathlib import Path

import yaml
from robot_contracts.profiles import semantic_hash

ROOT = Path(__file__).resolve().parents[2]


def test_learning_scene_changes_only_the_right_wrist_field_of_view():
    original = yaml.safe_load(
        (ROOT / "simulation/mujoco/configs/scene/panda_stack.yaml").read_text()
    )
    learning = yaml.safe_load(
        (ROOT / "simulation/mujoco/configs/scene/panda_stack_learning.yaml").read_text()
    )
    assert learning["cameras"]["right_wrist"]["fovy"] == 68.0
    learning["cameras"]["right_wrist"]["fovy"] = original["cameras"]["right_wrist"]["fovy"]
    assert learning == original


def test_learning_profile_preserves_robot_and_has_a_matching_camera_digest():
    original = json.loads((ROOT / "profiles/robots/panda_stack_v1/profile.json").read_text())
    learning = json.loads(
        (ROOT / "profiles/robots/panda_stack_learning_v1/profile.json").read_text()
    )
    description = json.loads(
        (ROOT / "profiles/robots/panda_stack_learning_v1/right_wrist_calibration.json").read_text()
    )
    assert learning["cameras"][2]["calibration_digest"] == semantic_hash(description)
    learning["profile_id"] = original["profile_id"]
    learning["cameras"][2]["calibration_digest"] = original["cameras"][2]["calibration_digest"]
    assert learning == original
