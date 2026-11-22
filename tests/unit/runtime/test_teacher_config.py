from pathlib import Path

import pytest
import yaml
from dual_arm_runtime.runtime_config import load_runtime_configuration
from robot_contracts.models import ContractError

ROOT = Path(__file__).resolve().parents[3]
TEACHER = ROOT / "shared/scenarios/marvin_sim/stack/configs/runtime/teacher_collect.yaml"


def test_teacher_config_is_task_terminated_and_policy_free():
    configuration = load_runtime_configuration(TEACHER, ROOT)
    assert configuration.recipe.action_source == "teacher"
    assert configuration.recipe.policy is None
    teacher = configuration.recipe.teacher
    assert teacher is not None
    assert teacher.termination_mode == "task"
    assert teacher.max_chunks >= 120
    assert configuration.snapshot["recipe"]["teacher"]["task_sim_budget_ns"] == 90_000_000_000


def test_teacher_and_policy_sections_are_mutually_exclusive(tmp_path):
    value = yaml.safe_load(TEACHER.read_text(encoding="utf-8"))
    for field in (
        "robot_profile",
        "observation_schema",
        "action_schema",
        "control_constraints",
        "follower_config",
    ):
        value[field] = str((TEACHER.parent / value[field]).resolve())
    value["policy"] = {
        "endpoint": "127.0.0.1:50051",
        "allow_test_policy": False,
        "expected_test_only": False,
        "task_id": "task",
        "task_text": "task",
    }
    path = tmp_path / "runtime.yaml"
    path.write_text(yaml.safe_dump(value), encoding="utf-8")
    with pytest.raises(ContractError, match="policy section requires"):
        load_runtime_configuration(path, ROOT)


def test_task_mode_rejects_a_chunk_budget_that_cannot_cover_30s(tmp_path):
    value = yaml.safe_load(TEACHER.read_text(encoding="utf-8"))
    for field in (
        "robot_profile",
        "observation_schema",
        "action_schema",
        "control_constraints",
        "follower_config",
    ):
        value[field] = str((TEACHER.parent / value[field]).resolve())
    value["teacher"]["max_chunks"] = 8
    path = tmp_path / "runtime.yaml"
    path.write_text(yaml.safe_dump(value), encoding="utf-8")
    with pytest.raises(ContractError, match="needs at least"):
        load_runtime_configuration(path, ROOT)


def test_teacher_requires_an_explicit_capability_identifier(tmp_path):
    configuration = load_runtime_configuration(TEACHER, ROOT)
    assert configuration.recipe.teacher_api == "marvin_teacher_api_msgs.teacher_chunk"
    assert "marvin_teacher_api_msgs.teacher_chunk" in configuration.required_capabilities
    assert (
        "marvin_teacher_api_msgs.teacher_chunk"
        in configuration.snapshot["recipe"]["required_capabilities"]
    )
    assert "teacher_chunk" not in configuration.required_capabilities

    value = yaml.safe_load(TEACHER.read_text(encoding="utf-8"))
    for field in (
        "robot_profile",
        "observation_schema",
        "action_schema",
        "control_constraints",
        "follower_config",
    ):
        value[field] = str((TEACHER.parent / value[field]).resolve())
    value.pop("teacher_api", None)
    path = tmp_path / "runtime.yaml"
    path.write_text(yaml.safe_dump(value), encoding="utf-8")
    with pytest.raises(ContractError, match="explicit extension capability"):
        load_runtime_configuration(path, ROOT)
