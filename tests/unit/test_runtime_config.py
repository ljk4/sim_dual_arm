from pathlib import Path

import pytest
import yaml
from dual_arm_runtime.runtime_config import (
    MAX_LEASE_DURATION_NS,
    load_runtime_configuration,
)
from robot_contracts.episodes import (
    DIAGNOSTIC_EPISODE_SCHEMA_VERSION,
    POLICY_EPISODE_SCHEMA_VERSION,
    manifest_schema_path,
)
from robot_contracts.models import ContractError

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "tests/fixtures/simulation/policy_unit.yaml"


def document():
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))


def write(tmp_path, value, *, absolute=True):
    if absolute:
        for field in (
            "robot_profile",
            "observation_schema",
            "action_schema",
            "control_constraints",
            "follower_config",
        ):
            if field in value:
                value[field] = str((CONFIG.parent / value[field]).resolve())
    tmp_path.mkdir(parents=True, exist_ok=True)
    path = tmp_path / "runtime.yaml"
    path.write_text(yaml.safe_dump(value), encoding="utf-8")
    return path


def test_config_snapshot_has_complete_constraints_and_motions():
    configuration = load_runtime_configuration(CONFIG, ROOT)
    snapshot = configuration.snapshot
    assert snapshot["profile_hash"] == configuration.profile.digest
    assert snapshot["recipe"]["reset"]["seed"] == 0
    assert snapshot["recipe"]["motions"] == []
    assert snapshot["recipe"]["control_constraints"]["max_accelerations"]
    assert snapshot["recipe"]["follower"]["target_count"] > 0
    assert snapshot["observation"]["camera_names"] == list(configuration.profile.camera_names)
    # A fresh snapshot cannot mutate runtime configuration.
    snapshot["recipe"]["reset"]["seed"] = 99
    assert configuration.snapshot["recipe"]["reset"]["seed"] != 99


def test_unknown_key_and_bool_integer_are_rejected(tmp_path):
    value = document()
    value["unknown"] = 1
    with pytest.raises(ContractError):
        load_runtime_configuration(write(tmp_path, value), ROOT)
    value = document()
    value["lease_duration_ns"] = True
    with pytest.raises(ContractError) as caught:
        load_runtime_configuration(write(tmp_path, value), ROOT)
    assert "lease_duration_ns" in str(caught.value)


def test_lease_renew_and_follower_dimensions_are_checked(tmp_path):
    value = document()
    value["lease_duration_ns"] = MAX_LEASE_DURATION_NS + 1
    with pytest.raises(ContractError, match="lease_duration_ns"):
        load_runtime_configuration(write(tmp_path, value), ROOT)
    value = document()
    value["renew_interval_ns"] = value["lease_duration_ns"]
    with pytest.raises(ContractError, match="less than half"):
        load_runtime_configuration(write(tmp_path, value), ROOT)
    value = document()
    follower_path = (CONFIG.parent / value["follower_config"]).resolve()
    follower_document = yaml.safe_load(follower_path.read_text(encoding="utf-8"))
    follower_document["max_target_deviations"] = follower_document["max_target_deviations"][:-1]
    follower_file = tmp_path / "follower-short.yaml"
    follower_file.write_text(yaml.safe_dump(follower_document), encoding="utf-8")
    value["follower_config"] = str(follower_file)
    with pytest.raises(ContractError, match="must match the profile joints"):
        load_runtime_configuration(write(tmp_path, value), ROOT)


def test_missing_referenced_file_fails_with_source(tmp_path):
    value = document()
    value["control_constraints"] = "missing.yaml"
    with pytest.raises(ContractError, match="does not exist"):
        load_runtime_configuration(write(tmp_path, value, absolute=False), ROOT)


def test_manifest_schema_is_a_single_v1():
    assert DIAGNOSTIC_EPISODE_SCHEMA_VERSION == 1
    assert POLICY_EPISODE_SCHEMA_VERSION == 1
    assert (
        manifest_schema_path(ROOT / "shared/common/interfaces/schemas", 1)
        .as_posix()
        .endswith("v1/episode_manifest.schema.json")
    )
    with pytest.raises(ContractError) as caught:
        manifest_schema_path(ROOT / "shared/common/interfaces/schemas", 2)
    assert caught.value.reason == "UNSUPPORTED_SCHEMA_VERSION"


def test_policy_max_chunks_can_cover_a_120_chunk_task(tmp_path):
    policy_config = CONFIG
    value = yaml.safe_load(policy_config.read_text(encoding="utf-8"))
    for field in (
        "robot_profile",
        "observation_schema",
        "action_schema",
        "control_constraints",
        "follower_config",
    ):
        value[field] = str((policy_config.parent / value[field]).resolve())
    value["policy"]["max_chunks"] = 120
    value["policy"]["total_sim_deadline_ns"] = 30_000_000_000
    value["policy"]["total_wall_deadline_s"] = 60.0
    path = write(tmp_path, value, absolute=False)
    configuration = load_runtime_configuration(path, ROOT)
    assert configuration.recipe.policy.max_chunks == 120
    assert configuration.recipe.policy.total_sim_deadline_ns == 30_000_000_000
    value["policy"]["max_chunks"] = 2049
    path = write(tmp_path / "over", value, absolute=False)
    with pytest.raises(ContractError, match="max_chunks"):
        load_runtime_configuration(path, ROOT)


def test_teacher_task_budget_is_bounded_and_covered_by_chunks(tmp_path):
    value = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    for field in (
        "robot_profile",
        "observation_schema",
        "action_schema",
        "control_constraints",
        "follower_config",
    ):
        value[field] = str((CONFIG.parent / value[field]).resolve())
    # This budget fixture intentionally models a ten-point window.
    follower = yaml.safe_load(Path(value["follower_config"]).read_text())
    follower["target_count"] = 10
    fixture_follower = tmp_path / "follower-budget.yaml"
    fixture_follower.write_text(yaml.safe_dump(follower))
    value["follower_config"] = str(fixture_follower)
    value.pop("policy")
    value["action_source"] = "teacher"
    value["teacher_api"] = "panda_stack_api_msgs.teacher_chunk.v1"
    value["teacher"] = {
        "chunk_size": 60,
        "action_period_ns": 50_000_000,
        "initial_lead_ns": 250_000_000,
        "max_chunks": 400,
        "total_sim_deadline_ns": 180_000_000_000,
        "total_wall_deadline_s": 240.0,
        "prepare_timeout_s": 12.0,
        "start_timeout_s": 5.0,
        "chunk_deadline_s": 2.0,
        "poll_interval_s": 0.02,
        "termination_mode": "task",
        "task_stop_margin_s": 1.0,
        "task_sim_budget_ns": 180_000_000_000,
        "task_id": "handover",
    }
    path = write(tmp_path / "teacher", value, absolute=False)
    configuration = load_runtime_configuration(path, ROOT)
    assert configuration.recipe.teacher.task_sim_budget_ns == 180_000_000_000
    assert configuration.recipe.teacher.max_chunks == 400
    assert "panda_stack_api_msgs.teacher_chunk.v1" in (configuration.required_capabilities)

    # 180 s at a ten-point, 50 ms window needs 360 chunks.
    value["teacher"]["max_chunks"] = 359
    path = write(tmp_path / "short", value, absolute=False)
    with pytest.raises(ContractError, match="at least 360 chunks"):
        load_runtime_configuration(path, ROOT)

    # The wall watchdog must cover the simulation budget plus cleanup.
    value["teacher"]["max_chunks"] = 400
    value["teacher"]["total_wall_deadline_s"] = 190.0
    path = write(tmp_path / "wall", value, absolute=False)
    with pytest.raises(ContractError, match="total_wall_deadline_s"):
        load_runtime_configuration(path, ROOT)

    # The bounded implementation limit still caps longer requests.
    value["teacher"]["total_wall_deadline_s"] = 400.0
    value["teacher"]["task_sim_budget_ns"] = 301_000_000_000
    path = write(tmp_path / "unbounded", value, absolute=False)
    with pytest.raises(ContractError, match="task_sim_budget_ns"):
        load_runtime_configuration(path, ROOT)


def policy_document():
    value = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    value["policy"].pop("startup_settle_sim_ns", None)
    return value


def write_policy(tmp_path, value):
    base = CONFIG
    for field in (
        "robot_profile",
        "observation_schema",
        "action_schema",
        "control_constraints",
        "follower_config",
    ):
        value[field] = str((base.parent / value[field]).resolve())
    tmp_path.mkdir(parents=True, exist_ok=True)
    path = tmp_path / "policy.yaml"
    path.write_text(yaml.safe_dump(value), encoding="utf-8")
    return path


def test_policy_startup_settle_defaults_to_disabled(tmp_path):
    configuration = load_runtime_configuration(write_policy(tmp_path, policy_document()), ROOT)
    assert configuration.recipe.policy.startup_settle_sim_ns == 0
    assert configuration.snapshot["recipe"]["policy"]["startup_settle_sim_ns"] == 0


@pytest.mark.parametrize("value", [0, 2_000_000_000, 5_000_000_000])
def test_policy_startup_settle_accepts_zero_to_five_seconds(tmp_path, value):
    document = policy_document()
    document["policy"]["startup_settle_sim_ns"] = value
    configuration = load_runtime_configuration(write_policy(tmp_path, document), ROOT)
    assert configuration.recipe.policy.startup_settle_sim_ns == value
    assert configuration.snapshot["recipe"]["policy"]["startup_settle_sim_ns"] == value


@pytest.mark.parametrize("value", [-1, True, 1.5, "1000", 5_000_000_001])
def test_policy_startup_settle_rejects_invalid_values(tmp_path, value):
    document = policy_document()
    document["policy"]["startup_settle_sim_ns"] = value
    with pytest.raises(ContractError) as caught:
        load_runtime_configuration(write_policy(tmp_path, document), ROOT)
    assert "startup_settle_sim_ns" in str(caught.value)
