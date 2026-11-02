from pathlib import Path

import pytest
import yaml
from dual_arm_learning.policy_config import (
    PolicyConfigError,
    load_policy_config,
    parse_policy_config,
)

ROOT = Path(__file__).resolve().parents[2]
FAKE = ROOT / "configs/learning/fake_policy.yaml"


def service_document():
    return yaml.safe_load(FAKE.read_text())["service"]


def test_fake_service_config_is_complete_and_ros_free():
    config = parse_policy_config(service_document())
    assert config.chunk_size == 60
    assert config.action_period_ns == 50_000_000
    assert len(config.joint_names) == 16
    assert config.action_joint_names == config.joint_names
    assert config.camera_names == ("global", "left_wrist", "right_wrist")
    assert config.test_only is True
    assert config.chunk_duration_ns == 3_000_000_000


def test_unknown_and_unsafe_fields_are_rejected():
    value = service_document()
    value["unknown"] = 1
    with pytest.raises(PolicyConfigError, match="unknown field"):
        parse_policy_config(value)
    value = service_document()
    value["test_only"] = "false"
    with pytest.raises(PolicyConfigError, match="must be a boolean"):
        parse_policy_config(value)
    value = service_document()
    value["chunk_size"] = 59
    with pytest.raises(PolicyConfigError, match="60"):
        parse_policy_config(value)
    value = service_document()
    value["action_period_ns"] = 40_000_000
    with pytest.raises(PolicyConfigError, match="50 ms"):
        parse_policy_config(value)


def test_load_fake_config_keeps_provider_settings_out_of_the_service_schema():
    from fake_provider import load_fake_config

    config, settings = load_fake_config(FAKE)
    assert isinstance(config, object)
    assert settings.fault == "none"
    assert load_policy_config(ROOT / "configs/learning/policy.yaml").chunk_size == 60
