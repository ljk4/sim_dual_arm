from pathlib import Path

import pytest
import yaml
from dual_arm_learning.serving.config import (
    PolicyConfigError,
    PolicyServiceConfig,
    parse_policy_config,
)

ROOT = Path(__file__).resolve().parents[3]
FAKE = ROOT / "tests/fakes/policy/fake_policy.yaml"


def service_document():
    return yaml.safe_load(FAKE.read_text())["service"]


def test_fake_service_config_is_complete_and_ros_free():
    config = parse_policy_config(service_document())
    assert config.chunk_size == 60
    assert config.action_period_ns == 50_000_000
    assert len(config.joint_names) == 16
    assert config.action_joint_names == config.joint_names
    assert config.camera_names == ("cam_high", "cam_left_wrist", "cam_right_wrist")
    assert config.test_only is True
    assert config.chunk_duration_ns == 3_000_000_000


def test_unknown_and_unsafe_fields_are_rejected():
    value = service_document()
    value["unknown"] = 1
    with pytest.raises(PolicyConfigError, match="unknown"):
        parse_policy_config(value)
    value = service_document()
    value["test_only"] = "false"
    with pytest.raises(PolicyConfigError, match="test_only"):
        parse_policy_config(value)
    value = service_document()
    value["chunk_size"] = 0
    with pytest.raises(PolicyConfigError, match="chunk_size"):
        parse_policy_config(value)
    value = service_document()
    value["action_period_ns"] = 0
    with pytest.raises(PolicyConfigError, match="action_period_ns"):
        parse_policy_config(value)


def test_load_fake_config_keeps_provider_settings_out_of_the_service_schema():
    from fake_provider import load_fake_config

    config, settings = load_fake_config(FAKE)
    assert isinstance(config, object)
    assert settings.fault == "none"
    assert parse_policy_config(service_document()).chunk_size == 60


def test_correction_quotas_are_per_axis_and_strictly_validated():
    assert "gripper_max_correction_m" not in PolicyServiceConfig.model_fields
    assert PolicyServiceConfig.model_fields["max_prediction_corrections"].default_factory() == {}

    document = service_document()
    document["max_prediction_corrections"] = {
        "left_joint7": 0.01,
        "left_gripper_width": 0.005,
    }
    config = parse_policy_config(document)
    assert config.max_prediction_corrections == {
        "left_joint7": 0.01,
        "left_gripper_width": 0.005,
    }

    for value in (
        {"missing_joint": 0.01},
        {"left_joint7": -0.01},
        {"left_joint7": float("inf")},
        {"left_joint7": float("nan")},
        {"left_joint7": "0.01"},
    ):
        document = service_document()
        document["max_prediction_corrections"] = value
        with pytest.raises(PolicyConfigError, match="max_prediction_corrections"):
            parse_policy_config(document)


def test_soft_prediction_corrections_and_streak_budget_are_strictly_validated():
    assert PolicyServiceConfig.model_fields["soft_prediction_corrections"].default_factory() == {}
    assert PolicyServiceConfig.model_fields["max_consecutive_soft_overruns"].default == 0

    document = service_document()
    document["max_prediction_corrections"] = {
        "left_gripper_width": 0.015,
        "right_gripper_width": 0.015,
    }
    document["soft_prediction_corrections"] = {
        "left_gripper_width": 0.03,
        "right_gripper_width": 0.03,
        "right_joint6": 0.02,
    }
    document["max_consecutive_soft_overruns"] = 2
    config = parse_policy_config(document)
    assert config.soft_prediction_corrections == {
        "left_gripper_width": 0.03,
        "right_gripper_width": 0.03,
        "right_joint6": 0.02,
    }
    assert config.max_consecutive_soft_overruns == 2

    for bad_soft in (
        {"missing_joint": 0.02},
        {"right_joint6": -0.01},
        {"right_joint6": float("inf")},
        {"right_joint6": float("nan")},
        {"right_joint6": "0.02"},
        {"left_gripper_width": 0.01},  # < Tier-1 0.015
    ):
        doc = service_document()
        doc["max_prediction_corrections"] = {"left_gripper_width": 0.015}
        doc["soft_prediction_corrections"] = bad_soft
        with pytest.raises(PolicyConfigError, match="soft_prediction_corrections"):
            parse_policy_config(doc)

    for bad_streak in (-1, 11, 1.5, "2", True):
        doc = service_document()
        doc["max_consecutive_soft_overruns"] = bad_streak
        with pytest.raises(PolicyConfigError, match="max_consecutive_soft_overruns"):
            parse_policy_config(doc)
