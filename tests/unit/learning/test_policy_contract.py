import json
from pathlib import Path

import pytest

pytest.importorskip("torch")
pytest.importorskip("lerobot")
import yaml
from act_fixtures import ACTION, OBSERVATION, PROFILE, make_act_model  # noqa: E402
from dual_arm_learning.serving.config import (  # noqa: E402
    PolicyConfigError,
    PolicyServiceConfig,
    load_policy_config,
    parse_policy_config,
)
from dual_arm_learning.training.bundle import deployment_summary  # noqa: E402
from fake_provider import load_fake_config

ROOT = Path(__file__).resolve().parents[3]
CONFIG = ROOT / "shared/scenarios/marvin/mujoco/stack/configs/learning/policy.yaml"
FAKE = ROOT / "tests/fakes/policy/fake_policy.yaml"


def resolve(model: Path, *, require_formal: bool = False):
    return load_policy_config(
        CONFIG,
        model_path=model,
        robot_profile_path=PROFILE,
        observation_schema_path=OBSERVATION,
        action_schema_path=ACTION,
        require_formal=require_formal,
    )


def test_contract_reports_model_identity_and_rejects_synthetic_formal(tmp_path):
    model = make_act_model(tmp_path / "act_20260101_000000")
    contract = resolve(model)
    summary = deployment_summary(contract)
    assert summary["checkpoint_step"] == 10
    assert summary["synthetic"] is True
    assert summary["state_dim"] == 16
    with pytest.raises(PolicyConfigError, match="synthetic"):
        resolve(model, require_formal=True)


def test_contract_rejects_wrong_observation_schema(tmp_path):
    model = make_act_model(tmp_path / "act_20260101_000000")
    wrong = tmp_path / "observation.json"
    document = json.loads(OBSERVATION.read_text())
    document["joint_names"] = list(reversed(document["joint_names"]))
    wrong.write_text(json.dumps(document) + "\n", encoding="utf-8")
    with pytest.raises(PolicyConfigError, match="joint order"):
        load_policy_config(
            CONFIG,
            model_path=model,
            robot_profile_path=PROFILE,
            observation_schema_path=wrong,
            action_schema_path=ACTION,
            require_formal=False,
        )


def test_contract_rejects_a_missing_model(tmp_path):
    with pytest.raises(PolicyConfigError, match="model.safetensors"):
        resolve(tmp_path / "missing")


def test_temporal_ensemble_setting_is_optional_and_strictly_bounded():
    template = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    assert template["model"]["temporal_ensemble_coeff"] is None

    assert PolicyServiceConfig.model_fields["temporal_ensemble_coeff"].default is None
    service = yaml.safe_load(FAKE.read_text(encoding="utf-8"))["service"]
    for value in (True, float("nan"), float("inf"), -0.01, 1.01, "0.1"):
        document = dict(service)
        document["temporal_ensemble_coeff"] = value
        with pytest.raises(PolicyConfigError, match="temporal_ensemble_coeff"):
            parse_policy_config(document)


def test_fake_service_config_is_unchanged_by_model_derivation():
    config, settings = load_fake_config(FAKE)
    assert config.chunk_size == 60
    assert settings.fault == "none"
