import json
from pathlib import Path

import pytest

pytest.importorskip("torch")
pytest.importorskip("lerobot")
import yaml
from act_fixtures import (  # noqa: E402
    ACTION,
    OBSERVATION,
    PROFILE,
    make_act_bundle,
    reseal_bundle,
)
from dual_arm_learning.bundle import deployment_summary
from dual_arm_learning.policy_config import (
    PolicyConfigError,
    load_policy_config,
    parse_policy_config,
)
from fake_provider import load_fake_config

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "shared/scenarios/panda_stack/configs/learning/policy_temporal.yaml"
FAKE = ROOT / "learning/lerobot/configs/fake_policy.yaml"


def resolve(bundle: Path, *, require_formal: bool = False):
    return load_policy_config(
        CONFIG,
        bundle_dir=bundle,
        robot_profile_path=PROFILE,
        observation_schema_path=OBSERVATION,
        action_schema_path=ACTION,
        require_formal=require_formal,
    )


def test_contract_verifies_stats_and_rejects_tampered_synthetic(tmp_path):
    bundle = tmp_path / "bundle"
    make_act_bundle(bundle)
    contract = resolve(bundle)
    summary = deployment_summary(contract)
    assert summary["train_episodes"] == 45
    assert summary["validation_episodes"] == 5
    assert summary["checkpoint_mode"] == "full"
    assert summary["synthetic"] is True

    def mutate_stats(root: Path) -> None:
        stats = {
            "source": "validation_split",
            "features": {
                "observation.state": {
                    "mean": [0.0] * 16,
                    "std": [1.0] * 16,
                    "min": [-1.0] * 16,
                    "max": [1.0] * 16,
                },
                "action": {
                    "mean": [0.0] * 16,
                    "std": [1.0] * 16,
                    "min": [-1.0] * 16,
                    "max": [1.0] * 16,
                },
            },
        }
        (root / "processors/train_stats_summary.json").write_text(
            json.dumps(stats) + "\n", encoding="utf-8"
        )
        (root / "metadata/train_stats_summary.json").write_text(
            json.dumps(stats) + "\n", encoding="utf-8"
        )

    reseal_bundle(bundle, mutate_stats)
    with pytest.raises(PolicyConfigError, match="training-split-only"):
        resolve(bundle)


def test_contract_rejects_tampered_dataset_split_and_temporal_ensemble(tmp_path):
    bundle = tmp_path / "bundle"
    make_act_bundle(bundle)

    def mutate_dataset(root: Path) -> None:
        document = json.loads((root / "metadata/dataset_manifest.json").read_text())
        document["splits"]["training"]["episode_count"] = 44
        document["splits"]["training"]["episodes"] = document["splits"]["training"][
            "episodes"
        ][:44]
        (root / "metadata/dataset_manifest.json").write_text(
            json.dumps(document) + "\n", encoding="utf-8"
        )

    reseal_bundle(bundle, mutate_dataset)
    with pytest.raises(PolicyConfigError, match="counts disagree"):
        resolve(bundle)

    bundle = tmp_path / "bundle-temporal"
    make_act_bundle(bundle)

    def mutate_config(root: Path) -> None:
        document = json.loads((root / "config.json").read_text())
        document["temporal_ensemble_coeff"] = 0.01
        (root / "config.json").write_text(json.dumps(document) + "\n", encoding="utf-8")

    reseal_bundle(bundle, mutate_config)
    with pytest.raises(PolicyConfigError, match="temporal"):
        resolve(bundle)

    schema_bundle = tmp_path / "bundle-schema"
    make_act_bundle(schema_bundle)
    wrong = tmp_path / "observation.json"
    document = json.loads(OBSERVATION.read_text())
    document["joint_names"] = list(reversed(document["joint_names"]))
    wrong.write_text(json.dumps(document) + "\n", encoding="utf-8")
    with pytest.raises(PolicyConfigError, match="joint order"):
        load_policy_config(
            CONFIG,
            bundle_dir=schema_bundle,
            robot_profile_path=PROFILE,
            observation_schema_path=wrong,
            action_schema_path=ACTION,
            require_formal=False,
        )


def test_formal_contract_rejects_small_or_diagnostic_bundles(tmp_path):
    bundle = tmp_path / "bundle-small"
    make_act_bundle(bundle, synthetic=False)

    def resize(root: Path) -> None:
        document = json.loads((root / "metadata/dataset_manifest.json").read_text())
        document["splits"]["training"]["episode_count"] = 1
        document["splits"]["validation"]["episode_count"] = 1
        (root / "metadata/dataset_manifest.json").write_text(
            json.dumps(document) + "\n", encoding="utf-8"
        )
        manifest = json.loads((root / "manifest.json").read_text())
        manifest["dataset"]["train_episodes"] = 1
        manifest["dataset"]["validation_episodes"] = 1
        manifest["dataset"]["split"] = "1train/1validation"
        (root / "manifest.json").write_text(json.dumps(manifest) + "\n", encoding="utf-8")

    reseal_bundle(bundle, resize)
    with pytest.raises(PolicyConfigError, match="45/5"):
        resolve(bundle, require_formal=True)
    # A non-formal diagnosis may deploy the same real, small bundle.
    assert resolve(bundle).model_bundle_digest

    diagnostic = tmp_path / "bundle-diagnostic"
    make_act_bundle(diagnostic, synthetic=False)

    def mark_diagnostic(root: Path) -> None:
        document = json.loads((root / "metadata/dataset_manifest.json").read_text())
        document["diagnostic"] = True
        (root / "metadata/dataset_manifest.json").write_text(
            json.dumps(document) + "\n", encoding="utf-8"
        )
        manifest = json.loads((root / "manifest.json").read_text())
        manifest["dataset"]["diagnostic"] = True
        (root / "manifest.json").write_text(json.dumps(manifest) + "\n", encoding="utf-8")

    reseal_bundle(diagnostic, mark_diagnostic)
    with pytest.raises(PolicyConfigError, match="diagnostic"):
        resolve(diagnostic, require_formal=True)
    assert resolve(diagnostic).model_bundle_digest


def test_temporal_ensemble_setting_is_optional_and_strictly_bounded(tmp_path):
    template = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    assert template["model"]["temporal_ensemble_coeff"] == pytest.approx(0.01)

    from dual_arm_learning.policy_config import PolicyServiceConfig

    assert PolicyServiceConfig.model_fields["temporal_ensemble_coeff"].default is None
    service = yaml.safe_load(FAKE.read_text(encoding="utf-8"))["service"]
    for value in (True, float("nan"), float("inf"), -0.01, 1.01, "0.1"):
        document = dict(service)
        document["temporal_ensemble_coeff"] = value
        with pytest.raises(PolicyConfigError, match="temporal_ensemble_coeff"):
            parse_policy_config(document)


def test_fake_service_config_is_unchanged_by_bundle_derivation():
    config, settings = load_fake_config(FAKE)
    assert config.chunk_size == 60
    assert settings.fault == "none"
