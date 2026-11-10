import json
from pathlib import Path

import pytest
import yaml
from act_fixtures import (
    ACTION,
    OBSERVATION,
    PROFILE,
    make_act_bundle,
    reseal_bundle,
)
from dual_arm_learning.act_contract import (
    ActContractError,
    load_act_settings,
    resolve_act_contract,
)

ROOT = Path(__file__).resolve().parents[2]


def resolve(bundle: Path, *, require_formal: bool = False):
    return resolve_act_contract(
        bundle=bundle,
        profile_path=PROFILE,
        observation_schema_path=OBSERVATION,
        action_schema_path=ACTION,
        require_formal=require_formal,
    )


def test_contract_verifies_stats_and_rejects_tampered_synthetic(tmp_path):
    bundle = tmp_path / "bundle"
    make_act_bundle(bundle)
    contract = resolve(bundle)
    assert contract.train_episodes == 45
    assert contract.validation_episodes == 5
    assert contract.checkpoint_mode == "full"
    assert contract.synthetic is True

    def mutate_stats(root: Path) -> None:
        summary = {
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
            json.dumps(summary) + "\n", encoding="utf-8"
        )
        (root / "metadata/train_stats_summary.json").write_text(
            json.dumps(summary) + "\n", encoding="utf-8"
        )

    reseal_bundle(bundle, mutate_stats)
    with pytest.raises(ActContractError, match="training-split-only"):
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
    with pytest.raises(ActContractError, match="45/5"):
        resolve(bundle)


def test_contract_rejects_temporal_ensemble_and_wrong_schema(tmp_path):
    bundle = tmp_path / "bundle"
    make_act_bundle(bundle)

    def mutate_config(root: Path) -> None:
        document = json.loads((root / "config.json").read_text())
        document["temporal_ensemble_coeff"] = 0.01
        (root / "config.json").write_text(json.dumps(document) + "\n", encoding="utf-8")

    reseal_bundle(bundle, mutate_config)
    with pytest.raises(ActContractError, match="temporal"):
        resolve(bundle)

    wrong = tmp_path / "observation.json"
    document = json.loads(OBSERVATION.read_text())
    document["joint_names"] = list(reversed(document["joint_names"]))
    wrong.write_text(json.dumps(document) + "\n", encoding="utf-8")
    with pytest.raises(ActContractError, match="joint_names"):
        resolve_act_contract(
            bundle=bundle,
            profile_path=PROFILE,
            observation_schema_path=wrong,
            action_schema_path=ACTION,
            require_formal=False,
        )


def test_temporal_ensemble_setting_is_optional_and_strictly_bounded(tmp_path):
    temporal = load_act_settings(ROOT / "learning/lerobot/configs/act_policy_temporal.yaml")
    assert temporal.temporal_ensemble_coeff == pytest.approx(0.01)

    document = yaml.safe_load(
        (ROOT / "learning/lerobot/configs/act_policy_temporal.yaml").read_text(encoding="utf-8")
    )
    for index, value in enumerate((True, float("nan"), float("inf"), -0.01, 1.01, "0.1")):
        document["model"]["temporal_ensemble_coeff"] = value
        path = tmp_path / f"temporal-{index}.yaml"
        path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
        with pytest.raises(ActContractError, match="temporal_ensemble_coeff"):
            load_act_settings(path)
