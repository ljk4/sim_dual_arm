from pathlib import Path

import pytest
import yaml
from dual_arm_learning.config import ConfigError, load_dataset_config, load_training_config

ROOT = Path(__file__).resolve().parents[2]
SCENARIO = ROOT / "shared/scenarios/panda_stack"


def test_real_learning_configs_load():
    dataset = load_dataset_config(SCENARIO / "configs/learning/act_dataset.yaml", ROOT)
    training = load_training_config(SCENARIO / "configs/learning/act_training.yaml", ROOT)
    assert dataset.camera_names == ("global", "left_wrist", "right_wrist")
    assert dataset.chunk_size == 60
    assert dataset.validation_positions == (9, 19, 29, 39, 49)
    assert training.mode == "smoke"
    assert training.modes["smoke"].steps == 128
    assert training.modes["overfit"].steps == 500
    assert training.modes["full"].steps == 5000
    assert training.model.pretrained_backbone_weights is None
    assert training.budget.limit_seconds == 14400.0
    assert training.dataset.camera_names == dataset.camera_names


def test_budget_cap_and_unknown_field_are_rejected(tmp_path):
    document = yaml.safe_load((SCENARIO / "configs/learning/act_training.yaml").read_text())
    document["budget"]["limit_seconds"] = 20000.0
    bad = tmp_path / "bad.yaml"
    bad.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match="budget.limit_seconds"):
        load_training_config(bad, ROOT)

    document = yaml.safe_load((SCENARIO / "configs/learning/act_training.yaml").read_text())
    document["modes"]["full"]["unexpected"] = 1
    bad.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match="unexpected"):
        load_training_config(bad, ROOT)

    document = yaml.safe_load((SCENARIO / "configs/learning/act_training.yaml").read_text())
    document["modes"]["full"]["episodes"] = [0]
    bad.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match="every configured training episode"):
        load_training_config(bad, ROOT)


def test_encoder_and_split_contract_are_strict(tmp_path):
    document = yaml.safe_load((SCENARIO / "configs/learning/act_dataset.yaml").read_text())
    document["lerobot"]["rgb_encoder"]["crf"] = 20
    bad = tmp_path / "bad-dataset.yaml"
    bad.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match="crf"):
        load_dataset_config(bad, ROOT)

    document = yaml.safe_load((SCENARIO / "configs/learning/act_dataset.yaml").read_text())
    document["expected"]["validation_positions"] = [9, 19, 29, 39, 48]
    bad.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match="validation_positions"):
        load_dataset_config(bad, ROOT)

    document = yaml.safe_load((SCENARIO / "configs/learning/act_dataset.yaml").read_text())
    document["expected"] = {
        "formal_episodes": 2,
        "training_episodes": 1,
        "validation_episodes": 1,
        "validation_positions": [1],
    }
    bad.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match="exactly 50/45/5"):
        load_dataset_config(bad, ROOT)


def test_diagnostic_dataset_config_is_explicit_and_consistent(tmp_path):
    document = yaml.safe_load((SCENARIO / "configs/learning/act_dataset.yaml").read_text())
    document["diagnostic"] = True
    document["expected"] = {
        "formal_episodes": 2,
        "training_episodes": 1,
        "validation_episodes": 1,
        "validation_positions": [1],
    }
    path = tmp_path / "diagnostic.yaml"
    path.write_text(yaml.safe_dump(document))
    config = load_dataset_config(path, ROOT)
    assert config.diagnostic is True
    assert config.expected_formal_episodes == 2
    assert config.validation_positions == (1,)

    document["expected"]["validation_episodes"] = 2
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match="formal_episodes"):
        load_dataset_config(path, ROOT)

    document["expected"]["validation_episodes"] = 1
    document["expected"]["validation_positions"] = [2]
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match="episode range"):
        load_dataset_config(path, ROOT)
