from pathlib import Path

import pytest
import yaml
from dual_arm_learning.config import ConfigError, load_dataset_config, load_training_config

ROOT = Path(__file__).resolve().parents[2]


def test_real_learning_configs_load():
    dataset = load_dataset_config(ROOT / "learning/lerobot/configs/act_dataset.yaml", ROOT)
    training = load_training_config(ROOT / "learning/lerobot/configs/act_training.yaml", ROOT)
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
    document = yaml.safe_load((ROOT / "learning/lerobot/configs/act_training.yaml").read_text())
    document["budget"]["limit_seconds"] = 20000.0
    bad = tmp_path / "bad.yaml"
    bad.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match="budget.limit_seconds"):
        load_training_config(bad, ROOT)

    document = yaml.safe_load((ROOT / "learning/lerobot/configs/act_training.yaml").read_text())
    document["modes"]["full"]["unexpected"] = 1
    bad.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match="unexpected"):
        load_training_config(bad, ROOT)

    document = yaml.safe_load((ROOT / "learning/lerobot/configs/act_training.yaml").read_text())
    document["modes"]["full"]["episodes"] = [0]
    bad.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match="all 45"):
        load_training_config(bad, ROOT)


def test_encoder_and_split_contract_are_strict(tmp_path):
    document = yaml.safe_load((ROOT / "learning/lerobot/configs/act_dataset.yaml").read_text())
    document["lerobot"]["rgb_encoder"]["crf"] = 20
    bad = tmp_path / "bad-dataset.yaml"
    bad.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match="crf"):
        load_dataset_config(bad, ROOT)

    document = yaml.safe_load((ROOT / "learning/lerobot/configs/act_dataset.yaml").read_text())
    document["expected"]["validation_positions"] = [9, 19, 29, 39, 48]
    bad.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match="validation_positions"):
        load_dataset_config(bad, ROOT)
