from pathlib import Path

import pytest
import yaml
from dual_arm_learning.config import ConfigError, load_dataset_config, load_training_config
from robot_contracts.project_paths import project_path

ROOT = Path(__file__).resolve().parents[2]
SCENARIO = ROOT / "shared/scenarios/marvin_sim/stack"


def test_real_learning_configs_load():
    dataset = load_dataset_config(SCENARIO / "configs/learning/act_dataset.yaml", ROOT)
    training = load_training_config(SCENARIO / "configs/learning/act_training.yaml", ROOT)
    assert dataset.camera_names == ("global", "left_wrist", "right_wrist")
    assert dataset.chunk_size == 60
    assert dataset.validation_percent == 10
    assert (dataset.expected_training_episodes, dataset.expected_validation_episodes) == (45, 5)
    assert training.mode == "smoke"
    assert training.modes["smoke"].steps == 128
    assert training.modes["overfit"].steps == 500
    assert training.modes["full"].steps == 5000
    assert training.model.pretrained_backbone_weights is None
    assert training.lr == pytest.approx(1e-4)
    assert training.lr_backbone == pytest.approx(1e-4)
    assert training.checkpoint.root == project_path("var/marvin_sim/stack/models", ROOT)
    assert training.dataset.camera_names == dataset.camera_names


def test_lr_backbone_contract_and_optimizer_param_groups(tmp_path):
    torch = pytest.importorskip("torch")
    from dual_arm_learning.training_support import build_optimizer_param_groups

    document = yaml.safe_load((SCENARIO / "configs/learning/act_training.yaml").read_text())
    document["training"]["lr_backbone"] = 0.001
    bad = tmp_path / "bad-lr.yaml"
    bad.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match="lr_backbone"):
        load_training_config(bad, ROOT)

    training = load_training_config(SCENARIO / "configs/learning/act_training.yaml", ROOT)

    class _FakePolicy(torch.nn.Module):
        def __init__(self) -> None:
            super().__init__()
            self.model = torch.nn.Module()
            self.model.backbone = torch.nn.Linear(2, 2)
            self.model.decoder = torch.nn.Linear(2, 2)

    groups = build_optimizer_param_groups(_FakePolicy(), training)
    assert len(groups) == 2
    assert groups[0]["lr"] == pytest.approx(1e-4)
    assert groups[1]["lr"] == pytest.approx(1e-4)
    assert len(groups[0]["params"]) == 2
    assert len(groups[1]["params"]) == 2


def test_unknown_field_and_invalid_mode_are_rejected(tmp_path):
    document = yaml.safe_load((SCENARIO / "configs/learning/act_training.yaml").read_text())
    document["training"]["batch_size"] = 0
    bad = tmp_path / "bad.yaml"
    bad.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match="batch_size"):
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


def test_encoder_contract_is_strict(tmp_path):
    document = yaml.safe_load((SCENARIO / "configs/learning/act_dataset.yaml").read_text())
    document["lerobot"]["rgb_encoder"]["crf"] = 20
    bad = tmp_path / "bad-dataset.yaml"
    bad.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match="crf"):
        load_dataset_config(bad, ROOT)


@pytest.mark.parametrize("key", ["video_files_size_mb", "data_files_size_mb"])
def test_shard_sizes_are_not_recipe_parameters(tmp_path, key):
    document = yaml.safe_load((SCENARIO / "configs/learning/act_dataset.yaml").read_text())
    document["lerobot"][key] = 256
    path = tmp_path / "dataset.yaml"
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match=rf"lerobot\.{key}"):
        load_dataset_config(path, ROOT)


@pytest.mark.parametrize(
    "split_key", ["training_episodes", "validation_episodes", "validation_positions"]
)
def test_split_rejects_counts_and_positions_in_recipe(tmp_path, split_key):
    document = yaml.safe_load((SCENARIO / "configs/learning/act_dataset.yaml").read_text())
    document["expected"][split_key] = 1
    bad = tmp_path / "bad-dataset.yaml"
    bad.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match=rf"expected\.{split_key}"):
        load_dataset_config(bad, ROOT)


@pytest.mark.parametrize("percent", [0, 100, -1, 100.1, "10", True, float("nan")])
def test_validation_percent_requires_number_between_zero_and_hundred(tmp_path, percent):
    document = yaml.safe_load((SCENARIO / "configs/learning/act_dataset.yaml").read_text())
    document["validation_percent"] = percent
    path = tmp_path / "dataset.yaml"
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match="validation_percent"):
        load_dataset_config(path, ROOT)


@pytest.mark.parametrize(
    ("count", "percent", "expected_validation"),
    [
        (50, 13.0, 7),
        (50, 12.75, 6),
        (50, 0.1, 1),
        (50, 99.9, 49),
        (2, 10, 1),
        (4, 62.5, 3),
    ],
)
def test_split_rounds_half_up_and_keeps_both_groups_non_empty(
    tmp_path, count, percent, expected_validation
):
    document = yaml.safe_load((SCENARIO / "configs/learning/act_dataset.yaml").read_text())
    document["expected"]["formal_episodes"] = count
    document["validation_percent"] = percent
    path = tmp_path / "dataset.yaml"
    path.write_text(yaml.safe_dump(document))
    config = load_dataset_config(path, ROOT)
    assert config.validation_percent == percent
    assert config.expected_validation_episodes == expected_validation
    assert config.expected_training_episodes == count - expected_validation


def test_split_requires_at_least_two_episodes(tmp_path):
    document = yaml.safe_load((SCENARIO / "configs/learning/act_dataset.yaml").read_text())
    document["expected"]["formal_episodes"] = 1
    path = tmp_path / "dataset.yaml"
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match="formal_episodes"):
        load_dataset_config(path, ROOT)


def test_dataset_recipe_rejects_diagnostic_field(tmp_path):
    document = yaml.safe_load((SCENARIO / "configs/learning/act_dataset.yaml").read_text())
    document["diagnostic"] = True
    path = tmp_path / "dataset.yaml"
    path.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match="diagnostic"):
        load_dataset_config(path, ROOT)


def test_training_recipe_rejects_removed_image_shift_field(tmp_path):
    document = yaml.safe_load((SCENARIO / "configs/learning/act_training.yaml").read_text())
    document["training"]["image_shift_px"] = 4
    bad = tmp_path / "bad-shift.yaml"
    bad.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match=r"training\.image_shift_px: unknown key"):
        load_training_config(bad, ROOT)
