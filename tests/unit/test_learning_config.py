from pathlib import Path

import pytest
import yaml
from dual_arm_learning.config import ConfigError, load_dataset_config, load_training_config

ROOT = Path(__file__).resolve().parents[2]
SCENARIO = ROOT / "shared/scenarios/panda_sim/stack"


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
    assert training.lr == pytest.approx(1e-4)
    assert training.lr_backbone == pytest.approx(1e-4)
    assert training.checkpoint.root == ROOT / "var/panda_sim/stack/models"
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


def test_image_shift_px_config_and_augmentation(tmp_path):
    torch = pytest.importorskip("torch")
    from dual_arm_learning.training_support import augment_image_shift

    panda_cfg = load_training_config(SCENARIO / "configs/learning/act_training.yaml", ROOT)
    assert panda_cfg.image_shift_px == 0
    marvin_cfg = load_training_config(
        ROOT / "shared/scenarios/marvin_sim/handover/configs/learning/act_training.yaml", ROOT
    )
    assert marvin_cfg.image_shift_px == 4

    document = yaml.safe_load((SCENARIO / "configs/learning/act_training.yaml").read_text())
    document["training"]["image_shift_px"] = 64
    bad = tmp_path / "bad-shift.yaml"
    bad.write_text(yaml.safe_dump(document))
    with pytest.raises(ConfigError, match="image_shift_px"):
        load_training_config(bad, ROOT)

    img = torch.arange(2 * 3 * 12 * 16, dtype=torch.float32).reshape(2, 3, 12, 16)
    state = torch.ones((2, 16), dtype=torch.float32)
    batch = {"observation.images.global": img, "observation.state": state}
    assert augment_image_shift(batch, 0) is batch

    torch.manual_seed(123)
    out1 = augment_image_shift(batch, 4)
    torch.manual_seed(123)
    out2 = augment_image_shift(batch, 4)
    assert torch.equal(out1["observation.images.global"], out2["observation.images.global"])
    assert out1["observation.images.global"].shape == img.shape
    assert out1["observation.images.global"].dtype == img.dtype
    assert not torch.equal(out1["observation.images.global"], img)
    assert torch.equal(out1["observation.state"], state)
    assert float(out1["observation.images.global"].min()) >= float(img.min())
    assert float(out1["observation.images.global"].max()) <= float(img.max())
