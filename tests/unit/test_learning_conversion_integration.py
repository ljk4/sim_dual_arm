"""Heavy synthetic conversion check; run with DSH035_HEAVY=1.

It deliberately creates all 50 accepted exports because the production
contract is 45/5.  It is skipped by default so the small NumPy-only suite stays
fast; the candidate report records the actual heavy run.
"""

import json
import os
from dataclasses import replace
from pathlib import Path

import pytest
from dual_arm_learning.config import load_dataset_config, load_training_config
from dual_arm_learning.dataset import ExportError, convert
from dual_arm_learning.training import verify_dataset_manifest
from learning_fixtures import make_select

ROOT = Path(__file__).resolve().parents[2]
pytestmark = pytest.mark.skipif(
    os.environ.get("DSH035_HEAVY") != "1",
    reason="set DSH035_HEAVY=1 for the 50-episode synthetic conversion",
)


def test_two_independent_datasets_train_only_processors_and_padding(tmp_path):
    pytest.importorskip("lerobot")
    import torch

    config = load_dataset_config(ROOT / "learning/lerobot/configs/act_dataset.yaml", ROOT)
    config = replace(config, output_root=tmp_path / "data", cache_root=tmp_path / "cache")
    select = make_select(tmp_path, config, first_frames=61)
    manifest = convert(select_path=select, config=config, synthetic=True)
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    assert manifest["synthetic"] is True
    assert manifest["splits"]["training"]["episode_count"] == 45
    assert manifest["splits"]["validation"]["episode_count"] == 5
    assert manifest["splits"]["training"]["frame_count"] == 149
    assert manifest["splits"]["validation"]["frame_count"] == 10
    assert manifest["processors"]["training_stats_only"] is True
    with pytest.raises(ExportError, match="already exists"):
        convert(select_path=select, config=config, synthetic=True)

    train = LeRobotDataset(
        repo_id="local/conversion-train",
        root=tmp_path / "data/train",
        delta_timestamps={"action": [index / 20 for index in range(60)]},
        video_backend="torchcodec",
    )
    validation = LeRobotDataset(
        repo_id="local/conversion-validation",
        root=tmp_path / "data/validation",
        delta_timestamps={"action": [index / 20 for index in range(60)]},
        video_backend="torchcodec",
    )
    first = train[0]
    assert tuple(first["action"].shape) == (60, 16)
    assert not bool(first["action_is_pad"].any())
    last = train[60]
    assert int((~last["action_is_pad"]).sum()) == 1
    assert torch.allclose(last["action"][-1], last["action"][0])
    for camera in config.camera_names:
        image = first[f"observation.images.{camera}"]
        assert tuple(image.shape) == (3, 240, 320)
    red = float(first["observation.images.global"][0].mean())
    green = float(first["observation.images.global"][1].mean())
    blue = float(first["observation.images.global"][2].mean())
    assert red < green < blue
    assert abs(red - 10 / 255) < 0.03
    assert abs(green - 20 / 255) < 0.03
    assert abs(blue - 30 / 255) < 0.03
    stats = manifest["processors"]
    assert stats is not None
    summary = json.loads((tmp_path / "data/processors/train_stats_summary.json").read_text())
    assert summary["source"] == "training_split_only"
    assert abs(float(summary["features"]["action"]["mean"][0]) - 0.01) < 1e-6
    assert abs(float(train.meta.stats["action"]["mean"][0]) - 0.01) < 1e-6
    assert abs(float(validation.meta.stats["action"]["mean"][0]) - 1.01) < 1e-6

    training = load_training_config(ROOT / "learning/lerobot/configs/act_training.yaml", ROOT)
    training = replace(training, data_root=tmp_path / "data", mode="smoke")
    audit = verify_dataset_manifest(training, allow_synthetic=True, require_full=False)
    assert audit.synthetic is True
    assert audit.validation_global_indices == (9, 19, 29, 39, 49)
