import copy
import hashlib
import json
from dataclasses import replace
from pathlib import Path

import pytest
from dual_arm_learning.config import load_training_config
from dual_arm_learning.dataset import canonical_hash
from dual_arm_learning.training import TrainingError, masked_l1, verify_dataset_manifest

ROOT = Path(__file__).resolve().parents[2]


def _write(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def _manifest(config, root: Path) -> dict:
    profile = json.loads(config.dataset.profile_path.read_text())
    semantic = hashlib.sha256(
        json.dumps(profile, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()
    features = {
        "action": {"dtype": "float32", "shape": [16]},
        "observation.state": {"dtype": "float32", "shape": [16]},
    }
    for camera in config.dataset.camera_names:
        features[f"observation.images.{camera}"] = {"dtype": "video", "shape": [240, 320, 3]}

    def episode(index: int) -> dict:
        facts = {
            "episode_id": f"ep-{index}",
            "task": {"outcome": "succeeded", "reason": "TASK_SUCCEEDED"},
            "runtime": {"outcome": "stopped", "reason": "TASK_SUCCEEDED"},
            "recording": {"complete": True, "failure_reason": ""},
            "quality_qualified": True,
        }
        facts["facts_sha256"] = canonical_hash(facts)
        return {
            "global_success_index": index,
            "attempt_id": f"attempt-{index}",
            "seed": 10000 + index,
            "frame_count": 60,
            "source_facts": facts,
        }

    training_episodes = [
        episode(index) for index in range(50) if index not in config.dataset.validation_positions
    ]
    validation_episodes = [episode(index) for index in config.dataset.validation_positions]
    for split, episodes in (("train", training_episodes), ("validation", validation_episodes)):
        split_root = root / split
        _write(
            split_root / "meta/info.json",
            {
                "total_episodes": len(episodes),
                "total_frames": len(episodes) * 60,
                "features": features,
            },
        )
        _write(split_root / "meta/stats.json", {"action": {"mean": [0.0] * 16, "count": [1]}})
    fact_records = [
        {
            "attempt_id": item["attempt_id"],
            "seed": item["seed"],
            "facts": {
                key: value for key, value in item["source_facts"].items() if key != "facts_sha256"
            },
        }
        for item in training_episodes + validation_episodes
    ]
    return {
        "status": "complete",
        "source_facts_sha256": canonical_hash(fact_records),
        "synthetic": False,
        "profile": {"semantic_sha256": semantic, "profile_id": profile.get("profile_id")},
        "lerobot": {
            "source_revision": config.dataset.lerobot_source_revision,
            "source_archive_sha256": config.dataset.lerobot_source_archive_sha256,
        },
        "processors": {"training_stats_only": True},
        "splits": {
            "training": {
                "root": "train",
                "episode_count": 45,
                "frame_count": 2700,
                "episodes": training_episodes,
                "files": {"meta/info.json": {}, "meta/stats.json": {}},
            },
            "validation": {
                "root": "validation",
                "episode_count": 5,
                "frame_count": 300,
                "episodes": validation_episodes,
                "files": {"meta/info.json": {}, "meta/stats.json": {}},
            },
        },
    }


def test_manifest_partitions_45_5_and_rejects_overlap(tmp_path):
    config = load_training_config(ROOT / "learning/lerobot/configs/act_training.yaml", ROOT)
    config = replace(config, data_root=tmp_path, verify_hashes=False, mode="full")
    manifest = _manifest(config, tmp_path)
    _write(tmp_path / "manifest.json", manifest)
    audit = verify_dataset_manifest(config, allow_synthetic=False, require_full=True)
    assert len(audit.train_episodes) == 45
    assert audit.validation_global_indices == (9, 19, 29, 39, 49)

    bad = copy.deepcopy(manifest)
    bad["splits"]["validation"]["episodes"][0]["global_success_index"] = 8
    _write(tmp_path / "manifest.json", bad)
    with pytest.raises(TrainingError, match="validation split"):
        verify_dataset_manifest(config, allow_synthetic=False, require_full=True)

    bad = copy.deepcopy(manifest)
    bad["splits"]["validation"]["episodes"][0]["global_success_index"] = 0
    _write(tmp_path / "manifest.json", bad)
    with pytest.raises(TrainingError, match="overlap"):
        verify_dataset_manifest(config, allow_synthetic=False, require_full=True)


def test_full_refuses_short_episode_and_synthetic_data(tmp_path):
    config = load_training_config(ROOT / "learning/lerobot/configs/act_training.yaml", ROOT)
    config = replace(config, data_root=tmp_path, verify_hashes=False, mode="full")
    manifest = _manifest(config, tmp_path)
    manifest["splits"]["training"]["episodes"][0]["frame_count"] = 1
    _write(tmp_path / "manifest.json", manifest)
    with pytest.raises(TrainingError, match="shorter than one action chunk"):
        verify_dataset_manifest(config, allow_synthetic=False, require_full=True)

    manifest = _manifest(config, tmp_path)
    manifest["synthetic"] = True
    _write(tmp_path / "manifest.json", manifest)
    with pytest.raises(TrainingError, match="synthetic"):
        verify_dataset_manifest(config, allow_synthetic=True, require_full=True)


def test_masked_l1_ignores_padded_future_actions():
    torch = pytest.importorskip("torch")
    prediction = torch.zeros((2, 3, 2), dtype=torch.float32)
    target = torch.tensor(
        [
            [[1.0, 2.0], [3.0, 4.0], [100.0, 100.0]],
            [[2.0, 2.0], [100.0, 100.0], [100.0, 100.0]],
        ]
    )
    mask = torch.tensor([[False, False, True], [False, True, True]])
    loss, count = masked_l1(prediction, target, mask)
    assert count == 6
    assert torch.isclose(loss, torch.tensor(14.0 / 6.0))
