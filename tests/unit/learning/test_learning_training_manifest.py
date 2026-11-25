import copy
import hashlib
import json
from pathlib import Path

import pytest
import yaml
from dual_arm_learning.config import TrainingConfig, load_training_config
from dual_arm_learning.training import TrainingError, masked_l1, verify_dataset_manifest
from model_helpers import replace

ROOT = Path(__file__).resolve().parents[3]
SCENARIO = ROOT / "shared/scenarios/marvin/mujoco/stack"


def _write(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8")


def _manifest(config: TrainingConfig, root: Path, *, episodes: int = 50) -> dict:
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
        return {
            "global_success_index": index,
            "attempt_id": f"attempt-{index}",
            "seed": 10000 + index,
            "frame_count": 60,
            "source_facts": facts,
        }

    # Use a valid stored split independent of the converter's random draw.
    _, expected_val = config.dataset.split_counts(episodes)
    validation_positions = list(range(1, expected_val + 1))
    training_episodes = [
        episode(index)
        for index in range(episodes)
        if index not in validation_positions
    ]
    validation_episodes = [episode(index) for index in validation_positions]
    for split, split_episodes in (("train", training_episodes), ("validation", validation_episodes)):
        split_root = root / split
        _write(
            split_root / "meta/info.json",
            {
                "total_episodes": len(split_episodes),
                "total_frames": len(split_episodes) * 60,
                "features": features,
            },
        )
        _write(split_root / "meta/stats.json", {"action": {"mean": [0.0] * 16, "count": [1]}})
    return {
        "status": "complete",
        "synthetic": False,
        "expected": {
            "formal_episodes": episodes,
            "training_episodes": len(training_episodes),
            "validation_episodes": len(validation_episodes),
            "validation_positions": validation_positions,
            "validation_percent": config.dataset.validation_percent,
        },
        "profile": {"semantic_sha256": semantic, "profile_id": profile.get("profile_id")},
        "lerobot": {
            "source_revision": config.dataset.lerobot_source_revision,
            "source_archive_sha256": config.dataset.lerobot_source_archive_sha256,
        },
        "processors": {"training_stats_only": True},
        "splits": {
            "training": {
                "root": "train",
                "episode_count": len(training_episodes),
                "frame_count": len(training_episodes) * 60,
                "episodes": training_episodes,
            },
            "validation": {
                "root": "validation",
                "episode_count": len(validation_episodes),
                "frame_count": len(validation_episodes) * 60,
                "episodes": validation_episodes,
            },
        },
    }


def test_manifest_reads_saved_45_5_split_and_validates_partition(tmp_path):
    config = load_training_config(SCENARIO / "configs/learning/act_training.yaml", ROOT)
    config = replace(config, data_root=tmp_path, mode="full")
    manifest = _manifest(config, tmp_path)
    manifest["expected"]["validation_positions"].reverse()
    _write(tmp_path / "manifest.json", manifest)
    audit = verify_dataset_manifest(config, allow_synthetic=False, require_full=True)
    assert len(audit.train_episodes) == 45
    assert audit.validation_global_indices == (1, 2, 3, 4, 5)

    bad = copy.deepcopy(manifest)
    bad["expected"]["validation_positions"][0] = 8
    _write(tmp_path / "manifest.json", bad)
    with pytest.raises(TrainingError, match="validation split"):
        verify_dataset_manifest(config, allow_synthetic=False, require_full=True)

    bad = copy.deepcopy(manifest)
    bad["splits"]["validation"]["episodes"][0]["global_success_index"] = 0
    _write(tmp_path / "manifest.json", bad)
    with pytest.raises(TrainingError, match="overlap"):
        verify_dataset_manifest(config, allow_synthetic=False, require_full=True)

    bad = copy.deepcopy(manifest)
    bad["splits"]["training"]["episodes"][0]["global_success_index"] = 50
    _write(tmp_path / "manifest.json", bad)
    with pytest.raises(TrainingError, match="partition"):
        verify_dataset_manifest(config, allow_synthetic=False, require_full=True)

    bad = copy.deepcopy(manifest)
    bad["expected"]["training_episodes"] -= 1
    bad["expected"]["validation_episodes"] += 1
    _write(tmp_path / "manifest.json", bad)
    with pytest.raises(TrainingError, match="episode counts do not match"):
        verify_dataset_manifest(config, allow_synthetic=False, require_full=True)


def test_full_refuses_short_episode_and_synthetic_data(tmp_path):
    config = load_training_config(SCENARIO / "configs/learning/act_training.yaml", ROOT)
    config = replace(config, data_root=tmp_path, mode="full")
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


def test_variable_manifest_counts_validate_against_dataset_percent(tmp_path):
    dataset_document = yaml.safe_load(
        (SCENARIO / "configs/learning/act_dataset.yaml").read_text()
    )
    dataset_document["validation_percent"] = 50
    dataset_path = tmp_path / "half_split_dataset.yaml"
    dataset_path.write_text(yaml.safe_dump(dataset_document))

    training_document = yaml.safe_load(
        (SCENARIO / "configs/learning/act_training.yaml").read_text()
    )
    training_document["data"]["config"] = str(dataset_path)
    training_document["mode"] = "smoke"
    training_path = tmp_path / "half_split_training.yaml"
    training_path.write_text(yaml.safe_dump(training_document))

    config = load_training_config(training_path, ROOT)
    config = replace(config, data_root=tmp_path)
    _write(tmp_path / "manifest.json", _manifest(config, tmp_path, episodes=2))
    audit = verify_dataset_manifest(config, allow_synthetic=False, require_full=False)
    assert audit.training_global_indices == (0,)
    assert audit.validation_global_indices == (1,)
    assert audit.train_frames == 60
    assert audit.validation_frames == 60

    standard = load_training_config(SCENARIO / "configs/learning/act_training.yaml", ROOT)
    standard = replace(standard, data_root=tmp_path, mode="smoke")
    with pytest.raises(TrainingError, match="validation_percent"):
        verify_dataset_manifest(standard, allow_synthetic=False, require_full=False)

    manifest_60 = tmp_path / "sixty"
    manifest_60.mkdir()
    standard_60 = replace(standard, data_root=manifest_60)
    _write(manifest_60 / "manifest.json", _manifest(standard_60, manifest_60, episodes=60))
    audit_60 = verify_dataset_manifest(standard_60, allow_synthetic=False, require_full=False)
    assert len(audit_60.train_episodes) == 54
    assert len(audit_60.validation_global_indices) == 6

    manifest = _manifest(config, tmp_path, episodes=2)
    manifest["splits"]["validation"]["episodes"] = []
    manifest["splits"]["validation"]["episode_count"] = 0
    _write(tmp_path / "manifest.json", manifest)
    with pytest.raises(TrainingError, match="validation split"):
        verify_dataset_manifest(config, allow_synthetic=False, require_full=False)


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
