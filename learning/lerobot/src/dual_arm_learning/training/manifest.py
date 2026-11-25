"""Dataset-manifest admission checks for ACT training."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from ..config import TrainingConfig
from ..dataset.common import canonical_hash, file_sha256
from ..dataset.sample_filter import SampleFilterMask, build_filter_mask
from .support import DatasetAudit, TrainingError


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise TrainingError(f"{path}: cannot read JSON: {error}") from error
    if not isinstance(value, dict):
        raise TrainingError(f"{path}: expected an object")
    return value


def _episode_indices(split: dict[str, Any]) -> list[int]:
    episodes = split.get("episodes")
    if not isinstance(episodes, list):
        raise TrainingError("manifest split episodes must be an array")
    result: list[int] = []
    for entry in episodes:
        if not isinstance(entry, dict):
            raise TrainingError("manifest episode must be an object")
        index = entry.get("global_success_index")
        if isinstance(index, bool) or not isinstance(index, int):
            raise TrainingError("manifest episode global_success_index must be an integer")
        result.append(index)
    return result


def _load_manifest(
    config: TrainingConfig, allow_synthetic: bool, require_full: bool | None
) -> tuple[Path, dict[str, Any], bool]:
    manifest_path = config.data_root / config.manifest_name
    manifest = _load_json(manifest_path)
    if manifest.get("status") != "complete":
        raise TrainingError(f"{manifest_path}: dataset manifest is not complete")
    synthetic = bool(manifest.get("synthetic"))
    if synthetic and (not allow_synthetic or require_full):
        raise TrainingError("refusing synthetic data for the requested training mode")
    return manifest_path, manifest, synthetic


def _manifest_splits(
    config: TrainingConfig, manifest: dict[str, Any]
) -> tuple[dict[str, Any], dict[str, Any], list[int], list[int]]:
    splits = manifest.get("splits")
    if not isinstance(splits, dict):
        raise TrainingError("manifest.splits must be an object")
    training = splits.get("training")
    validation = splits.get("validation")
    if not isinstance(training, dict) or not isinstance(validation, dict):
        raise TrainingError("manifest.splits must contain training and validation")
    expected = manifest.get("expected")
    if not isinstance(expected, dict):
        raise TrainingError("manifest.expected must be an object")
    expected_formal = expected.get("formal_episodes")
    if type(expected_formal) is not int or expected_formal < 2:
        raise TrainingError("manifest.expected.formal_episodes must be an integer >= 2")
    if expected.get("validation_percent") != config.dataset.validation_percent:
        raise TrainingError("manifest.expected.validation_percent does not match the dataset recipe")
    expected_training, expected_validation = config.dataset.split_counts(expected_formal)
    recorded_split = (expected.get("training_episodes"), expected.get("validation_episodes"))
    if (
        any(type(count) is not int for count in recorded_split)
        or recorded_split != (expected_training, expected_validation)
    ):
        raise TrainingError("manifest.expected episode counts do not match the dataset recipe")
    if training.get("episode_count") != expected_training:
        raise TrainingError(
            f"training split does not contain exactly {expected_training} episodes"
        )
    if validation.get("episode_count") != expected_validation:
        raise TrainingError(
            f"validation split does not contain exactly {expected_validation} episodes"
        )
    train_indices = _episode_indices(training)
    validation_indices = _episode_indices(validation)
    if len(train_indices) != expected_training or len(validation_indices) != expected_validation:
        raise TrainingError(
            f"split cardinality is not {expected_training}/{expected_validation}"
        )
    train_index_set = set(train_indices)
    validation_index_set = set(validation_indices)
    if train_index_set & validation_index_set:
        raise TrainingError("training and validation splits overlap")
    positions = expected.get("validation_positions")
    if (
        not isinstance(positions, list)
        or len(positions) != expected_validation
        or any(type(position) is not int for position in positions)
        or validation_index_set != set(positions)
    ):
        raise TrainingError(
            "validation split disagrees with manifest.expected.validation_positions"
        )
    if train_index_set | validation_index_set != set(range(expected_formal)):
        raise TrainingError("splits do not partition the manifest success range")
    return training, validation, train_indices, validation_indices


def _verify_profile(config: TrainingConfig, manifest: dict[str, Any]) -> None:
    profile_document = _load_json(config.dataset.profile_path)
    profile = manifest.get("profile")
    if not isinstance(profile, dict) or profile.get("semantic_sha256") != canonical_hash(
        profile_document
    ):
        raise TrainingError("manifest profile hash does not match the configured learning profile")
    if profile.get("profile_id") != profile_document.get("profile_id"):
        raise TrainingError("manifest profile id does not match")
    lerobot = manifest.get("lerobot", {})
    if lerobot.get("source_revision") != config.dataset.lerobot_source_revision:
        raise TrainingError("manifest LeRobot revision does not match the pinned source")
    declared_archive = config.dataset.lerobot_source_archive_sha256
    if declared_archive and lerobot.get("source_archive_sha256") != declared_archive:
        raise TrainingError("manifest LeRobot source archive does not match")


def _verify_processors(manifest: dict[str, Any]) -> None:
    processors = manifest.get("processors")
    if not isinstance(processors, dict) or processors.get("training_stats_only") is not True:
        raise TrainingError("manifest does not prove train-only processor statistics")


def _verify_split_metadata(
    config: TrainingConfig, root: Path, split_name: str, split: dict[str, Any]
) -> None:
    split_root = root / str(split.get("root", ""))
    if not split_root.is_dir():
        raise TrainingError(f"{split_name} dataset root is missing: {split_root}")
    info = _load_json(split_root / "meta/info.json")
    if info.get("total_episodes") != split.get("episode_count"):
        raise TrainingError(f"{split_name} meta total_episodes disagrees with manifest")
    features = info.get("features")
    if not isinstance(features, dict):
        raise TrainingError(f"{split_name} meta features are missing")
    for camera in config.dataset.camera_names:
        key = f"observation.images.{camera}"
        if key not in features or features[key].get("dtype") != "video":
            raise TrainingError(f"{split_name} is missing video feature {key}")
    if features.get("action", {}).get("shape") != [config.dataset.state_dim]:
        raise TrainingError(f"{split_name} action shape is not {config.dataset.state_dim}")
    if features.get("observation.state", {}).get("shape") != [config.dataset.state_dim]:
        raise TrainingError(
            f"{split_name} observation.state shape is not {config.dataset.state_dim}"
        )
    if not (split_root / "meta/stats.json").is_file():
        raise TrainingError(f"{split_name} meta/stats.json is missing")


def _verify_episode_facts(split_name: str, episode: dict[str, Any]) -> None:
    facts = episode.get("source_facts")
    if not isinstance(facts, dict):
        raise TrainingError(f"{split_name} episode has no source task/runtime facts")
    if (
        facts.get("task", {}).get("outcome") != "succeeded"
        or facts.get("task", {}).get("reason") != "TASK_SUCCEEDED"
    ):
        raise TrainingError(f"{split_name} source task did not succeed")
    if (
        facts.get("runtime", {}).get("outcome") not in {"stopped", "completed"}
        or facts.get("runtime", {}).get("reason") != "TASK_SUCCEEDED"
    ):
        raise TrainingError(f"{split_name} source runtime did not stop successfully")
    if facts.get("recording", {}).get("complete") is not True:
        raise TrainingError(f"{split_name} source recording is incomplete")
    if facts.get("quality_qualified") is not True:
        raise TrainingError(f"{split_name} source quality is not qualified")


def _verify_split_episodes(split_name: str, split: dict[str, Any]) -> None:
    for episode in split.get("episodes", []):
        if not isinstance(episode, dict):
            raise TrainingError(f"{split_name} episode entry is not an object")
        _verify_episode_facts(split_name, episode)


def _verify_full_lengths(
    config: TrainingConfig,
    require_full: bool | None,
    training: dict[str, Any],
    validation: dict[str, Any],
) -> None:
    if not (require_full or (require_full is None and config.mode == "full")):
        return
    for split_name, split in (("training", training), ("validation", validation)):
        for episode in split.get("episodes", []):
            if int(episode.get("frame_count", 0)) < config.dataset.chunk_size:
                raise TrainingError(
                    f"full training refuses {split_name} episode shorter than one action chunk"
                )


def _audit(
    *,
    root: Path,
    training: dict[str, Any],
    validation: dict[str, Any],
    train_indices: list[int],
    validation_indices: list[int],
    manifest_path: Path,
    synthetic: bool,
) -> DatasetAudit:
    return DatasetAudit(
        train_root=root / str(training["root"]),
        validation_root=root / str(validation["root"]),
        train_episodes=tuple(range(len(train_indices))),
        validation_episodes=tuple(range(len(validation_indices))),
        training_global_indices=tuple(train_indices),
        validation_global_indices=tuple(validation_indices),
        train_frames=int(training.get("frame_count", 0)),
        validation_frames=int(validation.get("frame_count", 0)),
        manifest_sha256=file_sha256(manifest_path),
        synthetic=synthetic,
    )


def verify_dataset_manifest(
    config: TrainingConfig,
    *,
    allow_synthetic: bool = False,
    require_full: bool | None = None,
) -> DatasetAudit:
    """Verify the recorded dataset split and source facts."""
    manifest_path, manifest, synthetic = _load_manifest(config, allow_synthetic, require_full)
    training, validation, train_indices, validation_indices = _manifest_splits(config, manifest)
    _verify_profile(config, manifest)
    _verify_processors(manifest)
    root = config.data_root
    for split_name, split in (("training", training), ("validation", validation)):
        _verify_split_metadata(config, root, split_name, split)
        _verify_split_episodes(split_name, split)
    _verify_full_lengths(config, require_full, training, validation)
    return _audit(
        root=root,
        training=training,
        validation=validation,
        train_indices=train_indices,
        validation_indices=validation_indices,
        manifest_path=manifest_path,
        synthetic=synthetic,
    )


def sample_filter(config: TrainingConfig, audit: DatasetAudit) -> SampleFilterMask | None:
    """Return the full-mode anchor mask, computing it in memory from the rule YAML."""
    if config.sample_filter is None or config.mode != "full":
        return None
    if config.sample_filter.suffix.lower() not in {".yaml", ".yml"}:
        raise TrainingError("data.sample_filter must be a rule YAML")
    mask = build_filter_mask(rule_path=config.sample_filter, dataset_root=config.data_root)
    if mask.manifest_sha256 != audit.manifest_sha256:
        raise TrainingError("sample filter was generated from a different dataset manifest")
    if mask.source_frames != audit.train_frames:
        raise TrainingError("sample filter source frame count differs from the training split")
    return mask


__all__ = ["sample_filter", "verify_dataset_manifest"]
