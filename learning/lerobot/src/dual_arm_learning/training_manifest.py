"""Dataset-manifest admission checks for ACT training."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .config import TrainingConfig
from .dataset import canonical_hash, file_sha256, tree_hashes
from .sample_filter import SampleFilterMask, build_filter_mask, load_filter_mask
from .training_support import DatasetAudit, TrainingError


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
    expected_meta = manifest.get("expected")
    if manifest.get("diagnostic") is True and isinstance(expected_meta, dict):
        expected_formal = int(expected_meta.get("formal_episodes", 0))
        expected_training = int(expected_meta.get("training_episodes", 0))
        expected_validation = int(expected_meta.get("validation_episodes", 0))
        val_positions = tuple(int(pos) for pos in expected_meta.get("validation_positions", []))
    else:
        expected_formal = config.dataset.expected_formal_episodes
        val_positions = config.dataset.validation_positions
        expected_training = config.dataset.expected_training_episodes
        expected_validation = config.dataset.expected_validation_episodes
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
    if set(train_indices) & set(validation_indices):
        raise TrainingError("training and validation splits overlap")
    if set(validation_indices) != set(val_positions):
        raise TrainingError("validation split is not at the configured success positions")
    if set(train_indices) | set(validation_indices) != set(range(expected_formal)):
        raise TrainingError("splits do not partition the configured success range")
    if len(train_indices) != expected_training or len(validation_indices) != expected_validation:
        raise TrainingError(
            f"split cardinality is not {expected_training}/{expected_validation}"
        )
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
        raise TrainingError(f"{split_name} action shape is not 16")
    if features.get("observation.state", {}).get("shape") != [config.dataset.state_dim]:
        raise TrainingError(f"{split_name} observation.state shape is not 16")
    if not (split_root / "meta/stats.json").is_file():
        raise TrainingError(f"{split_name} meta/stats.json is missing")
    if not config.verify_hashes:
        return
    declared = split.get("files")
    if not isinstance(declared, dict) or not declared:
        raise TrainingError(f"{split_name} manifest has no file hash inventory")
    actual = tree_hashes(split_root)
    if actual == declared:
        return
    missing = sorted(set(declared) - set(actual))
    extra = sorted(set(actual) - set(declared))
    changed = sorted(
        name for name in set(actual) & set(declared) if actual[name] != declared[name]
    )
    raise TrainingError(
        f"{split_name} dataset file hash mismatch: missing={missing[:4]} "
        f"extra={extra[:4]} changed={changed[:4]}"
    )


def _verify_episode_facts(
    config: TrainingConfig,
    split_name: str,
    episode: dict[str, Any],
    fact_records: list[dict[str, Any]],
) -> None:
    facts = episode.get("source_facts")
    if not isinstance(facts, dict):
        raise TrainingError(f"{split_name} episode has no source task/runtime facts")
    stored_facts_hash = facts.get("facts_sha256")
    payload = {key: value for key, value in facts.items() if key != "facts_sha256"}
    if stored_facts_hash != canonical_hash(payload):
        raise TrainingError(f"{split_name} source facts hash mismatch")
    if (
        payload.get("task", {}).get("outcome") != "succeeded"
        or payload.get("task", {}).get("reason") != "TASK_SUCCEEDED"
    ):
        raise TrainingError(f"{split_name} source task did not succeed")
    if (
        payload.get("runtime", {}).get("outcome") not in {"stopped", "completed"}
        or payload.get("runtime", {}).get("reason") != "TASK_SUCCEEDED"
    ):
        raise TrainingError(f"{split_name} source runtime did not stop successfully")
    if payload.get("recording", {}).get("complete") is not True:
        raise TrainingError(f"{split_name} source recording is incomplete")
    if payload.get("quality_qualified") is not True:
        raise TrainingError(f"{split_name} source quality is not qualified")
    fact_records.append(
        {
            "attempt_id": episode.get("attempt_id"),
            "seed": episode.get("seed"),
            "facts": payload,
        }
    )
    if not config.verify_hashes:
        return
    files = episode.get("source_files")
    if not isinstance(files, dict):
        raise TrainingError(f"{split_name} episode has no source file hashes")
    for _name, record in files.items():
        if not isinstance(record, dict) or "path" not in record or "sha256" not in record:
            raise TrainingError(f"{split_name} source hash record is malformed")
        source = Path(str(record["path"]))
        if not source.is_file() or file_sha256(source) != record["sha256"]:
            raise TrainingError(f"{split_name} source export hash mismatch: {source}")


def _verify_split_episodes(
    config: TrainingConfig,
    split_name: str,
    split: dict[str, Any],
    fact_records: list[dict[str, Any]],
) -> None:
    for episode in split.get("episodes", []):
        if not isinstance(episode, dict):
            raise TrainingError(f"{split_name} episode entry is not an object")
        _verify_episode_facts(config, split_name, episode, fact_records)


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
    config: TrainingConfig,
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
        verified_files=(
            len(training.get("files", {})) + len(validation.get("files", {}))
            if config.verify_hashes
            else 0
        ),
        synthetic=synthetic,
    )


def verify_dataset_manifest(
    config: TrainingConfig,
    *,
    allow_synthetic: bool = False,
    require_full: bool | None = None,
) -> DatasetAudit:
    """Check configured split disjointness, validation slots and every declared hash."""
    manifest_path, manifest, synthetic = _load_manifest(config, allow_synthetic, require_full)
    training, validation, train_indices, validation_indices = _manifest_splits(config, manifest)
    _verify_profile(config, manifest)
    _verify_processors(manifest)
    root = config.data_root
    fact_records: list[dict[str, Any]] = []
    for split_name, split in (("training", training), ("validation", validation)):
        _verify_split_metadata(config, root, split_name, split)
        _verify_split_episodes(config, split_name, split, fact_records)
    if manifest.get("source_facts_sha256") != canonical_hash(fact_records):
        raise TrainingError("dataset manifest source task/runtime facts hash mismatch")
    _verify_full_lengths(config, require_full, training, validation)
    return _audit(
        root=root,
        training=training,
        validation=validation,
        train_indices=train_indices,
        validation_indices=validation_indices,
        manifest_path=manifest_path,
        config=config,
        synthetic=synthetic,
    )


def sample_filter(config: TrainingConfig, audit: DatasetAudit) -> SampleFilterMask | None:
    """Return the full-mode anchor mask, computing it in memory from the rule YAML."""
    if config.sample_filter is None or config.mode != "full":
        return None
    suffix = config.sample_filter.suffix.lower()
    if suffix == ".json":
        mask = load_filter_mask(config.sample_filter)
    elif suffix in {".yaml", ".yml"}:
        mask = build_filter_mask(rule_path=config.sample_filter, dataset_root=config.data_root)
    else:
        raise TrainingError("data.sample_filter must be a rule YAML or a mask JSON")
    if mask.document["dataset"]["manifest"]["sha256"] != audit.manifest_sha256:
        raise TrainingError("sample filter was generated from a different dataset manifest")
    if mask.source_frames != audit.train_frames:
        raise TrainingError("sample filter source frame count differs from the training split")
    return mask


__all__ = ["sample_filter", "verify_dataset_manifest"]
