"""Reading, digesting and verifying a sample-filter mask document."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .dataset_common import canonical_hash, file_sha256
from .sample_filter_rules import FilterError

FILTER_FORMAT = "dual-arm-act-sample-filter-v1"


@dataclass(frozen=True, slots=True)
class SampleFilterMask:
    path: Path
    file_sha256: str
    digest: str
    eligible_indices: tuple[int, ...]
    source_frames: int
    excluded_frames: int
    document: dict[str, Any]

    def summary(self) -> dict[str, Any]:
        return {
            "path": str(self.path),
            "sha256": self.file_sha256,
            "digest": self.digest,
            "source_frames": self.source_frames,
            "eligible_frames": len(self.eligible_indices),
            "excluded_frames": self.excluded_frames,
            "reduction_ratio": self.excluded_frames / self.source_frames,
        }


def read_filter_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise FilterError(f"{path}: cannot read JSON: {error}") from error
    if not isinstance(value, dict):
        raise FilterError(f"{path}: expected an object")
    return value


def mask_digest(document: dict[str, Any]) -> str:
    payload = {
        key: value
        for key, value in document.items()
        if key not in {"filter_digest", "created_wall_time_ns"}
    }
    return canonical_hash(payload)


def write_filter_manifest(document: dict[str, Any], output: str | Path) -> Path:
    target = Path(output)
    if target.exists():
        raise FilterError(f"{target}: output already exists; refusing to overwrite")
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".tmp")
    payload = json.dumps(document, indent=2, allow_nan=False) + "\n"
    temporary.write_text(payload, encoding="utf-8")
    os.replace(temporary, target)
    return target


def _verify_sources(document: dict[str, Any]) -> None:
    dataset = document.get("dataset", {})
    for key in ("manifest", "select"):
        record = dataset.get(key, {})
        path = Path(str(record.get("path", "")))
        if not path.is_file() or file_sha256(path) != record.get("sha256"):
            raise FilterError(f"{path}: bound source hash changed")
    rule = document.get("rule", {})
    rule_path = Path(str(rule.get("path", "")))
    if not rule_path.is_file() or file_sha256(rule_path) != rule.get("sha256"):
        raise FilterError(f"{rule_path}: bound filter rule changed")
    for section in ("training", "validation"):
        record = document.get(section, {}).get("parquet", {})
        path = Path(str(record.get("path", "")))
        if not path.is_file() or file_sha256(path) != record.get("sha256"):
            raise FilterError(f"{path}: bound Parquet hash changed")
    for episode in document.get("training", {}).get("episodes", []):
        labels = episode.get("labels", {})
        path = Path(str(labels.get("path", "")))
        if not path.is_file() or file_sha256(path) != labels.get("sha256"):
            raise FilterError(f"{path}: bound labels hash changed")
        for source in episode.get("phase_gate_sources", []):
            path = Path(str(source.get("path", "")))
            if not path.is_file() or file_sha256(path) != source.get("sha256"):
                raise FilterError(f"{path}: bound phase/gate source hash changed")


def _validation_section(source: Path, validation: object) -> None:
    if not isinstance(validation, dict):
        raise FilterError(f"{source}: training/validation sections are missing")
    if validation.get("filtered") is not False:
        raise FilterError(f"{source}: validation set must not be filtered")
    attempt_ids = validation.get("attempt_ids")
    seeds = validation.get("seeds")
    if (
        not isinstance(attempt_ids, list)
        or not isinstance(seeds, list)
        or validation.get("episode_count") != len(attempt_ids)
        or len(attempt_ids) != len(seeds)
    ):
        raise FilterError(f"{source}: validation episode identity is inconsistent")
    if (
        validation.get("kept_frames") != validation.get("source_frames")
        or validation.get("excluded_frames") != 0
    ):
        raise FilterError(f"{source}: validation split is not preserved")


def _check_episode_indices(source: Path, episode: dict[str, Any]) -> tuple[list[int], int]:
    source_indices = episode.get("source_indices")
    kept_indices = episode.get("kept_indices")
    excluded_indices = episode.get("excluded_indices")
    if not all(
        isinstance(item, list) for item in (source_indices, kept_indices, excluded_indices)
    ):
        raise FilterError(f"{source}: episode index lists are missing")
    if episode.get("source_frame_count") != len(source_indices):
        raise FilterError(f"{source}: source frame count disagrees with its index list")
    if episode.get("kept_count") != len(kept_indices):
        raise FilterError(f"{source}: kept count disagrees with its index list")
    if episode.get("excluded_count") != len(excluded_indices):
        raise FilterError(f"{source}: excluded count disagrees with its index list")
    if len(kept_indices) < 1:
        raise FilterError(f"{source}: an episode has no training anchors")
    for name, values in (
        ("source", source_indices),
        ("kept", kept_indices),
        ("excluded", excluded_indices),
    ):
        if any(isinstance(value, bool) or not isinstance(value, int) for value in values):
            raise FilterError(f"{source}: {name} indices must be integers")
        if values != sorted(values):
            raise FilterError(f"{source}: {name} indices are not sorted")
    if set(kept_indices) & set(excluded_indices):
        raise FilterError(f"{source}: kept and excluded indices overlap")
    if set(kept_indices) | set(excluded_indices) != set(source_indices):
        raise FilterError(f"{source}: kept/excluded indices do not partition the source")
    reasons = episode.get("excluded_reasons", {})
    if not isinstance(reasons, dict) or any(
        reason != "redundant_static_anchor" for reason in reasons
    ):
        raise FilterError(f"{source}: excluded reasons are not recognised")
    if sum(reasons.values()) != len(excluded_indices):
        raise FilterError(f"{source}: excluded reason count disagrees with the index list")
    return kept_indices, len(source_indices)


def _training_section(
    source: Path, training: dict[str, Any]
) -> tuple[list[int], int]:
    episodes = training.get("episodes")
    if not isinstance(episodes, list) or not episodes:
        raise FilterError(f"{source}: training episodes are missing")
    seen: set[int] = set()
    eligible: list[int] = []
    source_total = 0
    for episode in episodes:
        if not isinstance(episode, dict):
            raise FilterError(f"{source}: episode entry is not an object")
        kept, count = _check_episode_indices(source, episode)
        for value in episode["source_indices"]:
            if value in seen:
                raise FilterError(f"{source}: duplicate original index {value}")
            seen.add(value)
        eligible.extend(kept)
        source_total += count
    if any(value < 0 or value >= source_total for value in seen):
        raise FilterError(f"{source}: original index is out of range")
    if training.get("source_frames") != source_total:
        raise FilterError(f"{source}: training source frame total disagrees")
    if training.get("eligible_frames") != len(eligible):
        raise FilterError(f"{source}: eligible frame total disagrees")
    if training.get("excluded_frames") != source_total - len(eligible):
        raise FilterError(f"{source}: excluded frame total disagrees")
    if len(set(eligible)) != len(eligible):
        raise FilterError(f"{source}: duplicate eligible index")
    return eligible, source_total


def load_filter_mask(path: str | Path, *, verify_source: bool = True) -> SampleFilterMask:
    source = Path(path).resolve()
    document = read_filter_json(source)
    if document.get("format") != FILTER_FORMAT:
        raise FilterError(f"{source}: unknown sample filter format")
    if document.get("filter_digest") != mask_digest(document):
        raise FilterError(f"{source}: filter digest does not match its contents")
    training = document.get("training")
    if not isinstance(training, dict):
        raise FilterError(f"{source}: training/validation sections are missing")
    _validation_section(source, document.get("validation"))
    eligible, source_total = _training_section(source, training)
    if verify_source:
        _verify_sources(document)
    return SampleFilterMask(
        path=source,
        file_sha256=file_sha256(source),
        digest=str(document["filter_digest"]),
        eligible_indices=tuple(eligible),
        source_frames=source_total,
        excluded_frames=source_total - len(eligible),
        document=document,
    )


__all__ = [
    "FILTER_FORMAT",
    "SampleFilterMask",
    "load_filter_mask",
    "mask_digest",
    "read_filter_json",
    "write_filter_manifest",
]
