"""ROS-free conversion from accepted episode exports to two LeRobot datasets.

The 034 export is the source of truth: JSON, PNG and the original teacher raw
chunks.  This module never opens MCAP and never imports ``rclpy``.  It verifies
capture/label clocks, PNG payloads, raw chunks and episode boundaries before a
single future action is written.  The two splits are materialised as separate
``LeRobotDataset`` roots; the validation split never contributes statistics.

Export validation lives in :mod:`dataset_export`, split writing and manifest
assembly in :mod:`dataset_manifest`, and shared primitives in
:mod:`dataset_common`.
"""

from __future__ import annotations

import json
import os
import shutil
import time
import uuid
from pathlib import Path
from typing import Any

from .config import DatasetConfig
from .console_progress import ConsoleProgress
from .dataset_common import (
    ExportError,
    canonical_hash,
    configure_local_caches,
    file_sha256,
    load_json_document,
    resolve_export_dir,
    source_archive_sha256,
    tree_hashes,
)
from .dataset_export import (  # noqa: F401 - re-exported for tests and callers
    SourceEpisode,
    _predict_episode,
    _profile_document,
    _select_entries,
    _validate_export,
    _validate_row,
)
from .dataset_manifest import _write_split, build_manifest


def _resolve_episodes(
    select: dict[str, Any],
    config: DatasetConfig,
    profile_hash: str,
    progress: ConsoleProgress | None = None,
) -> list[SourceEpisode]:
    training_entries, validation_entries = _select_entries(select, config)
    validation_slots = set(config.validation_positions)
    training_slots = [
        index for index in range(config.expected_formal_episodes) if index not in validation_slots
    ]
    if len(training_slots) != len(training_entries):
        raise ExportError(
            "select: training cardinality does not fit the fixed validation positions"
        )
    selected = [
        dict(entry, global_index=slot, split="training")
        for entry, slot in zip(training_entries, training_slots, strict=True)
    ] + [
        dict(entry, global_index=slot, split="validation")
        for entry, slot in zip(validation_entries, config.validation_positions, strict=True)
    ]
    if progress is not None:
        progress.phase("校验源数据", len(selected), unit="条")
    parsed = []
    for entry in selected:
        parsed.append(_predict_episode(entry, config, profile_hash))
        if progress is not None:
            progress.advance()
    if len(parsed) != config.expected_formal_episodes:
        raise ExportError(
            "select: did not resolve exactly "
            f"{config.expected_formal_episodes} accepted exports"
        )
    if len({episode.attempt_id for episode in parsed}) != len(parsed):
        raise ExportError("select: duplicate attempt IDs after resolution")
    if len({episode.seed for episode in parsed}) != len(parsed):
        raise ExportError("select: duplicate seeds after resolution")
    return parsed


def _validate_output_target(config: DatasetConfig, replace: bool, synthetic: bool) -> Path:
    output = config.output_root
    if output.exists():
        if not replace:
            raise ExportError(f"{output}: output root already exists; refusing to overwrite")
        if not output.is_dir():
            raise ExportError(f"{output}: output root is not a directory")
        manifest_path = output / "manifest.json"
        if not manifest_path.is_file():
            try:
                non_empty = any(output.iterdir())
            except OSError as error:
                raise ExportError(f"{output}: cannot inspect output root: {error}") from error
            if non_empty:
                raise ExportError(
                    f"{output}: not an owned dataset output (no manifest.json); refusing to delete"
                )
        existing = load_json_document(manifest_path) if manifest_path.is_file() else {}
        if existing.get("status") == "complete" and not synthetic:
            raise ExportError(f"{output}: complete dataset exists; refusing to overwrite")
    return output


def _prepare_output(config: DatasetConfig, replace: bool, synthetic: bool) -> Path:
    output = _validate_output_target(config, replace, synthetic)
    if output.exists():
        shutil.rmtree(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    return output


def _write_datasets(
    *,
    temp: Path,
    output_name: str,
    parsed: list[SourceEpisode],
    validation_slots: set[int],
    config: DatasetConfig,
    joint_names: tuple[str, ...],
    profile_id: str,
    progress: ConsoleProgress | None = None,
) -> tuple[list[SourceEpisode], list[SourceEpisode]]:
    train_eps = [item for item in parsed if item.global_index not in validation_slots]
    validation_eps = [item for item in parsed if item.global_index in validation_slots]
    if (
        len(train_eps) != config.expected_training_episodes
        or len(validation_eps) != config.expected_validation_episodes
    ):
        raise ExportError("select: split cardinality is inconsistent")
    # One continuous phase covers both splits so the bar never restarts at zero.
    if progress is not None:
        progress.phase(
            "转换数据集", len(train_eps) + len(validation_eps), unit="条"
        )
    _write_split(
        root=temp / "train",
        repo_id=f"local/{output_name}-train",
        episodes=train_eps,
        config=config,
        joint_names=joint_names,
        profile_id=profile_id,
        progress=progress,
        progress_label="训练集 ",
    )
    if progress is not None and validation_eps:
        progress.detail(f"训练集完成，编码验证集 {len(validation_eps)} 条")
    _write_split(
        root=temp / "validation",
        repo_id=f"local/{output_name}-validation",
        episodes=validation_eps,
        config=config,
        joint_names=joint_names,
        profile_id=profile_id,
        progress=progress,
        progress_label="验证集 ",
    )
    return train_eps, validation_eps


def _write_failure_marker(temp: Path, synthetic: bool) -> None:
    marker = temp / "FAILED.json"
    try:
        marker.write_text(
            json.dumps(
                {
                    "status": "failed",
                    "synthetic": bool(synthetic),
                    "failed_wall_time_ns": time.time_ns(),
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
    except OSError:
        pass


def convert(
    *,
    select_path: Path,
    config: DatasetConfig,
    config_path: Path | None = None,
    replace: bool = False,
    synthetic: bool = False,
    progress: ConsoleProgress | None = None,
    selection: dict[str, Any] | None = None,
    overrides: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Materialise the two independent datasets and return the manifest.

    ``synthetic=True`` is only for fixture tests.  It is recorded in the
    manifest and never claims the formal episodes were collected.
    """
    _validate_output_target(config, replace, synthetic)
    select = load_json_document(select_path)
    profile_doc, joint_names, camera_names = _profile_document(config)
    profile_hash = canonical_hash(profile_doc)
    profile_id = profile_doc.get("profile_id")
    if not isinstance(profile_id, str) or not profile_id:
        raise ExportError(f"{config.profile_path}: profile has no profile_id")
    parsed = _resolve_episodes(select, config, profile_hash, progress)
    output = _prepare_output(config, replace, synthetic)
    configure_local_caches(config.cache_root, data_root=config.output_root)
    free = shutil.disk_usage(output.parent).free
    if free < 1024 * 1024 * 1024:
        raise ExportError(f"{output.parent}: less than 1 GiB free for conversion")
    temp = output.parent / f".{output.name}.partial-{os.getpid()}-{uuid.uuid4().hex[:8]}"
    temp.mkdir(parents=True, exist_ok=False)
    started = time.time_ns()
    try:
        validation_slots = set(config.validation_positions)
        train_eps, validation_eps = _write_datasets(
            temp=temp,
            output_name=output.name,
            parsed=parsed,
            validation_slots=validation_slots,
            config=config,
            joint_names=joint_names,
            profile_id=profile_id,
            progress=progress,
        )
        from .dataset_manifest import _write_processors

        if progress is not None:
            progress.phase("保存训练统计")
        _write_processors(temp, output.name, config)
        if progress is not None:
            progress.phase("核对哈希并生成清单")
        manifest = build_manifest(
            temp=temp,
            output_name=output.name,
            config=config,
            select_path=select_path,
            select=select,
            config_path=config_path,
            parsed=parsed,
            train_eps=train_eps,
            validation_eps=validation_eps,
            joint_names=joint_names,
            camera_names=camera_names,
            profile_hash=profile_hash,
            profile_id=profile_id,
            started=started,
            synthetic=synthetic,
            selection=selection,
            overrides=overrides,
        )
        temp.replace(output)
        return manifest
    except Exception:
        _write_failure_marker(temp, synthetic)
        raise


__all__ = [
    "ExportError",
    "SourceEpisode",
    "canonical_hash",
    "configure_local_caches",
    "convert",
    "file_sha256",
    "source_archive_sha256",
    "resolve_export_dir",
    "tree_hashes",
]
