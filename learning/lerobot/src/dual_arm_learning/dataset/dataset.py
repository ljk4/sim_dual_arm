"""ROS-free conversion from accepted episode exports to two LeRobot datasets."""

from __future__ import annotations

import json
import os
import random
import shutil
import time
import uuid
from pathlib import Path
from typing import Any

from ..config import DatasetConfig
from ..console_progress import ConsoleProgress
from .common import (
    ExportError,
    canonical_hash,
    configure_local_caches,
    file_sha256,
    load_json_document,
    resolve_export_dir,
    source_archive_sha256,
    tree_hashes,
)
from .export import (  # noqa: F401 - re-exported for tests and callers
    SourceEpisode,
    _predict_episode,
    _profile_document,
    _select_entries,
    _validate_export,
    _validate_row,
)
from .manifest import _write_processors, _write_split, build_manifest


def _resolve_episodes(
    select: dict[str, Any],
    config: DatasetConfig,
    profile_hash: str,
    progress: ConsoleProgress | None = None,
) -> tuple[list[SourceEpisode], list[SourceEpisode]]:
    entries = _select_entries(select, config)
    validation_slots = set(
        random.Random(0).sample(range(len(entries)), config.expected_validation_episodes)
    )
    train_eps: list[SourceEpisode] = []
    validation_eps: list[SourceEpisode] = []
    if progress is not None:
        progress.phase("校验源数据", len(entries), unit="条")
    for entry in entries:
        episode = _predict_episode(entry, config, profile_hash)
        if episode.global_index in validation_slots:
            validation_eps.append(episode)
        else:
            train_eps.append(episode)
        if progress is not None:
            progress.advance()
    return train_eps, validation_eps


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
    train_eps: list[SourceEpisode],
    validation_eps: list[SourceEpisode],
    config: DatasetConfig,
    joint_names: tuple[str, ...],
    profile_id: str,
    progress: ConsoleProgress | None = None,
) -> None:
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
    """Materialise the two independent datasets and return the manifest."""
    _validate_output_target(config, replace, synthetic)
    select = load_json_document(select_path)
    profile_doc, joint_names, camera_names = _profile_document(config)
    profile_hash = canonical_hash(profile_doc)
    profile_id = profile_doc.get("profile_id")
    if not isinstance(profile_id, str) or not profile_id:
        raise ExportError(f"{config.profile_path}: profile has no profile_id")
    train_eps, validation_eps = _resolve_episodes(select, config, profile_hash, progress)
    output = _prepare_output(config, replace, synthetic)
    configure_local_caches(config.cache_root, data_root=config.output_root)
    free = shutil.disk_usage(output.parent).free
    if free < 1024 * 1024 * 1024:
        raise ExportError(f"{output.parent}: less than 1 GiB free for conversion")
    temp = output.parent / f".{output.name}.partial-{os.getpid()}-{uuid.uuid4().hex[:8]}"
    temp.mkdir(parents=True, exist_ok=False)
    started = time.time_ns()
    try:
        _write_datasets(
            temp=temp,
            output_name=output.name,
            train_eps=train_eps,
            validation_eps=validation_eps,
            config=config,
            joint_names=joint_names,
            profile_id=profile_id,
            progress=progress,
        )
        if progress is not None:
            progress.phase("保存训练统计")
        _write_processors(temp, output.name, config)
        if progress is not None:
            progress.phase("核对哈希并生成清单")
        manifest = build_manifest(
            temp=temp,
            config=config,
            select_path=select_path,
            select=select,
            config_path=config_path,
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
    "resolve_export_dir",
    "source_archive_sha256",
    "tree_hashes",
]
