"""LeRobot split writing and manifest assembly for dataset conversion."""

from __future__ import annotations

import hashlib
import json
import time
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

from ..config import DatasetConfig
from ..console_progress import ConsoleProgress
from .common import ExportError, file_sha256, source_archive_sha256
from .export import SourceEpisode

_NORMALIZED_VECTOR_FEATURES = ("observation.state", "observation.velocity", "action")
_SHARD_SIZE_MB = 256


def _stable_vector_statistics(values: np.ndarray) -> dict[str, list]:
    vectors = np.asarray(values, dtype=np.float64)
    result = {
        "min": vectors.min(axis=0).tolist(),
        "max": vectors.max(axis=0).tolist(),
        "mean": vectors.mean(axis=0).tolist(),
        "std": vectors.std(axis=0).tolist(),
        "count": [len(vectors)],
    }
    for name, quantile in (("q01", 0.01), ("q10", 0.1), ("q50", 0.5), ("q90", 0.9), ("q99", 0.99)):
        result[name] = np.quantile(vectors, quantile, axis=0).tolist()
    return result


def _write_vector_statistics(root: Path) -> None:
    """Derive normalization statistics from stored rows with centered float64 variance."""
    import pyarrow as pa
    import pyarrow.parquet as pq

    table = pq.ParquetDataset(root / "data").read(
        columns=["episode_index", *_NORMALIZED_VECTOR_FEATURES]
    )
    indices = table.column("episode_index").to_numpy()
    vectors = {
        key: np.asarray(table.column(key).to_pylist(), dtype=np.float64)
        for key in _NORMALIZED_VECTOR_FEATURES
    }
    episodes = {
        int(index): {
            key: _stable_vector_statistics(values[indices == index])
            for key, values in vectors.items()
        }
        for index in np.unique(indices)
    }
    for path in sorted((root / "meta/episodes").rglob("*.parquet")):
        metadata = pq.read_table(path)
        episode_indices = metadata.column("episode_index").to_pylist()
        for key in _NORMALIZED_VECTOR_FEATURES:
            for statistic in episodes[episode_indices[0]][key]:
                name = f"stats/{key}/{statistic}"
                values = [episodes[index][key][statistic] for index in episode_indices]
                column = pa.array(values, type=metadata.schema.field(name).type)
                metadata = metadata.set_column(metadata.schema.get_field_index(name), name, column)
        pq.write_table(metadata, path)
    path = root / "meta/stats.json"
    stats = json.loads(path.read_text(encoding="utf-8"))
    for key, values in vectors.items():
        stats[key] = _stable_vector_statistics(values)
    path.write_text(json.dumps(stats, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def _write_split(
    *,
    root: Path,
    repo_id: str,
    episodes: list[SourceEpisode],
    config: DatasetConfig,
    joint_names: tuple[str, ...],
    profile_id: str,
    progress: ConsoleProgress | None = None,
    progress_label: str = "",
) -> None:
    from lerobot.configs.video import RGBEncoderConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    features = config.feature_spec()
    for key in _NORMALIZED_VECTOR_FEATURES:
        features[key]["names"] = list(joint_names)
    for camera in config.camera_names:
        features[f"observation.images.{camera}"]["names"] = ["height", "width", "channels"]
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        root=root,
        fps=config.fps,
        features=features,
        robot_type=profile_id,
        use_videos=True,
        video_backend=config.video_backend,
        rgb_encoder=RGBEncoderConfig(
            vcodec=config.rgb_encoder.vcodec,
            g=config.rgb_encoder.g,
            crf=config.rgb_encoder.crf,
            preset=config.rgb_encoder.preset,
        ),
        video_files_size_in_mb=_SHARD_SIZE_MB,
        data_files_size_in_mb=_SHARD_SIZE_MB,
        encoder_threads=config.encoder_threads,
        image_writer_threads=config.image_writer_threads,
    )
    try:
        for episode_index, episode in enumerate(episodes):
            for _index, row in enumerate(episode.rows):
                if progress is not None and _index % 10 == 0:
                    progress.detail(
                        f"{progress_label}第 {episode_index + 1} 条 · "
                        f"{_index + 1}/{len(episode.rows)} 帧"
                    )
                frame: dict[str, Any] = {
                    "observation.state": row["_vectors"]["measured"].astype(np.float32),
                    "observation.velocity": row["_vectors"]["velocity"].astype(np.float32),
                    "action": row["_vectors"]["applied"].astype(np.float32),
                    "observation.backend_time_ns": np.asarray(
                        [row["observation_backend_time_ns"]], dtype=np.int64
                    ),
                    "action.backend_time_ns": np.asarray(
                        [row["label_backend_time_ns"]], dtype=np.int64
                    ),
                    "source.episode_index": np.asarray([episode.global_index], dtype=np.int64),
                    "source.seed": np.asarray([episode.seed], dtype=np.int64),
                    "task": config.task_text,
                }
                for camera in config.camera_names:
                    entry = row["rgb"][camera]
                    with Image.open(episode.export_dir / entry["path"]) as image:
                        image.load()
                        if image.mode != "RGB":
                            raise ExportError(f"rgb: {entry['path']} is not RGB")
                        frame[f"observation.images.{camera}"] = np.asarray(image, dtype=np.uint8)
                dataset.add_frame(frame)
            if progress is not None:
                progress.detail(f"{progress_label}第 {episode_index + 1} 条 · 编码视频")
            dataset.save_episode(parallel_encoding=True)
            if progress is not None:
                progress.advance()
    finally:
        if progress is not None:
            progress.detail("整理数据集文件")
        dataset.finalize()
    _write_vector_statistics(root)


def _episode_filter_metadata(episode: SourceEpisode, state_dim: int) -> dict[str, Any]:
    return {
        "chunk_seq": [int(row["raw_chunk_seq"]) for row in episode.rows],
        "applied_velocities": [
            [float(value) for value in row.get("applied_velocities", [0.0] * state_dim)]
            for row in episode.rows
        ],
        "phase": [
            row.get("phase") if isinstance(row.get("phase"), str) and row.get("phase") else None
            for row in episode.rows
        ],
        "gate_state": [
            row.get("gate_state")
            if isinstance(row.get("gate_state"), str) and row.get("gate_state")
            else None
            for row in episode.rows
        ],
    }


def _split_manifest(
    root: Path, episodes: list[SourceEpisode], config: DatasetConfig
) -> dict[str, Any]:
    records = []
    for dataset_index, episode in enumerate(episodes):
        first = episode.rows[0]
        last = episode.rows[-1]
        records.append(
            {
                "dataset_episode_index": dataset_index,
                "global_success_index": episode.global_index,
                "attempt_id": episode.attempt_id,
                "seed": episode.seed,
                "attempt_dir": str(episode.attempt_dir.resolve()),
                "export_dir": str(episode.export_dir.resolve()),
                "frame_count": len(episode.rows),
                "start_backend_time_ns": first["observation_backend_time_ns"],
                "end_backend_time_ns": last["label_backend_time_ns"],
                "source_facts": episode.source_facts,
                "raw_chunk_count": len(episode.raw_by_chunk),
                "action_lead_ns": config.action_lead_ns,
                "observation_period_ns": config.action_lead_ns,
                "filter_metadata": _episode_filter_metadata(episode, config.state_dim),
            }
        )
    return {
        "root": root.name,
        "episode_count": len(episodes),
        "frame_count": sum(len(episode.rows) for episode in episodes),
        "episodes": records,
    }


def _write_processors(temp: Path, output_name: str, config: DatasetConfig) -> None:
    from lerobot.configs import FeatureType, PolicyFeature
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.policies.act.configuration_act import ACTConfig
    from lerobot.policies.act.processor_act import make_act_pre_post_processors

    train_dataset = LeRobotDataset(
        repo_id=f"local/{output_name}-train",
        root=temp / "train",
        video_backend=config.video_backend,
    )
    stats = train_dataset.meta.stats
    if not stats or "action" not in stats:
        raise ExportError("train dataset has no action stats")
    processors = temp / "processors"
    processors.mkdir(parents=True, exist_ok=False)
    inputs = {"observation.state": PolicyFeature(FeatureType.STATE, (config.state_dim,))}
    for camera in config.camera_names:
        inputs[f"observation.images.{camera}"] = PolicyFeature(
            FeatureType.VISUAL, (3, config.camera_height, config.camera_width)
        )
    policy_config = ACTConfig(
        input_features=inputs,
        output_features={"action": PolicyFeature(FeatureType.ACTION, (config.state_dim,))},
        chunk_size=config.chunk_size,
        n_action_steps=config.chunk_size,
        device="cpu",
        pretrained_backbone_weights=None,
    )
    pre, post = make_act_pre_post_processors(policy_config, stats)
    pre.save_pretrained(processors, config_filename="policy_preprocessor.json")
    post.save_pretrained(processors, config_filename="policy_postprocessor.json")


def _split_sections(
    train_section: dict[str, Any],
    validation_section: dict[str, Any],
    train_eps: list[SourceEpisode],
    validation_eps: list[SourceEpisode],
) -> dict[str, Any]:
    return {
        "training": {
            "root": "train",
            "episode_count": len(train_eps),
            "frame_count": train_section["frame_count"],
            "episodes": train_section["episodes"],
            "stats_source": "train/meta/stats.json",
        },
        "validation": {
            "root": "validation",
            "episode_count": len(validation_eps),
            "frame_count": validation_section["frame_count"],
            "episodes": validation_section["episodes"],
            "stats_source": "validation/meta/stats.json",
        },
    }


def _manifest_body(
    *,
    config: DatasetConfig,
    select_path: Path,
    select: dict[str, Any],
    config_path: Path | None,
    train_section: dict[str, Any],
    validation_section: dict[str, Any],
    train_eps: list[SourceEpisode],
    validation_eps: list[SourceEpisode],
    joint_names: tuple[str, ...],
    camera_names: tuple[str, ...],
    profile_hash: str,
    profile_id: str,
    started: int,
    synthetic: bool,
    selection: dict[str, Any] | None = None,
    overrides: dict[str, Any] | None = None,
) -> dict[str, Any]:
    manifest = {
        "status": "complete",
        "synthetic": bool(synthetic),
        "created_wall_time_ns": time.time_ns(),
        "conversion_seconds": (time.time_ns() - started) / 1e9,
        "project_root": str(config.project_root),
        "select": {
            "path": str(select_path.resolve()),
            "sha256": file_sha256(select_path),
            "status": select.get("status"),
            "qualified": select.get("qualified"),
            "rule": select.get("rule", ""),
        },
        "expected": {
            "formal_episodes": len(train_eps) + len(validation_eps),
            "training_episodes": len(train_eps),
            "validation_episodes": len(validation_eps),
            "validation_percent": config.validation_percent,
            "validation_positions": [episode.global_index for episode in validation_eps],
        },
        "config": {
            "path": str(config_path.resolve()) if config_path else "",
            "sha256": file_sha256(config_path) if config_path else "",
            "dataset_config_sha256": hashlib.sha256(
                json.dumps(config.model_dump(mode="json"), sort_keys=True).encode()
            ).hexdigest(),
        },
        "profile": {
            "path": str(config.profile_path.resolve()),
            "file_sha256": file_sha256(config.profile_path),
            "semantic_sha256": profile_hash,
            "profile_id": profile_id,
            "joint_names": list(joint_names),
            "camera_names": list(camera_names),
        },
        "lerobot": {
            "source_revision": config.lerobot_source_revision,
            "source_archive_sha256": source_archive_sha256(config),
            "rgb_encoder": {
                "vcodec": config.rgb_encoder.vcodec,
                "g": config.rgb_encoder.g,
                "crf": config.rgb_encoder.crf,
                "preset": config.rgb_encoder.preset,
            },
            "video_backend": config.video_backend,
            "fps": config.fps,
            "chunk_size": config.chunk_size,
            "action_lead_ns": config.action_lead_ns,
        },
        "splits": _split_sections(train_section, validation_section, train_eps, validation_eps),
        "processors": {
            "root": "processors",
            "training_stats_only": True,
        },
    }
    if selection is not None:
        manifest["selection"] = selection
    if overrides is not None:
        manifest["overrides"] = overrides
    return manifest


def build_manifest(
    *,
    temp: Path,
    config: DatasetConfig,
    select_path: Path,
    select: dict[str, Any],
    config_path: Path | None,
    train_eps: list[SourceEpisode],
    validation_eps: list[SourceEpisode],
    joint_names: tuple[str, ...],
    camera_names: tuple[str, ...],
    profile_hash: str,
    profile_id: str,
    started: int,
    synthetic: bool,
    selection: dict[str, Any] | None = None,
    overrides: dict[str, Any] | None = None,
) -> dict[str, Any]:
    train_section = _split_manifest(temp / "train", train_eps, config)
    validation_section = _split_manifest(temp / "validation", validation_eps, config)
    manifest = _manifest_body(
        config=config,
        select_path=select_path,
        select=select,
        config_path=config_path,
        train_section=train_section,
        validation_section=validation_section,
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
    (temp / "manifest.json").write_text(
        json.dumps(manifest, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    return manifest


__all__ = ["_stable_vector_statistics", "_write_vector_statistics", "build_manifest"]
