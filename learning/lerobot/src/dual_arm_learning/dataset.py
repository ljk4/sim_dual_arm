"""ROS-free conversion from accepted stack exports to two LeRobot datasets.

The 034 export is the source of truth: JSON, PNG and the original teacher raw
chunks.  This module never opens MCAP and never imports ``rclpy``.  It verifies
capture/label clocks, PNG payloads, raw chunks and episode boundaries before a
single future action is written.  The two splits are materialised as separate
``LeRobotDataset`` roots; the validation split never contributes statistics.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import shutil
import time
import uuid
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

from .config import PNG_SHA, DatasetConfig


class ExportError(ValueError):
    """A source export failed an auditable precondition."""


def _json(path: Path) -> dict[str, Any]:
    def bad_constant(value: str) -> Any:
        raise ExportError(f"{path}: non-finite JSON constant {value!r}")

    try:
        value = json.loads(path.read_text(encoding="utf-8"), parse_constant=bad_constant)
    except OSError as error:
        raise ExportError(f"{path}: cannot read: {error}") from error
    except json.JSONDecodeError as error:
        raise ExportError(f"{path}: invalid JSON: {error}") from error
    if not isinstance(value, dict):
        raise ExportError(f"{path}: expected a JSON object")
    return value


def _int(value: object, location: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ExportError(f"{location}: expected an integer")
    return value


def _float(value: object, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ExportError(f"{location}: expected a number")
    result = float(value)
    if not math.isfinite(result):
        raise ExportError(f"{location}: expected a finite number")
    return result


def _vector(value: object, location: str, size: int) -> np.ndarray:
    if not isinstance(value, list) or len(value) != size:
        raise ExportError(f"{location}: expected an array of {size} numbers")
    result = np.asarray(
        [_float(item, f"{location}[{index}]") for index, item in enumerate(value)], dtype=np.float32
    )
    if not np.isfinite(result).all():
        raise ExportError(f"{location}: contains non-finite values")
    return result


def _chunk(value: object, location: str, rows: int, columns: int) -> np.ndarray:
    if not isinstance(value, list) or len(value) != rows:
        raise ExportError(f"{location}: expected {rows} rows")
    result = np.empty((rows, columns), dtype=np.float32)
    for index, point in enumerate(value):
        result[index] = _vector(point, f"{location}[{index}]", columns)
    if not np.isfinite(result).all():
        raise ExportError(f"{location}: contains non-finite values")
    return result


def configure_local_caches(root: Path, *, data_root: Path | None = None) -> Path:
    """Select and validate an explicit writable, offline cache root.

    ``LEARNING_CACHE_ROOT``/``HF_HOME`` override the configured path, but the
    chosen directory must be writable and must not be inside accepted data.
    No cache is ever created under a read-only dataset root.
    """
    candidate = Path(
        os.environ.get("LEARNING_CACHE_ROOT") or os.environ.get("HF_HOME") or root
    ).expanduser()
    resolved = candidate.resolve()
    if data_root is not None:
        data = Path(data_root).resolve()
        if resolved == data or resolved.is_relative_to(data):
            raise ExportError(f"cache root {resolved} must not be inside data root {data}")
    try:
        resolved.mkdir(parents=True, exist_ok=True)
        probe = resolved / f".write-probe-{os.getpid()}"
        probe.write_text("ok", encoding="utf-8")
        probe.unlink()
    except OSError as error:
        raise ExportError(f"cache root {resolved} is not writable: {error}") from error
    os.environ["HF_HOME"] = str(resolved)
    os.environ["HF_DATASETS_CACHE"] = str(resolved / "datasets")
    os.environ["HF_HUB_OFFLINE"] = "1"
    os.environ["HF_DATASETS_OFFLINE"] = "1"
    os.environ["TRANSFORMERS_OFFLINE"] = "1"
    return resolved


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_hash(document: object) -> str:
    payload = json.dumps(document, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()


def tree_files(root: Path) -> list[Path]:
    return sorted(path for path in root.rglob("*") if path.is_file())


def tree_hashes(root: Path) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for path in tree_files(root):
        result[path.relative_to(root).as_posix()] = {
            "sha256": file_sha256(path),
            "bytes": path.stat().st_size,
        }
    return result


@dataclass(frozen=True, slots=True)
class SourceEpisode:
    attempt_id: str
    seed: int
    attempt_dir: Path
    export_dir: Path
    global_index: int
    rows: tuple[dict[str, Any], ...]
    raw_by_chunk: dict[int, dict[str, Any]]
    frames: dict[int, dict[str, Any]]
    source_files: dict[str, dict[str, Any]]
    quality: dict[str, Any]
    summary: dict[str, Any]
    profile_hash: str
    source_facts: dict[str, Any]


def _profile_document(
    config: DatasetConfig,
) -> tuple[dict[str, Any], tuple[str, ...], tuple[str, ...]]:
    doc = _json(config.profile_path)
    joints = doc.get("joints")
    cameras = doc.get("cameras")
    if not isinstance(joints, list) or len(joints) != config.state_dim:
        raise ExportError("profile: expected exactly 16 joint entries")
    joint_names: list[str] = []
    for index, joint in enumerate(joints):
        if not isinstance(joint, dict) or not isinstance(joint.get("name"), str):
            raise ExportError(f"profile.joints[{index}].name: expected a string")
        joint_names.append(joint["name"])
    if not isinstance(cameras, list):
        raise ExportError("profile.cameras: expected an array")
    camera_names = tuple(str(item.get("name", "")) for item in cameras if isinstance(item, dict))
    if camera_names != config.camera_names:
        raise ExportError("profile.cameras: does not match the configured learning cameras")
    for camera in cameras:
        if not isinstance(camera, dict):
            raise ExportError("profile.cameras: every camera must be an object")
        if (camera.get("width"), camera.get("height")) != (
            config.camera_width,
            config.camera_height,
        ):
            raise ExportError(f"profile.cameras[{camera.get('name')}]: wrong RGB size")
    return doc, tuple(joint_names), camera_names


def resolve_export_dir(attempt_dir: Path, pattern: str) -> Path:
    candidates = sorted(
        path.parent if path.name == "labels.json" else path
        for path in attempt_dir.glob(pattern)
        if (path / "labels.json").is_file() or path.name == "labels.json"
    )
    # ``Path.glob`` already matched the export directory; normalise a direct
    # export directory as well as the campaign's ``stack/<run>/export`` shape.
    unique: list[Path] = []
    for candidate in candidates:
        if candidate not in unique:
            unique.append(candidate)
    if len(unique) != 1:
        raise ExportError(
            f"{attempt_dir}: expected exactly one accepted export matching {pattern!r}, "
            f"found {len(unique)}"
        )
    return unique[0]


def _select_entries(
    select: dict[str, Any], config: DatasetConfig
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    if select.get("status") != "SELECTED":
        raise ExportError(f"select: expected status SELECTED, got {select.get('status')!r}")
    qualified = _int(select.get("qualified"), "select.qualified")
    if qualified != config.expected_formal_episodes:
        raise ExportError(
            f"select.qualified: expected {config.expected_formal_episodes}, got {qualified}"
        )
    training = select.get("training")
    validation = select.get("validation")
    if not isinstance(training, list) or not isinstance(validation, list):
        raise ExportError("select: training and validation must be arrays")
    if len(training) != config.expected_training_episodes:
        raise ExportError(
            f"select.training: expected {config.expected_training_episodes}, got {len(training)}"
        )
    if len(validation) != config.expected_validation_episodes:
        raise ExportError(
            "select.validation: expected "
            f"{config.expected_validation_episodes}, got {len(validation)}"
        )
    all_ids: set[str] = set()
    all_seeds: set[int] = set()
    result: list[list[dict[str, Any]]] = []
    for split_name, entries in (("training", training), ("validation", validation)):
        parsed: list[dict[str, Any]] = []
        for index, raw in enumerate(entries):
            if not isinstance(raw, dict):
                raise ExportError(f"select.{split_name}[{index}]: expected an object")
            attempt_id = raw.get("attempt_id")
            attempt_dir = raw.get("attempt_dir")
            seed = _int(raw.get("seed"), f"select.{split_name}[{index}].seed")
            if not isinstance(attempt_id, str) or not attempt_id:
                raise ExportError(f"select.{split_name}[{index}].attempt_id: expected a string")
            if not isinstance(attempt_dir, str):
                raise ExportError(f"select.{split_name}[{index}].attempt_dir: expected a string")
            if attempt_id in all_ids:
                raise ExportError(f"select: duplicate attempt_id {attempt_id!r}")
            if seed in all_seeds:
                raise ExportError(f"select: duplicate seed {seed}")
            all_ids.add(attempt_id)
            all_seeds.add(seed)
            parsed.append(
                {
                    "attempt_id": attempt_id,
                    "seed": seed,
                    "attempt_dir": Path(attempt_dir)
                    if Path(attempt_dir).is_absolute()
                    else config.project_root / attempt_dir,
                    "split": split_name,
                }
            )
        result.append(parsed)
    return result[0], result[1]


def _validate_png(path: Path, expected: dict[str, Any], config: DatasetConfig) -> np.ndarray:
    if not path.is_file():
        raise ExportError(f"rgb: missing {path}")
    if file_sha256(path) != expected["png_sha256"]:
        raise ExportError(f"rgb: PNG SHA mismatch for {path}")
    try:
        with Image.open(path) as image:
            image.load()
            if image.mode != "RGB" or image.size != (config.camera_width, config.camera_height):
                raise ExportError(
                    f"rgb: {path} is {image.mode} {image.size}, expected RGB "
                    f"{config.camera_width}x{config.camera_height}"
                )
            array = np.asarray(image, dtype=np.uint8)
    except ExportError:
        raise
    except Exception as error:
        raise ExportError(f"rgb: cannot decode {path}: {error}") from error
    if config.verify_png_pixels:
        computed = hashlib.sha256(array.tobytes()).hexdigest()
        if computed != expected["rgb_sha256"]:
            raise ExportError(f"rgb: channel/ordering SHA mismatch for {path}")
    return array


def _validate_row(
    row: object,
    index: int,
    frame_entry: dict[str, Any] | None,
    config: DatasetConfig,
    export_dir: Path,
) -> tuple[dict[str, Any], dict[str, np.ndarray], np.ndarray]:
    if not isinstance(row, dict):
        raise ExportError(f"labels[{index}]: expected an object")
    location = f"labels[{index}]"
    observation_time = _int(
        row.get("observation_sim_time_ns"), f"{location}.observation_sim_time_ns"
    )
    label_time = _int(row.get("label_sim_time_ns"), f"{location}.label_sim_time_ns")
    if label_time != observation_time + config.action_lead_ns:
        raise ExportError(f"{location}: label time is not observation + 50 ms")
    period = _int(row.get("observation_period_ns"), f"{location}.observation_period_ns")
    if period != config.action_lead_ns:
        raise ExportError(f"{location}: observation period is not 50 ms")
    started = _int(row.get("started_sim_time_ns"), f"{location}.started_sim_time_ns")
    block_end = _int(row.get("block_end_sim_time_ns"), f"{location}.block_end_sim_time_ns")
    if not started <= label_time <= block_end:
        raise ExportError(f"{location}: label is outside its executed block")
    applied = _vector(
        row.get("applied_positions"), f"{location}.applied_positions", config.state_dim
    )
    measured = _vector(
        row.get("measured_positions"), f"{location}.measured_positions", config.state_dim
    )
    velocity = _vector(
        row.get("measured_velocities"), f"{location}.measured_velocities", config.state_dim
    )
    rgb = row.get("rgb")
    if not isinstance(rgb, dict) or tuple(sorted(rgb)) != tuple(sorted(config.camera_names)):
        raise ExportError(f"{location}.rgb: expected exactly the three learning cameras")
    if frame_entry is None:
        raise ExportError(f"{location}: no frame_index entry")
    frames = frame_entry.get("labels")
    if not isinstance(frames, dict):
        raise ExportError("frame_index.labels: expected an object")
    frame_mapping = frames.get(str(index))
    if not isinstance(frame_mapping, dict):
        raise ExportError(f"frame_index.labels[{index}]: missing")
    if set(frame_mapping) != set(config.camera_names):
        raise ExportError(f"frame_index.labels[{index}]: camera set does not match labels")
    for camera in config.camera_names:
        entry = rgb[camera]
        if not isinstance(entry, dict):
            raise ExportError(f"{location}.rgb.{camera}: expected an object")
        for field in (
            "path",
            "frame_seq",
            "observation_id",
            "observation_sim_time_ns",
            "png_sha256",
            "rgb_sha256",
        ):
            if field not in entry:
                raise ExportError(f"{location}.rgb.{camera}: missing {field}")
        if entry["observation_sim_time_ns"] != observation_time:
            raise ExportError(f"{location}.rgb.{camera}: capture clock does not match label clock")
        if entry["observation_id"] != row.get("observation_id"):
            raise ExportError(f"{location}.rgb.{camera}: observation ID mismatch")
        for field in (
            "frame_seq",
            "observation_id",
            "observation_sim_time_ns",
            "png_sha256",
            "rgb_sha256",
            "path",
        ):
            if frame_mapping[camera].get(field) != entry.get(field):
                raise ExportError(
                    f"{location}.rgb.{camera}: frame_index.{field} does not match labels"
                )
        if (
            not isinstance(entry["png_sha256"], str)
            or PNG_SHA.fullmatch(entry["png_sha256"]) is None
        ):
            raise ExportError(f"{location}.rgb.{camera}.png_sha256: bad digest")
        if (
            not isinstance(entry["rgb_sha256"], str)
            or PNG_SHA.fullmatch(entry["rgb_sha256"]) is None
        ):
            raise ExportError(f"{location}.rgb.{camera}.rgb_sha256: bad digest")
        relative = Path(str(entry["path"]))
        if relative.is_absolute() or ".." in relative.parts:
            raise ExportError(f"{location}.rgb.{camera}.path: must stay inside the export")
        png_path = export_dir / relative
        if not png_path.resolve().is_relative_to(export_dir.resolve()):
            raise ExportError(f"{location}.rgb.{camera}.path: escapes the export")
        _validate_png(png_path, entry, config)
    return (
        row,
        {"applied": applied, "measured": measured, "velocity": velocity},
        np.asarray([row["raw_chunk_seq"]], dtype=np.int64),
    )


def _source_facts(labels: dict[str, Any], summary: dict[str, Any]) -> dict[str, Any]:
    task = summary.get("task")
    runtime = summary.get("runtime")
    recording = summary.get("recording")
    if not isinstance(task, dict) or task.get("outcome") != "succeeded":
        raise ExportError("export: task outcome is not succeeded")
    if task.get("reason") != "TASK_SUCCEEDED":
        raise ExportError("export: task reason is not TASK_SUCCEEDED")
    if not isinstance(runtime, dict) or runtime.get("outcome") not in {"stopped", "completed"}:
        raise ExportError("export: runtime outcome is not a successful stop")
    if runtime.get("reason") != "TASK_SUCCEEDED":
        raise ExportError("export: runtime reason is not TASK_SUCCEEDED")
    if not isinstance(recording, dict) or recording.get("complete") is not True:
        raise ExportError("export: recording is not complete")
    if recording.get("failure_reason") not in {"", None}:
        raise ExportError("export: recording has a failure reason")
    if summary.get("episode_id") != labels.get("episode_id"):
        raise ExportError("export: summary/task episode identity does not match labels")
    facts = {
        "episode_id": labels.get("episode_id"),
        "task": {"outcome": task.get("outcome"), "reason": task.get("reason")},
        "runtime": {"outcome": runtime.get("outcome"), "reason": runtime.get("reason")},
        "recording": {
            "complete": recording.get("complete"),
            "failure_reason": recording.get("failure_reason", ""),
        },
        "quality_qualified": summary.get("quality_qualified"),
    }
    facts["facts_sha256"] = canonical_hash(facts)
    return facts


def _validate_export(
    export_dir: Path,
    config: DatasetConfig,
    profile_hash: str,
) -> tuple[
    tuple[dict[str, Any], ...],
    dict[int, dict[str, Any]],
    dict[int, dict[str, Any]],
    dict[str, Any],
    dict[str, Any],
]:
    names = (
        "labels.json",
        "raw_predictions.json",
        "frame_index.json",
        "quality.json",
        "summary.json",
    )
    source_files: dict[str, dict[str, Any]] = {}
    for name in names:
        path = export_dir / name
        if not path.is_file():
            raise ExportError(f"{export_dir}: missing {name}")
        source_files[name] = {
            "path": str(path.resolve()),
            "sha256": file_sha256(path),
            "bytes": path.stat().st_size,
        }
    labels = _json(export_dir / "labels.json")
    raw_doc = _json(export_dir / "raw_predictions.json")
    frame_doc = _json(export_dir / "frame_index.json")
    quality = _json(export_dir / "quality.json")
    summary = _json(export_dir / "summary.json")
    if labels.get("schema_version") != 1 or raw_doc.get("schema_version") != 1:
        raise ExportError("export: unsupported schema version")
    if frame_doc.get("schema_version") != 1:
        raise ExportError("frame_index: unsupported schema version")
    if labels.get("robot_profile_hash") != profile_hash:
        raise ExportError("export: robot profile hash does not match the learning profile")
    _source_facts(labels, summary)
    if quality.get("qualified") is not True:
        raise ExportError("export: quality sidecar is not qualified")
    if config.require_quality:
        if (
            quality.get("visibility_qualified") is False
            or quality.get("trajectory_qualified") is False
        ):
            raise ExportError("export: visibility/trajectory qualification is false")
    camera_order = frame_doc.get("camera_order")
    if tuple(camera_order or ()) != config.camera_names:
        raise ExportError("export: frame_index camera order does not match the profile")
    rows_value = labels.get("labels")
    if not isinstance(rows_value, list) or not rows_value:
        raise ExportError("export: labels array is empty")
    summary_labels = labels.get("summary")
    if not isinstance(summary_labels, dict) or summary_labels.get("labels") != len(rows_value):
        raise ExportError("export: label summary count is inconsistent")
    raw_value = raw_doc.get("predictions")
    if not isinstance(raw_value, list) or not raw_value:
        raise ExportError("export: raw_predictions array is empty")
    raw_by_chunk: dict[int, dict[str, Any]] = {}
    for index, prediction in enumerate(raw_value):
        if not isinstance(prediction, dict):
            raise ExportError(f"raw_predictions[{index}]: expected an object")
        chunk_seq = _int(prediction.get("chunk_seq"), f"raw_predictions[{index}].chunk_seq")
        if chunk_seq in raw_by_chunk:
            raise ExportError(f"raw_predictions: duplicate chunk_seq {chunk_seq}")
        times = prediction.get("times_from_observation_ns")
        if not isinstance(times, list) or len(times) != config.chunk_size:
            raise ExportError(f"raw_predictions[{index}]: expected 60 future points")
        expected_times = [
            (offset + 1) * config.action_lead_ns for offset in range(config.chunk_size)
        ]
        if [
            _int(value, f"raw_predictions[{index}].times[{offset}]")
            for offset, value in enumerate(times)
        ] != expected_times:
            raise ExportError(f"raw_predictions[{index}]: future times are not 1..60 x 50 ms")
        positions = _chunk(
            prediction.get("positions"),
            f"raw_predictions[{index}].positions",
            config.chunk_size,
            config.state_dim,
        )
        raw_by_chunk[chunk_seq] = {
            "chunk_seq": chunk_seq,
            "command_id": str(prediction.get("command_id", "")),
            "source_observation_id": _int(
                prediction.get("source_observation_id"),
                f"raw_predictions[{index}].source_observation_id",
            ),
            "source_observation_time_ns": _int(
                prediction.get("source_observation_time_ns"),
                f"raw_predictions[{index}].source_observation_time_ns",
            ),
            "times_from_observation_ns": tuple(expected_times),
            "positions": positions,
        }
    frame_entries = frame_doc.get("labels")
    if not isinstance(frame_entries, dict):
        raise ExportError("frame_index.labels: expected an object")
    rows: list[dict[str, Any]] = []
    frames: dict[int, dict[str, Any]] = {}
    previous_time: int | None = None
    for index, row in enumerate(rows_value):
        _, vectors, _ = _validate_row(
            row,
            index,
            None
            if not isinstance(frame_entries.get(str(index)), dict)
            else {"labels": frame_entries},
            config,
            export_dir,
        )
        observation_time = _int(
            row.get("observation_sim_time_ns"), f"labels[{index}].observation_sim_time_ns"
        )
        if previous_time is not None and observation_time - previous_time != config.action_lead_ns:
            raise ExportError(f"labels[{index}]: observation clocks are not contiguous 20 Hz")
        previous_time = observation_time
        raw_chunk = _int(row.get("raw_chunk_seq"), f"labels[{index}].raw_chunk_seq")
        prediction = raw_by_chunk.get(raw_chunk)
        if prediction is None:
            raise ExportError(f"labels[{index}]: references missing raw chunk {raw_chunk}")
        if row.get("chunk_seq") != raw_chunk:
            raise ExportError(f"labels[{index}]: applied chunk and raw chunk disagree")
        if row.get("raw_source_observation_id") != prediction["source_observation_id"]:
            raise ExportError(f"labels[{index}]: raw source observation ID mismatch")
        if row.get("raw_source_observation_time_ns") != prediction["source_observation_time_ns"]:
            raise ExportError(f"labels[{index}]: raw source observation time mismatch")
        row = dict(row)
        row["_vectors"] = vectors
        rows.append(row)
        frames[index] = frame_entries[str(index)]
    return tuple(rows), raw_by_chunk, frames, quality, summary


def _predict_episode(
    entry: dict[str, Any], config: DatasetConfig, profile_hash: str
) -> SourceEpisode:
    export_dir = resolve_export_dir(entry["attempt_dir"], config.attempt_export_glob)
    rows, raw_by_chunk, frames, quality, summary = _validate_export(
        export_dir, config, profile_hash
    )
    labels = _json(export_dir / "labels.json")
    if labels.get("seed") != entry["seed"]:
        raise ExportError(
            f"{export_dir}: labels seed {labels.get('seed')} != selected seed {entry['seed']}"
        )
    if config.require_quality and summary.get("quality_qualified") is not True:
        raise ExportError(f"{export_dir}: summary is not quality qualified")
    action_source = str(summary.get("action_source", ""))
    if action_source != "teacher":
        raise ExportError(
            f"{export_dir}: formal data must come from teacher, not {action_source!r}"
        )
    facts = _source_facts(labels, summary)
    source_files = {
        name: {
            "path": str((export_dir / name).resolve()),
            "sha256": file_sha256(export_dir / name),
            "bytes": (export_dir / name).stat().st_size,
        }
        for name in (
            "labels.json",
            "raw_predictions.json",
            "frame_index.json",
            "quality.json",
            "summary.json",
        )
    }
    return SourceEpisode(
        attempt_id=entry["attempt_id"],
        seed=entry["seed"],
        attempt_dir=entry["attempt_dir"],
        export_dir=export_dir,
        global_index=entry["global_index"],
        rows=rows,
        raw_by_chunk=raw_by_chunk,
        frames=frames,
        source_files=source_files,
        quality=quality,
        summary=summary,
        profile_hash=profile_hash,
        source_facts=facts,
    )


def _json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, list | tuple):
        return [_json_safe(item) for item in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def _stats_summary(stats: dict[str, dict[str, Any]]) -> dict[str, Any]:
    selected = {
        key: stats[key]
        for key in ("observation.state", "observation.velocity", "action", "raw_action_chunk")
        if key in stats
    }
    return {
        "source": "training_split_only",
        "features": _json_safe(selected),
    }


def _write_split(
    *,
    root: Path,
    repo_id: str,
    episodes: list[SourceEpisode],
    config: DatasetConfig,
    joint_names: tuple[str, ...],
    profile_id: str,
) -> None:
    from lerobot.configs.video import RGBEncoderConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    features = config.feature_spec()
    for key in ("observation.state", "observation.velocity", "action"):
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
        video_files_size_in_mb=config.video_files_size_mb,
        data_files_size_in_mb=config.data_files_size_mb,
        encoder_threads=config.encoder_threads,
        image_writer_threads=config.image_writer_threads,
    )
    try:
        for episode in episodes:
            for _index, row in enumerate(episode.rows):
                frame: dict[str, Any] = {
                    "observation.state": row["_vectors"]["measured"].astype(np.float32),
                    "observation.velocity": row["_vectors"]["velocity"].astype(np.float32),
                    "action": row["_vectors"]["applied"].astype(np.float32),
                    "raw_action_chunk": episode.raw_by_chunk[row["raw_chunk_seq"]][
                        "positions"
                    ].astype(np.float32),
                    "observation.sim_time_ns": np.asarray(
                        [row["observation_sim_time_ns"]], dtype=np.int64
                    ),
                    "action.sim_time_ns": np.asarray([row["label_sim_time_ns"]], dtype=np.int64),
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
            dataset.save_episode(parallel_encoding=False)
    finally:
        dataset.finalize()


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
                "source_files": episode.source_files,
                "frame_count": len(episode.rows),
                "start_sim_time_ns": first["observation_sim_time_ns"],
                "end_sim_time_ns": last["label_sim_time_ns"],
                "quality_sha256": episode.source_files["quality.json"]["sha256"],
                "source_facts": episode.source_facts,
                "raw_chunk_count": len(episode.raw_by_chunk),
                "action_lead_ns": config.action_lead_ns,
                "observation_period_ns": config.action_lead_ns,
            }
        )
    return {
        "root": root.name,
        "episode_count": len(episodes),
        "frame_count": sum(len(episode.rows) for episode in episodes),
        "episodes": records,
        "files": tree_hashes(root),
    }


def convert(
    *,
    select_path: Path,
    config: DatasetConfig,
    config_path: Path | None = None,
    replace: bool = False,
    synthetic: bool = False,
) -> dict[str, Any]:
    """Materialise the two independent datasets and return the manifest.

    ``synthetic=True`` is only for fixture tests.  It is recorded in the
    manifest and never claims the 50 formal episodes were collected.
    """
    select = _json(select_path)
    train_entries, validation_entries = _select_entries(select, config)
    profile_doc, joint_names, camera_names = _profile_document(config)
    profile_hash = canonical_hash(profile_doc)
    profile_id = str(profile_doc.get("profile_id", "panda_stack_learning_v1"))
    # The select tool emits the first 50 successes.  It keeps validation and
    # training in success order, so reconstruct each source's global success
    # position before placing it in a split.
    validation_slots = set(config.validation_positions)
    training_slots = [
        index for index in range(config.expected_formal_episodes) if index not in validation_slots
    ]
    if len(training_slots) != len(train_entries):
        raise ExportError(
            "select: training cardinality does not fit the fixed validation positions"
        )
    selected = [
        dict(entry, global_index=slot, split="training")
        for entry, slot in zip(train_entries, training_slots, strict=True)
    ] + [
        dict(entry, global_index=slot, split="validation")
        for entry, slot in zip(validation_entries, config.validation_positions, strict=True)
    ]
    parsed: list[SourceEpisode] = []
    for entry in selected:
        parsed.append(_predict_episode(entry, config, profile_hash))
    if len(parsed) != config.expected_formal_episodes:
        raise ExportError("select: did not resolve exactly 50 accepted exports")
    if len({episode.attempt_id for episode in parsed}) != len(parsed):
        raise ExportError("select: duplicate attempt IDs after resolution")
    if len({episode.seed for episode in parsed}) != len(parsed):
        raise ExportError("select: duplicate seeds after resolution")

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
        existing = _json(manifest_path) if manifest_path.is_file() else {}
        if existing.get("status") == "complete" and not synthetic:
            raise ExportError(f"{output}: complete dataset exists; refusing to overwrite")
        shutil.rmtree(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    configure_local_caches(config.cache_root, data_root=config.output_root)
    free = shutil.disk_usage(output.parent).free
    if free < 1024 * 1024 * 1024:
        raise ExportError(f"{output.parent}: less than 1 GiB free for conversion")
    temp = output.parent / f".{output.name}.partial-{os.getpid()}-{uuid.uuid4().hex[:8]}"
    temp.mkdir(parents=True, exist_ok=False)
    started = time.time_ns()
    try:
        train_eps = [item for item in parsed if item.global_index not in validation_slots]
        validation_eps = [item for item in parsed if item.global_index in validation_slots]
        if (
            len(train_eps) != config.expected_training_episodes
            or len(validation_eps) != config.expected_validation_episodes
        ):
            raise ExportError("select: split cardinality is inconsistent")
        _write_split(
            root=temp / "train",
            repo_id=f"local/{output.name}-train",
            episodes=train_eps,
            config=config,
            joint_names=joint_names,
            profile_id=profile_id,
        )
        _write_split(
            root=temp / "validation",
            repo_id=f"local/{output.name}-validation",
            episodes=validation_eps,
            config=config,
            joint_names=joint_names,
            profile_id=profile_id,
        )
        # Re-open only the training split and build the processors from its
        # own meta.stats.  The validation meta.stats is never passed here.
        from lerobot.configs import FeatureType, PolicyFeature
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        from lerobot.policies.act.configuration_act import ACTConfig
        from lerobot.policies.act.processor_act import make_act_pre_post_processors

        train_dataset = LeRobotDataset(
            repo_id=f"local/{output.name}-train",
            root=temp / "train",
            video_backend=config.video_backend,
        )
        stats = train_dataset.meta.stats
        if not stats or "action" not in stats:
            raise ExportError("train dataset has no action stats")
        stats_document = _stats_summary(stats)
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
        stats_path = processors / "train_stats_summary.json"
        stats_path.write_text(
            json.dumps(stats_document, indent=2, allow_nan=False) + "\n", encoding="utf-8"
        )
        train_files = tree_hashes(temp / "train")
        validation_files = tree_hashes(temp / "validation")
        processor_files = tree_hashes(processors)
        manifest = {
            "schema_version": 1,
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
            "config": {
                "path": str(config_path.resolve()) if config_path else "",
                "sha256": file_sha256(config_path) if config_path else "",
                "dataset_config_sha256": hashlib.sha256(
                    json.dumps(asdict(config), sort_keys=True, default=str).encode()
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
                "source_archive_sha256": config.lerobot_source_archive_sha256,
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
            "splits": {
                "training": {
                    "root": "train",
                    "episode_count": len(train_eps),
                    "frame_count": sum(len(item.rows) for item in train_eps),
                    "episodes": _split_manifest(temp / "train", train_eps, config)["episodes"],
                    "files": train_files,
                    "stats_source": "train/meta/stats.json",
                },
                "validation": {
                    "root": "validation",
                    "episode_count": len(validation_eps),
                    "frame_count": sum(len(item.rows) for item in validation_eps),
                    "episodes": _split_manifest(temp / "validation", validation_eps, config)[
                        "episodes"
                    ],
                    "files": validation_files,
                    "stats_source": "validation/meta/stats.json",
                },
            },
            "processors": {
                "root": "processors",
                "training_stats_only": True,
                "files": processor_files,
            },
            "source_facts_sha256": canonical_hash(
                [
                    {
                        "attempt_id": episode.attempt_id,
                        "seed": episode.seed,
                        "facts": {
                            key: value
                            for key, value in episode.source_facts.items()
                            if key != "facts_sha256"
                        },
                    }
                    for episode in parsed
                ]
            ),
            "source_code_sha256": _code_hashes(config.project_root),
        }
        (temp / "manifest.json").write_text(
            json.dumps(manifest, indent=2, allow_nan=False) + "\n", encoding="utf-8"
        )
        temp.replace(output)
        return manifest
    except Exception:
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
        raise


def _code_hashes(project_root: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for relative in (
        "learning/lerobot/src/dual_arm_learning/config.py",
        "learning/lerobot/src/dual_arm_learning/dataset.py",
        "learning/lerobot/src/dual_arm_learning/budget.py",
        "learning/lerobot/src/dual_arm_learning/training.py",
        "learning/lerobot/src/dual_arm_learning/bundle.py",
    ):
        path = project_root / relative
        if path.is_file():
            result[relative] = file_sha256(path)
    return result


__all__ = [
    "ExportError",
    "SourceEpisode",
    "canonical_hash",
    "convert",
    "file_sha256",
    "resolve_export_dir",
    "tree_hashes",
]
