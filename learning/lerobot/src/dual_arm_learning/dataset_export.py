"""Validation of one accepted flat export before any dataset frame is written."""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image
from robot_contracts.project_paths import project_path

from .config import PNG_SHA, DatasetConfig
from .dataset_common import (
    ExportError,
    canonical_hash,
    file_sha256,
    load_json_document,
    require_chunk,
    require_int,
    require_vector,
    resolve_export_dir,
)


@dataclass(frozen=True, slots=True)
class SourceEpisode:
    attempt_id: str
    seed: int
    attempt_dir: Path
    export_dir: Path
    global_index: int
    rows: tuple[dict[str, Any], ...]
    raw_by_chunk: dict[int, dict[str, Any]]
    source_files: dict[str, dict[str, Any]]
    quality: dict[str, Any]
    profile_hash: str
    source_facts: dict[str, Any]


def _profile_document(
    config: DatasetConfig,
) -> tuple[dict[str, Any], tuple[str, ...], tuple[str, ...]]:
    doc = load_json_document(config.profile_path)
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


def _select_entries(select: dict[str, Any], config: DatasetConfig) -> list[dict[str, Any]]:
    if select.get("status") != "SELECTED":
        raise ExportError(f"select: expected status SELECTED, got {select.get('status')!r}")
    qualified = require_int(select.get("qualified"), "select.qualified")
    if qualified != config.expected_formal_episodes:
        raise ExportError(
            f"select.qualified: expected {config.expected_formal_episodes}, got {qualified}"
        )
    selected = select.get("selected")
    if not isinstance(selected, list):
        raise ExportError("select.selected: expected an array")
    if len(selected) != qualified:
        raise ExportError(f"select.selected: expected {qualified} entries, got {len(selected)}")
    all_ids: set[str] = set()
    all_seeds: set[int] = set()
    parsed = []
    for index, raw in enumerate(selected):
        field = f"select.selected[{index}]"
        if not isinstance(raw, dict):
            raise ExportError(f"{field}: expected an object")
        attempt_id = raw.get("attempt_id")
        attempt_dir = raw.get("attempt_dir")
        seed = require_int(raw.get("seed"), f"{field}.seed")
        if not isinstance(attempt_id, str) or not attempt_id:
            raise ExportError(f"{field}.attempt_id: expected a non-empty string")
        if not isinstance(attempt_dir, str) or not attempt_dir:
            raise ExportError(f"{field}.attempt_dir: expected a non-empty string")
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
                "attempt_dir": project_path(attempt_dir, config.project_root),
                "global_index": index,
            }
        )
    return parsed


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


def _validate_row_clock(row: dict[str, Any], location: str, config: DatasetConfig) -> int:
    observation_time = require_int(
        row.get("observation_sim_time_ns"), f"{location}.observation_sim_time_ns"
    )
    label_time = require_int(row.get("label_sim_time_ns"), f"{location}.label_sim_time_ns")
    if label_time != observation_time + config.action_lead_ns:
        raise ExportError(f"{location}: label time is not observation + 50 ms")
    period = require_int(row.get("observation_period_ns"), f"{location}.observation_period_ns")
    if period != config.action_lead_ns:
        raise ExportError(f"{location}: observation period is not 50 ms")
    started = require_int(row.get("started_sim_time_ns"), f"{location}.started_sim_time_ns")
    block_end = require_int(row.get("block_end_sim_time_ns"), f"{location}.block_end_sim_time_ns")
    if not started <= label_time <= block_end:
        raise ExportError(f"{location}: label is outside its executed block")
    return observation_time


def _validate_row_vectors(
    row: dict[str, Any], location: str, config: DatasetConfig
) -> dict[str, np.ndarray]:
    return {
        "applied": require_vector(
            row.get("applied_positions"), f"{location}.applied_positions", config.state_dim
        ),
        "measured": require_vector(
            row.get("measured_positions"), f"{location}.measured_positions", config.state_dim
        ),
        "velocity": require_vector(
            row.get("measured_velocities"), f"{location}.measured_velocities", config.state_dim
        ),
    }


def _validate_camera_record(
    entry: dict[str, Any],
    camera: str,
    location: str,
    observation_time: int,
    row: dict[str, Any],
) -> None:
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
    for field in ("png_sha256", "rgb_sha256"):
        if not isinstance(entry[field], str) or PNG_SHA.fullmatch(entry[field]) is None:
            raise ExportError(f"{location}.rgb.{camera}.{field}: bad digest")


def _validate_row_rgb(
    row: dict[str, Any],
    location: str,
    observation_time: int,
    config: DatasetConfig,
    export_dir: Path,
) -> None:
    rgb = row.get("rgb")
    if not isinstance(rgb, dict) or tuple(sorted(rgb)) != tuple(sorted(config.camera_names)):
        raise ExportError(f"{location}.rgb: expected exactly the three learning cameras")
    for camera in config.camera_names:
        entry = rgb[camera]
        if not isinstance(entry, dict):
            raise ExportError(f"{location}.rgb.{camera}: expected an object")
        _validate_camera_record(entry, camera, location, observation_time, row)
        relative = Path(str(entry["path"]))
        if relative.is_absolute() or ".." in relative.parts:
            raise ExportError(f"{location}.rgb.{camera}.path: must stay inside the export")
        png_path = export_dir / relative
        if not png_path.resolve().is_relative_to(export_dir.resolve()):
            raise ExportError(f"{location}.rgb.{camera}.path: escapes the export")
        _validate_png(png_path, entry, config)


def _validate_row(
    row: object,
    index: int,
    config: DatasetConfig,
    export_dir: Path,
) -> tuple[dict[str, Any], dict[str, np.ndarray], np.ndarray]:
    if not isinstance(row, dict):
        raise ExportError(f"labels[{index}]: expected an object")
    location = f"labels[{index}]"
    observation_time = _validate_row_clock(row, location, config)
    vectors = _validate_row_vectors(row, location, config)
    _validate_row_rgb(row, location, observation_time, config, export_dir)
    return row, vectors, np.asarray([row["raw_chunk_seq"]], dtype=np.int64)


def _source_facts(labels: dict[str, Any], quality: dict[str, Any]) -> dict[str, Any]:
    task = labels.get("task")
    runtime = labels.get("runtime")
    recording = labels.get("recording")
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
    facts = {
        "episode_id": labels.get("episode_id"),
        "task": {"outcome": task.get("outcome"), "reason": task.get("reason")},
        "runtime": {"outcome": runtime.get("outcome"), "reason": runtime.get("reason")},
        "recording": {
            "complete": recording.get("complete"),
            "failure_reason": recording.get("failure_reason", ""),
        },
        "quality_qualified": quality.get("qualified"),
    }
    facts["facts_sha256"] = canonical_hash(facts)
    return facts


_EXPORT_NAMES = (
    "labels.json",
    "raw_predictions.json",
    "quality.json",
)


def _source_files(export_dir: Path) -> dict[str, dict[str, Any]]:
    source_files: dict[str, dict[str, Any]] = {}
    for name in _EXPORT_NAMES:
        path = export_dir / name
        if not path.is_file():
            raise ExportError(f"{export_dir}: missing {name}")
        source_files[name] = {
            "path": str(path.resolve()),
            "sha256": file_sha256(path),
            "bytes": path.stat().st_size,
        }
    return source_files


def _export_documents(export_dir: Path) -> tuple[dict[str, Any], ...]:
    return tuple(load_json_document(export_dir / name) for name in _EXPORT_NAMES)


def _validate_export_headers(
    labels: dict[str, Any],
    quality: dict[str, Any],
    config: DatasetConfig,
    profile_hash: str,
) -> None:
    if labels.get("robot_profile_hash") != profile_hash:
        raise ExportError("export: robot profile hash does not match the learning profile")
    if quality.get("qualified") is not True:
        raise ExportError("export: quality sidecar is not qualified")
    if config.require_quality and (
        quality.get("visibility_qualified") is False or quality.get("trajectory_qualified") is False
    ):
        raise ExportError("export: visibility/trajectory qualification is false")
    camera_order = labels.get("camera_order")
    if tuple(camera_order or ()) != config.camera_names:
        raise ExportError("export: labels camera order does not match the profile")


def _validate_raw_predictions(
    raw_doc: dict[str, Any], config: DatasetConfig
) -> dict[int, dict[str, Any]]:
    raw_value = raw_doc.get("predictions")
    if not isinstance(raw_value, list) or not raw_value:
        raise ExportError("export: raw_predictions array is empty")
    expected_times = [(offset + 1) * config.action_lead_ns for offset in range(config.chunk_size)]
    raw_by_chunk: dict[int, dict[str, Any]] = {}
    for index, prediction in enumerate(raw_value):
        if not isinstance(prediction, dict):
            raise ExportError(f"raw_predictions[{index}]: expected an object")
        chunk_seq = require_int(prediction.get("chunk_seq"), f"raw_predictions[{index}].chunk_seq")
        if chunk_seq in raw_by_chunk:
            raise ExportError(f"raw_predictions: duplicate chunk_seq {chunk_seq}")
        times = prediction.get("times_from_observation_ns")
        if not isinstance(times, list) or len(times) != config.chunk_size:
            raise ExportError(f"raw_predictions[{index}]: expected 60 future points")
        if [
            require_int(value, f"raw_predictions[{index}].times[{offset}]")
            for offset, value in enumerate(times)
        ] != expected_times:
            raise ExportError(f"raw_predictions[{index}]: future times are not 1..60 x 50 ms")
        raw_by_chunk[chunk_seq] = {
            "chunk_seq": chunk_seq,
            "command_id": str(prediction.get("command_id", "")),
            "source_observation_id": require_int(
                prediction.get("source_observation_id"),
                f"raw_predictions[{index}].source_observation_id",
            ),
            "source_observation_time_ns": require_int(
                prediction.get("source_observation_time_ns"),
                f"raw_predictions[{index}].source_observation_time_ns",
            ),
            "times_from_observation_ns": tuple(expected_times),
            "positions": require_chunk(
                prediction.get("positions"),
                f"raw_predictions[{index}].positions",
                config.chunk_size,
                config.state_dim,
            ),
        }
    return raw_by_chunk


def _validate_label_rows(
    rows_value: list[Any],
    raw_by_chunk: dict[int, dict[str, Any]],
    config: DatasetConfig,
    export_dir: Path,
) -> tuple[dict[str, Any], ...]:
    rows: list[dict[str, Any]] = []
    previous_time: int | None = None
    for index, row in enumerate(rows_value):
        _, vectors, _ = _validate_row(row, index, config, export_dir)
        observation_time = require_int(
            row.get("observation_sim_time_ns"), f"labels[{index}].observation_sim_time_ns"
        )
        if previous_time is not None and observation_time - previous_time != config.action_lead_ns:
            raise ExportError(f"labels[{index}]: observation clocks are not contiguous 20 Hz")
        previous_time = observation_time
        raw_chunk = require_int(row.get("raw_chunk_seq"), f"labels[{index}].raw_chunk_seq")
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
    return tuple(rows)


def _validate_export(
    export_dir: Path,
    config: DatasetConfig,
    profile_hash: str,
) -> tuple[
    tuple[dict[str, Any], ...],
    dict[int, dict[str, Any]],
    dict[str, Any],
]:
    labels, raw_doc, quality = _export_documents(export_dir)
    _validate_export_headers(labels, quality, config, profile_hash)
    _source_facts(labels, quality)
    rows_value = labels.get("labels")
    if not isinstance(rows_value, list) or not rows_value:
        raise ExportError("export: labels array is empty")
    summary_labels = labels.get("summary")
    if not isinstance(summary_labels, dict) or summary_labels.get("labels") != len(rows_value):
        raise ExportError("export: label summary count is inconsistent")
    raw_by_chunk = _validate_raw_predictions(raw_doc, config)
    rows = _validate_label_rows(rows_value, raw_by_chunk, config, export_dir)
    return rows, raw_by_chunk, quality


def _predict_episode(
    entry: dict[str, Any], config: DatasetConfig, profile_hash: str
) -> SourceEpisode:
    export_dir = resolve_export_dir(entry["attempt_dir"], config.attempt_export_glob)
    rows, raw_by_chunk, quality = _validate_export(export_dir, config, profile_hash)
    labels = load_json_document(export_dir / "labels.json")
    if labels.get("seed") != entry["seed"]:
        raise ExportError(
            f"{export_dir}: labels seed {labels.get('seed')} != selected seed {entry['seed']}"
        )
    if config.require_quality and quality.get("qualified") is not True:
        raise ExportError(f"{export_dir}: export is not quality qualified")
    action_source = str(labels.get("action_source", ""))
    if action_source != "teacher":
        raise ExportError(
            f"{export_dir}: formal data must come from teacher, not {action_source!r}"
        )
    facts = _source_facts(labels, quality)
    return SourceEpisode(
        attempt_id=entry["attempt_id"],
        seed=entry["seed"],
        attempt_dir=entry["attempt_dir"],
        export_dir=export_dir,
        global_index=entry["global_index"],
        rows=rows,
        raw_by_chunk=raw_by_chunk,
        source_files=_source_files(export_dir),
        quality=quality,
        profile_hash=profile_hash,
        source_facts=facts,
    )


__all__ = [
    "SourceEpisode",
    "_predict_episode",
    "_profile_document",
    "_select_entries",
    "_validate_export",
    "_validate_row",
]
