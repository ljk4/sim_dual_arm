"""Auditable down-sampling of redundant static training anchors.

The filter only chooses which *query* frames enter the training sampler.  It
never rewrites a source Parquet, video, clock or 60-step future action.  The
result is one JSON mask that binds the source dataset manifest, Parquet hashes,
rule hash, phase/gate event hashes and every episode's original index list.
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from .config import (
    ConfigError,
    integer,
    load_yaml,
    mapping,
    number,
    reject_unknown,
    require,
    text,
)
from .dataset import canonical_hash, file_sha256

FILTER_FORMAT = "dual-arm-act-sample-filter-v1"
REASON_REDUNDANT = "redundant_static_anchor"
LEAD_NS = 50_000_000
PERIOD_NS = 50_000_000
STATE_DIM = 16
RIGHT_ARM = (8, 9, 10, 11, 12, 13, 14)
GRIPPERS = (7, 15)
KNOWN_PHASES = (
    "pregrasp",
    "grasp",
    "close",
    "lift",
    "transport",
    "lower",
    "open",
    "retreat",
    "stable",
)


class FilterError(ValueError):
    """The filter rule or mask cannot be trusted."""


@dataclass(frozen=True, slots=True)
class FilterRule:
    schema_version: int
    fps: int
    chunk_size: int
    right_arm_indices: tuple[int, ...]
    gripper_indices: tuple[int, ...]
    max_right_step_rad: float
    max_right_velocity_rad_s: float
    max_gripper_step_m: float
    max_gripper_velocity_m_s: float
    min_static_run_frames: int
    keep_edge_frames: int
    interior_anchor_stride: int
    phase_boundary_frames: int
    episode_boundary_frames: int
    min_episode_kept_frames: int
    known_phases: tuple[str, ...]
    tracking_gate: str
    reason: str
    source: Path

    @property
    def sha256(self) -> str:
        return file_sha256(self.source)


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


def _indices(value: object, location: str, *, count: int) -> tuple[int, ...]:
    if not isinstance(value, list):
        raise ConfigError(location, "must be an array of joint indices")
    result = tuple(
        integer(item, f"{location}[{index}]", 0, STATE_DIM - 1) for index, item in enumerate(value)
    )
    if len(result) != count:
        raise ConfigError(location, f"must contain exactly {count} joint indices")
    if len(set(result)) != len(result):
        raise ConfigError(location, "must be unique")
    return result


def load_filter_rule(path: str | Path) -> FilterRule:
    source = Path(path).resolve()
    value = mapping(load_yaml(source), "<sample_filter>")
    reject_unknown(
        value,
        {
            "schema_version",
            "fps",
            "chunk_size",
            "right_arm",
            "grippers",
            "static_runs",
            "phase_gate",
        },
        "<sample_filter>",
    )
    right = mapping(require(value, "right_arm", "<sample_filter>"), "right_arm")
    reject_unknown(right, {"joint_indices", "max_step_rad", "max_velocity_rad_s"}, "right_arm")
    grippers = mapping(require(value, "grippers", "<sample_filter>"), "grippers")
    reject_unknown(grippers, {"joint_indices", "max_step_m", "max_velocity_m_s"}, "grippers")
    runs = mapping(require(value, "static_runs", "<sample_filter>"), "static_runs")
    reject_unknown(
        runs,
        {
            "min_frames",
            "edge_frames",
            "interior_stride",
            "phase_boundary_frames",
            "episode_boundary_frames",
            "min_episode_kept_frames",
            "reason",
        },
        "static_runs",
    )
    phase = mapping(require(value, "phase_gate", "<sample_filter>"), "phase_gate")
    reject_unknown(phase, {"known_phases", "tracking_gate"}, "phase_gate")
    known = require(phase, "known_phases", "phase_gate")
    if not isinstance(known, list):
        raise ConfigError("phase_gate.known_phases", "must be an array")
    known_phases = tuple(
        text(item, f"phase_gate.known_phases[{i}]") for i, item in enumerate(known)
    )
    if len(set(known_phases)) != len(known_phases):
        raise ConfigError("phase_gate.known_phases", "must be unique")
    if set(KNOWN_PHASES) - set(known_phases):
        raise ConfigError("phase_gate.known_phases", "must contain every panda stack phase")
    right_indices = _indices(
        require(right, "joint_indices", "right_arm"), "right_arm.joint_indices", count=7
    )
    gripper_indices = _indices(
        require(grippers, "joint_indices", "grippers"), "grippers.joint_indices", count=2
    )
    if right_indices != RIGHT_ARM:
        raise ConfigError("right_arm.joint_indices", "must be the seven right arm rad axes")
    if gripper_indices != GRIPPERS:
        raise ConfigError("grippers.joint_indices", "must be the left/right gripper metre axes")
    reason = text(require(runs, "reason", "static_runs"), "static_runs.reason")
    if reason != REASON_REDUNDANT:
        raise ConfigError("static_runs.reason", f"must be {REASON_REDUNDANT}")
    rule = FilterRule(
        schema_version=integer(
            require(value, "schema_version", "<sample_filter>"), "schema_version", 1, 1
        ),
        fps=integer(require(value, "fps", "<sample_filter>"), "fps", 20, 20),
        chunk_size=integer(require(value, "chunk_size", "<sample_filter>"), "chunk_size", 60, 60),
        right_arm_indices=right_indices,
        gripper_indices=gripper_indices,
        max_right_step_rad=number(
            require(right, "max_step_rad", "right_arm"), "right_arm.max_step_rad", maximum=1.0
        ),
        max_right_velocity_rad_s=number(
            require(right, "max_velocity_rad_s", "right_arm"),
            "right_arm.max_velocity_rad_s",
            maximum=100.0,
        ),
        max_gripper_step_m=number(
            require(grippers, "max_step_m", "grippers"), "grippers.max_step_m", maximum=1.0
        ),
        max_gripper_velocity_m_s=number(
            require(grippers, "max_velocity_m_s", "grippers"),
            "grippers.max_velocity_m_s",
            maximum=100.0,
        ),
        min_static_run_frames=integer(
            require(runs, "min_frames", "static_runs"), "static_runs.min_frames", 2, 100_000
        ),
        keep_edge_frames=integer(
            require(runs, "edge_frames", "static_runs"), "static_runs.edge_frames", 0, 100_000
        ),
        interior_anchor_stride=integer(
            require(runs, "interior_stride", "static_runs"),
            "static_runs.interior_stride",
            1,
            100_000,
        ),
        phase_boundary_frames=integer(
            require(runs, "phase_boundary_frames", "static_runs"),
            "static_runs.phase_boundary_frames",
            0,
            100_000,
        ),
        episode_boundary_frames=integer(
            require(runs, "episode_boundary_frames", "static_runs"),
            "static_runs.episode_boundary_frames",
            0,
            100_000,
        ),
        min_episode_kept_frames=integer(
            require(runs, "min_episode_kept_frames", "static_runs"),
            "static_runs.min_episode_kept_frames",
            0,
            100_000,
        ),
        known_phases=known_phases,
        tracking_gate=text(
            require(phase, "tracking_gate", "phase_gate"), "phase_gate.tracking_gate"
        ),
        reason=reason,
        source=source,
    )
    if rule.keep_edge_frames * 2 >= rule.min_static_run_frames:
        raise ConfigError(
            "static_runs", "min_frames must exceed twice edge_frames so a run has an interior"
        )
    return rule


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise FilterError(f"{path}: cannot read JSON: {error}") from error
    if not isinstance(value, dict):
        raise FilterError(f"{path}: expected an object")
    return value


def _single_parquet(split_root: Path) -> tuple[Path, str]:
    candidates = sorted(split_root.glob("data/chunk-*/*.parquet"))
    if len(candidates) != 1:
        raise FilterError(
            f"{split_root}: expected exactly one chunk Parquet, got {len(candidates)}"
        )
    return candidates[0], candidates[0].relative_to(split_root).as_posix()


def _read_parquet(path: Path) -> dict[str, list[Any]]:
    import pyarrow.parquet as parquet

    try:
        table = parquet.read_table(path)
    except Exception as error:
        raise FilterError(f"{path}: cannot read Parquet: {error}") from error
    required = {
        "index",
        "episode_index",
        "source.episode_index",
        "source.seed",
        "observation.state",
        "observation.velocity",
        "action",
        "observation.sim_time_ns",
        "action.sim_time_ns",
    }
    missing = sorted(required - set(table.column_names))
    if missing:
        raise FilterError(f"{path}: missing Parquet columns {missing}")
    return table.to_pydict()


def _split(
    *,
    root: Path,
    manifest: dict[str, Any],
    select: dict[str, Any],
    name: str,
) -> dict[str, Any]:
    split = manifest.get("splits", {}).get(name)
    if not isinstance(split, dict):
        raise FilterError(f"manifest.splits.{name} must be an object")
    episodes = split.get("episodes")
    if not isinstance(episodes, list) or len(episodes) != int(split.get("episode_count", -1)):
        raise FilterError(f"manifest.splits.{name}.episodes disagrees with episode_count")
    split_root = root / str(split.get("root", ""))
    parquet_path, relative = _single_parquet(split_root)
    parquet_sha = file_sha256(parquet_path)
    declared = split.get("files", {}).get(relative)
    if not isinstance(declared, dict) or declared.get("sha256") != parquet_sha:
        raise FilterError(f"{parquet_path}: hash does not match the source manifest")
    rows = _read_parquet(parquet_path)
    if len(rows["index"]) != int(split.get("frame_count", -1)):
        raise FilterError(f"{parquet_path}: row count disagrees with the source manifest")
    indices = rows["index"]
    if any(isinstance(value, bool) or not isinstance(value, int) for value in indices):
        raise FilterError(f"{parquet_path}: original index must be an integer")
    if len(set(indices)) != len(indices):
        raise FilterError(f"{parquet_path}: duplicate original index")
    if any(value < 0 or value >= len(indices) for value in indices):
        raise FilterError(f"{parquet_path}: original index is out of range")
    got_episodes = {int(item) for item in rows["episode_index"]}
    if got_episodes != set(range(len(episodes))):
        raise FilterError(f"{parquet_path}: episode_index does not cover 0..{len(episodes) - 1}")
    declared_episodes = sorted(int(episode["dataset_episode_index"]) for episode in episodes)
    if declared_episodes != list(range(len(episodes))):
        raise FilterError(
            f"{name}: manifest dataset_episode_index values are not 0..{len(episodes) - 1}"
        )
    if sum(int(episode.get("frame_count", 0)) for episode in episodes) != int(split["frame_count"]):
        raise FilterError(f"{name}: manifest episode frame counts disagree with the split")
    entries = select.get(name)
    if not isinstance(entries, list) or len(entries) != len(episodes):
        raise FilterError(f"{name}: select split does not contain {len(episodes)} attempts")
    selected: dict[str, dict[str, Any]] = {}
    for entry in entries:
        attempt_id = text(entry.get("attempt_id"), f"{name}.attempt_id")
        if attempt_id in selected:
            raise FilterError(f"{name}: duplicate attempt_id {attempt_id}")
        attempt_dir = Path(text(entry.get("attempt_dir"), f"{name}.attempt_dir"))
        if not attempt_dir.is_dir():
            raise FilterError(f"{attempt_dir}: selected attempt directory is missing")
        selected[attempt_id] = {
            "seed": integer(entry.get("seed"), f"{name}.seed", 0, 2**63 - 1),
            "attempt_dir": attempt_dir.resolve(),
        }
    manifest_ids = {
        text(episode.get("attempt_id"), f"{name}.episodes.attempt_id") for episode in episodes
    }
    if set(selected) != manifest_ids:
        raise FilterError(f"{name}: select and manifest attempt ids disagree")
    return {
        "split": split,
        "root": split_root,
        "parquet_path": parquet_path,
        "parquet_relative": relative,
        "parquet_sha256": parquet_sha,
        "rows": rows,
        "episodes": episodes,
        "selected": selected,
    }


def _source_paths(attempt_dir: Path) -> tuple[Path, list[Path]]:
    labels = sorted(attempt_dir.glob("stack/*/export/labels.json"))
    if len(labels) != 1:
        raise FilterError(
            f"{attempt_dir}: expected exactly one export labels.json, got {len(labels)}"
        )
    events = sorted(attempt_dir.glob("stack/*/runs/*/logs/runtime/*/events.*.jsonl"))
    return labels[0], events


def _phase_gate_map(paths: list[Path]) -> dict[int, tuple[str | None, str | None]]:
    result: dict[int, tuple[str | None, str | None]] = {}
    for path in paths:
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except OSError as error:
            raise FilterError(f"{path}: cannot read runtime events: {error}") from error
        for line_number, line in enumerate(lines, 1):
            if not line.strip():
                continue
            try:
                event = json.loads(line)
            except json.JSONDecodeError as error:
                raise FilterError(f"{path}:{line_number}: invalid JSON: {error}") from error
            if (
                not isinstance(event, dict)
                or event.get("event") != "native.follower.chunk_accepted"
            ):
                continue
            fields = event.get("fields")
            if not isinstance(fields, dict):
                raise FilterError(f"{path}:{line_number}: chunk_accepted fields are missing")
            chunk_seq = fields.get("chunk_seq")
            if isinstance(chunk_seq, bool) or not isinstance(chunk_seq, int):
                raise FilterError(f"{path}:{line_number}: chunk_seq must be an integer")
            phase = fields.get("phase")
            gate = fields.get("gate_state")
            value = (
                phase if isinstance(phase, str) and phase else None,
                gate if isinstance(gate, str) and gate else None,
            )
            old = result.get(chunk_seq)
            if old is not None and old != value:
                raise FilterError(
                    f"{path}:{line_number}: ambiguous phase/gate for chunk {chunk_seq}"
                )
            result[chunk_seq] = value
    return result


def _arrays(rows: dict[str, list[Any]], positions: list[int]) -> dict[str, np.ndarray]:
    return {
        "measured": np.asarray([rows["observation.state"][i] for i in positions], dtype=np.float64),
        "applied": np.asarray([rows["action"][i] for i in positions], dtype=np.float64),
        "measured_velocity": np.asarray(
            [rows["observation.velocity"][i] for i in positions], dtype=np.float64
        ),
        "observation_time_ns": np.asarray(
            [rows["observation.sim_time_ns"][i] for i in positions], dtype=np.int64
        ),
        "action_time_ns": np.asarray(
            [rows["action.sim_time_ns"][i] for i in positions], dtype=np.int64
        ),
    }


def _aligned_labels(
    labels: list[dict[str, Any]],
    arrays: dict[str, np.ndarray],
    *,
    lead_ns: int,
) -> list[dict[str, Any]]:
    by_time: dict[int, dict[str, Any]] = {}
    for label in labels:
        if not isinstance(label, dict):
            raise FilterError("labels.json contains a non-object label")
        moment = label.get("observation_sim_time_ns")
        if isinstance(moment, bool) or not isinstance(moment, int):
            raise FilterError("label observation_sim_time_ns must be an integer")
        if moment in by_time:
            raise FilterError(f"duplicate label observation time {moment}")
        by_time[moment] = label
    wanted = [int(item) for item in arrays["observation_time_ns"]]
    if len(set(wanted)) != len(wanted):
        raise FilterError("Parquet observation clock contains duplicate times")
    if set(wanted) != set(by_time):
        raise FilterError("labels.json observation times do not match the Parquet rows")
    ordered = [by_time[item] for item in wanted]
    if len(ordered) < 2:
        raise FilterError("an episode needs at least two labelled frames")
    if np.any(np.diff(arrays["observation_time_ns"]) != PERIOD_NS):
        raise FilterError("Parquet observation clock is not a uniform 20 Hz")
    if np.any(arrays["action_time_ns"] - arrays["observation_time_ns"] != lead_ns):
        raise FilterError("Parquet action clock does not lead observations by the configured 50 ms")
    for index, label in enumerate(ordered):
        if (
            int(label.get("label_sim_time_ns", -1)) - int(label.get("observation_sim_time_ns", -1))
            != lead_ns
        ):
            raise FilterError(f"label {index} does not lead observation by 50 ms")
        if int(label.get("observation_period_ns", -1)) != PERIOD_NS:
            raise FilterError(f"label {index} is not from a uniform 20 Hz timeline")
    return ordered


def _verify_episode_values(
    ordered: list[dict[str, Any]],
    arrays: dict[str, np.ndarray],
    *,
    state_dim: int,
) -> None:
    expected = {
        "measured_positions": arrays["measured"],
        "applied_positions": arrays["applied"],
        "measured_velocities": arrays["measured_velocity"],
    }
    for name, target in expected.items():
        values = [label.get(name) for label in ordered]
        if any(not isinstance(item, list) or len(item) != state_dim for item in values):
            raise FilterError(f"{name} must contain {state_dim}-joint arrays")
        actual = np.asarray(values, dtype=np.float64)
        if not np.allclose(actual, target, rtol=0.0, atol=1e-6):
            raise FilterError(f"labels.json {name} do not match the Parquet source")
    applied_velocity = np.asarray(
        [label.get("applied_velocities") for label in ordered], dtype=np.float64
    )
    if applied_velocity.shape != arrays["applied"].shape:
        raise FilterError("applied_velocities do not contain 16-joint arrays")


def _static_edges(
    measured: np.ndarray,
    applied: np.ndarray,
    measured_velocity: np.ndarray,
    applied_velocity: np.ndarray,
    rule: FilterRule,
) -> np.ndarray:
    right, grip = list(rule.right_arm_indices), list(rule.gripper_indices)
    measured_step = np.abs(np.diff(measured[:, right], axis=0))
    applied_step = np.abs(np.diff(applied[:, right], axis=0))
    grip_measured_step = np.abs(np.diff(measured[:, grip], axis=0))
    grip_applied_step = np.abs(np.diff(applied[:, grip], axis=0))
    measured_speed = np.maximum(
        np.abs(measured_velocity[:-1, right]), np.abs(measured_velocity[1:, right])
    )
    applied_speed = np.maximum(
        np.abs(applied_velocity[:-1, right]), np.abs(applied_velocity[1:, right])
    )
    grip_measured_speed = np.maximum(
        np.abs(measured_velocity[:-1, grip]), np.abs(measured_velocity[1:, grip])
    )
    grip_applied_speed = np.maximum(
        np.abs(applied_velocity[:-1, grip]), np.abs(applied_velocity[1:, grip])
    )
    return (
        (measured_step <= rule.max_right_step_rad).all(axis=1)
        & (applied_step <= rule.max_right_step_rad).all(axis=1)
        & (grip_measured_step <= rule.max_gripper_step_m).all(axis=1)
        & (grip_applied_step <= rule.max_gripper_step_m).all(axis=1)
        & (measured_speed <= rule.max_right_velocity_rad_s).all(axis=1)
        & (applied_speed <= rule.max_right_velocity_rad_s).all(axis=1)
        & (grip_measured_speed <= rule.max_gripper_velocity_m_s).all(axis=1)
        & (grip_applied_speed <= rule.max_gripper_velocity_m_s).all(axis=1)
    )


def _static_runs(edges: np.ndarray) -> list[tuple[int, int]]:
    runs: list[tuple[int, int]] = []
    index = 0
    while index < len(edges):
        if not edges[index]:
            index += 1
            continue
        start = index
        while index < len(edges) and edges[index]:
            index += 1
        runs.append((start, index))
    return runs


def _protections(phase: list[str | None], gate: list[str | None], rule: FilterRule) -> np.ndarray:
    count = len(phase)
    protected = np.zeros(count, dtype=bool)
    edge = rule.episode_boundary_frames
    if edge:
        protected[:edge] = True
        protected[max(0, count - edge) :] = True
    window = rule.phase_boundary_frames
    if window:
        for index in range(1, count):
            if phase[index] != phase[index - 1]:
                protected[max(0, index - window) : min(count, index + window + 1)] = True
    for index, (phase_name, gate_name) in enumerate(zip(phase, gate, strict=True)):
        if phase_name not in rule.known_phases or gate_name != rule.tracking_gate:
            protected[index] = True
    return protected


def _choose_anchors(
    edges: np.ndarray,
    phase: list[str | None],
    gate: list[str | None],
    rule: FilterRule,
) -> tuple[np.ndarray, list[str], list[tuple[int, int]]]:
    count = len(edges) + 1
    keep = np.ones(count, dtype=bool)
    reasons = [""] * count
    runs = _static_runs(edges)
    for start, end in runs:
        length = end - start + 1
        if length < rule.min_static_run_frames:
            continue
        interior_start = start + rule.keep_edge_frames
        interior_end = end - rule.keep_edge_frames
        for index in range(interior_start, interior_end + 1):
            if (index - interior_start) % rule.interior_anchor_stride:
                keep[index] = False
                reasons[index] = rule.reason
    keep |= _protections(phase, gate, rule)
    for index, saved in enumerate(keep):
        if saved:
            reasons[index] = ""
    return keep, reasons, runs


def _counts(values: list[str | None], keys: list[str]) -> dict[str, int]:
    result = {key: 0 for key in keys}
    result["unknown"] = 0
    for value in values:
        result[value if value in result else "unknown"] += 1
    return result


def _filter_episode(
    *,
    episode: dict[str, Any],
    split: dict[str, Any],
    selected: dict[str, Any],
    lead_ns: int,
    rule: FilterRule,
) -> dict[str, Any]:
    episode_index = int(episode["dataset_episode_index"])
    global_index = int(episode["global_success_index"])
    attempt_id = text(episode.get("attempt_id"), "episode.attempt_id")
    seed = integer(episode.get("seed"), "episode.seed", 0, 2**63 - 1)
    selection = selected[attempt_id]
    if selection["seed"] != seed:
        raise FilterError(f"{attempt_id}: select seed disagrees with the manifest")
    attempt_dir = selection["attempt_dir"]
    labels_path, event_paths = _source_paths(attempt_dir)
    labels_document = _read_json(labels_path)
    if labels_document.get("seed") != seed:
        raise FilterError(f"{labels_path}: label seed disagrees with the manifest")
    if labels_document.get("episode_id") != attempt_id.replace(":", "-"):
        raise FilterError(f"{labels_path}: label episode_id disagrees with the attempt")
    profile_hash = labels_document.get("robot_profile_hash")
    if profile_hash != split["manifest_profile_sha256"]:
        raise FilterError(f"{labels_path}: robot profile hash disagrees with the manifest")
    labels = labels_document.get("labels")
    if not isinstance(labels, list) or len(labels) != int(episode.get("frame_count", -1)):
        raise FilterError(f"{labels_path}: label count disagrees with the manifest episode")
    rows = split["rows"]
    positions = [
        index for index, value in enumerate(rows["episode_index"]) if int(value) == episode_index
    ]
    if not positions:
        raise FilterError(f"episode {episode_index}: no Parquet rows")
    positions.sort(key=lambda index: int(rows["index"][index]))
    source_indices = [int(rows["index"][index]) for index in positions]
    if any(
        int(rows["source.episode_index"][index]) != global_index
        or int(rows["source.seed"][index]) != seed
        for index in positions
    ):
        raise FilterError(f"episode {episode_index}: source episode/seed differs from the manifest")
    arrays = _arrays(rows, positions)
    ordered = _aligned_labels(labels, arrays, lead_ns=lead_ns)
    _verify_episode_values(ordered, arrays, state_dim=STATE_DIM)
    events = _phase_gate_map(event_paths)
    phase: list[str | None] = []
    gate: list[str | None] = []
    for label in ordered:
        chunk_seq = label.get("raw_chunk_seq")
        mapped = events.get(chunk_seq) if isinstance(chunk_seq, int) else None
        phase.append(None if mapped is None else mapped[0])
        gate.append(None if mapped is None else mapped[1])
    edges = _static_edges(
        arrays["measured"],
        arrays["applied"],
        arrays["measured_velocity"],
        np.asarray([label.get("applied_velocities") for label in ordered], dtype=np.float64),
        rule,
    )
    keep, reasons, runs = _choose_anchors(edges, phase, gate, rule)
    kept_count = int(keep.sum())
    fallback = None
    if kept_count < rule.min_episode_kept_frames:
        fallback = "minimum_kept_frames"
        keep[:] = True
        reasons = [""] * len(reasons)
        kept_count = len(keep)
    kept_indices = [index for index, saved in zip(source_indices, keep, strict=True) if saved]
    excluded_indices = [
        index for index, saved in zip(source_indices, keep, strict=True) if not saved
    ]
    kept_flags = [index for index, saved in enumerate(keep) if saved]
    excluded_flags = [index for index, saved in enumerate(keep) if not saved]
    long_runs = [run for run in runs if run[1] - run[0] + 1 >= rule.min_static_run_frames]
    phase_keys = list(rule.known_phases)
    gate_keys = [rule.tracking_gate, "WAITING_FOR_FEEDBACK", "HOLDING"]
    return {
        "dataset_episode_index": episode_index,
        "global_success_index": global_index,
        "attempt_id": attempt_id,
        "seed": seed,
        "source_frame_count": len(source_indices),
        "kept_count": kept_count,
        "excluded_count": len(excluded_indices),
        "source_indices": source_indices,
        "kept_indices": kept_indices,
        "excluded_indices": excluded_indices,
        "excluded_reasons": {REASON_REDUNDANT: len(excluded_indices)} if excluded_indices else {},
        "phase_counts": {
            "source": _counts(phase, phase_keys),
            "kept": _counts([phase[i] for i in kept_flags], phase_keys),
            "excluded": _counts([phase[i] for i in excluded_flags], phase_keys),
        },
        "gate_counts": {
            "source": _counts(gate, gate_keys),
            "kept": _counts([gate[i] for i in kept_flags], gate_keys),
            "excluded": _counts([gate[i] for i in excluded_flags], gate_keys),
        },
        "static_runs": {
            "total": len(runs),
            "long": len(long_runs),
            "frames_in_long_runs": sum(end - start + 1 for start, end in long_runs),
        },
        "fallback": fallback,
        "labels": {"path": str(labels_path), "sha256": file_sha256(labels_path)},
        "phase_gate_sources": [
            {"path": str(path), "sha256": file_sha256(path)} for path in event_paths
        ],
    }


def _total_counts(
    episodes: list[dict[str, Any]], section: str, keys: list[str]
) -> dict[str, dict[str, int]]:
    buckets = ("source", "kept", "excluded")
    result = {bucket: {key: 0 for key in [*keys, "unknown"]} for bucket in buckets}
    for episode in episodes:
        counts = episode[f"{section}_counts"]
        for bucket in buckets:
            for key, value in counts[bucket].items():
                result[bucket][key] += value
    return result


def _mask_digest(document: dict[str, Any]) -> str:
    payload = {
        key: value
        for key, value in document.items()
        if key not in {"filter_digest", "created_wall_time_ns"}
    }
    return canonical_hash(payload)


def build_filter_manifest(
    *, rule_path: str | Path, dataset_root: str | Path, select_path: str | Path
) -> dict[str, Any]:
    root = Path(dataset_root).resolve()
    rule = load_filter_rule(rule_path)
    manifest_path = root / "manifest.json"
    manifest = _read_json(manifest_path)
    if manifest.get("status") != "complete" or manifest.get("synthetic"):
        raise FilterError("source dataset manifest is not a complete formal dataset")
    select_source = Path(select_path).resolve()
    select = _read_json(select_source)
    if select.get("status") != "SELECTED":
        raise FilterError("formal select file is not SELECTED")
    lead_ns = int(manifest.get("lerobot", {}).get("action_lead_ns", -1))
    if lead_ns != LEAD_NS:
        raise FilterError("source dataset does not use the fixed 50 ms action lead")
    profile_hash = manifest.get("profile", {}).get("semantic_sha256")
    if not isinstance(profile_hash, str):
        raise FilterError("source manifest has no profile semantic hash")
    training = _split(root=root, manifest=manifest, select=select, name="training")
    validation = _split(root=root, manifest=manifest, select=select, name="validation")
    train_globals = {int(episode["global_success_index"]) for episode in training["episodes"]}
    validation_globals = {
        int(episode["global_success_index"]) for episode in validation["episodes"]
    }
    if train_globals & validation_globals:
        raise FilterError("training and validation episodes overlap")
    training["manifest_profile_sha256"] = profile_hash
    episodes = [
        _filter_episode(
            episode=episode,
            split=training,
            selected=training["selected"],
            lead_ns=lead_ns,
            rule=rule,
        )
        for episode in training["episodes"]
    ]
    source_frames = sum(episode["source_frame_count"] for episode in episodes)
    if source_frames != int(training["split"]["frame_count"]):
        raise FilterError("filtered episodes do not cover the training Parquet frame count")
    eligible = sum(episode["kept_count"] for episode in episodes)
    excluded = source_frames - eligible
    phase_keys = list(rule.known_phases)
    gate_keys = [rule.tracking_gate, "WAITING_FOR_FEEDBACK", "HOLDING"]
    document: dict[str, Any] = {
        "format": FILTER_FORMAT,
        "created_wall_time_ns": time.time_ns(),
        "dataset": {
            "root": str(root),
            "manifest": {"path": str(manifest_path), "sha256": file_sha256(manifest_path)},
            "select": {"path": str(select_source), "sha256": file_sha256(select_source)},
        },
        "rule": {
            "path": str(rule.source),
            "sha256": rule.sha256,
            "resolved": {
                "schema_version": rule.schema_version,
                "fps": rule.fps,
                "chunk_size": rule.chunk_size,
                "right_arm_indices": list(rule.right_arm_indices),
                "gripper_indices": list(rule.gripper_indices),
                "max_right_step_rad": rule.max_right_step_rad,
                "max_right_velocity_rad_s": rule.max_right_velocity_rad_s,
                "max_gripper_step_m": rule.max_gripper_step_m,
                "max_gripper_velocity_m_s": rule.max_gripper_velocity_m_s,
                "min_static_run_frames": rule.min_static_run_frames,
                "keep_edge_frames": rule.keep_edge_frames,
                "interior_anchor_stride": rule.interior_anchor_stride,
                "phase_boundary_frames": rule.phase_boundary_frames,
                "episode_boundary_frames": rule.episode_boundary_frames,
                "min_episode_kept_frames": rule.min_episode_kept_frames,
                "known_phases": list(rule.known_phases),
                "tracking_gate": rule.tracking_gate,
                "reason": rule.reason,
            },
        },
        "training": {
            "source_frames": source_frames,
            "eligible_frames": eligible,
            "excluded_frames": excluded,
            "reduction_ratio": excluded / source_frames,
            "parquet": {
                "path": str(training["parquet_path"]),
                "sha256": training["parquet_sha256"],
            },
            "phase_counts": _total_counts(episodes, "phase", phase_keys),
            "gate_counts": _total_counts(episodes, "gate", gate_keys),
            "fallback_episodes": [
                episode["dataset_episode_index"]
                for episode in episodes
                if episode["fallback"] is not None
            ],
            "episodes": episodes,
        },
        "validation": {
            "filtered": False,
            "source_frames": int(validation["split"]["frame_count"]),
            "kept_frames": int(validation["split"]["frame_count"]),
            "excluded_frames": 0,
            "episode_count": len(validation["episodes"]),
            "attempt_ids": [episode["attempt_id"] for episode in validation["episodes"]],
            "seeds": [int(episode["seed"]) for episode in validation["episodes"]],
            "parquet": {
                "path": str(validation["parquet_path"]),
                "sha256": validation["parquet_sha256"],
            },
        },
    }
    document["filter_digest"] = _mask_digest(document)
    return document


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


def load_filter_mask(path: str | Path, *, verify_source: bool = True) -> SampleFilterMask:
    source = Path(path).resolve()
    document = _read_json(source)
    if document.get("format") != FILTER_FORMAT:
        raise FilterError(f"{source}: unknown sample filter format")
    if document.get("filter_digest") != _mask_digest(document):
        raise FilterError(f"{source}: filter digest does not match its contents")
    training = document.get("training")
    validation = document.get("validation")
    if not isinstance(training, dict) or not isinstance(validation, dict):
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
    episodes = training.get("episodes")
    if not isinstance(episodes, list) or not episodes:
        raise FilterError(f"{source}: training episodes are missing")
    seen: set[int] = set()
    eligible: list[int] = []
    source_total = 0
    for episode in episodes:
        if not isinstance(episode, dict):
            raise FilterError(f"{source}: episode entry is not an object")
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
        for value in source_indices:
            if value in seen:
                raise FilterError(f"{source}: duplicate original index {value}")
            seen.add(value)
        eligible.extend(kept_indices)
        source_total += len(source_indices)
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
    "FilterError",
    "FilterRule",
    "SampleFilterMask",
    "build_filter_manifest",
    "load_filter_mask",
    "load_filter_rule",
    "write_filter_manifest",
]
