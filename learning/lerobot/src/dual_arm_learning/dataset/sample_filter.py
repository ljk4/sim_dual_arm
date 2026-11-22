"""Auditable down-sampling of redundant static training anchors.

The filter only chooses which *query* frames enter the training sampler.  It
never rewrites a source Parquet, video, clock or future action chunk.  The
result is an in-memory mask that binds the self-contained dataset manifest,
Parquet shard hashes, rule hash and every episode's original index list.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import numpy as np

from .common import file_sha256
from .sample_filter_mask import (  # noqa: F401 - public re-exports
    SampleFilterMask,
    load_filter_mask,
    mask_digest,
    mask_from_document,
    read_filter_json,
)
from .sample_filter_rules import (  # noqa: F401 - public re-exports
    ARM_INDICES,
    GRIPPERS,
    LEAD_NS,
    PERIOD_NS,
    REASON_REDUNDANT,
    STATE_DIM,
    FilterError,
    FilterRule,
    load_filter_rule,
)


def _text(value: object, location: str) -> str:
    if not isinstance(value, str) or not value:
        raise FilterError(f"{location}: must be a non-empty string")
    return value


def _integer(value: object, location: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise FilterError(f"{location}: must be an integer")
    if not minimum <= value <= maximum:
        raise FilterError(f"{location}: must be within [{minimum}, {maximum}]")
    return value


def _split_parquets(split_root: Path) -> list[tuple[Path, str]]:
    candidates = sorted(split_root.glob("data/chunk-*/*.parquet"))
    if not candidates:
        raise FilterError(f"{split_root}: expected at least one chunk Parquet, got 0")
    return [(path, path.relative_to(split_root).as_posix()) for path in candidates]


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
        "observation.backend_time_ns",
        "action.backend_time_ns",
    }
    missing = sorted(required - set(table.column_names))
    if missing:
        raise FilterError(f"{path}: missing Parquet columns {missing}")
    return table.to_pydict()


def _split(
    *,
    root: Path,
    manifest: dict[str, Any],
    name: str,
) -> dict[str, Any]:
    split = manifest.get("splits", {}).get(name)
    if not isinstance(split, dict):
        raise FilterError(f"manifest.splits.{name} must be an object")
    episodes = split.get("episodes")
    if not isinstance(episodes, list) or len(episodes) != int(split.get("episode_count", -1)):
        raise FilterError(f"manifest.splits.{name}.episodes disagrees with episode_count")
    split_root = root / str(split.get("root", ""))
    parquets = _split_parquets(split_root)
    parquet_records: list[dict[str, str]] = []
    combined_rows: dict[str, list[Any]] | None = None
    for parquet_path, relative in parquets:
        parquet_sha = file_sha256(parquet_path)
        declared = split.get("files", {}).get(relative)
        if not isinstance(declared, dict) or declared.get("sha256") != parquet_sha:
            raise FilterError(f"{parquet_path}: hash does not match the source manifest")
        shard_rows = _read_parquet(parquet_path)
        if combined_rows is None:
            combined_rows = {key: list(values) for key, values in shard_rows.items()}
        else:
            for key, values in shard_rows.items():
                if key in combined_rows:
                    combined_rows[key].extend(values)
        parquet_records.append({"path": str(parquet_path), "sha256": parquet_sha})
    assert combined_rows is not None
    rows = combined_rows
    first_parquet_path = parquets[0][0]
    first_parquet_sha = parquet_records[0]["sha256"]
    if len(rows["index"]) != int(split.get("frame_count", -1)):
        raise FilterError(f"{first_parquet_path}: row count disagrees with the source manifest")
    indices = rows["index"]
    if any(isinstance(value, bool) or not isinstance(value, int) for value in indices):
        raise FilterError(f"{first_parquet_path}: original index must be an integer")
    if len(set(indices)) != len(indices):
        raise FilterError(f"{first_parquet_path}: duplicate original index")
    if any(value < 0 or value >= len(indices) for value in indices):
        raise FilterError(f"{first_parquet_path}: original index is out of range")
    got_episodes = {int(item) for item in rows["episode_index"]}
    if got_episodes != set(range(len(episodes))):
        raise FilterError(
            f"{first_parquet_path}: episode_index does not cover 0..{len(episodes) - 1}"
        )
    declared_episodes = sorted(int(episode["dataset_episode_index"]) for episode in episodes)
    if declared_episodes != list(range(len(episodes))):
        raise FilterError(
            f"{name}: manifest dataset_episode_index values are not 0..{len(episodes) - 1}"
        )
    if sum(int(episode.get("frame_count", 0)) for episode in episodes) != int(split["frame_count"]):
        raise FilterError(f"{name}: manifest episode frame counts disagree with the split")
    return {
        "split": split,
        "root": split_root,
        "parquet_path": first_parquet_path,
        "parquet_sha256": first_parquet_sha,
        "parquets": parquet_records,
        "rows": rows,
        "episodes": episodes,
    }


def _arrays(rows: dict[str, list[Any]], positions: list[int]) -> dict[str, np.ndarray]:
    return {
        "measured": np.asarray([rows["observation.state"][i] for i in positions], dtype=np.float64),
        "applied": np.asarray([rows["action"][i] for i in positions], dtype=np.float64),
        "measured_velocity": np.asarray(
            [rows["observation.velocity"][i] for i in positions], dtype=np.float64
        ),
        "observation_time_ns": np.asarray(
            [rows["observation.backend_time_ns"][i] for i in positions], dtype=np.int64
        ),
        "action_time_ns": np.asarray(
            [rows["action.backend_time_ns"][i] for i in positions], dtype=np.int64
        ),
    }


def _static_edges(
    measured: np.ndarray,
    applied: np.ndarray,
    measured_velocity: np.ndarray,
    applied_velocity: np.ndarray,
    rule: FilterRule,
) -> np.ndarray:
    arms = list(rule.arm_indices)
    grippers = list(rule.gripper_indices)
    measured_step = np.abs(np.diff(measured[:, arms], axis=0))
    applied_step = np.abs(np.diff(applied[:, arms], axis=0))
    grip_measured_step = np.abs(np.diff(measured[:, grippers], axis=0))
    grip_applied_step = np.abs(np.diff(applied[:, grippers], axis=0))
    measured_speed = np.maximum(
        np.abs(measured_velocity[:-1, arms]), np.abs(measured_velocity[1:, arms])
    )
    applied_speed = np.maximum(
        np.abs(applied_velocity[:-1, arms]), np.abs(applied_velocity[1:, arms])
    )
    grip_measured_speed = np.maximum(
        np.abs(measured_velocity[:-1, grippers]), np.abs(measured_velocity[1:, grippers])
    )
    grip_applied_speed = np.maximum(
        np.abs(applied_velocity[:-1, grippers]), np.abs(applied_velocity[1:, grippers])
    )
    return (
        (measured_step <= rule.max_arm_step_rad).all(axis=1)
        & (applied_step <= rule.max_arm_step_rad).all(axis=1)
        & (grip_measured_step <= rule.max_gripper_step_m).all(axis=1)
        & (grip_applied_step <= rule.max_gripper_step_m).all(axis=1)
        & (measured_speed <= rule.max_arm_velocity_rad_s).all(axis=1)
        & (applied_speed <= rule.max_arm_velocity_rad_s).all(axis=1)
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
    """Protect episode edges, phase transitions, and gate transitions."""
    count = len(phase)
    protected = np.zeros(count, dtype=bool)
    edge = rule.episode_boundary_frames
    if edge:
        protected[:edge] = True
        protected[max(0, count - edge) :] = True
    window = rule.phase_boundary_frames
    if window:
        for index in range(1, count):
            if phase[index] != phase[index - 1] or gate[index] != gate[index - 1]:
                protected[max(0, index - window) : min(count, index + window + 1)] = True
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
    allowed_gates = None if rule.only_gate_states is None else set(rule.only_gate_states)
    for start, end in runs:
        length = end - start + 1
        if length < rule.min_static_run_frames:
            continue
        interior_start = start + rule.keep_edge_frames
        interior_end = end - rule.keep_edge_frames
        for index in range(interior_start, interior_end + 1):
            if allowed_gates is not None and gate[index] not in allowed_gates:
                continue
            if (index - interior_start) % rule.interior_anchor_stride:
                keep[index] = False
                reasons[index] = rule.reason
    keep |= _protections(phase, gate, rule)
    for index, saved in enumerate(keep):
        if saved:
            reasons[index] = ""
    return keep, reasons, runs


def _counts(values: list[str | None]) -> dict[str, int]:
    result: dict[str, int] = {}
    for value in values:
        key = "unknown" if value is None else value
        result[key] = result.get(key, 0) + 1
    return result


def _episode_inputs(
    *,
    episode: dict[str, Any],
    split: dict[str, Any],
    lead_ns: int,
    rule: FilterRule,
) -> dict[str, Any]:
    episode_index = int(episode["dataset_episode_index"])
    global_index = int(episode["global_success_index"])
    attempt_id = _text(episode.get("attempt_id"), "episode.attempt_id")
    seed = _integer(episode.get("seed"), "episode.seed", 0, 2**63 - 1)
    frame_count = int(episode.get("frame_count", -1))
    rows = split["rows"]
    positions = [
        index for index, value in enumerate(rows["episode_index"]) if int(value) == episode_index
    ]
    if not positions:
        raise FilterError(f"episode {episode_index}: no Parquet rows")
    positions.sort(key=lambda index: int(rows["index"][index]))
    if len(positions) != frame_count:
        raise FilterError(f"episode {episode_index}: Parquet row count disagrees with manifest")
    source_indices = [int(rows["index"][index]) for index in positions]
    if any(
        int(rows["source.episode_index"][index]) != global_index
        or int(rows["source.seed"][index]) != seed
        for index in positions
    ):
        raise FilterError(f"episode {episode_index}: source episode/seed differs from the manifest")
    arrays = _arrays(rows, positions)
    if len(positions) < 2:
        raise FilterError("an episode needs at least two labelled frames")
    if len(set(int(t) for t in arrays["observation_time_ns"])) != len(positions):
        raise FilterError("Parquet observation clock contains duplicate times")
    if np.any(np.diff(arrays["observation_time_ns"]) != PERIOD_NS):
        raise FilterError("Parquet observation clock is not a uniform 20 Hz")
    if np.any(arrays["action_time_ns"] - arrays["observation_time_ns"] != lead_ns):
        raise FilterError("Parquet action clock does not lead observations by the configured 50 ms")

    filter_meta = episode.get("filter_metadata")
    if not isinstance(filter_meta, dict):
        raise FilterError(
            f"episode {episode_index} ({attempt_id}): manifest has no embedded filter_metadata"
        )
    raw_applied_vel = filter_meta.get("applied_velocities")
    if not isinstance(raw_applied_vel, list) or len(raw_applied_vel) != frame_count:
        raise FilterError(
            f"episode {episode_index} ({attempt_id}): filter_metadata.applied_velocities length mismatch"
        )
    applied_velocity = np.asarray(raw_applied_vel, dtype=np.float64)
    if applied_velocity.shape != arrays["applied"].shape:
        raise FilterError(
            f"episode {episode_index} ({attempt_id}): applied_velocities shape {applied_velocity.shape} != {arrays['applied'].shape}"
        )
    raw_phase = filter_meta.get("phase")
    raw_gate = filter_meta.get("gate_state")
    if not isinstance(raw_phase, list) or len(raw_phase) != frame_count:
        raise FilterError(
            f"episode {episode_index} ({attempt_id}): filter_metadata.phase length mismatch"
        )
    if not isinstance(raw_gate, list) or len(raw_gate) != frame_count:
        raise FilterError(
            f"episode {episode_index} ({attempt_id}): filter_metadata.gate_state length mismatch"
        )
    phase: list[str | None] = [
        item if isinstance(item, str) and item else None for item in raw_phase
    ]
    gate: list[str | None] = [
        item if isinstance(item, str) and item else None for item in raw_gate
    ]
    if rule.only_gate_states is not None and any(item is None for item in gate):
        raise FilterError(
            f"episode {episode_index} ({attempt_id}): rule requires only_gate_states "
            f"{list(rule.only_gate_states)}, but gate_state metadata is missing"
        )
    edges = _static_edges(
        arrays["measured"],
        arrays["applied"],
        arrays["measured_velocity"],
        applied_velocity,
        rule,
    )
    return {
        "episode_index": episode_index,
        "global_index": global_index,
        "attempt_id": attempt_id,
        "seed": seed,
        "source_indices": source_indices,
        "arrays": arrays,
        "phase": phase,
        "gate": gate,
        "edges": edges,
    }


def _episode_report(inputs: dict[str, Any], rule: FilterRule) -> dict[str, Any]:
    keep, reasons, runs = _choose_anchors(
        inputs["edges"], inputs["phase"], inputs["gate"], rule
    )
    kept_count = int(keep.sum())
    fallback = None
    if kept_count < rule.min_episode_kept_frames:
        fallback = "minimum_kept_frames"
        keep[:] = True
        reasons = [""] * len(reasons)
        kept_count = len(keep)
    source_indices = inputs["source_indices"]
    kept_indices = [index for index, saved in zip(source_indices, keep, strict=True) if saved]
    excluded_indices = [
        index for index, saved in zip(source_indices, keep, strict=True) if not saved
    ]
    kept_flags = [index for index, saved in enumerate(keep) if saved]
    excluded_flags = [index for index, saved in enumerate(keep) if not saved]
    long_runs = [run for run in runs if run[1] - run[0] + 1 >= rule.min_static_run_frames]
    return {
        "dataset_episode_index": inputs["episode_index"],
        "global_success_index": inputs["global_index"],
        "attempt_id": inputs["attempt_id"],
        "seed": inputs["seed"],
        "source_frame_count": len(source_indices),
        "kept_count": kept_count,
        "excluded_count": len(excluded_indices),
        "source_indices": source_indices,
        "kept_indices": kept_indices,
        "excluded_indices": excluded_indices,
        "excluded_reasons": {REASON_REDUNDANT: len(excluded_indices)} if excluded_indices else {},
        "phase_counts": {
            "source": _counts(inputs["phase"]),
            "kept": _counts([inputs["phase"][i] for i in kept_flags]),
            "excluded": _counts([inputs["phase"][i] for i in excluded_flags]),
        },
        "gate_counts": {
            "source": _counts(inputs["gate"]),
            "kept": _counts([inputs["gate"][i] for i in kept_flags]),
            "excluded": _counts([inputs["gate"][i] for i in excluded_flags]),
        },
        "static_runs": {
            "total": len(runs),
            "long": len(long_runs),
            "frames_in_long_runs": sum(end - start + 1 for start, end in long_runs),
        },
        "fallback": fallback,
    }


def _filter_episode(
    *,
    episode: dict[str, Any],
    split: dict[str, Any],
    lead_ns: int,
    rule: FilterRule,
) -> dict[str, Any]:
    inputs = _episode_inputs(
        episode=episode, split=split, lead_ns=lead_ns, rule=rule
    )
    return _episode_report(inputs, rule)


def _total_counts(episodes: list[dict[str, Any]], section: str) -> dict[str, dict[str, int]]:
    buckets = ("source", "kept", "excluded")
    result: dict[str, dict[str, int]] = {bucket: {} for bucket in buckets}
    for episode in episodes:
        counts = episode[f"{section}_counts"]
        for bucket in buckets:
            for key, value in counts[bucket].items():
                result[bucket][key] = result[bucket].get(key, 0) + value
    return result


def _source_document(
    root: Path, manifest_path: Path, rule: FilterRule
) -> tuple[dict[str, Any], dict[str, Any]]:
    dataset = {
        "root": str(root),
        "manifest": {"path": str(manifest_path), "sha256": file_sha256(manifest_path)},
    }
    rule_document = {
        "path": str(rule.source),
        "sha256": rule.sha256,
        "resolved": {
            "fps": rule.fps,
            "chunk_size": rule.chunk_size,
            "arm_indices": list(rule.arm_indices),
            "gripper_indices": list(rule.gripper_indices),
            "max_arm_step_rad": rule.max_arm_step_rad,
            "max_arm_velocity_rad_s": rule.max_arm_velocity_rad_s,
            "max_gripper_step_m": rule.max_gripper_step_m,
            "max_gripper_velocity_m_s": rule.max_gripper_velocity_m_s,
            "min_static_run_frames": rule.min_static_run_frames,
            "keep_edge_frames": rule.keep_edge_frames,
            "interior_anchor_stride": rule.interior_anchor_stride,
            "phase_boundary_frames": rule.phase_boundary_frames,
            "episode_boundary_frames": rule.episode_boundary_frames,
            "min_episode_kept_frames": rule.min_episode_kept_frames,
            "only_gate_states": (
                None if rule.only_gate_states is None else list(rule.only_gate_states)
            ),
            "reason": rule.reason,
        },
    }
    return dataset, rule_document


def _filter_inputs(
    rule_path: str | Path, dataset_root: str | Path
) -> tuple[Path, FilterRule, Path, dict, dict, int, str]:
    root = Path(dataset_root).resolve()
    rule = load_filter_rule(rule_path)
    manifest_path = root / "manifest.json"
    manifest = read_filter_json(manifest_path)
    if manifest.get("status") != "complete" or manifest.get("synthetic"):
        raise FilterError("source dataset manifest is not a complete formal dataset")
    lead_ns = int(manifest.get("lerobot", {}).get("action_lead_ns", -1))
    if lead_ns != LEAD_NS:
        raise FilterError("source dataset does not use the fixed 50 ms action lead")
    profile_hash = manifest.get("profile", {}).get("semantic_sha256")
    if not isinstance(profile_hash, str):
        raise FilterError("source manifest has no profile semantic hash")
    training = _split(root=root, manifest=manifest, name="training")
    validation = _split(root=root, manifest=manifest, name="validation")
    train_globals = {int(episode["global_success_index"]) for episode in training["episodes"]}
    validation_globals = {
        int(episode["global_success_index"]) for episode in validation["episodes"]
    }
    if train_globals & validation_globals:
        raise FilterError("training and validation episodes overlap")
    training["manifest_profile_sha256"] = profile_hash
    return root, rule, manifest_path, training, validation, lead_ns, profile_hash


def _filter_document(
    *,
    root: Path,
    rule: FilterRule,
    manifest_path: Path,
    training: dict,
    validation: dict,
    episodes: list[dict[str, Any]],
) -> dict[str, Any]:
    source_frames = sum(episode["source_frame_count"] for episode in episodes)
    if source_frames != int(training["split"]["frame_count"]):
        raise FilterError("filtered episodes do not cover the training Parquet frame count")
    eligible = sum(episode["kept_count"] for episode in episodes)
    excluded = source_frames - eligible
    dataset_document, rule_document = _source_document(root, manifest_path, rule)
    document: dict[str, Any] = {
        "created_wall_time_ns": time.time_ns(),
        "dataset": dataset_document,
        "rule": rule_document,
        "training": {
            "source_frames": source_frames,
            "eligible_frames": eligible,
            "excluded_frames": excluded,
            "reduction_ratio": excluded / source_frames,
            "parquet": {
                "path": str(training["parquet_path"]),
                "sha256": training["parquet_sha256"],
            },
            "parquets": training["parquets"],
            "phase_counts": _total_counts(episodes, "phase"),
            "gate_counts": _total_counts(episodes, "gate"),
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
            "parquets": validation["parquets"],
        },
    }
    document["filter_digest"] = mask_digest(document)
    return document


def build_filter_manifest(
    *, rule_path: str | Path, dataset_root: str | Path
) -> dict[str, Any]:
    (
        root,
        rule,
        manifest_path,
        training,
        validation,
        lead_ns,
        _profile_hash,
    ) = _filter_inputs(rule_path, dataset_root)
    episodes = [
        _filter_episode(
            episode=episode,
            split=training,
            lead_ns=lead_ns,
            rule=rule,
        )
        for episode in training["episodes"]
    ]
    return _filter_document(
        root=root,
        rule=rule,
        manifest_path=manifest_path,
        training=training,
        validation=validation,
        episodes=episodes,
    )


def build_filter_mask(
    *, rule_path: str | Path, dataset_root: str | Path
) -> SampleFilterMask:
    """Compute the training anchor mask in memory from the self-contained dataset."""
    document = build_filter_manifest(rule_path=rule_path, dataset_root=dataset_root)
    return mask_from_document(
        document,
        source=Path(document["rule"]["path"]),
        source_sha256=str(document["rule"]["sha256"]),
        verify_source=False,
    )


__all__ = [
    "FilterError",
    "FilterRule",
    "SampleFilterMask",
    "build_filter_manifest",
    "build_filter_mask",
    "load_filter_mask",
    "load_filter_rule",
]
