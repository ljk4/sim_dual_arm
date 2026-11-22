"""In-memory down-sampling of redundant static training anchors."""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Annotated, Any, Literal

import numpy as np
from pydantic import BeforeValidator, Field, model_validator
from robot_contracts.validation import (
    NonEmptyStr,
    NonNegativeFloat,
    PositiveInt,
    StrictMappingModel,
    as_tuple,
    contract_error,
)

from ..config import ConfigError, load_yaml_mapping
from .common import canonical_hash, file_sha256

LEAD_NS = 50_000_000
PERIOD_NS = 50_000_000
REASON_REDUNDANT = "redundant_static_anchor"
ARM_INDICES = (0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14)
GRIPPERS = (7, 15)

JointIndices = Annotated[
    tuple[Annotated[int, Field(strict=True, ge=0, le=255)], ...],
    BeforeValidator(as_tuple),
    Field(min_length=1),
]
StepLimit = Annotated[NonNegativeFloat, Field(le=1.0)]
SpeedLimit = Annotated[NonNegativeFloat, Field(le=100.0)]
FrameCount = Annotated[int, Field(strict=True, ge=0, le=100_000)]


def _optional_tuple(value: object) -> object:
    return None if value is None else as_tuple(value)


OptionalGateStates = Annotated[
    tuple[NonEmptyStr, ...] | None,
    BeforeValidator(_optional_tuple),
]


class FilterError(ValueError):
    """The filter rule or source dataset cannot be trusted."""


@dataclass(frozen=True, slots=True)
class FilterRule:
    fps: int
    chunk_size: int
    arm_indices: tuple[int, ...]
    gripper_indices: tuple[int, ...]
    max_arm_step_rad: float
    max_arm_velocity_rad_s: float
    max_gripper_step_m: float
    max_gripper_velocity_m_s: float
    min_static_run_frames: int
    keep_edge_frames: int
    interior_anchor_stride: int
    phase_boundary_frames: int
    episode_boundary_frames: int
    min_episode_kept_frames: int
    reason: str
    source: Path
    only_gate_states: tuple[str, ...] | None = None

    @property
    def sha256(self) -> str:
        return file_sha256(self.source)


@dataclass(frozen=True, slots=True)
class SampleFilterMask:
    source: Path
    source_sha256: str
    digest: str
    eligible_indices: tuple[int, ...]
    source_frames: int
    excluded_frames: int
    manifest_sha256: str = ""

    def summary(self) -> dict[str, Any]:
        return {
            "source": str(self.source),
            "source_sha256": self.source_sha256,
            "digest": self.digest,
            "source_frames": self.source_frames,
            "eligible_frames": len(self.eligible_indices),
            "excluded_frames": self.excluded_frames,
            "reduction_ratio": self.excluded_frames / self.source_frames,
        }


class _ArmRule(StrictMappingModel):
    joint_indices: JointIndices = ARM_INDICES
    max_step_rad: StepLimit
    max_velocity_rad_s: SpeedLimit

    @model_validator(mode="after")
    def _unique(self) -> _ArmRule:
        if len(set(self.joint_indices)) != len(self.joint_indices):
            raise contract_error("joint_indices", "must contain unique joint indices")
        return self


class _GripperRule(StrictMappingModel):
    joint_indices: JointIndices = GRIPPERS
    max_step_m: StepLimit
    max_velocity_m_s: SpeedLimit

    @model_validator(mode="after")
    def _unique(self) -> _GripperRule:
        if len(set(self.joint_indices)) != len(self.joint_indices):
            raise contract_error("joint_indices", "must contain unique joint indices")
        return self


class _StaticRuns(StrictMappingModel):
    min_frames: Annotated[int, Field(strict=True, ge=2, le=100_000)]
    edge_frames: FrameCount
    interior_stride: Annotated[int, Field(strict=True, ge=1, le=100_000)]
    phase_boundary_frames: FrameCount
    episode_boundary_frames: FrameCount
    min_episode_kept_frames: FrameCount
    only_gate_states: OptionalGateStates = None
    reason: Literal["redundant_static_anchor"]

    @model_validator(mode="after")
    def _gates(self) -> _StaticRuns:
        if self.only_gate_states is not None:
            if not self.only_gate_states:
                raise contract_error("only_gate_states", "must not be empty")
            if len(set(self.only_gate_states)) != len(self.only_gate_states):
                raise contract_error("only_gate_states", "must contain unique gate states")
        return self


class _FilterDocument(StrictMappingModel):
    fps: PositiveInt
    chunk_size: PositiveInt
    arms: _ArmRule
    grippers: _GripperRule
    static_runs: _StaticRuns

    @model_validator(mode="after")
    def _run_shape(self) -> _FilterDocument:
        if self.static_runs.edge_frames * 2 >= self.static_runs.min_frames:
            raise contract_error(
                "static_runs",
                "min_frames must exceed twice edge_frames so a run has an interior",
            )
        return self


def load_filter_rule(path: str | Path) -> FilterRule:
    source = Path(path).resolve()
    try:
        document = _FilterDocument(**load_yaml_mapping(source))
    except ConfigError as error:
        raise ConfigError(str(source), error.field, error.message) from error
    arms, grippers, runs = document.arms, document.grippers, document.static_runs
    return FilterRule(
        fps=document.fps,
        chunk_size=document.chunk_size,
        arm_indices=tuple(arms.joint_indices),
        gripper_indices=tuple(grippers.joint_indices),
        max_arm_step_rad=float(arms.max_step_rad),
        max_arm_velocity_rad_s=float(arms.max_velocity_rad_s),
        max_gripper_step_m=float(grippers.max_step_m),
        max_gripper_velocity_m_s=float(grippers.max_velocity_m_s),
        min_static_run_frames=runs.min_frames,
        keep_edge_frames=runs.edge_frames,
        interior_anchor_stride=runs.interior_stride,
        phase_boundary_frames=runs.phase_boundary_frames,
        episode_boundary_frames=runs.episode_boundary_frames,
        min_episode_kept_frames=runs.min_episode_kept_frames,
        reason=runs.reason,
        source=source,
        only_gate_states=(
            None if runs.only_gate_states is None else tuple(runs.only_gate_states)
        ),
    )


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise FilterError(f"{path}: cannot read JSON: {error}") from error
    if not isinstance(value, dict):
        raise FilterError(f"{path}: expected an object")
    return value


def _split_parquets(split_root: Path) -> list[Path]:
    candidates = sorted(split_root.glob("data/chunk-*/*.parquet"))
    if not candidates:
        raise FilterError(f"{split_root}: expected at least one chunk Parquet, got 0")
    return candidates


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


def _split(root: Path, manifest: dict[str, Any], name: str) -> dict[str, Any]:
    split = manifest.get("splits", {}).get(name)
    if not isinstance(split, dict):
        raise FilterError(f"manifest.splits.{name} must be an object")
    episodes = split.get("episodes")
    if not isinstance(episodes, list) or len(episodes) != int(split.get("episode_count", -1)):
        raise FilterError(f"manifest.splits.{name}.episodes disagrees with episode_count")
    split_root = root / str(split.get("root", ""))
    parquets = _split_parquets(split_root)
    rows: dict[str, list[Any]] | None = None
    for parquet_path in parquets:
        shard_rows = _read_parquet(parquet_path)
        if rows is None:
            rows = {key: list(values) for key, values in shard_rows.items()}
        else:
            for key, values in shard_rows.items():
                if key in rows:
                    rows[key].extend(values)
    assert rows is not None
    first_parquet = parquets[0]
    frame_count = int(split.get("frame_count", -1))
    if len(rows["index"]) != frame_count:
        raise FilterError(f"{first_parquet}: row count disagrees with the source manifest")
    indices = rows["index"]
    if any(isinstance(v, bool) or not isinstance(v, int) for v in indices):
        raise FilterError(f"{first_parquet}: original index must be an integer")
    if len(set(indices)) != len(indices) or any(v < 0 or v >= len(indices) for v in indices):
        raise FilterError(f"{first_parquet}: invalid original index sequence")
    if {int(item) for item in rows["episode_index"]} != set(range(len(episodes))):
        raise FilterError(f"{first_parquet}: episode_index does not cover 0..{len(episodes) - 1}")
    if sorted(int(ep["dataset_episode_index"]) for ep in episodes) != list(range(len(episodes))):
        raise FilterError(
            f"{name}: manifest dataset_episode_index values are not 0..{len(episodes) - 1}"
        )
    if sum(int(ep.get("frame_count", 0)) for ep in episodes) != frame_count:
        raise FilterError(f"{name}: manifest episode frame counts disagree with the split")
    return {"split": split, "root": split_root, "rows": rows, "episodes": episodes}


def _static_edges(
    measured: np.ndarray,
    applied: np.ndarray,
    measured_vel: np.ndarray,
    applied_vel: np.ndarray,
    rule: FilterRule,
) -> np.ndarray:
    arms, grippers = list(rule.arm_indices), list(rule.gripper_indices)

    def _within(
        arr: np.ndarray, vel: np.ndarray, idx: list[int], max_step: float, max_speed: float
    ) -> np.ndarray:
        step_ok = (np.abs(np.diff(arr[:, idx], axis=0)) <= max_step).all(axis=1)
        speed_ok = (np.maximum(np.abs(vel[:-1, idx]), np.abs(vel[1:, idx])) <= max_speed).all(axis=1)
        return step_ok & speed_ok

    return (
        _within(measured, measured_vel, arms, rule.max_arm_step_rad, rule.max_arm_velocity_rad_s)
        & _within(applied, applied_vel, arms, rule.max_arm_step_rad, rule.max_arm_velocity_rad_s)
        & _within(measured, measured_vel, grippers, rule.max_gripper_step_m, rule.max_gripper_velocity_m_s)
        & _within(applied, applied_vel, grippers, rule.max_gripper_step_m, rule.max_gripper_velocity_m_s)
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
            if phase[index] != phase[index - 1] or gate[index] != gate[index - 1]:
                protected[max(0, index - window) : min(count, index + window + 1)] = True
    return protected


def _filter_episode(
    *,
    episode: dict[str, Any],
    split: dict[str, Any],
    lead_ns: int,
    period_ns: int,
    rule: FilterRule,
) -> dict[str, Any]:
    episode_index = int(episode["dataset_episode_index"])
    global_index = int(episode["global_success_index"])
    attempt_id = episode.get("attempt_id")
    if not isinstance(attempt_id, str) or not attempt_id:
        raise FilterError("episode.attempt_id: must be a non-empty string")
    seed = episode.get("seed")
    if isinstance(seed, bool) or not isinstance(seed, int) or not 0 <= seed <= 2**63 - 1:
        raise FilterError("episode.seed: must be a non-negative integer")
    frame_count = int(episode.get("frame_count", -1))
    rows = split["rows"]
    positions = sorted(
        (i for i, val in enumerate(rows["episode_index"]) if int(val) == episode_index),
        key=lambda i: int(rows["index"][i]),
    )
    if len(positions) != frame_count or len(positions) < 2:
        raise FilterError(f"episode {episode_index}: invalid Parquet row count {len(positions)}")
    source_indices = [int(rows["index"][i]) for i in positions]
    if any(
        int(rows["source.episode_index"][i]) != global_index or int(rows["source.seed"][i]) != seed
        for i in positions
    ):
        raise FilterError(f"episode {episode_index}: source episode/seed differs from the manifest")
    measured = np.asarray([rows["observation.state"][i] for i in positions], dtype=np.float64)
    applied = np.asarray([rows["action"][i] for i in positions], dtype=np.float64)
    measured_vel = np.asarray([rows["observation.velocity"][i] for i in positions], dtype=np.float64)
    obs_ns = np.asarray([rows["observation.backend_time_ns"][i] for i in positions], dtype=np.int64)
    act_ns = np.asarray([rows["action.backend_time_ns"][i] for i in positions], dtype=np.int64)
    if measured.ndim != 2 or applied.shape != measured.shape or measured_vel.shape != measured.shape:
        raise FilterError(f"episode {episode_index}: inconsistent state/action tensor shapes")
    state_dim = measured.shape[1]
    if max((*rule.arm_indices, *rule.gripper_indices)) >= state_dim:
        raise FilterError(
            f"episode {episode_index}: filter joint index exceeds state_dim={state_dim}"
        )
    if len(set(int(t) for t in obs_ns)) != len(positions) or np.any(np.diff(obs_ns) != period_ns):
        raise FilterError("Parquet observation clock is not uniform at the configured fps")
    if np.any(act_ns - obs_ns != lead_ns):
        raise FilterError("Parquet action clock does not lead observations by the configured lead")

    filter_meta = episode.get("filter_metadata")
    if not isinstance(filter_meta, dict):
        raise FilterError(f"episode {episode_index} ({attempt_id}): missing filter_metadata")
    raw_applied_vel = filter_meta.get("applied_velocities")
    if not isinstance(raw_applied_vel, list) or len(raw_applied_vel) != frame_count:
        raise FilterError(f"episode {episode_index} ({attempt_id}): applied_velocities mismatch")
    applied_vel = np.asarray(raw_applied_vel, dtype=np.float64)
    if applied_vel.shape != applied.shape:
        raise FilterError(
            f"episode {episode_index} ({attempt_id}): applied_velocities shape {applied_vel.shape} != {applied.shape}"
        )
    raw_phase, raw_gate = filter_meta.get("phase"), filter_meta.get("gate_state")
    if not isinstance(raw_phase, list) or len(raw_phase) != frame_count:
        raise FilterError(f"episode {episode_index} ({attempt_id}): phase metadata mismatch")
    if not isinstance(raw_gate, list) or len(raw_gate) != frame_count:
        raise FilterError(f"episode {episode_index} ({attempt_id}): gate_state metadata mismatch")
    phase: list[str | None] = [v if isinstance(v, str) and v else None for v in raw_phase]
    gate: list[str | None] = [v if isinstance(v, str) and v else None for v in raw_gate]
    if rule.only_gate_states is not None and any(item is None for item in gate):
        raise FilterError(
            f"episode {episode_index} ({attempt_id}): rule requires only_gate_states "
            f"{list(rule.only_gate_states)}, but gate_state metadata is missing"
        )

    edges = _static_edges(measured, applied, measured_vel, applied_vel, rule)
    runs = _static_runs(edges)
    keep = np.ones(len(edges) + 1, dtype=bool)
    allowed_gates = None if rule.only_gate_states is None else set(rule.only_gate_states)
    for start, end in runs:
        if end - start + 1 < rule.min_static_run_frames:
            continue
        interior_start = start + rule.keep_edge_frames
        interior_end = end - rule.keep_edge_frames
        for index in range(interior_start, interior_end + 1):
            if allowed_gates is not None and gate[index] not in allowed_gates:
                continue
            if (index - interior_start) % rule.interior_anchor_stride:
                keep[index] = False
    keep |= _protections(phase, gate, rule)
    fallback = None
    if int(keep.sum()) < rule.min_episode_kept_frames:
        fallback = "minimum_kept_frames"
        keep[:] = True
    kept_indices = [idx for idx, saved in zip(source_indices, keep, strict=True) if saved]
    excluded_indices = [idx for idx, saved in zip(source_indices, keep, strict=True) if not saved]
    return {
        "dataset_episode_index": episode_index,
        "global_success_index": global_index,
        "attempt_id": attempt_id,
        "seed": seed,
        "source_frame_count": len(source_indices),
        "kept_count": len(kept_indices),
        "excluded_count": len(excluded_indices),
        "kept_indices": kept_indices,
        "excluded_indices": excluded_indices,
        "static_runs": len(runs),
        "fallback": fallback,
    }


def build_filter_manifest(*, rule_path: str | Path, dataset_root: str | Path) -> dict[str, Any]:
    """Evaluate ``rule_path`` against ``dataset_root`` and return the filter summary document."""
    root = Path(dataset_root).resolve()
    rule = load_filter_rule(rule_path)
    manifest_path = root / "manifest.json"
    manifest = _read_json(manifest_path)
    if manifest.get("status") != "complete" or manifest.get("synthetic"):
        raise FilterError("source dataset manifest is not a complete formal dataset")
    period_ns = round(1_000_000_000 / rule.fps)
    lead_ns = int(manifest.get("lerobot", {}).get("action_lead_ns", -1))
    if lead_ns != period_ns:
        raise FilterError(
            f"source dataset action_lead_ns {lead_ns} disagrees with rule period {period_ns}"
        )
    training = _split(root, manifest, "training")
    validation = _split(root, manifest, "validation")
    train_globals = {int(ep["global_success_index"]) for ep in training["episodes"]}
    val_globals = {int(ep["global_success_index"]) for ep in validation["episodes"]}
    if train_globals & val_globals:
        raise FilterError("training and validation episodes overlap")

    episodes = [
        _filter_episode(
            episode=episode, split=training, lead_ns=lead_ns, period_ns=period_ns, rule=rule
        )
        for episode in training["episodes"]
    ]
    source_frames = sum(ep["source_frame_count"] for ep in episodes)
    if source_frames != int(training["split"]["frame_count"]):
        raise FilterError("filtered episodes do not cover the training Parquet frame count")
    eligible = sum(ep["kept_count"] for ep in episodes)
    excluded = source_frames - eligible
    payload = {
        "manifest_sha256": file_sha256(manifest_path),
        "rule": {
            "path": str(rule.source),
            "sha256": rule.sha256,
            "fps": rule.fps,
            "chunk_size": rule.chunk_size,
            "only_gate_states": (
                None if rule.only_gate_states is None else list(rule.only_gate_states)
            ),
            "reason": rule.reason,
        },
        "training": {
            "source_frames": source_frames,
            "eligible_frames": eligible,
            "excluded_frames": excluded,
            "reduction_ratio": excluded / source_frames,
            "episodes": episodes,
        },
        "validation": {
            "episode_count": len(validation["episodes"]),
            "source_frames": int(validation["split"]["frame_count"]),
        },
    }
    return {
        "created_wall_time_ns": time.time_ns(),
        "filter_digest": canonical_hash(payload),
        **payload,
    }


def build_filter_mask(*, rule_path: str | Path, dataset_root: str | Path) -> SampleFilterMask:
    """Compute the in-memory training anchor mask from a filter rule YAML."""
    document = build_filter_manifest(rule_path=rule_path, dataset_root=dataset_root)
    eligible: list[int] = []
    for episode in document["training"]["episodes"]:
        eligible.extend(episode["kept_indices"])
    source_frames = int(document["training"]["source_frames"])
    return SampleFilterMask(
        source=Path(document["rule"]["path"]),
        source_sha256=str(document["rule"]["sha256"]),
        digest=str(document["filter_digest"]),
        eligible_indices=tuple(eligible),
        source_frames=source_frames,
        excluded_frames=source_frames - len(eligible),
        manifest_sha256=str(document["manifest_sha256"]),
    )


__all__ = [
    "ARM_INDICES",
    "FilterError",
    "FilterRule",
    "GRIPPERS",
    "LEAD_NS",
    "PERIOD_NS",
    "REASON_REDUNDANT",
    "SampleFilterMask",
    "build_filter_manifest",
    "build_filter_mask",
    "load_filter_rule",
]
