"""Pure, bounded label extraction for the continuous follower stream.

A label is the actually applied reference at ``observation_time + 50 ms`` for a
real 20 Hz frame.  The raw 60-point prediction belongs to the executed chunk,
not to the frame that happened to request it.  Pending frames, not-yet-started
blocks, canceled/retired blocks and BRAKE-replaced intervals never become
labels.
"""

from __future__ import annotations

from dataclasses import dataclass

from robot_contracts.models import ContractError, RobotProfile

LABEL_LEAD_NS = 50_000_000
EXPECTED_OBSERVATION_PERIOD_NS = 50_000_000
# 30 s at 20 Hz is 600 frames; leave bounded room for start-up/shutdown frames.
DEFAULT_MAX_LABELS = 700
MAX_LABELS_HARD = 4096
MAX_RAW_POINTS = 60


@dataclass(frozen=True, slots=True)
class ObservationFact:
    observation_id: int
    sim_time_ns: int
    state_seq: int
    measured_positions: tuple[float, ...]
    measured_velocities: tuple[float, ...]
    frame_seqs: tuple[int, ...]
    source: str = "exact_capture"


@dataclass(frozen=True, slots=True)
class AppliedBlock:
    """One accepted block with real per-chunk STARTED/RETIRED/CANCELED facts."""

    chunk_seq: int
    command_id: str
    start_sim_time_ns: int
    end_sim_time_ns: int
    started_sim_time_ns: int | None
    retired_sim_time_ns: int | None
    canceled_sim_time_ns: int | None
    reference: object

    def effective_end_ns(self) -> int:
        ends = [self.end_sim_time_ns]
        if self.retired_sim_time_ns is not None:
            ends.append(self.retired_sim_time_ns)
        if self.canceled_sim_time_ns is not None:
            ends.append(self.canceled_sim_time_ns)
        return min(ends)


@dataclass(frozen=True, slots=True)
class BrakeInterval:
    start_sim_time_ns: int
    end_sim_time_ns: int


@dataclass(frozen=True, slots=True)
class RawPrediction:
    chunk_seq: int
    command_id: str
    source_observation_id: int
    source_observation_time_ns: int
    times_from_observation_ns: tuple[int, ...]
    positions: tuple[tuple[float, ...], ...]


def _ordered_observations(
    observations: list[ObservationFact], max_labels: int
) -> list[ObservationFact]:
    if len(observations) > max_labels:
        raise ContractError(
            "LABEL_EXPORT_LIMIT",
            f"{len(observations)} exact frames exceed the configured {max_labels} bound",
        )
    ordered = sorted(observations, key=lambda item: item.sim_time_ns)
    for left, right in zip(ordered, ordered[1:], strict=False):
        if right.sim_time_ns <= left.sim_time_ns:
            raise ContractError("LABEL_EXPORT_ORDER", "observations are not strictly increasing")
    return ordered


def _label_row(
    observation: ObservationFact,
    label_time: int,
    block: AppliedBlock,
    prediction: RawPrediction,
    applied: object,
    period_ns: int,
) -> dict[str, object]:
    return {
        "observation_id": observation.observation_id,
        "state_seq": observation.state_seq,
        "capture_source": observation.source,
        "observation_sim_time_ns": observation.sim_time_ns,
        "label_sim_time_ns": label_time,
        "observation_period_ns": period_ns,
        "chunk_seq": block.chunk_seq,
        "command_id": block.command_id,
        "started_sim_time_ns": block.started_sim_time_ns,
        "block_end_sim_time_ns": block.effective_end_ns(),
        "applied_positions": list(applied.applied_positions),
        "sampled_positions": list(applied.sampled_positions),
        "applied_velocities": list(applied.sampled_velocities),
        "measured_positions": list(observation.measured_positions),
        "measured_velocities": list(observation.measured_velocities),
        "frame_seqs": list(observation.frame_seqs),
        "raw_chunk_seq": prediction.chunk_seq,
        "raw_source_observation_id": prediction.source_observation_id,
        "raw_source_observation_time_ns": prediction.source_observation_time_ns,
    }


def build_labels(
    observations: list[ObservationFact],
    blocks: list[AppliedBlock],
    brakes: list[BrakeInterval],
    raw_by_chunk: dict[int, RawPrediction],
    *,
    profile: RobotProfile,
    lead_ns: int = LABEL_LEAD_NS,
    expected_period_ns: int = EXPECTED_OBSERVATION_PERIOD_NS,
    max_labels: int = DEFAULT_MAX_LABELS,
) -> tuple[list[dict[str, object]], dict[str, object]]:
    """Return bounded label rows plus an explicit rejection summary."""
    if isinstance(max_labels, bool) or not 1 <= max_labels <= MAX_LABELS_HARD:
        raise ContractError("LABEL_EXPORT_LIMIT", f"max_labels must be within 1..{MAX_LABELS_HARD}")
    ordered = _ordered_observations(observations, max_labels)
    rows: list[dict[str, object]] = []
    rejected: dict[str, int] = {
        "no_started_block": 0,
        "not_executed_interval": 0,
        "brake_replaced": 0,
        "missing_raw_block": 0,
        "non_uniform_observation_period": 0,
        "unknown": 0,
    }
    for index, observation in enumerate(ordered):
        label_time = observation.sim_time_ns + lead_ns
        block = _block_at(blocks, label_time)
        if block is None:
            rejected["no_started_block"] += 1
            continue
        if block.started_sim_time_ns is None or label_time < block.started_sim_time_ns:
            rejected["not_executed_interval"] += 1
            continue
        if label_time > block.effective_end_ns():
            rejected["not_executed_interval"] += 1
            continue
        if _braked(brakes, observation.sim_time_ns, label_time):
            rejected["brake_replaced"] += 1
            continue
        prediction = raw_by_chunk.get(block.chunk_seq)
        if prediction is None:
            rejected["missing_raw_block"] += 1
            continue
        try:
            applied = block.reference.applied_sample_at(label_time, profile)  # type: ignore[attr-defined]
        except ContractError:
            rejected["not_executed_interval"] += 1
            continue
        if index + 1 >= len(ordered):
            rejected["non_uniform_observation_period"] += 1
            continue
        period_ns = ordered[index + 1].sim_time_ns - observation.sim_time_ns
        if period_ns != expected_period_ns:
            rejected["non_uniform_observation_period"] += 1
            continue
        rows.append(
            _label_row(observation, label_time, block, prediction, applied, period_ns)
        )
        if len(rows) >= max_labels:
            break
    summary = {
        "observations": len(ordered),
        "labels": len(rows),
        "rejected": rejected,
        "lead_ns": lead_ns,
        "expected_period_ns": expected_period_ns,
        "max_labels": max_labels,
    }
    return rows, summary


def _block_at(blocks: list[AppliedBlock], sim_time_ns: int) -> AppliedBlock | None:
    candidates = [
        block
        for block in blocks
        if block.start_sim_time_ns <= sim_time_ns <= block.effective_end_ns()
        and block.started_sim_time_ns is not None
        and sim_time_ns >= block.started_sim_time_ns
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda item: item.start_sim_time_ns)


def _braked(brakes: list[BrakeInterval], start_ns: int, end_ns: int) -> bool:
    return any(
        brake.start_sim_time_ns <= end_ns and brake.end_sim_time_ns > start_ns for brake in brakes
    )


__all__ = [
    "AppliedBlock",
    "BrakeInterval",
    "DEFAULT_MAX_LABELS",
    "EXPECTED_OBSERVATION_PERIOD_NS",
    "LABEL_LEAD_NS",
    "MAX_LABELS_HARD",
    "ObservationFact",
    "RawPrediction",
    "build_labels",
]
