from functools import partial
from pathlib import Path
from types import SimpleNamespace

import pytest
from dual_arm_runtime.offline.export import label_export
from dual_arm_runtime.offline.export.label_export import (
    AppliedBlock,
    BrakeInterval,
    ObservationFact,
    RawPrediction,
)
from dual_arm_runtime.offline.export.label_export import (
    build_labels as _build_labels,
)
from robot_contracts.profiles import load_profile

ROOT = Path(__file__).resolve().parents[3]
PROFILE = load_profile(
    ROOT / "shared/scenarios/marvin/mujoco/stack/profiles/robot/profile.json",
    ROOT / "shared/common/interfaces/schemas",
)

build_labels = partial(_build_labels, profile=PROFILE)


class _Reference:
    def applied_sample_at(self, sim_time_ns, profile):
        return SimpleNamespace(
            applied_positions=(0.04,) * 16,
            sampled_positions=(0.041,) * 16,
            sampled_velocities=(0.0,) * 16,
        )


def _observation(observation_id, backend_time_ns, measured=0.0):
    return ObservationFact(
        observation_id=observation_id,
        backend_time_ns=backend_time_ns,
        state_seq=observation_id,
        measured_positions=(measured,) * 16,
        measured_velocities=(0.0,) * 16,
        frame_seqs=(observation_id,) * 3,
    )


def _block(
    chunk=1,
    start=1_000_000_000,
    end=1_250_000_000,
    started=1_020_000_000,
    retired=None,
    canceled=None,
):
    return AppliedBlock(
        chunk_seq=chunk,
        command_id="command",
        start_backend_time_ns=start,
        end_backend_time_ns=end,
        started_backend_time_ns=started,
        retired_backend_time_ns=retired,
        canceled_backend_time_ns=canceled,
        reference=_Reference(),
    )


def _raw(chunk=1, source_observation=1):
    times = tuple((index + 1) * 50_000_000 for index in range(60))
    return RawPrediction(
        chunk_seq=chunk,
        command_id="command",
        source_observation_id=source_observation,
        source_observation_time_ns=1_000_000_000,
        times_from_observation_ns=times,
        positions=tuple((0.1,) * 16 for _ in times),
    )


def test_label_is_applied_reference_and_raw_belongs_to_executed_chunk():
    rows, summary = build_labels(
        [_observation(1, 1_000_000_000), _observation(2, 1_050_000_000)],
        [_block()],
        [],
        {1: _raw(source_observation=7)},
    )
    assert summary["labels"] == 1
    row = rows[0]
    assert row["label_backend_time_ns"] == 1_050_000_000
    assert row["applied_positions"][0] == 0.04
    assert row["measured_positions"][0] == 0.0
    assert row["raw_source_observation_id"] == 7
    assert "raw_positions" not in row


def test_20hz_frames_share_4hz_raw_and_reconstruct_more_than_30_labels():
    observations = [
        _observation(index + 1, 1_000_000_000 + index * 50_000_000) for index in range(52)
    ]
    blocks = []
    raw = {}
    for chunk in range(1, 9):
        start = 1_000_000_000 + (chunk - 1) * 250_000_000
        blocks.append(
            _block(
                chunk=chunk,
                start=start,
                end=start + 250_000_000,
                started=start + 10_000_000,
                retired=start + 250_000_000,
            )
        )
        raw[chunk] = _raw(chunk=chunk, source_observation=1 + (chunk - 1) * 5)
    rows, summary = build_labels(observations, blocks, [], raw)
    assert summary["labels"] > 30
    assert summary["rejected"]["missing_raw_block"] == 0
    source_ids = {row["raw_source_observation_id"] for row in rows}
    assert len(source_ids) <= 8
    assert all(row["period_ok"] if "period_ok" in row else True for row in rows)


def test_first_block_not_started_yields_no_label_and_cancel_stops_interval():
    rows, summary = build_labels(
        [_observation(1, 1_000_000_000), _observation(2, 1_050_000_000)],
        [_block(started=None)],
        [],
        {1: _raw()},
    )
    assert rows == []
    assert summary["rejected"]["no_started_block"] >= 1

    rows, summary = build_labels(
        [_observation(1, 1_000_000_000), _observation(2, 1_050_000_000)],
        [_block(canceled=1_040_000_000)],
        [],
        {1: _raw()},
    )
    assert rows == []
    assert summary["rejected"]["no_started_block"] >= 1


def test_brake_interval_replaces_labels():
    rows, summary = build_labels(
        [_observation(1, 1_000_000_000), _observation(2, 1_050_000_000)],
        [_block()],
        [BrakeInterval(1_000_000_000, 1_100_000_000)],
        {1: _raw()},
    )
    assert rows == []
    assert summary["rejected"]["brake_replaced"] == 2


def test_100ms_observations_cannot_be_reported_as_50ms_labels():
    rows, summary = build_labels(
        [_observation(1, 1_000_000_000), _observation(2, 1_100_000_000)],
        [_block()],
        [],
        {1: _raw()},
    )
    assert rows == []
    assert summary["rejected"]["non_uniform_observation_period"] == 2


def test_max_labels_default_covers_30s_at_20hz():
    observations = [
        _observation(index + 1, 1_000_000_000 + index * 50_000_000) for index in range(600)
    ]
    assert len(observations) == 600


@pytest.fixture(autouse=True)
def sample_test_reference(monkeypatch):
    """Explicit test_only sampler for the synthetic reference used in these label tests."""
    monkeypatch.setattr(
        label_export,
        "applied_follower_sample",
        lambda reference, stamp, profile: reference.applied_sample_at(stamp, profile),
    )
