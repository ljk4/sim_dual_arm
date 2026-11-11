from __future__ import annotations

import sys
from dataclasses import FrozenInstanceError
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "simulation/mujoco/src"
CONTRACT_SRC = ROOT / "contracts/python/src"
for source in (SIM_SRC, CONTRACT_SRC):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

from robot_backend_sim.motion.stream_schedule import (  # noqa: E402
    ChunkEvent,
    StreamSchedule,
    boundary_rejection,
    grid_aligned,
    next_grid_time,
    observation_rejection,
)

STEP = 2_000_000


def window(observation_time_ns: int, observation_id: int = 1):
    return SimpleNamespace(
        observation_time_ns=observation_time_ns,
        observation_id=observation_id,
    )


def plan(positions, velocities):
    from robot_contracts.trajectory.spline import CubicSegment, ReferencePlan

    # Exact terminal p/v in a real data-only plan; no fake sample method.
    zero = (0.0,) * len(positions)
    c0 = tuple(p - v for p, v in zip(positions, velocities, strict=True))
    return ReferencePlan(
        ("a", "b"), (CubicSegment(1_000_000_000, c0, tuple(velocities), zero, zero),)
    )


def chunk(
    *,
    stream_id="stream-abc:motion-1",
    predecessor=0,
    start=0,
    end=1_000_000_000,
    profile="profile",
    joints=("a", "b"),
    accel=(1.0, 0.5),
    p=(0.0, 0.0),
    v=(0.0, 0.0),
    observation_time=0,
    observation_id=1,
):
    return SimpleNamespace(
        anchor=SimpleNamespace(
            stream_id=stream_id,
            predecessor_seq=predecessor,
            reference=SimpleNamespace(positions=tuple(p), velocities=tuple(v)),
            robot_profile_hash=profile,
        ),
        window=window(observation_time, observation_id),
        joints=tuple(joints),
        max_accelerations=tuple(accel),
        start_sim_time_ns=start,
        end_sim_time_ns=end,
        seq=predecessor + 1,
    )


def active(**overrides):
    values = dict(stream_id="stream-abc:motion-1", predecessor=0, start=0, end=1_000_000_000)
    values.update(overrides)
    item = chunk(**values)
    item.plan = plan((0.0, 0.0), (0.0, 0.0))
    return item


def check(successor, *, now=900_000_000, min_lead=50_000_000, max_age=500_000_000):
    return boundary_rejection(
        active(),
        successor,
        stream_id="stream-abc:motion-1",
        profile_digest="profile",
        joints=("a", "b"),
        max_accelerations=(1.0, 0.5),
        now_sim_ns=now,
        min_lead_ns=min_lead,
        max_observation_age_ns=max_age,
        step_ns=STEP,
    )


def test_grid_alignment_and_lead_never_round_down():
    assert grid_aligned(100_000_000, STEP)
    assert not grid_aligned(101_000_000, STEP)
    assert not grid_aligned(-2, STEP)
    assert next_grid_time(100_000_000, 50_000_000, STEP) == 150_000_000
    assert next_grid_time(101_000_000, 50_000_000, STEP) == 152_000_000
    assert next_grid_time(0, 1, STEP) == STEP
    assert next_grid_time(0, STEP, STEP) == STEP


@pytest.mark.parametrize(
    ("now", "observation", "expected"),
    [
        (100, 100, None),
        (600, 100, None),
        (601, 100, "OBSERVATION_STALE"),
        (99, 100, "OBSERVATION_FROM_FUTURE"),
    ],
)
def test_observation_window_is_not_future_and_has_hard_age(now, observation, expected):
    assert observation_rejection(window(observation), now, 500) == expected


def test_valid_continuation_is_accepted_with_independent_pv_comparison():
    successor = chunk(
        predecessor=1, start=1_000_000_000, end=2_000_000_000, observation_time=800_000_000
    )
    assert check(successor) == (None, False)


@pytest.mark.parametrize(
    ("overrides", "reason"),
    [
        ({"stream_id": "other"}, "STALE_STREAM"),
        ({"predecessor": 2}, "INVALID_CHUNK_ORDER"),
        ({"start": 999_000_000}, "INVALID_CHUNK_START"),
        ({"profile": "other"}, "PROFILE_MISMATCH"),
        ({"joints": ("a", "c")}, "PROFILE_MISMATCH"),
        ({"accel": (1.0, 0.6)}, "PROFILE_MISMATCH"),
        ({"p": (1e-6, 0.0)}, "INVALID_CHUNK_BOUNDARY"),
        ({"v": (1e-6, 0.0)}, "INVALID_CHUNK_BOUNDARY"),
        ({"end": 2_000_001_000}, "INVALID_CHUNK_GRID"),
    ],
)
def test_invalid_packages_only_reject_without_fault(overrides, reason):
    values = dict(
        predecessor=1,
        start=1_000_000_000,
        end=2_000_000_000,
        observation_time=800_000_000,
    )
    values.update(overrides)
    assert check(chunk(**values)) == (reason, False)


def test_legal_but_stale_future_or_late_candidate_latches_fault():
    base = dict(predecessor=1, start=1_000_000_000, end=2_000_000_000)
    assert check(chunk(**base, observation_time=100_000_000), now=900_000_000) == (
        "OBSERVATION_STALE",
        True,
    )
    assert check(chunk(**base, observation_time=901_000_000), now=900_000_000) == (
        "OBSERVATION_FROM_FUTURE",
        True,
    )
    # Admission closes exactly at handoff - min_lead; one nanosecond later is late.
    assert check(chunk(**base, observation_time=900_000_000), now=950_000_000) == (None, False)
    assert check(chunk(**base, observation_time=900_000_000), now=950_000_001) == (
        "CHUNK_LATE",
        True,
    )


def test_event_is_immutable_and_uses_monotonic_stream_sequence():
    event = ChunkEvent(1, 2, "s", 3, 4, 0, 100, 50, "ACCEPTED", "PENDING")
    assert event.event_seq == 1
    with pytest.raises(FrozenInstanceError):
        event.phase = "STARTED"  # type: ignore[misc]
    with pytest.raises(ValueError):
        ChunkEvent(True, 2, "s", 3, 4, 0, 100, 50, "ACCEPTED")


def test_schedule_composes_one_active_and_one_pending_then_handoff():
    schedule = StreamSchedule("stream-abc:motion-1", 7)
    first = chunk(predecessor=0, start=100, end=200)
    second = chunk(predecessor=1, start=200, end=300)
    schedule.install_start(first)
    schedule.install_pending(second)
    assert schedule.active is first and schedule.pending is second
    with pytest.raises(ValueError):
        schedule.install_pending(second)
    events = [
        schedule.make_event("ACCEPTED", first, 10),
        schedule.make_event("STARTED", first, 100),
        schedule.make_event("RETIRED", first, 200, "HANDOFF"),
        schedule.make_event("STARTED", second, 200, "HANDOFF"),
    ]
    assert [event.event_seq for event in events] == [1, 2, 3, 4]
    assert {event.stream_id for event in events} == {"stream-abc:motion-1"}
    retired, started = schedule.handoff()
    assert (retired, started) == (first, second)
    assert schedule.active is second and schedule.pending is None
    schedule.close()
    assert schedule.terminal is True


def test_schedule_cancel_and_close_are_bounded():
    schedule = StreamSchedule("s", 1)
    first = chunk(predecessor=0)
    pending = chunk(predecessor=1, start=1_000_000_000)
    schedule.install_start(first)
    assert schedule.cancel_pending() is None
    schedule.install_pending(pending)
    assert schedule.cancel_pending() is pending
    assert schedule.pending is None
    assert schedule.cancel_pending() is None
    schedule.close()
    assert schedule.active is first and schedule.terminal is True
