from __future__ import annotations

import math

import pytest
from robot_contracts.action_chunks import (
    INT64_MAX,
    UINT64_MAX,
    ActionWindow,
    ChunkAnchor,
)
from robot_contracts.models import ContractError, Target
from robot_contracts.trajectory import ReferenceSample


def target(time_ns: int, positions: tuple[float, ...] = (0.0,)) -> Target:
    return Target(time_ns, positions)


def make_window(
    *,
    target_count: int = 3,
    handoff_ns: int = 1_500_000_000,
    source: tuple[Target, ...] | None = None,
) -> ActionWindow:
    if source is None:
        source = tuple(
            target(time_ns)
            for time_ns in (
                100_000_000,
                500_000_000,
                1_000_000_000,
                1_500_000_000,
                2_000_000_000,
            )
        )
    return ActionWindow(
        observation_id=7,
        observation_time_ns=1_000_000_000,
        handoff_time_ns=handoff_ns,
        source_targets=source,
        target_count=target_count,
    )


def test_window_is_strictly_after_handoff_and_rewrites_only_times():
    source = tuple(
        Target(time_ns, (float(index),))
        for index, time_ns in enumerate(
            (
                100_000_000,
                500_000_000,
                1_000_000_000,
                1_500_000_000,
                2_000_000_000,
            )
        )
    )
    window = make_window(target_count=3, handoff_ns=1_500_000_000, source=source)

    assert window.selected_indices == (2, 3, 4)
    assert window.skipped_overlap_count == 2
    assert window.skipped_overlap_indices == (0, 1)
    assert window.targets == (
        Target(500_000_000, (2.0,)),
        Target(1_000_000_000, (3.0,)),
        Target(1_500_000_000, (4.0,)),
    )
    assert window.end_time_ns == 1_500_000_000
    assert window.source_targets == source
    assert window.selected_source_targets == (source[2], source[3], source[4])


def test_window_excludes_a_point_exactly_at_handoff():
    window = make_window(target_count=2, handoff_ns=2_000_000_000)
    assert window.selected_indices == (3, 4)
    with pytest.raises(ContractError) as caught:
        make_window(target_count=2, handoff_ns=3_000_000_000)
    assert caught.value.reason == "CHUNK_EXPIRED"


def test_window_reports_insufficient_horizon_and_expiry():
    with pytest.raises(ContractError) as caught:
        make_window(target_count=4, handoff_ns=1_500_000_000)
    assert caught.value.reason == "INSUFFICIENT_HORIZON"

    expired = tuple(target(time_ns) for time_ns in (100_000_000, 500_000_000))
    with pytest.raises(ContractError) as caught:
        make_window(target_count=2, handoff_ns=1_500_000_000, source=expired)
    assert caught.value.reason == "CHUNK_EXPIRED"


def test_window_validates_the_complete_source_including_unselected_prefix_and_tail():
    with pytest.raises(ContractError, match="positions"):
        target(100_000_000, (math.nan,))
    with pytest.raises(ContractError, match="positions"):
        target(2_000_000_000, (math.inf,))

    uneven = (
        target(500_000_000, (0.0,)),
        target(1_000_000_000, (0.0, 1.0)),
    )
    with pytest.raises(ContractError, match="source_targets\\[1\\]\\.positions"):
        make_window(target_count=2, handoff_ns=1_000_000_000, source=uneven)


def test_window_rejects_bad_time_and_identity_fields():
    for observation_id in (0, True, 1.0, UINT64_MAX + 1, -1):
        with pytest.raises(ContractError):
            ActionWindow(
                observation_id,  # type: ignore[arg-type]
                0,
                0,
                (target(1_000_000_000), target(2_000_000_000)),
                2,
            )
    for handoff in (-1, True, 1.0):
        with pytest.raises(ContractError):
            ActionWindow(
                1,
                0,
                handoff,  # type: ignore[arg-type]
                (target(1_000_000_000), target(2_000_000_000)),
                2,
            )
    for target_count in (1, 129, True, 2.0):
        with pytest.raises(ContractError):
            ActionWindow(
                1,
                0,
                0,
                (target(1_000_000_000), target(2_000_000_000)),
                target_count,  # type: ignore[arg-type]
            )
    with pytest.raises(ContractError, match="handoff_time_ns"):
        ActionWindow(1, 1_000, 999, (target(1_000_000_000), target(2_000_000_000)), 2)


def test_window_rejects_causal_reverse_bad_times_and_int64_overflow():
    with pytest.raises(ContractError, match="time_ns"):
        make_window(
            target_count=2,
            source=(target(1_000_000_000), target(500_000_000)),
        )
    with pytest.raises(ContractError, match="time_ns"):
        make_window(
            target_count=2,
            source=(target(1_000_000_000), target(1_000_000_000)),
        )
    with pytest.raises(ContractError, match="time_ns"):
        make_window(target_count=2, source=(Target(0, (0.0,)), Target(1_000_000_000, (0.0,))))

    near_max = INT64_MAX - 100
    source = (target(50), target(150))
    with pytest.raises(ContractError, match="int64"):
        ActionWindow(1, near_max, near_max + 60, source, 2)


def test_window_near_int64_uses_exact_integer_differences():
    observation_time = INT64_MAX - 1_000
    source = (target(700), target(1_000))
    window = ActionWindow(
        UINT64_MAX,
        observation_time,
        observation_time + 600,
        source,
        2,
    )
    assert window.targets == (
        Target(100, (0.0,)),
        Target(400, (0.0,)),
    )
    assert window.end_time_ns == 400
    assert window.absolute_times_ns == (observation_time + 700, observation_time + 1_000)


def test_window_and_anchor_are_deeply_immutable():
    source = [Target(1_000_000_000, [0.5]), Target(2_000_000_000, [1.0])]
    window = ActionWindow(1, 0, 0, source, 2)  # type: ignore[arg-type]
    source[0] = Target(1_000_000_000, (99.0,))
    source[1] = Target(2_000_000_000, (99.0,))
    assert window.source_targets == (
        Target(1_000_000_000, (0.5,)),
        Target(2_000_000_000, (1.0,)),
    )
    assert isinstance(window.targets[0].positions, tuple)

    sample = ReferenceSample((0.0,), (1.0,), (0.0,))
    anchor = ChunkAnchor("stream-1", 4, 1_000, sample, "profile-hash")
    assert anchor.seq == 5
    assert anchor.reference == sample


def test_anchor_validates_identity_time_reference_and_seq_bounds():
    sample = ReferenceSample((0.0,), (0.0,), (0.0,))
    assert ChunkAnchor("s", 0, 0, sample, "h").seq == 1
    assert ChunkAnchor("s", UINT64_MAX - 1, INT64_MAX, sample, "h").seq == UINT64_MAX

    for stream_id in ("", "x" * 129, 1):
        with pytest.raises(ContractError):
            ChunkAnchor(stream_id, 0, 0, sample, "h")  # type: ignore[arg-type]
    for seq in (-1, True, 1.0, UINT64_MAX):
        with pytest.raises(ContractError):
            ChunkAnchor("s", seq, 0, sample, "h")  # type: ignore[arg-type]
    for start in (-1, True, 1.0, INT64_MAX + 1):
        with pytest.raises(ContractError):
            ChunkAnchor("s", 0, start, sample, "h")  # type: ignore[arg-type]
    for reference in (None, (0.0,), object()):
        with pytest.raises(ContractError):
            ChunkAnchor("s", 0, 0, reference, "h")  # type: ignore[arg-type]
    for profile_hash in ("", None, 1):
        with pytest.raises(ContractError):
            ChunkAnchor("s", 0, 0, sample, profile_hash)  # type: ignore[arg-type]
