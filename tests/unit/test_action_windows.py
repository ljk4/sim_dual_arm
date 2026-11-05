from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest
from robot_contracts.action_chunks import (
    INT64_MAX,
    UINT64_MAX,
    ActionWindow,
    ChunkAnchor,
)
from robot_contracts.models import ContractError, JointSpec, Target
from robot_contracts.reference import (
    CubicSegment,
    ReferencePlan,
    ReferenceSample,
    verify_reference_plan,
)

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.chunk_plan import PreparedChunk  # noqa: E402


def target(time_ns: int, positions: tuple[float, ...] = (0.0,)) -> Target:
    return Target(time_ns, positions)


def joint(
    name: str = "joint1",
    minimum: float = -10.0,
    maximum: float = 10.0,
    max_velocity: float = 10.0,
) -> JointSpec:
    return JointSpec(name, "rad", minimum, maximum, max_velocity, 0.0)


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
    bad_prefix = (
        target(100_000_000, (math.nan,)),
        target(500_000_000, (0.0,)),
        target(1_000_000_000, (0.0,)),
    )
    with pytest.raises(ContractError, match="source_targets\\[0\\]"):
        make_window(target_count=2, handoff_ns=1_500_000_000, source=bad_prefix)

    bad_tail = (
        target(500_000_000, (0.0,)),
        target(1_000_000_000, (0.0,)),
        target(2_000_000_000, (math.inf,)),
    )
    with pytest.raises(ContractError, match="source_targets\\[2\\]"):
        make_window(target_count=2, handoff_ns=1_500_000_000, source=bad_tail)

    uneven = (
        target(500_000_000, (0.0,)),
        target(1_000_000_000, (0.0, 1.0)),
    )
    with pytest.raises(ContractError, match="positions"):
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


def linear_window_and_plan():
    source = (target(500_000_000, (0.5,)), target(1_000_000_000, (1.0,)))
    window = ActionWindow(1, 0, 0, source, 2)
    plan = ReferencePlan(
        ("joint1",),
        (
            CubicSegment(500_000_000, (0.0,), (1.0,), (0.0,), (0.0,)),
            CubicSegment(500_000_000, (0.5,), (1.0,), (0.0,), (0.0,)),
        ),
    )
    return window, plan


def make_prepared(**changes) -> PreparedChunk:
    window, plan = linear_window_and_plan()
    anchor = ChunkAnchor(
        "stream-1",
        3,
        0,
        ReferenceSample((0.0,), (1.0,), (0.0,)),
        "profile-hash",
    )
    arguments = {
        "anchor": anchor,
        "window": window,
        "plan": plan,
        "joints": (joint(),),
        "max_accelerations": (10.0,),
        "final_velocities": (1.0,),
        "settle_at_end": False,
    }
    arguments.update(changes)
    return PreparedChunk(**arguments)  # type: ignore[arg-type]


def test_prepared_chunk_samples_only_within_its_absolute_interval():
    prepared = make_prepared()
    assert prepared.seq == 4
    assert prepared.start_sim_time_ns == 0
    assert prepared.end_sim_time_ns == 1_000_000_000
    assert prepared.sample_at(0) == ReferenceSample((0.0,), (1.0,), (0.0,))
    assert prepared.sample_at(500_000_000) == ReferenceSample((0.5,), (1.0,), (0.0,))
    assert prepared.sample_at(1_000_000_000) == ReferenceSample((1.0,), (1.0,), (0.0,))

    for sim_time in (-1, 1_000_000_001):
        with pytest.raises(ContractError) as caught:
            prepared.sample_at(sim_time)
        assert caught.value.reason == "REFERENCE_EXPIRED"
    for sim_time in (True, 1.0, "0"):
        with pytest.raises(ContractError) as caught:
            prepared.sample_at(sim_time)  # type: ignore[arg-type]
        assert caught.value.reason == "INVALID_PREPARED_CHUNK"


def test_prepared_chunk_direct_construction_rejects_mismatched_plan_and_velocity():
    with pytest.raises(ContractError, match="plan.duration_ns"):
        source = (target(500_000_000, (0.5,)), target(1_500_000_000, (1.5,)))
        make_prepared(window=ActionWindow(1, 0, 0, source, 2))
    with pytest.raises(ContractError, match="end_velocities"):
        make_prepared(final_velocities=(0.5,))
    with pytest.raises(ContractError, match="settle_at_end"):
        make_prepared(final_velocities=(1.0,), settle_at_end=True)
    with pytest.raises(ContractError, match="settle_at_end"):
        make_prepared(settle_at_end=1)  # type: ignore[arg-type]


def test_prepared_chunk_rejects_bad_unselected_prefix_position():
    source = (
        target(250_000_000, (100.0,)),
        target(500_000_000, (0.5,)),
        target(1_000_000_000, (1.0,)),
    )
    window = ActionWindow(1, 0, 300_000_000, source, 2)
    plan = ReferencePlan(
        ("joint1",),
        (
            CubicSegment(200_000_000, (0.0,), (1.0,), (0.0,), (0.0,)),
            CubicSegment(500_000_000, (0.2,), (1.0,), (0.0,), (0.0,)),
        ),
    )
    anchor = ChunkAnchor(
        "stream-1",
        0,
        300_000_000,
        ReferenceSample((0.0,), (1.0,), (0.0,)),
        "profile-hash",
    )
    with pytest.raises(ContractError) as caught:
        PreparedChunk(
            anchor,
            window,
            plan,
            (joint(),),
            (10.0,),
            (1.0,),
            False,
        )
    assert caught.value.reason == "JOINT_LIMIT"


def linear_two_segment_prepared(
    start: float,
    velocity: float,
    *,
    joints: tuple[JointSpec, ...],
    max_acceleration: float = 1.0,
    final_velocities: tuple[float, ...] | None = None,
    settle_at_end: bool = False,
) -> PreparedChunk:
    duration_ns = 500_000_000
    midpoint = start + velocity * 0.5
    end = start + velocity
    plan = ReferencePlan(
        (joints[0].name,),
        (
            CubicSegment(duration_ns, (start,), (velocity,), (0.0,), (0.0,)),
            CubicSegment(duration_ns, (midpoint,), (velocity,), (0.0,), (0.0,)),
        ),
    )
    window = ActionWindow(
        1,
        0,
        0,
        (target(duration_ns, (midpoint,)), target(2 * duration_ns, (end,))),
        2,
    )
    anchor = ChunkAnchor(
        "stream-1",
        0,
        0,
        ReferenceSample((start,), (velocity,), (0.0,)),
        "profile-hash",
    )
    final = (velocity,) if final_velocities is None else final_velocities
    return PreparedChunk(
        anchor,
        window,
        plan,
        joints,
        (max_acceleration,),
        final,
        settle_at_end,
    )


def test_prepared_chunk_rejects_endpoint_that_cannot_brake_without_crossing_limit():
    upper = 10.0
    start = upper - 0.02
    velocity = 0.02
    joints = (joint(maximum=upper),)
    # The pure plan itself is valid: endpoint position/velocity/limits all pass.
    plan = ReferencePlan(
        ("joint1",),
        (
            CubicSegment(500_000_000, (start,), (velocity,), (0.0,), (0.0,)),
            CubicSegment(500_000_000, (start + velocity / 2,), (velocity,), (0.0,), (0.0,)),
        ),
    )
    targets = (
        Target(500_000_000, (start + velocity / 2,)),
        Target(1_000_000_000, (upper,)),
    )
    verify_reference_plan(
        plan,
        (start,),
        targets,
        joints,
        (1.0,),
        initial_velocities=(velocity,),
        final_velocities=(velocity,),
    )
    with pytest.raises(ContractError) as caught:
        PreparedChunk(
            ChunkAnchor(
                "stream-1",
                0,
                0,
                ReferenceSample((start,), (velocity,), (0.0,)),
                "profile-hash",
            ),
            ActionWindow(1, 0, 0, targets, 2),
            plan,
            joints,
            (1.0,),
            (velocity,),
            False,
        )
    assert caught.value.reason == "JOINT_LIMIT"


def test_prepared_chunk_accepts_endpoint_with_sufficient_braking_margin():
    upper = 10.0
    start = upper - 0.021
    velocity = 0.02
    prepared = linear_two_segment_prepared(
        start,
        velocity,
        joints=(joint(maximum=upper),),
    )
    assert prepared.final_velocities == (velocity,)
    assert prepared.sample_at(prepared.end_sim_time_ns).positions[0] == pytest.approx(upper - 0.001)


def test_prepared_chunk_checks_negative_lower_limit_and_gripper_metre_units():
    lower = -10.0
    with pytest.raises(ContractError) as caught:
        linear_two_segment_prepared(
            lower + 0.02,
            -0.02,
            joints=(joint(minimum=lower),),
        )
    assert caught.value.reason == "JOINT_LIMIT"

    gripper = JointSpec("joint1", "m", 0.0, 0.08, 0.04, 0.0)
    with pytest.raises(ContractError) as caught:
        linear_two_segment_prepared(
            0.0780,
            0.002,
            joints=(gripper,),
            max_acceleration=0.5,
        )
    assert caught.value.reason == "JOINT_LIMIT"
    accepted = linear_two_segment_prepared(
        0.0775,
        0.002,
        joints=(gripper,),
        max_acceleration=0.5,
    )
    assert accepted.final_velocities == (0.002,)


def test_settle_zero_speed_at_limit_needs_no_extra_braking_distance():
    upper = 10.0
    prepared = linear_two_segment_prepared(
        upper,
        0.0,
        joints=(joint(maximum=upper),),
        final_velocities=(0.0,),
        settle_at_end=True,
    )
    assert prepared.settle_at_end is True
    assert prepared.sample_at(prepared.end_sim_time_ns).positions == (upper,)


def test_prepared_chunk_requires_positive_acceleration_and_contract_error_metadata():
    with pytest.raises(ContractError) as caught:
        linear_two_segment_prepared(
            0.0,
            0.01,
            joints=(joint(),),
            max_acceleration=0.0,
        )
    assert caught.value.reason == "INVALID_PREPARED_CHUNK"

    bad_joint = JointSpec("joint1", "rad", None, 10.0, 10.0, 0.0)  # type: ignore[arg-type]
    with pytest.raises(ContractError) as caught:
        linear_two_segment_prepared(0.0, 0.01, joints=(bad_joint,))
    assert "joints[0].minimum" in str(caught.value)
