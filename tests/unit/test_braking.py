from __future__ import annotations

import math
import sys
from dataclasses import FrozenInstanceError
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.braking import BrakingPlan, plan_braking  # noqa: E402
from robot_contracts.models import ContractError, JointSpec  # noqa: E402
from robot_contracts.reference import (  # noqa: E402
    JointExtremes,
    ReferenceExtremes,
    check_continuous_limits,
)


def joint(
    name: str,
    minimum: float = -10.0,
    maximum: float = 10.0,
    max_velocity: float = 10.0,
) -> JointSpec:
    return JointSpec(name, "rad", minimum, maximum, max_velocity, 0.0)


def test_single_axis_closed_form_constant_deceleration_and_hold():
    plan = plan_braking((0.0,), (1.0,), (joint("j"),), (2.0,))
    assert isinstance(plan, BrakingPlan)
    assert plan.stop_times_ns == (500_000_000,)
    assert plan.duration_ns == 500_000_000
    assert plan.hold_positions == (pytest.approx(0.25),)

    start = plan.sample(0)
    assert start.positions == (0.0,)
    assert start.velocities == (1.0,)
    assert start.accelerations == (-2.0,)

    middle = plan.sample(250_000_000)
    assert middle.positions == (pytest.approx(0.1875),)
    assert middle.velocities == (pytest.approx(0.5),)
    assert middle.accelerations == (-2.0,)

    before_end = plan.sample(499_999_999)
    assert before_end.velocities[0] == pytest.approx(1e-9, abs=2e-9)

    held = plan.sample(500_000_000)
    assert held.positions == (pytest.approx(0.25),)
    assert held.velocities == (0.0,)
    assert held.accelerations == (0.0,)
    assert plan.sample(10**12) == held


def test_stop_time_rounds_up_and_actual_deceleration_never_exceeds_limit():
    velocity = 1.0 / 3.0
    plan = plan_braking((0.0,), (velocity,), (joint("j"),), (1.0,))
    assert plan.stop_times_ns == (333_333_334,)
    duration_s = plan.stop_times_ns[0] / 1_000_000_000
    actual = velocity / duration_s
    assert actual <= 1.0
    assert plan.hold_positions[0] == pytest.approx(velocity * duration_s / 2.0)
    acceleration = plan.sample(0).accelerations[0]
    assert abs(acceleration) == pytest.approx(actual)
    assert abs(acceleration) <= 1.0


def test_multi_axis_stop_boundaries_merge_into_positive_segments_with_continuous_state():
    plan = plan_braking(
        (0.0, 0.0, 0.0),
        (1.0, 0.5, 0.0),
        (joint("fast"), joint("slow"), joint("still")),
        (1.0, 1.0, 1.0),
    )
    assert plan.stop_times_ns == (1_000_000_000, 500_000_000, 0)
    assert tuple(segment.duration_ns for segment in plan.segments) == (
        500_000_000,
        500_000_000,
    )
    assert all(segment.duration_ns > 0 for segment in plan.segments)
    assert plan.hold_positions[1] == pytest.approx(0.125)
    assert plan.hold_positions[2] == 0.0

    first = plan.sample(0)
    assert first.velocities == (1.0, 0.5, 0.0)
    at_boundary = plan.sample(500_000_000)
    assert at_boundary.positions[1] == pytest.approx(0.125)
    assert at_boundary.velocities[1] == pytest.approx(0.0)
    assert at_boundary.velocities[2] == 0.0
    after_boundary = plan.sample(500_000_001)
    assert after_boundary.positions[1] == pytest.approx(0.125)
    assert after_boundary.velocities[1] == 0.0
    assert after_boundary.velocities[2] == 0.0

    for boundary in (500_000_000,):
        left = plan.sample(boundary - 1)
        right = plan.sample(boundary)
        for index in range(3):
            assert left.positions[index] == pytest.approx(right.positions[index], abs=1e-8)
            assert left.velocities[index] == pytest.approx(right.velocities[index], abs=1e-6)

    final = plan.sample(1_000_000_000)
    assert final.positions == pytest.approx(plan.hold_positions)
    assert final.velocities == (0.0, 0.0, 0.0)
    assert final.accelerations == (0.0, 0.0, 0.0)


def test_all_zero_velocity_uses_explicit_empty_hold_path_not_zero_duration_segment():
    plan = plan_braking((1.0, -2.0), (0.0, 0.0), (joint("a"), joint("b")), (1.0, 1.0))
    assert plan.segments == ()
    assert plan.duration_ns == 0
    assert plan.stop_times_ns == (0, 0)
    assert plan.hold_positions == (1.0, -2.0)
    sample = plan.sample(123)
    assert sample.positions == (1.0, -2.0)
    assert sample.velocities == (0.0, 0.0)
    assert sample.accelerations == (0.0, 0.0)


@pytest.mark.parametrize(
    ("positions", "velocities", "accelerations", "reason"),
    (
        ((9.9,), (1.0,), (1.0,), "JOINT_LIMIT"),
        ((0.0,), (11.0,), (1.0,), "VELOCITY_LIMIT"),
        ((11.0,), (0.0,), (1.0,), "JOINT_LIMIT"),
        ((0.0,), (1.0,), (0.0,), "INVALID_BRAKING_INPUT"),
        ((0.0,), (1.0,), (-1.0,), "INVALID_BRAKING_INPUT"),
        ((0.0,), (1.0,), (math.nan,), "INVALID_BRAKING_INPUT"),
        ((0.0,), (1.0,), (1e-300,), "INVALID_BRAKING_INPUT"),
        ((0.0,), (True,), (1.0,), "INVALID_BRAKING_INPUT"),
    ),
)
def test_invalid_or_limit_overshooting_braking_is_rejected(
    positions, velocities, accelerations, reason
):
    with pytest.raises(ContractError) as caught:
        plan_braking(positions, velocities, (joint("j"),), accelerations)
    assert caught.value.reason == reason


def test_plan_is_immutable_and_has_no_zero_length_segments():
    plan = plan_braking((0.0, 0.0), (1.0, 0.5), (joint("a"), joint("b")), (1.0, 2.0))
    assert isinstance(plan.segments, tuple)
    assert isinstance(plan.hold_positions, tuple)
    with pytest.raises(FrozenInstanceError):
        plan.stop_times_ns = (1, 2)
    with pytest.raises(TypeError):
        plan.hold_positions[0] = 5.0


@pytest.mark.parametrize(
    "extreme",
    [
        JointExtremes("wrong", 0.0, 0.0, 0.0, 0.0),
        JointExtremes("j", math.nan, 0.0, 0.0, 0.0),
        JointExtremes("j", 0.0, 0.0, math.inf, 0.0),
        JointExtremes("j", 0.0, 0.0, -0.1, 0.0),
        JointExtremes("j", 0.0, 0.0, 0.0, True),
        JointExtremes("j", 1.0, 0.0, 0.0, 0.0),
    ],
)
def test_public_continuous_limits_reject_invalid_extrema_and_order(extreme):
    with pytest.raises(ContractError) as caught:
        check_continuous_limits(ReferenceExtremes((extreme,)), (joint("j"),), (1.0,))
    assert caught.value.reason == "INVALID_REFERENCE_PLAN"


def test_public_continuous_limits_reject_nonfinite_joint_limit():
    with pytest.raises(ContractError) as caught:
        check_continuous_limits(
            ReferenceExtremes((JointExtremes("j", 0.0, 0.0, 0.0, 0.0),)),
            (joint("j", minimum=math.nan),),
            (1.0,),
        )
    assert caught.value.reason == "INVALID_REFERENCE_PLAN"


def test_nearly_coincident_stop_times_do_not_amplify_velocity_cancellation():
    """The former final 1 ns residual produced acceleration 1.000000013."""
    velocity = 0.10054068000000001
    plan = plan_braking(
        (0.0, 1.0), (velocity, velocity + 1e-9), (joint("a"), joint("b")), (1.0, 1.0)
    )
    assert plan.stop_times_ns[1] - plan.stop_times_ns[0] == 1
    assert all(abs(2.0 * c2) <= 1.0 + 1e-15 for segment in plan.segments for c2 in segment.c2)
    assert plan.sample(plan.duration_ns).velocities == (0.0, 0.0)


def test_merged_brakes_keep_acceleration_bounded_for_close_stop_times():
    for index in range(500):
        velocity = 0.1 + index * 0.000013517
        plan = plan_braking(
            (0.0, 1.0), (velocity, velocity + 1e-9), (joint("a"), joint("b")), (1.0, 1.0)
        )
        assert all(abs(2.0 * c2) <= 1.0 + 1e-15 for segment in plan.segments for c2 in segment.c2)
