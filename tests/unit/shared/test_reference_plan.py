from __future__ import annotations

import math

import pytest
from model_helpers import replace
from pydantic import ValidationError
from robot_contracts.models import ContractError, JointSpec, Target
from robot_contracts.trajectory.constants import MAX_HORIZON_NS, MAX_JOINTS, MAX_SEGMENTS
from robot_contracts.trajectory.spline import CubicSegment, ReferencePlan, ReferenceSample
from robot_trajectory.sampling import sample_reference_interval, sample_reference_plan
from robot_trajectory.spline_math import (
    compute_reference_extremes,
    validate_reference_inputs,
    validate_velocity_boundaries,
    verify_reference_plan,
)


def segment(
    duration_ns: int = 1_000_000_000,
    c0: tuple[float, ...] = (0.0,),
    c1: tuple[float, ...] = (0.0,),
    c2: tuple[float, ...] = (0.0,),
    c3: tuple[float, ...] = (0.0,),
) -> CubicSegment:
    return CubicSegment(duration_ns, c0, c1, c2, c3)


def joint(
    name: str = "joint1",
    minimum: float = -100.0,
    maximum: float = 100.0,
    max_velocity: float = 100.0,
) -> JointSpec:
    return JointSpec(name, "rad", minimum, maximum, max_velocity, 0.0)


def split_smoothstep_plan() -> ReferencePlan:
    """Position 0 -> 1 over 2 s, represented as two continuous segments."""
    return ReferencePlan(
        ("joint1",),
        (
            CubicSegment(1_000_000_000, (0.0,), (0.0,), (0.75,), (-0.25,)),
            CubicSegment(1_000_000_000, (0.5,), (0.75,), (0.0,), (-0.25,)),
        ),
    )


def split_smoothstep_arguments():
    plan = split_smoothstep_plan()
    return (
        plan,
        (0.0,),
        (Target(1_000_000_000, (0.5,)), Target(2_000_000_000, (1.0,))),
        (joint(),),
        (10.0,),
    )


def test_single_segment_closed_form_sample_and_hold():
    plan = ReferencePlan(
        ("joint1",),
        (CubicSegment(1_000_000_000, (1.0,), (2.0,), (3.0,), (4.0,)),),
    )

    start = sample_reference_plan(plan, 0)
    assert start == ReferenceSample((1.0,), (2.0,), (6.0,))

    middle = sample_reference_plan(plan, 500_000_000)
    assert middle == ReferenceSample((3.25,), (8.0,), (18.0,))

    end = sample_reference_plan(plan, 1_000_000_000)
    assert end == ReferenceSample((10.0,), (20.0,), (30.0,))

    held = sample_reference_plan(plan, 1_000_000_001)
    assert held == ReferenceSample((10.0,), (0.0,), (0.0,))
    assert sample_reference_plan(plan, 10**100) == held


def test_constant_multi_axis_plan_and_internal_boundary_choice():
    first = CubicSegment(1_000_000_000, (1.0, -2.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0))
    second = CubicSegment(2_000_000_000, (5.0, 7.0), (2.0, -3.0), (0.0, 0.0), (0.0, 0.0))
    plan = ReferencePlan(("left", "right"), (first, second))

    assert plan.duration_ns == 3_000_000_000
    assert sample_reference_plan(plan, 0) == ReferenceSample((1.0, -2.0), (0.0, 0.0), (0.0, 0.0))
    assert sample_reference_plan(plan, 999_999_999).positions == (1.0, -2.0)
    assert sample_reference_plan(plan, 1_000_000_000) == ReferenceSample(
        (1.0, -2.0), (0.0, 0.0), (0.0, 0.0)
    )
    after_boundary = sample_reference_plan(plan, 1_000_000_001)
    assert after_boundary.positions[0] == pytest.approx(5.0 + 2e-9)
    assert after_boundary.velocities == pytest.approx((2.0, -3.0))
    irregular = sample_reference_plan(plan, 2_500_000_000)
    assert irregular.positions == pytest.approx((5.0 + 2.0 * 1.5, 7.0 - 3.0 * 1.5))
    assert irregular.velocities == pytest.approx((2.0, -3.0))


def test_plan_and_segment_are_deeply_immutable():
    c0 = [1.0]
    c1 = [2.0]
    c2 = [3.0]
    c3 = [4.0]
    first = CubicSegment(1_000_000_000, c0, c1, c2, c3)
    c0[0] = 99.0
    c1[0] = 99.0
    c2[0] = 99.0
    c3[0] = 99.0
    assert first == CubicSegment(1_000_000_000, (1.0,), (2.0,), (3.0,), (4.0,))

    segments = [first]
    plan = ReferencePlan(["joint1"], segments)
    segments.clear()
    assert plan.segments == (first,)
    sample = sample_reference_plan(plan, 0)
    assert isinstance(sample.positions, tuple)
    with pytest.raises(ValidationError):
        sample.positions = (0.0,)  # type: ignore[misc]


@pytest.mark.parametrize(
    "duration_ns",
    [True, False, 0, -1, 1.0, "1", MAX_HORIZON_NS + 1],
)
def test_invalid_segment_duration_is_a_contract_error(duration_ns):
    with pytest.raises(ContractError) as caught:
        CubicSegment(duration_ns, (0.0,), (0.0,), (0.0,), (0.0,))
    assert caught.value.reason == "INVALID_REFERENCE_PLAN"


@pytest.mark.parametrize(
    "coefficients",
    [
        (True,),
        ("1",),
        (math.nan,),
        (math.inf,),
        (-math.inf,),
        (10**400,),
        (),
        tuple(0.0 for _ in range(MAX_JOINTS + 1)),
    ],
)
def test_invalid_coefficients_are_contract_errors_without_overflow(coefficients):
    with pytest.raises(ContractError) as caught:
        CubicSegment(1, coefficients, (0.0,), (0.0,), (0.0,))
    assert caught.value.reason == "INVALID_REFERENCE_PLAN"


def test_segment_coefficient_arrays_must_have_equal_lengths():
    with pytest.raises(ContractError, match="c1"):
        CubicSegment(1, (0.0,), (0.0, 0.0), (0.0,), (0.0,))


def test_invalid_plan_names_segments_and_horizon():
    with pytest.raises(ContractError, match="joint_names"):
        ReferencePlan((), (segment(),))
    with pytest.raises(ContractError, match="joint_names"):
        ReferencePlan(
            ("joint1", "joint1"),
            (segment(c0=(0.0, 0.0), c1=(0.0, 0.0), c2=(0.0, 0.0), c3=(0.0, 0.0)),),
        )
    with pytest.raises(ContractError, match="joint_names"):
        ReferencePlan(("",), (segment(),))
    with pytest.raises(ContractError, match="joint_names"):
        ReferencePlan((1,), (segment(),))  # type: ignore[arg-type]
    with pytest.raises(ContractError, match="joint_names"):
        ReferencePlan("joint1", (segment(),))  # type: ignore[arg-type]
    with pytest.raises(ContractError, match="segments"):
        ReferencePlan(("joint1",), ())
    with pytest.raises(ContractError, match="segments"):
        ReferencePlan(("joint1",), tuple(segment() for _ in range(MAX_SEGMENTS + 1)))
    with pytest.raises(ContractError, match="segments"):
        ReferencePlan(("joint1", "joint2"), (segment(),))
    with pytest.raises(ContractError, match="horizon"):
        ReferencePlan(
            ("joint1",),
            (segment(1_500_000_000), segment(1_500_000_001)),
        )
    with pytest.raises(ContractError, match="segments"):
        ReferencePlan(("joint1",), (object(),))  # type: ignore[arg-type]


def test_plan_accepts_exact_limits_and_max_segment_count():
    plan = ReferencePlan(("joint1",), (segment(MAX_HORIZON_NS),))
    assert plan.duration_ns == MAX_HORIZON_NS
    many = ReferencePlan(("joint1",), tuple(segment(1) for _ in range(MAX_SEGMENTS)))
    assert len(many.segments) == MAX_SEGMENTS


@pytest.mark.parametrize("elapsed_ns", [-1, True, False, 1.0, "0", None])
def test_sample_rejects_non_integer_or_negative_elapsed(elapsed_ns):
    plan = ReferencePlan(("joint1",), (segment(),))
    with pytest.raises(ContractError) as caught:
        sample_reference_plan(plan, elapsed_ns)  # type: ignore[arg-type]
    assert caught.value.reason == "INVALID_REFERENCE_PLAN"


def test_input_validation_normalizes_and_rejects_bad_targets():
    inputs = validate_reference_inputs(
        [0.0],
        [Target(1_000_000_000, [1.0])],
        [joint()],
        [10.0],
    )
    assert inputs.initial_positions == (0.0,)
    assert inputs.target_positions == ((1.0,),)

    cases = (
        ([0.0], [Target(1, (1.0,)), Target(1, (1.0,))], [joint()], [10.0]),
        ([0.0], [Target(1, (1.0,)), Target(2, (1.0,)), Target(1, (1.0,))], [joint()], [10.0]),
        ([0.0], [Target(MAX_HORIZON_NS + 1, (1.0,))], [joint()], [10.0]),
        ([0.0], [Target(1, (1.0, 2.0))], [joint()], [10.0]),
        ([0.0, 0.0], [Target(1, (1.0,))], [joint()], [10.0]),
        ([0.0], [Target(1, (1.0,))], [joint()], [10.0, 10.0]),
        ([0.0], [Target(1, (1.0,))], [joint()], [math.inf]),
        ([0.0], [Target(1, (1.0,))], [joint()], [-1.0]),
        ([0.0], [Target(1, (1.0,))], [joint(maximum=0.5)], [10.0]),
        ([0.0], [Target(1, (-1.0,))], [joint(minimum=-0.5)], [10.0]),
        ([0.0], [Target(1, (1.0,))], [joint(maximum=0.5)], [10.0]),
        ([0.0], [Target(1, (1.0,))], [], [10.0]),
        ([0.0], [Target(1, (1.0,))], [joint(), joint()], [10.0]),
        ([0.0], [Target(1, (1.0,))], [joint(name="a"), joint(name="a")], [10.0]),
    )
    for initial, targets, joints, accelerations in cases:
        with pytest.raises(ContractError):
            validate_reference_inputs(initial, targets, joints, accelerations)

    for invalid in (
        lambda: Target(0, (1.0,)),
        lambda: Target(True, (1.0,)),
        lambda: Target(1, (math.nan,)),
        lambda: joint(maximum=0.5, minimum=1.0),
        lambda: joint(max_velocity=0.0),
    ):
        with pytest.raises(ContractError):
            invalid()


def test_verify_accepts_continuous_split_smoothstep_and_returns_extrema():
    plan, initial, targets, joints, accelerations = split_smoothstep_arguments()
    extremes = verify_reference_plan(plan, initial, targets, joints, accelerations)
    assert extremes.joint_names == ("joint1",)
    joint_extremes = extremes.joints[0]
    assert joint_extremes.minimum_position == pytest.approx(0.0, abs=1e-12)
    assert joint_extremes.maximum_position == pytest.approx(1.0, abs=1e-12)
    assert joint_extremes.maximum_velocity == pytest.approx(0.75, abs=1e-12)
    assert joint_extremes.maximum_acceleration == pytest.approx(1.5, abs=1e-12)


def test_verify_rejects_internal_position_discontinuity():
    plan, initial, targets, joints, accelerations = split_smoothstep_arguments()
    broken = ReferencePlan(
        plan.joint_names,
        (plan.segments[0], replace(plan.segments[1], c0=(0.5001,))),
    )
    with pytest.raises(ContractError) as caught:
        verify_reference_plan(broken, initial, targets, joints, accelerations)
    assert caught.value.reason == "INVALID_REFERENCE_PLAN"
    assert "position" in str(caught.value)


def test_verify_rejects_internal_velocity_discontinuity_with_endpoints_intact():
    plan, initial, targets, joints, accelerations = split_smoothstep_arguments()
    delta = 1e-4
    second = CubicSegment(
        1_000_000_000,
        (0.5,),
        (0.75 + delta,),
        (-2.0 * delta,),
        (-0.25 + delta,),
    )
    broken = ReferencePlan(plan.joint_names, (plan.segments[0], second))
    with pytest.raises(ContractError) as caught:
        verify_reference_plan(broken, initial, targets, joints, accelerations)
    assert caught.value.reason == "INVALID_REFERENCE_PLAN"
    assert "velocity" in str(caught.value)


def test_verify_rejects_internal_acceleration_discontinuity_with_endpoints_intact():
    plan, initial, targets, joints, accelerations = split_smoothstep_arguments()
    epsilon = 1e-4
    second = CubicSegment(
        1_000_000_000,
        (0.5,),
        (0.75,),
        (epsilon,),
        (-0.25 - 2.0 * epsilon / 3.0,),
    )
    broken = ReferencePlan(plan.joint_names, (plan.segments[0], second))
    broken_targets = (
        Target(1_000_000_000, (0.5,)),
        Target(2_000_000_000, (1.0 + epsilon / 3.0,)),
    )
    with pytest.raises(ContractError) as caught:
        verify_reference_plan(broken, initial, broken_targets, joints, accelerations)
    assert caught.value.reason == "INVALID_REFERENCE_PLAN"
    assert "acceleration" in str(caught.value)


def test_verify_rejects_segment_durations_that_do_not_match_waypoint_times():
    plan, initial, targets, joints, accelerations = split_smoothstep_arguments()
    broken = ReferencePlan(
        plan.joint_names,
        (
            replace(plan.segments[0], duration_ns=1_500_000_000),
            replace(plan.segments[1], duration_ns=500_000_000),
        ),
    )
    with pytest.raises(ContractError, match="duration_ns"):
        verify_reference_plan(broken, initial, targets, joints, accelerations)


def test_verify_checks_initial_waypoints_times_and_horizon():
    plan, initial, targets, joints, accelerations = split_smoothstep_arguments()

    with pytest.raises(ContractError):
        verify_reference_plan(plan, (0.1,), targets, joints, accelerations)

    with pytest.raises(ContractError):
        verify_reference_plan(
            plan,
            initial,
            (Target(1_000_000_000, (0.5,)), Target(2_000_000_001, (1.0,))),
            joints,
            accelerations,
        )

    with pytest.raises(ContractError):
        verify_reference_plan(plan, initial, targets[:1], joints, accelerations)

    with pytest.raises(ContractError):
        verify_reference_plan(
            plan,
            initial,
            (Target(1_000_000_000, (0.5,)), Target(2_000_000_000, (0.9,))),
            joints,
            accelerations,
        )

    with pytest.raises(ContractError):
        verify_reference_plan(
            plan,
            initial,
            (Target(2_000_000_000, (0.5,)), Target(1_000_000_000, (1.0,))),
            joints,
            accelerations,
        )


def test_verify_rejects_nonzero_endpoint_velocity():
    plan = ReferencePlan(
        ("joint1",),
        (CubicSegment(1_000_000_000, (0.0,), (0.1,), (0.0,), (0.0,)),),
    )
    with pytest.raises(ContractError, match="velocity"):
        verify_reference_plan(
            plan,
            (0.0,),
            (Target(1_000_000_000, (0.1,)),),
            (joint(),),
            (10.0,),
        )


def test_waypoints_within_limits_but_internal_position_overshoot_is_rejected():
    first = CubicSegment(1_000_000_000, (0.0,), (0.0,), (10.0,), (-20.0,))
    second = CubicSegment(
        1_000_000_000,
        (-10.0,),
        (-40.0,),
        (-50.0,),
        (140.0 / 3.0,),
    )
    plan = ReferencePlan(("joint1",), (first, second))
    targets = (Target(1_000_000_000, (-10.0,)), Target(2_000_000_000, (-160.0 / 3.0,)))
    constrained = joint(minimum=-60.0, maximum=0.2, max_velocity=100.0)

    assert 10.0 / 27.0 > constrained.maximum
    extremes = compute_reference_extremes(plan)
    assert extremes.joints[0].maximum_position == pytest.approx(10.0 / 27.0)
    with pytest.raises(ContractError) as caught:
        verify_reference_plan(plan, (0.0,), targets, (constrained,), (250.0,))
    assert caught.value.reason == "JOINT_LIMIT"


def test_velocity_and_acceleration_peaks_are_limited():
    plan = ReferencePlan(
        ("joint1",),
        (CubicSegment(1_000_000_000, (0.0,), (0.0,), (3.0,), (-2.0,)),),
    )
    targets = (Target(1_000_000_000, (1.0,)),)
    extremes = compute_reference_extremes(plan)
    assert extremes.joints[0].maximum_velocity == pytest.approx(1.5)
    assert extremes.joints[0].maximum_acceleration == pytest.approx(6.0)

    with pytest.raises(ContractError) as caught:
        verify_reference_plan(
            plan,
            (0.0,),
            targets,
            (joint(max_velocity=1.0),),
            (100.0,),
        )
    assert caught.value.reason == "VELOCITY_LIMIT"

    with pytest.raises(ContractError) as caught:
        verify_reference_plan(
            plan,
            (0.0,),
            targets,
            (joint(max_velocity=10.0),),
            (5.9,),
        )
    assert caught.value.reason == "ACCELERATION_LIMIT"


def test_extrema_handle_constant_linear_and_near_zero_coefficients():
    constant = ReferencePlan(
        ("joint1", "joint2"),
        (CubicSegment(2_000_000_000, (1.5, -2.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)),),
    )
    constant_extremes = compute_reference_extremes(constant)
    assert constant_extremes.joints[0].minimum_position == pytest.approx(1.5)
    assert constant_extremes.joints[0].maximum_position == pytest.approx(1.5)
    assert constant_extremes.joints[0].maximum_velocity == 0.0
    assert constant_extremes.joints[0].maximum_acceleration == 0.0

    linear = ReferencePlan(
        ("joint1",),
        (CubicSegment(2_000_000_000, (0.0,), (1.0,), (0.0,), (0.0,)),),
    )
    linear_extremes = compute_reference_extremes(linear)
    assert linear_extremes.joints[0].minimum_position == pytest.approx(0.0)
    assert linear_extremes.joints[0].maximum_position == pytest.approx(2.0)
    assert linear_extremes.joints[0].maximum_velocity == pytest.approx(1.0)
    assert linear_extremes.joints[0].maximum_acceleration == pytest.approx(0.0)

    near_zero = ReferencePlan(
        ("joint1",),
        (CubicSegment(1, (0.0,), (1e-15,), (1e-15,), (1e-15,)),),
    )
    near_extremes = compute_reference_extremes(near_zero)
    assert math.isfinite(near_extremes.joints[0].maximum_velocity)


def test_nonfinite_polynomial_values_are_clear_failures():
    plan = ReferencePlan(
        ("joint1",),
        (CubicSegment(1_000_000_000, (0.0,), (0.0,), (0.0,), (1e308,)),),
    )
    with pytest.raises(ContractError, match="not finite"):
        compute_reference_extremes(plan)
    with pytest.raises(ContractError, match="not finite"):
        sample_reference_plan(plan, 1_000_000_000)


def test_verify_accepts_declared_nonzero_polynomial_velocity_boundaries():
    plan = ReferencePlan(
        ("joint1",),
        (CubicSegment(1_000_000_000, (1.0,), (2.0,), (3.0,), (4.0,)),),
    )
    targets = (Target(1_000_000_000, (10.0,)),)
    extremes = verify_reference_plan(
        plan,
        (1.0,),
        targets,
        (joint(max_velocity=100.0),),
        (100.0,),
        initial_velocities=(2.0,),
        final_velocities=(20.0,),
    )
    assert extremes.joints[0].maximum_velocity == pytest.approx(20.0)
    assert extremes.joints[0].maximum_acceleration == pytest.approx(30.0)
    assert sample_reference_interval(plan, 1_000_000_000).velocities == pytest.approx((20.0,))

    with pytest.raises(ContractError, match="start_velocities"):
        verify_reference_plan(
            plan,
            (1.0,),
            targets,
            (joint(max_velocity=100.0),),
            (100.0,),
            initial_velocities=(1.9,),
            final_velocities=(20.0,),
        )
    with pytest.raises(ContractError, match="end_velocities"):
        verify_reference_plan(
            plan,
            (1.0,),
            targets,
            (joint(max_velocity=100.0),),
            (100.0,),
            initial_velocities=(2.0,),
            final_velocities=(19.0,),
        )
    # No declaration still means the historical zero-velocity contract.
    with pytest.raises(ContractError, match="velocity"):
        verify_reference_plan(
            plan,
            (1.0,),
            targets,
            (joint(max_velocity=100.0),),
            (100.0,),
        )


def test_velocity_boundary_validation_reuses_strict_vector_rules_and_limits():
    joints = (joint(max_velocity=2.0), joint(name="joint2", max_velocity=3.0))
    assert validate_velocity_boundaries(None, joints, "v") == (0.0, 0.0)
    assert validate_velocity_boundaries([2.0, -3.0], joints, "v") == (2.0, -3.0)

    cases = (
        ((False, 0.0), "INVALID_REFERENCE_PLAN"),
        (("1", 0.0), "INVALID_REFERENCE_PLAN"),
        ((math.nan, 0.0), "INVALID_REFERENCE_PLAN"),
        ((math.inf, 0.0), "INVALID_REFERENCE_PLAN"),
        ((10**400, 0.0), "INVALID_REFERENCE_PLAN"),
        ((1.0,), "INVALID_REFERENCE_PLAN"),
        ((1.0, 2.0, 3.0), "INVALID_REFERENCE_PLAN"),
        ((2.000001, 0.0), "VELOCITY_LIMIT"),
        ((0.0, -3.000001), "VELOCITY_LIMIT"),
    )
    for value, reason in cases:
        with pytest.raises(ContractError) as caught:
            validate_velocity_boundaries(value, joints, "v")
        assert caught.value.reason == reason


def test_validate_reference_inputs_normalizes_declared_velocities():
    initial_velocities = [0.25]
    final_velocities = [-0.5]
    inputs = validate_reference_inputs(
        [0.0],
        [Target(1_000_000_000, [1.0])],
        [joint(max_velocity=1.0)],
        [10.0],
        initial_velocities=initial_velocities,
        final_velocities=final_velocities,
    )
    initial_velocities[0] = 99.0
    final_velocities[0] = 99.0
    assert inputs.initial_velocities == (0.25,)
    assert inputs.final_velocities == (-0.5,)


def test_sample_interval_is_strict_and_never_holds_a_nonzero_end():
    plan = ReferencePlan(
        ("joint1",),
        (CubicSegment(1_000_000_000, (1.0,), (2.0,), (3.0,), (4.0,)),),
    )
    assert sample_reference_interval(plan, 0) == sample_reference_plan(plan, 0)
    assert sample_reference_interval(plan, 1_000_000_000) == sample_reference_plan(
        plan, 1_000_000_000
    )
    assert sample_reference_interval(plan, 1_000_000_000).velocities == pytest.approx((20.0,))

    for elapsed in (1_000_000_001, -1):
        with pytest.raises(ContractError) as caught:
            sample_reference_interval(plan, elapsed)
        assert caught.value.reason == "REFERENCE_EXPIRED"
    for elapsed in (True, 1.0, "0"):
        with pytest.raises(ContractError) as caught:
            sample_reference_interval(plan, elapsed)  # type: ignore[arg-type]
        assert caught.value.reason == "INVALID_REFERENCE_PLAN"
