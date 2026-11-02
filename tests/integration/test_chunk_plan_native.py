from __future__ import annotations

import sys
from pathlib import Path

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("scipy")

ROOT = Path(__file__).resolve().parents[2]
for source in (ROOT / "components/backend_sim/src", ROOT / "contracts/python/src"):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

from robot_backend_sim.chunk_plan import PreparedChunk, prepare_chunk  # noqa: E402
from robot_backend_sim.motion_config import load_control_config  # noqa: E402
from robot_backend_sim.reference_compiler import compile_reference_plan  # noqa: E402
from robot_contracts.action_chunks import ActionWindow, ChunkAnchor  # noqa: E402
from robot_contracts.models import ContractError, Target  # noqa: E402
from robot_contracts.profiles import load_profile  # noqa: E402
from robot_contracts.reference import verify_reference_plan  # noqa: E402

RIGHT_JOINT1 = 8
LEFT_ARM = tuple(range(7))
GRIPPERS = (7, 15)
DELTA = 0.15
DURATION_NS = 3_000_000_000
SAMPLE_NS = 50_000_000


def smoothstep_offset(time_s: float) -> float:
    u = time_s / 3.0
    return DELTA * (3.0 * u * u - 2.0 * u * u * u)


def smoothstep_velocity(time_s: float) -> float:
    u = time_s / 3.0
    return 0.3 * u * (1.0 - u)


def profile_and_config():
    profile = load_profile(
        ROOT / "profiles/robots/panda_stack_v1/profile.json",
        ROOT / "contracts/schemas",
    )
    config = load_control_config(ROOT / "configs/simulation/panda_control.yaml")
    return profile, config


def original_plan(profile, config):
    initial = profile.initial_positions
    targets = tuple(
        Target(
            time_ns,
            tuple(
                value + smoothstep_offset(time_ns / 1_000_000_000)
                if index == RIGHT_JOINT1
                else value
                for index, value in enumerate(initial)
            ),
        )
        for time_ns in range(SAMPLE_NS, DURATION_NS + 1, SAMPLE_NS)
    )
    return compile_reference_plan(initial, targets, profile.joints, config.max_accelerations)


def source_predictions(plan):
    return tuple(
        Target(time_ns, plan.sample_interval(time_ns).positions)
        for time_ns in range(SAMPLE_NS, DURATION_NS + 1, SAMPLE_NS)
    )


def chunk(plan, profile, index: int, *, target_count: int = 20):
    handoff_ns = index * 1_000_000_000
    return (
        ActionWindow(
            index + 1,
            0,
            handoff_ns,
            source_predictions(plan),
            target_count,
        ),
        ChunkAnchor(
            "stream-1",
            index,
            handoff_ns,
            plan.sample_interval(handoff_ns),
            profile.digest,
        ),
    )


def test_three_one_second_chunks_recover_the_original_curve_and_limits():
    profile, config = profile_and_config()
    plan = original_plan(profile, config)
    for index in range(3):
        window, anchor = chunk(plan, profile, index)
        end_ns = (index + 1) * 1_000_000_000
        final = plan.sample_interval(end_ns).velocities
        prepared = prepare_chunk(
            anchor,
            window,
            profile,
            config,
            final_velocities=final,
        )
        assert prepared.duration_ns == 1_000_000_000
        assert prepared.seq == index + 1
        for local_ns in range(0, 1_000_000_001, 10_000_000):
            absolute_ns = prepared.start_sim_time_ns + local_ns
            actual = prepared.sample_at(absolute_ns)
            expected = plan.sample_interval(absolute_ns)
            np.testing.assert_allclose(actual.positions, expected.positions, atol=1e-9, rtol=0)
            np.testing.assert_allclose(actual.velocities, expected.velocities, atol=1e-9, rtol=0)
            np.testing.assert_allclose(
                actual.accelerations, expected.accelerations, atol=1e-9, rtol=0
            )

        verify_reference_plan(
            prepared.plan,
            anchor.reference.positions,
            window.targets,
            profile.joints,
            config.max_accelerations,
            initial_velocities=anchor.reference.velocities,
            final_velocities=final,
        )

        # The old per-chunk clamped boundary is still rejected by the same limits.
        with pytest.raises(ContractError) as caught:
            compile_reference_plan(
                anchor.reference.positions,
                window.targets,
                profile.joints,
                config.max_accelerations,
            )
        assert caught.value.reason in {"ACCELERATION_LIMIT", "VELOCITY_LIMIT"}


def test_panda_sixteen_dimensions_left_arm_and_grippers_stay_static():
    profile, config = profile_and_config()
    plan = original_plan(profile, config)
    window, anchor = chunk(plan, profile, 0)
    prepared = prepare_chunk(
        anchor,
        window,
        profile,
        config,
        final_velocities=plan.sample_interval(1_000_000_000).velocities,
    )

    assert prepared.joint_count == 16
    sample = prepared.sample_at(500_000_000)
    for index in LEFT_ARM:
        assert sample.positions[index] == pytest.approx(profile.initial_positions[index], abs=1e-12)
        assert sample.velocities[index] == pytest.approx(0.0, abs=1e-12)
    for index in GRIPPERS:
        assert sample.positions[index] == pytest.approx(profile.initial_positions[index], abs=1e-12)
        assert sample.velocities[index] == pytest.approx(0.0, abs=1e-12)
    assert sample.positions[RIGHT_JOINT1] == pytest.approx(
        profile.initial_positions[RIGHT_JOINT1] + smoothstep_offset(0.5),
        abs=1e-9,
    )
    assert sample.velocities[RIGHT_JOINT1] == pytest.approx(smoothstep_velocity(0.5), abs=1e-9)
    for index in (*LEFT_ARM, *GRIPPERS):
        assert prepared.final_velocities[index] == pytest.approx(0.0, abs=1e-12)


def test_default_final_velocity_is_backward_difference_of_selected_points():
    profile, config = profile_and_config()
    plan = original_plan(profile, config)
    window, anchor = chunk(plan, profile, 0)
    prepared = prepare_chunk(anchor, window, profile, config)
    last, previous = window.targets[-1], window.targets[-2]
    delta_s = (last.time_ns - previous.time_ns) / 1_000_000_000
    expected = tuple(
        (right - left) / delta_s
        for left, right in zip(previous.positions, last.positions, strict=True)
    )
    assert prepared.final_velocities == pytest.approx(expected)
    assert prepared.final_velocities[RIGHT_JOINT1] == pytest.approx(
        plan.sample_interval(1_000_000_000).velocities[RIGHT_JOINT1],
        abs=5e-3,
    )


def test_settle_conflict_and_explicit_velocity_limit_are_rejected():
    profile, config = profile_and_config()
    plan = original_plan(profile, config)
    window, anchor = chunk(plan, profile, 0)
    final = plan.sample_interval(1_000_000_000).velocities
    assert abs(final[RIGHT_JOINT1]) > 1e-6
    with pytest.raises(ContractError, match="settle_at_end"):
        prepare_chunk(
            anchor,
            window,
            profile,
            config,
            final_velocities=final,
            settle_at_end=True,
        )

    too_fast = list(final)
    too_fast[RIGHT_JOINT1] = profile.joints[RIGHT_JOINT1].max_velocity + 1.0
    with pytest.raises(ContractError) as caught:
        prepare_chunk(
            anchor,
            window,
            profile,
            config,
            final_velocities=tuple(too_fast),
        )
    assert caught.value.reason == "VELOCITY_LIMIT"


def test_settle_at_end_uses_zero_final_velocity_for_a_stopping_chunk():
    profile, config = profile_and_config()
    plan = original_plan(profile, config)
    window, anchor = chunk(plan, profile, 2)
    prepared = prepare_chunk(anchor, window, profile, config, settle_at_end=True)
    assert prepared.settle_at_end is True
    assert prepared.final_velocities == (0.0,) * 16
    end = prepared.sample_at(prepared.end_sim_time_ns)
    np.testing.assert_allclose(end.velocities, np.zeros(16), atol=1e-12, rtol=0)


def test_bad_unselected_prefix_is_rejected_before_compilation():
    profile, config = profile_and_config()
    plan = original_plan(profile, config)
    window, anchor = chunk(plan, profile, 1)
    source = list(window.source_targets)
    first = source[0]
    positions = list(first.positions)
    positions[0] = profile.joints[0].maximum + 1.0
    source[0] = Target(first.time_ns, tuple(positions))
    bad_window = ActionWindow(
        window.observation_id,
        window.observation_time_ns,
        window.handoff_time_ns,
        tuple(source),
        window.target_count,
    )
    with pytest.raises(ContractError) as caught:
        prepare_chunk(
            anchor,
            bad_window,
            profile,
            config,
            final_velocities=plan.sample_interval(2_000_000_000).velocities,
        )
    assert caught.value.reason == "JOINT_LIMIT"


def test_preparation_is_pure_and_samples_do_not_mutate_the_chunk():
    profile, config = profile_and_config()
    plan = original_plan(profile, config)
    window, anchor = chunk(plan, profile, 0)
    prepared = prepare_chunk(
        anchor,
        window,
        profile,
        config,
        final_velocities=plan.sample_interval(1_000_000_000).velocities,
    )
    before = (prepared.plan, prepared.anchor, prepared.window, prepared.final_velocities)
    prepared.sample_at(500_000_000)
    assert (prepared.plan, prepared.anchor, prepared.window, prepared.final_velocities) == before
    assert prepared.anchor.reference == anchor.reference


def linear_targets_for_axis(profile, index: int, start: float, velocity: float):
    initial = list(profile.initial_positions)
    initial[index] = start
    initial = tuple(initial)
    targets = tuple(
        Target(
            time_ns,
            tuple(
                start + velocity * (time_ns / 1_000_000_000) if axis == index else value
                for axis, value in enumerate(initial)
            ),
        )
        for time_ns in range(SAMPLE_NS, 1_000_000_001, SAMPLE_NS)
    )
    velocities = tuple(velocity if axis == index else 0.0 for axis in range(16))
    return initial, targets, velocities


def window_and_anchor(plan, targets, profile):
    return (
        ActionWindow(1, 0, 0, targets, len(targets)),
        ChunkAnchor("stream-1", 0, 0, plan.sample_interval(0), profile.digest),
    )


def test_pure_compile_accepts_end_velocity_that_cannot_be_braked():
    profile, config = profile_and_config()
    upper = profile.joints[RIGHT_JOINT1].maximum
    initial, targets, velocities = linear_targets_for_axis(
        profile,
        RIGHT_JOINT1,
        upper - 0.02,
        0.02,
    )
    plan = compile_reference_plan(
        initial,
        targets,
        profile.joints,
        config.max_accelerations,
        initial_velocities=velocities,
        final_velocities=velocities,
    )
    # The curve itself passes the original position/velocity/acceleration limits.
    verify_reference_plan(
        plan,
        initial,
        targets,
        profile.joints,
        config.max_accelerations,
        initial_velocities=velocities,
        final_velocities=velocities,
    )
    window, anchor = window_and_anchor(plan, targets, profile)

    with pytest.raises(ContractError) as caught:
        prepare_chunk(anchor, window, profile, config, final_velocities=velocities)
    assert caught.value.reason == "JOINT_LIMIT"

    # Direct construction uses the same validation path and cannot bypass this.
    with pytest.raises(ContractError) as caught:
        PreparedChunk(
            anchor,
            window,
            plan,
            tuple(profile.joints),
            tuple(config.max_accelerations),
            velocities,
            False,
        )
    assert caught.value.reason == "JOINT_LIMIT"


def test_endpoint_with_sufficient_braking_margin_is_accepted():
    profile, config = profile_and_config()
    upper = profile.joints[RIGHT_JOINT1].maximum
    initial, targets, velocities = linear_targets_for_axis(
        profile,
        RIGHT_JOINT1,
        upper - 0.021,
        0.02,
    )
    plan = compile_reference_plan(
        initial,
        targets,
        profile.joints,
        config.max_accelerations,
        initial_velocities=velocities,
        final_velocities=velocities,
    )
    window, anchor = window_and_anchor(plan, targets, profile)
    prepared = prepare_chunk(anchor, window, profile, config, final_velocities=velocities)
    assert prepared.sample_at(prepared.end_sim_time_ns).positions[RIGHT_JOINT1] == pytest.approx(
        upper - 0.001
    )


def test_negative_lower_limit_requires_braking_margin():
    profile, config = profile_and_config()
    lower = profile.joints[RIGHT_JOINT1].minimum
    initial, targets, velocities = linear_targets_for_axis(
        profile,
        RIGHT_JOINT1,
        lower + 0.02,
        -0.02,
    )
    plan = compile_reference_plan(
        initial,
        targets,
        profile.joints,
        config.max_accelerations,
        initial_velocities=velocities,
        final_velocities=velocities,
    )
    verify_reference_plan(
        plan,
        initial,
        targets,
        profile.joints,
        config.max_accelerations,
        initial_velocities=velocities,
        final_velocities=velocities,
    )
    window, anchor = window_and_anchor(plan, targets, profile)
    with pytest.raises(ContractError) as caught:
        prepare_chunk(anchor, window, profile, config, final_velocities=velocities)
    assert caught.value.reason == "JOINT_LIMIT"


def test_gripper_metre_units_use_the_configured_braking_limit():
    profile, config = profile_and_config()
    gripper = 15
    upper = profile.joints[gripper].maximum
    initial, targets, velocities = linear_targets_for_axis(
        profile,
        gripper,
        upper - 0.002,
        0.002,
    )
    plan = compile_reference_plan(
        initial,
        targets,
        profile.joints,
        config.max_accelerations,
        initial_velocities=velocities,
        final_velocities=velocities,
    )
    window, anchor = window_and_anchor(plan, targets, profile)
    with pytest.raises(ContractError) as caught:
        prepare_chunk(anchor, window, profile, config, final_velocities=velocities)
    assert caught.value.reason == "JOINT_LIMIT"

    initial, targets, velocities = linear_targets_for_axis(
        profile,
        gripper,
        upper - 0.0025,
        0.002,
    )
    plan = compile_reference_plan(
        initial,
        targets,
        profile.joints,
        config.max_accelerations,
        initial_velocities=velocities,
        final_velocities=velocities,
    )
    window, anchor = window_and_anchor(plan, targets, profile)
    prepared = prepare_chunk(anchor, window, profile, config, final_velocities=velocities)
    assert prepared.sample_at(prepared.end_sim_time_ns).positions[gripper] == pytest.approx(
        upper - 0.0005
    )


def test_settle_at_limit_with_zero_velocity_needs_no_extra_distance():
    profile, config = profile_and_config()
    upper = profile.joints[RIGHT_JOINT1].maximum
    initial, targets, velocities = linear_targets_for_axis(
        profile,
        RIGHT_JOINT1,
        upper,
        0.0,
    )
    plan = compile_reference_plan(
        initial,
        targets,
        profile.joints,
        config.max_accelerations,
        initial_velocities=velocities,
        final_velocities=velocities,
    )
    window, anchor = window_and_anchor(plan, targets, profile)
    prepared = prepare_chunk(anchor, window, profile, config, settle_at_end=True)
    assert prepared.settle_at_end is True
    assert prepared.sample_at(prepared.end_sim_time_ns).positions[RIGHT_JOINT1] == pytest.approx(
        upper
    )
