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

from robot_backend_sim.reference_compiler import compile_reference_plan  # noqa: E402
from robot_contracts.models import ContractError, JointSpec, Target  # noqa: E402
from robot_contracts.profiles import load_profile  # noqa: E402
from robot_contracts.reference import verify_reference_plan  # noqa: E402
from scipy.interpolate import CubicSpline  # noqa: E402


def joint(name: str, minimum: float = -10.0, maximum: float = 10.0) -> JointSpec:
    return JointSpec(name, "rad", minimum, maximum, 10.0, 0.0)


def scipy_spline(times_ns, initial, positions):
    return CubicSpline(
        np.array([0.0, *(time_ns / 1_000_000_000 for time_ns in times_ns)]),
        np.array([initial, *positions], dtype=float),
        axis=0,
        bc_type="clamped",
        extrapolate=False,
    )


def roots(polynomial) -> list[float]:
    values = np.asarray(
        polynomial.roots(discontinuity=False, extrapolate=False),
        dtype=float,
    ).reshape(-1)
    return [float(value) for value in values if np.isfinite(value)]


def test_compiler_matches_scipy_coefficients_and_samples_in_joint_order():
    times = (400_000_000, 900_000_000, 1_700_000_000)
    initial = (0.1, -0.2, 0.02)
    positions = (
        (0.2, -0.1, 0.03),
        (0.05, -0.3, 0.01),
        (-0.1, 0.0, 0.04),
    )
    joints = (joint("left_j1", -2.0, 2.0), joint("right_j1", -2.0, 2.0), joint("grip", 0.0, 0.08))
    plan = compile_reference_plan(
        initial,
        tuple(Target(time, values) for time, values in zip(times, positions, strict=True)),
        joints,
        (100.0, 100.0, 100.0),
    )
    spline = scipy_spline(times, initial, positions)

    assert plan.joint_names == tuple(item.name for item in joints)
    assert tuple(segment.duration_ns for segment in plan.segments) == (
        400_000_000,
        500_000_000,
        800_000_000,
    )
    for index, segment in enumerate(plan.segments):
        np.testing.assert_allclose(segment.c3, spline.c[0, index, :], atol=1e-14, rtol=0)
        np.testing.assert_allclose(segment.c2, spline.c[1, index, :], atol=1e-14, rtol=0)
        np.testing.assert_allclose(segment.c1, spline.c[2, index, :], atol=1e-14, rtol=0)
        np.testing.assert_allclose(segment.c0, spline.c[3, index, :], atol=1e-14, rtol=0)

    horizon_ns = times[-1]
    for elapsed_ns in (
        0,
        1,
        123_456_789,
        400_000_000,
        500_000_000,
        900_000_000,
        1_234_567_890,
        horizon_ns,
    ):
        sample = plan.sample(elapsed_ns)
        seconds = elapsed_ns / 1_000_000_000
        np.testing.assert_allclose(sample.positions, spline(seconds), atol=1e-12, rtol=0)
        np.testing.assert_allclose(sample.velocities, spline(seconds, 1), atol=1e-12, rtol=0)
        np.testing.assert_allclose(sample.accelerations, spline(seconds, 2), atol=1e-12, rtol=0)

    held = plan.sample(horizon_ns + 1)
    np.testing.assert_allclose(held.positions, spline(horizon_ns / 1e9), atol=1e-12, rtol=0)
    np.testing.assert_array_equal(held.velocities, np.zeros(3))
    np.testing.assert_array_equal(held.accelerations, np.zeros(3))


def test_compiled_extrema_match_scipy_interpolation_derivative_and_roots():
    times = (300_000_000, 800_000_000, 1_500_000_000, 2_400_000_000)
    initial = (0.0, 0.1)
    positions = ((0.2, 0.0), (0.1, 0.25), (0.3, 0.05), (0.15, 0.2))
    joints = (joint("a"), joint("b"))
    targets = tuple(Target(time, values) for time, values in zip(times, positions, strict=True))
    plan = compile_reference_plan(initial, targets, joints, (100.0, 100.0))
    extremes = verify_reference_plan(plan, initial, targets, joints, (100.0, 100.0))

    horizon_s = times[-1] / 1_000_000_000
    breakpoints = [0.0, *(time / 1_000_000_000 for time in times)]
    for index in range(2):
        one_dimensional = CubicSpline(
            [0.0, *(time / 1_000_000_000 for time in times)],
            [initial[index], *(values[index] for values in positions)],
            bc_type="clamped",
            extrapolate=False,
        )
        position_times = breakpoints + [
            value
            for value in roots(one_dimensional.derivative())
            if -1e-12 <= value <= horizon_s + 1e-12
        ]
        scipy_minimum = min(float(one_dimensional(value)) for value in position_times)
        scipy_maximum = max(float(one_dimensional(value)) for value in position_times)

        velocity_times = breakpoints + [
            value
            for value in roots(one_dimensional.derivative(2))
            if -1e-12 <= value <= horizon_s + 1e-12
        ]
        scipy_maximum_velocity = max(
            abs(float(one_dimensional(value, 1))) for value in velocity_times
        )
        scipy_maximum_acceleration = max(
            abs(float(one_dimensional(value, 2))) for value in breakpoints
        )

        actual = extremes.joints[index]
        assert actual.minimum_position == pytest.approx(scipy_minimum, abs=1e-9)
        assert actual.maximum_position == pytest.approx(scipy_maximum, abs=1e-9)
        assert actual.maximum_velocity == pytest.approx(scipy_maximum_velocity, abs=1e-9)
        assert actual.maximum_acceleration == pytest.approx(scipy_maximum_acceleration, abs=1e-9)


def test_panda_sixteen_dimensions_mixed_units_and_constant_standby_channels():
    profile = load_profile(
        ROOT / "profiles/robots/panda_stack_v1/profile.json",
        ROOT / "contracts/schemas",
    )
    initial = profile.initial_positions
    arm_indices = {index for index in range(16) if index not in (7, 15)}
    targets = tuple(
        Target(
            time_ns,
            tuple(
                value + (0.03 * step if index in arm_indices else 0.0)
                for index, value in enumerate(initial)
            ),
        )
        for step, time_ns in ((1, 600_000_000), (2, 1_500_000_000))
    )
    max_accelerations = tuple(20.0 for _ in range(16))
    plan = compile_reference_plan(initial, targets, profile.joints, max_accelerations)
    extremes = verify_reference_plan(
        plan,
        initial,
        targets,
        profile.joints,
        max_accelerations,
    )

    assert plan.joint_count == 16
    assert plan.duration_ns == 1_500_000_000
    for index in (7, 15):
        standby = extremes.joints[index]
        assert standby.minimum_position == pytest.approx(initial[index], abs=1e-12)
        assert standby.maximum_position == pytest.approx(initial[index], abs=1e-12)
        assert standby.maximum_velocity == pytest.approx(0.0, abs=1e-12)
        assert standby.maximum_acceleration == pytest.approx(0.0, abs=1e-12)

    spline = scipy_spline(
        tuple(target.time_ns for target in targets),
        initial,
        tuple(target.positions for target in targets),
    )
    sample = plan.sample(1_000_000_000)
    np.testing.assert_allclose(sample.positions, spline(1.0), atol=1e-12, rtol=0)
    np.testing.assert_allclose(sample.velocities, spline(1.0, 1), atol=1e-12, rtol=0)
    np.testing.assert_allclose(sample.accelerations, spline(1.0, 2), atol=1e-12, rtol=0)


def test_compiler_rejects_waypoints_that_only_look_valid_at_the_ends():
    joints = (JointSpec("joint1", "rad", -60.0, 0.2, 100.0, 0.0),)
    targets = (
        Target(1_000_000_000, (-10.0,)),
        Target(2_000_000_000, (-160.0 / 3.0,)),
    )
    with pytest.raises(ContractError) as caught:
        compile_reference_plan((0.0,), targets, joints, (250.0,))
    assert caught.value.reason == "JOINT_LIMIT"


def scipy_spline_with_boundaries(
    times_ns, initial, positions, initial_velocities, final_velocities
):
    return CubicSpline(
        np.array([0.0, *(time_ns / 1_000_000_000 for time_ns in times_ns)]),
        np.array([initial, *positions], dtype=float),
        axis=0,
        bc_type=(
            (1, np.asarray(initial_velocities, dtype=float)),
            (1, np.asarray(final_velocities, dtype=float)),
        ),
        extrapolate=False,
    )


def test_explicit_boundary_velocities_match_scipy_and_declared_derivatives():
    times = (400_000_000, 900_000_000, 1_700_000_000)
    initial = (0.1, -0.2, 0.02)
    positions = (
        (0.2, -0.1, 0.03),
        (0.05, -0.3, 0.01),
        (-0.1, 0.0, 0.04),
    )
    initial_velocities = (0.3, -0.4, 0.05)
    final_velocities = (-0.2, 0.1, -0.03)
    joints = (joint("left_j1", -2.0, 2.0), joint("right_j1", -2.0, 2.0), joint("grip", 0.0, 0.08))
    targets = tuple(Target(time, values) for time, values in zip(times, positions, strict=True))
    plan = compile_reference_plan(
        initial,
        targets,
        joints,
        (100.0, 100.0, 100.0),
        initial_velocities=initial_velocities,
        final_velocities=final_velocities,
    )
    spline = scipy_spline_with_boundaries(
        times, initial, positions, initial_velocities, final_velocities
    )

    for index, segment in enumerate(plan.segments):
        np.testing.assert_allclose(segment.c3, spline.c[0, index, :], atol=1e-14, rtol=0)
        np.testing.assert_allclose(segment.c2, spline.c[1, index, :], atol=1e-14, rtol=0)
        np.testing.assert_allclose(segment.c1, spline.c[2, index, :], atol=1e-14, rtol=0)
        np.testing.assert_allclose(segment.c0, spline.c[3, index, :], atol=1e-14, rtol=0)

    start = plan.sample_interval(0)
    np.testing.assert_allclose(start.velocities, initial_velocities, atol=1e-12, rtol=0)
    end = plan.sample_interval(times[-1])
    np.testing.assert_allclose(end.velocities, final_velocities, atol=1e-12, rtol=0)

    for elapsed_ns in (0, 200_000_000, 400_000_000, 900_000_000, 1_234_567_890, times[-1]):
        sample = plan.sample_interval(elapsed_ns)
        seconds = elapsed_ns / 1_000_000_000
        np.testing.assert_allclose(sample.positions, spline(seconds), atol=1e-12, rtol=0)
        np.testing.assert_allclose(sample.velocities, spline(seconds, 1), atol=1e-12, rtol=0)
        np.testing.assert_allclose(sample.accelerations, spline(seconds, 2), atol=1e-12, rtol=0)

    verify_reference_plan(
        plan,
        initial,
        targets,
        joints,
        (100.0, 100.0, 100.0),
        initial_velocities=initial_velocities,
        final_velocities=final_velocities,
    )
    with pytest.raises(ContractError, match="start_velocities"):
        verify_reference_plan(
            plan,
            initial,
            targets,
            joints,
            (100.0, 100.0, 100.0),
            initial_velocities=(0.0, -0.4, 0.05),
            final_velocities=final_velocities,
        )


def test_explicit_velocity_boundaries_are_limited_before_scipy_and_never_clipped():
    joints = (joint("joint1", -10.0, 10.0),)
    targets = (Target(1_000_000_000, (0.0,)),)
    with pytest.raises(ContractError) as caught:
        compile_reference_plan(
            (0.0,),
            targets,
            joints,
            (100.0,),
            initial_velocities=(10.000001,),
        )
    assert caught.value.reason == "VELOCITY_LIMIT"

    accepted = compile_reference_plan(
        (0.0,),
        targets,
        joints,
        (100.0,),
        initial_velocities=(10.0,),
        final_velocities=(-10.0,),
    )
    assert accepted.sample_interval(0).velocities == pytest.approx((10.0,))
    assert accepted.sample_interval(1_000_000_000).velocities == pytest.approx((-10.0,))
