from __future__ import annotations

import json
import math
import random
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
from robot_contracts.action_chunks import ActionWindow, ChunkAnchor
from robot_contracts.follower import PreparedFollowerPlan
from robot_contracts.models import ContractError, Target
from robot_contracts.profiles import load_profile
from robot_contracts.reference import (
    POSITION_ATOL,
    RELATIVE_TOLERANCE,
    VELOCITY_ATOL,
    ReferenceSample,
    check_continuous_limits,
    compute_reference_extremes,
)

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.chunk_plan import prepare_chunk  # noqa: E402
from robot_backend_sim.follower_config import load_follower_config  # noqa: E402
from robot_backend_sim.follower_plan import prepare_follower_plan  # noqa: E402
from robot_backend_sim.motion_config import load_control_config  # noqa: E402
from robot_backend_sim.ruckig_export import export_ruckig_reference  # noqa: E402

ruckig = pytest.importorskip("ruckig")

PROFILE = load_profile(
    ROOT / "profiles/robots/panda_stack_v1/profile.json",
    ROOT / "contracts/schemas/v1",
)
CONTROL = load_control_config(ROOT / "simulation/mujoco/configs/control/panda_control.yaml")
FOLLOWER = load_follower_config(ROOT / "simulation/mujoco/configs/control/panda_follower.yaml")
BASE = PROFILE.initial_positions
JOINT_COUNT = len(BASE)

_TINY_BRAKE_START_P = 0.010250677416454232
_TINY_BRAKE_START_V = 0.04000000000000001
_TINY_BRAKE_DURATION = 2.2013877787807815e-14
_TINY_BRAKE_ACCEL = -0.5
_TINY_BRAKE_TARGET = 0.01665229386424162
_TINY_BRAKE_MAIN_P = 0.010250677416455113
_TINY_BRAKE_MAIN_V = 0.039999999999989


def smoothstep(fraction: float) -> float:
    value = min(max(fraction, 0.0), 1.0)
    return 3.0 * value * value - 2.0 * value * value * value


def smoothstep_derivative(fraction: float, duration_s: float, total: float) -> float:
    value = min(max(fraction, 0.0), 1.0)
    return (6.0 * value - 6.0 * value * value) * total / duration_s


def calculate_trajectory(
    current_positions,
    current_velocities,
    target_positions,
    target_velocities,
    *,
    max_velocities=None,
    max_accelerations=None,
    minimum_duration_s=0.25,
):
    count = len(current_positions)
    max_velocities = max_velocities or [joint.max_velocity for joint in PROFILE.joints]
    max_accelerations = max_accelerations or list(CONTROL.max_accelerations)
    inp = ruckig.InputParameter(count)
    inp.current_position = list(current_positions)
    inp.current_velocity = list(current_velocities)
    inp.current_acceleration = [0.0] * count
    inp.target_position = list(target_positions)
    inp.target_velocity = list(target_velocities)
    inp.target_acceleration = [0.0] * count
    inp.max_velocity = list(max_velocities)
    inp.max_acceleration = list(max_accelerations)
    inp.max_jerk = [math.inf] * count
    inp.minimum_duration = minimum_duration_s
    trajectory = ruckig.Trajectory(count)
    result = ruckig.Ruckig(count).calculate(inp, trajectory)
    assert int(result) >= 0
    return trajectory


def calculate_tiny_brake_trajectory(target_position):
    inp = ruckig.InputParameter(1)
    inp.current_position = [_TINY_BRAKE_START_P]
    inp.current_velocity = [_TINY_BRAKE_START_V]
    inp.current_acceleration = [0.0]
    inp.target_position = [target_position]
    inp.target_velocity = [0.0]
    inp.target_acceleration = [0.0]
    inp.max_velocity = [0.04]
    inp.max_acceleration = [0.5]
    inp.max_jerk = [math.inf]
    inp.minimum_duration = 0.5
    trajectory = ruckig.Trajectory(1)
    result = ruckig.Ruckig(1).calculate(inp, trajectory)
    assert int(result) >= 0
    return trajectory


def make_tiny_brake_profile(
    *,
    duration=_TINY_BRAKE_DURATION,
    times=(_TINY_BRAKE_DURATION, 0.0),
    brake_p=(_TINY_BRAKE_START_P, 0.0),
    brake_v=(_TINY_BRAKE_START_V, 0.0),
    brake_a=(_TINY_BRAKE_ACCEL, 0.0),
    brake_j=(0.0, 0.0),
    main_p=_TINY_BRAKE_MAIN_P,
    main_v=_TINY_BRAKE_MAIN_V,
):
    "A typed public-profile fixture for invalid brake-prefix cases."
    brake = SimpleNamespace(
        duration=duration,
        t=list(times),
        p=list(brake_p),
        v=list(brake_v),
        a=list(brake_a),
        j=list(brake_j),
    )
    main = SimpleNamespace(
        t=[0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        p=[main_p] * 8,
        v=[main_v] * 8,
        a=[0.0] * 8,
        j=[0.0] * 7,
        brake=brake,
    )
    return SimpleNamespace(duration=0.5, profiles=((main,),))


def make_source_targets(
    anchor_positions,
    target_positions,
    duration_s,
    count=60,
    step_s=0.05,
):
    targets = []
    for index in range(1, count + 1):
        time_s = index * step_s
        fraction = time_s / duration_s
        positions = tuple(
            anchor + (target - anchor) * smoothstep(fraction)
            for anchor, target in zip(anchor_positions, target_positions, strict=True)
        )
        targets.append(Target(round(time_s * 1_000_000_000), positions))
    return tuple(targets)


def make_window(
    source_targets,
    *,
    observation_id=1,
    handoff_ns=250_000_000,
    target_count=5,
):
    return ActionWindow(
        observation_id=observation_id,
        observation_time_ns=0,
        handoff_time_ns=handoff_ns,
        source_targets=source_targets,
        target_count=target_count,
    )


def make_anchor(window, positions, velocities=None, *, stream_id="follower-test"):
    velocities = velocities or (0.0,) * JOINT_COUNT
    return ChunkAnchor(
        stream_id,
        0,
        window.handoff_time_ns,
        ReferenceSample(
            tuple(positions),
            tuple(velocities),
            (0.0,) * JOINT_COUNT,
        ),
        PROFILE.digest,
    )


def segment_state(segment, local_s):
    positions = []
    velocities = []
    accelerations = []
    for c0, c1, c2, c3 in zip(
        segment.c0, segment.c1, segment.c2, segment.c3, strict=True
    ):
        c3_s = c3 * local_s
        positions.append(((c3_s + c2) * local_s + c1) * local_s + c0)
        velocities.append((3.0 * c3_s + 2.0 * c2) * local_s + c1)
        accelerations.append(6.0 * c3_s + 2.0 * c2)
    return tuple(positions), tuple(velocities), tuple(accelerations)


def assert_plan_c1_and_limits(prepared: PreparedFollowerPlan) -> None:
    for index in range(1, len(prepared.plan.segments)):
        previous = prepared.plan.segments[index - 1]
        current = prepared.plan.segments[index]
        previous_positions, previous_velocities, _ = segment_state(
            previous, previous.duration_s
        )
        current_positions, current_velocities, _ = segment_state(current, 0.0)
        for axis in range(prepared.joint_count):
            assert previous_positions[axis] == pytest.approx(
                current_positions[axis], rel=RELATIVE_TOLERANCE, abs=POSITION_ATOL
            )
            assert previous_velocities[axis] == pytest.approx(
                current_velocities[axis], rel=RELATIVE_TOLERANCE, abs=VELOCITY_ATOL
            )
    extremes = compute_reference_extremes(prepared.plan)
    check_continuous_limits(extremes, prepared.joints, prepared.max_accelerations)
    assert len(extremes.joints) == prepared.joint_count


def assert_export_matches_ruckig(trajectory, exported, duration_ns):
    boundaries = set()
    elapsed = 0
    for segment in exported.plan.segments[:-1]:
        elapsed += segment.duration_ns
        boundaries.add(elapsed)
    max_position_error = 0.0
    max_velocity_error = 0.0
    max_acceleration_error = 0.0
    step = max(1, duration_ns // 500)
    for elapsed_ns in range(0, duration_ns + 1, step):
        if elapsed_ns in boundaries or elapsed_ns == duration_ns:
            continue
        sample = exported.plan.sample_interval(elapsed_ns)
        position, velocity, acceleration = trajectory.at_time(elapsed_ns / 1_000_000_000)
        for axis in range(exported.plan.joint_count):
            max_position_error = max(
                max_position_error, abs(sample.positions[axis] - position[axis])
            )
            max_velocity_error = max(
                max_velocity_error, abs(sample.velocities[axis] - velocity[axis])
            )
            max_acceleration_error = max(
                max_acceleration_error, abs(sample.accelerations[axis] - acceleration[axis])
            )
    for boundary in sorted(boundaries):
        for offset in (-1, 1):
            elapsed_ns = boundary + offset
            if elapsed_ns < 0 or elapsed_ns > duration_ns:
                continue
            sample = exported.plan.sample_interval(elapsed_ns)
            position, velocity, acceleration = trajectory.at_time(
                elapsed_ns / 1_000_000_000
            )
            for axis in range(exported.plan.joint_count):
                max_position_error = max(
                    max_position_error, abs(sample.positions[axis] - position[axis])
                )
                max_velocity_error = max(
                    max_velocity_error, abs(sample.velocities[axis] - velocity[axis])
                )
                max_acceleration_error = max(
                    max_acceleration_error,
                    abs(sample.accelerations[axis] - acceleration[axis]),
                )
    # Nanosecond boundary rounding and the exact C1 seam are the only sources
    # of error; acceleration may step at a seam and is checked one-sided above.
    assert max_position_error < 5e-9
    assert max_velocity_error < 5e-9
    assert max_acceleration_error < 5e-9
    return max_position_error, max_velocity_error, max_acceleration_error


def assert_export_pv_matches_ruckig(trajectory, exported, duration_ns):
    boundaries = set()
    elapsed = 0
    for segment in exported.plan.segments:
        elapsed += segment.duration_ns
        boundaries.add(elapsed)
    sample_times = {0, duration_ns}
    sample_times.update(boundaries)
    sample_times.update(range(0, duration_ns + 1, 2_000_000))
    max_position_error = 0.0
    max_velocity_error = 0.0
    for elapsed_ns in sorted(sample_times):
        sample = exported.plan.sample_interval(elapsed_ns)
        position, velocity, _ = trajectory.at_time(elapsed_ns / 1_000_000_000)
        max_position_error = max(
            max_position_error, abs(sample.positions[0] - position[0])
        )
        max_velocity_error = max(
            max_velocity_error, abs(sample.velocities[0] - velocity[0])
        )
    assert max_position_error < 5e-9
    assert max_velocity_error < 5e-9
    return max_position_error, max_velocity_error


@pytest.mark.parametrize(
    (
        "name",
        "current_positions",
        "current_velocities",
        "target_positions",
        "target_velocities",
        "minimum_s",
    ),
    (
        (
            "static",
            BASE,
            (0.0,) * JOINT_COUNT,
            BASE,
            (0.0,) * JOINT_COUNT,
            0.25,
        ),
        (
            "single-axis",
            BASE,
            (0.0,) * JOINT_COUNT,
            (BASE[0] + 0.1,) + BASE[1:],
            (0.0,) * JOINT_COUNT,
            0.25,
        ),
        (
            "sixteen-axis",
            BASE,
            (0.0,) * JOINT_COUNT,
            tuple(
                min(max(value + (0.01 if index % 7 else -0.008), joint.minimum + 1e-3),
                    joint.maximum - 1e-3)
                for index, (value, joint) in enumerate(zip(BASE, PROFILE.joints, strict=True))
            ),
            (0.0,) * JOINT_COUNT,
            0.5,
        ),
        (
            "reverse",
            (BASE[0],) + BASE[1:],
            (0.2,) + (0.0,) * (JOINT_COUNT - 1),
            (BASE[0] - 0.05,) + BASE[1:],
            (0.0,) * JOINT_COUNT,
            0.25,
        ),
        (
            "tiny",
            BASE,
            (0.0,) * JOINT_COUNT,
            (BASE[0] + 1e-6,) + BASE[1:],
            (0.0,) * JOINT_COUNT,
            0.0001,
        ),
        (
            "gripper-boundary",
            BASE,
            (0.0,) * JOINT_COUNT,
            BASE[:7] + (BASE[7] - 0.02,) + BASE[8:],
            (0.0,) * JOINT_COUNT,
            0.25,
        ),
        (
            "zero-duration-middle-phase",
            BASE,
            (0.0,) * JOINT_COUNT,
            (BASE[0] + 0.1,) + BASE[1:],
            (0.2,) + (0.0,) * (JOINT_COUNT - 1),
            1.0,
        ),
    ),
)
def test_export_matches_public_ruckig_curve_and_boundaries(
    name,
    current_positions,
    current_velocities,
    target_positions,
    target_velocities,
    minimum_s,
):
    trajectory = calculate_trajectory(
        current_positions,
        current_velocities,
        target_positions,
        target_velocities,
        minimum_duration_s=minimum_s,
    )
    duration_ns = round(trajectory.duration * 1_000_000_000)
    exported = export_ruckig_reference(
        trajectory,
        PROFILE.joint_names,
        truncate_ns=duration_ns,
    )
    assert exported.plan.duration_ns == duration_ns
    assert exported.profile_duration_ns == duration_ns
    errors = assert_export_matches_ruckig(trajectory, exported, duration_ns)
    assert errors[0] < 5e-9
    if name == "static":
        assert exported.plan.duration_ns == duration_ns


def test_export_rejects_unsupported_public_structures():
    phase = SimpleNamespace(
        t=[0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        p=[0.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1],
        v=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        a=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        j=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    )
    one_section = SimpleNamespace(duration=0.1, profiles=((phase,),))
    exported = export_ruckig_reference(one_section, ("j0",), truncate_ns=100_000_000)
    assert exported.plan.duration_ns == 100_000_000

    two_sections = SimpleNamespace(duration=0.1, profiles=((phase,), (phase,)))
    with pytest.raises(ContractError) as caught:
        export_ruckig_reference(two_sections, ("j0",))
    assert caught.value.reason == "INVALID_RUCKIG_EXPORT"

    bad_lengths = SimpleNamespace(
        duration=0.1,
        profiles=(
            (
                SimpleNamespace(
                    t=[0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    p=[0.0],
                    v=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    a=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    j=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                ),
            ),
        ),
    )
    with pytest.raises(ContractError):
        export_ruckig_reference(bad_lengths, ("j0",))

    with pytest.raises(ContractError):
        export_ruckig_reference(one_section, ("j0",), truncate_ns=100_000_001)

    braking_phase = SimpleNamespace(
        t=phase.t,
        p=phase.p,
        v=phase.v,
        a=phase.a,
        j=phase.j,
        brake=SimpleNamespace(
            duration=0.01,
            t=[0.01, 0.0],
            p=[0.0, 0.0],
            v=[0.0, 0.0],
            a=[0.0, 0.0],
            j=[0.0, 0.0],
        ),
    )
    braking_section = SimpleNamespace(duration=0.1, profiles=((braking_phase,),))
    with pytest.raises(ContractError):
        export_ruckig_reference(braking_section, ("j0",))


@pytest.mark.parametrize("target_position", (_TINY_BRAKE_TARGET, 0.03))
def test_export_folds_real_subnanosecond_brake_prefix(target_position):
    trajectory = calculate_tiny_brake_trajectory(target_position)
    brake = trajectory.profiles[0][0].brake
    assert 0.0 < brake.duration < 0.5e-9
    assert list(brake.t)[1] == 0.0
    assert list(brake.j)[0] == 0.0
    full_ns = round(trajectory.duration * 1_000_000_000)

    for truncate_ns in (full_ns, 250_000_000):
        exported = export_ruckig_reference(
            trajectory,
            ("j0",),
            truncate_ns=truncate_ns,
        )
        assert exported.profile_duration_ns == full_ns
        assert exported.plan.duration_ns == truncate_ns
        sample = exported.plan.sample_interval(0)
        assert sample.positions[0] == brake.p[0]
        assert sample.velocities[0] == brake.v[0]
        max_position_error, max_velocity_error = assert_export_pv_matches_ruckig(
            trajectory,
            exported,
            truncate_ns,
        )
        assert max_position_error < 5e-9
        assert max_velocity_error < 5e-9


def test_export_tiny_brake_fixture_ignores_uninitialized_padding():
    profile = make_tiny_brake_profile(
        brake_p=(_TINY_BRAKE_START_P, math.nan),
        brake_v=(_TINY_BRAKE_START_V, math.nan),
        brake_a=(_TINY_BRAKE_ACCEL, math.nan),
        brake_j=(0.0, math.nan),
    )
    exported = export_ruckig_reference(profile, ("j0",), truncate_ns=500_000_000)
    sample = exported.plan.sample_interval(0)
    assert sample.positions[0] == _TINY_BRAKE_START_P
    assert sample.velocities[0] == _TINY_BRAKE_START_V


def test_export_zero_brake_prefix_does_not_read_padding():
    profile = make_tiny_brake_profile(
        duration=0.0,
        times=(0.0, 0.0),
        brake_p=(math.nan, math.nan),
        brake_v=(math.nan, math.nan),
        brake_a=(math.nan, math.nan),
        brake_j=(math.nan, math.nan),
    )
    exported = export_ruckig_reference(profile, ("j0",), truncate_ns=500_000_000)
    sample = exported.plan.sample_interval(0)
    assert sample.positions[0] == _TINY_BRAKE_MAIN_P
    assert sample.velocities[0] == _TINY_BRAKE_MAIN_V


@pytest.mark.parametrize(
    "overrides",
    (
        {"duration": 1e-9, "times": (1e-9, 0.0)},
        {
            "duration": -_TINY_BRAKE_DURATION,
            "times": (-_TINY_BRAKE_DURATION, 0.0),
        },
        {"duration": math.nan},
        {"times": (_TINY_BRAKE_DURATION, math.nan)},
        {"brake_j": (1.0, 0.0)},
        {"brake_p": (_TINY_BRAKE_START_P,)},
        {"times": (_TINY_BRAKE_DURATION,)},
        {"brake_v": (1e6, 0.0)},
        {"brake_p": (1.0, 0.0)},
        {
            "duration": _TINY_BRAKE_DURATION + 1e-15,
            "times": (_TINY_BRAKE_DURATION, 1e-15),
        },
        {"times": (_TINY_BRAKE_DURATION + 1e-15, 0.0)},
    ),
)
def test_export_rejects_invalid_brake_prefixes(overrides):
    with pytest.raises(ContractError) as caught:
        export_ruckig_reference(
            make_tiny_brake_profile(**overrides),
            ("j0",),
            truncate_ns=500_000_000,
        )
    assert caught.value.reason == "INVALID_RUCKIG_EXPORT"


def counterexample_window(lead_ms: int = 250) -> ActionWindow:
    targets = make_source_targets(BASE, (BASE[0] + 0.04,) + BASE[1:], 3.0)
    return make_window(targets, handoff_ns=lead_ms * 1_000_000)


def test_strict_first_window_counterexample_and_bounded_follower():
    window = counterexample_window(250)
    anchor = make_anchor(window, BASE, (0.0,) * JOINT_COUNT)
    with pytest.raises(ContractError) as caught:
        prepare_chunk(anchor, window, PROFILE, CONTROL)
    assert caught.value.reason == "ACCELERATION_LIMIT"
    assert "1.828475" in str(caught.value)

    prepared = prepare_follower_plan(
        anchor,
        window,
        PROFILE,
        CONTROL,
        FOLLOWER,
        is_final=False,
    )
    assert prepared.duration_ns == window.end_time_ns
    assert prepared.plan.duration_ns == prepared.requested_window_ns
    assert prepared.window is window
    assert prepared.window.source_targets == window.source_targets
    assert prepared.window.targets == window.targets
    assert max(prepared.max_target_deviations) <= max(
        prepared.max_target_deviation_limits
    )
    assert_plan_c1_and_limits(prepared)
    assert prepared.flags.acceleration_limited[0] is True
    assert prepared.raw_terminal_velocities[0] != pytest.approx(
        prepared.restricted_terminal_velocities[0]
    ) or prepared.flags.velocity_limited[0] is False

    document = prepared.to_document()
    payload = json.dumps(document, allow_nan=False, indent=2)
    assert "bounded_target_follower" in payload
    assert "max_target_deviations" in payload
    assert "actual_terminal_velocities" in payload


@pytest.mark.parametrize("lead_ms", (50, 100, 250))
def test_two_radian_seven_second_fragment_with_three_leads(lead_ms):
    targets = make_source_targets(BASE, (BASE[0] + 2.0,) + BASE[1:], 7.0)
    window = make_window(targets, handoff_ns=lead_ms * 1_000_000)
    anchor = make_anchor(window, BASE, (0.0,) * JOINT_COUNT)

    strict_reason = None
    try:
        prepare_chunk(anchor, window, PROFILE, CONTROL)
    except ContractError as error:
        strict_reason = error.reason
    assert strict_reason == "ACCELERATION_LIMIT"

    for is_final in (False, True):
        prepared = prepare_follower_plan(
            anchor,
            window,
            PROFILE,
            CONTROL,
            FOLLOWER,
            is_final=is_final,
        )
        assert prepared.duration_ns <= 3_000_000_000
        if is_final:
            assert prepared.plan.duration_ns >= window.end_time_ns
            assert prepared.actual_terminal_velocities == pytest.approx(
                (0.0,) * JOINT_COUNT, abs=1e-9
            )
            if lead_ms == 250:
                assert prepared.extension_ns > 0
        else:
            assert prepared.plan.duration_ns == window.end_time_ns
        assert max(prepared.max_target_deviations) <= max(
            prepared.max_target_deviation_limits
        )
        assert_plan_c1_and_limits(prepared)
        assert len(prepared.raw_terminal_velocities) == JOINT_COUNT
        assert len(prepared.restricted_terminal_velocities) == JOINT_COUNT
        assert len(prepared.max_target_deviations) == JOINT_COUNT
        assert len(prepared.flags.velocity_limited) == JOINT_COUNT
        assert len(prepared.flags.acceleration_limited) == JOINT_COUNT
        assert len(prepared.flags.boundary_preserved) == JOINT_COUNT


def clip_to_joint(joint, value: float) -> float:
    lower = joint.minimum + 1e-3
    upper = joint.maximum - 1e-3
    return min(max(value, lower), upper)


def test_deterministic_random_100_sixteen_axis_cases():
    rng = random.Random(2029)
    successes = 0
    for case in range(100):
        anchor_positions = []
        target_positions = []
        anchor_velocities = []
        for joint in PROFILE.joints:
            span = joint.maximum - joint.minimum
            anchor_offset = rng.uniform(-min(0.02, span / 20), min(0.02, span / 20))
            anchor_positions.append(clip_to_joint(joint, joint.initial + anchor_offset))
            target_offset = rng.uniform(-min(0.05, span / 8), min(0.05, span / 8))
            target_positions.append(
                clip_to_joint(joint, anchor_positions[-1] + target_offset)
            )
            velocity_limit = min(0.05, joint.max_velocity * 0.5)
            anchor_velocities.append(rng.uniform(-velocity_limit, velocity_limit))
        source_targets = make_source_targets(
            anchor_positions,
            target_positions,
            3.0,
        )
        window = make_window(source_targets, observation_id=case + 1)
        anchor = make_anchor(
            window,
            anchor_positions,
            anchor_velocities,
            stream_id=f"random-{case}",
        )
        prepared = prepare_follower_plan(
            anchor,
            window,
            PROFILE,
            CONTROL,
            FOLLOWER,
            is_final=(case % 2 == 0),
        )
        assert prepared.window.source_targets == window.source_targets
        assert prepared.plan.joint_names == PROFILE.joint_names
        assert prepared.plan.duration_ns <= 3_000_000_000
        assert len(prepared.max_target_deviations) == JOINT_COUNT
        for deviation, limit in zip(
            prepared.max_target_deviations,
            prepared.max_target_deviation_limits,
            strict=True,
        ):
            assert 0.0 <= deviation <= limit
        assert_plan_c1_and_limits(prepared)
        if prepared.is_final:
            assert prepared.plan.duration_ns >= window.end_time_ns
            assert prepared.actual_terminal_velocities == pytest.approx(
                (0.0,) * JOINT_COUNT, abs=1e-9
            )
        else:
            assert prepared.plan.duration_ns == window.end_time_ns
        successes += 1
    assert successes == 100


def test_nonfinal_actual_endpoint_is_inherited_by_the_successor():
    source_targets = make_source_targets(BASE, (BASE[0] + 1.0,) + BASE[1:], 3.0)
    first_window = make_window(source_targets, handoff_ns=250_000_000)
    first_anchor = make_anchor(first_window, BASE, (0.0,) * JOINT_COUNT)
    first = prepare_follower_plan(
        first_anchor,
        first_window,
        PROFILE,
        CONTROL,
        FOLLOWER,
        is_final=False,
    )
    assert first.flags.boundary_preserved[0] is False
    assert first.actual_terminal_positions[0] < first_window.targets[-1].positions[0]

    successor_handoff_ns = first.end_sim_time_ns
    successor_window = make_window(source_targets, handoff_ns=successor_handoff_ns)
    successor_anchor = ChunkAnchor(
        "follower-test",
        first.seq,
        successor_handoff_ns,
        ReferenceSample(
            first.actual_terminal_positions,
            first.actual_terminal_velocities,
            first.actual_terminal_accelerations,
        ),
        PROFILE.digest,
    )
    second = prepare_follower_plan(
        successor_anchor,
        successor_window,
        PROFILE,
        CONTROL,
        FOLLOWER,
        is_final=False,
    )
    start_positions, start_velocities, _ = segment_state(second.plan.segments[0], 0.0)
    for axis in range(JOINT_COUNT):
        assert start_positions[axis] == pytest.approx(
            first.actual_terminal_positions[axis],
            rel=RELATIVE_TOLERANCE,
            abs=POSITION_ATOL,
        )
        assert start_velocities[axis] == pytest.approx(
            first.actual_terminal_velocities[axis],
            rel=RELATIVE_TOLERANCE,
            abs=VELOCITY_ATOL,
        )
    assert_plan_c1_and_limits(second)


def test_terminal_speed_is_capped_while_raw_targets_are_preserved():
    positions = (0.0, 0.01, 0.02, 0.03, 0.09)
    source_targets = tuple(
        Target(300_000_000 + index * 50_000_000, (position,) + BASE[1:])
        for index, position in enumerate(positions)
    )
    window = make_window(source_targets)
    anchor = make_anchor(window, BASE, (0.0,) * JOINT_COUNT)
    prepared = prepare_follower_plan(
        anchor,
        window,
        PROFILE,
        CONTROL,
        FOLLOWER,
        is_final=False,
    )
    raw = (positions[-1] - positions[-2]) / 0.05
    braking_cap = math.sqrt(2.0 * CONTROL.max_accelerations[0] * positions[-1])
    assert prepared.raw_terminal_velocities[0] == pytest.approx(raw)
    assert prepared.restricted_terminal_velocities[0] == pytest.approx(braking_cap)
    assert prepared.flags.velocity_limited[0] is True
    assert prepared.window.targets[-1].positions[0] == positions[-1]
    assert prepared.window.source_targets == window.source_targets
    assert_plan_c1_and_limits(prepared)


def gripper_boundary_source(first_positions, remaining_position):
    targets = []
    for index in range(60):
        position = (
            first_positions[index]
            if index < len(first_positions)
            else remaining_position
        )
        values = list(BASE)
        values[7] = position
        targets.append(Target((index + 1) * 50_000_000, tuple(values)))
    return tuple(targets)


@pytest.mark.parametrize(
    ("name", "start", "first_positions", "remaining", "expected_raw"),
    (
        (
            "min-boundary",
            0.002,
            (0.0016, 0.0012, 0.0008, 0.0004, 0.0),
            0.0,
            -0.008,
        ),
        (
            "max-boundary",
            0.078,
            (0.0784, 0.0788, 0.0792, 0.0796, 0.08),
            0.08,
            0.008,
        ),
    ),
)
def test_gripper_joint_hard_boundary_caps_terminal_speed(
    name,
    start,
    first_positions,
    remaining,
    expected_raw,
):
    source = gripper_boundary_source(first_positions, remaining)
    window = make_window(source, handoff_ns=0)
    anchor_positions = list(BASE)
    anchor_positions[7] = start
    anchor = make_anchor(window, anchor_positions, (0.0,) * JOINT_COUNT)
    prepared = prepare_follower_plan(
        anchor,
        window,
        PROFILE,
        CONTROL,
        FOLLOWER,
        is_final=False,
    )
    assert prepared.raw_terminal_velocities[7] == pytest.approx(expected_raw)
    assert prepared.restricted_terminal_velocities[7] == 0.0
    assert prepared.flags.velocity_limited[7] is True
    assert prepared.flags.boundary_preserved[7] is True
    assert prepared.actual_terminal_positions[7] >= -1e-9
    assert prepared.actual_terminal_positions[7] <= 0.08 + 1e-9
    assert prepared.actual_terminal_velocities[7] == pytest.approx(0.0, abs=1e-9)
    assert_plan_c1_and_limits(prepared)


def test_successive_windows_approach_each_joint_boundary_with_inherited_state():
    for start, goals in (
        (0.076, (0.078, 0.079, 0.0795, 0.0798, 0.08)),
        (0.004, (0.002, 0.001, 0.0005, 0.0002, 0.0)),
    ):
        current_positions = list(BASE)
        current_velocities = [0.0] * JOINT_COUNT
        current_positions[7] = start
        handoff_ns = 0
        for index, goal in enumerate(goals):
            source = tuple(
                Target(
                    handoff_ns + (point + 1) * 50_000_000,
                    tuple(
                        (
                            current_positions[axis]
                            + (goal - current_positions[7])
                            * ((point + 1) / 5.0)
                            if axis == 7
                            else current_positions[axis]
                        )
                        for axis in range(JOINT_COUNT)
                    ),
                )
                for point in range(5)
            )
            window = make_window(source, handoff_ns=handoff_ns)
            anchor = ChunkAnchor(
                "boundary-approach",
                index,
                handoff_ns,
                ReferenceSample(
                    tuple(current_positions),
                    tuple(current_velocities),
                    (0.0,) * JOINT_COUNT,
                ),
                PROFILE.digest,
            )
            prepared = prepare_follower_plan(
                anchor,
                window,
                PROFILE,
                CONTROL,
                FOLLOWER,
                is_final=False,
            )
            assert_plan_c1_and_limits(prepared)
            assert prepared.actual_terminal_positions[7] >= -1e-9
            assert prepared.actual_terminal_positions[7] <= 0.08 + 1e-9
            current_positions = list(prepared.actual_terminal_positions)
            current_velocities = list(prepared.actual_terminal_velocities)
            handoff_ns += prepared.duration_ns
        assert current_positions[7] == pytest.approx(goals[-1], abs=1e-8)
        assert current_velocities[7] == pytest.approx(0.0, abs=1e-9)
