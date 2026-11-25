from __future__ import annotations

import json
import math
import random
import sys
from pathlib import Path

import pytest
from robot_contracts.action_chunks import ActionWindow, ChunkAnchor
from robot_contracts.models import Target
from robot_contracts.profiles import load_profile
from robot_contracts.trajectory.constants import POSITION_ATOL, RELATIVE_TOLERANCE, VELOCITY_ATOL
from robot_contracts.trajectory.spline import ReferenceSample
from robot_trajectory.follower import PreparedFollowerPlan
from robot_trajectory.spline_math import check_continuous_limits, compute_reference_extremes

ROOT = Path(__file__).resolve().parents[3]
PROFILES = ROOT / "shared/scenarios/marvin/mujoco/stack/profiles"
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from follower_fixtures import generate_prepared_follower  # noqa: E402
from robot_backend_sim.common.config.follower_config import load_follower_config  # noqa: E402
from robot_backend_sim.common.config.motion_config import load_control_config  # noqa: E402
from scenario_fixtures import MARVIN_SHARED_PROFILES  # noqa: E402

PROFILE = load_profile(
    PROFILES / "robot/profile.json",
    ROOT / "shared/common/interfaces/schemas",
)
CONTROL = load_control_config(MARVIN_SHARED_PROFILES / "control/marvin_control.yaml")
# These trajectory fixtures use ten-point windows independently of scenario cadence.
FOLLOWER_PATH = MARVIN_SHARED_PROFILES / "control/marvin_follower.yaml"
FOLLOWER = load_follower_config(FOLLOWER_PATH).model_copy(update={"target_count": 10})
BASE = PROFILE.initial_positions
JOINT_COUNT = len(BASE)

def smoothstep(fraction: float) -> float:
    value = min(max(fraction, 0.0), 1.0)
    return 3.0 * value * value - 2.0 * value * value * value


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
    target_count=FOLLOWER.target_count,
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
    for c0, c1, c2, c3 in zip(segment.c0, segment.c1, segment.c2, segment.c3, strict=True):
        c3_s = c3 * local_s
        positions.append(((c3_s + c2) * local_s + c1) * local_s + c0)
        velocities.append((3.0 * c3_s + 2.0 * c2) * local_s + c1)
        accelerations.append(6.0 * c3_s + 2.0 * c2)
    return tuple(positions), tuple(velocities), tuple(accelerations)


def assert_plan_c1_and_limits(prepared: PreparedFollowerPlan) -> None:
    for index in range(1, len(prepared.plan.segments)):
        previous = prepared.plan.segments[index - 1]
        current = prepared.plan.segments[index]
        previous_positions, previous_velocities, _ = segment_state(previous, previous.duration_s)
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


def counterexample_window(lead_ms: int = 250) -> ActionWindow:
    targets = make_source_targets(BASE, (BASE[0] + 0.04,) + BASE[1:], 3.0)
    return make_window(targets, handoff_ns=lead_ms * 1_000_000)


def test_first_window_counterexample_and_bounded_follower():
    window = counterexample_window(250)
    anchor = make_anchor(window, BASE, (0.0,) * JOINT_COUNT)

    prepared = generate_prepared_follower(
        anchor,
        window,
        PROFILE,
        CONTROL,
        FOLLOWER,
        is_final=False,
    )
    assert prepared.duration_ns == window.end_time_ns
    assert prepared.plan.duration_ns == prepared.requested_window_ns
    assert prepared.window == window
    assert prepared.window.source_targets == window.source_targets
    assert prepared.window.targets == window.targets
    assert max(prepared.max_target_deviations) <= max(prepared.max_target_deviation_limits)
    assert_plan_c1_and_limits(prepared)
    assert (
        prepared.raw_terminal_velocities[0]
        != pytest.approx(prepared.restricted_terminal_velocities[0])
        or prepared.flags.velocity_limited[0] is False
    )

    document = prepared.to_document()
    payload = json.dumps(document, allow_nan=False, indent=2)
    assert "bounded_target_follower" in payload
    assert "max_target_deviations" in payload
    assert "actual_terminal_velocities" in payload


@pytest.mark.parametrize("lead_ms", (50, 100, 250))
def test_two_radian_ten_second_fragment_with_three_leads(lead_ms):
    targets = make_source_targets(BASE, (BASE[0] + 2.0,) + BASE[1:], 10.0)
    window = make_window(targets, handoff_ns=lead_ms * 1_000_000)
    anchor = make_anchor(window, BASE, (0.0,) * JOINT_COUNT)

    for is_final in (False, True):
        prepared = generate_prepared_follower(
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
        else:
            assert prepared.plan.duration_ns == window.end_time_ns
        assert max(prepared.max_target_deviations) <= max(prepared.max_target_deviation_limits)
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
            target_positions.append(clip_to_joint(joint, anchor_positions[-1] + target_offset))
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
        prepared = generate_prepared_follower(
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
    first = generate_prepared_follower(
        first_anchor,
        first_window,
        PROFILE,
        CONTROL,
        FOLLOWER,
        is_final=False,
    )
    assert first.flags.boundary_preserved[0] is False
    assert first.actual_terminal_positions[0] < first_window.targets[-1].positions[0]

    successor_handoff_ns = first.end_backend_time_ns
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
    second = generate_prepared_follower(
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
    offsets = (0.0, 0.003, 0.006, 0.009, 0.012, 0.015, 0.020, 0.025, 0.029, 0.045)
    positions = tuple(BASE[0] + offset for offset in offsets)
    source_targets = tuple(
        Target(300_000_000 + index * 50_000_000, (position,) + BASE[1:])
        for index, position in enumerate(positions)
    )
    window = make_window(source_targets)
    anchor = make_anchor(window, BASE, (0.0,) * JOINT_COUNT)
    prepared = generate_prepared_follower(
        anchor,
        window,
        PROFILE,
        CONTROL,
        FOLLOWER,
        is_final=False,
    )
    raw = (positions[-1] - positions[-2]) / 0.05
    approach_cap = math.sqrt(2.0 * CONTROL.max_accelerations[0] * offsets[-1])
    assert prepared.raw_terminal_velocities[0] == pytest.approx(raw)
    assert prepared.restricted_terminal_velocities[0] == pytest.approx(approach_cap)
    assert prepared.flags.velocity_limited[0] is True
    assert prepared.window.targets[-1].positions[0] == positions[-1]
    assert prepared.window.source_targets == window.source_targets
    assert_plan_c1_and_limits(prepared)


def gripper_boundary_source(first_positions, remaining_position):
    targets = []
    for index in range(60):
        position = first_positions[index] if index < len(first_positions) else remaining_position
        values = list(BASE)
        values[7] = position
        targets.append(Target((index + 1) * 50_000_000, tuple(values)))
    return tuple(targets)


@pytest.mark.parametrize("boundary", ("minimum", "maximum"))
def test_gripper_joint_hard_boundary_caps_terminal_speed(boundary):
    gripper = PROFILE.joints[7]
    limit = getattr(gripper, boundary)
    direction = -1 if boundary == "minimum" else 1
    start = limit - direction * 0.002
    first_positions = tuple(start + direction * index * 0.0002 for index in range(9))
    source = gripper_boundary_source((*first_positions, limit), limit)
    window = make_window(source, handoff_ns=0)
    anchor_positions = list(BASE)
    anchor_positions[7] = start
    anchor = make_anchor(window, anchor_positions, (0.0,) * JOINT_COUNT)
    prepared = generate_prepared_follower(
        anchor,
        window,
        PROFILE,
        CONTROL,
        FOLLOWER,
        is_final=False,
    )
    assert prepared.raw_terminal_velocities[7] == pytest.approx(direction * 0.008)
    assert prepared.restricted_terminal_velocities[7] == 0.0
    assert prepared.flags.velocity_limited[7] is True
    assert prepared.flags.boundary_preserved[7] is True
    assert prepared.actual_terminal_positions[7] == pytest.approx(limit, abs=1e-9)
    assert prepared.actual_terminal_velocities[7] == pytest.approx(0.0, abs=1e-9)
    assert_plan_c1_and_limits(prepared)


def test_successive_windows_approach_each_joint_boundary_with_inherited_state():
    gripper = PROFILE.joints[7]
    for limit, direction in ((gripper.minimum, -1), (gripper.maximum, 1)):
        start = limit - direction * 0.004
        goals = tuple(limit - direction * gap for gap in (0.002, 0.001, 0.0005, 0.0002, 0.0))
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
                            + (goal - current_positions[7]) * ((point + 1) / FOLLOWER.target_count)
                            if axis == 7
                            else current_positions[axis]
                        )
                        for axis in range(JOINT_COUNT)
                    ),
                )
                for point in range(FOLLOWER.target_count)
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
            prepared = generate_prepared_follower(
                anchor,
                window,
                PROFILE,
                CONTROL,
                FOLLOWER,
                is_final=False,
            )
            assert_plan_c1_and_limits(prepared)
            assert prepared.actual_terminal_positions[7] >= gripper.minimum - 1e-9
            assert prepared.actual_terminal_positions[7] <= gripper.maximum + 1e-9
            current_positions = list(prepared.actual_terminal_positions)
            current_velocities = list(prepared.actual_terminal_velocities)
            handoff_ns += prepared.duration_ns
        assert current_positions[7] == pytest.approx(goals[-1], abs=1e-8)
        assert current_velocities[7] == pytest.approx(0.0, abs=1e-9)
