"""Regression for explicit gripper transition retiming."""

from __future__ import annotations

import pytest

from robot_contracts.action_chunks import ActionWindow, ChunkAnchor
from robot_contracts.models import JointSpec, Target
from robot_contracts.trajectory.spline import ReferenceSample
from robot_trajectory.transition_retiming import (
    gripper_axes,
    reachable_bounds,
    retime_action_window,
)

JOINT_NAMES = (
    "left_joint1", "left_joint2", "left_joint3", "left_joint4", "left_joint5",
    "left_joint6", "left_joint7", "left_gripper_width",
    "right_joint1", "right_joint2", "right_joint3", "right_joint4", "right_joint5",
    "right_joint6", "right_joint7", "right_gripper_width",
)
AMAX = tuple(1.0 if "gripper" not in name else 0.5 for name in JOINT_NAMES)
BUDGET = tuple(0.3 if "gripper" not in name else 0.04 for name in JOINT_NAMES)


def _joints() -> tuple[JointSpec, ...]:
    return tuple(
        JointSpec(
            name=name,
            unit="m" if name.endswith("gripper_width") else "rad",
            min=0.0 if name.endswith("gripper_width") else -3.0,
            max=0.08 if name.endswith("gripper_width") else 3.0,
            max_velocity=0.04 if name.endswith("gripper_width") else 0.8,
            initial=0.0,
        )
        for name in JOINT_NAMES
    )


def _anchor(positions, velocities) -> ChunkAnchor:
    return ChunkAnchor(
        "stream", 0, 1_050_000_000,
        ReferenceSample(tuple(positions), tuple(velocities), (0.0,) * 16),
        "profile",
    )


def _right_gripper_window(right: list[float], left: list[float] | None = None):
    left = left if left is not None else [0.0] * len(right)
    source = []
    for index in range(60):
        values = [0.0] * 16
        if 21 <= index <= 40:
            values[7] = left[index - 21]
            values[15] = right[index - 21]
        source.append(Target((index + 1) * 50_000_000, tuple(values)))
    return ActionWindow(1, 0, 1_050_000_000, tuple(source), 20)


def test_retimes_a_stationary_start_on_the_first_selected_point():
    window = _right_gripper_window([0.079865] * 20)
    anchor = _anchor(
        [0.0] * 15 + [0.022205447806916776],
        [0.0] * 15 + [0.04],
    )
    result = retime_action_window(
        window, anchor, _joints(), AMAX, BUDGET, gripper_axes(JOINT_NAMES))
    assert result.retimed
    assert [axis.name for axis in result.axes] == ["right_gripper_width"]
    axis = result.axes[0]
    assert axis.axis == 15
    assert axis.original_positions[0] == pytest.approx(0.079865)
    assert axis.original_positions[-1] == pytest.approx(0.079865)
    # Reachable at 50 ms from (0.022205..., 0.04): 0.022205 + 0.04 * 0.05.
    assert axis.adapted_positions[0] == pytest.approx(0.024205447806916774)
    assert axis.adapted_positions[-1] == pytest.approx(0.062205447806916774)
    assert axis.original_excess == pytest.approx(0.05565955219308323)
    assert result.window.selected_indices == window.selected_indices
    assert [target.time_ns for target in result.window.source_targets] == [
        target.time_ns for target in window.source_targets]
    assert result.window.source_targets[41:] == window.source_targets[41:]


def test_retimes_a_reverse_switch_from_a_closing_velocity():
    right = [0.08139313757419586] * 20
    right[-1] = 0.061016
    window = _right_gripper_window(right, left=right)
    anchor = _anchor(
        [0.0] * 7 + [0.062495712249460864] + [0.0] * 7 + [0.0814],
        [0.0] * 7 + [-0.02887703449999999] + [0.0] * 7 + [0.0],
    )
    budgets = tuple(0.3 if "gripper" not in name else 0.01 for name in JOINT_NAMES)
    result = retime_action_window(
        window, anchor, _joints(), AMAX, budgets, gripper_axes(JOINT_NAMES))
    assert result.retimed
    axis = next(item for item in result.axes if item.name == "left_gripper_width")
    assert axis.original_excess == pytest.approx(0.019716277049734998)
    # The first point is clamped to the decelerating envelope, the last stays.
    assert axis.adapted_positions[0] == pytest.approx(0.061676860524460864)
    assert axis.adapted_positions[-1] == pytest.approx(0.061016)
    # Hand-off-relative time on the real window: 0.05, 0.10, ...
    # Independent hand-computed bounds at the first two points.
    assert window.targets[0].time_ns == 50_000_000
    assert reachable_bounds(
        0.062495712249460864, -0.02887703449999999, _joints()[7], 0.5, 0.05,
    ) == pytest.approx((0.060619432610975056, 0.061676860524460864))
    assert reachable_bounds(
        0.062495712249460864, -0.02887703449999999, _joints()[7], 0.5, 0.10,
    ) == pytest.approx((0.058619432610975054, 0.062108008799460865))
    assert axis.adapted_positions[0] == pytest.approx(0.061676860524460864)
    assert axis.adapted_positions[1] == pytest.approx(0.062108008799460865)
    assert axis.adapted_positions[2] == pytest.approx(0.062273755412041734)
    assert 0.05661943261097505 < axis.adapted_positions[2] < 0.06375166636794667
    for target, value in zip(window.targets, axis.adapted_positions, strict=True):
        lower, upper = reachable_bounds(
            0.062495712249460864, -0.02887703449999999, _joints()[7], 0.5,
            target.time_ns / 1_000_000_000)
        assert lower <= value <= upper


def test_keeps_targets_that_are_reachable_within_the_budget():
    right = [0.0222 + 0.0005 * index for index in range(20)]
    window = _right_gripper_window(right)
    anchor = _anchor([0.0] * 15 + [0.0222], [0.0] * 15 + [0.04])
    result = retime_action_window(
        window, anchor, _joints(), AMAX, BUDGET, gripper_axes(JOINT_NAMES))
    assert not result.retimed
    assert result.axes == ()
    assert result.window is window


def test_window_internal_direction_change_keeps_times_and_unselected_tail():
    right = [0.08] * 8 + [0.0] * 12
    window = _right_gripper_window(right)
    anchor = _anchor([0.0] * 15 + [0.0222], [0.0] * 15 + [0.04])
    result = retime_action_window(
        window, anchor, _joints(), AMAX, BUDGET, gripper_axes(JOINT_NAMES))
    assert result.retimed
    changed = [
        index for index, (before, after) in enumerate(
            zip(window.source_targets, result.window.source_targets, strict=True))
        if before != after
    ]
    assert changed == list(range(21, 40))
    assert 40 not in changed  # the last target already equals its adapted value
    assert all(
        before.time_ns == after.time_ns
        for before, after in zip(
            window.source_targets, result.window.source_targets, strict=True)
    )
    assert result.window.selected_indices == tuple(range(21, 41))


def test_only_profile_gripper_axes_are_retimed():
    source = []
    for index in range(60):
        values = [0.0] * 16
        if 21 <= index <= 40:
            values[0] = 1.0  # arm axis far beyond its reachable envelope
            values[15] = 0.08
        source.append(Target((index + 1) * 50_000_000, tuple(values)))
    window = ActionWindow(1, 0, 1_050_000_000, tuple(source), 20)
    anchor = _anchor([0.0] * 15 + [0.0222], [0.0] * 16)
    result = retime_action_window(
        window, anchor, _joints(), AMAX, BUDGET, gripper_axes(JOINT_NAMES))
    assert [axis.name for axis in result.axes] == ["right_gripper_width"]
    assert result.window.source_targets[21].positions[0] == pytest.approx(1.0)


def test_retiming_is_deterministic_and_stateless():
    window = _right_gripper_window([0.079865] * 20)
    anchor = _anchor([0.0] * 15 + [0.022205447806916776], [0.0] * 15 + [0.04])
    first = retime_action_window(
        window, anchor, _joints(), AMAX, BUDGET, gripper_axes(JOINT_NAMES))
    second = retime_action_window(
        window, anchor, _joints(), AMAX, BUDGET, gripper_axes(JOINT_NAMES))
    assert first.window == second.window
    assert first.axes == second.axes
    # The input window remains the original prediction.
    assert window.source_targets[21].positions[15] == pytest.approx(0.079865)


def test_coordinated_arm_group_scales_all_moving_arm_axes_proportionally():
    import math
    from robot_contracts.models import coordinated_axis_groups

    joints = _joints()
    groups = coordinated_axis_groups(joints)
    assert groups == ((0, 1, 2, 3, 4, 5, 6), (8, 9, 10, 11, 12, 13, 14))

    initial = [0.0] * 16
    initial[15] = 0.08
    # right_joint3 (index 10) moves 0.80 rad in 1.0 s (beyond 0.48 reachable & 0.25 stop bound);
    # right_joint1 (8), right_joint2 (9), right_joint4 (11) stay within their own budgets;
    # left_joint1 (0) and right_gripper_width (15) belong to other groups / independent axes.
    end_targets = list(initial)
    end_targets[0] = 0.10
    end_targets[8] = 0.20
    end_targets[9] = -0.16
    end_targets[10] = 0.80
    end_targets[11] = 0.12

    source = []
    for index in range(60):
        values = list(initial)
        if 21 <= index <= 40:
            values = list(end_targets)
        source.append(Target((index + 1) * 50_000_000, tuple(values)))
    window = ActionWindow(1, 0, 1_050_000_000, tuple(source), 20)
    anchor = _anchor(initial, [0.0] * 16)
    all_axes = tuple(range(16))

    result = retime_action_window(
        window, anchor, joints, AMAX, BUDGET, all_axes, axis_groups=groups
    )
    assert result.retimed
    retimed_names = [item.name for item in result.axes]
    assert set(retimed_names) == {
        "right_joint1",
        "right_joint2",
        "right_joint3",
        "right_joint4",
    }

    expected_alpha = 0.25 / 0.80
    adapted_end = result.window.targets[-1].positions
    assert adapted_end[0] == pytest.approx(0.10)
    assert adapted_end[15] == pytest.approx(0.08)
    assert adapted_end[10] == pytest.approx(0.25)
    assert adapted_end[8] == pytest.approx(0.20 * expected_alpha)
    assert adapted_end[9] == pytest.approx(-0.16 * expected_alpha)
    assert adapted_end[11] == pytest.approx(0.12 * expected_alpha)

    orig_vec = [end_targets[i] - initial[i] for i in range(8, 15)]
    adap_vec = [adapted_end[i] - initial[i] for i in range(8, 15)]
    dot = sum(a * b for a, b in zip(orig_vec, adap_vec, strict=True))
    norm_orig = math.sqrt(sum(a * a for a in orig_vec))
    norm_adap = math.sqrt(sum(b * b for b in adap_vec))
    assert dot / (norm_orig * norm_adap) == pytest.approx(1.0)


@pytest.mark.parametrize(
    ("position", "velocity", "time_s", "expected"),
    [
        (0.0, 0.0, 1.0, (0.0, 0.0384)),
        (0.02, 0.04, 0.05, (0.021375, 0.022)),
        (0.0625, -0.0288770345, 0.05, (0.06062372036151419, 0.061681148275)),
    ],
)
def test_reachable_bounds_match_the_documented_envelope(position, velocity, time_s, expected):
    joint = _joints()[15]
    lower, upper = reachable_bounds(position, velocity, joint, 0.5, time_s)
    assert lower == pytest.approx(expected[0], abs=1e-9)
    assert upper == pytest.approx(expected[1], abs=1e-9)
