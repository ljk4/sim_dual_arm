"""Real-Ruckig regression: unreachable gripper transitions stay rejected,
and the explicit retimed request is the one that is accepted."""

from __future__ import annotations

import re
from pathlib import Path

import pytest
from robot_backend_sim.common.config.follower_config import FollowerConfig
from robot_backend_sim.common.config.motion_config import ControlConfig
from robot_backend_sim.common.follower.follower_plan import prepare_follower_plan
from robot_contracts.action_chunks import ActionWindow, ChunkAnchor
from robot_contracts.models import ContractError, RobotProfile, Target
from robot_contracts.profiles import load_profile
from robot_contracts.trajectory.spline import ReferenceSample
from robot_trajectory.sampling import sample_reference_interval
from robot_trajectory.transition_retiming import retime_action_window

ROOT = Path(__file__).resolve().parents[2]
SCHEMAS = ROOT / "shared/common/interfaces/schemas"

STACK = {
    "profile": (
        ROOT / "shared/scenarios/marvin_sim/stack/profiles/robots/marvin_stack_v1/profile.json"
    ),
    "axis": "right_gripper_width",
    "anchor_position": 0.022205447806916776,
    "anchor_velocity": 0.04,
    "targets": [0.0798648298, 0.0799270198, 0.0799685493, 0.08, 0.08, 0.08, 0.08, 0.08,
                0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08, 0.08],
    "budget": 0.04,
    "original_deviation": 0.055660997126913504,
    "original_target_deviation": 0.05565938197175449,
}
HANDOVER = {
    "profile": (ROOT / "shared/scenarios/marvin_sim/handover/profiles/robots"
                / "marvin_handover_v1/profile.json"),
    "axis": "left_gripper_width",
    "anchor_position": 0.062495712249460864,
    "anchor_velocity": -0.02887703449999999,
    "targets": [0.0813931376, 0.0813669786, 0.0813036561, 0.080953598, 0.0807412416,
                0.0799215138, 0.0796599761, 0.0782527924, 0.0778305009, 0.0766446739,
                0.0752321482, 0.0736427903, 0.0721163824, 0.0712288246, 0.0688857734,
                0.0675751567, 0.0655966103, 0.0638245493, 0.0624155514, 0.0610159598],
    "budget": 0.01,
    "original_deviation": 0.019727525391288196,
    "original_target_deviation": 0.019716277049734998,
}
CASES = [
    pytest.param(STACK, id="stack-opening-transition"),
    pytest.param(HANDOVER, id="handover-closing-transition"),
]


def _control(profile: RobotProfile) -> ControlConfig:
    return ControlConfig(
        joint_names=profile.joint_names,
        max_accelerations=tuple(0.5 if "gripper" in name else 1.0 for name in profile.joint_names),
        position_tolerances=tuple(0.002 if "gripper" in name else 0.012
                                  for name in profile.joint_names),
        velocity_tolerances=tuple(0.004 if "gripper" in name else 0.025
                                  for name in profile.joint_names),
        stable_duration_s=0.2,
        settle_timeout_s=2.0,
        motion_wall_timeout_s=10.0,
        stop_wall_timeout_s=3.0,
        contact_force_min_n=0.1,
        finger_velocity_max_m_s=0.02,
    )


def _follower(profile: RobotProfile, budget: float) -> FollowerConfig:
    deviations = tuple(budget if joint.unit == "m" else 0.3 for joint in profile.joints)
    return FollowerConfig(
        joint_names=profile.joint_names,
        target_count=20,
        max_target_deviations=deviations,
        max_terminal_tail_duration_s=3.0,
    )


def _window(case, profile: RobotProfile) -> ActionWindow:
    axis = profile.joint_names.index(case["axis"])
    initial = [joint.initial for joint in profile.joints]
    source = []
    for index in range(60):
        values = list(initial)
        if 21 <= index <= 40:
            values[axis] = case["targets"][index - 21]
        source.append(Target((index + 1) * 50_000_000, tuple(values)))
    return ActionWindow(1, 0, 1_050_000_000, tuple(source), 20)


def _anchor(case, profile: RobotProfile) -> ChunkAnchor:
    axis = profile.joint_names.index(case["axis"])
    positions = [joint.initial for joint in profile.joints]
    velocities = [0.0] * 16
    positions[axis] = case["anchor_position"]
    velocities[axis] = case["anchor_velocity"]
    return ChunkAnchor(
        "stream", 0, 1_050_000_000,
        ReferenceSample(tuple(positions), tuple(velocities), (0.0,) * 16),
        profile.digest,
    )


@pytest.mark.parametrize("case", CASES)
def test_original_gripper_transition_is_still_rejected(case):
    profile = load_profile(case["profile"], SCHEMAS)
    with pytest.raises(ContractError) as error:
        prepare_follower_plan(
            _anchor(case, profile), _window(case, profile), profile, _control(profile),
            _follower(profile, case["budget"]), is_final=False,
            duration_grid_ns=2_000_000,
        )
    assert error.value.reason == "FOLLOWER_DEVIATION_EXCEEDED"
    assert case["axis"] in str(error.value)
    measured = float(re.search(r"deviation ([0-9.eE+-]+) exceeds", str(error.value)).group(1))
    assert measured == pytest.approx(case["original_deviation"], rel=1e-6)


@pytest.mark.parametrize("case", CASES)
def test_retimed_gripper_transition_is_executable_and_reports_both_deviations(case):
    profile = load_profile(case["profile"], SCHEMAS)
    control = _control(profile)
    follower = _follower(profile, case["budget"])
    anchor = _anchor(case, profile)
    window = _window(case, profile)
    linear_axes = tuple(
        index for index, joint in enumerate(profile.joints) if joint.unit == "m"
    )
    retimed = retime_action_window(
        window, anchor, profile.joints, control.max_accelerations,
        follower.max_target_deviations, linear_axes)
    assert retimed.retimed
    prepared = prepare_follower_plan(
        anchor, retimed.window, profile, control, follower, is_final=False,
        duration_grid_ns=2_000_000,
    )
    axis = profile.joint_names.index(case["axis"])
    assert prepared.max_target_deviations[axis] <= case["budget"]
    assert prepared.plan.duration_ns == retimed.window.end_time_ns
    # The executed reference still differs from the original model target by
    # the documented unavoidable amount; that original deviation is not hidden.
    worst = 0.0
    for target in window.targets:
        reference = sample_reference_interval(prepared.plan, target.time_ns)
        worst = max(worst, abs(reference.positions[axis] - target.positions[axis]))
    assert worst == pytest.approx(case["original_target_deviation"], abs=1e-4)
