"""Continuous and cross-chunk constraints for Runtime's trajectory generator."""

from itertools import pairwise

import pytest
from model_helpers import replace
from robot_contracts.action_chunks import ChunkAnchor
from robot_contracts.models import (
    CameraSpec,
    ContractError,
    JointSpec,
    RobotProfile,
    RunContext,
    Target,
)
from robot_contracts.stream_request import (
    REQUEST_STREAM_FIRST,
    REQUEST_STREAM_NEXT,
    StreamRequest,
    Trajectory,
)
from robot_contracts.trajectory.follower_reference import FollowerValidationLimits
from robot_contracts.trajectory.spline import ReferenceSample
from robot_trajectory.sampling import sample_reference_interval
from robot_trajectory.spline_math import check_continuous_limits, compute_reference_extremes
from robot_trajectory.trajectory_generation import generate_follower_plan


def _inputs(goal, *, speed=(0.0, 0.0), final=False, duration_ns=400_000_000):
    profile = RobotProfile(
        "robot",
        "fixture",
        "digest",
        True,
        (
            JointSpec("a", "rad", -1.0, 1.0, 1.0, 0.0),
            JointSpec("b", "rad", -1.0, 1.0, 1.0, 0.0),
        ),
        (CameraSpec("front", 64, 48, "front", "0" * 64),),
    )
    context = RunContext("robot", profile.digest, "backend", "time", 0)
    start_time_ns = 100_000_000
    anchor = ChunkAnchor(
        "stream",
        0,
        start_time_ns,
        ReferenceSample((0.0, 0.0), speed, (0.0, 0.0)),
        profile.digest,
    )
    request = StreamRequest(
        trajectory=Trajectory(
            context,
            "lease",
            "command",
            1,
            1,
            0,
            profile.joint_names,
            (Target(duration_ns // 2, goal), Target(duration_ns, goal)),
        ),
        kind=REQUEST_STREAM_FIRST,
        anchor=anchor,
        observation_time_ns=start_time_ns,
        target_count=2,
        settle_at_end=final,
    )
    limits = FollowerValidationLimits(
        joint_names=profile.joint_names,
        max_accelerations=(2.0, 2.5),
        max_target_deviations=(0.15, 0.15),
        target_count=2,
        max_terminal_tail_duration_ns=3_000_000_000,
        duration_grid_ns=2_000_000,
    )
    return request, profile, limits


def _check_continuous(plan, request, profile, limits):
    assert plan.segments[0].c0 == request.anchor.reference.positions
    assert plan.segments[0].c1 == request.anchor.reference.velocities
    check_continuous_limits(
        compute_reference_extremes(plan),
        profile.joints,
        limits.max_accelerations,
    )
    for preceding, following in pairwise(plan.segments):
        elapsed = preceding.duration_s
        assert following.c0 == pytest.approx(
            [
                p + (v + a * elapsed) * elapsed
                for p, v, a in zip(preceding.c0, preceding.c1, preceding.c2, strict=True)
            ],
            abs=1e-12,
        )
        assert following.c1 == pytest.approx(
            [v + 2.0 * a * elapsed for v, a in zip(preceding.c1, preceding.c2, strict=True)],
            abs=1e-12,
        )


def test_feasible_nonfinal_plan_is_deterministic_and_reaches_goal():
    inputs = _inputs((0.047, -0.061), speed=(0.08, -0.07))
    plan = generate_follower_plan(*inputs)
    assert plan == generate_follower_plan(*inputs)
    assert plan.duration_ns == 400_000_000
    _check_continuous(plan, *inputs)
    end = sample_reference_interval(plan, plan.duration_ns)
    assert end.positions == pytest.approx((0.047, -0.061), abs=1e-9)
    assert end.velocities == pytest.approx((0.0, 0.0), abs=1e-9)


def test_nonfinal_uses_prefix_without_moving_handoff():
    inputs = _inputs((0.35, 0.01))
    plan = generate_follower_plan(*inputs)
    _check_continuous(plan, *inputs)
    assert plan.duration_ns == 400_000_000
    end = sample_reference_interval(plan, plan.duration_ns)
    assert 0.0 < end.positions[0] < 0.35
    assert end.positions[1] == pytest.approx(0.01, abs=1e-9)


def test_final_uses_grid_aligned_tail_and_holds_shorter_axes():
    inputs = _inputs((0.35, -0.01), final=True, duration_ns=401_000_000)
    plan = generate_follower_plan(*inputs)
    _check_continuous(plan, *inputs)
    assert 402_000_000 < plan.duration_ns <= 3_000_000_000
    assert plan.duration_ns % 2_000_000 == 0
    end = sample_reference_interval(plan, plan.duration_ns)
    assert end.positions == pytest.approx((0.35, -0.01), abs=1e-9)
    assert end.velocities == pytest.approx((0.0, 0.0), abs=1e-9)


def test_nonfinal_handoff_requires_exact_grid():
    with pytest.raises(ContractError, match="FOLLOWER_GRID_MISMATCH"):
        generate_follower_plan(*_inputs((0.01, 0.01), duration_ns=401_000_000))


def test_handoff_inherits_end_velocity_and_position_without_stopping():
    request, profile, limits = _inputs((0.11, 0.0), speed=(0.22, 0.0))
    trajectory = replace(
        request.trajectory,
        targets=(Target(200_000_000, (0.045, 0.0)), Target(400_000_000, (0.11, 0.0))),
    )
    first = replace(request, trajectory=trajectory)
    plan = generate_follower_plan(first, profile, limits)
    end = sample_reference_interval(plan, plan.duration_ns)
    assert end.velocities[0] == pytest.approx(0.325)
    time_ns = first.anchor.start_backend_time_ns + plan.duration_ns
    next_request = replace(
        first,
        trajectory=replace(
            trajectory,
            command_seq=2,
            targets=(Target(200_000_000, (0.16, 0.0)), Target(400_000_000, (0.19, 0.0))),
        ),
        kind=REQUEST_STREAM_NEXT,
        anchor=ChunkAnchor("stream", 1, time_ns, end, profile.digest),
        observation_time_ns=time_ns,
    )
    next_plan = generate_follower_plan(next_request, profile, limits)
    _check_continuous(next_plan, next_request, profile, limits)
    assert next_plan.segments[0].c0 == end.positions
    assert next_plan.segments[0].c1 == end.velocities


def test_outward_boundary_velocity_does_not_yield_an_unsafe_plan():
    request, profile, limits = _inputs((0.90, 0.0), speed=(0.50, 0.0))
    start = replace(request.anchor.reference, positions=(0.99, 0.0))
    request = replace(request, anchor=replace(request.anchor, reference=start))
    with pytest.raises(ContractError, match="FOLLOWER_INFEASIBLE"):
        generate_follower_plan(request, profile, limits)


def test_final_unreachable_under_tail_bound_fails_admission():
    request, profile, limits = _inputs((0.35, 0.0), final=True)
    with pytest.raises(ContractError, match="FOLLOWER_INFEASIBLE"):
        generate_follower_plan(
            request, profile, replace(limits, max_terminal_tail_duration_ns=400_000_000)
        )
