"""Unit evidence for the independent bounded-follower reference record."""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest
from model_helpers import replace
from robot_contracts.action_chunks import ActionWindow, ChunkAnchor
from robot_contracts.models import (
    CameraSpec,
    ContractError,
    JointSpec,
    RobotProfile,
    RunContext,
    Target,
)
from robot_contracts.stream_request import REQUEST_STREAM_FIRST, StreamRequest, Trajectory
from robot_contracts.trajectory.follower_reference import (
    FollowerReference,
    FollowerValidationLimits,
    compute_follower_reference_digest,
)
from robot_contracts.trajectory.spline import CubicSegment, ReferencePlan, ReferenceSample
from robot_trajectory.follower import PreparedFollowerPlan
from robot_trajectory.follower_reference import (
    applied_follower_sample,
    follower_reference_from_prepared,
    sample_follower_reference,
    validate,
)
from robot_trajectory.sampling import sample_reference_interval

ROOT = Path(__file__).resolve().parents[2]
PROFILE_DIGEST = "profile-hash"
CONTEXT = RunContext(
    "fixture-robot",
    PROFILE_DIGEST,
    "backend-instance",
    "mujoco-sim",
    0,
    episode_id="episode-follower",
    control_epoch=1,
)
TARGET_POSITION = 0.03
PLAN_NS = 150_000_000
WINDOW_NS = 250_000_000
PRODUCER = "fixture-producer"

# Deliberately smaller than the accepted round-off allowance so the
# raw endpoint is outside the hard range but the shared normalizer legally clips
# it onto the boundary.
OVERSHOOT = 5e-11


def profile() -> RobotProfile:
    return RobotProfile(
        "fixture-robot",
        "fixture-profile",
        PROFILE_DIGEST,
        True,
        (
            JointSpec("joint1", "rad", -10.0, TARGET_POSITION - OVERSHOOT, 10.0, 0.0),
            JointSpec("joint2", "rad", -10.0, 10.0, 10.0, 0.0),
        ),
        (CameraSpec("front", 64, 48, "front", "0" * 64),),
    )


def context() -> RunContext:
    return CONTEXT


def source_targets() -> tuple[Target, ...]:
    return (
        Target(300_000_000, (TARGET_POSITION, 0.0)),
        Target(400_000_000, (TARGET_POSITION, 0.0)),
    )


def window() -> ActionWindow:
    return ActionWindow(
        observation_id=1,
        observation_time_ns=0,
        handoff_time_ns=WINDOW_NS,
        source_targets=source_targets(),
        target_count=2,
    )


def anchor() -> ChunkAnchor:
    return ChunkAnchor(
        "follower-stream",
        0,
        WINDOW_NS,
        ReferenceSample((0.0, 0.0), (0.0, 0.0), (0.0, 0.0)),
        PROFILE_DIGEST,
    )


def smoothstep_plan() -> ReferencePlan:
    duration_s = PLAN_NS / 1_000_000_000
    c2 = 3.0 * TARGET_POSITION / duration_s**2
    c3 = -2.0 * TARGET_POSITION / duration_s**3
    return ReferencePlan(
        ("joint1", "joint2"),
        (CubicSegment(PLAN_NS, (0.0, 0.0), (0.0, 0.0), (c2, 0.0), (c3, 0.0)),),
    )


def prepared() -> PreparedFollowerPlan:
    return PreparedFollowerPlan(
        anchor=anchor(),
        window=window(),
        plan=smoothstep_plan(),
        joints=profile().joints,
        max_accelerations=(10.0, 10.0),
        max_target_deviation_limits=(0.15, 0.15),
        raw_terminal_velocities=(0.0, 0.0),
        restricted_terminal_velocities=(0.0, 0.0),
        is_final=False,
        requested_window_ns=WINDOW_NS - 100_000_000,
        curve_duration_ns=PLAN_NS,
        duration_grid_ns=None,
        algorithm=PRODUCER,
    )


def request() -> StreamRequest:
    return StreamRequest(
        trajectory=Trajectory(
            context(),
            "lease-1",
            "command-follower",
            1,
            1,
            10_000_000_000,
            ("joint1", "joint2"),
            source_targets(),
        ),
        kind=REQUEST_STREAM_FIRST,
        anchor=anchor(),
        observation_time_ns=0,
        target_count=2,
        settle_at_end=False,
    )


def reference(**changes) -> FollowerReference:
    base = follower_reference_from_prepared(
        prepared(),
        request(),
        motion_seq=1,
        accepted_sim_time_ns=100_000_000,
        accepted_wall_time_ns=2_000_000_000,
    )
    return base if not changes else replace(base, **changes)


def constraints() -> FollowerValidationLimits:
    return FollowerValidationLimits(
        joint_names=("joint1", "joint2"),
        max_accelerations=(10.0, 10.0),
        max_target_deviations=(0.15, 0.15),
        target_count=2,
        max_terminal_tail_duration_ns=3_000_000_000,
        duration_grid_ns=None,
    )


def test_record_is_independent_and_digest_covers_every_field():
    record = reference()
    assert record.reference_digest == compute_follower_reference_digest(record)
    assert record.algorithm == PRODUCER
    assert record.chunk_seq == 1 and record.mode == "STREAM_FIRST"
    assert record.extension_ns == 0

    rebuilt = validate(record, profile(), constraints())
    assert rebuilt.plan == record.plan
    assert rebuilt.actual_terminal_positions == record.actual_terminal_positions
    assert rebuilt.flags == record.flags


@pytest.mark.parametrize(
    ("field", "value"),
    (
        ("motion_seq", 2),
        ("accepted_sim_time_ns", 1_000_000_001),
        ("accepted_wall_time_ns", 2_000_000_001),
        ("joint_names", ("joint2", "joint1")),
        ("max_accelerations", (9.0, 10.0)),
        ("max_target_deviation_limits", (0.14, 0.15)),
        ("raw_terminal_velocities", (1.0, 0.0)),
        ("restricted_terminal_velocities", (1.0, 0.0)),
        ("actual_terminal_positions", (TARGET_POSITION - 1e-6, 0.0)),
        ("actual_terminal_velocities", (1e-6, 0.0)),
        ("actual_terminal_accelerations", (1e-6, 0.0)),
        ("max_target_deviations", (0.0, 0.0)),
        ("is_final", True),
        ("requested_window_ns", 300_000_000),
        ("curve_duration_ns", 160_000_000),
        ("duration_grid_ns", 2_000_000),
        ("velocity_limited", (True, False)),
        ("acceleration_limited", (True, False)),
        ("boundary_preserved", (False, True)),
    ),
)
def test_any_protected_field_tamper_fails(field, value):
    record = reference()
    with pytest.raises(ContractError):
        replace(record, **{field: value})


def test_tampered_raw_source_tail_is_covered():
    record = reference()
    original = record.request.trajectory.targets
    changed = (original[0], Target(original[1].time_ns, (0.031, 0.0)))
    tampered_request = replace(
        record.request,
        trajectory=replace(record.request.trajectory, targets=changed),
    )
    with pytest.raises(ContractError):
        replace(record, request=tampered_request)


def test_digest_changes_with_acceptance_time_and_same_fact_is_stable():
    first = reference()
    repeated = follower_reference_from_prepared(
        prepared(),
        request(),
        motion_seq=1,
        accepted_sim_time_ns=100_000_000,
        accepted_wall_time_ns=2_000_000_000,
    )
    assert repeated == first
    assert repeated.reference_digest == first.reference_digest
    later = follower_reference_from_prepared(
        prepared(),
        request(),
        motion_seq=1,
        accepted_sim_time_ns=120_000_000,
        accepted_wall_time_ns=2_100_000_000,
    )
    assert later.reference_digest != first.reference_digest


def test_validate_rejects_stored_evidence_that_does_not_rebuild():
    # Build a syntactically valid record whose digest really covers the wrong
    # stored endpoint; the independent DSH-030 rebuild must still reject it.
    repaired = replace(
        reference(),
        actual_terminal_positions=(0.0, 0.0),
        reference_digest=None,
    )
    assert repaired.reference_digest == compute_follower_reference_digest(repaired)
    with pytest.raises(ContractError):
        validate(repaired, profile(), constraints())


def test_independent_actual_reference_reconstruction_and_corrections():
    record = reference()
    applied = applied_follower_sample(record, record.end_sim_time_ns, profile())
    assert applied.sampled_positions[0] == pytest.approx(TARGET_POSITION)
    assert applied.applied_positions[0] == profile().joints[0].maximum
    assert applied.corrections[0] == pytest.approx(OVERSHOOT)
    assert applied.applied_positions[1] == 0.0
    assert applied.has_correction
    assert applied_follower_sample(record, record.end_sim_time_ns, profile()).applied_positions[
        0
    ] == pytest.approx(profile().joints[0].maximum)
    start = applied_follower_sample(record, record.start_sim_time_ns, profile())
    assert start.applied_positions == (0.0, 0.0)
    mid = record.start_sim_time_ns + PLAN_NS // 2
    raw = sample_follower_reference(record, mid)
    assert raw.positions == sample_reference_interval(record.plan, PLAN_NS // 2).positions


def test_same_curve_accepts_different_producer_identity():
    first = reference()
    second = replace(
        first, algorithm="another-producer", reference_digest=None
    )
    assert first.plan == second.plan
    assert second.algorithm == "another-producer"
    assert second.reference_digest != first.reference_digest
    rebuilt_first = validate(first, profile(), constraints())
    rebuilt_second = validate(second, profile(), constraints())
    assert rebuilt_first.plan == rebuilt_second.plan
    assert rebuilt_first.actual_terminal_positions == rebuilt_second.actual_terminal_positions
    assert rebuilt_first.flags == rebuilt_second.flags
    assert rebuilt_second.algorithm == second.algorithm


def test_producer_identity_must_be_a_non_empty_string():
    with pytest.raises(ContractError) as caught:
        replace(reference(), algorithm="")
    assert caught.value.reason == "INVALID_FOLLOWER_REFERENCE"


def test_digest_payload_uses_curve_duration_field():
    record = reference()
    document = record.to_document()
    assert "curve_duration_ns" in document
    assert "ruckig_duration_ns" not in document


def test_recomputed_digest_cannot_bypass_final_or_timing_intent():
    record = reference()
    with pytest.raises(ContractError):
        replace(record, is_final=True, reference_digest=None)
    with pytest.raises(ContractError):
        replace(record, accepted_sim_time_ns=300_000_000, reference_digest=None)
    with pytest.raises(ContractError):
        replace(record, accepted_sim_time_ns=10_000_000_001, reference_digest=None)


def test_zero_accepted_sim_time_is_a_real_value():
    record = reference()
    zero = replace(
        record,
        accepted_sim_time_ns=0,
        accepted_wall_time_ns=0,
        reference_digest=None,
    )
    assert zero.accepted_sim_time_ns == 0
    validate(zero, profile(), constraints())


def test_validation_limits_bind_target_count_tail_and_grid():
    record = reference()
    with pytest.raises(ContractError) as caught:
        validate(
            record,
            profile(),
            replace(constraints(), target_count=3),
        )
    assert caught.value.reason == "FOLLOWER_LIMIT_MISMATCH"
    with pytest.raises(ContractError) as caught:
        validate(
            record,
            profile(),
            replace(constraints(), max_terminal_tail_duration_ns=PLAN_NS - 1),
        )
    assert caught.value.reason == "FOLLOWER_HORIZON_EXCEEDED"
    with pytest.raises(ContractError) as caught:
        validate(
            record,
            profile(),
            replace(constraints(), duration_grid_ns=2_000_000),
        )
    assert caught.value.reason == "FOLLOWER_GRID_MISMATCH"


def test_record_module_has_no_science_or_ros_dependency(tmp_path):
    script = """
import importlib
import sys
import robot_contracts.trajectory as module
forbidden = ("ruckig", "numpy", "scipy", "mujoco", "rclpy", "robot_api_msgs")
loaded = [name for name in forbidden if name in sys.modules]
assert not loaded, loaded
assert module.FollowerReference.model_fields
"""
    environment = os.environ.copy()
    environment["PYTHONPATH"] = str(ROOT / "shared/common/python/contracts/src")
    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=str(tmp_path),
        env=environment,
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr


def test_document_is_a_fresh_complete_copy():
    record = reference()
    first = record.to_document()
    second = record.to_document()
    assert first == second
    first["plan"]["segments"][0]["c0"][0] = 99.0
    assert second["plan"]["segments"][0]["c0"][0] == 0.0
    assert len(first["window"]["source_targets"]) == 2
    assert first["window"]["selected_indices"] == [0, 1]


@pytest.mark.parametrize("deadline_ns", [100_000_000, 200_000_000])
def test_admission_deadline_may_precede_execution_start(deadline_ns):
    record = reference()
    original = record.request
    source = replace(
        original,
        trajectory=replace(original.trajectory, accept_before_ns=deadline_ns),
    )
    corrected = replace(record, request=source, reference_digest=None)
    assert corrected.accepted_sim_time_ns <= deadline_ns < corrected.start_sim_time_ns
    assert validate(corrected, profile(), constraints()).plan == corrected.plan
