from dataclasses import replace

import pytest
from robot_contracts.action_chunks import ChunkAnchor
from robot_contracts.models import (
    ContractError,
    JointSpec,
    RobotProfile,
    RunContext,
    Target,
    Trajectory,
)
from robot_contracts.native import (
    NATIVE_MOTION,
    NATIVE_STREAM_FIRST,
    AcceptedReference,
    NativeRequest,
    compute_reference_digest,
    expected_final_velocities,
    sample_accepted_reference,
    validate_accepted_reference,
)
from robot_contracts.reference import CubicSegment, ReferencePlan, ReferenceSample

PROFILE_DIGEST = "profile-digest"
JOINTS = (JointSpec("j1", "rad", -5.0, 5.0, 10.0, 0.0),)
MAX_ACCELERATIONS = (20.0,)


def profile() -> RobotProfile:
    return RobotProfile("panda-stack", "panda", PROFILE_DIGEST, True, JOINTS, ())


def context() -> RunContext:
    return RunContext(
        "panda-stack",
        PROFILE_DIGEST,
        "backend-instance",
        "mujoco-sim",
        0,
        episode_id="episode-1",
        control_epoch=1,
    )


def motion_request() -> NativeRequest:
    return NativeRequest(
        trajectory=Trajectory(
            context(),
            "lease-1",
            "command-1",
            1,
            1,
            5_000_000_000,
            ("j1",),
            (Target(1_000_000_000, (1.0,)),),
        ),
        kind=NATIVE_MOTION,
    )


def motion_plan() -> ReferencePlan:
    return ReferencePlan(
        ("j1",),
        (CubicSegment(1_000_000_000, (0.0,), (0.0,), (3.0,), (-2.0,)),),
    )


def motion_record(**changes) -> AcceptedReference:
    values = {
        "request": motion_request(),
        "motion_seq": 1,
        "accepted_sim_time_ns": 200_000_000,
        "joint_names": ("j1",),
        "plan": motion_plan(),
        "initial_positions": (0.0,),
        "initial_velocities": (0.0,),
        "final_velocities": (0.0,),
        "max_accelerations": MAX_ACCELERATIONS,
        "selected_indices": (),
    }
    values.update(changes)
    return AcceptedReference(**values)


def stream_request(**changes) -> NativeRequest:
    values = {
        "trajectory": Trajectory(
            context(),
            "lease-1",
            "command-1",
            1,
            7,
            5_000_000_000,
            ("j1",),
            (
                Target(100_000_000, (-0.5,)),
                Target(600_000_000, (0.1,)),
                Target(1_000_000_000, (0.5,)),
            ),
        ),
        "kind": NATIVE_STREAM_FIRST,
        "anchor": ChunkAnchor(
            "stream-1",
            0,
            1_000_000_000,
            ReferenceSample((0.0,), (1.0,), (0.0,)),
            PROFILE_DIGEST,
        ),
        "observation_time_ns": 500_000_000,
        "target_count": 2,
        "settle_at_end": False,
        "final_velocities": None,
    }
    values.update(changes)
    return NativeRequest(**values)


def stream_plan() -> ReferencePlan:
    return ReferencePlan(
        ("j1",),
        (
            CubicSegment(100_000_000, (0.0,), (1.0,), (0.0,), (0.0,)),
            CubicSegment(400_000_000, (0.1,), (1.0,), (0.0,), (0.0,)),
        ),
    )


def stream_record(request=None, **changes) -> AcceptedReference:
    request = stream_request() if request is None else request
    values = {
        "request": request,
        "motion_seq": 1,
        "accepted_sim_time_ns": 0,
        "joint_names": ("j1",),
        "plan": stream_plan(),
        "initial_positions": (0.0,),
        "initial_velocities": request.anchor.reference.velocities,
        "final_velocities": expected_final_velocities(request),
        "max_accelerations": MAX_ACCELERATIONS,
        "selected_indices": (1, 2),
    }
    values.update(changes)
    return AcceptedReference(**values)


def test_motion_record_validates_and_rebuilds_from_actual_start():
    record = motion_record()
    validate_accepted_reference(record, profile(), MAX_ACCELERATIONS)
    assert compute_reference_digest(record) == record.reference_digest
    assert sample_accepted_reference(
        record, 700_000_000, actual_started_sim_ns=200_000_000
    ).positions == pytest.approx((0.5,))
    with pytest.raises(ContractError) as caught:
        sample_accepted_reference(record, 200_000_000)
    assert caught.value.reason == "INVALID_REFERENCE_RECORD"
    held = sample_accepted_reference(
        record, 1_300_000_001, actual_started_sim_ns=200_000_000
    )
    assert held.positions == pytest.approx((1.0,))
    assert held.velocities == (0.0,)
    assert held.accelerations == (0.0,)
    with pytest.raises(ContractError) as caught:
        sample_accepted_reference(
            record, 199_999_999, actual_started_sim_ns=200_000_000
        )
    assert caught.value.reason == "REFERENCE_EXPIRED"
    with pytest.raises(ContractError) as caught:
        sample_accepted_reference(record, 1_300_000_001)
    assert caught.value.reason == "INVALID_REFERENCE_RECORD"


def test_stream_record_recomputes_selection_and_anchor_continuity():
    record = stream_record()
    validate_accepted_reference(record, profile(), MAX_ACCELERATIONS)
    assert record.selected_indices == (1, 2)
    assert record.final_velocities == (1.0,)
    assert sample_accepted_reference(record, 1_200_000_000).positions == pytest.approx(
        (0.2,)
    )
    with pytest.raises(ContractError) as caught:
        sample_accepted_reference(record, 1_600_000_000)
    assert caught.value.reason == "REFERENCE_EXPIRED"
    with pytest.raises(ContractError) as caught:
        sample_accepted_reference(record, 1_000_000_000, actual_started_sim_ns=1)
    assert caught.value.reason == "INVALID_REFERENCE_RECORD"


def test_final_stream_holds_only_when_settle_is_declared():
    request = stream_request(settle_at_end=True, final_velocities=None)
    final_plan = ReferencePlan(
        ("j1",),
        (CubicSegment(500_000_000, (0.0,), (1.0,), (2.0,), (-4.0,)),),
    )
    record = stream_record(
        request=request, plan=final_plan, final_velocities=(0.0,)
    )
    sample = sample_accepted_reference(record, 2_000_000_000)
    assert sample.positions == pytest.approx((0.5,))
    assert sample.velocities == (0.0,)
    assert sample.accelerations == (0.0,)
    nonzero = stream_record(
        request=request, plan=final_plan, final_velocities=(1.0,)
    )
    with pytest.raises(ContractError) as caught:
        sample_accepted_reference(nonzero, 2_000_000_000)
    assert caught.value.reason == "REFERENCE_EXPIRED"


def test_digest_covers_request_plan_time_identity_and_mode():
    first = motion_record()
    same = motion_record()
    assert first.reference_digest == same.reference_digest
    assert len(first.reference_digest) == 64
    for changes in (
        {"motion_seq": 2},
        {"accepted_sim_time_ns": 201_000_000},
    ):
        with pytest.raises(ContractError) as caught:
            replace(first, **changes)
        assert caught.value.reason == "REFERENCE_DIGEST_MISMATCH"
    changed = motion_record()
    object.__setattr__(changed, "motion_seq", 2)
    with pytest.raises(ContractError) as caught:
        validate_accepted_reference(changed, profile(), MAX_ACCELERATIONS)
    assert caught.value.reason == "REFERENCE_DIGEST_MISMATCH"

    different_request = NativeRequest(
        trajectory=replace(motion_request().trajectory, command_id="command-2"),
        kind=NATIVE_MOTION,
    )
    assert motion_record(request=different_request).reference_digest != first.reference_digest


def test_selection_final_mode_and_coefficient_tampering_are_rejected():
    selected_tamper = replace(
        stream_record(), selected_indices=(0, 1), reference_digest=None
    )
    with pytest.raises(ContractError) as caught:
        validate_accepted_reference(selected_tamper, profile(), MAX_ACCELERATIONS)
    assert caught.value.reason == "REFERENCE_SELECTION_MISMATCH"

    final_tamper = replace(
        stream_record(), final_velocities=(5.0,), reference_digest=None
    )
    with pytest.raises(ContractError) as caught:
        validate_accepted_reference(final_tamper, profile(), MAX_ACCELERATIONS)
    assert caught.value.reason == "INVALID_REFERENCE_RECORD"

    settle_request = stream_request(settle_at_end=True, final_velocities=None)
    settle_tamper = stream_record(
        request=settle_request, final_velocities=(5.0,), reference_digest=None
    )
    with pytest.raises(ContractError) as caught:
        validate_accepted_reference(settle_tamper, profile(), MAX_ACCELERATIONS)
    assert caught.value.reason == "INVALID_REFERENCE_RECORD"

    broken_plan = ReferencePlan(
        ("j1",),
        (
            CubicSegment(100_000_000, (0.1,), (1.0,), (0.0,), (0.0,)),
            CubicSegment(400_000_000, (0.1,), (1.0,), (0.0,), (0.0,)),
        ),
    )
    plan_tamper = stream_record(plan=broken_plan, reference_digest=None)
    with pytest.raises(ContractError) as caught:
        validate_accepted_reference(plan_tamper, profile(), MAX_ACCELERATIONS)
    assert caught.value.reason == "INVALID_REFERENCE_PLAN"

    motion_selection = motion_record(selected_indices=(0,), reference_digest=None)
    with pytest.raises(ContractError) as caught:
        validate_accepted_reference(motion_selection, profile(), MAX_ACCELERATIONS)
    assert caught.value.reason == "INVALID_REFERENCE_RECORD"


def test_complete_bad_prefix_is_validated_even_when_not_executed():
    bad_prefix = stream_request(
        trajectory=Trajectory(
            context(),
            "lease-1",
            "command-1",
            1,
            7,
            5_000_000_000,
            ("j1",),
            (
                Target(100_000_000, (99.0,)),
                Target(600_000_000, (0.1,)),
                Target(1_000_000_000, (0.5,)),
            ),
        )
    )
    record = stream_record(request=bad_prefix, reference_digest=None)
    assert record.selected_indices == (1, 2)
    with pytest.raises(ContractError) as caught:
        validate_accepted_reference(record, profile(), MAX_ACCELERATIONS)
    assert caught.value.reason == "JOINT_LIMIT"


def test_mutated_digest_is_rejected_even_with_structurally_valid_fields():
    record = motion_record()
    object.__setattr__(record, "accepted_sim_time_ns", 123)
    with pytest.raises(ContractError) as caught:
        validate_accepted_reference(record, profile(), MAX_ACCELERATIONS)
    assert caught.value.reason == "REFERENCE_DIGEST_MISMATCH"
