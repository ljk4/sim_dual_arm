import math

import pytest
from pydantic import ValidationError
from robot_contracts.action_chunks import ChunkAnchor
from robot_contracts.execution_feedback import (
    ControlEvent,
    ExecutionFeedback,
    FingerFeedback,
    GripperFeedback,
    StreamChunkStatus,
)
from robot_contracts.models import ContractError, RunContext, Target
from robot_contracts.stream_request import (
    REQUEST_STREAM_FIRST,
    REQUEST_STREAM_NEXT,
    StreamRequest,
    Trajectory,
)
from robot_contracts.trajectory.spline import ReferenceSample

PROFILE_DIGEST = "profile-digest"


def context(**changes):
    return RunContext(
        "marvin-stack",
        changes.pop("robot_profile_hash", PROFILE_DIGEST),
        "backend-instance",
        "mujoco-sim",
        changes.pop("clock_epoch", 0),
        episode_id=changes.pop("episode_id", "episode-1"),
        control_epoch=changes.pop("control_epoch", 1),
        **changes,
    )


def trajectory(**changes):
    values = {
        "context": context(),
        "lease_id": "lease-1",
        "command_id": "command-1",
        "command_seq": 1,
        "observation_id": 1,
        "accept_before_ns": 5_000_000_000,
        "joint_names": ("j1", "j2"),
        "targets": (Target(1_000_000_000, (0.2, -0.2)),),
    }
    values.update(changes)
    return Trajectory(**values)


def stream_anchor(predecessor_seq=0, *, start=1_000_000_000):
    return ChunkAnchor(
        "stream-1",
        predecessor_seq,
        start,
        ReferenceSample((0.0, 0.0), (0.0, 0.0), (0.0, 0.0)),
        PROFILE_DIGEST,
    )


def stream_request(*, kind=REQUEST_STREAM_FIRST, **changes):
    values = {
        "trajectory": trajectory(
            targets=(
                Target(200_000_000, (0.1, -0.1)),
                Target(600_000_000, (0.2, -0.2)),
                Target(800_000_000, (0.3, -0.3)),
            )
        ),
        "kind": kind,
        "anchor": stream_anchor(0 if kind == REQUEST_STREAM_FIRST else 1),
        "observation_time_ns": 500_000_000,
        "target_count": 2,
        "settle_at_end": False,
        "final_velocities": (0.1, -0.1),
    }
    values.update(changes)
    return StreamRequest(**values)


def empty_gripper(side: str, finger_indices=(1, 2)) -> GripperFeedback:
    fingers = tuple(
        FingerFeedback(side, index, f"{side}_finger{index}", 0.0, (), (), ())
        for index in finger_indices
    )
    return GripperFeedback(
        side=side,
        mode="NONE",
        position_ok=False,
        contact_ok=False,
        contact_object=None,
        ambiguous=False,
        target_position_m=0.0,
        initial_reference_m=0.0,
        final_reference_m=0.0,
        measured_position_m=0.0,
        measured_velocity_m_s=0.0,
        target_error_m=0.0,
        fingers=fingers,
    )


def execution(**changes):
    values = {
        "context": context(),
        "command_id": "command-1",
        "command_seq": 1,
        "observation_id": 1,
        "phase": "STARTED",
        "reason": "",
        "stop_reason": "",
        "motion_seq": 1,
        "stream_id": "",
        "chunk_seq": 0,
        "accepted_backend_time_ns": 0,
        "accepted_wall_time_ns": 10,
        "has_started": True,
        "started_backend_time_ns": 20,
        "started_wall_time_ns": 20,
        "has_finished": False,
        "finished_backend_time_ns": None,
        "finished_wall_time_ns": None,
        "motion_phase": "RUNNING",
        "reference_application_backend_time_ns": 30,
        "measurement_backend_time_ns": 32,
        "reference_elapsed_ns": 10,
        "stable_for_ns": 0,
        "joint_names": ("j1", "j2"),
        "reference_positions": (0.2, -0.2),
        "reference_velocities": (0.1, -0.1),
        "reference_accelerations": (0.0, 0.0),
        "measured_positions": (0.1, -0.2),
        "measured_velocities": (0.05, -0.1),
        "position_errors": (0.1, 0.0),
        "velocity_errors": (0.05, 0.0),
        "reference_kind": "TRAJECTORY",
        "tracking_quality": "FOLLOWING",
        "tracking_violation_ns": 0,
        "grippers": (empty_gripper("left"), empty_gripper("right")),
    }
    values.update(changes)
    return ExecutionFeedback(**values)


def test_request_is_deeply_normalised_and_frozen():
    request = stream_request()
    assert request.trajectory.joint_names == ("j1", "j2")
    assert request.trajectory.targets[0].positions == (0.1, -0.1)
    assert request.mode == "STREAM_FIRST"
    assert request.command.command_id == "command-1"
    with pytest.raises(ValidationError):
        request.kind = 2  # type: ignore[misc]
    with pytest.raises((ValidationError, TypeError)):
        request.trajectory.targets[0].positions[0] = 1.0  # type: ignore[index]


def test_mode_defaults_and_anchor_semantics_are_explicit():
    with pytest.raises(ContractError) as caught:
        stream_request(kind=1)
    assert caught.value.reason == "INVALID_STREAM_REQUEST"
    with pytest.raises(ContractError):
        stream_request(anchor=None)

    first = stream_request()
    assert first.window is not None
    assert first.window.selected_indices == (1, 2)
    with pytest.raises(ContractError):
        stream_request(kind=REQUEST_STREAM_FIRST, anchor=stream_anchor(1))
    with pytest.raises(ContractError):
        stream_request(kind=REQUEST_STREAM_NEXT, anchor=stream_anchor(0))
    with pytest.raises(ContractError):
        stream_request(anchor=stream_anchor(start=100_000_000))
    with pytest.raises(ContractError):
        stream_request(target_count=1)
    with pytest.raises(ContractError):
        stream_request(target_count=4)
    with pytest.raises(ContractError):
        stream_request(settle_at_end=True, final_velocities=(0.1, 0.0))


def test_integer_bool_nan_and_size_bounds_are_rejected():
    with pytest.raises(ContractError):
        stream_request(kind=True)
    with pytest.raises(ContractError):
        stream_request(trajectory=trajectory(command_seq=True))
    with pytest.raises(ContractError):
        stream_request(
            trajectory=trajectory(targets=(Target(True, (0.0, 0.0)),))
        )
    with pytest.raises(ContractError):
        stream_request(
            trajectory=trajectory(targets=(Target(1_000_000_000, (math.nan, 0.0)),))
        )
    with pytest.raises(ContractError):
        stream_request(
            trajectory=trajectory(targets=(Target(1_000_000_000, (True, 0.0)),))
        )
    with pytest.raises(ContractError):
        stream_request(
            trajectory=trajectory(joint_names=tuple(f"j{index}" for index in range(65)))
        )
    too_many = tuple(
        Target((index + 1) * 1_000_000, (0.0, 0.0)) for index in range(129)
    )
    with pytest.raises(ContractError):
        stream_request(trajectory=trajectory(targets=too_many))
    with pytest.raises(ContractError):
        stream_request(
            trajectory=trajectory(
                targets=(
                    Target(2_000_000_000, (0.0, 0.0)),
                    Target(1_500_000_000, (0.1, 0.0)),
                )
            )
        )
    with pytest.raises(ContractError):
        stream_request(
            trajectory=trajectory(targets=(Target(3_000_000_001, (0.0, 0.0)),))
        )


def test_execution_flags_errors_and_gripper_evidence_are_validated():
    assert execution().has_started is True
    with pytest.raises(ContractError):
        execution(has_started=False)
    with pytest.raises(ContractError):
        execution(has_finished=True)
    with pytest.raises(ContractError):
        execution(has_finished=True, finished_backend_time_ns=40, finished_wall_time_ns=40)
    with pytest.raises(ContractError):
        execution(position_errors=(0.2, 0.0))
    with pytest.raises(ContractError):
        execution(velocity_errors=(0.2, 0.0))
    with pytest.raises(ContractError):
        execution(reference_positions=(math.inf, 0.0))
    with pytest.raises(ContractError):
        execution(chunk_seq=-1)
    with pytest.raises(ContractError):
        execution(motion_phase="COMPLETED")
    with pytest.raises(ContractError):
        execution(reference_kind="PLANNED_BRAKE")

    with pytest.raises(ContractError):
        FingerFeedback(
            "left",
            1,
            "left_finger1",
            0.0,
            ("source", "source"),
            (1.0, 1.0),
            (1, 1),
        )
    duplicate = FingerFeedback(
        "left",
        1,
        "left_finger1",
        0.0,
        ("source",),
        (1.0,),
        (1,),
    )
    with pytest.raises(ContractError):
        GripperFeedback(
            side="left",
            mode="NONE",
            position_ok=False,
            contact_ok=False,
            contact_object=None,
            ambiguous=False,
            target_position_m=0.0,
            initial_reference_m=0.0,
            final_reference_m=0.0,
            measured_position_m=0.0,
            measured_velocity_m_s=0.0,
            target_error_m=0.0,
            fingers=(duplicate, duplicate),
        )
    with pytest.raises(ContractError):
        FingerFeedback("left", 1, "body", -0.1, (), (), ())
    with pytest.raises(ContractError):
        FingerFeedback("left", 1, "body", 0.0, ("source",), (math.nan,), (1,))
    with pytest.raises(ContractError):
        FingerFeedback("left", 1, "body", 0.0, ("source",), (1.0,), (1, 2))


def test_gripper_set_is_optional_bounded_and_named():
    assert execution(grippers=()).grippers == ()
    assert execution(grippers=(empty_gripper("tool"),)).grippers[0].side == "tool"
    accepted = execution(grippers=(empty_gripper("z"), empty_gripper("a")))
    assert tuple(gripper.side for gripper in accepted.grippers) == ("z", "a")
    eight = tuple(empty_gripper(f"g{index}") for index in range(8))
    assert len(execution(grippers=eight).grippers) == 8
    with pytest.raises(ContractError):
        execution(grippers=tuple(empty_gripper(f"g{index}") for index in range(9)))
    with pytest.raises(ContractError):
        execution(grippers=(empty_gripper("left"), empty_gripper("left")))


def test_gripper_fingers_are_bounded_unique_and_not_fixed():
    assert tuple(
        finger.finger_index for finger in empty_gripper("tool", (1,)).fingers
    ) == (1,)
    assert tuple(
        finger.finger_index for finger in empty_gripper("tool", (2, 1, 5)).fingers
    ) == (2, 1, 5)
    assert len(empty_gripper("tool", tuple(range(1, 9))).fingers) == 8
    with pytest.raises(ContractError):
        empty_gripper("tool", ())
    with pytest.raises(ContractError):
        empty_gripper("tool", (1, 1))
    with pytest.raises(ContractError):
        empty_gripper("tool", tuple(range(1, 10)))
    with pytest.raises(ContractError):
        FingerFeedback("tool", 0, "body", 0.0, (), (), ())
    with pytest.raises(ContractError):
        FingerFeedback("tool", 9, "body", 0.0, (), (), ())
    with pytest.raises(ContractError):
        GripperFeedback(
            side="tool",
            mode="NONE",
            position_ok=False,
            contact_ok=False,
            contact_object=None,
            ambiguous=False,
            target_position_m=0.0,
            initial_reference_m=0.0,
            final_reference_m=0.0,
            measured_position_m=0.0,
            measured_velocity_m_s=0.0,
            target_error_m=0.0,
            fingers=(FingerFeedback("other", 1, "body", 0.0, (), (), ()),),
        )


def test_chunk_and_event_keep_real_phases_and_identity():
    chunk = StreamChunkStatus(
        context=context(),
        command_id="command-1",
        command_seq=1,
        chunk_seq=1,
        observation_id=1,
        phase="RETIRED",
        reason="FINAL_HOLD",
        accepted_backend_time_ns=0,
        accepted_wall_time_ns=10,
        has_started=True,
        started_backend_time_ns=20,
        started_wall_time_ns=20,
        has_finished=True,
        finished_backend_time_ns=100,
        finished_wall_time_ns=100,
    )
    assert chunk.phase == "RETIRED"
    with pytest.raises(ContractError):
        StreamChunkStatus(
            context=context(),
            command_id="command-1",
            command_seq=1,
            chunk_seq=1,
            observation_id=1,
            phase="COMPLETED",
            reason="",
            accepted_backend_time_ns=0,
            accepted_wall_time_ns=10,
            has_started=False,
            started_backend_time_ns=None,
            started_wall_time_ns=None,
            has_finished=False,
            finished_backend_time_ns=None,
            finished_wall_time_ns=None,
        )
    event = ControlEvent(
        context=context(),
        event_seq=1,
        command_id="command-1",
        command_seq=1,
        observation_id=2,
        is_chunk=True,
        chunk_seq=2,
        phase="STARTED",
        reason="HANDOFF",
        backend_time_ns=1_100_000_000,
    )
    assert event.is_chunk and event.chunk_seq == 2
    with pytest.raises(ContractError):
        ControlEvent(
            context=context(),
            event_seq=1,
            command_id="command-1",
            command_seq=1,
            observation_id=2,
            is_chunk=False,
            chunk_seq=2,
            phase="STARTED",
            reason="",
            backend_time_ns=0,
        )
