"""Real generated-type round trips for the bounded gripper feedback container.

Skipped when ROS is unavailable.  The test uses ``robot_api_msgs`` 0.3.0
generated classes and CDR serialization, not only the Python DTOs.
"""

from __future__ import annotations

import pytest

try:
    from rclpy.serialization import deserialize_message, serialize_message
    from robot_api_msgs.msg import ExecutionFeedback as ExecutionFeedbackMessage
    from robot_contracts.execution_feedback import (
        MAX_GRIPPER_FEEDBACK,
        ExecutionFeedback,
        FingerFeedback,
        GripperFeedback,
    )
    from robot_contracts.models import ContractError, RunContext
    from robot_ros_support.stream_conversions import (
        execution_feedback_from_message,
        execution_feedback_message,
    )
except ImportError as error:  # pragma: no cover - host without ROS
    pytest.skip(f"ROS unavailable: {error}", allow_module_level=True)


def context() -> RunContext:
    return RunContext(
        "panda-stack",
        "profile-digest",
        "backend-instance",
        "mujoco-sim",
        0,
        episode_id="episode-1",
        control_epoch=1,
    )


def finger(side: str, index: int) -> FingerFeedback:
    return FingerFeedback(side, index, f"{side}_finger{index}", 0.0, (), (), ())


def gripper(side: str, indices: tuple[int, ...] = (1, 2)) -> GripperFeedback:
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
        fingers=tuple(finger(side, index) for index in indices),
    )


def execution(grippers: tuple[GripperFeedback, ...]) -> ExecutionFeedback:
    return ExecutionFeedback(
        context=context(),
        command_id="command-1",
        command_seq=1,
        observation_id=1,
        phase="STARTED",
        reason="",
        stop_reason="",
        motion_seq=1,
        stream_id="",
        chunk_seq=0,
        accepted_sim_time_ns=0,
        accepted_wall_time_ns=10,
        has_started=True,
        started_sim_time_ns=20,
        started_wall_time_ns=20,
        has_finished=False,
        finished_sim_time_ns=None,
        finished_wall_time_ns=None,
        motion_phase="RUNNING",
        reference_application_sim_time_ns=30,
        measurement_sim_time_ns=32,
        reference_elapsed_ns=10,
        stable_for_ns=0,
        joint_names=("j1", "j2"),
        reference_positions=(0.2, -0.2),
        reference_velocities=(0.1, -0.1),
        reference_accelerations=(0.0, 0.0),
        measured_positions=(0.1, -0.2),
        measured_velocities=(0.05, -0.1),
        position_errors=(0.1, 0.0),
        velocity_errors=(0.05, 0.0),
        reference_kind="TRAJECTORY",
        tracking_quality="FOLLOWING",
        tracking_violation_ns=0,
        grippers=grippers,
    )


def cdr_round_trip(value: ExecutionFeedback) -> ExecutionFeedback:
    message = execution_feedback_message(value)
    assert isinstance(message, ExecutionFeedbackMessage)
    restored = deserialize_message(serialize_message(message), ExecutionFeedbackMessage)
    return execution_feedback_from_message(restored)


@pytest.mark.parametrize(
    "grippers",
    (
        (),
        (gripper("tool"),),
        (gripper("tool", (1, 3)),),
        (gripper("left"), gripper("right")),
        (gripper("z"), gripper("a"), gripper("gripper-3", (2, 5))),
        tuple(gripper(f"g{index}") for index in range(MAX_GRIPPER_FEEDBACK)),
    ),
    ids=("empty", "single", "single-noncontiguous", "panda-pair", "named-unsorted", "eight"),
)
def test_bounded_gripper_feedback_round_trips_through_generated_types(grippers):
    original = execution(grippers)
    restored = cdr_round_trip(original)
    assert restored == original
    assert tuple(item.side for item in restored.grippers) == tuple(
        item.side for item in grippers
    )


def test_duplicate_side_is_rejected_after_wire_conversion():
    message = execution_feedback_message(execution((gripper("tool"),)))
    message.grippers.append(message.grippers[0])
    with pytest.raises(ContractError):
        execution_feedback_from_message(message)


def test_more_than_the_bound_is_rejected_before_serialization():
    with pytest.raises(ContractError):
        execution(tuple(gripper(f"g{index}") for index in range(MAX_GRIPPER_FEEDBACK + 1)))
