"""Generic TaskState and TeacherChunk ROS conversions with delayed ROS imports."""

from __future__ import annotations

from typing import Any

from robot_contracts.task_state import TaskState

from robot_backend_sim.common.tasks.base import TaskChunk

__all__ = ["fill_teacher_chunk_response", "task_state_message"]


def fill_teacher_chunk_response(response: Any, chunk: TaskChunk) -> Any:
    """Fill one rectangular Teacher chunk response without mutating the immutable chunk."""
    from robot_ros_support.conversions import stamp

    if not chunk.positions or not chunk.positions[0]:
        raise ValueError("teacher chunk must contain at least one trajectory point")
    point_count = len(chunk.positions)
    joint_count = len(chunk.positions[0])
    if any(len(position) != joint_count for position in chunk.positions):
        raise ValueError("teacher chunk positions must be rectangular")
    flat = [float(value) for position in chunk.positions for value in position]
    response.task_epoch = chunk.task_epoch
    response.capture_seq = chunk.capture_seq
    response.plan_digest = chunk.plan_digest
    response.chunk_digest = chunk.chunk_digest
    response.observation_time = stamp(chunk.observation_time_ns)
    response.source_time = stamp(chunk.source_time_ns)
    response.robot_profile_hash = chunk.robot_profile_hash
    response.phase = chunk.phase
    response.point_count = point_count
    response.joint_count = joint_count
    response.positions = flat
    response.time_from_observation_ns = list(chunk.time_from_observation_ns)
    response.phase_codes = list(chunk.phase_codes)
    response.ik_max_position_error_m = chunk.ik_max_position_error_m
    response.ik_max_orientation_error_rad = chunk.ik_max_orientation_error_rad
    response.gate_state = chunk.gate_state
    response.gate_failure = chunk.gate_failure
    response.phase_start_time = stamp(chunk.phase_start_sim_time_ns)
    response.phase_wait_s = chunk.phase_wait_s
    return response


def task_state_message(state: TaskState) -> Any:
    """Copy one immutable generic state into the diagnostic ROS topic."""
    from robot_ros_support.conversions import context_message, stamp
    from sim_api_msgs.msg import TaskState as TaskStateMessage

    if state.context is None:
        raise ValueError("task state has no context")
    return TaskStateMessage(
        context=context_message(state.context),
        state_seq=state.state_seq,
        sample_time=stamp(state.sample_time_ns),
        task_epoch=state.task_epoch,
        task_type=state.task_type,
        phase=state.phase,
        succeeded=state.succeeded,
        terminal=state.terminal,
        failure_reason=state.failure_reason,
        plan_digest=state.plan_digest,
        task_config_digest=state.task_config_digest,
        metrics_json=state.metrics_json(),
    )
