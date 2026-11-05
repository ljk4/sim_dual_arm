"""ROS message adaptation for stack-task facts and teacher chunks.

Only wire conversion lives here.  Physics, planning and evaluation stay in
their own modules; this file never reads a scene or a clock.
"""

from __future__ import annotations

from typing import Any

from robot_api_msgs.msg import StackTaskState as StackTaskStateMessage
from robot_ros_support.conversions import context_message, stamp

from .stack_task import StackTaskState
from .task_config import PHASES
from .teacher import TeacherChunk

__all__ = [
    "PHASE_CODES",
    "fill_teacher_chunk_response",
    "phase_code",
    "stack_state_message",
]

PHASE_CODES = {phase: index for index, phase in enumerate(PHASES)}


def phase_code(phase: str) -> int:
    try:
        return PHASE_CODES[phase]
    except KeyError as error:
        raise ValueError(f"unknown teacher phase: {phase!r}") from error


def stack_state_message(state: StackTaskState) -> StackTaskStateMessage:
    """Copy one immutable domain state into the fixed-length diagnostic topic."""
    if state.context is None:
        raise ValueError("stack task state has no context")
    source = state.source
    target = state.target
    bitmask = 0
    for index, condition in enumerate(state.conditions):
        if condition.passed:
            bitmask |= 1 << index
    return StackTaskStateMessage(
        context=context_message(state.context),
        state_seq=state.state_seq,
        sample_time=stamp(state.sample_sim_time_ns),
        task_epoch=state.task_epoch,
        state=state.state,
        reason=state.reason,
        task_config_digest=state.task_config_digest,
        plan_digest=state.plan_digest,
        task_start_time=stamp(state.task_start_sim_time_ns),
        first_satisfied_time=stamp(state.first_satisfied_sim_time_ns),
        completed_time=stamp(state.completed_sim_time_ns),
        stable_elapsed_s=state.stable_elapsed_s,
        source_center=[0.0, 0.0, 0.0] if source is None else list(source.center),
        source_quaternion_wxyz=(
            [1.0, 0.0, 0.0, 0.0] if source is None else list(source.quaternion_wxyz)
        ),
        source_linear_velocity=(
            [0.0, 0.0, 0.0] if source is None else list(source.linear_velocity)
        ),
        source_angular_velocity=(
            [0.0, 0.0, 0.0] if source is None else list(source.angular_velocity)
        ),
        target_center=[0.0, 0.0, 0.0] if target is None else list(target.center),
        target_quaternion_wxyz=(
            [1.0, 0.0, 0.0, 0.0] if target is None else list(target.quaternion_wxyz)
        ),
        target_linear_velocity=(
            [0.0, 0.0, 0.0] if target is None else list(target.linear_velocity)
        ),
        target_angular_velocity=(
            [0.0, 0.0, 0.0] if target is None else list(target.angular_velocity)
        ),
        source_target_normal_force=state.source_target_normal_force_n,
        target_table_normal_force=state.target_table_normal_force_n,
        source_table_normal_force=state.source_table_normal_force_n,
        finger_contact_count=state.finger_contact_count,
        right_gripper_opening=state.right_gripper_opening_m,
        right_tcp_distance=state.right_tcp_distance_m,
        conditions_passed=state.conditions_passed,
        conditions_bitmask=bitmask,
    )


def fill_teacher_chunk_response(response: Any, chunk: TeacherChunk) -> Any:
    """Fill the 60x16 response without changing the immutable chunk."""
    flat = [value for position in chunk.positions for value in position]
    if len(flat) != 960:
        raise ValueError("teacher chunk is not 60x16")
    response.task_epoch = chunk.task_epoch
    response.capture_seq = chunk.capture_seq
    response.plan_digest = chunk.plan_digest
    response.chunk_digest = chunk.chunk_digest
    response.observation_time = stamp(chunk.observation_time_ns)
    response.source_time = stamp(chunk.source_time_ns)
    response.robot_profile_hash = chunk.robot_profile_hash
    response.phase = chunk.phase
    response.point_count = len(chunk.positions)
    response.joint_count = len(chunk.positions[0])
    response.positions = flat
    response.time_from_observation_ns = list(chunk.time_from_observation_ns)
    response.phase_codes = [phase_code(phase) for phase in chunk.phases]
    response.ik_max_position_error_m = chunk.ik_max_position_error_m
    response.ik_max_orientation_error_rad = chunk.ik_max_orientation_error_rad
    response.gate_state = chunk.gate_state
    response.gate_failure = chunk.gate_failure
    response.phase_start_time = stamp(chunk.phase_start_sim_time_ns)
    response.phase_wait_s = chunk.phase_wait_s
    return response
