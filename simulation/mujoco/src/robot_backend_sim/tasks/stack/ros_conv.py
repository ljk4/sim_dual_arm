"""ROS adaptation for generic task facts and stack teacher chunks.

Every ``robot_api_msgs``/``robot_ros_support`` import is delayed into the
conversion function body, so importing :mod:`robot_backend_sim.tasks` never
requires a ROS 2 workspace.
"""

from __future__ import annotations

from typing import Any

from robot_backend_sim.tasks.base import TaskChunk
from robot_backend_sim.tasks.stack.config import PHASES

__all__ = ["PHASE_CODES", "fill_teacher_chunk_response", "phase_code"]

PHASE_CODES = {phase: index for index, phase in enumerate(PHASES)}


def phase_code(phase: str) -> int:
    try:
        return PHASE_CODES[phase]
    except KeyError as error:
        raise ValueError(f"unknown teacher phase: {phase!r}") from error


def fill_teacher_chunk_response(response: Any, chunk: TaskChunk) -> Any:
    """Fill the 60x16 response without changing the immutable chunk."""
    from robot_ros_support.conversions import stamp

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
    response.phase_codes = list(chunk.phase_codes)
    response.ik_max_position_error_m = chunk.ik_max_position_error_m
    response.ik_max_orientation_error_rad = chunk.ik_max_orientation_error_rad
    response.gate_state = chunk.gate_state
    response.gate_failure = chunk.gate_failure
    response.phase_start_time = stamp(chunk.phase_start_sim_time_ns)
    response.phase_wait_s = chunk.phase_wait_s
    return response
