"""Generic TaskState ROS conversion with delayed ROS imports."""

from __future__ import annotations

from typing import Any

from robot_contracts.task_state import TaskState

__all__ = ["task_state_message"]


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
