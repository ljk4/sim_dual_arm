"""Task-plugin registry; concrete plugins are imported lazily."""

from __future__ import annotations

from robot_backend_sim.tasks.base import SimTaskPlugin

__all__ = ["create_task_plugin", "SUPPORTED_TASK_TYPES"]

SUPPORTED_TASK_TYPES = ("stack", "handover")


def create_task_plugin(
    task_type: str, *, task_path: object, task_mode: str | None, scene_config: object
) -> SimTaskPlugin:
    """Create one plugin without importing any ROS or task-specific module."""
    if task_type == "stack":
        from robot_backend_sim.tasks.stack import StackTaskPlugin

        return StackTaskPlugin.from_path(
            task_path, task_mode=task_mode, scene_config=scene_config
        )
    if task_type == "handover":
        from robot_backend_sim.tasks.handover.plugin import HandoverTaskPlugin

        return HandoverTaskPlugin.from_path(
            task_path, task_mode=task_mode, scene_config=scene_config
        )
    raise ValueError(f"unknown simulation task type: {task_type!r}")
