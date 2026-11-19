"""Task-plugin registry; concrete plugins are discovered dynamically."""

from __future__ import annotations

from robot_backend_sim.common.plugins import get_task_plugin, supported_task_types
from robot_backend_sim.common.tasks.base import SimTaskPlugin

__all__ = ["create_task_plugin", "SUPPORTED_TASK_TYPES"]

SUPPORTED_TASK_TYPES = supported_task_types()


def create_task_plugin(
    task_type: str, *, task_path: object, task_mode: str | None, scene_config: object
) -> SimTaskPlugin:
    """Create one plugin without hardcoding any robot or task module in common/."""
    entry = get_task_plugin(task_type)
    return entry.create(task_path, task_mode=task_mode, scene_config=scene_config)
