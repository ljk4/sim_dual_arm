"""Generic simulation-task plugin surface (no ROS imports at import time)."""

from robot_backend_sim.tasks.base import (
    SimTaskPlugin,
    TaskChunk,
    TaskChunkView,
    TaskPlan,
    TaskPrepareResult,
    TaskPrepareSnapshot,
)
from robot_backend_sim.tasks.registry import SUPPORTED_TASK_TYPES, create_task_plugin

__all__ = [
    "SUPPORTED_TASK_TYPES",
    "SimTaskPlugin",
    "TaskChunk",
    "TaskChunkView",
    "TaskPlan",
    "TaskPrepareResult",
    "TaskPrepareSnapshot",
    "create_task_plugin",
]
