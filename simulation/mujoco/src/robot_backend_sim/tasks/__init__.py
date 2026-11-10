"""Generic simulation-task plugin surface (no ROS imports at import time)."""

from .base import (
    SimTaskPlugin,
    TaskChunk,
    TaskChunkView,
    TaskPlan,
    TaskPrepareResult,
    TaskPrepareSnapshot,
)
from .registry import SUPPORTED_TASK_TYPES, create_task_plugin

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
