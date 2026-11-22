"""Small typed task sidecar derived from the frozen generic task fact."""

from __future__ import annotations

from dual_arm_runtime.simulation.task_result import (
    TaskFact,
    task_fact_document,
    task_result_for_runtime,
    task_result_from_fact,
)

__all__ = [
    "TaskFact",
    "task_fact_document",
    "task_result_for_runtime",
    "task_result_from_fact",
]
