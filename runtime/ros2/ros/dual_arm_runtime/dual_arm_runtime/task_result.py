"""Small typed task sidecar derived from the frozen generic task fact."""

from __future__ import annotations

import json
from typing import Any

from robot_contracts.episodes import TaskOutcome, TaskResult

TASK_FACT_SCHEMA_VERSION = 1


def task_result_from_fact(fact: object | None) -> TaskResult:
    """Map a frozen TaskState message; absent truth stays not_evaluated."""
    if fact is None:
        return TaskResult()
    phase = str(getattr(fact, "phase", ""))
    reason = str(getattr(fact, "failure_reason", ""))
    if phase == "SUCCEEDED":
        return TaskResult(TaskOutcome.SUCCEEDED, "TASK_SUCCEEDED")
    if phase == "FAILED":
        if reason.startswith("TIMEOUT") or reason.startswith("WALL_TIMEOUT"):
            return TaskResult(TaskOutcome.TIMED_OUT, "TASK_TIMEOUT")
        return TaskResult(TaskOutcome.FAILED, f"TASK_FAILED: {reason}" if reason else "TASK_FAILED")
    return TaskResult()


def task_result_for_runtime(runtime_reason: str, fact: object | None) -> TaskResult:
    """Task dimensions come only from a real frozen judge fact."""
    del runtime_reason
    return task_result_from_fact(fact)


def _metrics(fact: object) -> dict[str, Any]:
    try:
        value = json.loads(str(getattr(fact, "metrics_json", "") or "{}"))
    except (TypeError, ValueError):
        return {}
    return value if isinstance(value, dict) else {}


def task_fact_document(
    fact: object | None,
    result: TaskResult,
    *,
    action_source: str,
    test_only: bool,
) -> dict[str, Any]:
    """Return the bounded JSON sidecar written next to the MCAP manifest."""
    if fact is None:
        return {
            "schema_version": TASK_FACT_SCHEMA_VERSION,
            "evaluated": False,
            "action_source": action_source,
            "test_only": bool(test_only),
            "result": {"outcome": result.outcome.value, "reason": result.reason},
        }
    metrics = _metrics(fact)
    phase = str(getattr(fact, "phase", ""))
    failure_reason = str(getattr(fact, "failure_reason", ""))
    return {
        "schema_version": TASK_FACT_SCHEMA_VERSION,
        "evaluated": True,
        "action_source": action_source,
        "test_only": bool(test_only),
        "result": {"outcome": result.outcome.value, "reason": result.reason},
        "task": {
            "state": phase,
            "phase": phase,
            "task_type": str(getattr(fact, "task_type", "")),
            "reason": failure_reason,
            "failure_reason": failure_reason,
            "task_epoch": int(getattr(fact, "task_epoch", 0)),
            "state_seq": int(getattr(fact, "state_seq", 0)),
            "sample_sim_time_ns": _stamp_ns(fact, "sample_time"),
            "task_start_sim_time_ns": int(metrics.get("task_start_sim_time_ns", 0)),
            "first_satisfied_sim_time_ns": int(
                metrics.get("first_satisfied_sim_time_ns", 0)
            ),
            "completed_sim_time_ns": int(metrics.get("completed_sim_time_ns", 0)),
            "stable_elapsed_s": float(metrics.get("stable_elapsed_s", 0.0)),
            "task_config_digest": str(getattr(fact, "task_config_digest", "")),
            "plan_digest": str(getattr(fact, "plan_digest", "")),
            "conditions_passed": bool(metrics.get("conditions_passed", False)),
            "conditions_bitmask": int(metrics.get("conditions_bitmask", 0)),
            "source_target_normal_force_n": float(
                metrics.get("source_target_normal_force_n", 0.0)
            ),
            "target_table_normal_force_n": float(
                metrics.get("target_table_normal_force_n", 0.0)
            ),
            "finger_contact_count": int(metrics.get("finger_contact_count", 0)),
            "right_gripper_opening_m": float(metrics.get("right_gripper_opening_m", 0.0)),
            "right_tcp_distance_m": float(metrics.get("right_tcp_distance_m", 0.0)),
            "metrics": metrics,
        },
    }


def _stamp_ns(message: object, name: str) -> int:
    value = getattr(message, name, None)
    if value is None:
        return 0
    seconds = int(getattr(value, "sec", 0))
    nanoseconds = int(getattr(value, "nanosec", 0))
    return seconds * 1_000_000_000 + nanoseconds


__all__ = [
    "TASK_FACT_SCHEMA_VERSION",
    "task_fact_document",
    "task_result_for_runtime",
    "task_result_from_fact",
]
