"""Small typed task sidecar derived from the frozen physical judge fact."""

from __future__ import annotations

from typing import Any

from robot_contracts.episodes import TaskOutcome, TaskResult

TASK_FACT_SCHEMA_VERSION = 1


def task_result_from_fact(fact: object | None) -> TaskResult:
    """Map a frozen StackTaskState message; absent truth stays not_evaluated."""
    if fact is None:
        return TaskResult()
    name = str(getattr(fact, "state", ""))
    reason = str(getattr(fact, "reason", ""))
    if name == "SUCCEEDED":
        return TaskResult(TaskOutcome.SUCCEEDED, "TASK_SUCCEEDED")
    if name == "FAILED":
        if reason.startswith("TIMEOUT") or reason.startswith("WALL_TIMEOUT"):
            return TaskResult(TaskOutcome.TIMED_OUT, "TASK_TIMEOUT")
        return TaskResult(TaskOutcome.FAILED, f"TASK_FAILED: {reason}" if reason else "TASK_FAILED")
    return TaskResult()


def task_result_for_runtime(runtime_reason: str, fact: object | None) -> TaskResult:
    """Task dimensions come only from a real frozen judge fact.

    A runtime deadline/stop is not a judge timeout.  If the judge never froze a
    terminal state, the task dimension stays ``not_evaluated`` even when the
    runtime reason is TASK_TIMEOUT or TASK_SUCCEEDED (the caller separately
    rejects a claimed success without a SUCCEEDED fact).
    """
    del runtime_reason
    return task_result_from_fact(fact)


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
    return {
        "schema_version": TASK_FACT_SCHEMA_VERSION,
        "evaluated": True,
        "action_source": action_source,
        "test_only": bool(test_only),
        "result": {"outcome": result.outcome.value, "reason": result.reason},
        "task": {
            "state": str(getattr(fact, "state", "")),
            "reason": str(getattr(fact, "reason", "")),
            "task_epoch": int(getattr(fact, "task_epoch", 0)),
            "state_seq": int(getattr(fact, "state_seq", 0)),
            "sample_sim_time_ns": _stamp_ns(fact, "sample_time"),
            "task_start_sim_time_ns": _stamp_ns(fact, "task_start_time"),
            "first_satisfied_sim_time_ns": _stamp_ns(fact, "first_satisfied_time"),
            "completed_sim_time_ns": _stamp_ns(fact, "completed_time"),
            "stable_elapsed_s": float(getattr(fact, "stable_elapsed_s", 0.0)),
            "task_config_digest": str(getattr(fact, "task_config_digest", "")),
            "plan_digest": str(getattr(fact, "plan_digest", "")),
            "conditions_passed": bool(getattr(fact, "conditions_passed", False)),
            "conditions_bitmask": int(getattr(fact, "conditions_bitmask", 0)),
            "source_target_normal_force_n": float(
                getattr(fact, "source_target_normal_force", 0.0)
            ),
            "target_table_normal_force_n": float(
                getattr(fact, "target_table_normal_force", 0.0)
            ),
            "finger_contact_count": int(getattr(fact, "finger_contact_count", 0)),
            "right_gripper_opening_m": float(
                getattr(fact, "right_gripper_opening", 0.0)
            ),
            "right_tcp_distance_m": float(getattr(fact, "right_tcp_distance", 0.0)),
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
