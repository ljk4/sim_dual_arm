"""Typed target-source boundary shared by policy and program-teacher streams.

The continuous stream core owns capture cadence, the fixed anchor, the
60x16/50 ms prediction, configured selection window and SubmitFollower identity.  A source
only turns one exact capture into one complete raw prediction; it never
submits a command, writes physics or invents a target clock.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

from robot_contracts.models import RunContext


@dataclass(frozen=True, slots=True)
class Prediction:
    """One complete source prediction, independent of its wire protocol."""

    context: RunContext
    observation_id: int
    joint_names: tuple[str, ...]
    times_from_observation_ns: tuple[int, ...]
    positions: tuple[tuple[float, ...], ...]
    target_count: int
    phase: str = ""
    gate_state: str = ""
    gate_failure: str = ""
    chunk_digest: str = ""
    task_end: str = ""
    task_reason: str = ""
    task_state: object | None = None


class TaskTerminal(Exception):
    """The task judge froze before a teacher chunk could be produced."""

    def __init__(self, runtime_reason: str, task_reason: str, state: object) -> None:
        self.runtime_reason = runtime_reason
        self.task_reason = task_reason
        self.state = state
        super().__init__(runtime_reason)


class TargetSource(Protocol):
    """A source that is scheduled by one continuous bounded-follower stream."""

    chunk_size: int
    target_count: int
    max_chunks: int
    initial_lead_ns: int
    predict_deadline_s: float
    poll_interval_s: float
    termination_mode: str

    def begin(self, context: RunContext, episode_id: str, task_id: str) -> int | None:
        """Open the source and return the explicit task start sim time if managed."""
        ...

    def predict(
        self, observation: object, chunk_index: int, request_id: str
    ) -> Prediction:
        """Return exactly ``chunk_size`` raw points for this exact capture."""
        ...

    def task_terminal(self) -> tuple[str, str, object | None]:
        """Return (runtime reason, task reason, frozen state) or empty values."""
        ...

    def finish(self) -> None:
        """Close the source without touching the robot or the recording."""
        ...


def task_terminal_fact(state: object) -> tuple[str, str]:
    """Map a frozen StackTaskState message to runtime/task vocabulary."""
    name = str(getattr(state, "state", ""))
    reason = str(getattr(state, "reason", ""))
    if name == "SUCCEEDED":
        return "TASK_SUCCEEDED", "TASK_SUCCEEDED"
    if name == "FAILED":
        if reason.startswith("TIMEOUT") or reason.startswith("WALL_TIMEOUT"):
            return "TASK_TIMEOUT", "TASK_TIMEOUT"
        return "TASK_FAILED", "TASK_FAILED"
    return "", ""


__all__ = ["Prediction", "TargetSource", "TaskTerminal", "task_terminal_fact"]
