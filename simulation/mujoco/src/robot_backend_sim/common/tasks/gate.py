"""Shared immutable per-capture Teacher gate fact structure."""

from __future__ import annotations

from dataclasses import dataclass

__all__ = ["TaskGateFact"]


@dataclass(frozen=True, slots=True)
class TaskGateFact:
    """Gate progress snapshot bound to one captured observation."""

    step_index: int
    step: str
    phase: str
    status: str
    active_start_sim_time_ns: int
    phase_wait_s: float
    stable_elapsed_s: float
    failure: str
