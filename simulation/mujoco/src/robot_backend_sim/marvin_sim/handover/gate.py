"""Immutable per-capture gate facts for the handover Teacher.

The task runtime advances a behaviour step only from measured evidence.  The
owner thread freezes the resulting step fact together with its capture, so a
Teacher chunk can never describe a gate that had not passed at that capture.
"""

from __future__ import annotations

from dataclasses import dataclass

__all__ = ["HandoverFeedback", "HandoverGateFact"]


@dataclass(frozen=True, slots=True)
class HandoverGateFact:
    step_index: int
    step: str
    phase: str
    status: str
    active_start_sim_time_ns: int
    phase_wait_s: float
    stable_elapsed_s: float
    failure: str


@dataclass(frozen=True, slots=True)
class HandoverFeedback:
    capture_seq: int
    sim_time_ns: int
    gate: HandoverGateFact
