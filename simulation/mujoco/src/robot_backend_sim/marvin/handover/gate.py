"""Immutable per-capture gate facts for the handover Teacher.

The task runtime advances a behaviour step only from measured evidence.  The
owner thread freezes the resulting step fact together with its capture, so a
Teacher chunk can never describe a gate that had not passed at that capture.
"""

from __future__ import annotations

from dataclasses import dataclass

from robot_backend_sim.common.tasks.base import TaskGateFact

__all__ = ["HandoverFeedback", "HandoverGateFact"]

HandoverGateFact = TaskGateFact


@dataclass(frozen=True, slots=True)
class HandoverFeedback:
    capture_seq: int
    sim_time_ns: int
    gate: HandoverGateFact
