"""Pure bounded scheduling helpers for continuous follower streams.

No MuJoCo, clock, thread, IO or SciPy call lives here.  ``StreamSchedule`` only
composes the single active plan and the single optional successor plus the
monotonic per-stream event sequence; :class:`~robot_backend_sim.common.motion.MotionController`
owns the physical state and performs the one real step.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Integral

from robot_contracts.action_chunks import ActionWindow
from robot_trajectory.follower import PreparedFollowerPlan
from robot_trajectory.sampling import sample_reference_interval

__all__ = [
    "ChunkEvent",
    "ChunkSubmission",
    "StreamSchedule",
    "boundary_rejection",
    "grid_aligned",
    "next_grid_time",
    "observation_rejection",
]

_BOUNDARY_ATOL = 1e-9
_EVENT_PHASES = frozenset({"ACCEPTED", "STARTED", "RETIRED", "CANCELED", "REJECTED"})


@dataclass(frozen=True, slots=True)
class ChunkEvent:
    """One immutable stream/chunk stage observation produced by one API call."""

    event_seq: int
    motion_seq: int
    stream_id: str
    chunk_seq: int
    observation_id: int
    planned_start_sim_ns: int
    planned_end_sim_ns: int
    sim_time_ns: int
    phase: str
    reason: str = ""

    def __post_init__(self) -> None:
        for field in (
            "event_seq",
            "motion_seq",
            "chunk_seq",
            "observation_id",
            "planned_start_sim_ns",
            "planned_end_sim_ns",
            "sim_time_ns",
        ):
            value = getattr(self, field)
            if isinstance(value, bool) or not isinstance(value, Integral) or value < 0:
                raise ValueError(f"{field}: must be a non-negative integer")
        if not isinstance(self.stream_id, str) or not self.stream_id:
            raise ValueError("stream_id: must be a non-empty string")
        if not isinstance(self.phase, str) or not self.phase:
            raise ValueError("phase: must be a non-empty string")
        if not isinstance(self.reason, str):
            raise ValueError("reason: must be a string")


@dataclass(frozen=True, slots=True)
class ChunkSubmission:
    """Result of ``queue_chunk``: accepted flag, reason and this-call snapshot."""

    accepted: bool
    reason: str
    snapshot: object


def grid_aligned(time_ns: int, step_ns: int) -> bool:
    """Whether an absolute simulation time lies exactly on the physics grid."""
    if isinstance(time_ns, bool) or not isinstance(time_ns, Integral):
        return False
    if isinstance(step_ns, bool) or not isinstance(step_ns, Integral) or step_ns <= 0:
        return False
    return int(time_ns) % int(step_ns) == 0


def next_grid_time(now_sim_ns: int, lead_ns: int, step_ns: int) -> int:
    """Round ``now + lead`` up to the next physics-grid instant, never down."""
    target = int(now_sim_ns) + int(lead_ns)
    remainder = target % int(step_ns)
    if remainder:
        target += int(step_ns) - remainder
    return target


def observation_rejection(
    window: ActionWindow, now_sim_ns: int, max_observation_age_ns: int
) -> str | None:
    """Return a reason for an unusable observation, or ``None`` when admissible."""
    if window.observation_time_ns > now_sim_ns:
        return "OBSERVATION_FROM_FUTURE"
    if now_sim_ns - window.observation_time_ns > max_observation_age_ns:
        return "OBSERVATION_STALE"
    return None


def _matches(left: float, right: float) -> bool:
    return math.isclose(left, right, rel_tol=0.0, abs_tol=_BOUNDARY_ATOL)


def boundary_rejection(
    active: PreparedFollowerPlan,
    successor: PreparedFollowerPlan,
    *,
    stream_id: str,
    profile_digest: str,
    joints: tuple[object, ...],
    max_accelerations: tuple[float, ...],
    now_sim_ns: int,
    min_lead_ns: int,
    max_observation_age_ns: int,
    step_ns: int,
) -> tuple[str | None, bool]:
    """Validate a successor enough to queue it without re-fitting the curve.

    Returns ``(reason, latches_fault)``.  Identity/order/profile/boundary
    errors are invalid packages and only reject.  A package that is otherwise a
    legal continuation of this stream but is late or based on a stale/future
    observation latches a bounded fault so the current plan cannot silently
    continue unserved.
    """
    if successor.anchor.stream_id != stream_id:
        return "STALE_STREAM", False
    if successor.anchor.predecessor_seq != active.seq:
        return "INVALID_CHUNK_ORDER", False
    if successor.start_backend_time_ns != active.end_backend_time_ns:
        return "INVALID_CHUNK_START", False
    if successor.anchor.robot_profile_hash != profile_digest:
        return "PROFILE_MISMATCH", False
    if tuple(successor.joints) != tuple(joints):
        return "PROFILE_MISMATCH", False
    if tuple(successor.max_accelerations) != tuple(max_accelerations):
        return "PROFILE_MISMATCH", False
    end = sample_reference_interval(active.plan, active.plan.duration_ns)
    if len(successor.anchor.reference.positions) != len(end.positions):
        return "INVALID_CHUNK_BOUNDARY", False
    for left, right in zip(successor.anchor.reference.positions, end.positions, strict=True):
        if not _matches(left, right):
            return "INVALID_CHUNK_BOUNDARY", False
    for left, right in zip(successor.anchor.reference.velocities, end.velocities, strict=True):
        if not _matches(left, right):
            return "INVALID_CHUNK_BOUNDARY", False
    if not grid_aligned(successor.end_backend_time_ns, step_ns):
        return "INVALID_CHUNK_GRID", False
    observation = observation_rejection(successor.window, now_sim_ns, max_observation_age_ns)
    if observation is not None:
        return observation, True
    if now_sim_ns > successor.start_backend_time_ns - min_lead_ns:
        return "CHUNK_LATE", True
    return None, False


class StreamSchedule:
    """Single active chunk, single optional successor, monotonic event ids."""

    def __init__(self, stream_id: str, motion_seq: int) -> None:
        if not isinstance(stream_id, str) or not stream_id:
            raise ValueError("stream_id must be a non-empty string")
        if isinstance(motion_seq, bool) or not isinstance(motion_seq, Integral):
            raise ValueError("motion_seq must be an integer")
        self.stream_id = stream_id
        self.motion_seq = int(motion_seq)
        self.active: PreparedFollowerPlan | None = None
        self.pending: PreparedFollowerPlan | None = None
        self.started = False
        self.terminal = False
        self._event_seq = 0

    def make_event(
        self,
        phase: str,
        chunk: PreparedFollowerPlan,
        sim_time_ns: int,
        reason: str = "",
    ) -> ChunkEvent:
        """Create exactly one event; callers include only this call's events."""
        self._event_seq += 1
        return ChunkEvent(
            event_seq=self._event_seq,
            motion_seq=self.motion_seq,
            stream_id=self.stream_id,
            chunk_seq=chunk.seq,
            observation_id=chunk.window.observation_id,
            planned_start_sim_ns=chunk.start_backend_time_ns,
            planned_end_sim_ns=chunk.end_backend_time_ns,
            sim_time_ns=int(sim_time_ns),
            phase=phase,
            reason=reason,
        )

    def install_start(self, chunk: PreparedFollowerPlan) -> None:
        if self.active is not None or self.started:
            raise ValueError("stream already has an active chunk")
        self.active = chunk

    def install_pending(self, chunk: PreparedFollowerPlan) -> None:
        if self.active is None or self.pending is not None:
            raise ValueError("stream cannot accept a pending chunk")
        self.pending = chunk

    def handoff(self) -> tuple[PreparedFollowerPlan, PreparedFollowerPlan]:
        if self.active is None or self.pending is None:
            raise ValueError("handoff requires active and pending chunks")
        retired, started = self.active, self.pending
        self.active = started
        self.pending = None
        return retired, started

    def cancel_pending(self) -> PreparedFollowerPlan | None:
        canceled = self.pending
        self.pending = None
        return canceled

    def close(self) -> None:
        if self.active is not None or self.pending is not None or self.started:
            self.terminal = True
