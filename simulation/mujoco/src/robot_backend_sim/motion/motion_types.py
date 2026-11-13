"""Immutable motion state-machine types shared by the controller components."""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

from robot_contracts.trajectory.spline import ReferenceSample

from robot_backend_sim.motion.stream_schedule import ChunkEvent
from robot_backend_sim.physics.contacts import GripperCompletion, GripperContactEvidence
from robot_backend_sim.physics.robot_io import Measurement

__all__ = [
    "ACTIVE_PHASES",
    "TERMINAL_PHASES",
    "AppliedStep",
    "BoundarySelection",
    "FailedReason",
    "MotionResult",
    "MotionSnapshot",
    "Phase",
    "StopReason",
]


class Phase(StrEnum):
    """Local motion phase; this is not a ROS command acknowledgement."""

    IDLE = "IDLE"
    ACCEPTED = "ACCEPTED"
    RUNNING = "RUNNING"
    SETTLING = "SETTLING"
    STOPPING = "STOPPING"
    COMPLETED = "COMPLETED"
    STOPPED = "STOPPED"
    FAILED = "FAILED"


class StopReason(StrEnum):
    STOP_REQUESTED = "STOP_REQUESTED"
    TRACKING_TIMEOUT = "TRACKING_TIMEOUT"
    WALL_TIMEOUT = "WALL_TIMEOUT"


class FailedReason(StrEnum):
    TRACKING_TIMEOUT = "TRACKING_TIMEOUT"
    WALL_TIMEOUT = "WALL_TIMEOUT"
    STOP_TIMEOUT = "STOP_TIMEOUT"
    STOP_PLAN_REJECTED = "STOP_PLAN_REJECTED"
    PROGRESS_TIMEOUT = "PROGRESS_TIMEOUT"
    UNDERRUN = "UNDERRUN"
    CHUNK_LATE = "CHUNK_LATE"
    OBSERVATION_STALE = "OBSERVATION_STALE"
    OBSERVATION_FROM_FUTURE = "OBSERVATION_FROM_FUTURE"
    STREAM_START_MISSED = "STREAM_START_MISSED"


ACTIVE_PHASES = frozenset({Phase.ACCEPTED, Phase.RUNNING, Phase.SETTLING, Phase.STOPPING})
TERMINAL_PHASES = frozenset({Phase.COMPLETED, Phase.STOPPED, Phase.FAILED})


@dataclass(frozen=True, slots=True)
class MotionSnapshot:
    """One immutable live snapshot of reference, measurement and evidence."""

    phase: Phase
    motion_seq: int
    reason: str
    accepted_wall_ns: int | None
    started_wall_ns: int | None
    started_sim_ns: int | None
    reference_application_sim_time_ns: int
    reference_elapsed_ns: int
    measurement_sim_time_ns: int
    stable_for_ns: int
    joint_names: tuple[str, ...]
    reference_positions: tuple[float, ...]
    reference_velocities: tuple[float, ...]
    reference_accelerations: tuple[float, ...]
    measured_positions: tuple[float, ...]
    measured_velocities: tuple[float, ...]
    tracking_errors: tuple[float, ...]
    grippers: tuple[GripperCompletion, ...]
    contacts: tuple[GripperContactEvidence, ...]
    stream_id: str | None = None
    chunk_seq: int | None = None
    reference_kind: str = "HOLD"
    tracking_velocity_errors: tuple[float, ...] = ()
    tracking_quality: str = "HOLD"
    tracking_violation_ns: int = 0
    chunk_events: tuple[ChunkEvent, ...] = ()
    sampled_reference_positions: tuple[float, ...] = ()
    sampled_reference_velocities: tuple[float, ...] = ()
    reference_boundary_corrections: tuple[float, ...] = ()


@dataclass(frozen=True, slots=True)
class MotionResult:
    """Terminal result frozen at the first COMPLETED/STOPPED/FAILED transition."""

    motion_seq: int
    phase: Phase
    reason: str
    stop_reason: str | None
    accepted_wall_ns: int | None
    started_wall_ns: int | None
    started_sim_ns: int | None
    finished_wall_ns: int
    snapshot: MotionSnapshot

    @property
    def terminal_snapshot(self) -> MotionSnapshot:
        return self.snapshot


@dataclass(frozen=True, slots=True)
class BoundarySelection:
    """Pure reference choice for one active-chunk endpoint."""

    reference: ReferenceSample
    elapsed_ns: int
    reference_kind: str
    chunk_seq: int
    action: str


@dataclass(frozen=True, slots=True)
class AppliedStep:
    """One coherent physical application, used as the source for a later brake.

    ``reference`` is what was actually written to the selected robot I/O.  For a follower,
    ``sampled_reference`` keeps the untouched coefficient sample and
    ``reference_corrections`` keeps ``sampled - applied`` per axis so the shared
    boundary normalization is independently auditable.  Strict chunks keep the
    two references identical and corrections empty.
    """

    reference: ReferenceSample
    measurement: Measurement
    contacts: tuple[GripperContactEvidence, ...]
    application_sim_time_ns: int
    elapsed_ns: int
    reference_kind: str
    sampled_reference: ReferenceSample
    reference_corrections: tuple[float, ...]
