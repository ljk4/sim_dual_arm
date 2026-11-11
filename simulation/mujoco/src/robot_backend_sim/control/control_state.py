"""Pure control leases, bounded command/chunk records and canonical requests.

Nothing in this module reads a scene, clock, file or network, and no physical
step happens here.  :class:`~robot_backend_sim.control.ControlSession` owns the
one ``MotionController``; this module owns the concrete lease and bounded
history values that surround it.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from robot_contracts.control import ControlCommand, RunContext
from robot_contracts.stream_request import StreamRequest
from robot_contracts.trajectory.constants import FOLLOWER_ALGORITHM_VERSION
from robot_trajectory.follower import PreparedFollowerPlan

from robot_backend_sim.control.control_history import (
    MAX_CHUNK_HISTORY,
    MAX_COMMAND_HISTORY,
    MAX_DEFERRED_EVENTS,
    ChunkHistory,
    CommandHistory,
    ControlEvent,
    DeferredEvents,
    Lease,
    LeaseBook,
)

if TYPE_CHECKING:  # pragma: no cover - import only for static type checking
    from robot_backend_sim.motion.motion_types import MotionSnapshot

__all__ = [
    "MAX_CHUNK_HISTORY", "MAX_COMMAND_HISTORY", "MAX_DEFERRED_EVENTS", "MAX_EVENT_SEQUENCE",
    "ChunkHistory", "ChunkRecord", "ChunkStatus", "CommandHistory", "CommandRecord",
    "ControlEvent", "ControlReply", "ControlStatus", "DeferredEvents", "Lease", "LeaseBook",
    "PreparedControlCommand",
]

MAX_EVENT_SEQUENCE = 2**63 - 1
COMMAND_PHASES = frozenset({"ACCEPTED", "STARTED", "STOPPING", "COMPLETED", "CANCELED", "FAILED"})
COMMAND_ACTIVE_PHASES = frozenset({"ACCEPTED", "STARTED", "STOPPING"})
COMMAND_TERMINAL_PHASES = frozenset({"COMPLETED", "CANCELED", "FAILED"})
CHUNK_PHASES = frozenset({"ACCEPTED", "STARTED", "RETIRED", "CANCELED", "REJECTED"})
CHUNK_TERMINAL_PHASES = frozenset({"RETIRED", "CANCELED", "REJECTED"})


def _invalid(reason: str, detail: str) -> Exception:
    from robot_contracts.models import ContractError

    return ContractError(reason, detail)


def _fingerprint_payload(prepared: PreparedFollowerPlan) -> dict[str, object]:
    """Cover algorithm identity plus every excluded terminal/deviation fact."""
    payload = prepared.canonical_dict()
    payload.update(
        {
            "kind": "FOLLOWER",
            "algorithm": FOLLOWER_ALGORITHM_VERSION,
            "raw_terminal_velocities": list(prepared.raw_terminal_velocities),
            "restricted_terminal_velocities": list(prepared.restricted_terminal_velocities),
            "max_target_deviation_limits": list(prepared.max_target_deviation_limits),
            "max_target_deviations": list(prepared.max_target_deviations),
            "actual_terminal_positions": list(prepared.actual_terminal_positions),
            "actual_terminal_velocities": list(prepared.actual_terminal_velocities),
            "actual_terminal_accelerations": list(prepared.actual_terminal_accelerations),
            "extension_ns": prepared.extension_ns,
        }
    )
    return payload


def _kind_of(prepared: object) -> str:
    if isinstance(prepared, PreparedFollowerPlan):
        return "FOLLOWER"
    raise _invalid("INVALID_PREPARED_COMMAND", "prepared must be a PreparedFollowerPlan")


@dataclass(frozen=True, slots=True)
class PreparedControlCommand:
    """One immutable command envelope paired with its precompiled plan."""

    command: ControlCommand
    prepared: PreparedFollowerPlan
    request: StreamRequest | None = field(default=None, compare=False, repr=False)
    fingerprint: str = field(init=False, compare=False, repr=False)

    def __post_init__(self) -> None:
        if not isinstance(self.command, ControlCommand):
            raise _invalid("INVALID_PREPARED_COMMAND", "command must be a ControlCommand")
        if self.request is not None:
            if not isinstance(self.request, StreamRequest):
                raise _invalid("INVALID_PREPARED_COMMAND", "request must be a StreamRequest")
            if self.request.command != self.command:
                raise _invalid(
                    "INVALID_PREPARED_COMMAND",
                    "retained request identity differs from the compiled command",
                )
        if _kind_of(self.prepared) != "FOLLOWER":
            raise _invalid("INVALID_PREPARED_COMMAND", "unsupported prepared kind")
        if self.prepared.anchor.robot_profile_hash != self.command.context.robot_profile_hash:
            raise _invalid(
                "PROFILE_MISMATCH",
                "prepared follower profile digest differs from command context",
            )
        if self.command.observation_id != self.prepared.window.observation_id:
            raise _invalid(
                "INVALID_OBSERVATION",
                "command observation_id must equal the prepared follower window",
            )
        document = {
            "command": self.command.canonical_dict(),
            "prepared": _fingerprint_payload(self.prepared),
        }
        try:
            encoded = json.dumps(
                document, sort_keys=True, separators=(",", ":"), allow_nan=False
            ).encode()
        except (TypeError, ValueError) as error:
            raise _invalid("INVALID_PREPARED_COMMAND", "request is not finite JSON") from error
        object.__setattr__(self, "fingerprint", hashlib.sha256(encoded).hexdigest())

    @property
    def kind(self) -> str:
        return _kind_of(self.prepared)


@dataclass(frozen=True, slots=True)
class ControlStatus:
    """Immutable overall command status; terminal instances are frozen."""

    context: RunContext
    command_id: str
    command_seq: int
    observation_id: int
    phase: str
    reason: str
    stop_reason: str | None
    motion_seq: int | None
    stream_id: str | None
    chunk_seq: int | None
    accepted_sim_ns: int
    started_sim_ns: int | None
    finished_sim_ns: int | None
    accepted_wall_ns: int | None
    started_wall_ns: int | None
    finished_wall_ns: int | None
    snapshot: MotionSnapshot | None


@dataclass(frozen=True, slots=True)
class ChunkStatus:
    """Immutable per-chunk status preserving the real phase and reason."""

    context: RunContext
    command_id: str
    command_seq: int
    chunk_seq: int
    observation_id: int
    phase: str
    reason: str
    accepted_sim_ns: int
    started_sim_ns: int | None
    finished_sim_ns: int | None
    accepted_wall_ns: int | None
    started_wall_ns: int | None
    finished_wall_ns: int | None
    snapshot: MotionSnapshot | None


@dataclass(frozen=True, slots=True)
class ControlReply:
    """One call's live snapshot, aggregate status and this-call events."""

    snapshot: MotionSnapshot
    status: ControlStatus | None
    events: tuple[ControlEvent, ...]
    accepted: bool | None = None
    reason: str = ""


@dataclass(slots=True)
class CommandRecord:
    packet: PreparedControlCommand
    phase: str
    reason: str
    stop_reason: str | None
    accepted_sim_ns: int
    accepted_wall_ns: int
    started_sim_ns: int | None = None
    finished_sim_ns: int | None = None
    started_wall_ns: int | None = None
    finished_wall_ns: int | None = None
    motion_seq: int | None = None
    stream_id: str | None = None
    latest_snapshot: MotionSnapshot | None = None
    terminal_status: ControlStatus | None = None
    last_control_event_seq: int = 0
    max_chunk_seq: int = 0

    @property
    def command(self) -> ControlCommand:
        return self.packet.command

    @property
    def kind(self) -> str:
        return self.packet.kind


@dataclass(slots=True)
class ChunkRecord:
    packet: PreparedControlCommand
    phase: str
    reason: str
    accepted_sim_ns: int
    accepted_wall_ns: int
    started_sim_ns: int | None = None
    finished_sim_ns: int | None = None
    started_wall_ns: int | None = None
    finished_wall_ns: int | None = None
    latest_snapshot: MotionSnapshot | None = None

    @property
    def command(self) -> ControlCommand:
        return self.packet.command

    @property
    def chunk(self) -> PreparedFollowerPlan:
        return self.packet.prepared

    @property
    def key(self) -> tuple[str, int]:
        return self.command.command_id, self.chunk.seq


def _status_from_record(record: CommandRecord) -> ControlStatus:
    if record.terminal_status is not None:
        return record.terminal_status
    snapshot = record.latest_snapshot
    return ControlStatus(
        context=record.command.context,
        command_id=record.command.command_id,
        command_seq=record.command.command_seq,
        observation_id=record.command.observation_id,
        phase=record.phase,
        reason=record.reason,
        stop_reason=record.stop_reason,
        motion_seq=record.motion_seq,
        stream_id=record.stream_id,
        chunk_seq=None if snapshot is None else snapshot.chunk_seq,
        accepted_sim_ns=record.accepted_sim_ns,
        started_sim_ns=record.started_sim_ns,
        finished_sim_ns=record.finished_sim_ns,
        accepted_wall_ns=record.accepted_wall_ns,
        started_wall_ns=record.started_wall_ns,
        finished_wall_ns=record.finished_wall_ns,
        snapshot=snapshot,
    )


def _status_from_chunk(chunk: ChunkRecord) -> ChunkStatus:
    prepared = chunk.chunk
    return ChunkStatus(
        context=chunk.command.context,
        command_id=chunk.command.command_id,
        command_seq=chunk.command.command_seq,
        chunk_seq=prepared.seq,
        observation_id=prepared.window.observation_id,
        phase=chunk.phase,
        reason=chunk.reason,
        accepted_sim_ns=chunk.accepted_sim_ns,
        started_sim_ns=chunk.started_sim_ns,
        finished_sim_ns=chunk.finished_sim_ns,
        accepted_wall_ns=chunk.accepted_wall_ns,
        started_wall_ns=chunk.started_wall_ns,
        finished_wall_ns=chunk.finished_wall_ns,
        snapshot=chunk.latest_snapshot,
    )
