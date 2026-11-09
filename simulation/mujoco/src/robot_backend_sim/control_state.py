"""Pure control leases, bounded command/chunk records and canonical requests.

Nothing in this module reads a scene, clock, file or network, and no physical
step happens here.  :class:`~robot_backend_sim.control.ControlSession` owns
the one ``MotionController``; this module owns the concrete lease and bounded
history values that surround it.
"""

from __future__ import annotations

import hashlib
import json
import math
import uuid
from dataclasses import dataclass, field
from numbers import Integral, Real
from typing import TYPE_CHECKING

from robot_contracts.control import (
    INT64_MAX,
    ControlCommand,
    RunContext,
    validate_context,
    validate_lease_duration,
    validate_owner,
    validate_wall,
)
from robot_contracts.stream_request import StreamRequest
from robot_contracts.trajectory import FOLLOWER_ALGORITHM_VERSION, PreparedFollowerPlan

if TYPE_CHECKING:  # pragma: no cover - import only for static type checking
    from .motion import MotionSnapshot

__all__ = [
    "MAX_CHUNK_HISTORY",
    "MAX_COMMAND_HISTORY",
    "ChunkHistory",
    "ChunkRecord",
    "ChunkStatus",
    "CommandHistory",
    "CommandRecord",
    "ControlEvent",
    "ControlReply",
    "ControlStatus",
    "DeferredEvents",
    "Lease",
    "LeaseBook",
    "MAX_DEFERRED_EVENTS",
    "MAX_EVENT_SEQUENCE",
    "PreparedControlCommand",
]

MAX_COMMAND_HISTORY = 4096
MAX_CHUNK_HISTORY = 4096
MAX_DEFERRED_EVENTS = 256
MAX_EVENT_SEQUENCE = 2**63 - 1
COMMAND_PHASES = frozenset({"ACCEPTED", "STARTED", "STOPPING", "COMPLETED", "CANCELED", "FAILED"})
COMMAND_ACTIVE_PHASES = frozenset({"ACCEPTED", "STARTED", "STOPPING"})
COMMAND_TERMINAL_PHASES = frozenset({"COMPLETED", "CANCELED", "FAILED"})
CHUNK_PHASES = frozenset({"ACCEPTED", "STARTED", "RETIRED", "CANCELED", "REJECTED"})
CHUNK_TERMINAL_PHASES = frozenset({"RETIRED", "CANCELED", "REJECTED"})


def _invalid(reason: str, detail: str) -> Exception:
    from robot_contracts.models import ContractError

    return ContractError(reason, detail)


def _finite_list(values: object, location: str) -> list[float]:
    if isinstance(values, str | bytes):
        raise _invalid("INVALID_PREPARED_COMMAND", f"{location}: expected an array of numbers")
    try:
        items = tuple(values)  # type: ignore[arg-type]
    except TypeError as error:
        raise _invalid(
            "INVALID_PREPARED_COMMAND", f"{location}: expected an array of numbers"
        ) from error
    result: list[float] = []
    for index, item in enumerate(items):
        if isinstance(item, bool) or not isinstance(item, Real):
            raise _invalid(
                "INVALID_PREPARED_COMMAND",
                f"{location}[{index}]: expected a finite real number",
            )
        number = float(item)
        if not math.isfinite(number):
            raise _invalid(
                "INVALID_PREPARED_COMMAND",
                f"{location}[{index}]: must be finite",
            )
        result.append(number)
    return result


def _plan_payload(plan: object) -> dict[str, object]:
    segments = []
    for segment in plan.segments:  # type: ignore[attr-defined]
        segments.append(
            {
                "duration_ns": int(segment.duration_ns),
                "c0": _finite_list(segment.c0, "plan.c0"),
                "c1": _finite_list(segment.c1, "plan.c1"),
                "c2": _finite_list(segment.c2, "plan.c2"),
                "c3": _finite_list(segment.c3, "plan.c3"),
            }
        )
    return {"joint_names": list(plan.joint_names), "segments": segments}


def _joints_payload(joints: object) -> list[dict[str, object]]:
    return [
        {
            "name": joint.name,
            "unit": joint.unit,
            "minimum": float(joint.minimum),
            "maximum": float(joint.maximum),
            "max_velocity": float(joint.max_velocity),
            "initial": float(joint.initial),
        }
        for joint in joints  # type: ignore[union-attr]
    ]


def _targets_payload(targets: object) -> list[dict[str, object]]:
    return [
        {
            "time_ns": int(target.time_ns),
            "positions": _finite_list(target.positions, "target.positions"),
        }
        for target in targets  # type: ignore[union-attr]
    ]


def _reference_payload(reference: object) -> dict[str, object]:
    return {
        "positions": _finite_list(reference.positions, "reference.positions"),
        "velocities": _finite_list(reference.velocities, "reference.velocities"),
        "accelerations": _finite_list(reference.accelerations, "reference.accelerations"),
    }


def _follower_payload(prepared: PreparedFollowerPlan) -> dict[str, object]:
    return {
        "kind": "FOLLOWER",
        "algorithm": FOLLOWER_ALGORITHM_VERSION,
        "is_final": prepared.is_final,
        "duration_grid_ns": prepared.duration_grid_ns,
        "anchor": {
            "stream_id": prepared.anchor.stream_id,
            "predecessor_seq": prepared.anchor.predecessor_seq,
            "start_sim_time_ns": prepared.anchor.start_sim_time_ns,
            "reference": _reference_payload(prepared.anchor.reference),
            "robot_profile_hash": prepared.anchor.robot_profile_hash,
        },
        "window": {
            "observation_id": prepared.window.observation_id,
            "observation_time_ns": prepared.window.observation_time_ns,
            "handoff_time_ns": prepared.window.handoff_time_ns,
            "source_targets": _targets_payload(prepared.window.source_targets),
            "targets": _targets_payload(prepared.window.targets),
            "target_count": prepared.window.target_count,
            "selected_indices": list(prepared.window.selected_indices),
        },
        "plan": _plan_payload(prepared.plan),
        "joints": _joints_payload(prepared.joints),
        "max_accelerations": _finite_list(prepared.max_accelerations, "prepared.max_accelerations"),
        "raw_terminal_velocities": _finite_list(
            prepared.raw_terminal_velocities, "prepared.raw_terminal_velocities"
        ),
        "restricted_terminal_velocities": _finite_list(
            prepared.restricted_terminal_velocities,
            "prepared.restricted_terminal_velocities",
        ),
        "actual_terminal_positions": _finite_list(
            prepared.actual_terminal_positions, "prepared.actual_terminal_positions"
        ),
        "actual_terminal_velocities": _finite_list(
            prepared.actual_terminal_velocities, "prepared.actual_terminal_velocities"
        ),
        "actual_terminal_accelerations": _finite_list(
            prepared.actual_terminal_accelerations, "prepared.actual_terminal_accelerations"
        ),
        "max_target_deviations": _finite_list(
            prepared.max_target_deviations, "prepared.max_target_deviations"
        ),
        "max_target_deviation_limits": _finite_list(
            prepared.max_target_deviation_limits, "prepared.max_target_deviation_limits"
        ),
        "requested_window_ns": prepared.requested_window_ns,
        "ruckig_duration_ns": prepared.ruckig_duration_ns,
        "extension_ns": prepared.extension_ns,
        "flags": {
            "velocity_limited": list(prepared.flags.velocity_limited),
            "acceleration_limited": list(prepared.flags.acceleration_limited),
            "boundary_preserved": list(prepared.flags.boundary_preserved),
        },
    }


def _command_payload(command: ControlCommand) -> dict[str, object]:
    context = command.context
    return {
        "context": {
            "robot_id": context.robot_id,
            "robot_profile_hash": context.robot_profile_hash,
            "backend_instance_id": context.backend_instance_id,
            "time_domain_id": context.time_domain_id,
            "clock_epoch": int(context.clock_epoch),
            "episode_id": context.episode_id,
            "control_epoch": int(context.control_epoch),
        },
        "lease_id": command.lease_id,
        "command_id": command.command_id,
        "command_seq": int(command.command_seq),
        "observation_id": int(command.observation_id),
        "accept_before_ns": int(command.accept_before_ns),
    }


def _kind_of(prepared: object) -> str:
    if isinstance(prepared, PreparedFollowerPlan):
        return "FOLLOWER"
    raise _invalid(
        "INVALID_PREPARED_COMMAND",
        "prepared must be a PreparedFollowerPlan",
    )


@dataclass(frozen=True, slots=True)
class PreparedControlCommand:
    """One immutable command envelope paired with its precompiled request.

    Construction happens in the caller's planning phase.  It validates the
    command/profile relation and chunk observation identity, then hashes the
    complete immutable request (identity, joints, source/window/boundaries,
    every cubic segment and final-settle mode), not only the last target.  The
    ``FOLLOWER`` payload carries the algorithm version and bounded-follower
    evidence for every stream block.
    """

    command: ControlCommand
    prepared: PreparedFollowerPlan
    # The complete source request is retained only so an idempotent retry can
    # prove that *every* field still matches.  It is excluded from equality and
    # the packet fingerprint because the compiled plan already covers it.
    request: StreamRequest | None = field(default=None, compare=False, repr=False)
    fingerprint: str = field(init=False, compare=False, repr=False)

    def __post_init__(self) -> None:
        if not isinstance(self.command, ControlCommand):
            raise _invalid("INVALID_PREPARED_COMMAND", "command must be a ControlCommand")
        if self.request is not None:
            if not isinstance(self.request, StreamRequest):
                raise _invalid(
                    "INVALID_PREPARED_COMMAND", "request must be a StreamRequest"
                )
            if self.request.command != self.command:
                raise _invalid(
                    "INVALID_PREPARED_COMMAND",
                    "retained request identity differs from the compiled command",
                )
        kind = _kind_of(self.prepared)
        context = self.command.context
        if kind == "FOLLOWER":
            prepared = self.prepared
            if prepared.anchor.robot_profile_hash != context.robot_profile_hash:
                raise _invalid(
                    "PROFILE_MISMATCH",
                    "prepared follower profile digest differs from command context",
                )
            if self.command.observation_id != prepared.window.observation_id:
                raise _invalid(
                    "INVALID_OBSERVATION",
                    "command observation_id must equal the prepared follower window",
                )
            payload = _follower_payload(prepared)
        else:  # pragma: no cover - _kind_of already rejects every other value
            raise _invalid("INVALID_PREPARED_COMMAND", "unsupported prepared kind")
        document = {"command": _command_payload(self.command), "prepared": payload}
        try:
            encoded = json.dumps(
                document,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            ).encode()
        except (TypeError, ValueError) as error:
            raise _invalid("INVALID_PREPARED_COMMAND", "request is not finite JSON") from error
        object.__setattr__(self, "fingerprint", hashlib.sha256(encoded).hexdigest())

    @property
    def kind(self) -> str:
        return _kind_of(self.prepared)


@dataclass(frozen=True, slots=True)
class ControlEvent:
    """One immutable controller event with original context/command identity."""

    event_seq: int
    context: RunContext
    command_id: str
    command_seq: int
    observation_id: int
    chunk_seq: int | None
    phase: str
    reason: str
    sim_time_ns: int

    def __post_init__(self) -> None:
        if not isinstance(self.context, RunContext):
            raise ValueError("context: must be a RunContext")
        for name in ("event_seq", "command_seq", "observation_id"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, Integral) or int(value) < 1:
                raise ValueError(f"{name}: must be a positive integer")
        if self.chunk_seq is not None and (
            isinstance(self.chunk_seq, bool)
            or not isinstance(self.chunk_seq, Integral)
            or int(self.chunk_seq) < 1
        ):
            raise ValueError("chunk_seq: must be a positive integer or None")
        if isinstance(self.sim_time_ns, bool) or not isinstance(self.sim_time_ns, Integral):
            raise ValueError("sim_time_ns: must be an integer")
        if int(self.sim_time_ns) < 0:
            raise ValueError("sim_time_ns: must not be negative")
        if not isinstance(self.command_id, str) or not self.command_id:
            raise ValueError("command_id: must be a non-empty string")
        if not isinstance(self.phase, str) or not self.phase:
            raise ValueError("phase: must be a non-empty string")
        if not isinstance(self.reason, str):
            raise ValueError("reason: must be a string")

    @property
    def scope(self) -> str:
        return "COMMAND" if self.chunk_seq is None else "CHUNK"


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


def _history_size(value: object, name: str, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise _invalid("INVALID_HISTORY_BOUND", f"{name} must be an integer")
    result = int(value)
    if result < 1 or result > maximum:
        raise _invalid("INVALID_HISTORY_BOUND", f"{name} must be in 1..{maximum}")
    return result


class CommandHistory:
    """Insertion-ordered bounded command records; active records never evicted."""

    def __init__(self, max_history: int = 256) -> None:
        self._max_size = _history_size(max_history, "max_history", MAX_COMMAND_HISTORY)
        self._records: dict[str, CommandRecord] = {}

    def __len__(self) -> int:
        return len(self._records)

    def get(self, command_id: str) -> CommandRecord | None:
        return self._records.get(command_id)

    def values(self) -> tuple[CommandRecord, ...]:
        return tuple(self._records.values())

    def clear(self) -> None:
        self._records.clear()

    def _evict_oldest_terminal(self) -> bool:
        for key, record in self._records.items():
            if record.phase in COMMAND_TERMINAL_PHASES:
                del self._records[key]
                return True
        return False

    def has_capacity(self) -> bool:
        return len(self._records) < self._max_size or any(
            record.phase in COMMAND_TERMINAL_PHASES for record in self._records.values()
        )

    def ensure_capacity(self) -> None:
        if len(self._records) < self._max_size:
            return
        if not self._evict_oldest_terminal():
            raise _invalid(
                "HISTORY_FULL",
                "command history is full and every retained command is active",
            )

    def add(self, record: CommandRecord) -> None:
        self.ensure_capacity()
        self._records[record.command.command_id] = record


class ChunkHistory:
    """Insertion-ordered bounded chunk records; active/pending chunks stay."""

    def __init__(self, max_history: int = 128) -> None:
        self._max_size = _history_size(max_history, "max_chunk_history", MAX_CHUNK_HISTORY)
        self._records: dict[tuple[str, int], ChunkRecord] = {}

    def __len__(self) -> int:
        return len(self._records)

    def get(self, command_id: str, chunk_seq: int) -> ChunkRecord | None:
        return self._records.get((command_id, chunk_seq))

    def values(self) -> tuple[ChunkRecord, ...]:
        return tuple(self._records.values())

    def clear(self) -> None:
        self._records.clear()

    def _evict_oldest_terminal(self) -> bool:
        for key, record in self._records.items():
            if record.phase in CHUNK_TERMINAL_PHASES:
                del self._records[key]
                return True
        return False

    def has_capacity(self) -> bool:
        return len(self._records) < self._max_size or any(
            record.phase in CHUNK_TERMINAL_PHASES for record in self._records.values()
        )

    def ensure_capacity(self) -> None:
        if len(self._records) < self._max_size:
            return
        if not self._evict_oldest_terminal():
            raise _invalid(
                "CHUNK_HISTORY_FULL",
                "chunk history is full and every retained chunk is active/pending",
            )

    def add(self, record: ChunkRecord) -> None:
        self.ensure_capacity()
        self._records[record.key] = record


class DeferredEvents:
    """Small bounded FIFO of already-created events awaiting a ControlReply."""

    def __init__(self, max_events: int = MAX_DEFERRED_EVENTS) -> None:
        if isinstance(max_events, bool) or not isinstance(max_events, Integral):
            raise _invalid("INVALID_DEFERRED_BOUND", "max_events must be an integer")
        size = int(max_events)
        if size < 1 or size > MAX_DEFERRED_EVENTS:
            raise _invalid(
                "INVALID_DEFERRED_BOUND",
                f"max_events must be in 1..{MAX_DEFERRED_EVENTS}",
            )
        self._max_size = size
        self._events: list[ControlEvent] = []

    def __len__(self) -> int:
        return len(self._events)

    def defer(self, events: tuple[ControlEvent, ...] | list[ControlEvent]) -> None:
        """Keep every event or fail before adding any of them."""
        items = tuple(events)
        if any(not isinstance(event, ControlEvent) for event in items):
            raise TypeError("deferred events must be ControlEvent values")
        if len(self._events) + len(items) > self._max_size:
            raise _invalid(
                "DEFERRED_EVENTS_FULL",
                "undelivered control events exceed the bounded capacity",
            )
        self._events.extend(items)

    def drain(self) -> tuple[ControlEvent, ...]:
        items = tuple(self._events)
        self._events.clear()
        return items


@dataclass(frozen=True, slots=True)
class Lease:
    context: RunContext
    owner: str
    lease_id: str
    expires_at_ns: int


class LeaseBook:
    """At most one explicit-wall lease; never reads a hidden clock."""

    def __init__(self) -> None:
        self._lease: Lease | None = None

    @property
    def current(self) -> Lease | None:
        return self._lease

    def clear(self) -> None:
        self._lease = None

    def acquire(
        self,
        context: RunContext,
        owner: str,
        duration_ns: int,
        wall_ns: int,
    ) -> Lease:
        validate_context(context)
        owner = validate_owner(owner)
        duration = validate_lease_duration(duration_ns)
        wall = validate_wall(wall_ns)
        if self._lease is not None:
            raise _invalid("CONTROL_BUSY", "a control lease already exists")
        expires = wall + duration
        if expires > INT64_MAX:
            raise _invalid("INVALID_WALL_TIME", "lease expiry exceeds int64 nanoseconds")
        lease = Lease(context, owner, f"lease-{uuid.uuid4().hex}", expires)
        self._lease = lease
        return lease

    def renew(self, lease_id: str, duration_ns: int, wall_ns: int) -> Lease:
        lease = self._lease
        if lease is None or lease.lease_id != lease_id:
            raise _invalid("LEASE_EXPIRED", "no matching control lease")
        duration = validate_lease_duration(duration_ns)
        wall = validate_wall(wall_ns)
        expires = wall + duration
        if expires > INT64_MAX:
            raise _invalid("INVALID_WALL_TIME", "lease expiry exceeds int64 nanoseconds")
        renewed = Lease(lease.context, lease.owner, lease.lease_id, expires)
        self._lease = renewed
        return renewed

    def release(self, lease_id: str) -> Lease:
        lease = self._lease
        if lease is None or lease.lease_id != lease_id:
            raise _invalid("LEASE_EXPIRED", "no matching control lease")
        self._lease = None
        return lease

    def expire(self, wall_ns: int) -> Lease | None:
        lease = self._lease
        if lease is None or int(wall_ns) < lease.expires_at_ns:
            return None
        self._lease = None
        return lease
