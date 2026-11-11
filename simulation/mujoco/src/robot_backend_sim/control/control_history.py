"""Bounded control histories and the single explicit-wall lease."""

from __future__ import annotations

import uuid
from dataclasses import dataclass
from numbers import Integral
from typing import TYPE_CHECKING

from robot_contracts.control import (
    INT64_MAX,
    RunContext,
    validate_context,
    validate_lease_duration,
    validate_owner,
    validate_wall,
)

if TYPE_CHECKING:
    from robot_backend_sim.control.control_state import ChunkRecord, CommandRecord

__all__ = [
    "MAX_CHUNK_HISTORY", "MAX_COMMAND_HISTORY", "MAX_DEFERRED_EVENTS",
    "ChunkHistory", "CommandHistory", "ControlEvent", "DeferredEvents", "Lease", "LeaseBook",
]

MAX_COMMAND_HISTORY = 4096
MAX_CHUNK_HISTORY = 4096
MAX_DEFERRED_EVENTS = 256
COMMAND_TERMINAL_PHASES = frozenset({"COMPLETED", "CANCELED", "FAILED"})
CHUNK_TERMINAL_PHASES = frozenset({"RETIRED", "CANCELED", "REJECTED"})


def _invalid(reason: str, detail: str) -> Exception:
    from robot_contracts.models import ContractError

    return ContractError(reason, detail)


class _History:
    """Insertion-ordered bounded records; active records are never evicted."""

    _terminal: frozenset[str]
    _full_reason: str
    _name: str

    def __init__(self, max_history: int, maximum: int) -> None:
        if isinstance(max_history, bool) or not isinstance(max_history, Integral):
            raise _invalid("INVALID_HISTORY_BOUND", f"{self._name} must be an integer")
        size = int(max_history)
        if size < 1 or size > maximum:
            raise _invalid("INVALID_HISTORY_BOUND", f"{self._name} must be in 1..{maximum}")
        self._max_size = size
        self._records: dict[object, object] = {}

    def __len__(self) -> int:
        return len(self._records)

    def get(self, key: object) -> object | None:
        return self._records.get(key)

    def values(self) -> tuple[object, ...]:
        return tuple(self._records.values())

    def clear(self) -> None:
        self._records.clear()

    def has_capacity(self) -> bool:
        return len(self._records) < self._max_size or any(
            record.phase in self._terminal  # type: ignore[attr-defined]
            for record in self._records.values()
        )

    def ensure_capacity(self) -> None:
        if len(self._records) < self._max_size:
            return
        for key, record in self._records.items():
            if record.phase in self._terminal:  # type: ignore[attr-defined]
                del self._records[key]
                return
        raise _invalid(self._full_reason, f"{self._name} is full and every record is active")

    def add(self, key: object, record: object) -> None:
        self.ensure_capacity()
        self._records[key] = record


class CommandHistory(_History):
    """Command records keyed by immutable command id."""

    _terminal = COMMAND_TERMINAL_PHASES
    _full_reason = "HISTORY_FULL"
    _name = "max_history"

    def __init__(self, max_history: int = 256) -> None:
        super().__init__(max_history, MAX_COMMAND_HISTORY)

    def get(self, command_id: str) -> CommandRecord | None:
        return super().get(command_id)  # type: ignore[return-value]

    def values(self) -> tuple[CommandRecord, ...]:
        return super().values()  # type: ignore[return-value]

    def add(self, record: CommandRecord) -> None:
        super().add(record.command.command_id, record)


class ChunkHistory(_History):
    """Chunk records keyed by ``(command_id, chunk_seq)``."""

    _terminal = CHUNK_TERMINAL_PHASES
    _full_reason = "CHUNK_HISTORY_FULL"
    _name = "max_chunk_history"

    def __init__(self, max_history: int = 128) -> None:
        super().__init__(max_history, MAX_CHUNK_HISTORY)

    def get(self, command_id: str, chunk_seq: int) -> ChunkRecord | None:
        return super().get((command_id, chunk_seq))  # type: ignore[return-value]

    def values(self) -> tuple[ChunkRecord, ...]:
        return super().values()  # type: ignore[return-value]

    def add(self, record: ChunkRecord) -> None:
        super().add(record.key, record)


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
        if (
            isinstance(self.sim_time_ns, bool)
            or not isinstance(self.sim_time_ns, Integral)
            or int(self.sim_time_ns) < 0
        ):
            raise ValueError("sim_time_ns: must be a non-negative integer")
        if not isinstance(self.command_id, str) or not self.command_id:
            raise ValueError("command_id: must be a non-empty string")
        if not isinstance(self.phase, str) or not self.phase:
            raise ValueError("phase: must be a non-empty string")
        if not isinstance(self.reason, str):
            raise ValueError("reason: must be a string")

    @property
    def scope(self) -> str:
        return "COMMAND" if self.chunk_seq is None else "CHUNK"


class DeferredEvents:
    """Small bounded FIFO of already-created events awaiting a ControlReply."""

    def __init__(self, max_events: int = MAX_DEFERRED_EVENTS) -> None:
        if isinstance(max_events, bool) or not isinstance(max_events, Integral):
            raise _invalid("INVALID_DEFERRED_BOUND", "max_events must be an integer")
        size = int(max_events)
        if size < 1 or size > MAX_DEFERRED_EVENTS:
            raise _invalid(
                "INVALID_DEFERRED_BOUND", f"max_events must be in 1..{MAX_DEFERRED_EVENTS}"
            )
        self._max_size = size
        self._events: list[ControlEvent] = []

    def __len__(self) -> int:
        return len(self._events)

    def defer(self, events: tuple[ControlEvent, ...] | list[ControlEvent]) -> None:
        items = tuple(events)
        if any(not isinstance(event, ControlEvent) for event in items):
            raise TypeError("deferred events must be ControlEvent values")
        if len(self._events) + len(items) > self._max_size:
            raise _invalid(
                "DEFERRED_EVENTS_FULL", "undelivered control events exceed the bounded capacity"
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

    def _validate(self, wall_ns: int, duration_ns: int) -> tuple[int, int]:
        duration = validate_lease_duration(duration_ns)
        wall = validate_wall(wall_ns)
        if wall + duration > INT64_MAX:
            raise _invalid("INVALID_WALL_TIME", "lease expiry exceeds int64 nanoseconds")
        return wall, duration

    def acquire(self, context: RunContext, owner: str, duration_ns: int, wall_ns: int) -> Lease:
        validate_context(context)
        owner = validate_owner(owner)
        wall, duration = self._validate(wall_ns, duration_ns)
        if self._lease is not None:
            raise _invalid("CONTROL_BUSY", "a control lease already exists")
        lease = Lease(context, owner, f"lease-{uuid.uuid4().hex}", wall + duration)
        self._lease = lease
        return lease

    def renew(self, lease_id: str, duration_ns: int, wall_ns: int) -> Lease:
        lease = self._lease
        if lease is None or lease.lease_id != lease_id:
            raise _invalid("LEASE_EXPIRED", "no matching control lease")
        wall, duration = self._validate(wall_ns, duration_ns)
        renewed = Lease(lease.context, lease.owner, lease.lease_id, wall + duration)
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
