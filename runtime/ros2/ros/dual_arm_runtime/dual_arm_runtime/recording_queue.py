"""Bounded, write-behind queue in front of one MCAP writer thread.

The queue is deliberately transport-agnostic so it can be exercised without ROS.
It owns all producer-facing state and exactly one consumer thread.  The
``rosbag2_py`` writer is created, opened, used and closed only by that thread.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol


@dataclass(frozen=True)
class RecordingSettings:
    """Immutable limits for one episode recorder."""

    queue_capacity: int = 1024
    max_pending_bytes: int = 64 * 1024 * 1024
    startup_timeout_s: float = 5.0
    close_timeout_s: float = 5.0

    def __post_init__(self) -> None:
        for name in ("queue_capacity", "max_pending_bytes"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise ValueError(f"{name} must be a positive integer")
        for name in ("startup_timeout_s", "close_timeout_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, int | float):
                raise ValueError(f"{name} must be a finite positive number")
            number = float(value)
            if not math.isfinite(number) or number <= 0:
                raise ValueError(f"{name} must be a finite positive number")
            object.__setattr__(self, name, number)


@dataclass(frozen=True)
class RecordingStats:
    """Read-only queue counters; ``written`` counts only completed writes."""

    accepted: int
    written: int
    pending_items: int
    pending_bytes: int
    peak_items: int
    peak_bytes: int
    failure_reason: str
    written_counts: tuple[tuple[str, int], ...]


class RecordingQueueError(RuntimeError):
    """An append, writer or close failure that must not be reported as success."""


class Writer(Protocol):
    """Minimal writer surface owned by the queue's consumer thread."""

    def open(self) -> None: ...

    def write(self, topic: str, payload: bytes, timestamp_ns: int) -> None: ...

    def close(self) -> None: ...


class BoundedRecordingQueue:
    """One producer-facing bounded queue and one writer thread.

    Producers serialize outside this class and hand over immutable ``bytes``.
    A dequeued item remains in the byte/count budget until its write returns, so a
    slow disk cannot make a large in-flight frame disappear from the limits.
    """

    def __init__(
        self,
        settings: RecordingSettings,
        writer_factory: Callable[[], Writer],
        topics: dict[str, str],
        *,
        thread_name: str = "mcap-writer",
    ) -> None:
        self.settings = settings
        self._writer_factory = writer_factory
        self._topics = frozenset(topics)
        self._condition = threading.Condition(threading.Lock())
        self._items: deque[tuple[str, bytes, int]] = deque()
        self._pending_items = 0
        self._pending_bytes = 0
        self._accepted = 0
        self._written = 0
        self._peak_items = 0
        self._peak_bytes = 0
        self._written_counts: dict[str, int] = {}
        self._failure_reason = ""
        self._accepting = True
        self._closing = False
        self._startup_ready = False
        self._writer_done = False
        self._thread = threading.Thread(
            target=self._run_writer,
            name=thread_name,
            daemon=True,
        )
        self._thread.start()
        self._wait_ready()

    # ---- Producer surface ----------------------------------------------------------

    def fail(self, reason: str) -> None:
        """Latch a producer-side rejection as an irremovable recording fault."""
        if not isinstance(reason, str) or not reason:
            raise ValueError("recording failure reason must be a non-empty string")
        with self._condition:
            self._latch_failure_locked(reason)

    def append(self, topic: str, payload: bytes, timestamp_ns: int) -> None:
        """Enqueue an already serialized payload without waiting for disk/capacity.

        Every rejected append latches the same permanent failure as an asynchronous
        writer error, so catching the exception cannot later turn into a success.
        """
        if not isinstance(topic, str) or topic not in self._topics:
            reason = f"unregistered recording topic: {topic!r}"
            self.fail(reason)
            raise RecordingQueueError(reason)
        if isinstance(timestamp_ns, bool) or not isinstance(timestamp_ns, int):
            reason = "recording timestamp must be an integer"
            self.fail(reason)
            raise RecordingQueueError(reason)
        if timestamp_ns < 0:
            reason = "recording timestamp must not be negative"
            self.fail(reason)
            raise RecordingQueueError(reason)
        if not isinstance(payload, bytes):
            reason = "recording payload must be bytes"
            self.fail(reason)
            raise RecordingQueueError(reason)
        size = len(payload)
        with self._condition:
            if not self._accepting:
                reason = self._failure_reason or "recording queue is closed"
                raise self._reject_locked(reason)
            if size > self.settings.max_pending_bytes:
                raise self._reject_locked(
                    f"payload of {size} bytes exceeds max_pending_bytes "
                    f"{self.settings.max_pending_bytes}"
                )
            if (
                self._pending_items >= self.settings.queue_capacity
                or self._pending_bytes + size > self.settings.max_pending_bytes
            ):
                raise self._reject_locked(
                    "recording queue is full: "
                    f"{self._pending_items}/{self.settings.queue_capacity} items, "
                    f"{self._pending_bytes + size}/{self.settings.max_pending_bytes} bytes"
                )
            self._items.append((topic, payload, timestamp_ns))
            self._pending_items += 1
            self._pending_bytes += size
            self._accepted += 1
            self._peak_items = max(self._peak_items, self._pending_items)
            self._peak_bytes = max(self._peak_bytes, self._pending_bytes)
            self._condition.notify_all()

    def check_health(self) -> None:
        """Raise the latched asynchronous failure without touching the filesystem."""
        with self._condition:
            failure = self._failure_reason
        if failure:
            raise RecordingQueueError(failure)

    def stats(self) -> RecordingStats:
        with self._condition:
            return RecordingStats(
                accepted=self._accepted,
                written=self._written,
                pending_items=self._pending_items,
                pending_bytes=self._pending_bytes,
                peak_items=self._peak_items,
                peak_bytes=self._peak_bytes,
                failure_reason=self._failure_reason,
                written_counts=tuple(sorted(self._written_counts.items())),
            )

    def close(self, timeout_s: float | None = None) -> RecordingStats:
        """Stop accepting, drain and wait for the writer's own close call.

        A timeout or failure is latched and re-raised by every later close.  The
        caller never closes the writer handle and never waits without a deadline.
        """
        timeout = self.settings.close_timeout_s if timeout_s is None else timeout_s
        if isinstance(timeout, bool) or not isinstance(timeout, int | float):
            raise ValueError("close timeout must be a finite positive number")
        timeout = float(timeout)
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("close timeout must be a finite positive number")
        deadline = time.monotonic() + timeout
        with self._condition:
            self._accepting = False
            self._closing = True
            self._condition.notify_all()
            while not self._writer_done:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    if not self._failure_reason:
                        self._latch_failure_locked(f"writer close timed out after {timeout:g}s")
                    break
                self._condition.wait(timeout=min(remaining, 0.05))
            if self._writer_done and not self._failure_reason:
                if self._pending_items or self._pending_bytes:
                    self._latch_failure_locked("writer closed with queued data still pending")
            failure = self._failure_reason
        if failure:
            raise RecordingQueueError(failure)
        return self.stats()

    # ---- Consumer thread -----------------------------------------------------------

    def _wait_ready(self) -> None:
        deadline = time.monotonic() + self.settings.startup_timeout_s
        with self._condition:
            while not self._startup_ready and not self._failure_reason:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    self._latch_failure_locked(
                        f"writer startup timed out after {self.settings.startup_timeout_s:g}s"
                    )
                    break
                self._condition.wait(timeout=min(remaining, 0.05))
            failure = self._failure_reason
        if failure:
            raise RecordingQueueError(failure)

    def _reject_locked(self, reason: str) -> RecordingQueueError:
        self._latch_failure_locked(reason)
        return RecordingQueueError(self._failure_reason or reason)

    def _latch_failure_locked(self, reason: str) -> None:
        if not self._failure_reason:
            self._failure_reason = reason
        self._accepting = False
        self._condition.notify_all()

    def _run_writer(self) -> None:
        writer: Writer | None = None
        opened = False
        try:
            writer = self._writer_factory()
            writer.open()
            opened = True
            with self._condition:
                if self._failure_reason:
                    return
                self._startup_ready = True
                self._condition.notify_all()
            while True:
                with self._condition:
                    while not self._items and not self._closing and not self._failure_reason:
                        self._condition.wait(timeout=0.05)
                    if self._failure_reason:
                        break
                    if not self._items:
                        break
                    topic, payload, timestamp_ns = self._items.popleft()
                    # Still counted as pending until write() returns.
                writer.write(topic, payload, timestamp_ns)
                with self._condition:
                    self._pending_items -= 1
                    self._pending_bytes -= len(payload)
                    self._written += 1
                    self._written_counts[topic] = self._written_counts.get(topic, 0) + 1
                    self._condition.notify_all()
        except BaseException as error:
            with self._condition:
                self._latch_failure_locked(f"writer failed: {type(error).__name__}: {error}")
        finally:
            if opened and writer is not None:
                try:
                    writer.close()
                except BaseException as error:
                    with self._condition:
                        self._latch_failure_locked(
                            f"writer close failed: {type(error).__name__}: {error}"
                        )
            with self._condition:
                self._writer_done = True
                self._closing = True
                self._condition.notify_all()
