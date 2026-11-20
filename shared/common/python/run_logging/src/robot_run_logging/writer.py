"""Single-writer event logger with bounded queues and explicit failure state."""

from __future__ import annotations

import json
import os
import queue
import sys
import threading
import time
import uuid
from collections.abc import Mapping
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from typing import BinaryIO

from .events import (
    EventContext,
    LogFailure,
    LogSettings,
    Scalar,
    prepare_event,
    validate_identifier,
)


@dataclass(frozen=True)
class _QueueItem:
    body: bytes
    critical: bool
    console: str | None


class _BudgetExceeded(RuntimeError):
    pass


def _utc_now() -> str:
    return datetime.now(UTC).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _sync_file(fd: int) -> None:
    os.fsync(fd)


def _bounded_reason(reason: str) -> str:
    text = " ".join(str(reason).splitlines())
    return text[:500] if len(text) > 500 else text


def _exception_reason(prefix: str, exception: BaseException) -> str:
    return f"{prefix}: {type(exception).__name__}: {exception}"


class EventLogger:
    """One process-wide writer thread owning one run/component directory.

    The constructor exclusively creates the instance directory.  A single
    writer thread owns the event file handles, rotations, and summary files;
    producers only validate, render, and enqueue without waiting for disk.
    """

    def __init__(
        self,
        directory: Path,
        *,
        run_id: str,
        component: str,
        settings: LogSettings,
        instance_id: str | None = None,
    ) -> None:
        if not isinstance(settings, LogSettings):
            raise ValueError("settings must be a LogSettings")
        self._settings = settings
        self._run_id = validate_identifier(run_id, "run_id")
        self._component = validate_identifier(component, "component")
        self._instance_id = (
            str(uuid.uuid4())
            if instance_id is None
            else validate_identifier(instance_id, "instance_id")
        )
        self._directory = Path(directory)
        self._directory.mkdir(parents=True, exist_ok=False)

        self._queue: queue.Queue[_QueueItem] = queue.Queue(maxsize=settings.queue_capacity)
        self._state_lock = threading.Lock()
        self._submit_lock = threading.Lock()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._writer_done = threading.Event()

        self._failure: str | None = None
        self._failure_reported = False
        self._closed = False
        self._close_complete = False
        self._close_failed = False
        self._close_failure: str | None = None
        self._writer_terminal = False

        self._attempted = 0
        self._enqueued = 0
        self._written = 0
        self._dropped_noncritical = 0
        self._next_seq = 1

        self._handle: BinaryIO | None = None
        self._files: list[str] = []
        self._next_file_index = 1
        self._file_bytes = 0
        self._total_bytes = 0
        self._last_flush = time.monotonic()
        self._started_ns = time.monotonic_ns()

        self._thread = threading.Thread(
            target=self._run,
            name=f"run-logging-{self._instance_id}",
            daemon=True,
        )
        self._thread.start()
        if not self._ready.wait(max(1.0, settings.close_timeout_s)):
            self._set_failure("writer did not become ready within close_timeout_s")
            self._stop.set()
            raise LogFailure("writer did not become ready within close_timeout_s")
        failure = self._get_failure()
        if failure is not None:
            raise LogFailure(failure)

    @property
    def directory(self) -> Path:
        return self._directory

    @property
    def instance_id(self) -> str:
        return self._instance_id

    def emit(
        self,
        event: str,
        *,
        message: str = "",
        level: str = "INFO",
        reason: str | None = None,
        context: EventContext | None = None,
        fields: Mapping[str, Scalar] | None = None,
        exception: BaseException | None = None,
        critical: bool = True,
    ) -> bool:
        """Validate and enqueue one event without waiting for the writer."""
        with self._state_lock:
            if self._closed:
                raise LogFailure("logger is closed")
            failure = self._failure
            if failure is not None:
                raise LogFailure(failure)
            if self._writer_done.is_set():
                raise LogFailure("logger writer has exited")
        prepared = prepare_event(
            run_id=self._run_id,
            component=self._component,
            instance_id=self._instance_id,
            event=event,
            message=message,
            level=level,
            reason=reason,
            context=context,
            fields=fields,
            exception=exception,
            critical=critical,
            event_limit=self._settings.max_event_bytes,
            console_enabled=self._settings.console,
        )
        with self._submit_lock:
            with self._state_lock:
                if self._closed:
                    raise LogFailure("logger is closed")
                failure = self._failure
                if failure is not None:
                    raise LogFailure(failure)
                if self._writer_done.is_set():
                    raise LogFailure("logger writer has exited")
            sequence = self._next_seq
            encoded = prepared.render(
                event_seq=sequence,
                wall_time_utc=_utc_now(),
                elapsed_ns=time.monotonic_ns() - self._started_ns,
            )
            self._next_seq += 1
            item = _QueueItem(
                body=encoded.body,
                critical=encoded.critical,
                console=encoded.console,
            )
            # Admission and close share this short, I/O-free critical section.
            # A producer still rendering when close begins must not enqueue
            # after the writer has drained its final item.
            with self._state_lock:
                if self._closed:
                    raise LogFailure("logger is closed")
                self._attempted += 1
                failure = self._failure
                if failure is not None:
                    raise LogFailure(failure)
                if self._writer_done.is_set():
                    raise LogFailure("logger writer has exited")
                try:
                    self._queue.put_nowait(item)
                except queue.Full:
                    if critical:
                        self._failure = f"critical event {event!r} rejected: queue full"
                        raise LogFailure(self._failure) from None
                    self._dropped_noncritical += 1
                    return False
                self._enqueued += 1
            return True

    def check_health(self) -> None:
        """Raise the first permanent logger failure, if any."""
        failure = self._get_failure()
        if failure is not None:
            raise LogFailure(failure)

    def close(self) -> None:
        """Stop accepting events, drain within the bound, and finalize summary."""
        deadline = time.monotonic() + self._settings.close_timeout_s
        # Concurrent close calls need no separate lock. The state lock owns
        # admission and terminal outcomes; no thread holds it across I/O,
        # rendering, or join. Each caller retains its own complete budget.
        with self._state_lock:
            if self._close_complete:
                return
            if self._close_failed:
                raise LogFailure(self._close_failure or "logger close failed")
            self._closed = True
            self._stop.set()
        self._thread.join(timeout=max(0.0, deadline - time.monotonic()))
        with self._state_lock:
            # Checking terminal and latching timeout must be atomic with the
            # writer's final-summary handshake; otherwise a completed summary
            # could be followed by an unreported timeout.
            if not self._writer_terminal and self._failure is None:
                if self._thread.is_alive():
                    self._failure = f"close timed out after {self._settings.close_timeout_s:g}s"
                else:
                    self._failure = "writer exited before the final summary handshake"
            failure = self._failure
            if failure is not None:
                self._close_failed = True
                self._close_failure = failure
            else:
                self._close_complete = True
        if failure is not None:
            raise LogFailure(failure)

    def _get_failure(self) -> str | None:
        with self._state_lock:
            return self._failure

    def _set_failure(self, reason: str) -> None:
        text = _bounded_reason(reason)
        with self._state_lock:
            if self._failure is None:
                self._failure = text

    def _report_failure_once(self) -> None:
        """Writer-thread-only bounded stderr fallback; never called by producers."""
        with self._state_lock:
            failure = self._failure
            if failure is None or self._failure_reported:
                return
            self._failure_reported = True
        try:
            sys.stderr.write(f"run logging failure: {failure}\n")
            sys.stderr.flush()
        except Exception:
            pass

    def _mark_terminal(self) -> None:
        with self._state_lock:
            self._writer_terminal = True

    def _run(self) -> None:
        handled = False
        try:
            self._open_next_event_file()
            self._write_summary("running")
            self._last_flush = time.monotonic()
            self._ready.set()
            while True:
                try:
                    item = self._queue.get(timeout=0.05)
                except queue.Empty:
                    self._report_failure_once()
                    if self._stop.is_set():
                        break
                    self._maybe_flush()
                    continue
                self._write_item(item)
                self._report_failure_once()
                self._maybe_flush()
            self._flush_and_sync()
            self._close_event_file()
            self._finalize_summary()
            handled = True
        except BaseException as exception:
            self._set_failure(_exception_reason("writer failed", exception))
            try:
                self._close_event_file()
            except BaseException as close_exception:
                self._set_failure(_exception_reason("event file close failed", close_exception))
            try:
                self._write_summary("failed")
            except BaseException as summary_exception:
                self._set_failure(
                    _exception_reason("failed summary write failed", summary_exception)
                )
            self._mark_terminal()
            handled = True
        finally:
            self._ready.set()
            if not handled:
                self._set_failure("writer exited without completing")
                try:
                    self._write_summary("failed")
                except BaseException:
                    pass
                self._mark_terminal()
            self._report_failure_once()
            self._writer_done.set()

    def _finalize_summary(self) -> None:
        state = "failed" if self._get_failure() is not None else "complete"
        while True:
            committed = self._write_summary(state)
            with self._state_lock:
                failure = self._failure
                if failure is None or committed == "failed":
                    self._writer_terminal = True
                    return
            # A close timeout may land after the complete rename.  Rewrite the
            # failed state before announcing the terminal handshake.
            state = "failed"

    def _open_next_event_file(self) -> None:
        if self._handle is not None:
            self._flush_and_sync()
            self._handle.close()
            self._handle = None
        name = f"events.{self._next_file_index:06d}.jsonl"
        handle = open(self._directory / name, "xb")
        self._handle = handle
        self._files.append(name)
        self._next_file_index += 1
        self._file_bytes = 0

    def _write_item(self, item: _QueueItem) -> None:
        if not isinstance(item, _QueueItem):
            raise RuntimeError("unexpected queue item")
        line = item.body + b"\n"
        if len(line) > self._settings.max_event_bytes:
            raise RuntimeError(
                f"event line {len(line)} exceeds max_event_bytes={self._settings.max_event_bytes}"
            )
        if self._total_bytes + len(line) > self._settings.max_total_bytes:
            raise _BudgetExceeded(
                f"total byte budget {self._settings.max_total_bytes} would be exceeded"
            )
        if self._file_bytes and self._file_bytes + len(line) > self._settings.max_file_bytes:
            self._open_next_event_file()
        if self._handle is None:
            raise RuntimeError("event file is not open")
        written = self._handle.write(line)
        if written is not None and written != len(line):
            raise OSError(f"short event write: {written} of {len(line)} bytes")
        self._file_bytes += len(line)
        self._total_bytes += len(line)
        with self._state_lock:
            self._written += 1
        if item.console is not None:
            try:
                sys.stderr.write(item.console + "\n")
                sys.stderr.flush()
            except Exception:
                pass

    def _maybe_flush(self) -> None:
        now = time.monotonic()
        if now - self._last_flush >= self._settings.flush_interval_s:
            self._flush_and_sync()
            self._last_flush = now

    def _flush_and_sync(self) -> None:
        if self._handle is None:
            return
        self._handle.flush()
        _sync_file(self._handle.fileno())

    def _close_event_file(self) -> None:
        handle = self._handle
        if handle is None:
            return
        self._handle = None
        handle.close()

    def _write_summary(self, state: str) -> str:
        with self._state_lock:
            failure = self._failure
            attempted = self._attempted
            enqueued = self._enqueued
            written = self._written
            dropped_noncritical = self._dropped_noncritical
            files = list(self._files)
        if state == "complete" and failure is not None:
            state = "failed"
        summary = {
            "run_id": self._run_id,
            "component": self._component,
            "instance_id": self._instance_id,
            "state": state,
            "reason": failure if state == "failed" else None,
            "attempted": attempted,
            "enqueued": enqueued,
            "written": written,
            "dropped_noncritical": dropped_noncritical,
            "files": files,
        }
        body = self._summary_body(summary)
        if state == "complete":
            self._write_summary_temp(body, sync=True)
        elif state == "failed":
            self._write_summary_temp(body, sync=False, best_effort_sync=True)
        else:
            # The initial "running" marker is informational; only "complete"
            # needs an fsync guarantee.
            self._write_summary_temp(body, sync=False)
        os.replace(self._directory / "summary.json.tmp", self._directory / "summary.json")
        return state

    @staticmethod
    def _summary_body(summary: dict[str, object]) -> bytes:
        return (
            json.dumps(
                summary,
                ensure_ascii=False,
                allow_nan=False,
                separators=(",", ":"),
            ).encode("utf-8")
            + b"\n"
        )

    def _write_summary_temp(
        self, body: bytes, *, sync: bool, best_effort_sync: bool = False
    ) -> None:
        temporary = self._directory / "summary.json.tmp"
        with open(temporary, "wb") as handle:
            handle.write(body)
            handle.flush()
            if sync:
                _sync_file(handle.fileno())
            elif best_effort_sync:
                # A failed summary is diagnostic only.  Best-effort durability
                # keeps the failure observable when fsync itself is broken.
                try:
                    _sync_file(handle.fileno())
                except Exception:
                    pass
