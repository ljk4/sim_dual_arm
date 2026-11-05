"""Crash-safe wall-clock budget with stable runs and unique segments.

A *training run* is the stable identity used for checkpoints and reports.  A
*budget segment* is one process invocation of that run; resuming a run appends
a new segment, never overwrites an old one.  All segments are summed.

The ledger is append-only.  A global lifecycle lock prevents two processes
from consuming the budget concurrently.  While a process holds that lock its
segments are live and status shows live elapsed time instead of freezing them
as crashed.  When the lock is free, unfinished segments are conservatively
charged once by ``ORPHAN_CHARGED`` (actual elapsed or full reservation).
"""

from __future__ import annotations

import fcntl
import json
import os
import time
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any


class BudgetError(RuntimeError):
    """A malformed ledger or a segment that cannot be honoured."""


class BudgetExceeded(BudgetError):
    """The cumulative task-training wall clock has no room for this segment."""


def _now_ns() -> int:
    return time.time_ns()


def _safe_seconds(value: object, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise BudgetError(f"{location}: must be a finite number")
    number = float(value)
    if number < 0 or number != number or number == float("inf"):
        raise BudgetError(f"{location}: must be a finite non-negative number")
    return number


def _segment_key(record: dict[str, Any]) -> str:
    return str(record.get("segment_id") or record.get("run_id") or "")


class BudgetLedger:
    """Append-only JSONL ledger guarded by ledger and lifecycle file locks."""

    def __init__(
        self,
        path: str | Path,
        lock_path: str | Path,
        limit_seconds: float,
        lifecycle_path: str | Path | None = None,
    ) -> None:
        self.path = Path(path)
        self.lock_path = Path(lock_path)
        self.lifecycle_path = (
            Path(lifecycle_path)
            if lifecycle_path is not None
            else Path(str(self.lock_path) + ".lifecycle")
        )
        self.limit_seconds = _safe_seconds(limit_seconds, "limit_seconds")
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.lock_path.parent.mkdir(parents=True, exist_ok=True)
        self.lifecycle_path.parent.mkdir(parents=True, exist_ok=True)
        self._held_lifecycle = False

    @contextmanager
    def _locked(self) -> Iterator[None]:
        descriptor = os.open(self.lock_path, os.O_CREAT | os.O_RDWR, 0o644)
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX)
            yield
        finally:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
            os.close(descriptor)

    def _try_lifecycle(self) -> int | None:
        if self._held_lifecycle:
            return None
        descriptor = os.open(self.lifecycle_path, os.O_CREAT | os.O_RDWR, 0o644)
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(descriptor)
            return None
        return descriptor

    @staticmethod
    def _release(descriptor: int) -> None:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
        finally:
            os.close(descriptor)

    def _records(self) -> list[dict[str, Any]]:
        if not self.path.exists():
            return []
        records: list[dict[str, Any]] = []
        with self.path.open("r", encoding="utf-8") as stream:
            for number, line in enumerate(stream, start=1):
                text = line.strip()
                if not text:
                    continue
                try:
                    record = json.loads(text)
                except json.JSONDecodeError as error:
                    raise BudgetError(f"budget ledger line {number} is corrupt: {error}") from error
                if not isinstance(record, dict) or not isinstance(record.get("event"), str):
                    raise BudgetError(f"budget ledger line {number} has no event")
                records.append(record)
        return records

    def _append(self, record: dict[str, Any]) -> None:
        payload = json.dumps(record, sort_keys=True, separators=(",", ":"), allow_nan=False)
        descriptor = os.open(self.path, os.O_APPEND | os.O_CREAT | os.O_WRONLY, 0o644)
        try:
            os.write(descriptor, (payload + "\n").encode("utf-8"))
            os.fsync(descriptor)
        finally:
            os.close(descriptor)

    @staticmethod
    def _segments(records: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
        segments: dict[str, dict[str, Any]] = {}
        for record in records:
            segment_id = _segment_key(record)
            run_id = str(record.get("run_id") or "")
            if not segment_id or not run_id:
                continue
            entry = segments.setdefault(
                segment_id,
                {
                    "run_id": run_id,
                    "reserved": None,
                    "started": None,
                    "terminal": None,
                    "orphan": None,
                },
            )
            event = record["event"]
            if event == "RESERVED":
                entry["reserved"] = record
            elif event == "STARTED":
                entry["started"] = record
            elif event == "FINISHED":
                entry["terminal"] = record
            elif event == "ORPHAN_CHARGED":
                entry["orphan"] = record
        return segments

    def _charge(self, entry: dict[str, Any], now_ns: int) -> float:
        terminal = entry["terminal"]
        if terminal is not None:
            return _safe_seconds(terminal.get("elapsed_seconds", 0.0), "FINISHED.elapsed_seconds")
        orphan = entry["orphan"]
        if orphan is not None:
            return _safe_seconds(
                orphan.get("charged_seconds", 0.0), "ORPHAN_CHARGED.charged_seconds"
            )
        reserved = entry["reserved"]
        if reserved is None:
            return 0.0
        reservation = _safe_seconds(
            reserved.get("reserved_seconds", 0.0), "RESERVED.reserved_seconds"
        )
        started = entry["started"]
        if started is None:
            return reservation
        started_ns = int(started.get("wall_time_ns", reserved.get("wall_time_ns", now_ns)))
        elapsed = max(0.0, (now_ns - started_ns) / 1e9)
        return max(reservation, elapsed)

    def _used_locked(self, records: list[dict[str, Any]], now_ns: int) -> float:
        return sum(self._charge(entry, now_ns) for entry in self._segments(records).values())

    def _recover_locked(self, now_ns: int) -> list[dict[str, Any]]:
        records = self._records()
        segments = self._segments(records)
        recovered: list[dict[str, Any]] = []
        for segment_id, entry in sorted(segments.items()):
            if (
                entry["terminal"] is not None
                or entry["orphan"] is not None
                or entry["reserved"] is None
            ):
                continue
            charged = self._charge(entry, now_ns)
            record = {
                "event": "ORPHAN_CHARGED",
                "run_id": entry["run_id"],
                "segment_id": segment_id,
                "charged_seconds": charged,
                "detected_wall_time_ns": now_ns,
                "reason": "unfinished segment with no lifecycle owner; conservative charge",
            }
            self._append(record)
            recovered.append(record)
        return recovered

    def _active(self) -> tuple[bool, int | None]:
        descriptor = self._try_lifecycle()
        return (False, descriptor) if descriptor is not None else (True, None)

    def recover_orphans(self) -> list[dict[str, Any]]:
        """Charge crashed segments once; never freeze a live owner."""
        descriptor = self._try_lifecycle()
        if descriptor is None:
            return []
        try:
            with self._locked():
                return self._recover_locked(_now_ns())
        finally:
            self._release(descriptor)

    def status(self) -> dict[str, Any]:
        descriptor = self._try_lifecycle()
        active = descriptor is None
        try:
            with self._locked():
                now = _now_ns()
                if not active:
                    self._recover_locked(now)
                records = self._records()
                segments = self._segments(records)
                used = self._used_locked(records, now)
                runs: dict[str, dict[str, Any]] = {}
                for segment_id, entry in sorted(segments.items()):
                    run = runs.setdefault(entry["run_id"], {"charged_seconds": 0.0, "segments": {}})
                    charge = self._charge(entry, now)
                    run["charged_seconds"] += charge
                    run["segments"][segment_id] = {
                        "reserved_seconds": (
                            None
                            if entry["reserved"] is None
                            else entry["reserved"].get("reserved_seconds")
                        ),
                        "charged_seconds": charge,
                        "outcome": (
                            None if entry["terminal"] is None else entry["terminal"].get("outcome")
                        ),
                        "orphan": entry["orphan"] is not None,
                        "active": (
                            active and entry["terminal"] is None and entry["orphan"] is None
                        ),
                    }
                return {
                    "limit_seconds": self.limit_seconds,
                    "used_seconds": used,
                    "remaining_seconds": max(0.0, self.limit_seconds - used),
                    "active": active,
                    "runs": runs,
                    "records": len(records),
                }
        finally:
            if descriptor is not None:
                self._release(descriptor)

    def begin(
        self,
        *,
        run_id: str,
        segment_id: str,
        mode: str,
        reserved_seconds: float,
        config_sha256: str,
        source: str = "",
        instance_id: str = "",
    ) -> BudgetTicket:
        reservation = _safe_seconds(reserved_seconds, "reserved_seconds")
        if not run_id or len(run_id) > 128:
            raise BudgetError("run_id must be 1..128 characters")
        if not segment_id or len(segment_id) > 160:
            raise BudgetError("segment_id must be 1..160 characters")
        descriptor = self._try_lifecycle()
        if descriptor is None:
            raise BudgetError("another training process holds the global lifecycle lock")
        self._held_lifecycle = True
        try:
            with self._locked():
                now = _now_ns()
                self._recover_locked(now)
                records = self._records()
                if any(_segment_key(record) == segment_id for record in records):
                    raise BudgetError(f"budget segment {segment_id!r} already exists")
                used = self._used_locked(records, now)
                if used + reservation > self.limit_seconds:
                    raise BudgetExceeded(
                        f"reservation {reservation:.1f}s + used {used:.1f}s exceeds "
                        f"{self.limit_seconds:.1f}s"
                    )
                common = {
                    "run_id": run_id,
                    "segment_id": segment_id,
                    "mode": mode,
                    "pid": os.getpid(),
                    "instance_id": instance_id,
                }
                self._append(
                    {
                        **common,
                        "event": "RESERVED",
                        "reserved_seconds": reservation,
                        "wall_time_ns": now,
                        "config_sha256": config_sha256,
                        "source": source,
                    }
                )
                self._append(
                    {
                        **common,
                        "event": "STARTED",
                        "wall_time_ns": now,
                        "monotonic_ns": time.monotonic_ns(),
                    }
                )
                return BudgetTicket(
                    ledger=self,
                    run_id=run_id,
                    segment_id=segment_id,
                    instance_id=instance_id,
                    mode=mode,
                    reserved_seconds=reservation,
                    used_before=used,
                    started_wall_ns=now,
                    started_monotonic_ns=time.monotonic_ns(),
                    lifecycle_descriptor=descriptor,
                )
        except BaseException:
            self._held_lifecycle = False
            self._release(descriptor)
            raise


@dataclass
class BudgetTicket:
    ledger: BudgetLedger
    run_id: str
    segment_id: str
    instance_id: str
    mode: str
    reserved_seconds: float
    used_before: float
    started_wall_ns: int
    started_monotonic_ns: int
    lifecycle_descriptor: int | None
    _closed: bool = False

    @property
    def elapsed_seconds(self) -> float:
        return max(0.0, (time.monotonic_ns() - self.started_monotonic_ns) / 1e9)

    @property
    def used_seconds(self) -> float:
        return self.used_before + self.elapsed_seconds

    @property
    def remaining_seconds(self) -> float:
        return max(0.0, self.ledger.limit_seconds - self.used_seconds)

    def check(self, stop_margin_seconds: float = 0.0) -> None:
        if self.remaining_seconds <= stop_margin_seconds:
            raise BudgetExceeded(
                f"remaining {self.remaining_seconds:.1f}s is within the "
                f"{stop_margin_seconds:.1f}s stop margin"
            )

    def finish(self, outcome: str, reason: str = "") -> dict[str, Any]:
        if self._closed:
            return {}
        self._closed = True
        elapsed = self.elapsed_seconds
        record = {
            "event": "FINISHED",
            "run_id": self.run_id,
            "segment_id": self.segment_id,
            "instance_id": self.instance_id,
            "mode": self.mode,
            "outcome": outcome,
            "reason": reason,
            "reserved_seconds": self.reserved_seconds,
            "elapsed_seconds": elapsed,
            "used_before_seconds": self.used_before,
            "ended_wall_time_ns": _now_ns(),
            "overrun_seconds": max(0.0, elapsed - self.reserved_seconds),
        }
        try:
            with self.ledger._locked():
                self.ledger._append(record)
        finally:
            if self.lifecycle_descriptor is not None:
                self.ledger._release(self.lifecycle_descriptor)
                self.lifecycle_descriptor = None
                self.ledger._held_lifecycle = False
        return record


__all__ = ["BudgetError", "BudgetExceeded", "BudgetLedger", "BudgetTicket"]
