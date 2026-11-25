"""Append-only campaign ledger and seed/byte budget allocation."""

from __future__ import annotations

import fcntl
import json
import os
import shutil
import time
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from robot_contracts.project_paths import project_path

PURPOSES = {"pilot", "formal"}
MODES = {"teacher"}
KINDS = {"rollout"}
TERMINAL_EVENTS = {"FINISHED", "FAILED", "INTERRUPTED"}


class CampaignError(RuntimeError):
    pass


def now_ns() -> int:
    return time.time_ns()


def load(path: Path) -> dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise CampaignError(f"cannot read campaign config {path}: {error}") from error
    if not isinstance(document, dict):
        raise CampaignError("campaign config must be an object")
    return document


def safe_component(value: str, name: str) -> str:
    if (
        not value
        or len(value) > 128
        or any(
            character not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-"
            for character in value
        )
    ):
        raise CampaignError(f"{name} must be 1..128 safe ASCII characters")
    return value


def read_records(path: Path) -> list[dict[str, Any]]:
    """Read a ledger without creating files or directories (preflight-safe)."""
    if not path.exists():
        return []
    result = []
    with path.open("r", encoding="utf-8") as stream:
        for number, line in enumerate(stream, start=1):
            text = line.strip()
            if not text:
                continue
            try:
                record = json.loads(text)
            except json.JSONDecodeError as error:
                raise CampaignError(f"ledger line {number} is corrupt: {error}") from error
            if not isinstance(record, dict) or "event" not in record:
                raise CampaignError(f"ledger line {number} has no event")
            result.append(record)
    return result


class Ledger:
    """Append-only JSONL ledger with one advisory lock for allocation."""

    def __init__(self, path: Path, lock_path: Path) -> None:
        self.path = path
        self.lock_path = lock_path
        path.parent.mkdir(parents=True, exist_ok=True)
        lock_path.parent.mkdir(parents=True, exist_ok=True)

    @contextmanager
    def locked(self) -> Iterator[None]:
        descriptor = os.open(self.lock_path, os.O_CREAT | os.O_RDWR, 0o644)
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX)
            yield
        finally:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
            os.close(descriptor)

    def records(self) -> list[dict[str, Any]]:
        return read_records(self.path)

    def append(self, record: dict[str, Any]) -> None:
        payload = json.dumps(record, sort_keys=True, separators=(",", ":"), allow_nan=False)
        descriptor = os.open(self.path, os.O_APPEND | os.O_CREAT | os.O_WRONLY, 0o644)
        try:
            os.write(descriptor, (payload + "\n").encode())
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


@dataclass(frozen=True)
class Attempt:
    attempt_id: str
    purpose: str
    kind: str
    mode: str
    seed: int
    run_id: str
    episode_id: str
    directory: Path
    config: str
    budget: dict[str, int]
    reserved_at_ns: int


def attempts(records: list[dict[str, Any]], purpose: str | None = None) -> list[dict[str, Any]]:
    return [
        record
        for record in records
        if record.get("event") == "RESERVED"
        and (purpose is None or record.get("purpose") == purpose)
    ]


def finished(records: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for record in records:
        if record.get("event") in TERMINAL_EVENTS:
            result[str(record.get("attempt_id"))] = record
    return result


def used_seeds(records: list[dict[str, Any]], purpose: str | None = None) -> set[int]:
    return {
        int(record["seed"])
        for record in attempts(records, purpose)
        if int(record.get("seed", -1)) >= 0
    }


def next_seed(config: dict[str, Any], records: list[dict[str, Any]], purpose: str) -> int:
    first = int(config["budget"][f"{purpose}_seed_first"])
    if first < 0:
        raise CampaignError(f"{purpose}_seed_first must be non-negative")
    used = used_seeds(records)
    seed = first
    while seed in used:
        seed += 1
    return seed


def measure(directory: Path) -> int:
    return sum(path.stat().st_size for path in directory.rglob("*") if path.is_file())


def raw_consumed(records: list[dict[str, Any]], root: Path | None = None) -> int:
    terminal = finished(records)
    total = 0
    for record in attempts(records):
        attempt_id = str(record["attempt_id"])
        terminal_record = terminal.get(attempt_id)
        directory = Path(str(record.get("attempt_dir", "")))
        if terminal_record is None:
            continue
        if root is not None and directory.is_dir() and directory.is_relative_to(root):
            total += measure(directory)
        else:
            total += int(terminal_record.get("raw_bytes", 0))
    if root is not None and root.is_dir():
        total = max(total, measure(root))
    return total


def check_environment(project_root: Path, config: dict[str, Any], output: Path) -> None:
    required = [
        project_path(str(item), project_root) for item in config["paths"].get("required_paths", [])
    ]
    for path in required:
        if not path.exists():
            raise CampaignError(f"required path is missing: {path}")
    marker = config["paths"].get("sdk_marker")
    if marker and not (project_path(str(marker), project_root)).exists():
        raise CampaignError(f"SDK marker is missing: {marker}")
    free = shutil.disk_usage(output.parent if output.parent.exists() else project_root).free
    minimum = int(config["budget"]["min_free_bytes"])
    if free < minimum:
        raise CampaignError(f"free disk {free} is below the required {minimum} bytes")


def allocate(
    config: dict[str, Any],
    records: list[dict[str, Any]],
    purpose: str,
    kind: str,
    mode: str,
    requested_seed: int | None,
    root: Path,
) -> tuple[int, str]:
    del root
    if purpose not in PURPOSES:
        raise CampaignError(f"unsupported campaign purpose: {purpose!r}")
    if kind != "rollout":
        raise CampaignError(f"unsupported attempt kind: {kind!r}")
    if mode != "teacher":
        raise CampaignError(f"{purpose} data must come from teacher rollouts, not {mode!r}")
    seed = next_seed(config, records, purpose)
    if requested_seed is not None and requested_seed != seed:
        raise CampaignError(
            f"explicit rollout seed {requested_seed} would skip the next seed {seed}"
        )
    return seed, mode


def summarize(
    records: list[dict[str, Any]], config: dict[str, Any], root: Path | None = None
) -> dict[str, Any]:
    del config
    terminal = finished(records)
    summary: dict[str, Any] = {"purposes": {}, "raw_bytes": 0, "ledger_records": len(records)}
    for purpose in sorted(PURPOSES):
        reserved = attempts(records, purpose)
        terminal_records = [terminal.get(str(item["attempt_id"])) for item in reserved]
        qualified = sum(1 for item in terminal_records if item and item.get("qualified"))
        summary["purposes"][purpose] = {
            "reserved_attempts": len(reserved),
            "terminal_attempts": sum(1 for item in terminal_records if item is not None),
            "running_or_crashed": sum(1 for item in terminal_records if item is None),
            "qualified": qualified,
        }
    summary["raw_bytes"] = raw_consumed(records, root)
    return summary


__all__ = [
    "Attempt",
    "CampaignError",
    "KINDS",
    "Ledger",
    "MODES",
    "PURPOSES",
    "TERMINAL_EVENTS",
    "allocate",
    "attempts",
    "check_environment",
    "finished",
    "load",
    "measure",
    "next_seed",
    "now_ns",
    "raw_consumed",
    "read_records",
    "safe_component",
    "summarize",
    "used_seeds",
]
