"""Append-only campaign ledger, budget allocation and code fingerprints."""

from __future__ import annotations

import fcntl
import hashlib
import json
import os
import shutil
import subprocess
import time
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from robot_contracts.overrides import OverrideError, apply_overrides

PURPOSES = {"pilot", "formal", "test", "evaluation"}
MODES = {"teacher", "act"}
KINDS = {"rollout", "label_replay"}
TERMINAL_EVENTS = {"FINISHED", "FAILED", "INTERRUPTED"}


class CampaignError(RuntimeError):
    pass


def now_ns() -> int:
    return time.time_ns()


def load(path: Path, overrides: tuple[str, ...] | list[str] = ()) -> dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise CampaignError(f"cannot read campaign config {path}: {error}") from error
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise CampaignError("campaign config must be an object with schema_version 1")
    if overrides:
        try:
            document = apply_overrides(document, list(overrides))
        except OverrideError as error:
            raise CampaignError(str(error)) from error
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


def tree_sha256(project_root: Path) -> str:
    digest = hashlib.sha256()
    roots = (
        "simulation",
        "runtime",
        "learning",
        "hardware",
        "scripts",
        "shared",
    )
    for root_name in roots:
        root = project_root / root_name
        if not root.is_dir():
            continue
        for path in sorted(root.rglob("*")):
            if not path.is_file() or "__pycache__" in path.parts:
                continue
            if path.suffix not in {".py", ".yaml", ".yml", ".json", ".proto", ".msg", ".srv"}:
                continue
            try:
                digest.update(path.relative_to(project_root).as_posix().encode())
                digest.update(b"\0")
                digest.update(path.read_bytes())
                digest.update(b"\0")
            except OSError as error:
                raise CampaignError(f"cannot hash source content {path}: {error}") from error
    return digest.hexdigest()


def code_fingerprint(project_root: Path, *config_paths: Path) -> dict[str, str]:
    """Identify the working tree and the recipes this attempt actually uses.

    Only the explicitly supplied campaign/runtime recipes are hashed; the
    fingerprint carries no scenario-specific path.
    """
    head = ""
    dirty = "unknown"
    try:
        head = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=project_root,
            check=True,
            capture_output=True,
            text=True,
            timeout=5,
        ).stdout.strip()
        status = subprocess.run(
            ["git", "status", "--porcelain"],
            cwd=project_root,
            check=True,
            capture_output=True,
            text=True,
            timeout=5,
        ).stdout
        dirty = "clean" if not status.strip() else "dirty"
    except (OSError, subprocess.SubprocessError):
        pass
    files: dict[str, str] = {}
    for name in config_paths:
        try:
            files[str(name)] = hashlib.sha256(name.read_bytes()).hexdigest()
        except OSError:
            files[str(name)] = ""
    return {
        "head": head,
        "worktree": dirty,
        "files": files,
        "source_tree_sha256": tree_sha256(project_root),
    }


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
    code: dict[str, str]
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


def seed_range(config: dict[str, Any], purpose: str) -> range:
    prefix = "pilot" if purpose == "pilot" else "test" if purpose == "evaluation" else purpose
    first = int(config["budget"][f"{prefix}_seed_first"])
    if purpose == "pilot" and "pilot_seed_last" in config["budget"]:
        last = int(config["budget"]["pilot_seed_last"])
    else:
        last = int(config["budget"][f"{prefix}_seed_last"])
    return range(first, last + 1)


def used_seeds(records: list[dict[str, Any]], purpose: str) -> set[int]:
    purposes = {"test", "evaluation"} if purpose in {"test", "evaluation"} else {purpose}
    return {
        int(record["seed"])
        for item in purposes
        for record in attempts(records, item)
        if int(record.get("seed", -1)) >= 0
    }


def measure(directory: Path) -> int:
    return sum(path.stat().st_size for path in directory.rglob("*") if path.is_file())


def raw_consumed(records: list[dict[str, Any]], estimated: int, root: Path | None = None) -> int:
    terminal = finished(records)
    total = 0
    for record in attempts(records):
        attempt_id = str(record["attempt_id"])
        terminal_record = terminal.get(attempt_id)
        directory = Path(str(record.get("attempt_dir", "")))
        if terminal_record is None:
            total += estimated
        elif root is not None and directory.is_dir() and directory.is_relative_to(root):
            total += measure(directory)
        else:
            total += int(terminal_record.get("raw_bytes", 0))
    if root is not None and root.is_dir():
        # Count every raw byte under the attempt root, including files not yet
        # reported by a crashed process.  This is the conservative budget.
        total = max(total, measure(root))
    return total


def check_environment(project_root: Path, config: dict[str, Any], output: Path) -> None:
    required = [project_root / str(item) for item in config["paths"].get("required_paths", [])]
    for path in required:
        if not path.exists():
            raise CampaignError(f"required path is missing: {path}")
    marker = config["paths"].get("sdk_marker")
    if marker and not (project_root / str(marker)).exists():
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
    budget = config["budget"]
    reserved = attempts(records, purpose)
    terminal = finished(records)
    if purpose == "formal" and mode != "teacher":
        raise CampaignError("formal data must come from teacher rollouts, not ACT")
    if purpose in {"test", "evaluation"} and mode != "act":
        raise CampaignError(f"{purpose} seeds are reserved for ACT inference")
    if purpose == "formal":
        qualified = sum(
            1
            for record in attempts(records, "formal")
            if (terminal.get(str(record["attempt_id"])) or {}).get("qualified")
        )
        if qualified >= int(budget["formal_target_qualified"]):
            raise CampaignError("formal qualified target is already met")
    raw_limit = int(budget["max_raw_bytes"])
    consumed = raw_consumed(records, int(budget["estimated_attempt_bytes"]), root)
    if consumed + int(budget["estimated_attempt_bytes"]) + int(budget["reserve_bytes"]) > raw_limit:
        raise CampaignError(
            f"raw-byte budget would be exceeded: consumed {consumed}, limit {raw_limit}"
        )
    if purpose == "pilot":
        return _allocate_pilot(config, records, kind, mode, requested_seed, reserved)
    if purpose in {"test", "evaluation"}:
        maximum = int(budget.get(f"{purpose}_max_attempts", budget["test_max_attempts"]))
        used_attempts = [
            record
            for record in attempts(records)
            if record.get("purpose") in {"test", "evaluation"}
        ]
        if len(used_attempts) >= maximum:
            raise CampaignError(f"{purpose} attempt budget is exhausted")
    else:
        maximum = int(budget[f"{purpose}_max_attempts"])
        if len(reserved) >= maximum:
            raise CampaignError(f"{purpose} attempt budget is exhausted")
    used = used_seeds(records, purpose)
    for seed in seed_range(config, purpose):
        if seed not in used:
            if requested_seed is not None and requested_seed != seed:
                raise CampaignError(
                    f"explicit rollout seed {requested_seed} would skip the next seed {seed}"
                )
            return seed, mode
    raise CampaignError(f"no unused {purpose} seed remains")


def _allocate_pilot(
    config: dict[str, Any],
    records: list[dict[str, Any]],
    kind: str,
    mode: str,
    requested_seed: int | None,
    reserved: list[dict[str, Any]],
) -> tuple[int, str]:
    budget = config["budget"]
    if len(reserved) >= int(budget["pilot_max_attempts"]):
        raise CampaignError("pilot attempt budget is exhausted")
    reserved_act = int(budget.get("pilot_act_reserved_attempts", 0))
    teacher_used = sum(item.get("mode", "teacher") != "act" for item in reserved)
    if mode != "act" and teacher_used >= int(budget["pilot_max_attempts"]) - reserved_act:
        raise CampaignError("teacher/replay budget must preserve the reserved ACT slots")
    if kind == "label_replay":
        count = sum(1 for record in attempts(records) if record.get("kind") == "label_replay")
        if count >= int(budget["pilot_label_replay_max"]):
            raise CampaignError("pilot label-replay budget is exhausted")
        if requested_seed is None:
            raise CampaignError("label replay needs --seed of an existing teacher attempt")
        if requested_seed not in used_seeds(records, "pilot"):
            raise CampaignError("label replay seed was never reserved")
        return requested_seed, "teacher"
    first = int(budget["pilot_seed_first"])
    teacher_last = int(budget["pilot_teacher_seed_last"])
    act_first = int(budget["pilot_teacher_seed_last"]) + 1
    act_last = int(budget["pilot_seed_last"])
    start, end = (act_first, act_last) if mode == "act" else (first, teacher_last)
    used = used_seeds(records, "pilot")
    for seed in range(start, end + 1):
        if seed not in used:
            if requested_seed is not None and requested_seed != seed:
                raise CampaignError(
                    f"explicit rollout seed {requested_seed} would skip the next seed {seed}"
                )
            return seed, mode
    raise CampaignError(f"no unused {mode} pilot seed remains")


def summarize(
    records: list[dict[str, Any]], config: dict[str, Any], root: Path | None = None
) -> dict[str, Any]:
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
    summary["raw_bytes"] = raw_consumed(
        records, int(config["budget"]["estimated_attempt_bytes"]), root
    )
    summary["raw_limit_bytes"] = int(config["budget"]["max_raw_bytes"])
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
    "code_fingerprint",
    "finished",
    "load",
    "measure",
    "now_ns",
    "raw_consumed",
    "read_records",
    "safe_component",
    "seed_range",
    "summarize",
    "tree_sha256",
    "used_seeds",
]
