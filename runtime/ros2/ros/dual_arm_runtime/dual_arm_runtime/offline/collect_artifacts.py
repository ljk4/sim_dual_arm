"""Bounded evidence handling for the automatic collection launcher."""

from __future__ import annotations

import hashlib
import json
import os
import shutil
import stat
import subprocess
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

MAX_MANIFEST_BYTES = 2 * 1024 * 1024
MAX_LAUNCHER_JSON_BYTES = 4 * 1024 * 1024

_SOURCE_DIRECTORIES = (
    "simulation",
    "runtime",
    "learning",
    "shared",
    "hardware",
    "scripts",
)
_SOURCE_FILES = ("pyproject.toml",)
_EXCLUDED_PARTS = frozenset(
    {
        ".git",
        ".build",
        "var",
        "__pycache__",
        ".pytest_cache",
        ".ruff_cache",
        "node_modules",
        "local",
    }
)
_EXCLUDED_SUFFIXES = frozenset({".pyc", ".pyo", ".mcap", ".log", ".tmp"})
_HASH_CHUNK = 1024 * 1024


def utc_now() -> str:
    return datetime.now(UTC).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(_HASH_CHUNK):
            digest.update(chunk)
    return digest.hexdigest()


def _fsync_directory(path: Path) -> None:
    try:
        descriptor = os.open(path, os.O_RDONLY)
    except OSError:
        return
    try:
        os.fsync(descriptor)
    except OSError:
        pass
    finally:
        os.close(descriptor)


def atomic_write_json(path: Path, document: object) -> None:
    """Replace one JSON file atomically; a failed write never leaves a partial name."""
    path.parent.mkdir(parents=True, exist_ok=True)
    body = json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n"
    temporary = path.with_name(f".{path.name}.tmp.{os.getpid()}")
    try:
        with temporary.open("wb") as stream:
            stream.write(body.encode("utf-8"))
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        _fsync_directory(path.parent)
    except BaseException:
        try:
            temporary.unlink(missing_ok=True)
        except OSError:
            pass
        raise


def write_diagnostic(path: Path, content: str, limit: int) -> dict[str, Any]:
    """Store a clearly bounded tail and record whether it was truncated."""
    encoded = content.encode("utf-8", "replace")
    original = len(encoded)
    truncated = original > limit
    if truncated:
        marker = (
            f"...[diagnostic truncated; original_bytes={original}; kept_bytes={limit}]\n"
        ).encode()
        if len(marker) < limit:
            stored = encoded[-(limit - len(marker)) :] + marker
        else:
            stored = marker[:limit]
    else:
        stored = encoded
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(stored)
    return {
        "path": str(path),
        "bytes": len(stored),
        "original_bytes": original,
        "truncated": truncated,
        "kind": "tail" if truncated else "complete",
    }


def read_json_file(path: Path, *, limit: int = MAX_MANIFEST_BYTES) -> dict[str, Any]:
    descriptor = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    with os.fdopen(descriptor, "rb") as stream:
        if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
            raise ValueError(f"{path.name} must be a regular JSON file")
        payload = stream.read(limit + 1)
    if len(payload) > limit:
        raise ValueError(f"{path.name} is larger than {limit} bytes")
    document = json.loads(payload)
    if not isinstance(document, dict):
        raise ValueError(f"{path.name} must contain one JSON object")
    return document


def source_manifest(root: Path) -> dict[str, Any]:
    """Hash an explicit source/config/profile/deploy whitelist, never var/.build/.git."""
    files: dict[str, str] = {}
    for directory in _SOURCE_DIRECTORIES:
        base = root / directory
        if not base.is_dir():
            continue
        for path in sorted(base.rglob("*")):
            if not path.is_file() or path.is_symlink():
                continue
            relative = path.relative_to(root)
            if any(part in _EXCLUDED_PARTS for part in relative.parts):
                continue
            if relative.parts[:4] == ("shared", "common", "tools", "local"):
                continue
            if path.suffix.lower() in _EXCLUDED_SUFFIXES:
                continue
            files[relative.as_posix()] = sha256_file(path)
    for name in _SOURCE_FILES:
        path = root / name
        if path.is_file() and not path.is_symlink():
            files[name] = sha256_file(path)
    lines = "".join(f"{name}\0{digest}\n" for name, digest in sorted(files.items()))
    return {
        "root": str(root),
        "sha256": hashlib.sha256(lines.encode("utf-8")).hexdigest(),
        "file_count": len(files),
        "files": files,
    }


def contracts_digest(manifest: dict[str, Any]) -> str | None:
    files = manifest.get("files")
    if not isinstance(files, dict):
        return None
    lines = "".join(
        f"{name}\0{digest}\n"
        for name, digest in sorted(files.items())
        if str(name).startswith(("contracts/", "shared/common/interfaces/"))
    )
    if not lines:
        return None
    return hashlib.sha256(lines.encode("utf-8")).hexdigest()


def move_launch_evidence(launch_dir: Path, run_dir: Path) -> dict[str, list[str]]:
    """Move launcher-owned evidence without ever replacing runtime-owned files."""
    moved: list[str] = []
    skipped: list[str] = []
    if not launch_dir.is_dir():
        return {"moved": moved, "skipped": skipped}
    for entry in sorted(launch_dir.iterdir()):
        target = run_dir / entry.name
        if os.path.lexists(target):
            skipped.append(entry.name)
            continue
        shutil.move(str(entry), str(target))
        moved.append(entry.name)
    try:
        launch_dir.rmdir()
    except OSError:
        pass
    return {"moved": moved, "skipped": skipped}


_MAX_EVENT_LINE_BYTES = 1024 * 1024


def _scan_event_file(
    path: Path,
    *,
    run_id: str,
    instance_id: str,
    previous_seq: int,
) -> tuple[int, int, list[str]]:
    """Stream one event file line by line; return (lines, last_seq, problems)."""
    problems: list[str] = []
    count = 0
    last = previous_seq
    try:
        with path.open("rb") as stream:
            while True:
                line = stream.readline(_MAX_EVENT_LINE_BYTES + 1)
                if not line:
                    break
                if len(line) > _MAX_EVENT_LINE_BYTES and not line.endswith(b"\n"):
                    problems.append("RUNTIME_LOG_LINE_TOO_LONG")
                    break
                count += 1
                try:
                    event = json.loads(line)
                except (UnicodeError, ValueError):
                    problems.append("RUNTIME_LOG_EVENT_INVALID")
                    continue
                if not isinstance(event, dict):
                    problems.append("RUNTIME_LOG_EVENT_INVALID")
                    continue
                if event.get("run_id") != run_id:
                    problems.append("RUNTIME_LOG_EVENT_RUN_MISMATCH")
                if event.get("component") != "runtime":
                    problems.append("RUNTIME_LOG_EVENT_COMPONENT_MISMATCH")
                if event.get("instance_id") != instance_id:
                    problems.append("RUNTIME_LOG_EVENT_INSTANCE_MISMATCH")
                sequence = event.get("event_seq")
                if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence <= 0:
                    problems.append("RUNTIME_LOG_EVENT_SEQUENCE_INVALID")
                elif sequence <= last:
                    problems.append("RUNTIME_LOG_EVENT_SEQUENCE")
                else:
                    last = sequence
    except OSError:
        problems.append("RUNTIME_LOG_FILE_MISSING")
    return count, last, problems


def runtime_log_evidence(run_dir: Path, run_id: str) -> tuple[dict[str, Any], list[str]]:
    """Check the one runtime summary and stream-validate every event line."""
    problems: list[str] = []
    logs_root = run_dir / "logs" / "runtime"
    if not logs_root.is_dir():
        return {"drained": False, "path": str(logs_root)}, ["RUNTIME_LOG_MISSING"]
    instances = sorted(path for path in logs_root.iterdir() if path.is_dir())
    if len(instances) != 1:
        return (
            {"drained": False, "path": str(logs_root), "instances": len(instances)},
            ["RUNTIME_LOG_AMBIGUOUS"],
        )
    directory = instances[0]
    summary_path = directory / "summary.json"
    evidence: dict[str, Any] = {
        "drained": False,
        "instance_id": directory.name,
        "path": str(directory),
        "summary_path": str(summary_path),
    }
    try:
        summary = read_json_file(summary_path, limit=MAX_MANIFEST_BYTES)
    except (OSError, ValueError) as error:
        return evidence, [f"RUNTIME_LOG_SUMMARY_INVALID: {error}"]
    evidence["summary"] = summary
    if summary.get("run_id") != run_id:
        problems.append("RUNTIME_LOG_RUN_MISMATCH")
    if summary.get("component") != "runtime":
        problems.append("RUNTIME_LOG_COMPONENT_MISMATCH")
    if summary.get("instance_id") != directory.name:
        problems.append("RUNTIME_LOG_INSTANCE_MISMATCH")
    if summary.get("state") != "complete":
        problems.append("RUNTIME_LOG_NOT_DRAINED")
    if summary.get("reason") is not None:
        problems.append("RUNTIME_LOG_HAS_FAILURE")
    counts = [
        summary.get(name) for name in ("attempted", "enqueued", "written", "dropped_noncritical")
    ]
    if not all(isinstance(value, int) and not isinstance(value, bool) for value in counts):
        problems.append("RUNTIME_LOG_COUNTS_INVALID")
        written = None
    else:
        attempted, enqueued, written, dropped = counts
        if not (
            attempted >= enqueued >= written >= 0
            and enqueued == written
            and attempted == enqueued + dropped
        ):
            problems.append("RUNTIME_LOG_COUNTS_INCONSISTENT")
    files = summary.get("files")
    if not isinstance(files, list) or not files:
        problems.append("RUNTIME_LOG_FILES_MISSING")
    else:
        actual_lines = 0
        last_sequence = 0
        for name in files:
            if not isinstance(name, str) or Path(name).name != name:
                problems.append("RUNTIME_LOG_FILE_INVALID")
                continue
            path = directory / name
            try:
                size = path.stat().st_size
            except OSError:
                problems.append("RUNTIME_LOG_FILE_MISSING")
                continue
            if size <= 0:
                problems.append("RUNTIME_LOG_FILE_EMPTY")
                continue
            lines, last_sequence, scan_problems = _scan_event_file(
                path,
                run_id=run_id,
                instance_id=directory.name,
                previous_seq=last_sequence,
            )
            actual_lines += lines
            problems.extend(scan_problems)
        evidence["actual_event_lines"] = actual_lines
        evidence["last_event_seq"] = last_sequence
        if isinstance(written, int) and actual_lines != written:
            problems.append("RUNTIME_LOG_LINE_COUNT_MISMATCH")
    if not problems:
        evidence["drained"] = True
    return evidence, problems


def select_episode(raw_run_dir: Path, run_id: str) -> tuple[dict[str, Any], list[str]]:
    """Select the single committed episode and minimally check its manifest identity."""
    problems: list[str] = []
    evidence: dict[str, Any] = {"path": None, "episode_id": None}
    if not raw_run_dir.is_dir():
        return evidence, ["RECORDING_DIRECTORY_MISSING"]
    episodes = sorted(
        path for path in raw_run_dir.iterdir() if path.is_dir() and path.name.startswith("episode-")
    )
    if len(episodes) != 1:
        evidence["candidates"] = [path.name for path in episodes]
        return evidence, ["RECORDING_MISSING" if not episodes else "RECORDING_AMBIGUOUS"]
    episode = episodes[0]
    evidence.update(path=str(episode), episode_id=episode.name)
    try:
        manifest = read_json_file(episode / "manifest.json")
    except (OSError, ValueError) as error:
        return evidence, [f"MANIFEST_INVALID: {error}"]
    evidence["manifest"] = manifest
    if manifest.get("schema_version") != 2:
        problems.append("MANIFEST_VERSION")
    if manifest.get("episode_id") != episode.name:
        problems.append("MANIFEST_EPISODE_MISMATCH")
    if manifest.get("run_id") != run_id:
        problems.append("MANIFEST_RUN_MISMATCH")
    if manifest.get("source") != "mock":
        problems.append("MANIFEST_SOURCE")
    if manifest.get("test_only") is not True:
        problems.append("MANIFEST_NOT_TEST_ONLY")
    if not isinstance(manifest.get("recording"), dict):
        problems.append("MANIFEST_RECORDING_MISSING")
    return evidence, problems


def audit_problems(
    summary: object,
    *,
    episode_id: str,
    min_motion_rad: float,
) -> list[str]:
    """Apply the independent-audit success gates without rewriting task labels."""
    if not isinstance(summary, dict):
        return ["AUDIT_NOT_OBJECT"]
    problems: list[str] = []
    if summary.get("schema_version") != 2:
        problems.append("AUDIT_VERSION")
    if summary.get("episode_id") != episode_id:
        problems.append("AUDIT_EPISODE_MISMATCH")
    if summary.get("source") != "mock":
        problems.append("AUDIT_SOURCE")
    if summary.get("test_only") is not True:
        problems.append("AUDIT_NOT_TEST_ONLY")
    recording = summary.get("recording")
    if not isinstance(recording, dict) or recording.get("complete") is not True:
        problems.append("AUDIT_RECORDING_INCOMPLETE")
    runtime = summary.get("runtime")
    if not isinstance(runtime, dict) or runtime.get("outcome") != "completed":
        problems.append("AUDIT_RUNTIME_NOT_COMPLETED")
    task = summary.get("task")
    if not isinstance(task, dict) or task.get("outcome") != "not_evaluated":
        problems.append("AUDIT_TASK_LABEL")
    for name in ("observations", "frames"):
        value = summary.get(name)
        if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
            problems.append(f"AUDIT_{name.upper()}_EMPTY")
    commands = summary.get("commands")
    if isinstance(commands, bool) or not isinstance(commands, int) or commands != 3:
        problems.append("AUDIT_COMMANDS")
    terminal = summary.get("terminal_events")
    if isinstance(terminal, bool) or not isinstance(terminal, int) or terminal != 3:
        problems.append("AUDIT_TERMINALS")
    if summary.get("commands_without_terminal") != 0:
        problems.append("AUDIT_COMMANDS_WITHOUT_TERMINAL")
    motion = summary.get("max_joint_motion")
    if isinstance(motion, bool) or not isinstance(motion, int | float):
        problems.append("AUDIT_MOTION_INVALID")
    elif not (float(motion) > min_motion_rad):
        problems.append("AUDIT_MOTION_BELOW_THRESHOLD")
    return problems


def parse_runtime_stdout(text: str) -> dict[str, Any] | None:
    """Return the last runtime JSON summary line, or None when absent/invalid."""
    for line in reversed(text.splitlines()):
        line = line.strip()
        if not line.startswith("{"):
            continue
        try:
            value = json.loads(line)
        except ValueError:
            continue
        if isinstance(value, dict) and "exit_code" in value:
            return value
    return None


# ---- status and launcher evidence ---------------------------------------------


def git_reference(root: Path) -> dict[str, Any]:
    def run(arguments: list[str]) -> str | None:
        try:
            completed = subprocess.run(
                ["git", "-C", str(root), *arguments],
                check=False,
                capture_output=True,
                text=True,
                timeout=3,
            )
        except (OSError, subprocess.SubprocessError):
            return None
        return completed.stdout.strip() if completed.returncode == 0 else None

    revision = run(["rev-parse", "HEAD"])
    status = run(["status", "--porcelain", "--untracked-files=normal"])
    return {
        "head": revision,
        "dirty": None if status is None else bool(status),
        "worktree_matches_head": None if status is None else status == "",
        "note": "HEAD is reference only; uncommitted files are not claimed to match it",
    }
