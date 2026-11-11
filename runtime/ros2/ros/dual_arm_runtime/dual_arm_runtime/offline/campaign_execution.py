"""Attempt launch, disk guard, classification and run-lock helpers."""

from __future__ import annotations

import json
import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Any

from dual_arm_runtime.offline.campaign_ledger import Attempt, CampaignError


def read_json(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def classify(directory: Path, quality_sidecar: str) -> dict[str, Any]:
    manifest = read_json(directory / "manifest.json")
    task = read_json(directory / "task_result.json")
    quality = read_json(directory / quality_sidecar)
    recording_complete = bool(
        manifest
        and isinstance(manifest.get("recording"), dict)
        and manifest["recording"].get("complete")
    )
    runtime = manifest.get("runtime") if isinstance(manifest, dict) else None
    runtime_outcome = str(runtime.get("outcome")) if isinstance(runtime, dict) else ""
    runtime_reason = str(runtime.get("reason")) if isinstance(runtime, dict) else ""
    task_outcome = str(task.get("result", {}).get("outcome")) if isinstance(task, dict) else ""
    quality_ok = bool(quality and quality.get("qualified") is True)
    succeeded = task_outcome == "succeeded" and runtime_outcome in {"stopped", "completed"}
    qualified = bool(recording_complete and succeeded and quality_ok)
    return {
        "recording_complete": recording_complete,
        "runtime_outcome": runtime_outcome,
        "runtime_reason": runtime_reason,
        "task_outcome": task_outcome,
        "quality_qualified": quality_ok,
        "qualified": qualified,
    }


def _environment(attempt: Attempt) -> dict[str, str]:
    environment = dict(os.environ)
    environment.update(
        {
            "CAMPAIGN_ATTEMPT_ID": attempt.attempt_id,
            "CAMPAIGN_PURPOSE": attempt.purpose,
            "CAMPAIGN_KIND": attempt.kind,
            "CAMPAIGN_MODE": attempt.mode,
            "CAMPAIGN_SEED": str(attempt.seed),
            "CAMPAIGN_RUN_ID": attempt.run_id,
            "CAMPAIGN_EPISODE_ID": attempt.episode_id,
            "CAMPAIGN_ATTEMPT_DIR": str(attempt.directory),
            "CAMPAIGN_RUNTIME_CONFIG": attempt.config,
        }
    )
    return environment


def run_command(
    command: list[str],
    attempt: Attempt,
    project_root: Path,
    timeout_s: float,
    min_free_bytes: int,
    disk_root: Path,
) -> tuple[int, float, str]:
    started = time.monotonic()
    deadline = started + timeout_s
    try:
        process = subprocess.Popen(
            command, cwd=project_root, env=_environment(attempt), start_new_session=True
        )
    except OSError as error:
        return 127, time.monotonic() - started, f"launch failed: {error}"

    def stop_group(reason: str) -> tuple[int, float, str]:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait()
        return (
            130 if reason == "interrupted" else (124 if reason == "timeout" else 125),
            time.monotonic() - started,
            reason,
        )

    failure = ""
    try:
        while process.poll() is None:
            now = time.monotonic()
            if now >= deadline:
                return stop_group(f"launch timeout after {timeout_s}s")
            free = shutil_disk_free(disk_root)
            if free < min_free_bytes:
                return stop_group(f"disk free {free} below {min_free_bytes}; launch stopped")
            time.sleep(1.0)
    except KeyboardInterrupt:
        stop_group("interrupted")
        raise
    code = int(process.returncode)
    if code != 0:
        failure = f"launch exit {code}"
    return code, time.monotonic() - started, failure


def shutil_disk_free(path: Path) -> int:
    import shutil

    return shutil.disk_usage(path).free


def lock_owner_alive(path: Path) -> bool:
    try:
        pid = int(path.read_text(encoding="utf-8").strip())
    except (OSError, ValueError):
        return False
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def acquire_run_lock(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    for _attempt in range(2):
        try:
            descriptor = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
        except FileExistsError:
            if lock_owner_alive(path):
                raise CampaignError(f"another campaign attempt holds {path}") from None
            # A crashed process leaves only its RESERVED ledger fact; the stale
            # lock is not a reason to skip the next seed forever.
            try:
                path.unlink()
            except FileNotFoundError:
                pass
            continue
        os.write(descriptor, str(os.getpid()).encode())
        os.close(descriptor)
        return
    raise CampaignError(f"could not acquire campaign run lock {path}")


def handle_sigterm(signum: int, frame: object) -> None:
    del signum, frame
    raise KeyboardInterrupt


def reject_docker_command(command: list[str], label: str) -> None:
    """The campaign may launch business CLIs, never a host Docker wrapper."""
    forbidden = {"docker", "podman", "docker-compose", "env.sh"}
    for part in command:
        if Path(str(part)).name in forbidden:
            raise CampaignError(
                f"{label} must not call Docker or the host environment entry: {part}"
            )


__all__ = [
    "acquire_run_lock",
    "classify",
    "handle_sigterm",
    "lock_owner_alive",
    "read_json",
    "reject_docker_command",
    "run_command",
]
