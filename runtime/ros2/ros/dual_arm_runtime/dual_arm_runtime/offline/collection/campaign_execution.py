"""Attempt launch, disk guard, classification and run-lock helpers."""

from __future__ import annotations

import json
import os
import shutil
import signal
import subprocess
import time
from pathlib import Path
from typing import Any, TextIO

from robot_contracts.project_paths import container_data_root, logs_root, project_path

from dual_arm_runtime.offline.collection.campaign_ledger import Attempt, CampaignError


def default_backend_log(
    robot: str | None, backend: str | None = None, task: str | None = None
) -> Path:
    backend_root = logs_root(container_data_root(), robot, backend, task) / "backend"
    if backend_root.is_dir():
        candidates = [
            child / "log"
            for child in backend_root.iterdir()
            if child.is_dir()
            and (child / "log").is_dir()
            and (
                child.name.startswith("teacher-backend")
                or "-teacher" in child.name
                or "_teacher" in child.name
            )
        ]
        if candidates:
            candidates.sort(key=lambda item: item.parent.stat().st_mtime_ns, reverse=True)
            return candidates[0]
    return backend_root / "teacher-backend" / "log"


def attempt_logs_root(config: dict[str, Any], project_root: Path) -> Path:
    """Attempt runtime logs live beside the campaign ledger, never in a dataset."""
    ledger = project_path(str(config["paths"]["ledger"]), project_root)
    return ledger.parent / "attempts"


def attempt_log_directory(
    config: dict[str, Any], project_root: Path, attempt: Attempt
) -> Path:
    return attempt_logs_root(config, project_root) / attempt.directory.name


def discard_unqualified_attempt(
    directory: Path, classification: dict[str, Any]
) -> None:
    """Keep ``datasets/raw`` clean: only qualified episodes survive."""
    if not classification.get("qualified"):
        shutil.rmtree(directory, ignore_errors=True)


def read_json(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _episode_file(directory: Path, name: str) -> Path | None:
    """Return one recorded episode artefact under the attempt's ``raw/``."""
    path = directory / "raw" / name
    return path if path.is_file() else None


def episode_manifest(directory: Path) -> dict[str, Any] | None:
    """Read the manifest the runtime wrote directly under ``<attempt>/raw``."""
    path = _episode_file(directory, "manifest.json")
    return None if path is None else read_json(path)


def episode_task_result(directory: Path) -> dict[str, Any] | None:
    path = _episode_file(directory, "task_result.json")
    return None if path is None else read_json(path)


class LaunchInterrupted(KeyboardInterrupt):
    """An interrupted launch that still carries its measured stop outcome."""

    def __init__(self, code: int, wall_s: float, failure: str) -> None:
        self.code = code
        self.wall_s = wall_s
        self.failure = failure
        super().__init__(failure)


def classify(directory: Path, quality_sidecar: str) -> dict[str, Any]:
    """Classify the attempt and collect failure details before its files are removed."""
    manifest = episode_manifest(directory)
    task = episode_task_result(directory)
    quality_path = directory / quality_sidecar
    quality = read_json(quality_path)
    recording = (manifest or {}).get("recording")
    recording_complete = isinstance(recording, dict) and bool(recording.get("complete"))
    runtime = manifest.get("runtime") if isinstance(manifest, dict) else None
    runtime_outcome = str(runtime.get("outcome")) if isinstance(runtime, dict) else ""
    runtime_reason = str(runtime.get("reason")) if isinstance(runtime, dict) else ""
    task_result = (task or {}).get("result", {})
    task_outcome = str(task_result.get("outcome")) if task else ""
    quality_ok = bool(quality and quality.get("qualified") is True)
    succeeded = task_outcome == "succeeded" and runtime_outcome in {"stopped", "completed"}
    qualified = bool(recording_complete and succeeded and quality_ok)
    classification = {
        "recording_complete": recording_complete,
        "runtime_outcome": runtime_outcome,
        "runtime_reason": runtime_reason,
        "task_outcome": task_outcome,
        "quality_qualified": quality_ok,
        "qualified": qualified,
    }
    if not qualified:
        labels = read_json(quality_path.with_name("labels.json"))
        quality = quality or {}
        classification["diagnostics"] = {
            "recording_failure_reason": (
                recording.get("failure_reason", "") if isinstance(recording, dict) else ""
            ),
            "task_reason": task_result.get("reason", ""),
            "visibility_qualified": quality.get("visibility_qualified"),
            "trajectory_qualified": quality.get("trajectory_qualified"),
            "quality_failure_reasons": quality.get("failure_reasons", []),
            "label_rejections": (labels or {}).get("summary", {}).get("rejected", {}),
        }
    return classification


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


def stop_process(process: subprocess.Popen, stop_grace_s: float) -> None:
    """Allow the whole launch group to close its outputs before forcing a stop."""
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        process.wait()
        return
    deadline = time.monotonic() + stop_grace_s
    while time.monotonic() < deadline:
        process.poll()
        try:
            os.killpg(process.pid, 0)
        except ProcessLookupError:
            break
        time.sleep(0.05)
    else:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    process.wait(timeout=5)


def run_command(
    command: list[str],
    attempt: Attempt,
    project_root: Path,
    timeout_s: float,
    min_free_bytes: int,
    disk_root: Path,
    *,
    stop_grace_s: float = 10.0,
    console: TextIO | None = None,
) -> tuple[int, float, str]:
    started = time.monotonic()
    deadline = started + timeout_s
    output = subprocess.DEVNULL if console is None else console
    try:
        process = subprocess.Popen(
            command,
            cwd=project_root,
            env=_environment(attempt),
            start_new_session=True,
            stdout=output,
            stderr=subprocess.STDOUT,
        )
    except OSError as error:
        return 127, time.monotonic() - started, f"launch failed: {error}"

    def stop_group(code: int, reason: str) -> tuple[int, float, str]:
        stop_process(process, stop_grace_s)
        return code, time.monotonic() - started, reason

    failure = ""
    try:
        while process.poll() is None:
            now = time.monotonic()
            if now >= deadline:
                return stop_group(124, f"launch timeout after {timeout_s}s")
            free = shutil_disk_free(disk_root)
            if free < min_free_bytes:
                return stop_group(125, f"disk free {free} below {min_free_bytes}; launch stopped")
            time.sleep(1.0)
    except KeyboardInterrupt:
        raise LaunchInterrupted(*stop_group(130, "interrupted")) from None
    code = int(process.returncode)
    if code != 0:
        failure = f"launch exit {code}"
    return code, time.monotonic() - started, failure


def shutil_disk_free(path: Path) -> int:
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
    "LaunchInterrupted",
    "acquire_run_lock",
    "attempt_log_directory",
    "attempt_logs_root",
    "classify",
    "default_backend_log",
    "discard_unqualified_attempt",
    "episode_manifest",
    "episode_task_result",
    "handle_sigterm",
    "lock_owner_alive",
    "read_json",
    "reject_docker_command",
    "run_command",
    "stop_process",
]
