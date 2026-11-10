#!/usr/bin/env python3
"""Budgeted, crash-safe stack-collection ledger.

Every complete rollout must be reserved here before a runtime process starts.
The ledger is append-only: RESERVED and RUNNING attempts count toward the
budget even when the process crashes.  This tool never resets the budget and
never silently skips a seed.  It does not run any rollout unless the supervisor
explicitly invokes ``run`` with a configured launch command.
"""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

PURPOSES = {"pilot", "formal", "test", "evaluation"}
MODES = {"teacher", "act"}
KINDS = {"rollout", "label_replay"}
TERMINAL_EVENTS = {"FINISHED", "FAILED", "INTERRUPTED"}


class CampaignError(RuntimeError):
    pass


def _now_ns() -> int:
    return time.time_ns()


def _load(path: Path) -> dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise CampaignError(f"cannot read campaign config {path}: {error}") from error
    if not isinstance(document, dict) or document.get("schema_version") != 1:
        raise CampaignError("campaign config must be an object with schema_version 1")
    return document


def _safe_component(value: str, name: str) -> str:
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


def _tree_sha256(project_root: Path) -> str:
    digest = hashlib.sha256()
    roots = (
        "simulation",
        "runtime",
        "learning",
        "infrastructure",
        "hardware",
        "contracts",
        "libraries",
        "profiles",
        "scripts",
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


def _code_fingerprint(project_root: Path, config_path: Path) -> dict[str, str]:
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
    for name in (
        config_path,
        project_root / "runtime/ros2/configs/experiments/stack_campaign.yaml",
        project_root / "simulation/mujoco/configs/scene/panda_stack_task.yaml",
    ):
        try:
            files[str(name)] = hashlib.sha256(name.read_bytes()).hexdigest()
        except OSError:
            files[str(name)] = ""
    return {
        "head": head,
        "worktree": dirty,
        "files": files,
        "source_tree_sha256": _tree_sha256(project_root),
    }


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
        if not self.path.exists():
            return []
        result = []
        with self.path.open("r", encoding="utf-8") as stream:
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


def _attempts(records: list[dict[str, Any]], purpose: str | None = None) -> list[dict[str, Any]]:
    return [
        record
        for record in records
        if record.get("event") == "RESERVED"
        and (purpose is None or record.get("purpose") == purpose)
    ]


def _finished(records: list[dict[str, Any]]) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for record in records:
        if record.get("event") in TERMINAL_EVENTS:
            result[str(record.get("attempt_id"))] = record
    return result


def _seed_range(config: dict[str, Any], purpose: str) -> range:
    prefix = "pilot" if purpose == "pilot" else "test" if purpose == "evaluation" else purpose
    first = int(config["budget"][f"{prefix}_seed_first"])
    if purpose == "pilot" and "pilot_seed_last" in config["budget"]:
        last = int(config["budget"]["pilot_seed_last"])
    else:
        last = int(config["budget"][f"{prefix}_seed_last"])
    return range(first, last + 1)


def _used_seeds(records: list[dict[str, Any]], purpose: str) -> set[int]:
    purposes = {"test", "evaluation"} if purpose in {"test", "evaluation"} else {purpose}
    return {
        int(record["seed"])
        for item in purposes
        for record in _attempts(records, item)
        if int(record.get("seed", -1)) >= 0
    }


def _raw_consumed(records: list[dict[str, Any]], estimated: int, root: Path | None = None) -> int:
    finished = _finished(records)
    total = 0
    for record in _attempts(records):
        attempt_id = str(record["attempt_id"])
        terminal = finished.get(attempt_id)
        directory = Path(str(record.get("attempt_dir", "")))
        if terminal is None:
            total += estimated
        elif root is not None and directory.is_dir() and directory.is_relative_to(root):
            total += _measure(directory)
        else:
            total += int(terminal.get("raw_bytes", 0))
    if root is not None and root.is_dir():
        # Count every raw byte under the attempt root, including files not yet
        # reported by a crashed process.  This is the conservative budget.
        total = max(total, _measure(root))
    return total


def _check_environment(project_root: Path, config: dict[str, Any], output: Path) -> None:
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


def _allocate(
    config: dict[str, Any],
    records: list[dict[str, Any]],
    purpose: str,
    kind: str,
    mode: str,
    requested_seed: int | None,
    root: Path,
) -> tuple[int, str]:
    budget = config["budget"]
    attempts = _attempts(records, purpose)
    finished = _finished(records)
    if purpose == "formal" and mode != "teacher":
        raise CampaignError("formal data must come from teacher rollouts, not ACT")
    if purpose in {"test", "evaluation"} and mode != "act":
        raise CampaignError(f"{purpose} seeds are reserved for ACT inference")
    if purpose == "formal":
        qualified = sum(
            1
            for record in _attempts(records, "formal")
            if (finished.get(str(record["attempt_id"])) or {}).get("qualified")
        )
        if qualified >= int(budget["formal_target_qualified"]):
            raise CampaignError("formal qualified target is already met")
    raw_limit = int(budget["max_raw_bytes"])
    consumed = _raw_consumed(records, int(budget["estimated_attempt_bytes"]), root)
    if consumed + int(budget["estimated_attempt_bytes"]) + int(budget["reserve_bytes"]) > raw_limit:
        raise CampaignError(
            f"raw-byte budget would be exceeded: consumed {consumed}, limit {raw_limit}"
        )
    if purpose == "pilot":
        if len(attempts) >= int(budget["pilot_max_attempts"]):
            raise CampaignError("pilot attempt budget is exhausted")
        reserved_act = int(budget.get("pilot_act_reserved_attempts", 0))
        teacher_used = sum(item.get("mode", "teacher") != "act" for item in attempts)
        if mode != "act" and teacher_used >= int(budget["pilot_max_attempts"]) - reserved_act:
            raise CampaignError("teacher/replay budget must preserve the reserved ACT slots")
        if kind == "label_replay":
            count = sum(1 for record in _attempts(records) if record.get("kind") == "label_replay")
            if count >= int(budget["pilot_label_replay_max"]):
                raise CampaignError("pilot label-replay budget is exhausted")
            if requested_seed is None:
                raise CampaignError("label replay needs --seed of an existing teacher attempt")
            if requested_seed not in _used_seeds(records, "pilot"):
                raise CampaignError("label replay seed was never reserved")
            return requested_seed, "teacher"
        first = int(budget["pilot_seed_first"])
        teacher_last = int(budget["pilot_teacher_seed_last"])
        act_first = int(budget["pilot_teacher_seed_last"]) + 1
        act_last = int(budget["pilot_seed_last"])
        start, end = (act_first, act_last) if mode == "act" else (first, teacher_last)
        used = _used_seeds(records, "pilot")
        for seed in range(start, end + 1):
            if seed not in used:
                if requested_seed is not None and requested_seed != seed:
                    raise CampaignError(
                        f"explicit rollout seed {requested_seed} would skip the next seed {seed}"
                    )
                return seed, mode
        raise CampaignError(f"no unused {mode} pilot seed remains")
    if purpose in {"test", "evaluation"}:
        maximum = int(budget.get(f"{purpose}_max_attempts", budget["test_max_attempts"]))
        used_attempts = [
            record
            for record in _attempts(records)
            if record.get("purpose") in {"test", "evaluation"}
        ]
        if len(used_attempts) >= maximum:
            raise CampaignError(f"{purpose} attempt budget is exhausted")
    else:
        maximum = int(budget[f"{purpose}_max_attempts"])
        if len(attempts) >= maximum:
            raise CampaignError(f"{purpose} attempt budget is exhausted")
    used = _used_seeds(records, purpose)
    for seed in _seed_range(config, purpose):
        if seed not in used:
            if requested_seed is not None and requested_seed != seed:
                raise CampaignError(
                    f"explicit rollout seed {requested_seed} would skip the next seed {seed}"
                )
            return seed, mode
    raise CampaignError(f"no unused {purpose} seed remains")


def _summarize(
    records: list[dict[str, Any]], config: dict[str, Any], root: Path | None = None
) -> dict[str, Any]:
    finished = _finished(records)
    summary: dict[str, Any] = {"purposes": {}, "raw_bytes": 0, "ledger_records": len(records)}
    for purpose in sorted(PURPOSES):
        attempts = _attempts(records, purpose)
        terminal = [finished.get(str(item["attempt_id"])) for item in attempts]
        qualified = sum(1 for item in terminal if item and item.get("qualified"))
        summary["purposes"][purpose] = {
            "reserved_attempts": len(attempts),
            "terminal_attempts": sum(1 for item in terminal if item is not None),
            "running_or_crashed": sum(1 for item in terminal if item is None),
            "qualified": qualified,
        }
    summary["raw_bytes"] = _raw_consumed(
        records, int(config["budget"]["estimated_attempt_bytes"]), root
    )
    summary["raw_limit_bytes"] = int(config["budget"]["max_raw_bytes"])
    return summary


def _measure(directory: Path) -> int:
    return sum(path.stat().st_size for path in directory.rglob("*") if path.is_file())


def _read_json(path: Path) -> dict[str, Any] | None:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def _classify(directory: Path, quality_sidecar: str) -> dict[str, Any]:
    manifest = _read_json(directory / "manifest.json")
    task = _read_json(directory / "task_result.json")
    quality = _read_json(directory / quality_sidecar)
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


def _run_command(
    command: list[str],
    attempt: Attempt,
    project_root: Path,
    timeout_s: float,
    min_free_bytes: int,
    disk_root: Path,
) -> tuple[int, float, str]:
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
    started = time.monotonic()
    deadline = started + timeout_s
    try:
        process = subprocess.Popen(
            command, cwd=project_root, env=environment, start_new_session=True
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
            free = shutil.disk_usage(disk_root).free
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


def _lock_owner_alive(path: Path) -> bool:
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


def _acquire_run_lock(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    for _attempt in range(2):
        try:
            descriptor = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o644)
        except FileExistsError:
            if _lock_owner_alive(path):
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


def _handle_sigterm(signum: int, frame: object) -> None:
    del signum, frame
    raise KeyboardInterrupt


def _reject_docker_command(command: list[str], label: str) -> None:
    """The campaign may launch business CLIs, never a host Docker wrapper."""
    forbidden = {"docker", "podman", "docker-compose", "env.sh"}
    for part in command:
        if Path(str(part)).name in forbidden:
            raise CampaignError(
                f"{label} must not call Docker or the host environment entry: {part}"
            )


def command_run(args: argparse.Namespace) -> int:
    project_root = args.project_root.resolve()
    config = _load(project_root / args.config)
    paths = config["paths"]
    attempts_root = args.attempts_root or str(config["paths"]["attempts_root"])
    output = (project_root / attempts_root).resolve()
    ledger = Ledger(project_root / paths["ledger"], project_root / paths["lock"])
    run_lock = project_root / paths["run_lock"]
    _check_environment(project_root, config, output)
    command = list(args.launch_command or config["launch"].get("command") or [])
    if command and command[0] == "--":
        command = command[1:]
    # Seed/run identity is only known after allocation; substitute after reserve.
    if not command:
        raise CampaignError(
            "no launch command: pass it after -- or configure launch.command; no seed was consumed"
        )
    _reject_docker_command(command, "launch command")
    if args.kind == "label_replay" and args.purpose != "pilot":
        raise CampaignError("label replay is a pilot attempt, not formal/test")
    run_id = _safe_component(args.run_id or f"run-{_now_ns()}", "run_id")
    _acquire_run_lock(run_lock)
    previous_sigterm = signal.signal(signal.SIGTERM, _handle_sigterm)
    attempt_id = ""
    try:
        with ledger.locked():
            records = ledger.records()
            seed, mode = _allocate(
                config, records, args.purpose, args.kind, args.mode, args.seed, output
            )
            if args.kind == "label_replay" and args.mode != "teacher":
                raise CampaignError("label replay is a teacher-mode replay")
            episode_id = _safe_component(
                args.episode_id or f"{args.purpose}-{seed:05d}-{run_id}", "episode_id"
            )
            attempt_id = f"{args.purpose}:{seed}:{run_id}"
            if any(str(item["attempt_id"]) == attempt_id for item in _attempts(records)):
                raise CampaignError(f"attempt already exists: {attempt_id}")
            directory = output / f"{args.purpose}-{seed:05d}-{run_id}"
            attempt = Attempt(
                attempt_id=attempt_id,
                purpose=args.purpose,
                kind=args.kind,
                mode=mode,
                seed=seed,
                run_id=run_id,
                episode_id=episode_id,
                directory=directory,
                config=args.runtime_config,
                code=_code_fingerprint(project_root, project_root / args.runtime_config),
                budget={
                    "max_raw_bytes": int(config["budget"]["max_raw_bytes"]),
                    "estimated_attempt_bytes": int(config["budget"]["estimated_attempt_bytes"]),
                    "reserve_bytes": int(config["budget"]["reserve_bytes"]),
                    "purpose_attempt_limit": int(config["budget"][f"{args.purpose}_max_attempts"]),
                },
                reserved_at_ns=_now_ns(),
            )
            directory.mkdir(parents=True, exist_ok=False)
            ledger.append(
                {
                    "event": "RESERVED",
                    "attempt_id": attempt_id,
                    "purpose": attempt.purpose,
                    "kind": attempt.kind,
                    "mode": attempt.mode,
                    "seed": attempt.seed,
                    "run_id": attempt.run_id,
                    "episode_id": attempt.episode_id,
                    "attempt_dir": str(directory),
                    "config": attempt.config,
                    "code": attempt.code,
                    "budget": attempt.budget,
                    "reserved_at_ns": attempt.reserved_at_ns,
                }
            )
        with ledger.locked():
            ledger.append(
                {
                    "event": "RUNNING",
                    "attempt_id": attempt_id,
                    "started_at_ns": _now_ns(),
                }
            )
        substitutions = {
            "{seed}": str(attempt.seed),
            "{run_id}": attempt.run_id,
            "{episode_id}": attempt.episode_id,
            "{attempt_dir}": str(attempt.directory),
            "{config}": attempt.config,
            "{project_python}": sys.executable,
        }
        resolved = list(command)
        for index, part in enumerate(resolved):
            for placeholder, value in substitutions.items():
                part = part.replace(placeholder, value)
            resolved[index] = part
        try:
            code, wall_s, failure = _run_command(
                resolved,
                attempt,
                project_root,
                float(config["launch"].get("timeout_s", 180.0)),
                int(config["budget"]["min_free_bytes"]),
                output,
            )
        except KeyboardInterrupt:
            code, wall_s, failure = 130, 0.0, "interrupted"
        if code in {124, 125, 130}:
            cleanup = list(config["launch"].get("cleanup_command") or [])
            if cleanup:
                _reject_docker_command(cleanup, "cleanup command")
                try:
                    subprocess.run(
                        cleanup,
                        cwd=project_root,
                        timeout=30.0,
                        check=False,
                    )
                except (OSError, subprocess.SubprocessError) as error:
                    failure = f"{failure}; cleanup failed: {error}"
        classification = _classify(attempt.directory, str(config["quality"]["sidecar"]))
        launch = _read_json(attempt.directory / "launch_result.json") or {}
        if isinstance(launch.get("act"), dict):
            classification["act"] = launch["act"]
        if isinstance(launch.get("act_metrics"), dict):
            classification["act_metrics"] = launch["act_metrics"]
        classification["launch_ok"] = code == 0
        classification["qualified"] = classification["qualified"] and code == 0
        raw_bytes = _measure(attempt.directory)
        event = "FINISHED" if code == 0 else "FAILED"
        if code == 130:
            event = "INTERRUPTED"
        with ledger.locked():
            ledger.append(
                {
                    "event": event,
                    "attempt_id": attempt_id,
                    "seed": attempt.seed,
                    "purpose": attempt.purpose,
                    "kind": attempt.kind,
                    "mode": attempt.mode,
                    "exit_code": code,
                    "wall_s": round(wall_s, 6),
                    "failure": failure,
                    "raw_bytes": raw_bytes,
                    "attempt_dir": str(attempt.directory),
                    "finished_at_ns": _now_ns(),
                    **classification,
                }
            )
        print(
            json.dumps(
                {
                    "attempt_id": attempt_id,
                    "seed": attempt.seed,
                    "event": event,
                    "exit_code": code,
                    "wall_s": round(wall_s, 3),
                    "raw_bytes": raw_bytes,
                    **classification,
                },
                sort_keys=True,
            )
        )
        return 1 if code else 0
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm)
        try:
            run_lock.unlink()
        except FileNotFoundError:
            pass


def command_status(args: argparse.Namespace) -> int:
    project_root = args.project_root.resolve()
    config = _load(project_root / args.config)
    paths = config["paths"]
    ledger = Ledger(project_root / paths["ledger"], project_root / paths["lock"])
    output = (project_root / str(config["paths"]["attempts_root"])).resolve()
    print(json.dumps(_summarize(ledger.records(), config, output), indent=2, sort_keys=True))
    return 0


def command_select(args: argparse.Namespace) -> int:
    project_root = args.project_root.resolve()
    config = _load(project_root / args.config)
    paths = config["paths"]
    ledger = Ledger(project_root / paths["ledger"], project_root / paths["lock"])
    records = ledger.records()
    finished = _finished(records)
    ordered = []
    for record in _attempts(records, "formal"):
        terminal = finished.get(str(record["attempt_id"]))
        if (
            terminal is not None
            and terminal.get("qualified")
            and terminal.get("event") == "FINISHED"
        ):
            ordered.append(
                {
                    "attempt_id": record["attempt_id"],
                    "seed": record["seed"],
                    "attempt_dir": record["attempt_dir"],
                    "raw_bytes": terminal.get("raw_bytes", 0),
                }
            )
    target = int(config["budget"]["formal_target_qualified"])
    selected = ordered[:target]
    validation_positions = {
        int(value) for value in config["budget"]["formal_validation_qualified_indices"]
    }
    if len(selected) < target:
        print(
            json.dumps(
                {
                    "status": "INSUFFICIENT_QUALIFIED",
                    "qualified": len(selected),
                    "target": target,
                    "selected": selected,
                },
                indent=2,
            )
        )
        return 1
    training = [item for index, item in enumerate(selected) if index not in validation_positions]
    validation = [item for index, item in enumerate(selected) if index in validation_positions]
    print(
        json.dumps(
            {
                "status": "SELECTED",
                "qualified": len(selected),
                "training": training,
                "validation": validation,
                "rule": "first 50 qualified formal attempts; 10th/20th/30th/40th/50th validate",
            },
            indent=2,
        )
    )
    return 0


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--project-root", type=Path, default=Path.cwd())
    result.add_argument("--config", default="runtime/ros2/configs/experiments/stack_campaign.yaml")
    result.add_argument("--attempts-root", default=None)
    sub = result.add_subparsers(dest="action", required=True)

    run = sub.add_parser("run", help="reserve one attempt, run it, and record the result")
    run.add_argument("--purpose", choices=sorted(PURPOSES), required=True)
    run.add_argument("--kind", choices=sorted(KINDS), default="rollout")
    run.add_argument("--mode", choices=sorted(MODES), default="teacher")
    run.add_argument("--seed", type=int, default=None)
    run.add_argument("--run-id", default=None)
    run.add_argument("--episode-id", default=None)
    run.add_argument("--runtime-config", default="runtime/ros2/configs/teacher_collect.yaml")
    run.add_argument("launch_command", nargs=argparse.REMAINDER)

    status = sub.add_parser("status", help="show budgets and consumed raw bytes")
    status.set_defaults()
    select = sub.add_parser("select", help="select 50 formal attempts and validation split")
    select.set_defaults()
    return result


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        if args.action == "run":
            return command_run(args)
        if args.action == "status":
            return command_status(args)
        if args.action == "select":
            return command_select(args)
        raise CampaignError(f"unknown command {args.action}")
    except CampaignError as error:
        print(json.dumps({"error": str(error)}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
