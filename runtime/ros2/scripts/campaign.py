#!/usr/bin/env python3
"""Budgeted, crash-safe stack-collection ledger.

Every complete rollout must be reserved here before a runtime process starts.
The ledger is append-only: RESERVED and RUNNING attempts count toward the
budget even when the process crashes.  This tool never resets the budget and
never silently skips a seed.  It does not run any rollout unless the supervisor
explicitly invokes ``run`` with a configured launch command.

Ledger/budget allocation lives in :mod:`campaign_ledger`; launch and
classification helpers live in :mod:`campaign_execution`.
"""

from __future__ import annotations

import argparse
import json
import signal
import subprocess
import sys
from pathlib import Path
from typing import Any

from campaign_execution import (
    acquire_run_lock,
    classify,
    handle_sigterm,
    read_json,
    reject_docker_command,
    run_command,
)
from campaign_ledger import (
    KINDS,
    MODES,
    PURPOSES,
    Attempt,
    CampaignError,
    Ledger,
)
from campaign_ledger import (
    allocate as _allocate,
)
from campaign_ledger import (
    attempts as _attempts,
)
from campaign_ledger import (
    check_environment as _check_environment,
)
from campaign_ledger import (
    code_fingerprint as _code_fingerprint,
)
from campaign_ledger import (
    finished as _finished,
)
from campaign_ledger import (
    load as _load,
)
from campaign_ledger import (
    now_ns as _now_ns,
)
from campaign_ledger import (
    safe_component as _safe_component,
)
from campaign_ledger import (
    summarize as _summarize,
)


def _reserve(
    ledger: Ledger,
    config: dict[str, Any],
    args: argparse.Namespace,
    output: Path,
    run_id: str,
    project_root: Path,
) -> Attempt:
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
                "attempt_id": attempt.attempt_id,
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
    return attempt


def _resolved_command(command: list[str], attempt: Attempt) -> list[str]:
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
    return resolved


def _execute(
    command: list[str],
    attempt: Attempt,
    config: dict[str, Any],
    project_root: Path,
    output: Path,
) -> tuple[int, float, str]:
    try:
        code, wall_s, failure = run_command(
            command,
            attempt,
            project_root,
            float(config["launch"].get("timeout_s", 180.0)),
            int(config["budget"]["min_free_bytes"]),
            output,
        )
    except KeyboardInterrupt:
        code, wall_s, failure = 130, 0.0, "interrupted"
    if code not in {124, 125, 130}:
        return code, wall_s, failure
    cleanup = list(config["launch"].get("cleanup_command") or [])
    if cleanup:
        reject_docker_command(cleanup, "cleanup command")
        try:
            subprocess.run(cleanup, cwd=project_root, timeout=30.0, check=False)
        except (OSError, subprocess.SubprocessError) as error:
            failure = f"{failure}; cleanup failed: {error}"
    return code, wall_s, failure


def _finish_attempt(
    ledger: Ledger,
    attempt: Attempt,
    config: dict[str, Any],
    code: int,
    wall_s: float,
    failure: str,
) -> tuple[str, dict[str, Any], int]:
    classification = classify(attempt.directory, str(config["quality"]["sidecar"]))
    launch = read_json(attempt.directory / "launch_result.json") or {}
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
                "attempt_id": attempt.attempt_id,
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
    return event, classification, raw_bytes


def _measure(directory: Path) -> int:
    from campaign_ledger import measure

    return measure(directory)


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
    reject_docker_command(command, "launch command")
    if args.kind == "label_replay" and args.purpose != "pilot":
        raise CampaignError("label replay is a pilot attempt, not formal/test")
    run_id = _safe_component(args.run_id or f"run-{_now_ns()}", "run_id")
    acquire_run_lock(run_lock)
    previous_sigterm = signal.signal(signal.SIGTERM, handle_sigterm)
    try:
        attempt = _reserve(ledger, config, args, output, run_id, project_root)
        with ledger.locked():
            ledger.append(
                {
                    "event": "RUNNING",
                    "attempt_id": attempt.attempt_id,
                    "started_at_ns": _now_ns(),
                }
            )
        resolved = _resolved_command(command, attempt)
        code, wall_s, failure = _execute(resolved, attempt, config, project_root, output)
        event, classification, raw_bytes = _finish_attempt(
            ledger, attempt, config, code, wall_s, failure
        )
        print(
            json.dumps(
                {
                    "attempt_id": attempt.attempt_id,
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
    terminal = _finished(records)
    ordered = []
    for record in _attempts(records, "formal"):
        terminal_record = terminal.get(str(record["attempt_id"]))
        if (
            terminal_record is not None
            and terminal_record.get("qualified")
            and terminal_record.get("event") == "FINISHED"
        ):
            ordered.append(
                {
                    "attempt_id": record["attempt_id"],
                    "seed": record["seed"],
                    "attempt_dir": record["attempt_dir"],
                    "raw_bytes": terminal_record.get("raw_bytes", 0),
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
