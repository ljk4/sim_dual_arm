#!/usr/bin/env python3
"""Budgeted, crash-safe collection ledger.

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
import os
import signal
import subprocess
import sys
from pathlib import Path
from typing import Any, TextIO

from robot_contracts.project_paths import project_path

from dual_arm_runtime.offline.campaign_execution import (
    LaunchInterrupted,
    acquire_run_lock,
    attempt_log_directory,
    classify,
    discard_unqualified_attempt,
    episode_manifest,
    episode_task_result,
    handle_sigterm,
    reject_docker_command,
    run_command,
)
from dual_arm_runtime.offline.campaign_ledger import (
    KINDS,
    MODES,
    PURPOSES,
    Attempt,
    CampaignError,
    Ledger,
)
from dual_arm_runtime.offline.campaign_ledger import (
    allocate as _allocate,
)
from dual_arm_runtime.offline.campaign_ledger import (
    attempts as _attempts,
)
from dual_arm_runtime.offline.campaign_ledger import (
    check_environment as _check_environment,
)
from dual_arm_runtime.offline.campaign_ledger import (
    code_fingerprint as _code_fingerprint,
)
from dual_arm_runtime.offline.campaign_ledger import (
    finished as _finished,
)
from dual_arm_runtime.offline.campaign_ledger import (
    load as _load,
)
from dual_arm_runtime.offline.campaign_ledger import (
    now_ns as _now_ns,
)
from dual_arm_runtime.offline.campaign_ledger import (
    safe_component as _safe_component,
)
from dual_arm_runtime.offline.campaign_ledger import (
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
            code=_code_fingerprint(
                project_root,
                project_path(args.config, project_root),
                project_path(args.runtime_config, project_root),
            ),
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
                "overrides": list(getattr(args, "overrides", [])),
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
    *,
    console: TextIO | None = None,
) -> tuple[int, float, str]:
    try:
        code, wall_s, failure = run_command(
            command,
            attempt,
            project_root,
            float(config["launch"].get("timeout_s", 180.0)),
            int(config["budget"]["min_free_bytes"]),
            output,
            stop_grace_s=float(config["launch"].get("stop_grace_s", 10.0)),
            console=console,
        )
    except LaunchInterrupted as interrupted:
        code, wall_s, failure = interrupted.code, interrupted.wall_s, interrupted.failure
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
    # The ledger keeps the failure reason and size; the dataset keeps only
    # qualified episodes.
    discard_unqualified_attempt(attempt.directory, classification)
    return event, classification, raw_bytes


def _measure(directory: Path) -> int:
    from dual_arm_runtime.offline.campaign_ledger import measure

    return measure(directory)


def command_run(args: argparse.Namespace) -> int:
    project_root = args.project_root.resolve()
    config = _load(project_path(args.config, project_root), args.overrides)
    paths = config["paths"]
    attempts_root = args.attempts_root or str(config["paths"]["attempts_root"])
    output = (project_path(attempts_root, project_root)).resolve()
    ledger = Ledger(
        project_path(paths["ledger"], project_root), project_path(paths["lock"], project_root)
    )
    run_lock = project_path(paths["run_lock"], project_root)
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
        log_directory = attempt_log_directory(config, project_root, attempt)
        log_directory.mkdir(parents=True, exist_ok=True)
        console = (log_directory / "console.log").open("w", encoding="utf-8")
        try:
            code, wall_s, failure = _execute(
                resolved, attempt, config, project_root, output, console=console
            )
        finally:
            console.close()
        event, classification, raw_bytes = _finish_attempt(
            ledger, attempt, config, code, wall_s, failure
        )
        runtime_reason = classification.get("runtime_reason") or ""
        reason_text = f"（{runtime_reason}）" if runtime_reason else ""
        verdict = "合格" if classification["qualified"] else "不合格"
        print(
            f"本次尝试：{attempt.attempt_id} | {event} | 退出码 {code} | "
            f"用时 {wall_s:.1f}s | 任务 "
            f"{classification.get('task_outcome') or '-'}{reason_text} | "
            f"{verdict} | 日志 {log_directory / 'console.log'}",
            flush=True,
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
    config = _load(project_path(args.config, project_root), args.overrides)
    paths = config["paths"]
    ledger = Ledger(
        project_path(paths["ledger"], project_root), project_path(paths["lock"], project_root)
    )
    output = (project_path(str(config["paths"]["attempts_root"]), project_root)).resolve()
    summary = _summarize(ledger.records(), config, output)
    summary["formal_target_qualified"] = int(config["budget"]["formal_target_qualified"])
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0


def _formal_selection(
    config: dict[str, Any], records: list[dict[str, Any]], config_name: str
) -> dict[str, Any]:
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
    validation_positions = sorted(
        {int(value) for value in config["budget"].get("formal_validation_qualified_indices", [])}
    )
    rule = str(config.get("formal_split_description") or "").strip() or (
        f"first {target} qualified formal attempts; "
        f"zero-based qualified positions {validation_positions} validate"
    )
    document: dict[str, Any] = {
        "schema_version": 1,
        "campaign": config_name,
        "diagnostic": bool(config.get("diagnostic", False)),
        "target": target,
        "validation_positions": validation_positions,
        "rule": rule,
    }
    if len(selected) < target:
        document.update(
            {
                "status": "INSUFFICIENT_QUALIFIED",
                "qualified": len(selected),
                "selected": selected,
            }
        )
        return document
    training = [item for index, item in enumerate(selected) if index not in validation_positions]
    validation = [item for index, item in enumerate(selected) if index in validation_positions]
    document.update(
        {
            "status": "SELECTED",
            "qualified": len(selected),
            "training": training,
            "validation": validation,
        }
    )
    return document


def formal_selection_path(config: dict[str, Any]) -> Path:
    """The training selection manifest belongs to the dataset directory."""
    return Path(str(config["paths"]["formal_select"]))


def write_selection(document: dict[str, Any], destination: Path) -> None:
    """Write one selection manifest atomically (``.tmp`` + ``os.replace``)."""
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.name + ".tmp")
    temporary.write_text(
        json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, destination)


def write_formal_selection(
    project_root: Path,
    config: dict[str, Any],
    records: list[dict[str, Any]],
    config_name: str,
) -> Path | None:
    """Atomically publish ``formal-select.json`` once the formal target is met."""
    document = _formal_selection(config, records, config_name)
    if document["status"] != "SELECTED":
        return None
    destination = (project_path(formal_selection_path(config), project_root)).resolve()
    write_selection(document, destination)
    return destination


def command_select(args: argparse.Namespace) -> int:
    project_root = args.project_root.resolve()
    config = _load(project_path(args.config, project_root), args.overrides)
    paths = config["paths"]
    ledger = Ledger(
        project_path(paths["ledger"], project_root), project_path(paths["lock"], project_root)
    )
    document = _formal_selection(config, ledger.records(), str(args.config))
    if args.output is not None and document["status"] == "SELECTED":
        destination = (project_path(args.output, project_root)).resolve()
        write_selection(document, destination)
        document["written"] = str(destination)
    print(json.dumps(document, indent=2))
    return 0 if document["status"] == "SELECTED" else 1


def command_backfill_terminal(args: argparse.Namespace) -> int:
    """Append the real terminal summary an interrupted launch produced.

    The original RESERVED/INTERRUPTED history is never rewritten: this appends
    one terminal record re-read from the recorded episode artefacts.  It only
    accepts an attempt that already has an INTERRUPTED/FAILED terminal fact and
    only fills facts that are still empty (or a zero wall time).
    """
    project_root = args.project_root.resolve()
    config = _load(project_path(args.config, project_root), args.overrides)
    paths = config["paths"]
    ledger = Ledger(
        project_path(paths["ledger"], project_root), project_path(paths["lock"], project_root)
    )
    with ledger.locked():
        records = ledger.records()
        terminal = _finished(records)
        reservation = next(
            (
                record
                for record in _attempts(records)
                if str(record.get("attempt_id")) == args.attempt_id
            ),
            None,
        )
        previous = terminal.get(args.attempt_id)
        if reservation is None:
            raise CampaignError(f"attempt is not reserved in the ledger: {args.attempt_id}")
        if previous is None or previous.get("event") not in {"INTERRUPTED", "FAILED"}:
            raise CampaignError(
                f"terminal backfill needs an INTERRUPTED/FAILED attempt: {args.attempt_id}"
            )
        directory = Path(str(reservation["attempt_dir"]))
        classification = classify(directory, str(config["quality"]["sidecar"]))
        manifest = episode_manifest(directory)
        if manifest is None or episode_task_result(directory) is None:
            raise CampaignError(
                f"attempt has no recorded episode manifest/task result: {directory}"
            )
        wall_s = float(previous.get("wall_s") or 0.0)
        if wall_s <= 0.0:
            ended = manifest.get("ended_at_ns")
            reserved_at = reservation.get("reserved_at_ns")
            if isinstance(ended, int) and isinstance(reserved_at, int) and ended > reserved_at:
                wall_s = round((ended - reserved_at) / 1e9, 6)
        if (
            wall_s > 0.0
            and previous.get("runtime_outcome")
            and previous.get("task_outcome")
            and previous.get("recording_complete") == classification["recording_complete"]
        ):
            raise CampaignError(f"terminal summary is already complete for {args.attempt_id}")
        record = {
            "event": previous["event"],
            "attempt_id": args.attempt_id,
            "seed": reservation.get("seed"),
            "purpose": reservation.get("purpose"),
            "kind": reservation.get("kind"),
            "mode": reservation.get("mode"),
            "exit_code": previous.get("exit_code"),
            "wall_s": wall_s,
            "failure": previous.get("failure", ""),
            "raw_bytes": _measure(directory),
            "attempt_dir": str(directory),
            "finished_at_ns": previous.get("finished_at_ns", _now_ns()),
            "recorded_at_ns": _now_ns(),
            **classification,
            "backfilled_from": [
                str(path)
                for path in (
                    directory / "raw" / "manifest.json",
                    directory / "raw" / "task_result.json",
                )
                if path.is_file()
            ],
            "backfill_reason": "terminal summary re-read from recorded episode artefacts",
        }
        ledger.append(record)
    print(json.dumps(record, indent=2, sort_keys=True))
    return 0


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--project-root", type=Path, default=Path.cwd())
    result.add_argument("--robot")
    result.add_argument("--task")
    result.add_argument("--shared-root", type=Path)
    result.add_argument(
        "--config",
        default=None,
        help="direct campaign recipe (conflicts with --robot/--task)",
    )
    result.add_argument(
        "--set",
        dest="overrides",
        action="append",
        default=[],
        metavar="PATH=VALUE",
        help="override one existing campaign configuration path (repeatable)",
    )
    result.add_argument("--attempts-root", default=None)
    sub = result.add_subparsers(dest="action", required=True)
    run = sub.add_parser("run", help="reserve one attempt, run it, and record the result")
    run.add_argument("--purpose", choices=sorted(PURPOSES), required=True)
    run.add_argument("--kind", choices=sorted(KINDS), default="rollout")
    run.add_argument("--mode", choices=sorted(MODES), default="teacher")
    run.add_argument("--seed", type=int, default=None)
    run.add_argument("--run-id", default=None)
    run.add_argument("--episode-id", default=None)
    run.add_argument(
        "--runtime-config",
        default=None,
        help="direct runtime teacher recipe (conflicts with --robot/--task)",
    )
    run.add_argument("launch_command", nargs=argparse.REMAINDER)
    status = sub.add_parser("status", help="show budgets and consumed raw bytes")
    status.set_defaults()
    select = sub.add_parser("select", help="select the qualified formal split")
    select.add_argument(
        "--output",
        default=None,
        help="atomically write the generated SELECTED manifest to this path",
    )
    select.set_defaults()
    backfill = sub.add_parser(
        "backfill-terminal",
        help="append a real terminal summary for an interrupted/failed attempt",
    )
    backfill.add_argument("--attempt-id", required=True)
    return result


def _resolve_selection(args: argparse.Namespace, cli: argparse.ArgumentParser) -> None:
    if args.robot is not None:
        if args.task is None:
            cli.error("--robot requires --task")
        if args.config is not None:
            cli.error("--robot/--task cannot be combined with --config")
        if args.action == "run" and args.runtime_config is not None:
            cli.error("--robot/--task cannot be combined with --runtime-config")
        from dual_arm_runtime.offline.scenario_selection import select_collection

        selection = select_collection(
            robot=args.robot,
            task=args.task,
            shared_root=args.shared_root,
            project_root=args.project_root.resolve(),
            require_parallel=False,
        )
        args.config = selection["recipes"]["campaign"]["path"]
        if args.action == "run":
            args.runtime_config = selection["recipes"]["runtime_teacher"]["path"]
    else:
        if args.task is not None:
            cli.error("--task requires --robot")
        if args.shared_root is not None:
            cli.error("--shared-root requires --robot/--task")
        if args.config is None:
            cli.error("one of --robot/--task or --config is required")
        if args.action == "run" and args.runtime_config is None:
            cli.error("--runtime-config is required for a direct run")


def main(argv: list[str] | None = None) -> int:
    cli = parser()
    args = cli.parse_args(argv)
    try:
        _resolve_selection(args, cli)
        if args.action == "run":
            return command_run(args)
        if args.action == "status":
            return command_status(args)
        if args.action == "select":
            return command_select(args)
        if args.action == "backfill-terminal":
            return command_backfill_terminal(args)
        raise CampaignError(f"unknown command {args.action}")
    except CampaignError as error:
        print(json.dumps({"error": str(error)}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
