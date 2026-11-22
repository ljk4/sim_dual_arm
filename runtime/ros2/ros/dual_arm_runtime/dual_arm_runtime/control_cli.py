"""One-shot human client for the runtime's public episode services.

It is invoked inside the running runtime environment via
``python3 -m dual_arm_runtime.control_cli``.  It only uses the public ROS
services and the final episode manifest; control, episode and recording
business stays in the runtime.  ROS session, service calls and manifest
validation live in :mod:`control_cli_support`.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

from robot_contracts.episodes import RuntimeOutcome

from dual_arm_runtime.control_cli_support import (
    ACTIVE_PHASES,
    RAW_ROOT,
    Ack,
    CliError,
    StatusSnapshot,
    UsageError,
    _Parser,
    _RosSession,
    _RuntimeClient,
    positive_timeout,
    timeout_argument,
    validate_episode_id,
    validate_run_id,
)


def _manifest_result(
    client: _RuntimeClient,
    args: argparse.Namespace,
    deadline: float,
    expected: RuntimeOutcome,
    *,
    episode_id: str,
    terminal: StatusSnapshot,
    ack: Ack,
) -> dict[str, Any]:
    while True:
        try:
            manifest = client.read_manifest(args.run_id, episode_id)
        except CliError as error:
            if (
                error.reason == "MANIFEST_MISSING"
                and terminal.phase != "FAULT"
                and time.monotonic() < deadline
            ):
                time.sleep(0.05)
                continue
            if terminal.phase == "FAULT":
                raise CliError(
                    "RUNTIME_FAULT",
                    terminal.reason or error.detail,
                    episode_id=episode_id,
                    status=terminal.as_json(),
                    ack=ack.as_json(),
                    manifest_error=error.reason,
                ) from error
            error.fields.setdefault("episode_id", episode_id)
            error.fields.setdefault("status", terminal.as_json())
            error.fields.setdefault("ack", ack.as_json())
            raise
        break

    if terminal.phase == "FAULT":
        raise CliError(
            "RUNTIME_FAULT",
            terminal.reason or manifest["runtime"]["reason"],
            episode_id=episode_id,
            status=terminal.as_json(),
            ack=ack.as_json(),
            manifest=manifest,
        )
    if not manifest["recording"]["complete"]:
        raise CliError(
            "RECORDING_INCOMPLETE",
            manifest["recording"]["failure_reason"],
            episode_id=episode_id,
            status=terminal.as_json(),
            ack=ack.as_json(),
            manifest=manifest,
        )
    outcome = RuntimeOutcome(manifest["runtime"]["outcome"])
    if expected is RuntimeOutcome.COMPLETED and outcome is not RuntimeOutcome.COMPLETED:
        raise CliError(
            "RUNTIME_" + outcome.value.upper(),
            manifest["runtime"]["reason"],
            episode_id=episode_id,
            status=terminal.as_json(),
            ack=ack.as_json(),
            manifest=manifest,
        )
    if expected is RuntimeOutcome.STOPPED and outcome is RuntimeOutcome.COMPLETED:
        raise CliError(
            "EPISODE_ALREADY_COMPLETED",
            "episode completed before the manual end took effect",
            episode_id=episode_id,
            status=terminal.as_json(),
            ack=ack.as_json(),
            manifest=manifest,
        )
    if expected is RuntimeOutcome.STOPPED and outcome is not RuntimeOutcome.STOPPED:
        raise CliError(
            "RUNTIME_" + outcome.value.upper(),
            manifest["runtime"]["reason"],
            episode_id=episode_id,
            status=terminal.as_json(),
            ack=ack.as_json(),
            manifest=manifest,
        )
    return manifest


def _base(operation: str, args: argparse.Namespace) -> dict[str, Any]:
    return {
        "operation": operation,
        "run_id": args.run_id,
        "success": True,
        "reason": "OK",
        "audit_verified": False,
    }


def _status(
    client: _RuntimeClient,
    args: argparse.Namespace,
    deadline: float,
) -> dict[str, Any]:
    result = _base("status", args)
    try:
        client.wait_ready(deadline, require_status=True, require_services=("start", "end"))
    except CliError as error:
        error.fields.setdefault("episode_id", "")
        raise
    status = client.status
    assert status is not None
    result["status"] = status.as_json()
    result["services"] = client.services()
    result["healthy"] = status.phase != "FAULT"
    result["reason"] = "OK"
    return result


def _start(
    client: _RuntimeClient,
    args: argparse.Namespace,
    deadline: float,
) -> dict[str, Any]:
    result = _base("start", args)
    client.wait_ready(
        deadline,
        require_status=bool(args.wait),
        require_services=("start",),
    )
    ack = client.call_start(args.task_id, deadline)
    result["ack"] = ack.as_json()
    if not ack.success:
        raise CliError("START_REJECTED", ack.reason, ack=ack.as_json())
    if not ack.episode_id:
        raise CliError(
            "START_NO_EPISODE_ID", "service accepted without an episode id", ack=ack.as_json()
        )
    try:
        validate_episode_id(ack.episode_id)
    except ValueError as error:
        raise CliError(
            "START_INVALID_EPISODE_ID",
            str(error),
            ack=ack.as_json(),
        ) from error
    result["episode_id"] = ack.episode_id
    if not args.wait:
        current = client.status
        if current is not None and current.episode_id == ack.episode_id:
            result["status"] = current.as_json()
        result["reason"] = "START_ACCEPTED"
        return result
    try:
        status = client.wait_terminal(ack.episode_id, deadline)
        result["status"] = status.as_json()
        result["manifest"] = _manifest_result(
            client,
            args,
            deadline,
            RuntimeOutcome.COMPLETED,
            episode_id=ack.episode_id,
            terminal=status,
            ack=ack,
        )
    except CliError as error:
        error.fields.setdefault("episode_id", ack.episode_id)
        error.fields.setdefault("ack", ack.as_json())
        raise
    result["reason"] = "COMPLETED"
    return result


def _end(
    client: _RuntimeClient,
    args: argparse.Namespace,
    deadline: float,
) -> dict[str, Any]:
    result = _base("end", args)
    result["episode_id"] = args.episode_id
    client.wait_ready(
        deadline,
        require_status=True,
        require_services=("end",),
    )
    # The service wire carries no episode id.  Check the latest status first and
    # allow a short, bounded grace for a just-issued start to become visible.
    status = client.wait_status(deadline)
    grace = min(0.75, max(0.0, deadline - time.monotonic()))
    if status.episode_id != args.episode_id and grace > 0:
        try:
            status = client.wait_for_target(args.episode_id, time.monotonic() + grace)
        except CliError as error:
            if error.reason != "WAIT_TIMEOUT":
                raise
            status = client.status or status
    result["status"] = status.as_json()
    if status.episode_id != args.episode_id:
        raise CliError(
            "EPISODE_MISMATCH",
            "latest runtime status names a different episode",
            episode_id=args.episode_id,
            status=status.as_json(),
        )
    if status.phase not in ACTIVE_PHASES:
        raise CliError(
            "EPISODE_NOT_ACTIVE",
            f"episode is {status.phase or 'unknown'}, not active",
            episode_id=args.episode_id,
            status=status.as_json(),
        )
    try:
        ack = client.call_end(args.reason, deadline)
    except CliError as error:
        error.fields.setdefault("episode_id", args.episode_id)
        raise
    result["ack"] = ack.as_json()
    if not ack.success:
        raise CliError("END_REJECTED", ack.reason, ack=ack.as_json())
    if ack.reason != "ENDING":
        raise CliError(
            "UNEXPECTED_ACK",
            f"end service acknowledged with {ack.reason!r}",
            ack=ack.as_json(),
        )
    result["reason"] = "END_ACCEPTED"
    if not args.wait:
        return result
    try:
        terminal = client.wait_terminal(args.episode_id, deadline)
        result["status"] = terminal.as_json()
        result["manifest"] = _manifest_result(
            client,
            args,
            deadline,
            RuntimeOutcome.STOPPED,
            episode_id=args.episode_id,
            terminal=terminal,
            ack=ack,
        )
    except CliError as error:
        error.fields.setdefault("episode_id", args.episode_id)
        error.fields.setdefault("ack", ack.as_json())
        raise
    result["reason"] = "STOPPED"
    return result


def _parser() -> argparse.ArgumentParser:
    parser = _Parser(description=__doc__)
    parser.add_argument("--run-id", required=True, help="run identity to address")
    parser.add_argument(
        "--timeout",
        type=timeout_argument,
        default=30.0,
        help="shared monotonic deadline in seconds (default: 30)",
    )
    subcommands = parser.add_subparsers(dest="operation", required=True)
    subcommands.add_parser("status", help="report the latest runtime status")
    start = subcommands.add_parser("start", help="start one episode")
    start.add_argument("--task-id", required=True)
    start.add_argument("--wait", action="store_true", help="wait for completion and manifest")
    end = subcommands.add_parser("end", help="end the current episode")
    end.add_argument("--episode-id", required=True)
    end.add_argument("--reason", default="OPERATOR_STOP")
    end.add_argument("--wait", action="store_true", help="wait for the stopped manifest")
    return parser


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    return _parser().parse_args(argv)


def _operation_hint(argv: list[str]) -> str:
    for token in argv:
        if token in {"status", "start", "end"}:
            return token
    return "cli"


def _run_id_hint(argv: list[str]) -> str:
    for index, token in enumerate(argv):
        if token == "--run-id" and index + 1 < len(argv):
            return argv[index + 1]
        if token.startswith("--run-id="):
            return token.split("=", 1)[1]
    return ""


def execute(arguments: argparse.Namespace, *, raw_root: Path = RAW_ROOT) -> dict[str, Any]:
    """Execute one parsed operation and return its single JSON-compatible result."""
    try:
        timeout = positive_timeout(arguments.timeout)
        validate_run_id(arguments.run_id)
        if arguments.operation == "end":
            validate_episode_id(arguments.episode_id)
    except ValueError as error:
        raise CliError("INVALID_ARGUMENT", str(error)) from error
    deadline = time.monotonic() + timeout
    with _RosSession() as session:
        client = _RuntimeClient(session, raw_root)
        if arguments.operation == "status":
            return _status(client, arguments, deadline)
        if arguments.operation == "start":
            return _start(client, arguments, deadline)
        if arguments.operation == "end":
            return _end(client, arguments, deadline)
        raise CliError("INVALID_ARGUMENT", f"unknown operation {arguments.operation!r}")


def main(argv: list[str] | None = None, *, raw_root: Path = RAW_ROOT) -> int:
    arguments_list = list(sys.argv[1:] if argv is None else argv)
    operation = _operation_hint(arguments_list)
    arguments: argparse.Namespace | None = None
    try:
        arguments = parse_args(arguments_list)
        result = execute(arguments, raw_root=raw_root)
        code = 0 if result.get("success") is True else 1
    except UsageError as error:
        result = {
            "operation": operation,
            "run_id": _run_id_hint(arguments_list),
            "success": False,
            "reason": "INVALID_ARGUMENT",
            "detail": str(error),
            "audit_verified": False,
        }
        code = 2
    except CliError as error:
        result = {
            "operation": operation,
            "run_id": "" if arguments is None else arguments.run_id,
            "success": False,
            "reason": error.reason,
            "detail": error.detail,
            "audit_verified": False,
            **error.fields,
        }
        code = 1
    except KeyboardInterrupt:
        result = {
            "operation": operation,
            "run_id": "",
            "success": False,
            "reason": "INTERRUPTED",
            "audit_verified": False,
        }
        code = 130
    except Exception as error:
        print(f"runtime cli: {type(error).__name__}: {error}", file=sys.stderr)
        result = {
            "operation": operation,
            "run_id": "",
            "success": False,
            "reason": "INTERNAL_ERROR",
            "detail": f"{type(error).__name__}: {error}",
            "audit_verified": False,
        }
        code = 1
    print(json.dumps(result, sort_keys=True), flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
