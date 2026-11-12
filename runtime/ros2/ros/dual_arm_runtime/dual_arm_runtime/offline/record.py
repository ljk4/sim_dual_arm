#!/usr/bin/env python3
"""Record new qualified teacher episodes, with a consecutive-failure limit."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

from dual_arm_runtime.offline import campaign
from dual_arm_runtime.offline.campaign_ledger import CampaignError, Ledger, attempts, finished, load
from dual_arm_runtime.offline.collect_attempt import check_ros_sdk
from dual_arm_runtime.offline.scenario_selection import select_collection

ROOT = Path(os.environ.get("PROJECT_ROOT") or Path(__file__).resolve().parents[6])
DEFAULT_CAMPAIGN = (
    "shared/scenarios/panda_stack/configs/runtime/experiments/stack_campaign.yaml"
)
DEFAULT_RUNTIME_TEACHER = (
    "shared/scenarios/panda_stack/configs/runtime/teacher_collect.yaml"
)
DEFAULT_BACKEND_LOG = Path("/data/runs/backend/teacher-backend/log")


def positive_int(value: str) -> int:
    try:
        number = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("must be a positive integer") from error
    if number < 1:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return number


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--project-root", type=Path, default=ROOT)
    result.add_argument(
        "--count", type=positive_int, default=1,
        help="number of NEW qualified episodes to record in this invocation (default: 1)",
    )
    result.add_argument(
        "--max-consecutive-failures", type=positive_int, default=3,
        help="stop after this many consecutive unqualified attempts (default: 3)",
    )
    result.add_argument("--purpose", choices=("formal", "pilot"), default="formal")
    result.add_argument("--scenario")
    result.add_argument("--shared-root", type=Path)
    result.add_argument(
        "--binding-sha256", help="expected binding digest from the read-only preflight"
    )
    result.add_argument("--config", help="direct campaign recipe (conflicts with --scenario)")
    result.add_argument(
        "--runtime-config", help="direct runtime teacher recipe (conflicts with --scenario)"
    )
    result.add_argument("--no-previews", action="store_true", help="skip export preview PNGs/GIFs")
    result.add_argument("--worker-manifest", type=Path, help=argparse.SUPPRESS)
    result.add_argument("--target-total", type=positive_int, help=argparse.SUPPRESS)
    result.add_argument(
        "--backend-log", type=Path, default=None,
        help="log directory of the running teacher backend",
    )
    return result


def _resolve_selection(args: argparse.Namespace, cli: argparse.ArgumentParser) -> None:
    """Resolve scenario/direct inputs once, before any ledger or backend work."""
    if args.scenario is not None:
        if args.config is not None or args.runtime_config is not None:
            cli.error(
                "--scenario cannot be combined with --config or --runtime-config"
            )
        selection = select_collection(
            scenario=args.scenario,
            shared_root=args.shared_root,
            project_root=args.project_root,
            binding_sha256=args.binding_sha256,
            require_parallel=args.worker_manifest is not None,
        )
        args.config = selection["recipes"]["campaign"]["path"]
        args.runtime_config = selection["recipes"]["runtime_teacher"]["path"]
        args.selection = selection
    else:
        if args.shared_root is not None:
            cli.error("--shared-root requires --scenario")
        if args.binding_sha256 is not None:
            cli.error("--binding-sha256 requires --scenario")
        args.config = args.config or DEFAULT_CAMPAIGN
        args.runtime_config = args.runtime_config or DEFAULT_RUNTIME_TEACHER
        args.selection = None
    if args.backend_log is None:
        args.backend_log = DEFAULT_BACKEND_LOG


def _collector_command(args: argparse.Namespace) -> list[str]:
    command = [
        sys.executable,
        "-m", "dual_arm_runtime.offline.collect_attempt",
        "--campaign-config", str(args.config),
        "--backend-log", str(args.backend_log),
        *(["--no-previews"] if args.no_previews else []),
    ]
    if getattr(args, "scenario", None) is not None:
        command += ["--scenario", args.scenario]
        if getattr(args, "shared_root", None) is not None:
            command += ["--shared-root", str(args.shared_root)]
    return command


def _preflight(args: argparse.Namespace) -> int:
    if not (args.project_root / args.runtime_config).is_file():
        raise CampaignError(f"runtime config is missing: {args.runtime_config}")
    if not args.backend_log.is_dir():
        raise CampaignError(f"teacher backend log is missing: {args.backend_log}")
    return check_ros_sdk(
        args.project_root,
        scenario=getattr(args, "scenario", None),
        shared_root=getattr(args, "shared_root", None),
    )


def record(args: argparse.Namespace, ledger: Ledger) -> int:
    qualified = attempted = consecutive_failures = 0
    stop_reason, exit_code = "target_reached", 0
    try:
        while qualified < args.count:
            run_id = f"record-{time.time_ns()}"
            code = campaign.main(
                [
                    "--project-root", str(args.project_root),
                    "--config", args.config,
                    "run", "--purpose", args.purpose, "--mode", "teacher",
                    "--run-id", run_id, "--runtime-config", args.runtime_config,
                    "--", *_collector_command(args),
                ]
            )
            with ledger.locked():
                records = ledger.records()
            reservation = next(
                (item for item in attempts(records) if item.get("run_id") == run_id), None
            )
            if reservation is None:
                stop_reason, exit_code = "campaign_stopped", code or 2
                break
            attempted += 1
            result = finished(records).get(reservation["attempt_id"])
            if result is None:
                stop_reason, exit_code = "attempt_incomplete", code or 1
                break
            if code == 0 and result["event"] == "FINISHED" and result.get("qualified") is True:
                qualified += 1
                consecutive_failures = 0
            else:
                consecutive_failures += 1
            print(
                f"录制进度：合格 {qualified}/{args.count}，已尝试 {attempted} 次，"
                f"连续失败 {consecutive_failures}/{args.max_consecutive_failures}",
                flush=True,
            )
            if code == 130 or result["event"] == "INTERRUPTED":
                stop_reason, exit_code = "interrupted", 130
                break
            if consecutive_failures >= args.max_consecutive_failures:
                stop_reason, exit_code = "consecutive_failures", 1
                break
    except KeyboardInterrupt:
        stop_reason, exit_code = "interrupted", 130
    summary = {
        "event": "recording.finished", "reason": stop_reason,
        "requested": args.count, "qualified": qualified, "attempted": attempted,
        "consecutive_failures": consecutive_failures,
        "max_consecutive_failures": args.max_consecutive_failures,
    }
    if args.selection is not None:
        summary["selection"] = {
            "mode": args.selection["mode"],
            "scenario_id": args.selection["scenario_id"],
            "binding": args.selection["binding"],
        }
    print(json.dumps(summary, sort_keys=True), flush=True)
    return exit_code


def main(argv: list[str] | None = None) -> int:
    cli = parser()
    args = cli.parse_args(argv)
    if args.target_total is not None and args.worker_manifest is None:
        cli.error("--target-total is supported through record_parallel.sh")
    args.project_root = args.project_root.resolve()
    try:
        _resolve_selection(args, cli)
        args.backend_log = (args.project_root / args.backend_log).resolve()
        config = load(args.project_root / args.config)
        if args.worker_manifest is not None:
            from dual_arm_runtime.offline.record_pool import record_parallel

            return record_parallel(args, config)
        code = _preflight(args)
        if code:
            return code
        paths = config["paths"]
        ledger = Ledger(args.project_root / paths["ledger"], args.project_root / paths["lock"])
        return record(args, ledger)
    except (CampaignError, OSError) as error:
        print(json.dumps({"error": str(error)}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
