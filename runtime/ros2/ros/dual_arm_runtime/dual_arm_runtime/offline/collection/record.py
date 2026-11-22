#!/usr/bin/env python3
"""Record new qualified teacher episodes, with a consecutive-failure limit."""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

import yaml
from robot_contracts.project_paths import project_path

from dual_arm_runtime.artifact_paths import default_backend_log
from dual_arm_runtime.offline.collection import campaign
from dual_arm_runtime.offline.collection.campaign_ledger import (
    CampaignError,
    Ledger,
    attempts,
    finished,
    load,
    summarize,
)
from dual_arm_runtime.offline.collection.collect_attempt import check_ros_sdk
from dual_arm_runtime.offline.collection.scenario_selection import select_collection

ROOT = Path(os.environ.get("PROJECT_ROOT") or Path(__file__).resolve().parents[7])


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
        "--count",
        type=positive_int,
        default=1,
        help="number of NEW qualified episodes to record in this invocation (default: 1)",
    )
    result.add_argument(
        "--max-consecutive-failures",
        type=positive_int,
        default=3,
        help="stop after this many consecutive unqualified attempts (default: 3)",
    )
    result.add_argument("--purpose", choices=("formal", "pilot"), default="formal")
    result.add_argument("--robot")
    result.add_argument("--task")
    result.add_argument("--shared-root", type=Path)
    result.add_argument(
        "--binding-sha256", help="expected binding digest from the read-only preflight"
    )
    result.add_argument("--config", help="direct campaign recipe (conflicts with --robot)")
    result.add_argument(
        "--runtime-config", help="direct runtime teacher recipe (conflicts with --robot)"
    )
    result.add_argument("--worker-manifest", type=Path, help=argparse.SUPPRESS)
    result.add_argument("--target-total", type=positive_int, help=argparse.SUPPRESS)
    result.add_argument(
        "--backend-log",
        type=Path,
        default=None,
        help="log directory of the running teacher backend",
    )
    return result


def _resolve_selection(args: argparse.Namespace, cli: argparse.ArgumentParser) -> None:
    """Resolve scenario/direct inputs once, before any ledger or backend work."""
    if args.robot is not None:
        if args.task is None:
            cli.error("--robot requires --task")
        if args.config is not None or args.runtime_config is not None:
            cli.error("--robot/--task cannot be combined with --config or --runtime-config")
        selection = select_collection(
            robot=args.robot,
            task=args.task,
            shared_root=args.shared_root,
            project_root=args.project_root,
            binding_sha256=args.binding_sha256,
            require_parallel=args.worker_manifest is not None,
        )
        args.config = selection["recipes"]["campaign"]["path"]
        args.runtime_config = selection["recipes"]["runtime_teacher"]["path"]
        args.selection = selection
    else:
        if args.task is not None:
            cli.error("--task requires --robot")
        if args.shared_root is not None:
            cli.error("--shared-root requires --robot/--task")
        if args.binding_sha256 is not None:
            cli.error("--binding-sha256 requires --robot/--task")
        if args.config is None:
            cli.error("one of --robot/--task or --config is required")
        if args.runtime_config is None:
            cli.error("--runtime-config is required with --config")
        args.selection = None
    args.config = str(project_path(args.config, args.project_root))
    args.runtime_config = str(project_path(args.runtime_config, args.project_root))
    if args.backend_log is None:
        args.backend_log = default_backend_log(args.robot, args.task)


def _collector_command(args: argparse.Namespace) -> list[str]:
    command = [
        sys.executable,
        "-m",
        "dual_arm_runtime.offline.collection.collect_attempt",
        "--campaign-config",
        str(args.config),
        "--backend-log",
        str(args.backend_log),
    ]
    if getattr(args, "robot", None) is not None:
        command += ["--robot", args.robot, "--task", args.task]
        if getattr(args, "shared_root", None) is not None:
            command += ["--shared-root", str(args.shared_root)]
        if getattr(args, "robot_namespace", None) is not None:
            command += ["--robot-namespace", args.robot_namespace]
    else:
        command += ["--runtime-config", str(args.runtime_config)]
    return command


def _preflight(args: argparse.Namespace) -> int:
    if not (project_path(args.runtime_config, args.project_root)).is_file():
        raise CampaignError(f"runtime config is missing: {args.runtime_config}")
    if not args.backend_log.is_dir():
        raise CampaignError(f"teacher backend log is missing: {args.backend_log}")
    return check_ros_sdk(
        args.project_root,
        robot=getattr(args, "robot", None),
        task=getattr(args, "task", None),
        shared_root=getattr(args, "shared_root", None),
    )


def _unique_timestamped_path(parent: Path, prefix: str, suffix: str = "") -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    candidate = parent / f"{prefix}_{stamp}{suffix}"
    if not candidate.exists():
        return candidate
    index = 2
    while (parent / f"{prefix}_{stamp}_{index}{suffix}").exists():
        index += 1
    return parent / f"{prefix}_{stamp}_{index}{suffix}"


def _next_run_id(records: list[dict], prefix: str = "record") -> str:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    existing = {item.get("run_id") for item in records if isinstance(item, dict)}
    base = f"{prefix}_{stamp}"
    if base not in existing:
        return base
    index = 2
    while f"{base}_{index}" in existing:
        index += 1
    return f"{base}_{index}"


def record(args: argparse.Namespace, config: dict, ledger: Ledger) -> int:
    qualified = attempted = consecutive_failures = 0
    stop_reason, exit_code = "target_reached", 0
    try:
        while qualified < args.count:
            with ledger.locked():
                existing_records = ledger.records()
            run_id = _next_run_id(existing_records)
            code = campaign.main(
                [
                    "--project-root",
                    str(args.project_root),
                    "--config",
                    args.config,
                    "run",
                    "--purpose",
                    args.purpose,
                    "--mode",
                    "teacher",
                    "--run-id",
                    run_id,
                    "--runtime-config",
                    args.runtime_config,
                    "--",
                    *_collector_command(args),
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
        "event": "recording.finished",
        "reason": stop_reason,
        "requested": args.count,
        "qualified": qualified,
        "attempted": attempted,
        "consecutive_failures": consecutive_failures,
        "max_consecutive_failures": args.max_consecutive_failures,
    }
    if args.selection is not None:
        summary["selection"] = {
            "mode": args.selection["mode"],
            "scenario_id": args.selection["scenario_id"],
            "binding": args.selection["binding"],
        }
    if args.purpose == "formal" and stop_reason == "target_reached":
        destination = campaign.write_formal_selection(
            args.project_root, config, ledger.records(), args.config
        )
        if destination is not None:
            summary["formal_select"] = str(destination)
    summaries_dir = project_path(config["paths"]["ledger"], args.project_root).parent / "summaries"
    summaries_dir.mkdir(parents=True, exist_ok=True)
    summary_path = _unique_timestamped_path(summaries_dir, "record", ".json")
    summary_path.write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    reason_text = {
        "target_reached": "目标达成",
        "consecutive_failures": "连续失败达到上限",
        "interrupted": "已中断",
        "campaign_stopped": "采集停止",
        "attempt_incomplete": "本次尝试未完整结束",
    }.get(stop_reason, stop_reason)
    print(
        f"采集结束：{reason_text} | 合格 {qualified}/{args.count} | "
        f"已尝试 {attempted} | 连续失败 {consecutive_failures}/"
        f"{args.max_consecutive_failures}",
        flush=True,
    )
    if "formal_select" in summary:
        print(f"合格录制清单: {summary['formal_select']}", flush=True)
    print(f"运行摘要: {summary_path}", flush=True)
    return exit_code


def _print_status(args: argparse.Namespace, config: dict, ledger: Ledger) -> None:
    paths = config["paths"]
    output = (project_path(str(paths["attempts_root"]), args.project_root)).resolve()
    summary = summarize(ledger.records(), config, output)
    target = int(config["budget"]["formal_target_qualified"])
    formal = summary["purposes"].get("formal", {})
    pilot = summary["purposes"].get("pilot", {})
    formal_qualified = int(formal.get("qualified", 0))
    remaining = max(0, target - formal_qualified)
    used_mb = summary["raw_bytes"] / (1024 * 1024)
    limit_mb = summary["raw_limit_bytes"] / (1024 * 1024)
    title = args.selection["scenario_id"] if args.selection is not None else "direct"
    print(f"=== 数据集与采集状态 ({title}) ===", flush=True)
    print(
        f"正式采集 (formal): 合格 {formal_qualified}/{target} 条（还差 {remaining} 条），"
        f"总尝试 {formal.get('reserved_attempts', 0)} 次"
        f"（已结束 {formal.get('terminal_attempts', 0)}，"
        f"进行中/异常 {formal.get('running_or_crashed', 0)}）",
        flush=True,
    )
    print(
        f"试采验证 (pilot):  合格 {pilot.get('qualified', 0)} 条，"
        f"总尝试 {pilot.get('reserved_attempts', 0)} 次（不计入正式采集）",
        flush=True,
    )
    print(f"磁盘占用:          {used_mb:.1f} MB / {limit_mb:.1f} MB", flush=True)
    formal_select_rel = paths.get("formal_select")
    if formal_select_rel:
        formal_select_path = (project_path(formal_select_rel, args.project_root)).resolve()
        if formal_select_path.is_file():
            doc = json.loads(formal_select_path.read_text(encoding="utf-8"))
            status = doc.get("status", "UNKNOWN")
            qualified = doc["qualified"]
            print(
                f"合格录制清单:      {status}（{qualified} 条） -> {formal_select_path}",
                flush=True,
            )
        else:
            print(
                f"合格录制清单:      未生成（正式合格达 {target} 条后自动生成于 "
                f"{formal_select_path}）",
                flush=True,
            )
    print("======================================", flush=True)


def _print_startup_info(args: argparse.Namespace, config: dict, ledger: Ledger) -> None:
    ledger_path = project_path(config["paths"]["ledger"], args.project_root)
    ledger_path.parent.mkdir(parents=True, exist_ok=True)
    detail_path = _unique_timestamped_path(ledger_path.parent, "startup", ".yaml")
    detail_path.write_text(
        "# 采集配方与运行参数\n"
        + yaml.safe_dump(
            {
                "selection": {
                    key: args.selection[key]
                    for key in ("mode", "scenario_id", "binding")
                    if args.selection is not None and key in args.selection
                }
                if args.selection is not None
                else None,
                "campaign_config": str(project_path(args.config, args.project_root)),
                "runtime_config": str(project_path(args.runtime_config, args.project_root)),
                "backend_log": str(args.backend_log),
                "purpose": args.purpose,
                "count": args.count,
                "max_consecutive_failures": args.max_consecutive_failures,
                "campaign": config,
            },
            sort_keys=False,
            allow_unicode=True,
        ),
        encoding="utf-8",
    )
    print("=== 采集启动 ===", flush=True)
    if args.selection is not None:
        print(f"方案: {args.selection['scenario_id']}", flush=True)
        print(f"Binding: {args.selection['binding']['path']}", flush=True)
    print(
        f"用途: {args.purpose} | 目标: 合格 {args.count} 条 | "
        f"连败上限: {args.max_consecutive_failures}",
        flush=True,
    )
    print(f"Backend 日志: {args.backend_log}", flush=True)
    print(f"详细参数: {detail_path}", flush=True)
    _print_status(args, config, ledger)


def _status_command(argv: list[str]) -> int:
    cli = argparse.ArgumentParser(
        prog="scripts/record.sh status",
        description="Show human-readable dataset and campaign status.",
    )
    cli.add_argument("--project-root", type=Path, default=ROOT)
    cli.add_argument("--robot", default=None)
    cli.add_argument("--task", default=None)
    cli.add_argument("--shared-root", type=Path)
    cli.add_argument("--config", help="direct campaign recipe (conflicts with --robot)")
    args = cli.parse_args(argv)
    args.project_root = args.project_root.resolve()
    if args.robot is None and args.config is None:
        cli.error("one of --robot/--task or --config is required")
    try:
        if args.robot is not None:
            if args.task is None:
                cli.error("--robot requires --task")
            if args.config is not None:
                cli.error("--robot/--task cannot be combined with --config")
            selection = select_collection(
                robot=args.robot,
                task=args.task,
                shared_root=args.shared_root,
                project_root=args.project_root,
                require_parallel=False,
            )
            args.config = selection["recipes"]["campaign"]["path"]
            args.selection = selection
        else:
            if args.task is not None:
                cli.error("--task requires --robot")
            if args.shared_root is not None:
                cli.error("--shared-root requires --robot/--task")
            args.selection = None
        config = load(project_path(args.config, args.project_root))
        paths = config["paths"]
        ledger = Ledger(
            project_path(paths["ledger"], args.project_root),
            project_path(paths["lock"], args.project_root),
        )
        _print_status(args, config, ledger)
        return 0
    except (CampaignError, OSError, ValueError) as error:
        print(f"status error: {error}", file=sys.stderr)
        return 2


def main(argv: list[str] | None = None) -> int:
    raw_argv = list(sys.argv[1:] if argv is None else argv)
    if raw_argv and raw_argv[0] == "status":
        return _status_command(raw_argv[1:])
    cli = parser()
    args = cli.parse_args(raw_argv)
    if args.target_total is not None and args.worker_manifest is None:
        cli.error("--target-total is supported through record_parallel.sh")
    args.project_root = args.project_root.resolve()
    try:
        _resolve_selection(args, cli)
        args.backend_log = (project_path(args.backend_log, args.project_root)).resolve()
        config = load(project_path(args.config, args.project_root))
        if args.worker_manifest is not None:
            from dual_arm_runtime.offline.collection.record_pool import record_parallel

            return record_parallel(args, config)
        code = _preflight(args)
        if code:
            return code
        paths = config["paths"]
        if "formal_select" in paths:
            (project_path(paths["formal_select"], args.project_root)).parent.mkdir(
                parents=True, exist_ok=True
            )
        ledger = Ledger(
            project_path(paths["ledger"], args.project_root),
            project_path(paths["lock"], args.project_root),
        )
        _print_startup_info(args, config, ledger)
        return record(args, config, ledger)
    except (CampaignError, OSError) as error:
        print(json.dumps({"error": str(error)}), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
