#!/usr/bin/env python3
"""Headless batch policy inference runner for simulation backends."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

from dual_arm_runtime.artifact_paths import container_data_root, debug_root, logs_root
from dual_arm_runtime.runtime_cli import _load_configuration


def positive_int(value: str) -> int:
    try:
        number = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("must be a positive integer") from error
    if number < 1:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return number


def non_negative_int(value: str) -> int:
    try:
        number = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("must be a non-negative integer") from error
    if number < 0:
        raise argparse.ArgumentTypeError("must be a non-negative integer")
    return number


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=Path("/app"))
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument("--config", type=Path, help="direct runtime recipe path")
    selection.add_argument(
        "--robot", help="robot id under <shared-root>/scenarios/<robot>/<task>/binding.yaml"
    )
    parser.add_argument("--task", help="task id for --robot")
    parser.add_argument("--role", choices=("policy",), default="policy")
    parser.add_argument("--shared-root", type=Path)
    parser.add_argument(
        "--count", type=positive_int, default=10, help="number of episodes (default: 10)"
    )
    parser.add_argument(
        "--seed-start", type=non_negative_int, default=20000, help="starting reset seed"
    )
    parser.add_argument(
        "--max-consecutive-failures",
        type=positive_int,
        default=None,
        help="optional early-stop threshold for consecutive failed episodes",
    )
    parser.add_argument(
        "--set",
        dest="overrides",
        action="append",
        default=[],
        metavar="PATH=VALUE",
        help="override one existing configuration path (repeatable)",
    )
    parser.add_argument("--batch-id", default=None)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--runs-output", type=Path, default=None)
    return parser


def _verify_simulation_backend(namespace: str, timeout_s: float = 5.0) -> dict[str, object]:
    """Ensure the live backend is a MuJoCo simulation backend with reset support."""
    import rclpy
    from rclpy.node import Node
    from robot_api_msgs.msg import BackendInfo
    from robot_ros_support.qos import INFO_QOS

    topic = f"{namespace.rstrip('/')}/info"
    received: list[BackendInfo] = []
    own_context = not rclpy.ok()
    if own_context:
        rclpy.init()
    node = Node("batch_policy_probe")
    try:
        node.create_subscription(BackendInfo, topic, received.append, INFO_QOS)
        deadline = time.monotonic() + timeout_s
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if own_context and rclpy.ok():
            rclpy.shutdown()

    if not received:
        raise RuntimeError(
            f"未检测到后端 {topic}；请先在 simulation 容器启动仿真后端（scripts/policy_backend.sh）"
        )
    info = received[-1]
    if info.backend_kind != "mujoco":
        raise RuntimeError(
            f"批量推理仅支持仿真后端（backend_kind='mujoco'），当前后端为 {info.backend_kind!r}"
        )
    if "simulation_reset" not in set(info.capabilities):
        raise RuntimeError("当前仿真后端未声明 'simulation_reset' 能力，无法无人工干预重置批量推理")
    return {
        "backend_kind": info.backend_kind,
        "capabilities": list(info.capabilities),
        "ready": bool(info.ready),
    }


def _format_duration(seconds: float) -> str:
    total = max(0, int(seconds))
    minutes, secs = divmod(total, 60)
    hours, minutes = divmod(minutes, 60)
    return f"{hours:02d}:{minutes:02d}:{secs:02d}" if hours else f"{minutes:02d}:{secs:02d}"


def _render_progress(
    done: int,
    total: int,
    succeeded: int,
    failed: int,
    current_seed: int,
    started: float,
    *,
    force: bool = False,
) -> None:
    is_tty = sys.stdout.isatty()
    if not is_tty and not force:
        return
    ratio = min(1.0, done / max(1, total))
    width = 20
    filled = int(width * ratio)
    bar = "█" * filled + "░" * (width - filled)
    elapsed = time.monotonic() - started
    line = (
        f"批量仿真推理 [{bar}] {done}/{total} ({int(ratio * 100)}%) "
        f"| 成功 {succeeded} | 失败 {failed} | 当前 seed {current_seed} "
        f"| 用时 {_format_duration(elapsed)}"
    )
    if is_tty:
        sys.stdout.write(f"\r\033[K{line}")
        sys.stdout.flush()
    else:
        print(line, flush=True)


def _print_startup_info(
    args: argparse.Namespace,
    configuration: object,
    batch_id: str,
    recordings_dir: Path,
    runs_dir: Path,
    report_path: Path,
    backend_info: dict[str, object],
) -> None:
    print("=== Batch Policy 批量仿真推理配置与详细参数 ===", flush=True)
    selection = getattr(configuration, "selection", None)
    if isinstance(selection, dict) and selection.get("mode") == "scenario":
        print(
            f"方案 (scenario): {selection.get('robot')}/{selection.get('task')} "
            f"({selection.get('role')})",
            flush=True,
        )
        binding = selection.get("binding") or {}
        if binding.get("path"):
            print(f"Binding 文件:    {binding['path']}", flush=True)
    print(f"配置文件路径:    {configuration.configuration_source}", flush=True)
    print(f"后端类型:        {backend_info['backend_kind']} (已禁止可视化)", flush=True)
    print(
        f"批量参数:        batch_id={batch_id}, count={args.count}, "
        f"seed_start={args.seed_start}, "
        f"max_consecutive_failures={args.max_consecutive_failures}, "
        f"overrides={args.overrides}",
        flush=True,
    )
    print(f"录制目录:        {recordings_dir}", flush=True)
    print(f"运行日志目录:    {runs_dir}", flush=True)
    print(f"报告输出路径:    {report_path}", flush=True)
    print("--- 详细参数 ---", flush=True)
    print(json.dumps(configuration.snapshot, indent=2, ensure_ascii=False), flush=True)
    print("================================================", flush=True)


def _episode_command(
    args: argparse.Namespace,
    run_id: str,
    seed: int,
    recordings_dir: Path,
    runs_dir: Path,
) -> list[str]:
    command = [
        sys.executable,
        "-m",
        "dual_arm_runtime.runtime_node",
        "--project-root",
        str(args.project_root),
        "--runs-output",
        str(runs_dir),
        "--output",
        str(recordings_dir),
        "--once",
        "--run-id",
        run_id,
        "--episode-id",
        run_id,
        "--reset-seed",
        str(seed),
    ]
    if args.robot is not None:
        command += ["--robot", args.robot, "--task", args.task, "--role", args.role]
        if args.shared_root is not None:
            command += ["--shared-root", str(args.shared_root)]
    else:
        command += ["--config", str(args.config)]
    for item in args.overrides:
        command += ["--set", item]
    return command


def _parse_summary(stdout: str) -> dict[str, object] | None:
    for line in reversed(stdout.splitlines()):
        stripped = line.strip()
        if stripped.startswith("{") and stripped.endswith("}"):
            try:
                summary = json.loads(stripped)
            except json.JSONDecodeError:
                continue
            if isinstance(summary, dict) and "task" in summary and "runtime" in summary:
                return summary
    return None


def _episode_succeeded(exit_code: int, summary: dict[str, object]) -> bool:
    return (
        exit_code == 0
        and summary.get("task", {}).get("outcome") == "succeeded"
        and summary.get("runtime", {}).get("outcome") in {"completed", "stopped"}
        and summary.get("recording", {}).get("complete") is True
    )


def main(argv: list[str] | None = None) -> int:
    cli = _parser()
    args = cli.parse_args(argv)
    if args.robot is not None and args.task is None:
        cli.error("--robot requires --task")
    if args.task is not None and args.robot is None:
        cli.error("--task requires --robot")
    for item in args.overrides:
        lowered = item.lower()
        if "visualization" in lowered or "viewer" in lowered:
            cli.error("批量仿真推理禁止可视化，不可覆盖 visualization/viewer 配置")

    batch_id = (
        args.batch_id
        or f"batch-policy-{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}-{os.getpid()}"
    )
    base_data = container_data_root()
    logs_dir = logs_root(base_data, args.robot, args.task) / "batch-policy" / batch_id
    recordings_dir = args.output or (logs_dir / "recordings")
    runs_dir = args.runs_output or (logs_dir / "runs")
    report_path = (
        debug_root(base_data, args.robot, args.task) / "batch-policy" / batch_id / "report.json"
    )

    load_args = argparse.Namespace(
        robot=args.robot,
        task=args.task,
        config=args.config,
        role=args.role,
        shared_root=args.shared_root,
        project_root=args.project_root,
        reset_seed=args.seed_start,
        overrides=args.overrides,
    )
    configuration = _load_configuration(load_args, batch_id)

    try:
        backend_info = _verify_simulation_backend(configuration.recipe.robot_namespace)
    except RuntimeError as error:
        print(f"batch_policy error: {error}", file=sys.stderr)
        return 2

    recordings_dir.mkdir(parents=True, exist_ok=True)
    runs_dir.mkdir(parents=True, exist_ok=True)
    report_path.parent.mkdir(parents=True, exist_ok=True)

    _print_startup_info(
        args,
        configuration,
        batch_id,
        recordings_dir,
        runs_dir,
        report_path,
        backend_info,
    )

    child_env = {**os.environ, "DISPLAY": "", "WAYLAND_DISPLAY": ""}
    episodes: list[dict[str, object]] = []
    succeeded = 0
    failed = 0
    consecutive_failures = 0
    interrupted = False
    started = time.monotonic()

    try:
        for index in range(args.count):
            seed = args.seed_start + index
            run_id = f"{batch_id}-ep-{index:04d}"
            _render_progress(index, args.count, succeeded, failed, seed, started)
            command = _episode_command(args, run_id, seed, recordings_dir, runs_dir)
            result = subprocess.run(
                command,
                env=child_env,
                capture_output=True,
                text=True,
            )
            console_log = runs_dir / f"{run_id}.console.log"
            console_log.write_text(result.stdout + result.stderr, encoding="utf-8")
            summary = _parse_summary(result.stdout) or {}
            episode_ok = _episode_succeeded(result.returncode, summary)
            if episode_ok:
                succeeded += 1
                consecutive_failures = 0
            else:
                failed += 1
                consecutive_failures += 1
            episodes.append(
                {
                    "index": index,
                    "seed": seed,
                    "run_id": run_id,
                    "exit_code": result.returncode,
                    "succeeded": episode_ok,
                    "summary": summary,
                    "console_log": str(console_log),
                }
            )
            _render_progress(index + 1, args.count, succeeded, failed, seed, started)
            if (
                args.max_consecutive_failures is not None
                and consecutive_failures >= args.max_consecutive_failures
            ):
                break
    except KeyboardInterrupt:
        interrupted = True

    _render_progress(
        len(episodes),
        args.count,
        succeeded,
        failed,
        args.seed_start + max(0, len(episodes) - 1),
        started,
        force=not sys.stdout.isatty(),
    )
    if sys.stdout.isatty():
        sys.stdout.write("\n")
        sys.stdout.flush()

    elapsed = round(time.monotonic() - started, 3)
    attempted = len(episodes)
    success_rate = round((succeeded / attempted) * 100.0, 2) if attempted else 0.0
    report = {
        "batch_id": batch_id,
        "robot": args.robot,
        "task": args.task,
        "config": str(configuration.configuration_source),
        "backend": backend_info,
        "visualization_disabled": True,
        "requested": args.count,
        "attempted": attempted,
        "succeeded": succeeded,
        "failed": failed,
        "success_rate_percent": success_rate,
        "interrupted": interrupted,
        "elapsed_s": elapsed,
        "recordings_dir": str(recordings_dir),
        "runs_dir": str(runs_dir),
        "episodes": episodes,
    }
    report_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    print("=== 批量仿真推理结果 ===", flush=True)
    print(
        f"完成局数: {attempted}/{args.count} | 成功: {succeeded} | 失败: {failed} "
        f"| 成功率: {success_rate:.2f}%",
        flush=True,
    )
    print(f"总耗时:   {elapsed:.1f}s", flush=True)
    print(f"详细报告: {report_path}", flush=True)
    print("========================", flush=True)
    if interrupted:
        return 130
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
