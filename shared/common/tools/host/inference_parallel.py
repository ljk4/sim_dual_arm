#!/usr/bin/env python3
"""Host-only launcher for isolated parallel policy inference workers (stdlib + Docker)."""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import math
import random
import re
import secrets
import signal
import subprocess
import sys
import time
from pathlib import Path

from host import docker_pool as _pool
from host.docker_pool import (
    ROOT,
    BaseDockerWorkers,
    automatic_workers,
    container_data_root,
    container_path_to_host,
    debug_root,
    host_path_to_container,
    positive_int,
    scenario_root,
    shared_root_to_container,
)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument(
        "--count", type=positive_int, default=10, help="number of episodes (default: 10)"
    )
    result.add_argument(
        "--seed-start", type=int, default=None, help="first reset seed (default: random)"
    )
    result.add_argument("--workers", default="auto", help="1..16 or auto (default: auto)")
    result.add_argument("--max-consecutive-failures", type=positive_int, default=None)
    result.add_argument("--project-root", type=Path, default=ROOT)
    result.add_argument("--robot")
    result.add_argument("--task")
    result.add_argument(
        "--shared-root",
        type=Path,
        help="host path of the scenario shared tree (default: <project>/shared)",
    )
    result.add_argument(
        "--runtime-config", default=None, help="direct runtime policy recipe"
    )
    result.add_argument(
        "--backend-template", default=None, help="direct simulation policy recipe"
    )
    result.add_argument(
        "--policy-endpoint", metavar="HOST:PORT", help="Policy Service gRPC address"
    )
    result.add_argument(
        "--realtime-speed",
        type=float,
        default=None,
        help="simulation realtime speed (default: the simulation policy recipe value)",
    )
    result.add_argument("--domain-base", type=int)
    result.add_argument("--batch-id", default=None)
    return result


class DockerWorkers(BaseDockerWorkers):
    def __init__(self, args: argparse.Namespace, directory: Path, batch_id: str) -> None:
        super().__init__(args, directory, batch_name=batch_id)
        self.batch_id = batch_id

    def _selection_arguments(self) -> list[str]:
        if self.args.robot is not None:
            return [
                "--robot",
                self.args.robot,
                "--task",
                self.args.task,
                "--shared-root",
                shared_root_to_container(
                    getattr(self.args, "shared_root", None), self.args.project_root
                ),
            ]
        return [
            "--runtime-config",
            host_path_to_container(self.args.runtime_config, self.args.project_root),
            "--backend-template",
            host_path_to_container(self.args.backend_template, self.args.project_root),
        ]

    def preflight(self) -> dict:
        self.ensure_runtime()
        result = subprocess.run(
            self.runtime_command(
                [
                    "python3",
                    "-m",
                    "dual_arm_runtime.offline.evaluation.inference_preflight",
                    "--project-root",
                    "/app",
                    *self._selection_arguments(),
                ]
            ),
            check=False,
            capture_output=True,
            text=True,
            timeout=120,
        )
        if result.returncode:
            message = result.stderr.strip() or result.stdout.strip()
            try:
                message = json.loads(message.splitlines()[-1])["error"]
            except (ValueError, KeyError, IndexError):
                message = message or f"preflight exited {result.returncode}"
            raise RuntimeError(f"preflight failed: {message}")
        self.plan = json.loads(result.stdout)
        return self.plan

    def prepare(self, plan: dict | None = None) -> list[dict]:
        self.ensure_runtime()
        result = subprocess.run(
            self.runtime_command(
                [
                    "python3",
                    "-m",
                    "dual_arm_runtime.offline.evaluation.inference_workers",
                    "--directory",
                    host_path_to_container(self.directory, self.args.project_root),
                    "--workers",
                    str(self.args.workers),
                    "--domain-base",
                    str(self.args.domain_base),
                    *(
                        ["--realtime-speed", str(self.args.realtime_speed)]
                        if self.args.realtime_speed is not None
                        else []
                    ),
                    *self._worker_selection_arguments(plan),
                ]
            ),
            check=False,
            capture_output=True,
            text=True,
            timeout=60,
        )
        if result.returncode:
            raise RuntimeError(result.stderr.strip() or result.stdout.strip())
        return json.loads((self.directory / "workers.json").read_text())["workers"]

    def wait_ready(self, workers: list[dict]) -> None:
        super().wait_ready(
            workers,
            timeout_message="parallel inference backends did not become ready within 60 seconds",
        )

    def runtime_command(
        self, arguments: list[str], *, pid_file: str | None = None
    ) -> list[str]:
        command = super().runtime_command(arguments, pid_file=pid_file)
        # Insert INFERENCE_HOST_MANAGED=1 right after "docker exec"
        return [*command[:2], "-e", "INFERENCE_HOST_MANAGED=1", *command[2:]]

    def run(self) -> int:
        relative = Path(host_path_to_container(self.directory, self.args.project_root))
        command = self.runtime_command(
            [
                "python3",
                "-m",
                "dual_arm_runtime.offline.evaluation.inference_worker",
                "--manifest",
                str(relative / "workers.json"),
                "--recordings",
                str(relative / "recordings"),
                "--runs-root",
                str(relative / "runs"),
                "--console-root",
                str(relative / "runs"),
                "--report",
                str(
                    debug_root(container_data_root(), self.args.robot, self.args.task)
                    / "batch-policy"
                    / self.batch_id
                    / "report.json"
                ),
                "--batch-id",
                self.batch_id,
                "--count",
                str(self.args.count),
                "--seed-start",
                str(self.args.seed_start),
                *(
                    ["--policy-endpoint", self.args.policy_endpoint]
                    if self.args.policy_endpoint is not None
                    else []
                ),
                *(
                    [
                        "--max-consecutive-failures",
                        str(self.args.max_consecutive_failures),
                    ]
                    if self.args.max_consecutive_failures is not None
                    else []
                ),
            ],
            pid_file=str(relative / "controller.pid"),
        )
        self.controller = subprocess.Popen(command, start_new_session=True)
        return self.controller.wait()

    def cleanup(self) -> None:
        original = _pool.subprocess
        _pool.subprocess = subprocess
        try:
            super().cleanup()
        finally:
            _pool.subprocess = original


def _print_startup_info(args: argparse.Namespace, plan: dict, directory: Path) -> None:
    print("=== 并行推理启动 ===", flush=True)
    selection = plan.get("selection") or {}
    if selection.get("mode") == "scenario":
        print(
            f"方案: {selection.get('robot')}/{selection.get('task')} | "
            f"Binding: {(selection.get('binding') or {}).get('path', '-')}",
            flush=True,
        )
    else:
        print(f"Runtime 配方: {args.runtime_config}", flush=True)
        print(f"Backend 配方: {args.backend_template}", flush=True)
    print(
        f"局数: {args.count} | 起始 seed: {args.seed_start} | "
        f"并行 worker: {args.workers}（无可视化） | "
        f"最大连败: {args.max_consecutive_failures or '不限'} | "
        f"实时倍速: {args.realtime_speed if args.realtime_speed is not None else '配方默认'}",
        flush=True,
    )
    print(f"批次目录: {directory.parent}", flush=True)
    if args.policy_endpoint is not None:
        print(f"策略服务: {args.policy_endpoint}", flush=True)


def main(argv: list[str] | None = None) -> int:
    cli = parser()
    args = cli.parse_args(argv)
    if args.robot is not None:
        if args.task is None:
            cli.error("--robot requires --task")
        if args.runtime_config is not None or args.backend_template is not None:
            cli.error("--robot/--task cannot be combined with direct recipes")
    else:
        if args.task is not None:
            cli.error("--task requires --robot")
        if args.shared_root is not None:
            cli.error("--shared-root requires --robot/--task")
        if args.runtime_config is None or args.backend_template is None:
            cli.error(
                "one of --robot/--task or --runtime-config/--backend-template is required"
            )
    if args.seed_start is None:
        args.seed_start = secrets.randbelow(max(1, 2**31 - args.count))
    if args.seed_start < 0 or args.seed_start + args.count - 1 > 2**63 - 1:
        cli.error("--seed-start must keep episode seeds within [0, 2^63-1]")
    if args.robot and args.task:
        segment = f"{args.robot}/{args.task}"
    else:
        segment = "_direct"
    args.workers = automatic_workers() if args.workers == "auto" else int(args.workers)
    if not 1 <= args.workers <= 16:
        cli.error("workers must be 1..16 or auto")
    if args.realtime_speed is not None and (
        not math.isfinite(args.realtime_speed) or not 0.1 < args.realtime_speed <= 100
    ):
        cli.error("realtime speed must be within (0.1, 100]")
    if args.domain_base is None:
        args.domain_base = random.randint(100, 200 - args.workers)
    if not 0 <= args.domain_base <= 232 - args.workers + 1:
        cli.error("ROS domain IDs must stay in 0..232")
    if (
        args.batch_id is not None
        and re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,99}", args.batch_id) is None
    ):
        cli.error("--batch-id must be 1..100 letters, digits, '.', '_' or '-'")
    args.project_root = args.project_root.resolve()
    root = scenario_root(args.project_root / "var", args.robot, args.task)
    batch_id = args.batch_id
    if not batch_id:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        candidate = f"batch-policy_{stamp}"
        if (root / "logs" / "batch-policy" / candidate).exists():
            suffix = 2
            while (root / "logs" / "batch-policy" / f"{candidate}_{suffix}").exists():
                suffix += 1
            candidate = f"{candidate}_{suffix}"
        batch_id = candidate
    directory = root / "logs" / "batch-policy" / batch_id / "workers"
    report_path = (
        args.project_root
        / "var"
        / segment
        / "debug"
        / "batch-policy"
        / batch_id
        / "report.json"
    )
    docker = DockerWorkers(args, directory, batch_id)

    def interrupted(signum: int, frame: object) -> None:
        raise KeyboardInterrupt

    previous = signal.signal(signal.SIGTERM, interrupted)
    try:
        plan = docker.preflight()
        args.workers = min(args.workers, args.count)
        _print_startup_info(args, plan, directory)
        directory.parent.mkdir(parents=True, exist_ok=True)
        workers = docker.prepare(plan)
        print(f"正在启动 {args.workers} 个无头仿真 worker 并等待就绪...", flush=True)
        with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as executor:
            list(executor.map(docker.start_backend, workers))
        docker.wait_ready(workers)
        print(f"全部 {args.workers} 个 worker 已就绪，开始并行推理。", flush=True)
        code = docker.run()
        if report_path.is_file():
            print(f"报告: {report_path}", flush=True)
        return code
    except KeyboardInterrupt:
        print("\n收到中断信号，正在停止并行推理并清理 worker 容器...", file=sys.stderr, flush=True)
        return 130
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as error:
        print(f"错误: 并行推理失败: {error}", file=sys.stderr, flush=True)
        return 2
    finally:
        signal.signal(signal.SIGTERM, previous)
        docker.cleanup()


__all__ = [
    "DockerWorkers",
    "ROOT",
    "automatic_workers",
    "container_path_to_host",
    "host_path_to_container",
    "main",
    "parser",
    "positive_int",
    "shared_root_to_container",
]


if __name__ == "__main__":
    raise SystemExit(main())
