#!/usr/bin/env python3
"""Host-only launcher for isolated headless recording workers (stdlib + Docker)."""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import math
import random
import signal
import subprocess
import sys
import time
from pathlib import Path

from host import docker_pool as _pool
from host.docker_pool import (
    CONTAINER_MOUNTS,
    ROOT,
    BaseDockerWorkers,
    automatic_workers,
    container_path_to_host,
    host_path_to_container,
    positive_int,
    scenario_root,
    shared_root_to_container,
)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    quantity = result.add_mutually_exclusive_group()
    quantity.add_argument(
        "--count",
        type=positive_int,
        help="number of NEW qualified episodes (default: 1)",
    )
    quantity.add_argument(
        "--target-total",
        type=positive_int,
        help="resume until the campaign has this many qualified episodes in total",
    )
    result.add_argument("--workers", default="auto", help="1..16 or auto (default: auto)")
    result.add_argument("--max-consecutive-failures", type=positive_int, default=3)
    result.add_argument("--purpose", choices=("formal", "pilot"), default="formal")
    result.add_argument("--project-root", type=Path, default=ROOT)
    result.add_argument("--robot")
    result.add_argument("--task")
    result.add_argument(
        "--shared-root",
        type=Path,
        help="host path of the scenario shared tree (default: <project>/shared)",
    )
    result.add_argument(
        "--config",
        default=None,
        help="direct campaign recipe (conflicts with --robot/--task)",
    )
    result.add_argument(
        "--runtime-config",
        default=None,
        help="direct runtime teacher recipe (conflicts with --robot/--task)",
    )
    result.add_argument(
        "--backend-template",
        default=None,
        help="direct simulation teacher recipe (conflicts with --robot/--task)",
    )
    result.add_argument("--realtime-speed", type=float, default=1.0)
    result.add_argument("--domain-base", type=int)
    return result


class DockerWorkers(BaseDockerWorkers):
    def __init__(self, args: argparse.Namespace, directory: Path) -> None:
        super().__init__(args, directory, batch_name=directory.name)

    def preflight_command(self) -> list[str]:
        command = [
            "python3",
            "-m",
            "dual_arm_runtime.offline.collection.record_preflight",
            "--project-root",
            "/app",
            "--purpose",
            self.args.purpose,
            "--parallel",
        ]
        if getattr(self.args, "robot", None) is not None:
            command += [
                "--robot",
                self.args.robot,
                "--task",
                self.args.task,
                "--shared-root",
                shared_root_to_container(
                    getattr(self.args, "shared_root", None), self.args.project_root
                ),
            ]
        else:
            command += [
                "--config",
                host_path_to_container(self.args.config, self.args.project_root),
                "--runtime-config",
                host_path_to_container(self.args.runtime_config, self.args.project_root),
                "--backend-template",
                host_path_to_container(self.args.backend_template, self.args.project_root),
            ]
        if self.args.target_total is not None:
            command += ["--target-total", str(self.args.target_total)]
        return command

    def preflight(self) -> dict:
        """Resolve the binding and validate every read-only gate before Docker work."""
        self.ensure_runtime()
        result = subprocess.run(
            self.runtime_command(self.preflight_command()),
            check=False,
            capture_output=True,
            text=True,
            timeout=120,
        )
        if result.returncode:
            message = result.stderr.strip()
            try:
                message = json.loads(message.splitlines()[-1])["error"]
            except (ValueError, KeyError, IndexError):
                message = message or f"preflight exited {result.returncode}"
            raise RuntimeError(f"preflight failed: {message}")
        self.plan = json.loads(result.stdout)
        return self.plan

    def resolve_target(self, plan: dict) -> bool:
        """Read campaign progress before creating any simulation workers."""
        status = plan["campaign_status"]
        current = int(status["purposes"][self.args.purpose]["qualified"])
        self.args.count = max(0, self.args.target_total - current)
        print(
            f"{self.args.purpose} 合格总数 {current}/{self.args.target_total}，"
            f"本次需补采 {self.args.count} 条",
            flush=True,
        )
        return self.args.count == 0

    def prepare(self, plan: dict | None = None) -> list[dict]:
        self.ensure_runtime()
        result = subprocess.run(
            self.runtime_command(
                [
                    "python3",
                    "-m",
                    "dual_arm_runtime.offline.collection.record_workers",
                    "--directory",
                    host_path_to_container(self.directory, self.args.project_root),
                    "--workers",
                    str(self.args.workers),
                    "--domain-base",
                    str(self.args.domain_base),
                    "--realtime-speed",
                    str(self.args.realtime_speed),
                    *self._worker_selection_arguments(plan),
                ]
            ),
            capture_output=True,
            text=True,
            timeout=60,
        )
        if result.returncode:
            raise RuntimeError(result.stderr.strip() or result.stdout.strip())
        return json.loads((self.directory / "workers.json").read_text())["workers"]

    def run(self, plan: dict | None = None) -> int:
        relative = Path(host_path_to_container(self.directory, self.args.project_root))
        selection_arguments = self._scenario_selection_arguments(plan)
        if selection_arguments is None:
            selection_arguments = [
                "--config",
                host_path_to_container(self.args.config, self.args.project_root),
                "--runtime-config",
                host_path_to_container(self.args.runtime_config, self.args.project_root),
            ]
        command = self.runtime_command(
            [
                "python3",
                "-m",
                "dual_arm_runtime.offline.collection.record",
                "--count",
                str(self.args.count),
                "--max-consecutive-failures",
                str(self.args.max_consecutive_failures),
                "--purpose",
                self.args.purpose,
                *selection_arguments,
                "--worker-manifest",
                str(relative / "workers.json"),
                *(
                    ["--target-total", str(self.args.target_total)]
                    if self.args.target_total is not None
                    else []
                ),
            ],
            pid_file=str(relative / "controller.pid"),
        )
        self.controller = subprocess.Popen(command, start_new_session=True)
        return self.controller.wait()

    def cleanup(self) -> None:
        # Delegate through module-local subprocess so unit test monkeypatching works.
        original = _pool.subprocess
        _pool.subprocess = subprocess
        try:
            super().cleanup()
        finally:
            _pool.subprocess = original


def _print_startup_info(
    args: argparse.Namespace, plan: dict, directory: Path
) -> None:
    print("=== 并行采集启动 ===", flush=True)
    selection = plan.get("selection") or {}
    if selection.get("mode") == "scenario":
        print(
            f"方案: {selection.get('robot')}/{selection.get('task')} | "
            f"Binding: {(selection.get('binding') or {}).get('path', '-')}",
            flush=True,
        )
    else:
        print(f"Campaign 配置: {args.config}", flush=True)
        print(f"Runtime 配置: {args.runtime_config}", flush=True)
        print(f"Backend 模板: {args.backend_template}", flush=True)
    target = (
        f"目标总数 {args.target_total}"
        if args.target_total is not None
        else f"本次 {args.count} 条"
    )
    print(
        f"用途: {args.purpose} | {target} | worker: {args.workers} | "
        f"最大连败: {args.max_consecutive_failures} | 实时倍速: {args.realtime_speed}",
        flush=True,
    )
    print(f"批次目录: {directory}", flush=True)
    status = plan.get("campaign_status")
    if isinstance(status, dict):
        formal = (status.get("purposes") or {}).get("formal", {})
        print(
            f"当前台账: 正式合格 {formal.get('qualified', 0)} 条",
            flush=True,
        )


def main(argv: list[str] | None = None) -> int:
    cli = parser()
    args = cli.parse_args(argv)
    if args.count is None and args.target_total is None:
        args.count = 1
    try:
        if args.robot is not None:
            if args.task is None:
                cli.error("--robot requires --task")
            explicit = [
                name
                for name in ("config", "runtime_config", "backend_template")
                if getattr(args, name) is not None
            ]
            if explicit:
                cli.error(
                    "--robot/--task cannot be combined with "
                    + ", ".join("--" + name.replace("_", "-") for name in explicit)
                )
        else:
            if args.task is not None:
                cli.error("--task requires --robot")
            if args.shared_root is not None:
                cli.error("--shared-root requires --robot/--task")
            missing = [
                name
                for name in ("config", "runtime_config", "backend_template")
                if getattr(args, name) is None
            ]
            if missing:
                cli.error(
                    "one of --robot/--task or all of --config/--runtime-config/"
                    "--backend-template is required"
                )
        args.workers = automatic_workers() if args.workers == "auto" else int(args.workers)
        if not 1 <= args.workers <= 16:
            raise ValueError("workers must be 1..16 or auto")
        if not math.isfinite(args.realtime_speed) or not 0.1 < args.realtime_speed <= 100:
            raise ValueError("realtime speed must be within (0.1, 100]")
        if args.domain_base is None:
            args.domain_base = random.randint(100, 200 - args.workers)
        if not 0 <= args.domain_base <= 232 - args.workers + 1:
            raise ValueError("ROS domain IDs must stay in 0..232")
        args.project_root = args.project_root.resolve()
    except ValueError as error:
        cli.error(str(error))
    pools_root = (
        scenario_root(args.project_root / "var", args.robot, args.task)
        / "logs"
        / "collection"
        / "record-pools"
    )
    pool_stamp = time.strftime("%Y%m%d_%H%M%S")
    directory = pools_root / f"pool_{pool_stamp}"
    if directory.exists():
        suffix = 2
        while (pools_root / f"pool_{pool_stamp}_{suffix}").exists():
            suffix += 1
        directory = pools_root / f"pool_{pool_stamp}_{suffix}"
    docker = DockerWorkers(args, directory)

    def interrupted(signum: int, frame: object) -> None:
        raise KeyboardInterrupt

    previous = signal.signal(signal.SIGTERM, interrupted)
    try:
        plan = docker.preflight()
        _print_startup_info(args, plan, directory)
        if args.target_total is not None and docker.resolve_target(plan):
            return 0
        directory.parent.mkdir(parents=True, exist_ok=True)
        args.workers = min(args.workers, args.count)
        workers = docker.prepare(plan)
        print(f"并行 worker: {args.workers}（无可视化） | 批次日志: {directory}", flush=True)
        with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as executor:
            list(executor.map(docker.start_backend, workers))
        docker.wait_ready(workers)
        return docker.run(plan)
    except KeyboardInterrupt:
        return 130
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as error:
        print(f"parallel recording: {error}", file=sys.stderr)
        return 2
    finally:
        signal.signal(signal.SIGTERM, previous)
        docker.cleanup()


__all__ = [
    "CONTAINER_MOUNTS",
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
