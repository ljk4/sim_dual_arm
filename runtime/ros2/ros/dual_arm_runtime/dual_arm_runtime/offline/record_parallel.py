#!/usr/bin/env python3
"""Host-only launcher for isolated headless recording workers (stdlib + Docker)."""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import math
import os
import random
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(os.environ.get("PROJECT_ROOT") or Path(__file__).resolve().parents[6])


def positive_int(value: str) -> int:
    try:
        number = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("must be a positive integer") from error
    if number < 1:
        raise argparse.ArgumentTypeError("must be a positive integer")
    return number


def automatic_workers() -> int:
    cpus = len(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else os.cpu_count() or 1
    limits = [4, max(1, cpus // 4)]
    try:
        memory = next(line for line in Path("/proc/meminfo").read_text().splitlines()
                      if line.startswith("MemAvailable:"))
        limits.append(max(1, (int(memory.split()[1]) - 1024 * 1024) // (1536 * 1024)))
    except (OSError, StopIteration, ValueError):
        pass
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=memory.free", "--format=csv,noheader,nounits"],
            capture_output=True, text=True, timeout=5, check=False,
        )
        if result.returncode == 0:
            # Leave rendering headroom; memory alone cannot predict GPU throughput.
            limits.append(max(1, (int(result.stdout.splitlines()[0]) - 512) // 1024))
    except (OSError, ValueError, IndexError, subprocess.TimeoutExpired):
        limits.append(2)
    return min(limits)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    quantity = result.add_mutually_exclusive_group()
    quantity.add_argument(
        "--count", type=positive_int,
        help="number of NEW qualified episodes (default: 1)",
    )
    quantity.add_argument(
        "--target-total", type=positive_int,
        help="resume until the campaign has this many qualified episodes in total",
    )
    result.add_argument("--workers", default="auto", help="1..16 or auto (default: auto)")
    result.add_argument("--max-consecutive-failures", type=positive_int, default=3)
    result.add_argument("--purpose", choices=("formal", "pilot"), default="formal")
    result.add_argument("--project-root", type=Path, default=ROOT)
    result.add_argument("--config", default="runtime/ros2/configs/experiments/stack_campaign.yaml")
    result.add_argument("--runtime-config", default="runtime/ros2/configs/teacher_collect.yaml")
    result.add_argument(
        "--backend-template", default="simulation/mujoco/configs/backend/backend_stack_teacher.yaml"
    )
    result.add_argument("--realtime-speed", type=float, default=1.0)
    result.add_argument("--domain-base", type=int)
    return result


class DockerWorkers:
    def __init__(self, args: argparse.Namespace, directory: Path) -> None:
        self.args, self.directory = args, directory
        self.created: list[str] = []
        self.runtime = ""
        self.controller: subprocess.Popen | None = None
        self.controller_pid = directory / "controller.pid"

    def compose(self, service: str) -> list[str]:
        section = "simulation/mujoco" if service == "simulation" else "runtime/ros2"
        compose = self.args.project_root / section / "docker/docker-compose.yml"
        return ["docker", "compose", "-f", str(compose)]

    def runtime_command(self, arguments: list[str], *, pid_file: str | None = None) -> list[str]:
        shell = "source /workspace/infrastructure/docker/setup.sh; "
        if pid_file is not None:
            shell += 'echo "$$" > "$1"; shift; '
        return [
            "docker", "exec", self.runtime, "bash", "-c",
            shell + 'exec "$@"', "bash", *([pid_file] if pid_file else []), *arguments,
        ]

    def ensure_runtime(self) -> None:
        if self.runtime:
            return
        runtime = subprocess.run(
            [*self.compose("runtime"), "ps", "-q", "runtime"],
            check=True, text=True, capture_output=True, timeout=20,
        ).stdout.strip()
        if not runtime:
            subprocess.run(
                [*self.compose("runtime"), "up", "-d", "--no-build", "--pull", "never", "runtime"],
                check=True, timeout=60,
            )
            runtime = subprocess.run(
                [*self.compose("runtime"), "ps", "-q", "runtime"],
                check=True, text=True, capture_output=True, timeout=20,
            ).stdout.strip()
        self.runtime = runtime

    def resolve_target(self) -> bool:
        """Read campaign progress before creating any simulation workers."""
        self.ensure_runtime()
        result = subprocess.run(
            self.runtime_command(
                ["python3", "-m", "dual_arm_runtime.offline.campaign",
                 "--project-root", "/workspace", "--config", self.args.config, "status"]
            ), check=True, capture_output=True, text=True, timeout=30,
        )
        status = json.loads(result.stdout)
        if self.args.purpose == "formal" and self.args.target_total > int(
            status["formal_target_qualified"]
        ):
            raise RuntimeError("target total exceeds the campaign formal qualified target")
        current = int(status["purposes"][self.args.purpose]["qualified"])
        self.args.count = max(0, self.args.target_total - current)
        print(
            f"{self.args.purpose} 合格总数 {current}/{self.args.target_total}，"
            f"本次需补采 {self.args.count} 条", flush=True,
        )
        return self.args.count == 0

    def prepare(self) -> list[dict]:
        self.ensure_runtime()
        relative = self.directory.relative_to(self.args.project_root)
        subprocess.run(
            self.runtime_command(
                [
                    "python3", "-m", "dual_arm_runtime.offline.record_workers",
                    "--directory", str(Path("/workspace") / relative),
                    "--workers", str(self.args.workers),
                    "--domain-base", str(self.args.domain_base),
                    "--realtime-speed", str(self.args.realtime_speed),
                    "--runtime-config", str(Path("/workspace") / self.args.runtime_config),
                    "--backend-template", str(Path("/workspace") / self.args.backend_template),
                ]
            ), check=True, timeout=30,
        )
        return json.loads((self.directory / "workers.json").read_text())["workers"]

    def start_backend(self, worker: dict) -> str:
        name = f"sim-dual-arm-{self.directory.name}-{worker['id']}"
        # Record names before starting so partial Docker failures are also cleaned.
        self.created.append(name)
        subprocess.run(
            [
                *self.compose("simulation"), "run", "--rm", "-d", "--no-deps", "--name", name,
                "-e", f"ROS_DOMAIN_ID={worker['domain_id']}",
                "-e", "OMP_NUM_THREADS=1", "-e", "OPENBLAS_NUM_THREADS=1",
                "-e", "MKL_NUM_THREADS=1", "-e", "DISPLAY=", "-e", "WAYLAND_DISPLAY=",
                "simulation", "scripts/run.sh", "backend", "--config", worker["backend_config"],
                "--run-root", worker["run_root"], "--run-id", "backend",
            ], check=True, capture_output=True, text=True, timeout=60,
        )
        return name

    def wait_ready(self, workers: list[dict]) -> None:
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            ready = []
            for worker in workers:
                relative = Path(worker["backend_log"]).relative_to("/workspace")
                log = self.args.project_root / relative
                ready.append(any('"event":"backend.start"' in p.read_text()
                                 for p in log.glob("events*.jsonl")))
            if all(ready):
                return
            time.sleep(0.25)
        raise RuntimeError("parallel backends did not become ready within 60 seconds")

    def run(self) -> int:
        relative = Path("/workspace") / self.directory.relative_to(self.args.project_root)
        command = self.runtime_command(
            [
                "python3", "-m", "dual_arm_runtime.offline.record",
                "--count", str(self.args.count),
                "--max-consecutive-failures", str(self.args.max_consecutive_failures),
                "--purpose", self.args.purpose, "--config", self.args.config,
                "--worker-manifest", str(relative / "workers.json"),
                *(["--target-total", str(self.args.target_total)]
                  if self.args.target_total is not None else []),
            ], pid_file=str(relative / "controller.pid"),
        )
        # A PID file lets cleanup signal the controller inside the existing runtime
        # container: terminating docker exec alone does not stop its remote process.
        self.controller = subprocess.Popen(command, start_new_session=True)
        return self.controller.wait()

    def cleanup(self) -> None:
        if self.controller is not None and self.controller.poll() is None:
            try:
                deadline = time.monotonic() + 3
                while not self.controller_pid.exists() and time.monotonic() < deadline:
                    if self.controller.poll() is not None:
                        break
                    time.sleep(0.1)
                pid = int(self.controller_pid.read_text().strip())
                subprocess.run(
                    ["docker", "exec", self.runtime, "kill", "-TERM", str(pid)],
                    check=False, timeout=10, capture_output=True,
                )
                self.controller.wait(timeout=35)
            except (OSError, ValueError, subprocess.TimeoutExpired):
                self.controller.terminate()
        def stop(name: str) -> None:
            try:
                logs = subprocess.run(
                    ["docker", "logs", name], text=True, capture_output=True,
                    check=False, timeout=10,
                )
                (self.directory / f"{name}.log").write_text(logs.stdout + logs.stderr)
            finally:
                subprocess.run(
                    ["docker", "stop", "--timeout", "20", name],
                    check=False, capture_output=True, timeout=30,
                )
        count = max(1, len(self.created))
        with concurrent.futures.ThreadPoolExecutor(max_workers=count) as executor:
            list(executor.map(stop, self.created))


def main(argv: list[str] | None = None) -> int:
    cli = parser()
    args = cli.parse_args(argv)
    if args.count is None and args.target_total is None:
        args.count = 1
    try:
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
        for field in ("config", "runtime_config", "backend_template"):
            path = (args.project_root / getattr(args, field)).resolve()
            setattr(args, field, str(path.relative_to(args.project_root)))
    except ValueError as error:
        cli.error(str(error))
    directory = args.project_root / "var/record-pools" / f"pool-{time.time_ns()}"
    directory.parent.mkdir(parents=True, exist_ok=True)
    docker = DockerWorkers(args, directory)
    def interrupted(signum: int, frame: object) -> None:
        raise KeyboardInterrupt

    previous = signal.signal(signal.SIGTERM, interrupted)
    try:
        if args.target_total is not None and docker.resolve_target():
            return 0
        args.workers = min(args.workers, args.count)
        workers = docker.prepare()
        print(f"并行 worker：{args.workers}；可视化与预览已禁用；日志：{directory}", flush=True)
        with concurrent.futures.ThreadPoolExecutor(max_workers=args.workers) as executor:
            list(executor.map(docker.start_backend, workers))
        docker.wait_ready(workers)
        return docker.run()
    except KeyboardInterrupt:
        return 130
    except (OSError, RuntimeError, subprocess.SubprocessError) as error:
        print(f"parallel recording: {error}", file=sys.stderr)
        return 2
    finally:
        signal.signal(signal.SIGTERM, previous)
        docker.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
