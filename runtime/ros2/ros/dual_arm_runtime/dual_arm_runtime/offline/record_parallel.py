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
DEFAULT_CAMPAIGN = (
    "shared/scenarios/panda_stack/configs/runtime/experiments/stack_campaign.yaml"
)
DEFAULT_RUNTIME_TEACHER = (
    "shared/scenarios/panda_stack/configs/runtime/teacher_collect.yaml"
)
DEFAULT_BACKEND_TEMPLATE = (
    "shared/scenarios/panda_stack/configs/simulation/backend/backend_stack_teacher.yaml"
)
CONTAINER_MOUNTS = (
    ("/workspace", Path(".")),
    ("/shared", Path("shared")),
    ("/data", Path("var")),
    ("/build", Path("build")),
)


def _host_path(value: str | Path, project_root: Path) -> Path:
    path = Path(value)
    if not path.is_absolute():
        path = project_root / path
    return path.resolve()


def host_path_to_container(value: str | Path, project_root: Path) -> str:
    """Map a host path to the runtime container; reject unmounted paths."""
    path = _host_path(value, project_root)
    project_root = project_root.resolve()
    if path.is_relative_to(project_root):
        return str(Path("/workspace") / path.relative_to(project_root))
    raise ValueError(
        f"host path is outside the mounted project root and has no container "
        f"equivalent: {path}"
    )


def container_path_to_host(value: str | Path, project_root: Path) -> Path:
    """Map a runtime/simulation container path back to the host project."""
    path = Path(value)
    project_root = project_root.resolve()
    for container, relative in CONTAINER_MOUNTS:
        root = Path(container)
        if path == root or path.is_relative_to(root):
            return (project_root / relative / path.relative_to(root)).resolve()
    raise ValueError(f"container path is not mounted in the project: {path}")


def shared_root_to_container(value: str | Path | None, project_root: Path) -> str:
    """The scenario shared tree is mounted at /shared; anything else must map."""
    host = _host_path(value, project_root) if value is not None else (
        project_root / "shared"
    ).resolve()
    expected = (project_root / "shared").resolve()
    if host == expected:
        return "/shared"
    return host_path_to_container(host, project_root)


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
    result.add_argument("--scenario")
    result.add_argument(
        "--shared-root", type=Path,
        help="host path of the scenario shared tree (default: <project>/shared)",
    )
    result.add_argument(
        "--config", default=None,
        help="direct campaign recipe (conflicts with --scenario)",
    )
    result.add_argument(
        "--runtime-config", default=None,
        help="direct runtime teacher recipe (conflicts with --scenario)",
    )
    result.add_argument(
        "--backend-template", default=None,
        help="direct simulation teacher recipe (conflicts with --scenario)",
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
        self.plan: dict | None = None

    def compose(self, service: str) -> list[str]:
        section = "simulation/mujoco" if service == "simulation" else "runtime/ros2"
        compose = self.args.project_root / section / "docker/docker-compose.yml"
        return ["docker", "compose", "-f", str(compose)]

    def runtime_command(self, arguments: list[str], *, pid_file: str | None = None) -> list[str]:
        shell = "source /shared/common/tools/docker/setup.sh; "
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

    def preflight_command(self) -> list[str]:
        command = [
            "python3", "-m", "dual_arm_runtime.offline.record_preflight",
            "--project-root", "/workspace", "--purpose", self.args.purpose, "--parallel",
        ]
        if getattr(self.args, "scenario", None) is not None:
            command += [
                "--scenario", self.args.scenario,
                "--shared-root", shared_root_to_container(
                    getattr(self.args, "shared_root", None), self.args.project_root
                ),
            ]
        else:
            command += [
                "--config", host_path_to_container(self.args.config, self.args.project_root),
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
            check=False, capture_output=True, text=True, timeout=120,
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

    def resolve_target(self, plan: dict | None = None) -> bool:
        """Read campaign progress before creating any simulation workers."""
        if plan is None:
            self.ensure_runtime()
            result = subprocess.run(
                self.runtime_command(
                    ["python3", "-m", "dual_arm_runtime.offline.campaign",
                     "--project-root", "/workspace", "--config", self.args.config, "status"]
                ), check=True, capture_output=True, text=True, timeout=30,
            )
            status = json.loads(result.stdout)
        else:
            status = plan["campaign_status"]
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

    def _scenario_selection_arguments(self, plan: dict | None) -> list[str] | None:
        if plan is None or plan["selection"]["mode"] != "scenario":
            return None
        return [
            "--scenario", plan["selection"]["scenario_id"],
            "--shared-root", shared_root_to_container(
                getattr(self.args, "shared_root", None), self.args.project_root
            ),
            "--binding-sha256", plan["selection"]["binding"]["sha256"],
        ]

    def _worker_selection_arguments(self, plan: dict | None) -> list[str]:
        scenario = self._scenario_selection_arguments(plan)
        if scenario is not None:
            return scenario
        if getattr(self.args, "scenario", None) is not None:
            raise RuntimeError("scenario preflight did not return a scenario selection")
        return [
            "--runtime-config",
            host_path_to_container(self.args.runtime_config, self.args.project_root),
            "--backend-template",
            host_path_to_container(self.args.backend_template, self.args.project_root),
        ]

    def prepare(self, plan: dict | None = None) -> list[dict]:
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
                    *self._worker_selection_arguments(plan),
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
                log = container_path_to_host(
                    worker["backend_log"], self.args.project_root
                )
                ready.append(any('"event":"backend.start"' in p.read_text()
                                 for p in log.glob("events*.jsonl")))
            if all(ready):
                return
            time.sleep(0.25)
        raise RuntimeError("parallel backends did not become ready within 60 seconds")

    def run(self, plan: dict | None = None) -> int:
        relative = Path("/workspace") / self.directory.relative_to(self.args.project_root)
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
                "python3", "-m", "dual_arm_runtime.offline.record",
                "--count", str(self.args.count),
                "--max-consecutive-failures", str(self.args.max_consecutive_failures),
                "--purpose", self.args.purpose,
                *selection_arguments,
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
        if (
            not self.created
            and self.controller is None
            and not self.directory.exists()
        ):
            return
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
        if args.scenario is not None:
            explicit = [
                name
                for name in ("config", "runtime_config", "backend_template")
                if getattr(args, name) is not None
            ]
            if explicit:
                cli.error(
                    "--scenario cannot be combined with "
                    + ", ".join("--" + name.replace("_", "-") for name in explicit)
                )
        else:
            if args.shared_root is not None:
                cli.error("--shared-root requires --scenario")
            args.config = args.config or DEFAULT_CAMPAIGN
            args.runtime_config = args.runtime_config or DEFAULT_RUNTIME_TEACHER
            args.backend_template = args.backend_template or DEFAULT_BACKEND_TEMPLATE
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
    directory = args.project_root / "var/record-pools" / f"pool-{time.time_ns()}"
    docker = DockerWorkers(args, directory)
    def interrupted(signum: int, frame: object) -> None:
        raise KeyboardInterrupt

    previous = signal.signal(signal.SIGTERM, interrupted)
    try:
        plan = docker.preflight()
        if args.target_total is not None and docker.resolve_target(plan):
            return 0
        # Only now may this batch create worker configuration and containers.
        directory.parent.mkdir(parents=True, exist_ok=True)
        args.workers = min(args.workers, args.count)
        workers = docker.prepare(plan)
        print(f"并行 worker：{args.workers}；可视化与预览已禁用；日志：{directory}", flush=True)
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


if __name__ == "__main__":
    raise SystemExit(main())
