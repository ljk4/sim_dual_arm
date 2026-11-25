"""Shared host-only stdlib + Docker CLI utilities for parallel worker pools.

This module runs directly on the host Python interpreter and must never import
ROS 2, Pydantic, PyYAML, or any container-only package.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import json
import os
import re
import shutil
import subprocess
import sys
import time
import unicodedata
from pathlib import Path
from typing import TextIO

ROOT = Path(os.environ.get("PROJECT_ROOT") or Path(__file__).resolve().parents[4])
DIRECT_SEGMENT = "_direct"
RESERVED_ROBOT_SEGMENTS = frozenset({"build"})
_SEGMENT_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}")

CONTAINER_MOUNTS = (
    ("/app", Path("runtime/ros2")),
    ("/shared", Path("shared")),
    ("/build", Path("output/build")),
    ("/data", Path("output")),
)


def _safe_segment(segment: str, label: str) -> str:
    if not _SEGMENT_PATTERN.fullmatch(segment) or ".." in segment:
        raise ValueError(
            f"{label} must contain 1..128 safe ASCII characters and no path traversal"
        )
    if label == "robot" and segment in RESERVED_ROBOT_SEGMENTS:
        raise ValueError(f"robot id {segment!r} is reserved")
    return segment


def scenario_segment(
    robot: str | None, backend: str | None = None, task: str | None = None
) -> str:
    if robot is None:
        if backend is not None or task is not None:
            raise ValueError("backend and task require a robot id")
        return DIRECT_SEGMENT
    if backend is None and task is None:
        if "/" not in robot:
            raise ValueError(
                "backend and task are required unless robot is '<robot>/<backend>/<task>'"
            )
        parts = robot.split("/")
        if len(parts) != 3:
            raise ValueError("scenario string must be '<robot>/<backend>/<task>'")
        robot, backend, task = parts
    elif backend is None or task is None:
        raise ValueError("robot, backend, and task must be passed together")
    return (
        f"{_safe_segment(robot, 'robot')}/"
        f"{_safe_segment(backend, 'backend')}/"
        f"{_safe_segment(task, 'task')}"
    )


def scenario_root(
    base: Path, robot: str | None, backend: str | None = None, task: str | None = None
) -> Path:
    return Path(base) / scenario_segment(robot, backend, task)


def container_data_root() -> Path:
    return Path(os.environ.get("DATA_ROOT", "/data"))


def debug_root(
    base: Path, robot: str | None, backend: str | None = None, task: str | None = None
) -> Path:
    return scenario_root(base, robot, backend, task) / "debug"


def _host_path(value: str | Path, project_root: Path) -> Path:
    path = Path(value)
    if not path.is_absolute():
        path = project_root / path
    return path.resolve()


def host_path_to_container(value: str | Path, project_root: Path) -> str:
    """Map a host path to the runtime container; reject unmounted paths."""
    path = _host_path(value, project_root)
    project_root = project_root.resolve()
    for container, relative in CONTAINER_MOUNTS:
        root = (project_root / relative).resolve()
        if path == root or path.is_relative_to(root):
            return str(Path(container) / path.relative_to(root))
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
    host = (
        _host_path(value, project_root)
        if value is not None
        else (project_root / "shared").resolve()
    )
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
    cpus = (
        len(os.sched_getaffinity(0))
        if hasattr(os, "sched_getaffinity")
        else os.cpu_count() or 1
    )
    limits = [4, max(1, cpus // 4)]
    try:
        memory = next(
            line
            for line in Path("/proc/meminfo").read_text().splitlines()
            if line.startswith("MemAvailable:")
        )
        limits.append(max(1, (int(memory.split()[1]) - 1024 * 1024) // (1536 * 1024)))
    except (OSError, StopIteration, ValueError):
        pass
    try:
        result = subprocess.run(
            ["nvidia-smi", "--query-gpu=memory.free", "--format=csv,noheader,nounits"],
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        )
        if result.returncode == 0:
            limits.append(max(1, (int(result.stdout.splitlines()[0]) - 512) // 1024))
    except (OSError, ValueError, IndexError, subprocess.TimeoutExpired):
        limits.append(2)
    return min(limits)


def host_terminal_columns() -> int:
    """Return host stdout terminal width (minimum 20)."""
    try:
        cols = os.get_terminal_size(sys.stdout.fileno()).columns
        if cols > 0:
            return max(20, cols)
    except (OSError, ValueError, AttributeError):
        pass
    return max(20, shutil.get_terminal_size((80, 24)).columns)


def fit_terminal_line(text: str, max_width: int) -> str:
    """Truncate text so its terminal display width never exceeds max_width."""
    if max_width <= 0:
        return ""
    result: list[str] = []
    used = 0
    for ch in text:
        size = 2 if unicodedata.east_asian_width(ch) in {"W", "F"} else 1
        if used + size > max_width:
            break
        result.append(ch)
        used += size
    return "".join(result)


def format_duration(seconds: float | None) -> str:
    """Format seconds as HH:MM:SS or --:--:-- when unknown."""
    if seconds is None:
        return "--:--:--"
    total = max(0, round(seconds))
    minutes, secs = divmod(total, 60)
    hours, minutes = divmod(minutes, 60)
    return f"{hours:02d}:{minutes:02d}:{secs:02d}"


def ascii_bar(done: int, total: int, width: int = 10) -> str:
    """Render an ASCII progress bar free of ambiguous-width Unicode blocks."""
    safe_total = max(1, total)
    ratio = min(1.0, max(0.0, done / safe_total))
    filled = int(width * ratio)
    return "#" * filled + "-" * (width - filled)


def write_progress_line(
    line: str, *, tty: bool, stream: TextIO | None = None
) -> None:
    """Write a single progress update in-place on TTY or as a plain line otherwise."""
    target = sys.stdout if stream is None else stream
    if tty:
        max_width = max(1, host_terminal_columns() - 1)
        fitted = fit_terminal_line(line, max_width)
        target.write(f"\r\x1b[2K{fitted}")
        target.flush()
    else:
        target.write(f"{line}\n")
        target.flush()


def finish_progress_line(*, tty: bool, stream: TextIO | None = None) -> None:
    """Terminate an active in-place TTY progress line with a newline."""
    if not tty:
        return
    target = sys.stdout if stream is None else stream
    target.write("\n")
    target.flush()


class BaseDockerWorkers:
    """Common Docker orchestration for parallel recording and inference pools."""

    def __init__(
        self,
        args: argparse.Namespace,
        directory: Path,
        *,
        batch_name: str | None = None,
    ) -> None:
        self.args = args
        self.directory = directory
        self.batch_name = batch_name or directory.name
        self.created: list[str] = []
        self.runtime = ""
        self.controller: subprocess.Popen | None = None
        self.controller_pid = directory / "controller.pid"
        self.plan: dict | None = None

    def compose(self, service: str) -> list[str]:
        section = "runtime/ros2" if service == "runtime" else f"simulation/{service}"
        compose = self.args.project_root / section / "docker/docker-compose.yml"
        return ["docker", "compose", "-f", str(compose)]

    def _backend_service(self) -> str:
        selection = (self.plan or {}).get("selection") or {}
        if isinstance(selection, dict) and selection.get("backend"):
            return str(selection["backend"])
        explicit = getattr(self.args, "backend", None)
        if explicit:
            return str(explicit)
        template = getattr(self.args, "backend_template", None)
        if template:
            candidate = Path(template)
            if not candidate.is_absolute():
                candidate = (self.args.project_root / candidate).resolve()
            else:
                candidate = candidate.resolve()
            sim_root = (self.args.project_root / "simulation").resolve()
            if candidate.is_relative_to(sim_root):
                parts = candidate.relative_to(sim_root).parts
                if len(parts) >= 2:
                    return parts[0]
            shared_arg = getattr(self.args, "shared_root", None)
            shared_root = (
                Path(shared_arg).resolve()
                if shared_arg
                else (self.args.project_root / "shared").resolve()
            )
            scenarios_root = shared_root / "scenarios"
            if candidate.is_relative_to(scenarios_root):
                parts = candidate.relative_to(scenarios_root).parts
                if len(parts) >= 3:
                    return parts[1]
        sim_dir = self.args.project_root / "simulation"
        if sim_dir.is_dir():
            backends = [
                entry.name
                for entry in sorted(sim_dir.iterdir())
                if entry.is_dir() and (entry / "docker/docker-compose.yml").is_file()
            ]
            if len(backends) == 1:
                return backends[0]
        raise ValueError(
            "cannot infer simulation backend in direct mode; place --backend-template "
            "under simulation/<backend>/ or shared/scenarios/<robot>/<backend>/<task>/"
        )

    def runtime_command(
        self, arguments: list[str], *, pid_file: str | None = None
    ) -> list[str]:
        shell = "source /shared/common/tools/docker/setup.sh; "
        if pid_file is not None:
            shell += 'echo "$$" > "$1"; shift; '
        return [
            "docker",
            "exec",
            *(
                [
                    "-e",
                    "RECORD_PROGRESS_TTY=1",
                    "-e",
                    f"COLUMNS={host_terminal_columns()}",
                ]
                if sys.stdout.isatty()
                else []
            ),
            self.runtime,
            "bash",
            "-c",
            shell + 'exec "$@"',
            "bash",
            *([pid_file] if pid_file else []),
            *arguments,
        ]

    def ensure_runtime(self) -> None:
        if self.runtime:
            return
        runtime = subprocess.run(
            [*self.compose("runtime"), "ps", "-q", "runtime"],
            check=True,
            text=True,
            capture_output=True,
            timeout=20,
        ).stdout.strip()
        if not runtime:
            (self.args.project_root / "output/build/ros-dev").mkdir(
                parents=True, exist_ok=True
            )
            (self.args.project_root / "output/build/generated/policy-sdk").mkdir(
                parents=True, exist_ok=True
            )
            subprocess.run(
                [
                    *self.compose("runtime"),
                    "up",
                    "-d",
                    "--no-build",
                    "--pull",
                    "never",
                    "runtime",
                ],
                check=True,
                timeout=60,
            )
            runtime = subprocess.run(
                [*self.compose("runtime"), "ps", "-q", "runtime"],
                check=True,
                text=True,
                capture_output=True,
                timeout=20,
            ).stdout.strip()
        self.runtime = runtime

    def _scenario_selection_arguments(self, plan: dict | None) -> list[str] | None:
        if plan is None or plan["selection"]["mode"] != "scenario":
            return None
        return [
            "--robot",
            plan["selection"]["robot"],
            "--backend",
            plan["selection"]["backend"],
            "--task",
            plan["selection"]["task"],
            "--shared-root",
            shared_root_to_container(
                getattr(self.args, "shared_root", None), self.args.project_root
            ),
            "--binding-sha256",
            plan["selection"]["binding"]["sha256"],
        ]

    def _worker_selection_arguments(self, plan: dict | None) -> list[str]:
        scenario = self._scenario_selection_arguments(plan)
        if scenario is not None:
            return scenario
        if getattr(self.args, "robot", None) is not None:
            raise RuntimeError("binding preflight did not return a scenario selection")
        return [
            "--runtime-config",
            host_path_to_container(self.args.runtime_config, self.args.project_root),
            "--backend-template",
            host_path_to_container(self.args.backend_template, self.args.project_root),
        ]

    def start_backend(self, worker: dict) -> str:
        name = f"sim-dual-arm-{self.batch_name}-{worker['id']}"
        self.created.append(name)
        backend_service = self._backend_service()
        overrides = worker.get("overrides") or {}
        speed = overrides.get("realtime_speed", getattr(self.args, "realtime_speed", None))
        command = [
            *self.compose(backend_service),
            "run",
            "--rm",
            "-d",
            "--no-deps",
            "--name",
            name,
            "-e",
            f"ROS_DOMAIN_ID={worker['domain_id']}",
            "-e",
            "OMP_NUM_THREADS=1",
            "-e",
            "OPENBLAS_NUM_THREADS=1",
            "-e",
            "MKL_NUM_THREADS=1",
            "-e",
            "DISPLAY=",
            "-e",
            "WAYLAND_DISPLAY=",
            backend_service,
            "scripts/backend.sh",
            "--config",
            worker["backend_config"],
            "--namespace",
            worker["namespace"],
            *(["--realtime-speed", str(speed)] if speed is not None else []),
            "--no-visualization",
            "--run-root",
            worker["run_root"],
            "--run-id",
            "backend",
        ]
        subprocess.run(
            command,
            check=True,
            capture_output=True,
            text=True,
            timeout=60,
        )
        return name

    def _container_exited(self, name: str) -> bool:
        if name not in self.created:
            return False
        result = subprocess.run(
            ["docker", "inspect", "-f", "{{.State.Running}}", name],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
        )
        if result.returncode != 0:
            return True
        text = (result.stdout or "").strip().lower()
        return text == "false"

    def _worker_ready(self, worker: dict) -> bool:
        ready_raw = worker.get("backend_ready")
        if ready_raw:
            ready_path = container_path_to_host(ready_raw, self.args.project_root)
        else:
            log_path = container_path_to_host(worker["backend_log"], self.args.project_root)
            ready_path = log_path.parent / "ready.json"
        if not ready_path.is_file():
            return False
        try:
            payload = json.loads(ready_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return False
        return isinstance(payload, dict) and payload.get("ready") is True

    def wait_ready(
        self,
        workers: list[dict],
        *,
        timeout_message: str = "parallel backends did not become ready within 60 seconds",
    ) -> None:
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline:
            ready = []
            for worker in workers:
                is_ready = self._worker_ready(worker)
                if not is_ready:
                    name = f"sim-dual-arm-{self.batch_name}-{worker['id']}"
                    if self._container_exited(name):
                        tail = ""
                        try:
                            logs = subprocess.run(
                                ["docker", "logs", "--tail", "20", name],
                                check=False,
                                capture_output=True,
                                text=True,
                                timeout=5,
                            )
                            lines = [
                                line
                                for line in ((logs.stdout or "") + (logs.stderr or "")).splitlines()
                                if line.strip()
                            ]
                            if lines:
                                tail = "\n最近容器输出:\n  " + "\n  ".join(lines[-8:])
                        except (OSError, subprocess.SubprocessError):
                            pass
                        log_file = self.directory / f"{name}.log"
                        raise RuntimeError(
                            f"仿真 worker 容器 {name} 在就绪前意外退出（日志将保存至 {log_file}）{tail}"
                        )
                ready.append(is_ready)
            if all(ready):
                return
            time.sleep(0.25)
        raise RuntimeError(timeout_message)

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
                    check=False,
                    timeout=10,
                    capture_output=True,
                )
                grace = (self.plan or {}).get("stop_grace_s", 0.0)
                self.controller.wait(timeout=max(35.0, grace + 10.0))
            except (OSError, ValueError, subprocess.TimeoutExpired):
                self.controller.terminate()

        def stop(name: str) -> None:
            try:
                logs = subprocess.run(
                    ["docker", "logs", name],
                    text=True,
                    capture_output=True,
                    check=False,
                    timeout=10,
                )
                (self.directory / f"{name}.log").write_text(
                    (logs.stdout or "") + (logs.stderr or "")
                )
            finally:
                subprocess.run(
                    ["docker", "stop", "--timeout", "20", name],
                    check=False,
                    capture_output=True,
                    timeout=30,
                )

        count = max(1, len(self.created))
        with concurrent.futures.ThreadPoolExecutor(max_workers=count) as executor:
            list(executor.map(stop, self.created))


__all__ = [
    "BaseDockerWorkers",
    "CONTAINER_MOUNTS",
    "DIRECT_SEGMENT",
    "ROOT",
    "ascii_bar",
    "automatic_workers",
    "container_data_root",
    "container_path_to_host",
    "debug_root",
    "finish_progress_line",
    "fit_terminal_line",
    "format_duration",
    "host_path_to_container",
    "host_terminal_columns",
    "positive_int",
    "scenario_root",
    "scenario_segment",
    "shared_root_to_container",
    "write_progress_line",
]
