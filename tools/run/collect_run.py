"""Supervise one automatic Compose mock collection and publish its status.

The flow is deliberately linear and readable: prepare -> start -> supervise ->
verify runtime logs -> independent audit -> stop backend -> cleanup -> status.
Docker/container identity checks are reused from ``tools/run/mock.py``.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import signal
import sys
import time
import uuid
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from datetime import UTC, datetime
from pathlib import Path
from types import SimpleNamespace
from typing import Any

_TOOLS_RUN = Path(__file__).resolve().parent
if str(_TOOLS_RUN) not in sys.path:
    sys.path.insert(0, str(_TOOLS_RUN))
_RUN_LOGGING_SRC = Path(__file__).resolve().parents[2] / "libraries/run_logging/src"
if str(_RUN_LOGGING_SRC) not in sys.path:
    sys.path.insert(0, str(_RUN_LOGGING_SRC))

import collect_artifacts  # noqa: E402
import collect_config  # noqa: E402
import dev_compose  # noqa: E402
import mock_stack  # noqa: E402
from robot_run_logging import EventLogger, LogSettings  # noqa: E402

ROOT = dev_compose.ROOT
DEFAULT_CONFIG = ROOT / "configs/deploy/mock_collect.json"
PROJECT_PREFIX = "sim-dual-arm-collect"
EXIT_FAILURE = 1
EXIT_CONFIG = 2
EXIT_TIMEOUT = 124
EXIT_SUCCESS = 0
SIGNAL_EXIT_CODES = {"SIGINT": 130, "SIGTERM": 143}


class UsageError(Exception):
    """Invalid host invocation; no Docker command has been started."""


BACKEND_NORMAL_EXIT_CODES = frozenset({0, 143})


class LauncherError(RuntimeError):
    """A stable, reportable launcher failure with an optional detail text."""

    def __init__(self, reason: str, detail: str = "") -> None:
        super().__init__(detail or reason)
        self.reason = reason
        self.detail = detail


@dataclass
class StopIntent:
    """Signal handlers only set this value; the main loop owns all side effects."""

    reason: str | None = None

    @property
    def exit_code(self) -> int | None:
        return SIGNAL_EXIT_CODES.get(self.reason or "")

    def request(self, signum: int) -> None:
        if self.reason is None:
            self.reason = "SIGTERM" if signum == signal.SIGTERM else "SIGINT"

    def handler(self, signum: int, _frame: object) -> None:
        self.request(signum)


@dataclass
class Arguments:
    project: str
    image: str
    build_root: Path
    raw_root: Path
    runs_root: Path
    run_id: str
    config: Path
    docker: str
    probe_backend: bool
    output: Path | None


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", help="fresh Compose project; defaults to an isolated name")
    parser.add_argument("--image", default=os.environ.get("DEV_IMAGE", dev_compose.DEFAULT_IMAGE))
    parser.add_argument("--build-root", type=Path, help="host generated ROS build root")
    parser.add_argument("--raw-root", type=Path, help="host parent for raw run data")
    parser.add_argument("--runs-root", type=Path, help="host parent for run logs and status")
    parser.add_argument("--run-id", help="fresh run identity; defaults to a new id")
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help="explicit JSON launcher recipe path",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="legacy mapping: one raw run directory (<raw-root>/<run-id>)",
    )
    parser.add_argument(
        "--probe-backend",
        action="store_true",
        help="run the public mock control probe before starting --once runtime",
    )
    parser.add_argument("--docker", default=os.environ.get("DEV_DOCKER", "docker"))
    return parser


def _absolute(value: Path, default: Path | None = None) -> Path:
    path = default if value is None and default is not None else value
    assert path is not None
    if not path.is_absolute():
        path = ROOT / path
    return path.resolve()


def _validate_root(path: Path, label: str) -> Path:
    try:
        return dev_compose._validate_output_root(path, label)
    except dev_compose.DevComposeError as error:
        raise UsageError(str(error)) from error


def _validate_run_id(value: str) -> str:
    try:
        return dev_compose.validate_run_id(value)
    except dev_compose.DevComposeError as error:
        raise UsageError(str(error)) from error


def _new_run_id() -> str:
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
    return f"collect-{stamp}-{uuid.uuid4().hex[:6]}"


def _arguments(argv: Sequence[str] | None) -> Arguments:
    parsed = _parser().parse_args(argv)
    run_id = parsed.run_id
    output = parsed.output
    raw_root = parsed.raw_root
    explicit_run = run_id is not None
    explicit_raw = raw_root is not None
    if output is not None:
        output = _absolute(output)
        if os.path.lexists(output):
            raise UsageError(
                f"legacy --output maps to one raw run directory and already exists: {output}"
            )
        if explicit_run and run_id != output.name:
            raise UsageError("--output final directory name must equal --run-id")
        if explicit_raw and _absolute(raw_root) != output.parent:
            raise UsageError("--output conflicts with the explicit --raw-root")
        run_id = run_id or output.name
        raw_root = output.parent
    run_id = _validate_run_id(run_id or _new_run_id())
    raw_root = _validate_root(_absolute(raw_root, dev_compose.DEFAULT_RAW_ROOT), "raw output root")
    runs_root = _validate_root(
        _absolute(parsed.runs_root, dev_compose.DEFAULT_RUNS_ROOT), "runs output root"
    )
    build_root = _validate_root(
        _absolute(parsed.build_root, dev_compose.DEFAULT_BUILD_ROOT), "build root"
    )
    project = parsed.project or f"{PROJECT_PREFIX}-{uuid.uuid4().hex[:8]}"
    if not re.fullmatch(r"[a-z0-9][a-z0-9-]{1,60}", project):
        raise UsageError(
            "project must contain 2-61 lowercase letters, digits or hyphens and start alphanumeric"
        )
    if output is not None and (runs_root / run_id).resolve() == output.resolve():
        raise UsageError(
            "legacy --output would collide with the run log directory; use "
            "--output var/data/raw/<run-id> or pass a separate --runs-root"
        )
    return Arguments(
        project=project,
        image=parsed.image,
        build_root=build_root,
        raw_root=raw_root,
        runs_root=runs_root,
        run_id=run_id,
        config=_absolute(parsed.config),
        docker=parsed.docker,
        probe_backend=bool(parsed.probe_backend),
        output=output,
    )


def _log_settings(config: collect_config.CollectConfig) -> LogSettings:
    budget = int(config.values["launcher_log_max_bytes"])
    return LogSettings(
        queue_capacity=int(config.values["launcher_log_queue_capacity"]),
        max_event_bytes=int(config.values["launcher_log_max_event_bytes"]),
        max_file_bytes=budget,
        max_total_bytes=budget,
        close_timeout_s=float(config.values["launcher_log_close_timeout_s"]),
        flush_interval_s=0.2,
        console=False,
    )


def _tail(value: object, limit: int = 2000) -> str:
    text = "" if value is None else str(value).strip()
    return text if len(text) <= limit else text[-limit:]


class Collector:
    """One launcher instance owning one isolated Compose project and run identity."""

    def __init__(
        self,
        arguments: Arguments,
        config: collect_config.CollectConfig,
        *,
        docker: mock_stack.BoundedDocker,
        stop_intent: StopIntent,
        clock: Callable[[], float] = time.monotonic,
        sleeper: Callable[[float], None] = time.sleep,
        logger_factory: Callable[..., EventLogger] = EventLogger,
    ) -> None:
        self.arguments = arguments
        self.config = config
        self.docker = docker
        self.stop_intent = stop_intent
        self.clock = clock
        self.sleeper = sleeper
        self.logger_factory = logger_factory

        self.run_dir = arguments.runs_root / arguments.run_id
        self.raw_dir = arguments.raw_root / arguments.run_id
        self.launch_dir = arguments.runs_root / ".launch" / arguments.run_id
        self.launcher_instance = str(uuid.uuid4())
        self.started_mono = clock()
        self.started_utc = collect_artifacts.utc_now()
        self.deadline = self.started_mono + config.work_deadline_s

        self.environment: dict[str, str] = {}
        self.expected: mock_stack.ExpectedStack | None = None
        self.runtime: mock_stack.Container | None = None
        self.backend: mock_stack.Container | None = None
        self.runtime_exit: int | None = None
        self.backend_exit: int | None = None
        self.runtime_image_id: str | None = None
        self.backend_image_id: str | None = None
        self.stop_deadline: float | None = None
        self.cleanup_deadline: float | None = None
        self.tool_ids: set[str] = set()
        self.foreign_resources: list[dict[str, str]] = []
        self.backend_stopped_by_launcher = False
        self.observed_runtime_running = False
        self.runtime_finished = False
        self.runtime_confirmed_stopped = False
        self.start_attempted = False
        self.backend_stop_attempted = False
        self.cleanup_attempted = False
        self.committed = False
        self.launch_dir_created = False

        self.logger: EventLogger | None = None
        self.logger_close_failed = False
        self.logger_ready = False
        self.errors: list[dict[str, str]] = []
        self.reason: str | None = None
        self.phase = "prepare"
        self.audit: dict[str, Any] = {
            "passed": False,
            "path": None,
            "summary_path": None,
            "problems": [],
        }
        self.recording: dict[str, Any] = {
            "complete": False,
            "episode_id": None,
            "path": None,
        }
        self.runtime_dimension: dict[str, Any] = {"outcome": "unknown", "reason": ""}
        self.task_dimension: dict[str, Any] = {"outcome": "not_evaluated", "reason": ""}
        self.logs: dict[str, Any] = {"runtime_drained": False, "launcher_closed": False}
        self.cleanup: dict[str, Any] = {"complete": False, "removed": [], "reason": ""}
        self.launcher_metadata: dict[str, Any] = {}
        self.source_start: dict[str, Any] | None = None
        self.status_write_error: str | None = None
        self.compose_up_diagnostic: dict[str, Any] | None = None
        self.runtime_stdout_text = ""

    def _stop_budget(self) -> float:
        if self.stop_deadline is None:
            self.stop_deadline = self.clock() + self.config.stop_grace_s + 2.0
        return self.stop_deadline

    def _cleanup_budget(self) -> float:
        if self.cleanup_deadline is None:
            self.cleanup_deadline = self.clock() + self.config.cleanup_timeout_s
        return self.cleanup_deadline

    def _cache_container(self, service: str, container: mock_stack.Container | None) -> None:
        if container is None:
            return
        image_id = container.image_id or self._container_image_id(container)
        if service == "runtime":
            self.runtime_image_id = image_id or self.runtime_image_id
        else:
            self.backend_image_id = image_id or self.backend_image_id

    def _capture(
        self,
        name: str,
        arguments: Sequence[str],
        deadline: float,
        *,
        environment: dict[str, str] | None = None,
        argv: Sequence[str] | None = None,
    ):
        if argv is None:
            try:
                completed = self.docker.run(arguments, deadline, environment=environment)
            except mock_stack.HostError as error:
                return None, error
        else:
            try:
                completed = self.docker.run_argv(argv, deadline, environment=environment)
            except mock_stack.HostError as error:
                return None, error
        text = (completed.stdout or "") + (completed.stderr or "")
        if getattr(completed, "truncated", False):
            text += (
                "\n...[command output truncated; "
                f"stdout_bytes={getattr(completed, 'stdout_bytes', '?')}; "
                f"stderr_bytes={getattr(completed, 'stderr_bytes', '?')}]"
            )
        try:
            info = collect_artifacts.write_diagnostic(
                self._diagnostic_path(name),
                text,
                self.config.diagnostic_max_bytes,
            )
        except OSError as error:
            self._fail(self.phase, "DIAGNOSTIC_WRITE_FAILED", str(error))
            info = None
        if info is not None and getattr(completed, "truncated", False):
            info.update(
                {
                    "command_output_truncated": True,
                    "command_stdout_bytes": getattr(completed, "stdout_bytes", None),
                    "command_stderr_bytes": getattr(completed, "stderr_bytes", None),
                }
            )
        return (completed, text, info), None

    def _capture_container_logs(
        self, service: str, container: mock_stack.Container, deadline: float | None = None
    ) -> None:
        if not self.committed and not self.launch_dir_created:
            return
        result = self._capture(
            f"{service}.log",
            ["logs", "--tail", "400", container.identifier],
            deadline
            if deadline is not None
            else min(self.deadline, self.clock() + self.config.command_timeout_s),
        )
        if result is not None:
            payload, error = result
            if error is not None:
                self._fail(
                    "diagnostics", "LOG_CAPTURE_FAILED", getattr(error, "detail", str(error))
                )
            elif payload is not None:
                completed, text, _info = payload
                if completed.returncode != 0:
                    self._fail("diagnostics", "LOG_CAPTURE_FAILED", _tail(completed.stderr))
                if service == "runtime":
                    self.runtime_stdout_text = text

    def _exit_code(
        self, container: mock_stack.Container | None, deadline: float | None = None
    ) -> int | None:
        """Read ExitCode from raw state so a missing value stays null, not zero."""
        if container is None or container.running:
            return None
        try:
            completed = self.docker.run(
                ["inspect", "--format", "{{json .State}}", container.identifier],
                deadline
                if deadline is not None
                else min(self.deadline, self.clock() + self.config.command_timeout_s),
            )
        except mock_stack.HostError:
            return None
        try:
            state = json.loads(completed.stdout or "")
        except ValueError:
            return None
        if not isinstance(state, dict):
            return None
        value = state.get("ExitCode")
        if isinstance(value, bool) or not isinstance(value, int):
            return None
        return value

    def _container_image_id(self, container: mock_stack.Container | None) -> str | None:
        if container is None:
            return None
        try:
            completed = self.docker.run(
                ["inspect", "--format", "{{.Image}}", container.identifier],
                min(self.deadline, self.clock() + self.config.command_timeout_s),
            )
        except mock_stack.HostError:
            return None
        value = (completed.stdout or "").strip()
        return value if completed.returncode == 0 and value else None

    def _inspect_named(self, name: str, deadline: float) -> mock_stack.Container | None:
        try:
            found = mock_stack.inspect_containers(self.docker, [name], deadline, allow_missing=True)
        except mock_stack.HostError:
            return None
        if len(found) != 1:
            return None
        container = found[0]
        if container.project != self.arguments.project:
            return None
        if container.service not in {"runtime", "backend"}:
            return None
        if container.name != name.lstrip("/") or container.image != self.arguments.image:
            return None
        return container

    def _remove_ids(self, identifiers: Sequence[str], deadline: float) -> list[str]:
        removed: list[str] = []
        for identifier in identifiers:
            try:
                completed = self.docker.run(["rm", "-f", identifier], deadline)
            except mock_stack.HostError as error:
                self._fail("cleanup", error.reason, error.detail)
                continue
            if completed.returncode == 0:
                removed.append(identifier)
                self.tool_ids.discard(identifier)
            else:
                self._fail("cleanup", "CLEANUP_REMOVE_FAILED", _tail(completed.stderr))
        return removed

    def _remove_named(self, name: str, deadline: float | None = None) -> bool:
        limit = self._cleanup_budget() if deadline is None else deadline
        container = self._inspect_named(name, limit)
        if container is None:
            return False
        self.tool_ids.add(container.identifier)
        return bool(self._remove_ids([container.identifier], limit))

    def _stop_runtime(self) -> None:
        if self.runtime is None or not self.runtime.running:
            return
        self.phase = "stop"
        deadline = self._stop_budget()
        self._event(
            "stop.requested",
            reason=self.reason or "STOP",
            fields={"service": "runtime", "container_id": self.runtime.identifier[:12]},
        )
        forced = False
        try:
            completed = self.docker.run(
                [
                    "stop",
                    "--time",
                    str(max(1, math.ceil(self.config.stop_grace_s))),
                    self.runtime.identifier,
                ],
                deadline,
            )
        except mock_stack.HostError as error:
            forced = True
            self._fail("stop", "RUNTIME_STOP_FAILED", error.detail)
            completed = None
        if completed is not None and completed.returncode != 0:
            forced = True
            self._fail("stop", "RUNTIME_STOP_FAILED", _tail(completed.stderr))
        try:
            refreshed = mock_stack.inspect_containers(
                self.docker, [self.runtime.identifier], deadline, allow_missing=True
            )
        except mock_stack.HostError as error:
            self._fail("stop", "RUNTIME_STOP_INSPECT_FAILED", error.detail)
            refreshed = []
        if refreshed:
            self.runtime = refreshed[0]
            self._cache_container("runtime", self.runtime)
        if self.runtime.running or forced:
            try:
                self.docker.run(["kill", self.runtime.identifier], deadline)
            except mock_stack.HostError as error:
                self._fail("stop", "RUNTIME_KILL_FAILED", error.detail)
            try:
                refreshed = mock_stack.inspect_containers(
                    self.docker, [self.runtime.identifier], deadline, allow_missing=True
                )
            except mock_stack.HostError:
                refreshed = []
            if refreshed:
                self.runtime = refreshed[0]
                self._cache_container("runtime", self.runtime)
        self.runtime_exit = self._exit_code(self.runtime, deadline)
        if self.runtime is not None:
            self._capture_container_logs("runtime", self.runtime, deadline)
        if self.runtime is None or not self.runtime.running:
            self.runtime_confirmed_stopped = True
        if forced or self.runtime_exit in {137, None}:
            self._fail(
                "stop",
                "RUNTIME_FORCED_STOP",
                f"runtime exit code after bounded stop is {self.runtime_exit!r}",
            )
        self._event(
            "stop.completed",
            reason="OK" if not forced else "FORCED",
            level="INFO" if not forced else "ERROR",
            fields={
                "service": "runtime",
                "exit_code": self.runtime_exit,
                "forced": forced,
            },
        )

    def _stop_backend(self) -> None:
        if self.backend_stop_attempted:
            return
        self.backend_stop_attempted = True
        if self.backend is None:
            return
        self.phase = "stop"
        deadline = self._stop_budget()
        try:
            refreshed = mock_stack.inspect_containers(
                self.docker, [self.backend.identifier], deadline, allow_missing=True
            )
        except mock_stack.HostError as error:
            self._fail("stop", "BACKEND_STOP_INSPECT_FAILED", error.detail)
            return
        if refreshed:
            self.backend = refreshed[0]
            self._cache_container("backend", self.backend)
        if self.backend.running:
            self.backend_stopped_by_launcher = True
            try:
                completed = self.docker.run(
                    [
                        "stop",
                        "--time",
                        str(max(1, math.ceil(self.config.stop_grace_s))),
                        self.backend.identifier,
                    ],
                    deadline,
                )
            except mock_stack.HostError as error:
                self._fail("stop", "BACKEND_STOP_FAILED", error.detail)
                completed = None
            if completed is not None and completed.returncode != 0:
                self._fail("stop", "BACKEND_STOP_FAILED", _tail(completed.stderr))
            try:
                refreshed = mock_stack.inspect_containers(
                    self.docker, [self.backend.identifier], deadline, allow_missing=True
                )
            except mock_stack.HostError as error:
                self._fail("stop", "BACKEND_STOP_INSPECT_FAILED", error.detail)
                refreshed = []
            if refreshed:
                self.backend = refreshed[0]
                self._cache_container("backend", self.backend)
        self.backend_exit = self._exit_code(self.backend, deadline)
        if self.backend is not None:
            self._capture_container_logs("backend", self.backend, deadline)
        if not self.backend_stopped_by_launcher:
            self._fail(
                "stop",
                "BACKEND_EXITED_BEFORE_STOP",
                "backend was already stopped before the supervisor's normal stop",
            )
        if self.backend_exit not in BACKEND_NORMAL_EXIT_CODES:
            self._fail(
                "stop",
                "BACKEND_STOP_UNEXPECTED_EXIT",
                f"backend exit code {self.backend_exit!r} is not a normal stop (0/143)",
            )
        self._event(
            "stop.completed",
            reason="OK" if self.backend_exit in BACKEND_NORMAL_EXIT_CODES else "UNEXPECTED_EXIT",
            level="INFO" if self.backend_exit in BACKEND_NORMAL_EXIT_CODES else "ERROR",
            fields={
                "service": "backend",
                "exit_code": self.backend_exit,
                "stopped_by_launcher": self.backend_stopped_by_launcher,
            },
        )

    def _cleanup(self) -> None:
        if self.cleanup_attempted:
            return
        self.cleanup_attempted = True
        self.phase = "cleanup"
        deadline = self._cleanup_budget()
        removed: list[str] = []
        networks_removed: list[str] = []
        foreign: list[dict[str, str]] = []
        remaining: list[str] = []
        remaining_owned: list[str] = []
        try:
            # Recover identified runtime/backend after a startup failure without
            # touching containers that do not match this run's identity.
            try:
                identifiers = mock_stack.project_ids(self.docker, self.arguments.project, deadline)
                containers = mock_stack.inspect_containers(
                    self.docker, identifiers, deadline, allow_missing=True
                )
            except mock_stack.HostError as error:
                self._fail("cleanup", error.reason, error.detail)
                containers = []
            for container in containers:
                if container.service not in {"runtime", "backend"}:
                    continue
                if self.expected is None:
                    continue
                try:
                    mock_stack.validate_container(container, self.expected, container.service)
                except mock_stack.HostError:
                    continue
                if container.service == "backend" and "mock_robot_backend.node" not in (
                    container.command
                ):
                    continue
                self._cache_container(container.service, container)
                if container.service == "runtime" and self.runtime is None:
                    self.runtime = container
                if container.service == "backend" and self.backend is None:
                    self.backend = container
            owned: list[str] = []
            for container in (self.runtime, self.backend):
                if container is not None and container.identifier not in owned:
                    owned.append(container.identifier)
            for identifier in sorted(self.tool_ids):
                if identifier not in owned:
                    owned.append(identifier)
            removed.extend(self._remove_ids(owned, deadline))
            remaining = mock_stack.project_ids(self.docker, self.arguments.project, deadline)
            remaining_owned = [identifier for identifier in remaining if identifier in owned]
            for identifier in remaining:
                if identifier not in owned:
                    foreign.append({"kind": "container", "id": identifier})
            if remaining_owned:
                self._fail(
                    "cleanup",
                    "CLEANUP_REMOVE_FAILED",
                    f"{len(remaining_owned)} owned container(s) still present",
                )
            try:
                completed = self.docker.run(
                    [
                        "network",
                        "ls",
                        "--filter",
                        f"label=com.docker.compose.project={self.arguments.project}",
                        "--format",
                        "{{.ID}}",
                    ],
                    deadline,
                )
            except mock_stack.HostError as error:
                self._fail("cleanup", error.reason, error.detail)
                networks: list[str] = []
            else:
                networks = [
                    line.strip() for line in (completed.stdout or "").splitlines() if line.strip()
                ]
            for network in networks:
                try:
                    inspected = self.docker.run(
                        ["network", "inspect", "--format", "{{json .Containers}}", network],
                        deadline,
                    )
                except mock_stack.HostError as error:
                    self._fail("cleanup", error.reason, error.detail)
                    foreign.append({"kind": "network", "id": network})
                    continue
                try:
                    endpoints = json.loads(inspected.stdout or "{}")
                except ValueError:
                    self._fail("cleanup", "NETWORK_INSPECT_FAILED", network)
                    foreign.append({"kind": "network", "id": network})
                    continue
                if not isinstance(endpoints, dict):
                    self._fail("cleanup", "NETWORK_INSPECT_FAILED", network)
                    foreign.append({"kind": "network", "id": network})
                    continue
                outside = sorted(set(endpoints) - set(owned))
                if outside:
                    foreign.append(
                        {
                            "kind": "network-endpoint",
                            "id": network,
                            "containers": ",".join(outside),
                        }
                    )
                    continue
                try:
                    result = self.docker.run(["network", "rm", network], deadline)
                except mock_stack.HostError as error:
                    self._fail("cleanup", error.reason, error.detail)
                    foreign.append({"kind": "network", "id": network})
                    continue
                if result.returncode == 0:
                    networks_removed.append(network)
                else:
                    self._fail("cleanup", "NETWORK_REMOVE_FAILED", _tail(result.stderr))
                    foreign.append({"kind": "network", "id": network})
            complete = not foreign and not remaining_owned
        except mock_stack.HostError as error:
            self._fail("cleanup", error.reason, error.detail)
            complete = False
        cleanup_reason = ""
        if not complete:
            if remaining_owned:
                cleanup_reason = "CLEANUP_REMOVE_FAILED"
            elif foreign:
                cleanup_reason = "CLEANUP_FOREIGN_RESOURCE"
            else:
                cleanup_reason = self.reason or "CLEANUP_INCOMPLETE"
        self.cleanup = {
            "complete": complete,
            "removed": removed,
            "remaining": list(remaining),
            "networks_removed": networks_removed,
            "foreign": foreign,
            "reason": cleanup_reason,
        }
        self.foreign_resources = foreign
        if not complete:
            if remaining_owned:
                detail = f"{len(remaining_owned)} owned container(s) still present"
            elif foreign:
                detail = f"left {len(foreign)} unowned resource(s) untouched"
            else:
                detail = "cleanup did not finish within its bounded budget"
            self._fail("cleanup", cleanup_reason, detail)
        self._event(
            "cleanup.completed",
            reason="OK" if complete else "FOREIGN_RESOURCE",
            level="INFO" if complete else "ERROR",
            fields={
                "complete": complete,
                "removed_containers": len(removed),
                "remaining_containers": len(remaining),
                "foreign_resources": len(foreign),
            },
        )

    def _wait_backend_only(self) -> None:
        deadline = min(self.deadline, self.clock() + self.config.command_timeout_s)
        while True:
            try:
                identifiers = mock_stack.project_ids(self.docker, self.arguments.project, deadline)
                containers = mock_stack.inspect_containers(
                    self.docker, identifiers, deadline, allow_missing=True
                )
            except mock_stack.HostError as error:
                raise LauncherError(error.reason, error.detail) from error
            candidates = [item for item in containers if item.service == "backend"]
            if candidates:
                if self.expected is None:
                    raise LauncherError("INTERNAL_ERROR", "expected stack is not prepared")
                mock_stack.validate_container(candidates[0], self.expected, "backend")
                if candidates[0].running:
                    self.backend = candidates[0]
                    return
            if self.clock() >= deadline:
                raise LauncherError("BACKEND_NOT_RUNNING", "backend did not reach running state")
            self.sleeper(min(self.config.poll_interval_s, max(0.0, deadline - self.clock())))

    def _probe_backend(self) -> None:
        self.phase = "probe"
        deadline = min(self.deadline, self.clock() + self.config.command_timeout_s)
        composed, error = self._compose(
            "compose-probe-up.log",
            ["up", "-d", "--no-build", "--no-deps", "backend"],
            deadline,
        )
        if error is not None:
            raise LauncherError(error.reason, error.detail) from error
        assert composed is not None
        completed, _text, _info = composed
        if completed.returncode != 0:
            raise LauncherError(
                "PROBE_BACKEND_START_FAILED",
                _tail(completed.stderr) or "docker compose up backend failed",
            )
        self._wait_backend_only()
        name = f"collect-probe-{uuid.uuid4().hex[:10]}"
        deadline = min(self.deadline, self.clock() + self.config.command_timeout_s)
        result, error = self._compose(
            "probe.log",
            [
                "run",
                "--rm",
                "--no-deps",
                "-T",
                "--name",
                name,
                "backend",
                "python",
                "/workspace/tests/integration/probe_backend.py",
            ],
            deadline,
        )
        tool_deadline = min(self.deadline, self.clock() + self.config.command_timeout_s)
        if error is not None:
            self._remove_named(name, tool_deadline)
            raise LauncherError(error.reason, error.detail) from error
        assert result is not None
        probe, text, _info = result
        if probe.returncode != 0:
            self._remove_named(name, tool_deadline)
            raise LauncherError(
                "PROBE_BACKEND_FAILED",
                _tail(text) or "public backend probe failed",
            )
        # --rm normally removes the one-off; explicit removal keeps a failed
        # removal from being mistaken for a second backend container.
        self._remove_named(name, tool_deadline)
        self._event("probe.completed", reason="OK", message="public mock backend probe passed")

    def _wait_stack(self) -> None:
        self.phase = "start"
        deadline = min(self.deadline, self.clock() + self.config.command_timeout_s)
        temporary = {"STACK_NOT_FOUND", "RUNTIME_NOT_FOUND", "BACKEND_NOT_FOUND"}
        while True:
            try:
                containers, runtime, backend = mock_stack.resolve_stack(
                    self.docker, self.expected, deadline
                )
            except mock_stack.HostError as error:
                if (
                    error.reason in temporary
                    and self.clock() < deadline
                    and self.stop_intent.reason is None
                ):
                    self.sleeper(
                        min(self.config.poll_interval_s, max(0.0, deadline - self.clock()))
                    )
                    continue
                if error.reason in temporary:
                    raise LauncherError(
                        "STARTUP_TIMEOUT", "containers did not appear in time"
                    ) from error
                raise LauncherError(error.reason, error.detail) from error
            self.runtime, self.backend = runtime, backend
            self._cache_container("runtime", runtime)
            self._cache_container("backend", backend)
            if "--once" not in runtime.command:
                raise LauncherError("RUNTIME_COMMAND_MISMATCH", "runtime container is not --once")
            if runtime.running and backend.running:
                self.observed_runtime_running = True
                self._event(
                    "container.created",
                    reason="OK",
                    fields={
                        "runtime_id": runtime.identifier[:12],
                        "backend_id": backend.identifier[:12],
                        "image_id": self.launcher_metadata.get("image", {}).get("id"),
                    },
                )
                return
            if not runtime.running:
                self.runtime_confirmed_stopped = True
                self.runtime_exit = self._exit_code(runtime)
                self._capture_container_logs("runtime", runtime)
                if self.runtime_exit == 0 and backend.running:
                    self.runtime_finished = True
                    self._event(
                        "container.exited",
                        reason="OK",
                        message="runtime completed before the first readiness poll",
                        fields={
                            "runtime_id": runtime.identifier[:12],
                            "exit_code": 0,
                        },
                    )
                    return
                raise LauncherError(
                    "RUNTIME_EXITED_EARLY",
                    f"runtime is {runtime.state or 'missing'} with exit code {self.runtime_exit!r}",
                )
            if not backend.running:
                self.backend_exit = self._exit_code(backend)
                self._capture_container_logs("backend", backend)
                raise LauncherError(
                    "BACKEND_EXITED",
                    f"backend is {backend.state or 'missing'} before runtime finished",
                )
            if self.clock() >= deadline:
                raise LauncherError("STARTUP_TIMEOUT", "stack did not become ready")
            self.sleeper(min(self.config.poll_interval_s, max(0.0, deadline - self.clock())))

    def _refresh(self) -> dict[str, mock_stack.Container | None]:
        identifiers = {
            "runtime": None if self.runtime is None else self.runtime.identifier,
            "backend": None if self.backend is None else self.backend.identifier,
        }
        present = [value for value in identifiers.values() if value]
        try:
            found = mock_stack.inspect_containers(
                self.docker,
                present,
                min(self.deadline, self.clock() + self.config.command_timeout_s),
                allow_missing=True,
            )
        except mock_stack.HostError as error:
            raise LauncherError(error.reason, error.detail) from error
        by_id = {item.identifier: item for item in found}
        result: dict[str, mock_stack.Container | None] = {"runtime": None, "backend": None}
        for service, identifier in identifiers.items():
            if identifier is None:
                continue
            container = by_id.get(identifier)
            if container is not None and self.expected is not None:
                mock_stack.validate_container(container, self.expected, service)
            if container is not None:
                self._cache_container(service, container)
            result[service] = container
        return result

    def _supervise(self) -> None:
        self.phase = "supervise"
        while True:
            if not self.committed:
                self._observe_run_dir()
            if self.stop_intent.reason is not None:
                raise LauncherError(self.stop_intent.reason, "termination signal received")
            if self.clock() >= self.deadline:
                raise LauncherError(
                    "WORK_DEADLINE_EXCEEDED",
                    f"work deadline of {self.config.work_deadline_s:g}s expired",
                )
            refreshed = self._refresh()
            runtime, backend = refreshed["runtime"], refreshed["backend"]
            if runtime is None:
                raise LauncherError("RUNTIME_LOST", "runtime container disappeared during the run")
            self.runtime = runtime
            if runtime.running:
                self.observed_runtime_running = True
            if not runtime.running:
                self.runtime_confirmed_stopped = True
                self.runtime_exit = self._exit_code(runtime)
                self._capture_container_logs("runtime", runtime)
                self._event(
                    "container.exited",
                    reason="OK" if self.runtime_exit == 0 else "RUNTIME_EXIT_NONZERO",
                    level="INFO" if self.runtime_exit == 0 else "ERROR",
                    fields={
                        "runtime_id": runtime.identifier[:12],
                        "exit_code": self.runtime_exit,
                    },
                )
                if backend is None or not backend.running:
                    self.backend_exit = self._exit_code(backend)
                    raise LauncherError("BACKEND_EXITED", "backend stopped before runtime finished")
                self.runtime_finished = True
                if self.runtime_exit != 0:
                    raise LauncherError(
                        "RUNTIME_EXIT_NONZERO",
                        f"runtime exited with code {self.runtime_exit!r}",
                    )
                return
            if backend is None:
                raise LauncherError("BACKEND_LOST", "backend container disappeared")
            self.backend = backend
            if not backend.running:
                self.backend_exit = self._exit_code(backend)
                self._capture_container_logs("backend", backend)
                raise LauncherError("BACKEND_EXITED", "backend exited before runtime finished")
            self.sleeper(self.config.poll_interval_s)

    def _status_document(
        self,
        *,
        state: str,
        reason: str,
        exit_code: int | None,
        finished: bool,
        phase: str,
    ) -> dict[str, Any]:
        now = self.clock()
        return {
            "schema_version": 1,
            "run_id": self.arguments.run_id,
            "status": state,
            "phase": phase,
            "reason": reason,
            "started_at_utc": self.started_utc,
            "finished_at_utc": collect_artifacts.utc_now() if finished else None,
            "elapsed_ms": int(round((now - self.started_mono) * 1000)) if finished else None,
            "exit_code": exit_code,
            "components": {
                "runtime": {
                    "container_id": None if self.runtime is None else self.runtime.identifier,
                    "exit_code": self.runtime_exit,
                    "running": bool(
                        self.runtime is not None
                        and self.runtime.running
                        and not self.cleanup.get("complete")
                    ),
                },
                "backend": {
                    "container_id": None if self.backend is None else self.backend.identifier,
                    "exit_code": self.backend_exit,
                    "running": bool(
                        self.backend is not None
                        and self.backend.running
                        and not self.cleanup.get("complete")
                    ),
                    "stopped_by_launcher": self.backend_stopped_by_launcher,
                },
            },
            "recording": dict(self.recording),
            "runtime": dict(self.runtime_dimension),
            "task": dict(self.task_dimension),
            "audit": dict(self.audit),
            "logs": dict(self.logs),
            "cleanup": dict(self.cleanup),
            "errors": list(self.errors),
        }

    def _publish_running(self, phase: str) -> None:
        self.phase = phase
        document = self._status_document(
            state="running",
            reason="STARTING",
            exit_code=None,
            finished=False,
            phase=phase,
        )
        try:
            collect_artifacts.atomic_write_json(self._evidence_dir() / "status.json", document)
        except OSError as error:
            self.status_write_error = f"{type(error).__name__}: {error}"
            self._fail(phase, "STATUS_WRITE_FAILED", self.status_write_error)

    def _write_launcher(self) -> None:
        if self.source_start is None:
            self.source_start = collect_artifacts.source_manifest(ROOT)
        config_values = self.config.document()
        self.launcher_metadata = {
            "schema_version": 1,
            "run_id": self.arguments.run_id,
            "project": self.arguments.project,
            "launcher_instance_id": self.launcher_instance,
            "source": {
                "root": str(ROOT),
                "start_sha256": self.source_start["sha256"],
                "file_count": self.source_start["file_count"],
                "files": self.source_start["files"],
                "contracts_sha256": collect_artifacts.contracts_digest(self.source_start),
                "git": collect_artifacts.git_reference(ROOT),
            },
            "configuration": {
                "path": str(self.arguments.config),
                "sha256": self.config.sha256,
                "values": config_values,
            },
            "image": {
                "reference": self.arguments.image,
                "id": self.launcher_metadata.get("image", {}).get("id"),
            },
            "mounts": {
                "build_root": str(self.arguments.build_root),
                "raw_root": str(self.arguments.raw_root),
                "runs_root": str(self.arguments.runs_root),
            },
        }
        try:
            collect_artifacts.atomic_write_json(
                self._evidence_dir() / "launcher.json", self.launcher_metadata
            )
        except OSError as error:
            self._fail("prepare", "LAUNCHER_EVIDENCE_WRITE_FAILED", str(error))

    def _ensure_failure_evidence(self) -> None:
        """Keep a launch directory when Docker itself was unreachable."""
        if self.committed or self.launch_dir_created:
            return
        if self.reason not in {"DOCKER_UNAVAILABLE", "DOCKER_ERROR", "COMMAND_TIMEOUT"}:
            return
        if any(os.path.lexists(path) for path in (self.launch_dir, self.run_dir, self.raw_dir)):
            return
        try:
            self.launch_dir.parent.mkdir(parents=True, exist_ok=True)
            self.launch_dir.mkdir(exist_ok=False)
        except OSError:
            return
        self.launch_dir_created = True
        self._publish_running("prepare")
        self._write_launcher()

    def _final_state(self) -> tuple[str, int]:
        if self.stop_intent.reason is not None:
            return "interrupted", self.stop_intent.exit_code or EXIT_FAILURE
        if (
            self.reason is None
            and self.runtime_exit == 0
            and self.logs.get("runtime_drained")
            and self.cleanup.get("complete")
            and self.audit.get("passed")
        ):
            return "succeeded", EXIT_SUCCESS
        if self.reason == "WORK_DEADLINE_EXCEEDED":
            return "failed", EXIT_TIMEOUT
        return "failed", EXIT_FAILURE

    def _read_manifest_dimensions_if_present(self) -> None:
        """Preserve a matching manifest's three dimensions even on launcher faults."""
        if self.runtime_dimension.get("outcome") not in {None, "unknown"}:
            return
        if not self.launch_dir_created or not self.start_attempted:
            return
        if not self.raw_dir.is_dir():
            return
        episode, _problems = collect_artifacts.select_episode(self.raw_dir, self.arguments.run_id)
        manifest = episode.get("manifest")
        if (
            not isinstance(manifest, dict)
            or manifest.get("run_id") != self.arguments.run_id
            or manifest.get("episode_id") != episode.get("episode_id")
            or manifest.get("schema_version") != 2
        ):
            return
        runtime_dimension = manifest.get("runtime")
        if isinstance(runtime_dimension, dict) and isinstance(
            runtime_dimension.get("outcome"), str
        ):
            self.runtime_dimension = dict(runtime_dimension)
        task_dimension = manifest.get("task")
        if isinstance(task_dimension, dict) and isinstance(task_dimension.get("outcome"), str):
            self.task_dimension = dict(task_dimension)
        recording_dimension = manifest.get("recording")
        if isinstance(recording_dimension, dict):
            self.recording["complete"] = recording_dimension.get("complete") is True
            self.recording["episode_id"] = manifest.get("episode_id")
            self.recording["path"] = episode.get("path")
            self.recording["runtime"] = (
                dict(runtime_dimension) if isinstance(runtime_dimension, dict) else {}
            )

    def _finalize(self, exit_code_hint: int | None = None) -> int:
        self._ensure_failure_evidence()
        if not self.committed and self.run_dir.is_dir():
            self._observe_run_dir()
        # A runtime identified and confirmed stopped may own the final failure dir.
        if not self.committed and not os.path.lexists(self.run_dir):
            self._commit_failure_directory()
        if not self.committed and self.launch_dir_created:
            self._open_logger()
        self.phase = "finalize"
        if self.logger is None and (self.committed or self.launch_dir_created):
            self._open_logger()

        state, code = self._final_state()
        if exit_code_hint is not None:
            if exit_code_hint == EXIT_TIMEOUT:
                state, code = "failed", EXIT_TIMEOUT
            elif exit_code_hint in SIGNAL_EXIT_CODES.values() and self.stop_intent.reason:
                state, code = "interrupted", exit_code_hint
        planned = {
            "state": state,
            "exit_code": code,
            "reason": self.reason or ("OK" if state == "succeeded" else "INTERRUPTED"),
        }
        self._event(
            "run.finishing",
            reason="CLOSE_PENDING",
            level="INFO" if state == "succeeded" else "WARNING",
            fields={
                **planned,
                "log_close_pending": True,
            },
        )
        if self.logger is not None:
            try:
                self.logger.close()
                self.logs["launcher_closed"] = True
            except Exception as error:
                self.logger_close_failed = True
                self.logs["launcher_closed"] = False
                self._fail(
                    "logging", "LAUNCHER_LOG_CLOSE_FAILED", f"{type(error).__name__}: {error}"
                )
        else:
            self.logs["launcher_closed"] = False
            if self.reason is None:
                self._fail("logging", "LAUNCHER_LOG_MISSING", "launcher logger was never created")
        self._read_manifest_dimensions_if_present()
        # Preserve the writer's declaration separately from this launcher's
        # independent verification. An unavailable audit is not proof of integrity.
        declared_complete = (
            self.recording.get("complete") if self.recording.get("episode_id") else None
        )
        self.recording["manifest_complete"] = declared_complete
        self.recording["complete"] = declared_complete is True and self.audit.get("passed") is True
        if self.runtime_dimension.get("outcome") in {None, "unknown"}:
            if self.stop_intent.reason is not None:
                self.runtime_dimension = {
                    "outcome": "interrupted",
                    "reason": self.stop_intent.reason,
                }
            elif self.runtime_exit == 0:
                self.runtime_dimension = {"outcome": "completed", "reason": ""}
            elif self.runtime_exit is not None:
                self.runtime_dimension = {
                    "outcome": "failed",
                    "reason": "RUNTIME_EXIT_NONZERO",
                }
        fallback_image = self.launcher_metadata.get("image", {}).get("id")
        self.launcher_metadata.setdefault(
            "components",
            {
                "runtime": {
                    "container_id": None if self.runtime is None else self.runtime.identifier,
                    "image_id": self.runtime_image_id or fallback_image,
                    "exit_code": self.runtime_exit,
                },
                "backend": {
                    "container_id": None if self.backend is None else self.backend.identifier,
                    "image_id": self.backend_image_id or fallback_image,
                    "exit_code": self.backend_exit,
                },
            },
        )
        if self.source_start is not None:
            try:
                end = collect_artifacts.source_manifest(ROOT)
            except OSError as error:
                self._fail("finalize", "SOURCE_DIGEST_FAILED", str(error))
            else:
                self.launcher_metadata.setdefault("source", {})["end_sha256"] = end["sha256"]
                self.launcher_metadata["source"]["changed_during_run"] = (
                    end["sha256"] != self.source_start["sha256"]
                )
        if self.committed or self.launch_dir_created:
            try:
                collect_artifacts.atomic_write_json(
                    self._evidence_dir() / "launcher.json", self.launcher_metadata
                )
            except OSError as error:
                self._fail("finalize", "LAUNCHER_EVIDENCE_WRITE_FAILED", str(error))

        if self.logger_close_failed or self.errors:
            state, code = self._final_state()
            if state == "succeeded":
                state, code = "failed", EXIT_FAILURE
        reason = self.reason or ("OK" if state == "succeeded" else "INTERRUPTED")
        status = self._status_document(
            state=state,
            reason=reason,
            exit_code=code,
            finished=True,
            phase="finished",
        )
        status_path = self._evidence_dir() / "status.json"
        if self.committed or self.launch_dir_created:
            try:
                collect_artifacts.atomic_write_json(status_path, status)
            except OSError as error:
                self.status_write_error = f"{type(error).__name__}: {error}"
                print(f"collect: cannot write final status: {error}", file=sys.stderr)
                state = "failed"
                reason = "STATUS_WRITE_FAILED"
                code = EXIT_FAILURE
        print(
            json.dumps(
                {
                    "run_id": self.arguments.run_id,
                    "status": state,
                    "reason": reason,
                    "exit_code": code,
                    "status_path": (
                        str(status_path) if self.committed or self.launch_dir_created else None
                    ),
                    "run_directory": str(self.run_dir) if self.committed else None,
                    "launch_evidence": str(self.launch_dir) if not self.committed else None,
                    "audit_path": self.audit.get("path"),
                    "errors": self.errors,
                },
                sort_keys=True,
            ),
            flush=True,
        )
        return code

    # ---- stable evidence helpers -------------------------------------------------

    def _fail(self, stage: str, reason: str, detail: str = "") -> None:
        if self.reason is None:
            self.reason = reason
        entry = {"stage": stage, "reason": reason, "detail": str(detail)[:4000]}
        if not any(
            item.get("reason") == reason and item.get("stage") == stage for item in self.errors
        ):
            self.errors.append(entry)

    def _event(
        self,
        event: str,
        *,
        message: str = "",
        level: str = "INFO",
        reason: str | None = None,
        fields: dict[str, Any] | None = None,
    ) -> None:
        if self.logger is None:
            return
        try:
            self.logger.emit(
                event,
                message=message,
                level=level,
                reason=reason,
                fields=fields or {},
                critical=True,
            )
        except Exception as error:
            self._fail("logging", "LAUNCHER_LOG_FAILED", f"{type(error).__name__}: {error}")

    def _evidence_dir(self) -> Path:
        return self.run_dir if self.committed else self.launch_dir

    def _diagnostic_path(self, name: str) -> Path:
        return self._evidence_dir() / "diagnostics" / name

    # ---- status/launcher metadata -------------------------------------------------

    # ---- preflight ---------------------------------------------------------------

    def _preflight_paths(self) -> None:
        for path in (self.run_dir, self.raw_dir, self.launch_dir):
            if os.path.lexists(path):
                raise LauncherError(
                    "PRECONDITION_FAILED",
                    f"run/product path already exists: {path}; choose a fresh --run-id",
                )

    def _check_project(self) -> None:
        try:
            identifiers = mock_stack.project_ids(
                self.docker,
                self.arguments.project,
                self.clock() + self.config.command_timeout_s,
            )
        except mock_stack.HostError as error:
            raise LauncherError(error.reason, error.detail) from error
        if identifiers:
            raise LauncherError(
                "PROJECT_EXISTS",
                f"Compose project {self.arguments.project!r} already has containers; "
                "stop or remove it before starting another run",
            )

    def _create_launch_evidence(self) -> None:
        try:
            self.launch_dir.parent.mkdir(parents=True, exist_ok=True)
            self.launch_dir.mkdir(exist_ok=False)
        except FileExistsError as error:
            raise LauncherError(
                "PRECONDITION_FAILED", f"launch evidence exists: {self.launch_dir}"
            ) from error
        except OSError as error:
            raise LauncherError("LAUNCH_DIRECTORY_FAILED", str(error)) from error
        self.launch_dir_created = True
        self._publish_running("prepare")
        self._write_launcher()

    def _prepare_compose(self) -> None:
        try:
            argv, environment, _command = dev_compose.prepare(
                SimpleNamespace(
                    project=self.arguments.project,
                    image=self.arguments.image,
                    build_root=self.arguments.build_root,
                    raw_root=self.arguments.raw_root,
                    runs_root=self.arguments.runs_root,
                    docker=self.arguments.docker,
                    run_id=self.arguments.run_id,
                    compose_arguments=["up", "-d", "--no-build", "runtime"],
                )
            )
        except dev_compose.DevComposeError as error:
            raise LauncherError("PRECONDITION_FAILED", str(error)) from error
        except OSError as error:
            raise LauncherError(
                "PRECONDITION_FAILED", f"cannot prepare mount paths: {error}"
            ) from error
        del argv
        self.environment = environment
        self.expected = mock_stack.ExpectedStack.from_environment(environment)

    def _inspect_image(self) -> None:
        deadline = min(self.deadline, self.clock() + self.config.command_timeout_s)
        try:
            completed = self.docker.run(
                ["image", "inspect", "--format", "{{.Id}}", self.arguments.image], deadline
            )
        except mock_stack.HostError as error:
            self._fail("prepare", error.reason, error.detail)
            image_id = None
        else:
            image_id = (
                (completed.stdout or "").strip()
                if completed.returncode == 0 and (completed.stdout or "").strip()
                else None
            )
        self.launcher_metadata = {"image": {"reference": self.arguments.image, "id": image_id}}
        self._write_launcher()

    # ---- start/supervise ---------------------------------------------------------

    def _compose_argv(self, arguments: Sequence[str], *, audit: bool = False) -> list[str]:
        return mock_stack.compose_argv_with_override(
            docker=self.arguments.docker,
            project=self.arguments.project,
            arguments=arguments,
            audit=audit,
        )

    def _compose(
        self, name: str, arguments: Sequence[str], deadline: float, *, audit: bool = False
    ):
        return self._capture(
            name,
            arguments,
            deadline,
            environment=self.environment,
            argv=self._compose_argv(arguments, audit=audit),
        )

    def _start(self) -> None:
        if self.stop_intent.reason is not None:
            raise LauncherError(self.stop_intent.reason, "termination signal received before start")
        if self.clock() >= self.deadline:
            raise LauncherError("WORK_DEADLINE_EXCEEDED", "work deadline expired before start")
        self.start_attempted = True
        if self.arguments.probe_backend:
            self._probe_backend()
        self.phase = "start"
        result, error = self._compose(
            "compose-up.log",
            ["up", "-d", "--no-build", "runtime"],
            min(self.deadline, self.clock() + self.config.command_timeout_s),
        )
        if error is not None:
            raise LauncherError(error.reason, error.detail) from error
        assert result is not None
        completed, text, info = result
        self.compose_up_diagnostic = info
        if completed.returncode != 0:
            raise LauncherError(
                "COMPOSE_UP_FAILED", _tail(completed.stderr or text) or "docker compose up failed"
            )
        self._observe_run_dir()
        self._wait_stack()

    # ---- run directory ownership and runtime evidence ----------------------------

    def _observe_run_dir(self) -> bool:
        if self.committed:
            return True
        # A rejected preflight must never attach to or rewrite an older run.
        # The launch claim and identified runtime establish this attempt's ownership.
        if not self.launch_dir_created or not self.start_attempted or self.runtime is None:
            return False
        if not self.run_dir.is_dir():
            return False
        moved = collect_artifacts.move_launch_evidence(self.launch_dir, self.run_dir)
        if moved["skipped"]:
            self._fail(
                "evidence",
                "EVIDENCE_CONFLICT",
                "runtime-owned paths already exist: " + ", ".join(moved["skipped"]),
            )
        self.committed = True
        self._publish_running("supervise")
        self._open_logger()
        self._event(
            "run.started",
            reason="STARTUP",
            fields={
                "run_id": self.arguments.run_id,
                "project": self.arguments.project,
                "config_sha256": self.config.sha256,
                "work_deadline_s": self.config.work_deadline_s,
                "runtime_id": None if self.runtime is None else self.runtime.identifier[:12],
                "backend_id": None if self.backend is None else self.backend.identifier[:12],
            },
        )
        return True

    def _commit_failure_directory(self) -> bool:
        """Create the final run directory only after this runtime is confirmed stopped."""
        if self.committed or os.path.lexists(self.run_dir):
            return False
        if self.runtime is None:
            return False
        if not self.runtime_confirmed_stopped:
            try:
                refreshed = mock_stack.inspect_containers(
                    self.docker,
                    [self.runtime.identifier],
                    self._cleanup_budget()
                    if self.cleanup_attempted
                    else min(self.deadline, self.clock() + self.config.command_timeout_s),
                    allow_missing=True,
                )
            except mock_stack.HostError:
                return False
            if len(refreshed) != 1 or refreshed[0].running:
                return False
            self.runtime = refreshed[0]
            self.runtime_confirmed_stopped = True
        try:
            self.run_dir.mkdir(parents=True, exist_ok=False)
        except FileExistsError:
            self._fail("evidence", "RUN_DIRECTORY_CONFLICT", "another run created the output path")
            return False
        except OSError as error:
            self._fail("evidence", "RUN_DIRECTORY_FAILED", str(error))
            return False
        moved = collect_artifacts.move_launch_evidence(self.launch_dir, self.run_dir)
        if moved["skipped"]:
            self._fail("evidence", "EVIDENCE_CONFLICT", ", ".join(moved["skipped"]))
        self.committed = True
        self._publish_running("finalize")
        self._open_logger()
        return True

    def _open_logger(self) -> None:
        if self.logger is not None or self.logger_close_failed:
            return
        directory = self._evidence_dir() / "logs" / "launcher" / self.launcher_instance
        try:
            self.logger = self.logger_factory(
                directory,
                run_id=self.arguments.run_id,
                component="launcher",
                settings=_log_settings(self.config),
                instance_id=self.launcher_instance,
            )
            self.logger_ready = True
        except Exception as error:
            self.logger = None
            self._fail("logging", "LAUNCHER_LOG_INIT_FAILED", f"{type(error).__name__}: {error}")

    def _verify_runtime_evidence(self) -> None:
        if not self._observe_run_dir():
            raise LauncherError(
                "RUNTIME_DIRECTORY_MISSING",
                f"runtime did not create {self.run_dir}; launch evidence kept at {self.launch_dir}",
            )
        evidence, problems = collect_artifacts.runtime_log_evidence(
            self.run_dir, self.arguments.run_id
        )
        self.logs["runtime"] = evidence
        self.logs["runtime_drained"] = not problems
        runtime_stdout = collect_artifacts.parse_runtime_stdout(self.runtime_stdout_text)
        if runtime_stdout is not None:
            evidence["stdout_summary"] = runtime_stdout
            if runtime_stdout.get("exit_code") != self.runtime_exit:
                problems.append("RUNTIME_STDOUT_EXIT_MISMATCH")
            if runtime_stdout.get("logging_closed") is not True:
                problems.append("RUNTIME_STDOUT_LOG_NOT_CLOSED")
        if problems:
            raise LauncherError("RUNTIME_LOG_INVALID", "; ".join(problems))
        self._event(
            "runtime.logs_drained",
            reason="OK",
            fields={
                "instance_id": evidence.get("instance_id"),
                "events_written": evidence.get("summary", {}).get("written"),
            },
        )

        episode, problems = collect_artifacts.select_episode(self.raw_dir, self.arguments.run_id)
        self.recording.update(
            {
                "complete": False,
                "episode_id": episode.get("episode_id"),
                "path": episode.get("path"),
            }
        )
        if problems:
            raise LauncherError("RECORDING_INVALID", "; ".join(problems))
        manifest = episode.get("manifest", {})
        recording_dimension = manifest.get("recording")
        self.recording.update(
            {
                "complete": bool(
                    isinstance(recording_dimension, dict)
                    and recording_dimension.get("complete") is True
                ),
                "runtime": dict(manifest.get("runtime", {})),
            }
        )
        runtime_dimension = manifest.get("runtime")
        if isinstance(runtime_dimension, dict) and isinstance(
            runtime_dimension.get("outcome"), str
        ):
            self.runtime_dimension = dict(runtime_dimension)
        task_dimension = manifest.get("task")
        if isinstance(task_dimension, dict) and isinstance(task_dimension.get("outcome"), str):
            self.task_dimension = dict(task_dimension)
        self._episode = episode

    def _run_audit(self) -> None:
        self.phase = "audit"
        if not self._observe_run_dir():
            raise LauncherError("RUNTIME_DIRECTORY_MISSING", "no run directory for audit")
        episode = getattr(self, "_episode", None)
        if episode is None:
            raise LauncherError("AUDIT_NO_EPISODE", "no committed episode to audit")
        if self.clock() >= self.deadline:
            raise LauncherError(
                "WORK_DEADLINE_EXCEEDED", "work deadline expired before independent audit"
            )
        deadline = min(self.deadline, self.clock() + self.config.audit_timeout_s)
        name = f"collect-audit-{uuid.uuid4().hex[:10]}"
        self._event(
            "audit.started", reason="STARTING", fields={"episode_id": episode["episode_id"]}
        )
        result, error = self._compose(
            "audit.log",
            [
                "run",
                "--rm",
                "--no-deps",
                "-T",
                "--name",
                name,
                "runtime",
                "python",
                "-m",
                "dual_arm_runtime.audit",
                f"/data/raw/{self.arguments.run_id}/{episode['episode_id']}",
            ],
            deadline,
            audit=True,
        )
        if error is not None:
            self._remove_named(name)
            if error.reason == "COMMAND_TIMEOUT" and self.clock() >= self.deadline:
                raise LauncherError(
                    "WORK_DEADLINE_EXCEEDED", "audit crossed the work deadline"
                ) from error
            if error.reason == "COMMAND_TIMEOUT":
                raise LauncherError("AUDIT_TIMEOUT", "independent audit tool timed out") from error
            raise LauncherError(error.reason, error.detail) from error
        assert result is not None
        completed, text, info = result
        if getattr(completed, "truncated", False):
            raise LauncherError(
                "AUDIT_OUTPUT_TRUNCATED",
                "independent audit output exceeded the bounded capture limit",
            )
        if completed.returncode != 0:
            raise LauncherError(
                "AUDIT_FAILED", _tail(completed.stderr or text) or "audit exited non-zero"
            )
        try:
            summary = json.loads(completed.stdout or "")
        except ValueError as error:
            raise LauncherError("AUDIT_INVALID_JSON", str(error)) from error
        if not isinstance(summary, dict):
            raise LauncherError("AUDIT_INVALID_JSON", "audit stdout is not one JSON object")
        audit_path = self.run_dir / "artifacts" / "audit.json"
        try:
            collect_artifacts.atomic_write_json(audit_path, summary)
        except (OSError, ValueError) as error:
            raise LauncherError("AUDIT_EVIDENCE_WRITE_FAILED", str(error)) from error
        self.audit.update(
            {
                "path": str(audit_path),
                "summary_path": None if info is None else info.get("path"),
                "summary": summary,
            }
        )
        problems = collect_artifacts.audit_problems(
            summary,
            episode_id=str(episode["episode_id"]),
            min_motion_rad=self.config.min_mock_motion_rad,
        )
        self.audit["problems"] = problems
        if problems:
            self.audit["passed"] = False
            raise LauncherError("AUDIT_REJECTED", "; ".join(problems))
        self.audit["passed"] = True
        self.recording.update(
            {
                "complete": summary["recording"]["complete"],
                "episode_id": summary["episode_id"],
            }
        )
        if self.runtime_dimension.get("outcome") in {None, "unknown"}:
            self.runtime_dimension = dict(summary["runtime"])
        self._event(
            "audit.completed",
            reason="OK",
            fields={
                "episode_id": summary["episode_id"],
                "observations": summary["observations"],
                "frames": summary["frames"],
                "commands": summary["commands"],
                "terminal_events": summary["terminal_events"],
                "max_joint_motion": float(summary["max_joint_motion"]),
            },
        )

    # ---- bounded stop/cleanup ----------------------------------------------------

    # ---- finalization ------------------------------------------------------------

    # ---- top-level orchestration -------------------------------------------------

    def run(self) -> int:
        exit_hint: int | None = None
        try:
            self._preflight_paths()
            self._check_project()
            self._create_launch_evidence()
            self._prepare_compose()
            self._inspect_image()
            if self.stop_intent.reason is not None:
                raise LauncherError(
                    self.stop_intent.reason, "termination signal received before start"
                )
            self._start()
            if not self.runtime_finished:
                self._supervise()
            if self.reason is None and self.runtime_exit == 0:
                self._verify_runtime_evidence()
                self._run_audit()
            if self.reason is None:
                self._stop_backend()
            self._cleanup()
        except LauncherError as error:
            self._fail(self.phase, error.reason, error.detail)
            if error.reason == "WORK_DEADLINE_EXCEEDED":
                exit_hint = EXIT_TIMEOUT
            if error.reason in SIGNAL_EXIT_CODES:
                exit_hint = SIGNAL_EXIT_CODES[error.reason]
        except Exception as error:
            self._fail(self.phase, "INTERNAL_ERROR", f"{type(error).__name__}: {error}")
        finally:
            if self.stop_intent.reason is not None:
                exit_hint = self.stop_intent.exit_code
            if not self.cleanup_attempted and self.runtime is not None and self.runtime.running:
                self._stop_runtime()
            elif (
                not self.cleanup_attempted
                and self.runtime is not None
                and self.runtime_exit is None
            ):
                self.runtime_exit = self._exit_code(self.runtime)
            if (
                not self.cleanup_attempted
                and self.backend is not None
                and self.backend.running
                and self.reason is not None
            ):
                self._stop_backend()
            elif (
                not self.cleanup_attempted
                and self.backend is not None
                and self.backend_exit is None
            ):
                self.backend_exit = self._exit_code(self.backend)
            if (
                not self.cleanup_attempted
                and self.runtime_exit == 0
                and self.reason is None
                and self.backend_exit is None
            ):
                self._stop_backend()
            if self.start_attempted:
                self._cleanup()
            else:
                self.cleanup = {
                    "complete": True,
                    "removed": [],
                    "remaining": [],
                    "reason": "NOT_STARTED",
                }
        return self._finalize(exit_hint)


def main(
    argv: Sequence[str] | None = None,
    *,
    runner: Callable[..., object] | None = None,
    clock: Callable[[], float] = time.monotonic,
    sleeper: Callable[[float], None] = time.sleep,
    logger_factory: Callable[..., EventLogger] = EventLogger,
    stop_intent: StopIntent | None = None,
) -> int:
    try:
        arguments = _arguments(argv)
    except UsageError as error:
        print(f"collect: {error}", file=sys.stderr)
        print(
            json.dumps(
                {
                    "status": "failed",
                    "reason": "INVALID_ARGUMENT",
                    "detail": str(error),
                    "exit_code": EXIT_CONFIG,
                },
                sort_keys=True,
            ),
            flush=True,
        )
        return EXIT_CONFIG
    try:
        config = collect_config.load_config(arguments.config)
    except collect_config.ConfigError as error:
        print(f"collect: configuration error: {error}", file=sys.stderr)
        print(
            json.dumps(
                {
                    "status": "failed",
                    "reason": "CONFIG_INVALID",
                    "detail": str(error),
                    "config_path": str(arguments.config),
                    "exit_code": EXIT_CONFIG,
                },
                sort_keys=True,
            ),
            flush=True,
        )
        return EXIT_CONFIG

    intent = stop_intent or StopIntent()
    previous: dict[int, Any] = {}
    if stop_intent is None:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous[signum] = signal.signal(signum, intent.handler)
    try:
        collector = Collector(
            arguments,
            config,
            docker=mock_stack.BoundedDocker(
                arguments.docker,
                runner=runner,
                clock=clock,
                output_limit=config.diagnostic_max_bytes,
            ),
            stop_intent=intent,
            clock=clock,
            sleeper=sleeper,
            logger_factory=logger_factory,
        )
        try:
            return collector.run()
        except KeyboardInterrupt:
            intent.request(signal.SIGINT)
            return intent.exit_code or EXIT_FAILURE
        except Exception as error:
            print(
                f"collect: unexpected launcher failure: {type(error).__name__}: {error}",
                file=sys.stderr,
            )
            print(
                json.dumps(
                    {
                        "run_id": arguments.run_id,
                        "status": "failed",
                        "reason": "INTERNAL_ERROR",
                        "detail": f"{type(error).__name__}: {error}",
                        "exit_code": EXIT_FAILURE,
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
            return EXIT_FAILURE
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)


if __name__ == "__main__":
    raise SystemExit(main())
