"""Human host entry point for the Compose mock development stack.

This module owns deployment and bounded client invocation only.  Path
preparation, fresh-run checks and Compose argument forwarding are reused from
``tools/run/dev_compose.py``; episode/control/audit logic stays in the runtime.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from types import SimpleNamespace
from typing import Any

_TOOLS_RUN = Path(__file__).resolve().parent
if str(_TOOLS_RUN) not in sys.path:
    sys.path.insert(0, str(_TOOLS_RUN))

import dev_compose  # noqa: E402

ROOT = dev_compose.ROOT
COMPOSE_FILE = dev_compose.COMPOSE_FILE
CONTAINER_WORKSPACE = "/workspace"
CONTAINER_ENTRYPOINT = "/workspace/deploy/docker/entrypoints/dev_ros.sh"
SERVICES = frozenset({"runtime", "backend"})
PROJECT_LABEL = "com.docker.compose.project"
SERVICE_LABEL = "com.docker.compose.service"
WORKING_DIR_LABEL = "com.docker.compose.project.working_dir"
STDERR_LIMIT = 2000


class HostError(Exception):
    """A bounded, reportable host failure with stable diagnostic fields."""

    def __init__(self, reason: str, detail: str = "", **fields: Any) -> None:
        super().__init__(detail or reason)
        self.reason = reason
        self.detail = detail
        self.fields = fields


class UsageError(Exception):
    """Parsing or validation failed before Docker was invoked."""


@dataclass(frozen=True)
class Container:
    identifier: str
    name: str
    project: str
    service: str
    image: str
    state: str
    exit_code: int
    command: tuple[str, ...]
    mounts: dict[str, str]
    labels: dict[str, str]
    image_id: str = ""

    @property
    def running(self) -> bool:
        return self.state == "running"

    def summary(self) -> dict[str, Any]:
        return {
            "id": self.identifier[:12],
            "name": self.name,
            "service": self.service,
            "state": self.state,
            "exit_code": self.exit_code,
        }


@dataclass(frozen=True)
class ExpectedStack:
    project: str
    image: str
    run_id: str
    repo_root: Path
    build_root: Path
    raw_root: Path
    runs_root: Path

    @classmethod
    def from_environment(cls, environment: dict[str, str]) -> ExpectedStack:
        return cls(
            project=environment["DEV_COMPOSE_PROJECT"],
            image=environment["DEV_IMAGE"],
            run_id=environment["DEV_RUN_ID"],
            repo_root=Path(environment["DEV_REPO_ROOT"]).resolve(),
            build_root=Path(environment["DEV_BUILD_ROOT"]).resolve(),
            raw_root=Path(environment["DEV_RAW_ROOT"]).resolve(),
            runs_root=Path(environment["DEV_RUNS_ROOT"]).resolve(),
        )


@dataclass(frozen=True)
class Prepared:
    argv: list[str]
    environment: dict[str, str] = field(repr=False)


@dataclass
class Operation:
    arguments: argparse.Namespace
    docker: Docker
    deadline: float


class Docker:
    """Bounded argument-list Docker invocation with one injectable runner."""

    def __init__(
        self,
        executable: str,
        runner: Callable[..., subprocess.CompletedProcess] | None = None,
    ) -> None:
        self.executable = executable
        self._runner = subprocess.run if runner is None else runner

    def run(
        self,
        arguments: Sequence[str],
        deadline: float,
        *,
        environment: dict[str, str] | None = None,
    ) -> subprocess.CompletedProcess:
        return self.run_argv([self.executable, *arguments], deadline, environment=environment)

    def run_argv(
        self,
        argv: Sequence[str],
        deadline: float,
        *,
        environment: dict[str, str] | None = None,
    ) -> subprocess.CompletedProcess:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise HostError("COMMAND_TIMEOUT", "the shared command deadline expired")
        try:
            return self._runner(
                list(argv),
                env=environment,
                check=False,
                capture_output=True,
                text=True,
                timeout=remaining,
            )
        except subprocess.TimeoutExpired as error:
            raise HostError(
                "COMMAND_TIMEOUT",
                f"command exceeded the shared deadline: {argv[0]}",
                may_have_executed=True,
            ) from error
        except FileNotFoundError as error:
            raise HostError("DOCKER_UNAVAILABLE", f"cannot execute {argv[0]!r}") from error
        except OSError as error:
            raise HostError("DOCKER_ERROR", f"{type(error).__name__}: {error}") from error


def positive_timeout(value: object) -> float:
    if isinstance(value, bool):
        raise ValueError("timeout must be a finite positive number of seconds")
    try:
        seconds = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError("timeout must be a finite positive number of seconds") from error
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError("timeout must be a finite positive number of seconds")
    return seconds


def _timeout_argument(value: str) -> float:
    try:
        return positive_timeout(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error


class _Parser(argparse.ArgumentParser):
    def error(self, message: str) -> None:
        raise UsageError(message)


def _dev_arguments(arguments: argparse.Namespace, compose_arguments: list[str]) -> SimpleNamespace:
    return SimpleNamespace(
        project=arguments.project,
        image=arguments.image,
        build_root=arguments.build_root,
        raw_root=arguments.raw_root,
        runs_root=arguments.runs_root,
        docker=arguments.docker,
        run_id=arguments.run_id,
        compose_arguments=compose_arguments,
    )


def _prepare(arguments: argparse.Namespace, compose_arguments: list[str]) -> Prepared:
    try:
        argv, environment, _command = dev_compose.prepare(
            _dev_arguments(arguments, compose_arguments)
        )
    except dev_compose.DevComposeError as error:
        raise HostError("PRECONDITION_FAILED", str(error)) from error
    except OSError as error:
        raise HostError("PRECONDITION_FAILED", f"cannot prepare mount paths: {error}") from error
    return Prepared(list(argv), environment)


def _environment(arguments: argparse.Namespace) -> tuple[dict[str, str], ExpectedStack]:
    prepared = _prepare(arguments, ["ps"])
    return prepared.environment, ExpectedStack.from_environment(prepared.environment)


def _tail(value: object, limit: int = STDERR_LIMIT) -> str:
    text = "" if value is None else str(value).strip()
    if len(text) <= limit:
        return text
    return text[-limit:]


def _project_ids(docker: Docker, project: str, deadline: float) -> list[str]:
    completed = docker.run(
        [
            "ps",
            "-a",
            "--no-trunc",
            "--filter",
            f"label={PROJECT_LABEL}={project}",
            "--format",
            "{{.ID}}",
        ],
        deadline,
    )
    if completed.returncode != 0:
        raise HostError(
            "DOCKER_ERROR",
            _tail(completed.stderr) or "docker ps failed",
            exit_code=int(completed.returncode),
        )
    return [line.strip() for line in (completed.stdout or "").splitlines() if line.strip()]


def _container(document: dict[str, Any]) -> Container:
    config = document.get("Config") or {}
    state = document.get("State") or {}
    labels = config.get("Labels") or {}
    mounts = {
        str(mount.get("Destination")): str(mount.get("Source"))
        for mount in document.get("Mounts") or []
        if mount.get("Destination") and mount.get("Source")
    }
    command = config.get("Cmd") or []
    return Container(
        identifier=str(document.get("Id") or ""),
        name=str(document.get("Name") or "").lstrip("/"),
        project=str(labels.get(PROJECT_LABEL) or ""),
        service=str(labels.get(SERVICE_LABEL) or ""),
        image=str(config.get("Image") or ""),
        state=str(state.get("Status") or ""),
        exit_code=int(state.get("ExitCode") or 0),
        command=tuple(str(item) for item in command),
        mounts=mounts,
        labels={str(key): str(value) for key, value in labels.items()},
        image_id=str(document.get("Image") or ""),
    )


def _inspect(
    docker: Docker,
    identifiers: Sequence[str],
    deadline: float,
    *,
    allow_missing: bool = False,
) -> list[Container]:
    if not identifiers:
        return []
    completed = docker.run(["inspect", *identifiers], deadline)
    try:
        document = json.loads(completed.stdout or "[]")
    except (TypeError, ValueError) as error:
        raise HostError(
            "DOCKER_OUTPUT",
            f"docker inspect did not return JSON: {error}",
            exit_code=int(completed.returncode),
            stderr=_tail(completed.stderr),
        ) from error
    if not isinstance(document, list) or not all(isinstance(item, dict) for item in document):
        raise HostError("DOCKER_OUTPUT", "docker inspect returned an unexpected document")
    if completed.returncode != 0 and not allow_missing:
        raise HostError(
            "DOCKER_ERROR",
            _tail(completed.stderr) or "docker inspect failed",
            exit_code=int(completed.returncode),
        )
    return [_container(item) for item in document]


def _split_services(
    containers: Sequence[Container], expected: ExpectedStack
) -> dict[str, list[Container]]:
    grouped: dict[str, list[Container]] = {}
    for container in containers:
        if container.project != expected.project:
            raise HostError(
                "PROJECT_MISMATCH",
                "container "
                f"{container.name or container.identifier} belongs to {container.project!r}",
            )
        if container.service not in SERVICES:
            raise HostError(
                "UNEXPECTED_SERVICE",
                f"unexpected Compose service {container.service!r}",
                name=container.name,
            )
        grouped.setdefault(container.service, []).append(container)
    return grouped


def _option_value(command: Sequence[str], option: str) -> str | None:
    for index, token in enumerate(command):
        if token == option and index + 1 < len(command):
            return command[index + 1]
        prefix = f"{option}="
        if token.startswith(prefix):
            return token[len(prefix) :]
    return None


def _validate_container(container: Container, expected: ExpectedStack, service: str) -> None:
    if container.project != expected.project:
        raise HostError("PROJECT_MISMATCH", "Compose project label differs")
    if container.service != service:
        raise HostError("SERVICE_MISMATCH", f"expected service {service!r}")
    if container.image != expected.image:
        raise HostError(
            "IMAGE_MISMATCH",
            f"container uses {container.image!r}, expected {expected.image!r}",
        )
    working_directory = container.labels.get(WORKING_DIR_LABEL)
    if working_directory and Path(working_directory).resolve() != ROOT:
        raise HostError(
            "PROJECT_MISMATCH",
            f"Compose working directory is {working_directory!r}, expected {str(ROOT)!r}",
        )
    mounts = {
        CONTAINER_WORKSPACE: expected.repo_root,
        "/opt/ros-dev": expected.build_root,
    }
    if service == "runtime":
        mounts["/data/raw"] = expected.raw_root
        mounts["/data/runs"] = expected.runs_root
    for target, source in mounts.items():
        actual = container.mounts.get(target)
        if actual is None:
            raise HostError("MOUNT_MISMATCH", f"missing mount {target!r}")
        if Path(actual).resolve() != source:
            raise HostError(
                "MOUNT_MISMATCH",
                f"mount {target!r} is {actual!r}, expected {str(source)!r}",
            )

    if service == "runtime":
        if "dual_arm_runtime.node" not in container.command:
            raise HostError("RUNTIME_COMMAND_MISMATCH", "runtime container does not run the node")
        run_id = _option_value(container.command, "--run-id")
        if run_id != expected.run_id:
            raise HostError(
                "RUN_MISMATCH",
                f"runtime command run id is {run_id!r}, expected {expected.run_id!r}",
            )
        output = _option_value(container.command, "--output")
        if output != f"/data/raw/{expected.run_id}":
            raise HostError(
                "OUTPUT_MISMATCH",
                f"runtime output is {output!r}, expected /data/raw/{expected.run_id}",
            )
        runs_output = _option_value(container.command, "--runs-output")
        if runs_output != "/data/runs":
            raise HostError("OUTPUT_MISMATCH", f"runtime runs output is {runs_output!r}")


def _resolve_stack(
    docker: Docker,
    expected: ExpectedStack,
    deadline: float,
) -> tuple[list[Container], Container, Container]:
    identifiers = _project_ids(docker, expected.project, deadline)
    containers = _inspect(docker, identifiers, deadline)
    if not containers:
        raise HostError(
            "STACK_NOT_FOUND", f"no containers for Compose project {expected.project!r}"
        )
    grouped = _split_services(containers, expected)
    runtime = grouped.get("runtime", [])
    backend = grouped.get("backend", [])
    if not runtime:
        raise HostError("RUNTIME_NOT_FOUND", "the stack has no runtime container")
    if len(runtime) > 1:
        raise HostError("RUNTIME_MULTIPLE", "the stack has more than one runtime container")
    if not backend:
        raise HostError("BACKEND_NOT_FOUND", "the stack has no backend container")
    if len(backend) > 1:
        raise HostError("BACKEND_MULTIPLE", "the stack has more than one backend container")
    _validate_container(runtime[0], expected, "runtime")
    _validate_container(backend[0], expected, "backend")
    return containers, runtime[0], backend[0]


def _wait_running_stack(
    docker: Docker,
    expected: ExpectedStack,
    deadline: float,
) -> tuple[list[Container], Container, Container]:
    while True:
        try:
            containers, runtime, backend = _resolve_stack(docker, expected, deadline)
        except HostError as error:
            if error.reason not in {
                "STACK_NOT_FOUND",
                "RUNTIME_NOT_FOUND",
                "BACKEND_NOT_FOUND",
            }:
                raise
            if time.monotonic() >= deadline:
                raise
            time.sleep(0.1)
            continue
        if runtime.running and backend.running:
            return containers, runtime, backend
        if time.monotonic() >= deadline:
            if not runtime.running:
                raise HostError(
                    "RUNTIME_NOT_RUNNING",
                    f"runtime is {runtime.state or 'missing'}",
                    container=runtime.summary(),
                )
            raise HostError(
                "BACKEND_NOT_RUNNING",
                f"backend is {backend.state or 'missing'}",
                container=backend.summary(),
            )
        time.sleep(0.1)


def _refresh(
    docker: Docker,
    expected: ExpectedStack,
    identifiers: Sequence[tuple[str, str]],
    deadline: float,
) -> dict[str, Container | None]:
    """Re-inspect known container ids and refuse to call a cached status healthy."""
    by_id: dict[str, str] = {identifier: service for identifier, service in identifiers}
    found = _inspect(docker, list(by_id), deadline, allow_missing=True)
    result: dict[str, Container | None] = {identifier: None for identifier in by_id}
    for container in found:
        if container.identifier in result:
            _validate_container(container, expected, by_id[container.identifier])
            result[container.identifier] = container
    return result


def _compose_exec_argv(
    arguments: argparse.Namespace,
    cli_arguments: Sequence[str],
) -> list[str]:
    return dev_compose.compose_argv(
        docker=arguments.docker,
        project=arguments.project,
        compose_file=COMPOSE_FILE,
        project_directory=ROOT,
        arguments=[
            "exec",
            "-T",
            "runtime",
            "bash",
            CONTAINER_ENTRYPOINT,
            "python3",
            "-m",
            "dual_arm_runtime.cli",
            *cli_arguments,
        ],
    )


def _client_arguments(
    operation: str,
    expected: ExpectedStack,
    deadline: float,
    extra: Sequence[str],
) -> list[str]:
    remaining = max(0.05, deadline - time.monotonic())
    reserve = min(1.5, max(0.5, remaining * 0.1))
    client_timeout = max(0.05, remaining - reserve)
    return [
        "--run-id",
        expected.run_id,
        "--timeout",
        f"{client_timeout:.6f}",
        operation,
        *extra,
    ]


def _run_client(
    operation: Operation,
    prepared_environment: dict[str, str],
    expected: ExpectedStack,
    cli_operation: str,
    extra: Sequence[str] = (),
) -> dict[str, Any]:
    cli_arguments = _client_arguments(cli_operation, expected, operation.deadline, extra)
    argv = _compose_exec_argv(operation.arguments, cli_arguments)
    try:
        completed = operation.docker.run_argv(
            argv,
            operation.deadline,
            environment=prepared_environment,
        )
    except HostError as error:
        if error.reason == "COMMAND_TIMEOUT":
            error.reason = "CLIENT_TIMEOUT"
            error.fields["may_have_executed"] = True
        raise
    stdout = (completed.stdout or "").strip()
    try:
        result = json.loads(stdout)
    except (TypeError, ValueError) as error:
        raise HostError(
            "CLIENT_INVALID_JSON",
            f"client stdout is not one JSON object: {error}",
            exit_code=int(completed.returncode),
            stderr=_tail(completed.stderr),
        ) from error
    if not isinstance(result, dict) or not all(
        key in result for key in ("operation", "run_id", "success", "reason")
    ):
        raise HostError(
            "CLIENT_INVALID_RESPONSE",
            "client response is missing required fields",
            exit_code=int(completed.returncode),
        )
    if result["operation"] != cli_operation or result["run_id"] != expected.run_id:
        raise HostError(
            "CLIENT_RESPONSE_MISMATCH",
            "client response operation/run id differs from the request",
            exit_code=int(completed.returncode),
        )
    if not isinstance(result["success"], bool) or not isinstance(result["reason"], str):
        raise HostError(
            "CLIENT_INVALID_RESPONSE",
            "client success/reason have unexpected types",
            exit_code=int(completed.returncode),
        )
    if result["success"] and completed.returncode != 0:
        raise HostError(
            "CLIENT_EXIT_MISMATCH",
            "client claimed success but exited non-zero",
            exit_code=int(completed.returncode),
            client=result,
        )
    if not result["success"] and completed.returncode == 0:
        raise HostError(
            "CLIENT_EXIT_MISMATCH",
            "client claimed failure but exited zero",
            exit_code=int(completed.returncode),
            client=result,
        )
    return result


def _base(arguments: argparse.Namespace, operation: str) -> dict[str, Any]:
    return {
        "operation": operation,
        "run_id": arguments.run_id,
        "success": True,
        "reason": "OK",
        "audit_verified": False,
    }


def _cleanup_started(
    operation: Operation,
    expected: ExpectedStack,
) -> dict[str, Any]:
    """Remove only containers that still match this project/run/service identity."""
    timeout = operation.arguments.timeout
    cleanup_deadline = time.monotonic() + max(0.5, min(5.0, timeout))
    try:
        identifiers = _project_ids(operation.docker, expected.project, cleanup_deadline)
        containers = _inspect(operation.docker, identifiers, cleanup_deadline, allow_missing=True)
    except HostError as error:
        return {"complete": False, "reason": error.reason, "removed": []}
    owned: list[str] = []
    for container in containers:
        try:
            _validate_container(container, expected, container.service)
        except HostError:
            continue
        if container.service in SERVICES:
            owned.append(container.identifier)
    if not owned:
        return {"complete": True, "reason": "", "removed": []}
    try:
        completed = operation.docker.run(
            ["rm", "-f", *owned],
            cleanup_deadline,
        )
    except HostError as error:
        return {"complete": False, "reason": error.reason, "removed": []}
    return {
        "complete": completed.returncode == 0,
        "reason": "" if completed.returncode == 0 else _tail(completed.stderr),
        "removed": owned,
        "exit_code": int(completed.returncode),
    }


def _up(operation: Operation) -> dict[str, Any]:
    arguments = operation.arguments
    if _project_ids(operation.docker, arguments.project, operation.deadline):
        raise HostError(
            "PROJECT_EXISTS",
            f"Compose project {arguments.project!r} already has containers; "
            "stop or remove it before starting another run",
        )
    prepared = _prepare(arguments, ["up", "-d"])
    expected = ExpectedStack.from_environment(prepared.environment)
    try:
        completed = operation.docker.run_argv(
            prepared.argv,
            operation.deadline,
            environment=prepared.environment,
        )
    except HostError as error:
        cleanup = _cleanup_started(operation, expected)
        error.fields.setdefault("cleanup", cleanup)
        if error.reason == "COMMAND_TIMEOUT":
            error.reason = "COMPOSE_UP_TIMEOUT"
        raise
    if completed.returncode != 0:
        cleanup = _cleanup_started(operation, expected)
        raise HostError(
            "COMPOSE_UP_FAILED",
            _tail(completed.stderr) or "docker compose up failed",
            exit_code=int(completed.returncode),
            cleanup=cleanup,
        )

    try:
        _containers, runtime, backend = _wait_running_stack(
            operation.docker,
            expected,
            operation.deadline,
        )
        client = _run_client(operation, prepared.environment, expected, "status")
        status = client.get("status")
        if (
            not client.get("success")
            or not isinstance(status, dict)
            or status.get("phase") != "IDLE"
            or not client.get("services", {}).get("start")
            or not client.get("services", {}).get("end")
        ):
            raise HostError(
                "STACK_NOT_READY",
                "runtime did not publish initial IDLE with both services discoverable",
                client=client,
            )
        refreshed = _refresh(
            operation.docker,
            expected,
            (
                (runtime.identifier, "runtime"),
                (backend.identifier, "backend"),
            ),
            operation.deadline,
        )
        for service, container in (("runtime", runtime), ("backend", backend)):
            current = refreshed.get(container.identifier)
            if current is None or not current.running:
                raise HostError(
                    "STACK_NOT_READY",
                    f"{service} stopped while checking readiness",
                    container=None if current is None else current.summary(),
                    client=client,
                )
    except HostError as error:
        cleanup = _cleanup_started(operation, expected)
        error.fields.setdefault("cleanup", cleanup)
        raise

    result = _base(arguments, "up")
    result.update(
        {
            "success": True,
            "reason": "STACK_READY",
            "status": client["status"],
            "services": client.get("services", {}),
            "containers": {
                "runtime": runtime.summary(),
                "backend": backend.summary(),
            },
            "client": client,
        }
    )
    return result


def _status(operation: Operation) -> dict[str, Any]:
    environment, expected = _environment(operation.arguments)
    _containers, runtime, backend = _resolve_stack(
        operation.docker,
        expected,
        operation.deadline,
    )
    result = _base(operation.arguments, "status")
    result["containers"] = {
        "runtime": runtime.summary(),
        "backend": backend.summary(),
    }
    if not runtime.running:
        result.update(
            {
                "success": False,
                "reason": "RUNTIME_NOT_RUNNING",
                "status": None,
                "healthy": False,
                "container": runtime.summary(),
            }
        )
        return result

    client_error: HostError | None = None
    client: dict[str, Any] | None = None
    try:
        client = _run_client(operation, environment, expected, "status")
    except HostError as error:
        client_error = error

    refreshed = _refresh(
        operation.docker,
        expected,
        (
            (runtime.identifier, "runtime"),
            (backend.identifier, "backend"),
        ),
        operation.deadline,
    )
    current_runtime = refreshed.get(runtime.identifier)
    current_backend = refreshed.get(backend.identifier)
    result["containers"] = {
        "runtime": None if current_runtime is None else current_runtime.summary(),
        "backend": None if current_backend is None else current_backend.summary(),
    }
    if current_runtime is None or not current_runtime.running:
        result.update(
            {
                "success": False,
                "reason": "RUNTIME_EXITED",
                "status": None if client is None else client.get("status"),
                "healthy": False,
                "client": client,
            }
        )
        result["container"] = None if current_runtime is None else current_runtime.summary()
        return result
    if client_error is not None:
        result.update(
            {
                "success": False,
                "reason": client_error.reason,
                "status": None,
                "healthy": False,
                **client_error.fields,
            }
        )
        return result
    assert client is not None
    result.update(client)
    result["operation"] = "status"
    result["run_id"] = operation.arguments.run_id
    result["audit_verified"] = False
    result["containers"] = {
        "runtime": current_runtime.summary(),
        "backend": None if current_backend is None else current_backend.summary(),
    }
    result["healthy"] = bool(
        client.get("healthy") and current_backend is not None and current_backend.running
    )
    if result["healthy"]:
        return result
    if current_backend is None or not current_backend.running:
        result["success"] = False
        result["reason"] = "BACKEND_NOT_RUNNING"
    elif client.get("success") is True and client.get("healthy") is False:
        result["success"] = False
        result["reason"] = "RUNTIME_FAULT"
    return result


def _start(operation: Operation) -> dict[str, Any]:
    environment, expected = _environment(operation.arguments)
    _containers, runtime, backend = _resolve_stack(
        operation.docker,
        expected,
        operation.deadline,
    )
    if not runtime.running:
        raise HostError(
            "RUNTIME_NOT_RUNNING",
            f"runtime is {runtime.state or 'missing'}",
            container=runtime.summary(),
        )
    if not backend.running:
        raise HostError(
            "BACKEND_NOT_RUNNING",
            f"backend is {backend.state or 'missing'}",
            container=backend.summary(),
        )
    extra = ["--task-id", operation.arguments.task_id]
    if operation.arguments.wait:
        extra.append("--wait")
    client = _run_client(operation, environment, expected, "start", extra)
    return _finish_client_operation(operation, expected, runtime, backend, client)


def _end(operation: Operation) -> dict[str, Any]:
    environment, expected = _environment(operation.arguments)
    _containers, runtime, backend = _resolve_stack(
        operation.docker,
        expected,
        operation.deadline,
    )
    if not runtime.running:
        raise HostError(
            "RUNTIME_NOT_RUNNING",
            f"runtime is {runtime.state or 'missing'}",
            container=runtime.summary(),
        )
    if not backend.running:
        raise HostError(
            "BACKEND_NOT_RUNNING",
            f"backend is {backend.state or 'missing'}",
            container=backend.summary(),
        )
    extra = [
        "--episode-id",
        operation.arguments.episode_id,
        "--reason",
        operation.arguments.reason,
    ]
    if operation.arguments.wait:
        extra.append("--wait")
    client = _run_client(operation, environment, expected, "end", extra)
    return _finish_client_operation(operation, expected, runtime, backend, client)


def _finish_client_operation(
    operation: Operation,
    expected: ExpectedStack,
    runtime: Container,
    backend: Container,
    client: dict[str, Any],
) -> dict[str, Any]:
    refreshed = _refresh(
        operation.docker,
        expected,
        (
            (runtime.identifier, "runtime"),
            (backend.identifier, "backend"),
        ),
        operation.deadline,
    )
    current_runtime = refreshed.get(runtime.identifier)
    current_backend = refreshed.get(backend.identifier)
    result = _base(operation.arguments, operation.arguments.operation)
    result.update(client)
    result["operation"] = operation.arguments.operation
    result["run_id"] = operation.arguments.run_id
    result["audit_verified"] = False
    result["containers"] = {
        "runtime": None if current_runtime is None else current_runtime.summary(),
        "backend": None if current_backend is None else current_backend.summary(),
    }
    if current_runtime is None or not current_runtime.running:
        result.update(
            {
                "success": False,
                "reason": "RUNTIME_EXITED",
                "client": client,
            }
        )
        result["container"] = None if current_runtime is None else current_runtime.summary()
    elif current_backend is None or not current_backend.running:
        result.update(
            {
                "success": False,
                "reason": "BACKEND_EXITED",
                "client": client,
            }
        )
        result["container"] = None if current_backend is None else current_backend.summary()
    return result


def _down(operation: Operation) -> dict[str, Any]:
    arguments = operation.arguments
    prepared = _prepare(arguments, ["down"])
    expected = ExpectedStack.from_environment(prepared.environment)
    identifiers = _project_ids(operation.docker, expected.project, operation.deadline)
    result = _base(arguments, "down")
    if not identifiers:
        result.update({"success": True, "reason": "STACK_ALREADY_STOPPED", "containers": []})
        return result

    try:
        _containers, runtime, backend = _resolve_stack(
            operation.docker,
            expected,
            operation.deadline,
        )
    except HostError as error:
        error.fields.setdefault("containers", identifiers)
        raise
    completed = operation.docker.run_argv(
        prepared.argv,
        operation.deadline,
        environment=prepared.environment,
    )
    remaining_ids = _project_ids(operation.docker, expected.project, operation.deadline)
    result["containers"] = {
        "runtime": runtime.summary(),
        "backend": backend.summary(),
    }
    result["compose_exit_code"] = int(completed.returncode)
    if completed.returncode != 0 or remaining_ids:
        result.update(
            {
                "success": False,
                "reason": "COMPOSE_DOWN_FAILED",
                "detail": _tail(completed.stderr) or "docker compose down failed",
                "remaining_containers": remaining_ids,
            }
        )
        return result
    result.update({"success": True, "reason": "STACK_STOPPED"})
    return result


def _parser() -> argparse.ArgumentParser:
    parser = _Parser(description=__doc__)
    parser.add_argument("--run-id", default=os.environ.get("DEV_RUN_ID"))
    parser.add_argument(
        "--project",
        default=os.environ.get("DEV_COMPOSE_PROJECT", dev_compose.DEFAULT_PROJECT),
    )
    parser.add_argument("--image", default=os.environ.get("DEV_IMAGE", dev_compose.DEFAULT_IMAGE))
    parser.add_argument("--build-root", type=Path)
    parser.add_argument("--raw-root", type=Path)
    parser.add_argument("--runs-root", type=Path)
    parser.add_argument("--docker", default=os.environ.get("DEV_DOCKER", "docker"))
    parser.add_argument(
        "--timeout",
        type=_timeout_argument,
        default=30.0,
        help="shared monotonic deadline in seconds for the whole command (default: 30)",
    )
    subcommands = parser.add_subparsers(dest="operation", required=True)
    subcommands.add_parser("up", help="start a fresh stack and wait until it is ready")
    subcommands.add_parser("status", help="report the latest status and container liveness")
    start = subcommands.add_parser("start", help="start one episode")
    start.add_argument("--task-id", required=True)
    start.add_argument("--wait", action="store_true", help="wait for completion and manifest")
    end = subcommands.add_parser("end", help="end the named current episode")
    end.add_argument("--episode-id", required=True)
    end.add_argument("--reason", default="OPERATOR_STOP")
    end.add_argument("--wait", action="store_true", help="wait for the stopped manifest")
    subcommands.add_parser("down", help="stop the stack without deleting mounted products")
    return parser


def _operation_hint(argv: Sequence[str]) -> str:
    for token in argv:
        if token in {"up", "status", "start", "end", "down"}:
            return token
    return "cli"


def _run_id_hint(argv: Sequence[str]) -> str:
    for index, token in enumerate(argv):
        if token == "--run-id" and index + 1 < len(argv):
            return argv[index + 1]
        if token.startswith("--run-id="):
            return token.split("=", 1)[1]
    return ""


def _validate_arguments(arguments: argparse.Namespace) -> None:
    if not arguments.run_id:
        raise UsageError("--run-id is required for every operation")
    try:
        dev_compose.validate_run_id(arguments.run_id)
    except dev_compose.DevComposeError as error:
        raise UsageError(str(error)) from error
    if arguments.operation == "end" and not str(arguments.episode_id).strip():
        raise UsageError("--episode-id must not be empty")
    if arguments.operation == "start" and not str(arguments.task_id).strip():
        raise UsageError("--task-id must not be empty")


def execute(
    arguments: argparse.Namespace,
    *,
    runner: Callable[..., subprocess.CompletedProcess] | None = None,
) -> dict[str, Any]:
    _validate_arguments(arguments)
    try:
        timeout = positive_timeout(arguments.timeout)
    except ValueError as error:
        raise UsageError(str(error)) from error
    deadline = time.monotonic() + timeout
    operation = Operation(arguments, Docker(arguments.docker, runner), deadline)
    if arguments.operation == "up":
        return _up(operation)
    if arguments.operation == "status":
        return _status(operation)
    if arguments.operation == "start":
        return _start(operation)
    if arguments.operation == "end":
        return _end(operation)
    if arguments.operation == "down":
        return _down(operation)
    raise UsageError(f"unknown operation {arguments.operation!r}")


def _failure(
    arguments: argparse.Namespace | None, operation: str, error: HostError
) -> dict[str, Any]:
    result = {
        "operation": operation,
        "run_id": "" if arguments is None else (arguments.run_id or ""),
        "success": False,
        "reason": error.reason,
        "detail": error.detail,
        "audit_verified": False,
        **error.fields,
    }
    return result


def main(
    argv: list[str] | None = None,
    *,
    runner: Callable[..., subprocess.CompletedProcess] | None = None,
) -> int:
    arguments_list = list(sys.argv[1:] if argv is None else argv)
    operation = _operation_hint(arguments_list)
    arguments: argparse.Namespace | None = None
    try:
        arguments = _parser().parse_args(arguments_list)
        result = execute(arguments, runner=runner)
        code = 0 if result.get("success") is True else 1
    except UsageError as error:
        print(f"mock entry: {error}", file=sys.stderr)
        result = {
            "operation": operation,
            "run_id": _run_id_hint(arguments_list),
            "success": False,
            "reason": "INVALID_ARGUMENT",
            "detail": str(error),
        }
        code = 2
    except HostError as error:
        result = _failure(arguments, operation, error)
        code = 1
    except KeyboardInterrupt:
        result = {
            "operation": operation,
            "run_id": "" if arguments is None else (arguments.run_id or ""),
            "success": False,
            "reason": "INTERRUPTED",
        }
        code = 130
    except Exception as error:
        print(f"mock entry: {type(error).__name__}: {error}", file=sys.stderr)
        result = {
            "operation": operation,
            "run_id": "" if arguments is None else (arguments.run_id or ""),
            "success": False,
            "reason": "INTERNAL_ERROR",
            "detail": f"{type(error).__name__}: {error}",
        }
        code = 1
    print(json.dumps(result, sort_keys=True), flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
