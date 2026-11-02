#!/usr/bin/env python3
"""Foreground launcher for the isolated native backend Compose project.

The launcher owns only one project/container: it prepares explicit read-only
mount variables, forwards signals to Compose, and always removes its own
project on exit.  It reuses ``dev_compose`` path/run-id validation instead of
copying the mock/sim-check lifecycle framework.  Physics and ROS lifecycle
stay in ``robot_backend_sim.node`` inside the container.
"""

from __future__ import annotations

import argparse
import math
import os
import re
import signal
import subprocess
import sys
import time
import uuid
from datetime import UTC, datetime
from pathlib import Path

_TOOLS_RUN = Path(__file__).resolve().parent
if str(_TOOLS_RUN) not in sys.path:
    sys.path.insert(0, str(_TOOLS_RUN))

import dev_compose  # noqa: E402

ROOT = dev_compose.ROOT
COMPOSE_FILE = ROOT / "deploy/compose/native_backend.yaml"
DEFAULT_IMAGE = "sim-dual-arm/backend-sim:mujoco-dev"
DEFAULT_BUILD_ROOT = ROOT / ".build/ros-dev"
DEFAULT_ASSETS_ROOT = ROOT / "third_party/src/mujoco_menagerie"
DEFAULT_SUMMARY = ROOT / ".build/assets/panda.json"
DEFAULT_CONFIG = ROOT / "configs/simulation/backend.yaml"
DEFAULT_RUN_ROOT = ROOT / ".build/runs/native-backend"
PROJECT_PATTERN = re.compile(r"^[a-z0-9][a-z0-9_-]{1,62}$")
CLEANUP_TIMEOUT_S = 30.0
EXIT_USAGE = 2


class UsageError(Exception):
    """Invalid invocation before Compose is started."""


def _absolute(value: Path | None, default: Path) -> Path:
    path = default if value is None else Path(value)
    if not path.is_absolute():
        path = ROOT / path
    return path.resolve()


def _require_file(path: Path, label: str) -> Path:
    if not path.is_file():
        raise UsageError(f"{label} must be an existing file: {path}")
    return path


def _require_dir(path: Path, label: str) -> Path:
    if not path.is_dir():
        raise UsageError(f"{label} must be an existing directory: {path}")
    return path


def _new_run_id() -> str:
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
    return f"native-{stamp}-{uuid.uuid4().hex[:6]}"


def _container_config(path: Path) -> str:
    try:
        relative = path.relative_to(ROOT)
    except ValueError as error:
        raise UsageError(
            "--config must live under the repository root so its relative asset/config "
            "paths remain valid inside the read-only /workspace mount"
        ) from error
    return f"/workspace/{relative.as_posix()}"


def parse_arguments(argv: list[str] | None = None) -> tuple[argparse.Namespace, dict[str, str]]:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--assets-root", type=Path, default=DEFAULT_ASSETS_ROOT)
    parser.add_argument("--asset-summary", type=Path, default=DEFAULT_SUMMARY)
    parser.add_argument("--run-root", type=Path, default=DEFAULT_RUN_ROOT)
    parser.add_argument("--run-id")
    parser.add_argument("--build-root", type=Path, default=DEFAULT_BUILD_ROOT)
    parser.add_argument("--image", default=os.environ.get("NATIVE_IMAGE", DEFAULT_IMAGE))
    parser.add_argument("--project")
    parser.add_argument("--docker", default=os.environ.get("DEV_DOCKER", "docker"))
    parser.add_argument("--timeout", type=float, default=0.0, help="0 means no overall deadline")
    parsed = parser.parse_args(argv)
    try:
        run_id = dev_compose.validate_run_id(parsed.run_id or _new_run_id())
    except dev_compose.DevComposeError as error:
        raise UsageError(str(error)) from error
    explicit_project = parsed.project is not None
    project = parsed.project or f"native-backend-{uuid.uuid4().hex[:10]}"
    if not PROJECT_PATTERN.fullmatch(project):
        raise UsageError("--project must be 2-63 lowercase letters, digits, '_' or '-'")
    if not math.isfinite(parsed.timeout) or parsed.timeout < 0:
        raise UsageError("--timeout must be a finite non-negative number of seconds")
    config = _require_file(_absolute(parsed.config, DEFAULT_CONFIG), "config")
    assets_root = _require_dir(_absolute(parsed.assets_root, DEFAULT_ASSETS_ROOT), "assets root")
    summary = _require_file(_absolute(parsed.asset_summary, DEFAULT_SUMMARY), "asset summary")
    build_root = _require_dir(_absolute(parsed.build_root, DEFAULT_BUILD_ROOT), "build root")
    run_root = dev_compose._validate_output_root(
        _absolute(parsed.run_root, DEFAULT_RUN_ROOT), "run root"
    )
    run_dir = (run_root / run_id).resolve()
    if run_dir.exists():
        raise UsageError(f"run directory already exists: {run_dir}")
    run_root.mkdir(parents=True, exist_ok=True)
    environment = {
        "NATIVE_PROJECT": project,
        "NATIVE_IMAGE": str(parsed.image),
        "NATIVE_REPO_ROOT": str(ROOT),
        "NATIVE_BUILD_ROOT": str(build_root),
        "NATIVE_ASSETS_ROOT": str(assets_root),
        "NATIVE_ASSET_SUMMARY": str(summary),
        "NATIVE_CONFIG_CONTAINER": _container_config(config),
        "NATIVE_RUN_ROOT": str(run_root),
        "NATIVE_RUN_ID": run_id,
        "NATIVE_UID": str(os.getuid()),
        "NATIVE_GID": str(os.getgid()),
        "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "42"),
    }
    parsed.run_id = run_id
    parsed.project = project
    parsed.project_explicit = explicit_project
    parsed.run_dir = run_dir
    parsed.docker = str(parsed.docker)
    return parsed, environment


def _project_resources(docker: str, project: str) -> tuple[str, ...]:
    """Return existing containers/networks belonging to an explicit project."""
    found: list[str] = []
    for arguments in (
        ["ps", "-aq", "--filter", f"label=com.docker.compose.project={project}"],
        ["network", "ls", "-q", "--filter", f"label=com.docker.compose.project={project}"],
    ):
        try:
            result = subprocess.run(
                [docker, *arguments],
                cwd=ROOT,
                check=False,
                capture_output=True,
                text=True,
                timeout=10.0,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            raise UsageError(f"cannot query Docker project {project!r}: {error}") from error
        if result.returncode != 0:
            raise UsageError(f"cannot query Docker project {project!r}: {result.stderr.strip()}")
        found.extend(line for line in result.stdout.splitlines() if line.strip())
    return tuple(found)


def _stop_project(
    process: subprocess.Popen, parsed: argparse.Namespace, environment: dict[str, str]
) -> None:
    """Stop owned containers once through Docker, then reap the foreground CLI.

    Signalling the entire Docker CLI/plugin process group can deliver duplicate
    stop signals. The explicit project command leaves signal forwarding to the
    daemon and gives the backend its configured close interval.
    """
    result = _compose(
        parsed.docker,
        parsed.project,
        ["stop", "--timeout", "20", "native-backend"],
        environment,
        timeout=CLEANUP_TIMEOUT_S,
    )
    if result.returncode:
        sys.stderr.write(f"native_backend: compose stop failed: {result.returncode}\n")
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def _compose(
    docker: str,
    project: str,
    arguments: list[str],
    environment: dict[str, str],
    timeout: float | None = None,
) -> subprocess.CompletedProcess:
    argv = [
        docker,
        "compose",
        "-f",
        str(COMPOSE_FILE),
        "-p",
        project,
        *arguments,
    ]
    return subprocess.run(
        argv,
        cwd=ROOT,
        env={**os.environ, **environment},
        check=False,
        timeout=timeout,
    )


def main(argv: list[str] | None = None) -> int:
    try:
        parsed, environment = parse_arguments(argv)
    except UsageError as error:
        sys.stderr.write(f"native_backend: {error}\n")
        return EXIT_USAGE
    if parsed.project_explicit:
        try:
            resources = _project_resources(parsed.docker, parsed.project)
        except UsageError as error:
            sys.stderr.write(f"native_backend: {error}\n")
            return EXIT_USAGE
        if resources:
            sys.stderr.write(
                f"native_backend: project {parsed.project!r} already has resources; "
                "refusing to take ownership of a user or previous run project\n"
            )
            return EXIT_USAGE
    stop_signal = {"value": 0}

    def remember(signum, _frame) -> None:
        if not stop_signal["value"]:
            stop_signal["value"] = signum

    previous = {signum: signal.getsignal(signum) for signum in (signal.SIGINT, signal.SIGTERM)}
    for signum in previous:
        signal.signal(signum, remember)
    argv_up = [
        "up",
        "--abort-on-container-exit",
        "--exit-code-from",
        "native-backend",
    ]
    code = 1
    timed_out = False
    process: subprocess.Popen | None = None
    try:
        process = subprocess.Popen(
            [parsed.docker, "compose", "-f", str(COMPOSE_FILE), "-p", parsed.project, *argv_up],
            cwd=ROOT,
            env={**os.environ, **environment},
            start_new_session=True,
        )
        deadline = None if parsed.timeout == 0 else time.monotonic() + parsed.timeout
        while process.poll() is None:
            if stop_signal["value"]:
                _stop_project(process, parsed, environment)
                break
            if deadline is not None and time.monotonic() >= deadline:
                timed_out = True
                _stop_project(process, parsed, environment)
                break
            try:
                process.wait(timeout=0.2)
            except subprocess.TimeoutExpired:
                continue
        code = process.wait()
        if timed_out:
            code = 124
        elif stop_signal["value"]:
            code = 128 + stop_signal["value"]
    except FileNotFoundError:
        sys.stderr.write(f"native_backend: docker executable not found: {parsed.docker}\n")
        code = 1
    finally:
        try:
            cleanup = _compose(
                parsed.docker,
                parsed.project,
                ["down", "--remove-orphans", "--timeout", "20"],
                environment,
                timeout=CLEANUP_TIMEOUT_S,
            )
            if cleanup.returncode != 0:
                sys.stderr.write(
                    f"native_backend: compose cleanup failed with exit {cleanup.returncode}\n"
                )
                if code == 0:
                    code = 1
        except Exception as error:  # noqa: BLE001 - cleanup failure must be visible
            sys.stderr.write(f"native_backend: compose cleanup failed: {error}\n")
            if code == 0:
                code = 1
        for signum, handler in previous.items():
            signal.signal(signum, handler)
    sys.stdout.write(
        f"native_backend: run_id={parsed.run_id} project={parsed.project} "
        f"run_dir={parsed.run_dir} exit={code} timed_out={timed_out}\n"
    )
    return code if code >= 0 else 128 - code


if __name__ == "__main__":
    raise SystemExit(main())
