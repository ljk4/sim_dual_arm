#!/usr/bin/env python3
"""Thin launcher for backend + native runtime + fake Policy RPC.

It reuses the accepted ``native_backend`` project guard and ``dev_compose``
path/run-id validation.  It never copies the old large collection launcher and
never starts a training job.  Signal, overall timeout, explicit stop and project
cleanup stay in this one small lifecycle surface.
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
import native_backend  # noqa: E402

ROOT = dev_compose.ROOT
COMPOSE_FILES = (
    ROOT / "deploy/compose/native_backend.yaml",
    ROOT / "deploy/compose/native_runtime.yaml",
    ROOT / "deploy/compose/native_policy.yaml",
)
DEFAULT_CONFIG = ROOT / "configs/runtime/native_policy.yaml"
DEFAULT_POLICY_CONFIG = ROOT / "configs/learning/fake_policy.yaml"
DEFAULT_ACT_POLICY_CONFIG = ROOT / "configs/learning/act_policy.yaml"
DEFAULT_BACKEND_IMAGE = "sim-dual-arm/backend-sim:follower-dev"
DEFAULT_SDK_ROOT = ROOT / ".build/generated/policy-sdk"
DEFAULT_BUILD_ROOT = ROOT / ".build/ros-dev"
DEFAULT_ASSETS_ROOT = ROOT / "third_party/src/mujoco_menagerie"
DEFAULT_SUMMARY = ROOT / ".build/assets/panda.json"
DEFAULT_DATA_ROOT = ROOT / ".build/runs/native-policy"
DEFAULT_LEARNING_IMAGE = "sim-dual-arm/learning:act-dev"
DEFAULT_LEARNING_VENV = ROOT / ".build/learning-dev-venv"
DEFAULT_LEARNING_SOURCE = ROOT / "third_party/src/lerobot-act-v0.6.1"
DEFAULT_LEARNING_CACHE = ROOT / ".build/cache"
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
    return f"policy-{stamp}-{uuid.uuid4().hex[:6]}"


def _container_path(path: Path, label: str) -> str:
    try:
        relative = path.relative_to(ROOT)
    except ValueError as error:
        raise UsageError(
            f"{label} must live under the repository root so its relative "
            "profile/schema paths remain valid inside the read-only mount"
        ) from error
    return f"/workspace/{relative.as_posix()}"


def parse_arguments(argv: list[str] | None = None) -> tuple[argparse.Namespace, dict[str, str]]:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--policy-mode", choices=("fake", "act"), default="fake")
    parser.add_argument("--policy-config", type=Path, default=DEFAULT_POLICY_CONFIG)
    parser.add_argument("--act-policy-config", type=Path, default=DEFAULT_ACT_POLICY_CONFIG)
    parser.add_argument("--bundle", type=Path, default=None)
    parser.add_argument(
        "--learning-image",
        default=os.environ.get("NATIVE_ACT_IMAGE", DEFAULT_LEARNING_IMAGE),
    )
    parser.add_argument("--learning-venv", type=Path, default=DEFAULT_LEARNING_VENV)
    parser.add_argument("--learning-source", type=Path, default=DEFAULT_LEARNING_SOURCE)
    parser.add_argument("--learning-cache", type=Path, default=DEFAULT_LEARNING_CACHE)
    parser.add_argument(
        "--backend-config", type=Path, default=ROOT / "configs/simulation/backend_follower.yaml"
    )
    parser.add_argument("--stack-mode", choices=("policy", "teacher"), default="policy")
    parser.add_argument("--episode-id", default="")
    parser.add_argument("--policy-sdk", type=Path, default=DEFAULT_SDK_ROOT)
    parser.add_argument("--assets-root", type=Path, default=DEFAULT_ASSETS_ROOT)
    parser.add_argument("--asset-summary", type=Path, default=DEFAULT_SUMMARY)
    parser.add_argument("--build-root", type=Path, default=DEFAULT_BUILD_ROOT)
    parser.add_argument("--data-root", type=Path, default=DEFAULT_DATA_ROOT)
    parser.add_argument("--run-id")
    parser.add_argument("--image", default=os.environ.get("NATIVE_IMAGE", DEFAULT_BACKEND_IMAGE))
    parser.add_argument(
        "--policy-image",
        default=os.environ.get("NATIVE_POLICY_IMAGE", "sim-dual-arm/learning:policy-dev"),
    )
    parser.add_argument("--project")
    parser.add_argument("--docker", default=os.environ.get("DEV_DOCKER", "docker"))
    parser.add_argument("--timeout", type=float, default=120.0)
    parsed = parser.parse_args(argv)
    try:
        run_id = dev_compose.validate_run_id(parsed.run_id or _new_run_id())
    except dev_compose.DevComposeError as error:
        raise UsageError(str(error)) from error
    explicit_project = parsed.project is not None
    project = parsed.project or f"native-policy-{uuid.uuid4().hex[:10]}"
    if not PROJECT_PATTERN.fullmatch(project):
        raise UsageError("--project must be 2-63 lowercase letters, digits, '_' or '-'")
    if not math.isfinite(parsed.timeout) or parsed.timeout <= 0:
        raise UsageError("--timeout must be a finite positive number of seconds")
    config = _require_file(_absolute(parsed.config, DEFAULT_CONFIG), "config")
    policy_config = _require_file(
        _absolute(parsed.policy_config, DEFAULT_POLICY_CONFIG), "policy config"
    )
    act_policy_config = _require_file(
        _absolute(parsed.act_policy_config, DEFAULT_ACT_POLICY_CONFIG), "ACT policy config"
    )
    bundle: Path | None = None
    if parsed.policy_mode == "act":
        if parsed.bundle is None:
            raise UsageError("--bundle is required for --policy-mode act")
        bundle = _require_dir(_absolute(parsed.bundle, DEFAULT_ACT_POLICY_CONFIG), "ACT bundle")
        _require_dir(_absolute(parsed.learning_venv, DEFAULT_LEARNING_VENV), "learning venv")
        _require_dir(_absolute(parsed.learning_source, DEFAULT_LEARNING_SOURCE), "LeRobot source")
        _require_dir(_absolute(parsed.learning_cache, DEFAULT_LEARNING_CACHE), "learning cache")
    sdk_root = _require_dir(_absolute(parsed.policy_sdk, DEFAULT_SDK_ROOT), "policy SDK")
    assets_root = _require_dir(_absolute(parsed.assets_root, DEFAULT_ASSETS_ROOT), "assets root")
    summary = _require_file(_absolute(parsed.asset_summary, DEFAULT_SUMMARY), "asset summary")
    build_root = _require_dir(_absolute(parsed.build_root, DEFAULT_BUILD_ROOT), "build root")
    data_root = dev_compose._validate_output_root(
        _absolute(parsed.data_root, DEFAULT_DATA_ROOT), "data root"
    )
    data_root.mkdir(parents=True, exist_ok=True)
    run_root = (data_root / run_id).resolve()
    if run_root.exists():
        raise UsageError(f"run directory already exists: {run_root}")
    run_root.mkdir(parents=True)
    environment = {
        "NATIVE_PROJECT": project,
        "NATIVE_STACK_MODE": parsed.stack_mode,
        "NATIVE_EPISODE_ID": parsed.episode_id,
        "NATIVE_IMAGE": str(parsed.image),
        "NATIVE_POLICY_IMAGE": str(parsed.policy_image),
        "NATIVE_ACT_IMAGE": str(parsed.learning_image),
        "NATIVE_POLICY_MODE": parsed.policy_mode,
        "NATIVE_ACT_BUNDLE": "" if bundle is None else str(bundle),
        "NATIVE_ACT_BUNDLE_CONTAINER": "/opt/act-bundle",
        "NATIVE_ACT_POLICY_CONFIG_CONTAINER": _container_path(
            act_policy_config, "ACT policy config"
        ),
        "NATIVE_LEARNING_VENV": str(_absolute(parsed.learning_venv, DEFAULT_LEARNING_VENV)),
        "NATIVE_LEARNING_SOURCE": str(
            _absolute(parsed.learning_source, DEFAULT_LEARNING_SOURCE)
        ),
        "NATIVE_LEARNING_CACHE": str(_absolute(parsed.learning_cache, DEFAULT_LEARNING_CACHE)),
        "NATIVE_REPO_ROOT": str(ROOT),
        "NATIVE_BUILD_ROOT": str(build_root),
        "NATIVE_ASSETS_ROOT": str(assets_root),
        "NATIVE_ASSET_SUMMARY": str(summary),
        "NATIVE_POLICY_SDK_ROOT": str(sdk_root),
        "NATIVE_CONFIG_CONTAINER": _container_path(
            _require_file(
                _absolute(parsed.backend_config, parsed.backend_config), "backend config"
            ),
            "backend config",
        ),
        "NATIVE_RUNTIME_CONFIG_CONTAINER": _container_path(config, "runtime config"),
        "NATIVE_POLICY_CONFIG_CONTAINER": _container_path(policy_config, "policy config"),
        "NATIVE_DATA_ROOT": str(run_root),
        "NATIVE_RUN_ROOT": str(run_root),
        "NATIVE_RUN_ID": run_id,
        "NATIVE_UID": str(os.getuid()),
        "NATIVE_GID": str(os.getgid()),
        "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "42"),
        "STACK_REPLAY_LABELS": os.environ.get("STACK_REPLAY_LABELS", ""),
        "STACK_REPLAY_LABELS_HOST": os.environ.get("STACK_REPLAY_LABELS_HOST", ""),
    }
    parsed.run_id = run_id
    parsed.project = project
    parsed.project_explicit = explicit_project
    parsed.run_root = run_root
    parsed.docker = str(parsed.docker)
    return parsed, environment


def _compose_files(environment: dict[str, str]) -> tuple[Path, ...]:
    if environment.get("NATIVE_STACK_MODE") == "teacher":
        return (*COMPOSE_FILES[:2], ROOT / "deploy/compose/native_task.yaml")
    if environment.get("NATIVE_POLICY_MODE") == "act":
        return (
            *COMPOSE_FILES[:2],
            ROOT / "deploy/compose/native_act_policy.yaml",
        )
    if environment.get("STACK_REPLAY_LABELS_HOST"):
        return (*COMPOSE_FILES, ROOT / "deploy/compose/label_replay.yaml")
    return COMPOSE_FILES


def _compose(
    docker: str,
    project: str,
    arguments: list[str],
    environment: dict[str, str],
    timeout: float | None = None,
) -> subprocess.CompletedProcess:
    argv = [docker, "compose"]
    for path in _compose_files(environment):
        argv.extend(["-f", str(path)])
    argv.extend(["-p", project, *arguments])
    return subprocess.run(
        argv,
        cwd=ROOT,
        env={**os.environ, **environment},
        check=False,
        timeout=timeout,
    )


def _stop_stack(
    process: subprocess.Popen, parsed: argparse.Namespace, environment: dict[str, str]
) -> None:
    result = _compose(
        parsed.docker,
        parsed.project,
        [
            "stop",
            "--timeout",
            "20",
            "native-runtime",
            *(
                []
                if parsed.stack_mode == "teacher"
                else ["act-policy" if parsed.policy_mode == "act" else "fake-policy"]
            ),
            "native-backend",
        ],
        environment,
        timeout=CLEANUP_TIMEOUT_S,
    )
    if result.returncode:
        sys.stderr.write(f"native_policy: compose stop failed: {result.returncode}\n")
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def main(argv: list[str] | None = None) -> int:
    try:
        parsed, environment = parse_arguments(argv)
    except UsageError as error:
        sys.stderr.write(f"native_policy: {error}\n")
        return EXIT_USAGE
    if parsed.project_explicit:
        try:
            resources = native_backend._project_resources(parsed.docker, parsed.project)
        except native_backend.UsageError as error:
            sys.stderr.write(f"native_policy: {error}\n")
            return EXIT_USAGE
        if resources:
            sys.stderr.write(
                f"native_policy: project {parsed.project!r} already has resources; "
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
    code = 1
    timed_out = False
    process: subprocess.Popen | None = None
    try:
        process = subprocess.Popen(
            [
                parsed.docker,
                "compose",
                *sum((["-f", str(path)] for path in _compose_files(environment)), []),
                "-p",
                parsed.project,
                "up",
                "--abort-on-container-exit",
                "--exit-code-from",
                "native-runtime",
            ],
            cwd=ROOT,
            env={**os.environ, **environment},
            start_new_session=True,
        )
        deadline = time.monotonic() + parsed.timeout
        while process.poll() is None:
            if stop_signal["value"]:
                _stop_stack(process, parsed, environment)
                break
            if time.monotonic() >= deadline:
                timed_out = True
                _stop_stack(process, parsed, environment)
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
        sys.stderr.write(f"native_policy: docker executable not found: {parsed.docker}\n")
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
            if cleanup.returncode:
                sys.stderr.write(
                    f"native_policy: compose cleanup failed with exit {cleanup.returncode}\n"
                )
                if code == 0:
                    code = 1
        except Exception as error:  # noqa: BLE001 - cleanup failure must be visible
            sys.stderr.write(f"native_policy: compose cleanup failed: {error}\n")
            if code == 0:
                code = 1
        for signum, handler in previous.items():
            signal.signal(signum, handler)
    return code


if __name__ == "__main__":
    sys.exit(main())
