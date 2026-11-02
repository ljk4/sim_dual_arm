#!/usr/bin/env python3
"""Run one bounded, one-shot MuJoCo environment check in Compose.

The host entry only prepares read-only mounts, a fresh run identity and the
Compose invocation.  Environment probing and report creation live in
``robot_backend_sim.probe_environment`` inside the container.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import sys
import time
import uuid
from collections.abc import Callable, Sequence
from dataclasses import dataclass, replace
from datetime import UTC, datetime
from pathlib import Path

_TOOLS_RUN = Path(__file__).resolve().parent
if str(_TOOLS_RUN) not in sys.path:
    sys.path.insert(0, str(_TOOLS_RUN))

import collect_artifacts  # noqa: E402
import dev_compose  # noqa: E402
import mock_stack  # noqa: E402

ROOT = dev_compose.ROOT
COMPOSE_FILE = ROOT / "deploy/compose/sim_check.yaml"
SOURCE_FILE = ROOT / "third_party/panda-source.json"
DEFAULT_IMAGE = "sim-dual-arm/backend-sim:mujoco-dev"
DEFAULT_BUILD_ROOT = ROOT / ".build/ros-dev"
DEFAULT_ASSETS_ROOT = ROOT / "third_party/src/mujoco_menagerie"
DEFAULT_SUMMARY = ROOT / ".build/assets/panda.json"
DEFAULT_REPORT_ROOT = ROOT / ".build/reports/sim-check"
DEFAULT_SCENE_CONFIG = ROOT / "configs/simulation/panda_stack.yaml"
DEFAULT_CHECK_CONFIG = ROOT / "configs/simulation/scene_check.yaml"
DEFAULT_KINEMATICS_CONFIG = ROOT / "configs/simulation/kinematics_check.yaml"
CONTAINER_SCENE_CONFIG = "/sim-configs/panda_stack.yaml"
CONTAINER_CHECK_CONFIG = "/sim-configs/scene_check.yaml"
CONTAINER_KINEMATICS_CONFIG = "/sim-configs/kinematics_check.yaml"
PROJECT_PREFIX = "sim-check"
PROJECT_PATTERN = re.compile(r"[a-z0-9][a-z0-9_-]{1,62}")
EXIT_SUCCESS = 0
EXIT_FAILURE = 1
EXIT_USAGE = 2
EXIT_TIMEOUT = 124
SIGNAL_EXIT_CODES = {signal.SIGINT: 130, signal.SIGTERM: 143}
DIAGNOSTIC_LIMIT_BYTES = 256 * 1024
CLEANUP_TIMEOUT_S = 20.0
REPORT_JSON_LIMIT = 8 * 1024 * 1024
CLEANUP_LABEL_KEY = "io.sim-dual-arm.sim-check.token"


class UsageError(Exception):
    """Invalid host invocation before a tool container is started."""


class StopRequested(RuntimeError):
    def __init__(self, signum: int) -> None:
        super().__init__(signal.Signals(signum).name)
        self.signum = signum


@dataclass(frozen=True)
class Arguments:
    image: str
    build_root: Path
    assets_root: Path
    summary: Path
    report_root: Path
    run_id: str
    project: str
    source: Path
    docker: str
    timeout_s: float
    container_name: str = ""
    cleanup_label: str = ""
    check: str = "environment"
    scene_config: Path = DEFAULT_SCENE_CONFIG
    check_config: Path = DEFAULT_CHECK_CONFIG
    kinematics_config: Path = DEFAULT_KINEMATICS_CONFIG


@dataclass(frozen=True)
class CommandResult:
    returncode: int
    stdout: str
    stderr: str


@dataclass(frozen=True)
class CheckOutcome:
    exit_code: int
    reason: str
    report_path: Path
    detail: str = ""
    stdout: str = ""
    stderr: str = ""


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", default=os.environ.get("SIM_IMAGE", DEFAULT_IMAGE))
    parser.add_argument("--build-root", type=Path, help="generated ROS workspace mount")
    parser.add_argument("--assets-root", type=Path, help="pinned Menagerie checkout root")
    parser.add_argument("--summary", type=Path, help="read-only asset summary JSON")
    parser.add_argument("--report-root", type=Path, help="host report parent directory")
    parser.add_argument("--run-id", help="fresh report/run identity")
    parser.add_argument("--project", help="isolated Compose project name")
    parser.add_argument("--docker", default=os.environ.get("DEV_DOCKER", "docker"))
    parser.add_argument("--timeout", type=float, default=600.0, help="overall command deadline")
    parser.add_argument(
        "--check",
        choices=("environment", "scene", "kinematics"),
        default="environment",
        help="environment probe (default), scene visibility or kinematics diagnostics",
    )
    parser.add_argument("--scene-config", type=Path, help="scene parameter YAML")
    parser.add_argument("--check-config", type=Path, help="scene-check parameter YAML")
    parser.add_argument("--kinematics-config", type=Path, help="kinematics parameter YAML")
    return parser


def _absolute(value: Path | None, default: Path) -> Path:
    path = default if value is None else value
    if not path.is_absolute():
        path = ROOT / path
    return path.resolve()


def _new_run_id() -> str:
    stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%SZ")
    return f"sim-{stamp}-{uuid.uuid4().hex[:6]}"


def _validate_run_id(value: str) -> str:
    try:
        return dev_compose.validate_run_id(value)
    except dev_compose.DevComposeError as error:
        raise UsageError(str(error)) from error


def _validate_report_root(path: Path) -> Path:
    try:
        return dev_compose._validate_output_root(path, "report root")
    except dev_compose.DevComposeError as error:
        raise UsageError(str(error)) from error


def _require_directory(path: Path, label: str) -> Path:
    resolved = path.resolve()
    if not resolved.is_dir():
        raise UsageError(f"{label} must be an existing directory: {resolved}")
    return resolved


def _require_file(path: Path, label: str) -> Path:
    resolved = path.resolve()
    if not resolved.is_file():
        raise UsageError(f"{label} must be an existing regular file: {resolved}")
    return resolved


def parse_arguments(argv: Sequence[str] | None = None) -> Arguments:
    parsed = _parser().parse_args(argv)
    run_id = _validate_run_id(parsed.run_id or _new_run_id())
    project = parsed.project or f"{PROJECT_PREFIX}-{uuid.uuid4().hex[:10]}"
    if not PROJECT_PATTERN.fullmatch(project):
        raise UsageError(
            "project must contain 2-63 lowercase letters, digits, '_' or '-' "
            "and start with a letter or digit"
        )
    timeout = float(parsed.timeout)
    if not (0 < timeout <= 3600):
        raise UsageError("--timeout must be greater than 0 and at most 3600 seconds")
    report_root = _validate_report_root(_absolute(parsed.report_root, DEFAULT_REPORT_ROOT))
    scene_config = _require_file(
        _absolute(parsed.scene_config, DEFAULT_SCENE_CONFIG), "scene config"
    )
    check_config = _require_file(
        _absolute(parsed.check_config, DEFAULT_CHECK_CONFIG), "check config"
    )
    kinematics_config = _require_file(
        _absolute(parsed.kinematics_config, DEFAULT_KINEMATICS_CONFIG),
        "kinematics config",
    )
    return Arguments(
        image=str(parsed.image),
        build_root=_require_directory(
            _absolute(parsed.build_root, DEFAULT_BUILD_ROOT), "build root"
        ),
        assets_root=_require_directory(
            _absolute(parsed.assets_root, DEFAULT_ASSETS_ROOT), "assets root"
        ),
        summary=_require_file(_absolute(parsed.summary, DEFAULT_SUMMARY), "asset summary"),
        report_root=report_root,
        run_id=run_id,
        project=project,
        source=_require_file(SOURCE_FILE, "source description"),
        docker=str(parsed.docker),
        timeout_s=timeout,
        container_name=f"sim-check-{uuid.uuid4().hex[:12]}",
        cleanup_label=uuid.uuid4().hex,
        check=parsed.check,
        scene_config=scene_config,
        check_config=check_config,
        kinematics_config=kinematics_config,
    )


def report_path(arguments: Arguments) -> Path:
    return arguments.report_root / arguments.run_id / "report.json"


def scene_command(run_id: str) -> list[str]:
    """Fixed in-container scene command; no caller-supplied module string."""
    return [
        "python",
        "-m",
        "robot_backend_sim.check_scene",
        "--scene-config",
        CONTAINER_SCENE_CONFIG,
        "--check-config",
        CONTAINER_CHECK_CONFIG,
        "--assets-root",
        "/sim-assets",
        "--summary",
        "/sim-assets-summary/panda.json",
        "--source",
        "/workspace/third_party/panda-source.json",
        "--packages",
        "/workspace/deploy/docker/sim-packages.json",
        "--report-root",
        "/sim-reports",
        "--run-id",
        run_id,
    ]


def kinematics_command(run_id: str) -> list[str]:
    """Fixed in-container kinematics command; no caller-supplied module string."""
    return [
        "python",
        "-m",
        "robot_backend_sim.check_kinematics",
        "--scene-config",
        CONTAINER_SCENE_CONFIG,
        "--check-config",
        CONTAINER_CHECK_CONFIG,
        "--kinematics-config",
        CONTAINER_KINEMATICS_CONFIG,
        "--assets-root",
        "/sim-assets",
        "--summary",
        "/sim-assets-summary/panda.json",
        "--source",
        "/workspace/third_party/panda-source.json",
        "--packages",
        "/workspace/deploy/docker/sim-packages.json",
        "--report-root",
        "/sim-reports",
        "--run-id",
        run_id,
    ]


def compose_argv(arguments: Arguments) -> list[str]:
    argv = [
        arguments.docker,
        "compose",
        "-p",
        arguments.project,
        "--project-directory",
        str(ROOT),
        "-f",
        str(COMPOSE_FILE),
        "run",
        "--rm",
        "--no-deps",
        "-T",
        "--name",
        arguments.container_name,
        "--label",
        f"{CLEANUP_LABEL_KEY}={arguments.cleanup_label}",
        "sim-check",
    ]
    if arguments.check == "scene":
        argv.extend(scene_command(arguments.run_id))
    elif arguments.check == "kinematics":
        argv.extend(kinematics_command(arguments.run_id))
    return argv


def compose_environment(arguments: Arguments) -> dict[str, str]:
    environment = os.environ.copy()
    # The Compose file supplies the only container PYTHONPATH.  Do not let a
    # host value leak into the tool container even through the CLI environment.
    environment.pop("PYTHONPATH", None)
    environment.update(
        {
            "SIM_COMPOSE_PROJECT": arguments.project,
            "SIM_IMAGE": arguments.image,
            "SIM_REPO_ROOT": str(ROOT),
            "SIM_BUILD_ROOT": str(arguments.build_root),
            "SIM_ASSETS_ROOT": str(arguments.assets_root),
            "SIM_ASSET_SUMMARY": str(arguments.summary),
            "SIM_REPORT_ROOT": str(arguments.report_root),
            "SIM_RUN_ID": arguments.run_id,
            "SIM_UID": str(os.getuid()),
            "SIM_GID": str(os.getgid()),
            "SIM_SCENE_CONFIG": str(arguments.scene_config),
            "SIM_CHECK_CONFIG": str(arguments.check_config),
            "SIM_KINEMATICS_CONFIG": str(arguments.kinematics_config),
        }
    )
    return environment


def _bounded(text: str, limit: int = 8192) -> str:
    return text if len(text) <= limit else text[-limit:]


def _bounded_docker(arguments: Arguments) -> mock_stack.BoundedDocker:
    return mock_stack.BoundedDocker(
        arguments.docker,
        output_limit=DIAGNOSTIC_LIMIT_BYTES,
    )


def cleanup_container(
    arguments: Arguments,
    *,
    docker: mock_stack.BoundedDocker | None = None,
) -> list[str]:
    """Remove only this invocation's single named, token-labelled container."""
    runner = _bounded_docker(arguments) if docker is None else docker
    name = arguments.container_name
    token = arguments.cleanup_label
    if not name or not token:
        return ["cleanup identity is missing"]
    deadline = time.monotonic() + CLEANUP_TIMEOUT_S
    try:
        inspected = runner.run(
            [
                "inspect",
                "--format",
                "{{.Id}}\n{{.Name}}\n{{json .Config.Labels}}",
                name,
            ],
            deadline,
        )
    except mock_stack.HostError as error:
        return [f"inspect: {error.reason}: {error}"]
    detail = _bounded((inspected.stdout or "") + (inspected.stderr or ""))
    if inspected.returncode != 0:
        lowered = detail.lower()
        if "no such object" in lowered or "no such container" in lowered:
            return []
        return [f"inspect failed: {detail}"]
    lines = inspected.stdout.splitlines()
    if len(lines) < 3:
        return [f"inspect returned incomplete output: {detail}"]
    identifier = lines[0].strip()
    if not re.fullmatch(r"[0-9a-f]{64}", identifier):
        return ["inspect returned an invalid container ID; container preserved"]
    if lines[1].strip() != f"/{name}":
        return [f"inspect name mismatch: {lines[1]!r}, expected /{name}"]
    try:
        labels = json.loads("\n".join(lines[2:]))
    except (TypeError, ValueError) as error:
        return [f"inspect labels unreadable: {type(error).__name__}"]
    if not isinstance(labels, dict) or labels.get(CLEANUP_LABEL_KEY) != token:
        return ["inspect label token does not match this invocation; container preserved"]
    try:
        removed = runner.run(["rm", "-f", identifier], deadline)
    except mock_stack.HostError as error:
        return [f"remove: {error.reason}: {error}"]
    if removed.returncode != 0:
        return [f"remove failed: {_bounded((removed.stdout or '') + (removed.stderr or ''))}"]
    return []


def _project_has_containers(arguments: Arguments) -> bool:
    runner = _bounded_docker(arguments)
    try:
        completed = runner.run(
            [
                "ps",
                "-aq",
                "--filter",
                f"label=com.docker.compose.project={arguments.project}",
            ],
            time.monotonic() + 15.0,
        )
    except mock_stack.HostError as error:
        raise UsageError(
            f"cannot query the isolated Compose project: {error.reason}: {error}"
        ) from error
    if completed.returncode != 0:
        detail = _bounded((completed.stderr or completed.stdout).strip())
        raise UsageError(f"cannot query the isolated Compose project: {detail}")
    return bool(completed.stdout.strip())


def _run_process(
    arguments: Arguments,
    *,
    timeout_s: float,
) -> tuple[CommandResult, bool]:
    """Run the foreground Compose CLI through the shared bounded Docker helper."""
    runner = _bounded_docker(arguments)
    deadline = time.monotonic() + max(0.1, timeout_s)
    try:
        result = runner.run_argv(
            compose_argv(arguments),
            deadline,
            environment=compose_environment(arguments),
        )
    except mock_stack.HostError as error:
        if error.reason == "COMMAND_TIMEOUT":
            return CommandResult(EXIT_TIMEOUT, "", ""), True
        raise
    return (
        CommandResult(
            int(result.returncode),
            _bounded(result.stdout),
            _bounded(result.stderr),
        ),
        False,
    )


def verify_report(
    path: Path,
    *,
    run_id: str,
    process_exit: int,
    kind: str | None = None,
) -> tuple[str, str, dict[str, object] | None]:
    """Return (status, reason, report); status is succeeded/failed/mismatch."""
    if path.is_symlink():
        return "mismatch", "report_symlink", None
    try:
        document = collect_artifacts.read_json_file(path, limit=REPORT_JSON_LIMIT)
    except FileNotFoundError:
        return "mismatch", "report_missing", None
    except ValueError as error:
        text = str(error)
        if "larger than" in text:
            return "mismatch", "report_too_large", None
        if "regular JSON file" in text:
            return "mismatch", "report_not_regular", None
        return "mismatch", "report_unreadable", None
    except (OSError, UnicodeError):
        return "mismatch", "report_unreadable", None
    if not isinstance(document, dict):
        return "mismatch", "report_not_object", None
    if document.get("run_id") != run_id:
        return "mismatch", "report_run_id_mismatch", document
    if kind in {"scene", "kinematics"} and document.get("kind") != kind:
        return "mismatch", "report_kind_mismatch", document
    status = document.get("status")
    if status not in {"succeeded", "failed"}:
        return "mismatch", "report_status_invalid", document
    report_exit = document.get("exit_code")
    if status == "succeeded":
        if process_exit != 0 or report_exit != 0:
            return "mismatch", "report_success_but_process_failed", document
        if document.get("logs_closed") is not True:
            return "mismatch", "report_logs_not_closed", document
        artifact = document.get("artifact")
        if not isinstance(artifact, str) or not artifact:
            return "mismatch", "report_artifact_missing", document
        artifact_path = (path.parent / artifact).resolve()
        report_parent = path.parent.resolve()
        if report_parent not in artifact_path.parents:
            return "mismatch", "report_artifact_outside_run", document
        if not artifact_path.is_file() or artifact_path.stat().st_size <= 0:
            return "mismatch", "report_artifact_missing", document
        return "succeeded", "", document
    if process_exit == 0:
        return "mismatch", "report_failed_but_process_succeeded", document
    if report_exit == 0:
        return "mismatch", "report_failed_but_exit_zero", document
    reason = document.get("reason")
    if not isinstance(reason, str) or not reason:
        return "mismatch", "report_reason_missing", document
    return "failed", reason, document


def execute(arguments: Arguments) -> CheckOutcome:
    if not arguments.container_name or not arguments.cleanup_label:
        arguments = replace(
            arguments,
            container_name=f"sim-check-{uuid.uuid4().hex[:12]}",
            cleanup_label=uuid.uuid4().hex,
        )
    destination = report_path(arguments)
    if os.path.lexists(destination.parent):
        raise UsageError(
            f"run id {arguments.run_id!r} already has a report directory: {destination.parent}"
        )
    if _project_has_containers(arguments):
        raise UsageError(
            f"Compose project {arguments.project!r} already owns containers; choose a fresh project"
        )
    arguments.report_root.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    result: CommandResult | None = None
    timed_out = False
    signal_code: int | None = None
    primary_reason: str | None = None
    primary_detail = ""
    try:
        result, timed_out = _run_process(arguments, timeout_s=arguments.timeout_s)
    except StopRequested as stop:
        signal_code = SIGNAL_EXIT_CODES.get(stop.signum, EXIT_FAILURE)
        primary_reason = "interrupted"
    except KeyboardInterrupt:
        signal_code = 130
        primary_reason = "interrupted"
    except mock_stack.HostError as error:
        primary_reason = (
            "docker_unavailable" if error.reason == "DOCKER_UNAVAILABLE" else "docker_error"
        )
        primary_detail = f"{error.reason}: {error}"
    except BaseException as error:
        primary_reason = "host_error"
        primary_detail = f"{type(error).__name__}: {error}"

    # Every branch after the tool container may have been created gets one
    # bounded cleanup.  Cleanup is never allowed to turn a failed run into a
    # success, and only this invocation's name+token container can be removed.
    try:
        diagnostics = cleanup_container(arguments)
    except Exception as error:
        diagnostics = [f"cleanup raised {type(error).__name__}: {error}"]
    cleanup_detail = "; ".join(diagnostics)

    if signal_code is not None:
        return CheckOutcome(
            signal_code,
            primary_reason or "interrupted",
            destination,
            cleanup_detail,
        )
    if timed_out:
        assert result is not None
        return CheckOutcome(
            EXIT_TIMEOUT,
            "timeout",
            destination,
            cleanup_detail,
            result.stdout,
            result.stderr,
        )
    if primary_reason is not None:
        detail = primary_detail
        if cleanup_detail:
            detail = "; ".join(part for part in (detail, cleanup_detail) if part)
        return CheckOutcome(EXIT_FAILURE, primary_reason, destination, detail)
    assert result is not None
    if cleanup_detail:
        return CheckOutcome(
            EXIT_FAILURE,
            "cleanup_failed",
            destination,
            cleanup_detail,
            result.stdout,
            result.stderr,
        )

    status, reason, _document = verify_report(
        destination,
        run_id=arguments.run_id,
        process_exit=result.returncode,
        kind=arguments.check,
    )
    if status == "succeeded":
        return CheckOutcome(EXIT_SUCCESS, "", destination, "", result.stdout, result.stderr)
    if status == "failed":
        return CheckOutcome(EXIT_FAILURE, reason, destination, "", result.stdout, result.stderr)
    elapsed = time.monotonic() - started
    return CheckOutcome(
        EXIT_FAILURE,
        reason,
        destination,
        f"process exit {result.returncode}, elapsed {elapsed:.1f}s",
        result.stdout,
        result.stderr,
    )


def _request_stop(signum: int, _frame: object) -> None:
    raise StopRequested(signum)


def _print_result(outcome: CheckOutcome) -> None:
    if outcome.stdout:
        print(_bounded(outcome.stdout), end="" if outcome.stdout.endswith("\n") else "\n")
    if outcome.stderr:
        ending = "" if outcome.stderr.endswith("\n") else "\n"
        print(_bounded(outcome.stderr), file=sys.stderr, end=ending)
    if outcome.exit_code == EXIT_SUCCESS:
        print(f"status: succeeded\nreport: {outcome.report_path}")
    else:
        detail = f": {outcome.detail}" if outcome.detail else ""
        print(f"error: {outcome.reason}{detail}", file=sys.stderr)
        print(f"report: {outcome.report_path}", file=sys.stderr)


def main(
    argv: Sequence[str] | None = None,
    *,
    runner: Callable[[Arguments], CheckOutcome] | None = None,
) -> int:
    try:
        arguments = parse_arguments(argv)
    except UsageError as error:
        print(f"error: {error}", file=sys.stderr)
        return EXIT_USAGE
    execute_check = execute if runner is None else runner
    previous: dict[int, object] = {}
    for signum in (signal.SIGTERM,):
        previous[signum] = signal.getsignal(signum)
        signal.signal(signum, _request_stop)
    try:
        outcome = execute_check(arguments)
    except UsageError as error:
        print(f"error: {error}", file=sys.stderr)
        return EXIT_USAGE
    except StopRequested as stop:
        print(f"error: interrupted: {stop}", file=sys.stderr)
        return SIGNAL_EXIT_CODES.get(stop.signum, EXIT_FAILURE)
    except KeyboardInterrupt:
        print("error: interrupted: SIGINT", file=sys.stderr)
        return 130
    except OSError as error:
        print(f"error: host_io_error: {type(error).__name__}: {error}", file=sys.stderr)
        return EXIT_FAILURE
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
    _print_result(outcome)
    return outcome.exit_code


if __name__ == "__main__":
    raise SystemExit(main())
