"""Forward a Compose command for the host-mounted mock development stack.

Only path preparation, run identity validation and Compose argument forwarding
live here.  Episode business logic stays in the runtime and backend modules.
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
from collections.abc import Callable, Sequence
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
COMPOSE_FILE = ROOT / "deploy/compose/mock.yaml"
DEFAULT_PROJECT = "sim-dual-arm-dev"
DEFAULT_IMAGE = "sim-dual-arm/dev-ros:stage2"
DEFAULT_BUILD_ROOT = ROOT / ".build/ros-dev"
DEFAULT_RAW_ROOT = ROOT / "var/data/raw"
DEFAULT_RUNS_ROOT = ROOT / "var/runs"
RUN_ID_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}")
START_COMMANDS = {"up", "start", "create", "run", "restart"}
VALUE_OPTIONS = {
    "-f",
    "--file",
    "--project-directory",
    "--project-name",
    "-p",
    "--profile",
    "--env-file",
    "--ansi",
    "--progress",
}


class DevComposeError(RuntimeError):
    """An actionable host-entry failure before Compose is invoked."""


def validate_run_id(value: str) -> str:
    if not isinstance(value, str) or not RUN_ID_PATTERN.fullmatch(value):
        raise DevComposeError(
            "run id must contain 1..128 ASCII letters, digits, '.', '_' or '-', "
            "and must start with a letter or digit"
        )
    if value in {".", ".."} or ".." in value:
        raise DevComposeError("run id must not be '.' or '..' or contain '..'")
    return value


def _absolute(value: Path | None, default: Path) -> Path:
    path = default if value is None else Path(value)
    if not path.is_absolute():
        path = ROOT / path
    return path.resolve()


def _validate_output_root(path: Path, label: str) -> Path:
    root = path.resolve()
    if root == Path("/"):
        raise DevComposeError(f"{label} must not be the filesystem root")
    if root == ROOT or root in ROOT.parents:
        raise DevComposeError(f"{label} must not be the repository root or one of its parents")
    home = Path.home().resolve()
    if root == home or root in home.parents:
        raise DevComposeError(f"{label} must not be the home directory or one of its parents")
    if root.exists() and not root.is_dir():
        raise DevComposeError(f"{label} exists but is not a directory: {root}")
    return root


def compose_subcommand(arguments: Sequence[str]) -> str | None:
    """Return the first Compose subcommand while skipping global option values."""
    index = 0
    while index < len(arguments):
        token = arguments[index]
        if token == "--":
            index += 1
            continue
        if token in VALUE_OPTIONS:
            index += 2
            continue
        if token.startswith("-"):
            index += 1
            continue
        return token
    return None


def compose_argv(
    *,
    docker: str,
    project: str,
    compose_file: Path,
    project_directory: Path,
    arguments: Sequence[str],
) -> list[str]:
    return [
        docker,
        "compose",
        "-p",
        project,
        "--project-directory",
        str(project_directory.resolve()),
        "-f",
        str(compose_file.resolve()),
        *arguments,
    ]


def _build_environment(
    *,
    project: str,
    image: str,
    build_root: Path,
    raw_root: Path,
    runs_root: Path,
    run_id: str,
) -> dict[str, str]:
    environment = os.environ.copy()
    environment.update(
        {
            "DEV_COMPOSE_PROJECT": project,
            "DEV_IMAGE": image,
            "DEV_REPO_ROOT": str(ROOT),
            "DEV_BUILD_ROOT": str(build_root),
            "DEV_RAW_ROOT": str(raw_root),
            "DEV_RUNS_ROOT": str(runs_root),
            "DEV_RUN_ID": run_id,
            "DEV_UID": str(os.getuid()),
            "DEV_GID": str(os.getgid()),
        }
    )
    return environment


def _check_start_conflicts(run_id: str, runs_root: Path, raw_root: Path) -> None:
    conflicts = [path for path in (runs_root / run_id, raw_root / run_id) if os.path.lexists(path)]
    if conflicts:
        rendered = ", ".join(str(path) for path in conflicts)
        raise DevComposeError(
            f"run id {run_id!r} already has products ({rendered}); choose a new "
            "--run-id instead of reusing or overwriting an older run"
        )


def _make_start_directories(build_root: Path, runs_root: Path, raw_root: Path) -> None:
    for path in (build_root, runs_root, raw_root):
        path.mkdir(parents=True, exist_ok=True)
    # The runtime owns the exclusive <runs-root>/<run-id> and raw run directory.


def prepare(
    arguments: argparse.Namespace,
) -> tuple[list[str], dict[str, str], str]:
    project = arguments.project
    image = arguments.image
    build_root = _validate_output_root(
        _absolute(arguments.build_root, DEFAULT_BUILD_ROOT), "build root"
    )
    raw_root = _validate_output_root(
        _absolute(arguments.raw_root, DEFAULT_RAW_ROOT), "raw output root"
    )
    runs_root = _validate_output_root(
        _absolute(arguments.runs_root, DEFAULT_RUNS_ROOT), "runs output root"
    )
    compose_arguments = list(arguments.compose_arguments)
    if compose_arguments and compose_arguments[0] == "build-contracts":
        if len(compose_arguments) != 1:
            raise DevComposeError("build-contracts does not accept further Compose arguments")
        if arguments.run_id is not None:
            raise DevComposeError("build-contracts does not use --run-id")
        build_root.mkdir(parents=True, exist_ok=True)
        environment = _build_environment(
            project=project,
            image=image,
            build_root=build_root,
            raw_root=raw_root,
            runs_root=runs_root,
            run_id="builder",
        )
        argv = compose_argv(
            docker=arguments.docker,
            project=project,
            compose_file=COMPOSE_FILE,
            project_directory=ROOT,
            arguments=["--profile", "build", "run", "--rm", "builder"],
        )
        return argv, environment, "build-contracts"

    if not compose_arguments:
        raise DevComposeError("a Compose command is required, for example: up -d")

    command = compose_subcommand(compose_arguments)
    supplied_run_id = arguments.run_id or os.environ.get("DEV_RUN_ID")
    if command in START_COMMANDS:
        if not supplied_run_id:
            raise DevComposeError(
                "starting the stack requires --run-id <new-id>; use a fresh id for each run"
            )
        run_id = validate_run_id(supplied_run_id)
        _check_start_conflicts(run_id, runs_root, raw_root)
        _make_start_directories(build_root, runs_root, raw_root)
    else:
        # ps/down/logs/config do not touch a run directory.  A placeholder keeps
        # Compose interpolation valid without creating or claiming a new run.
        run_id = validate_run_id(supplied_run_id) if supplied_run_id else "unused"
    environment = _build_environment(
        project=project,
        image=image,
        build_root=build_root,
        raw_root=raw_root,
        runs_root=runs_root,
        run_id=run_id,
    )
    argv = compose_argv(
        docker=arguments.docker,
        project=project,
        compose_file=COMPOSE_FILE,
        project_directory=ROOT,
        arguments=compose_arguments,
    )
    return argv, environment, command or ""


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", help="new run identity for start/run commands")
    parser.add_argument("--project", default=os.environ.get("DEV_COMPOSE_PROJECT", DEFAULT_PROJECT))
    parser.add_argument("--image", default=os.environ.get("DEV_IMAGE", DEFAULT_IMAGE))
    parser.add_argument("--build-root", type=Path, help="host ROS build output root")
    parser.add_argument("--raw-root", type=Path, help="host parent for MCAP run output")
    parser.add_argument("--runs-root", type=Path, help="host parent for run logs")
    parser.add_argument("--docker", default=os.environ.get("DEV_DOCKER", "docker"))
    parser.add_argument("compose_arguments", nargs=argparse.REMAINDER)
    return parser


def main(
    argv: list[str] | None = None,
    *,
    runner: Callable[..., subprocess.CompletedProcess] | None = None,
) -> int:
    parser = _parser()
    arguments = parser.parse_args(argv)
    try:
        command, environment, _ = prepare(arguments)
    except DevComposeError as error:
        parser.error(str(error))
    except OSError as error:
        parser.error(f"cannot prepare the development mount paths: {error}")
    execute = subprocess.run if runner is None else runner
    completed = execute(command, env=environment, check=False)
    return int(completed.returncode)


if __name__ == "__main__":
    raise SystemExit(main())
