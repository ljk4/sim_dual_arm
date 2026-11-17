"""Build and verify the host-mounted ROS contract workspace.

The default build compiles every common and scenario-extension ROS interface
package into a single install tree, so switching scenarios never needs a
rebuild.  colcon's build, log and cache directories are created in a temporary
directory that is removed when the build finishes; only ``install/`` and
``contracts.json`` remain in the output root.  Consumers verify that marker, the
recorded package set, per-package source digests and generated module origins
before sourcing the install tree, so a changed IDL cannot silently keep old
generated code.
"""

from __future__ import annotations

import argparse
import fcntl
import json
import os
import shutil
import subprocess
import sys
import tempfile
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from pathlib import Path

from sdk_marker import (
    BUILD_COMMAND,  # noqa: F401 - re-exported for the existing tool/test API
    MARKER_NAME,
    print_status,
    verify_workspace,
    write_marker,
)
from sdk_packages import (
    PackageSelection,
    PackageSelectionError,
    SdkCheckError,
    SdkError,
    binding_selection,
    explicit_selection,
    full_selection,
    package_record,
    package_set_sha256,
    shared_root_path,
    validate_build_root,
)

ROOT = Path(__file__).resolve().parents[4]
DEFAULT_BUILD_ROOT = Path("build/ros-dev")
ROS_LOG_DIRECTORY = "/tmp/ros-dev-logs"


# Public aliases keep the existing tool API while the shared marker module owns
# the exact exception types.
DevWorkspaceError = SdkError
ContractCheckError = SdkCheckError


class BuildBusyError(DevWorkspaceError):
    """Another builder already owns this output root."""


def resolve_selection(
    project_root: Path,
    *,
    shared_root: Path | None = None,
    scenario: str | None = None,
    robot: str | None = None,
    task: str | None = None,
    packages: list[str] | None = None,
) -> PackageSelection:
    shared = shared_root_path(project_root, shared_root)
    if robot is not None or task is not None:
        if robot is None or task is None:
            raise DevWorkspaceError("--robot and --task must be given together")
        if scenario is not None:
            raise DevWorkspaceError("--scenario and --robot/--task are mutually exclusive")
        scenario = f"{robot}/{task}"
    if scenario is not None and packages:
        raise DevWorkspaceError("--scenario and --package are mutually exclusive")
    try:
        if scenario is not None:
            return binding_selection(shared, scenario)
        if packages:
            return explicit_selection(Path(project_root), shared, packages)
        return full_selection(shared)
    except PackageSelectionError as error:
        raise DevWorkspaceError(str(error)) from error


@contextmanager
def build_lock(build_root: Path) -> Iterator[None]:
    """Hold an exclusive non-blocking lock on the output root directory."""
    root = Path(build_root).resolve()
    root.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(root, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise BuildBusyError(
                f"another builder is already writing {root}; wait for it or use a "
                "different --build-root"
            ) from error
        yield
    finally:
        os.close(descriptor)


def _remove_known_output(path: Path) -> None:
    if not path.exists() and not path.is_symlink():
        return
    if path.is_symlink() or path.is_file():
        path.unlink()
        return
    shutil.rmtree(path)


def _clean_outputs(root: Path) -> None:
    # ``build``/``log``/``colcon-home``/``workspace.json`` are removed in case an
    # older build left its intermediates in the output root.
    for name in ("build", "install", "log", "colcon-home", "workspace.json"):
        _remove_known_output(root / name)


def colcon_command(
    project_root: Path,
    build_root: Path,
    selection: PackageSelection,
    *,
    build_base: Path,
    log_base: Path,
) -> list[str]:
    """Use the existing interpreter-policy wrapper, never a host colcon binary."""
    root = Path(build_root).resolve()
    return [
        sys.executable,
        str(Path(__file__).resolve().with_name("ros_workspace.py")),
        "--log-base",
        str(log_base),
        "build",
        "--base-paths",
        *(str(package.directory) for package in selection.packages),
        "--build-base",
        str(build_base),
        "--install-base",
        str(root / "install"),
        "--event-handlers",
        "console_direct+",
    ]


def run_colcon(project_root: Path, build_root: Path, selection: PackageSelection) -> int:
    project = Path(project_root).resolve()
    root = Path(build_root).resolve()
    with tempfile.TemporaryDirectory(prefix="sim-dual-arm-ros-") as scratch:
        scratch_path = Path(scratch)
        build_base = scratch_path / "build"
        log_base = scratch_path / "log"
        colcon_home = scratch_path / "colcon-home"
        colcon_home.mkdir()
        environment = os.environ.copy()
        environment["COLCON_HOME"] = str(colcon_home)
        environment["ROS_LOG_DIR"] = ROS_LOG_DIRECTORY
        environment["PYTHONUNBUFFERED"] = "1"
        completed = subprocess.run(
            colcon_command(project, root, selection, build_base=build_base, log_base=log_base),
            cwd=project,
            env=environment,
            check=False,
        )
        return completed.returncode


def _selection_records(selection: PackageSelection) -> dict[str, dict[str, object]]:
    return {package.name: package_record(package) for package in selection.packages}


def build_workspace(
    project_root: Path,
    build_root: Path,
    *,
    selection: PackageSelection | None = None,
    shared_root: Path | None = None,
    runner: Callable[[Path, Path, PackageSelection], int] | None = None,
) -> dict[str, object]:
    """Build the selected package set and publish the marker after verification."""
    project = Path(project_root).resolve()
    shared = shared_root_path(project, shared_root)
    chosen = selection or full_selection(shared)
    root = validate_build_root(project, build_root, shared)
    marker_path = root / MARKER_NAME
    build = run_colcon if runner is None else runner
    with build_lock(root):
        # Record the exact source snapshot this build consumes.
        records = _selection_records(chosen)
        marker_path.unlink(missing_ok=True)
        _clean_outputs(root)
        try:
            code = build(project, root, chosen)
            if code != 0:
                raise DevWorkspaceError(f"colcon build failed with exit code {code}")
            document = write_marker(root, chosen, records)
            return document
        except BaseException:
            marker_path.unlink(missing_ok=True)
            raise


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    def common(subparser: argparse.ArgumentParser) -> None:
        subparser.add_argument("--project-root", type=Path, default=ROOT)
        subparser.add_argument("--shared-root", type=Path)
        subparser.add_argument(
            "--scenario", help="scenario as <robot>/<task> selecting its binding package set"
        )
        subparser.add_argument("--robot", help="robot id below <shared-root>/scenarios")
        subparser.add_argument("--task", help="task id below <shared-root>/scenarios/<robot>")
        subparser.add_argument(
            "--package",
            action="append",
            default=[],
            help="explicit common package name or extension directory (repeatable)",
        )

    digest = subparsers.add_parser("digest", help="print the selected package-set digest")
    common(digest)

    build = subparsers.add_parser("build", help="build the selected packages")
    common(build)
    build.add_argument("--build-root", type=Path, required=True)

    check = subparsers.add_parser("check", help="verify marker, package set, digests, modules")
    common(check)
    check.add_argument("--build-root", type=Path, required=True)

    status = subparsers.add_parser("status", help="verify and print generated module origins")
    common(status)
    status.add_argument("--build-root", type=Path, required=True)
    return parser


def _digest_record(selection: PackageSelection) -> dict[str, object]:
    records = _selection_records(selection)
    return {
        "packages": records,
        "package_set_sha256": package_set_sha256(records),
    }


def main(argv: list[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    project = Path(arguments.project_root).resolve()
    try:
        explicit = (
            arguments.scenario is not None
            or arguments.robot is not None
            or arguments.task is not None
            or bool(arguments.package)
        )
        selection: PackageSelection | None
        if arguments.command in {"check", "status"} and not explicit:
            # A service-side check verifies whatever package set was recorded.
            selection = None
        elif arguments.command == "build" and not arguments.package:
            # One build covers every scenario; --scenario is accepted on build
            # for compatibility but never narrows the generated SDK.
            selection = resolve_selection(project, shared_root=arguments.shared_root)
        else:
            selection = resolve_selection(
                project,
                shared_root=arguments.shared_root,
                scenario=arguments.scenario,
                robot=arguments.robot,
                task=arguments.task,
                packages=arguments.package,
            )
        shared = (
            selection.shared_root
            if selection is not None
            else shared_root_path(project, arguments.shared_root)
        )
        if arguments.command == "digest":
            assert selection is not None
            print(json.dumps(_digest_record(selection), sort_keys=True))
        elif arguments.command == "build":
            assert selection is not None
            document = build_workspace(
                project, arguments.build_root, selection=selection, shared_root=shared
            )
            print(
                f"built {len(selection.packages)} packages "
                f"({', '.join(selection.names)}): {document['package_set_sha256']} -> "
                f"{Path(arguments.build_root).resolve()}"
            )
        elif arguments.command == "check":
            document = verify_workspace(
                project, arguments.build_root, selection=selection, shared_root=shared
            )
            print(
                f"contracts ok: {', '.join(sorted(document['packages']))} "
                f"({document['package_set_sha256']})"
            )
        else:
            document = verify_workspace(
                project, arguments.build_root, selection=selection, shared_root=shared
            )
            print_status(document, arguments.build_root)
    except (SdkError, PackageSelectionError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
