"""Build and verify the host-mounted ROS contract workspace.

The core SDK contains only ``robot_api_msgs``.  A scenario build adds exactly the
packages selected by its binding (common packages and extension packages).  A
successful colcon build writes an atomic completion marker next to the generated
``build``/``install``/``log`` directories.  Consumers verify that marker, the
recorded package set, per-package source digests and generated module origins
before sourcing the install tree, so a core build cannot masquerade as a full
scenario SDK and a changed IDL cannot silently keep old generated code.
"""

from __future__ import annotations

import argparse
import fcntl
import json
import os
import shutil
import subprocess
import sys
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from datetime import UTC, datetime
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
    core_selection,
    explicit_selection,
    package_record,
    package_set_sha256,
    shared_root_path,
    validate_build_root,
    write_json_atomic,
)

ROOT = Path(__file__).resolve().parents[4]
DEFAULT_BUILD_ROOT = Path(".build/ros-dev")
OWNERSHIP_NAME = "workspace.json"
OWNERSHIP_KIND = "sim-dual-arm-dev-ros-workspace"
OWNERSHIP_SCHEMA_VERSION = 1
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
    packages: list[str] | None = None,
) -> PackageSelection:
    shared = shared_root_path(project_root, shared_root)
    if scenario is not None and packages:
        raise DevWorkspaceError("--scenario and --package are mutually exclusive")
    try:
        if scenario is not None:
            return binding_selection(shared, scenario)
        if packages:
            return explicit_selection(Path(project_root), shared, packages)
        return core_selection(shared)
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
    for name in ("build", "install", "log"):
        _remove_known_output(root / name)


def _prepare_cache_directories(root: Path) -> None:
    (root / "colcon-home").mkdir(parents=True, exist_ok=True)


def colcon_command(
    project_root: Path, build_root: Path, selection: PackageSelection
) -> list[str]:
    """Use the existing interpreter-policy wrapper, never a host colcon binary."""
    project = Path(project_root).resolve()
    root = Path(build_root).resolve()
    return [
        sys.executable,
        str(project / "shared/common/tools/scripts/ros_workspace.py"),
        "--log-base",
        str(root / "log"),
        "build",
        "--base-paths",
        *(str(package.directory) for package in selection.packages),
        "--build-base",
        str(root / "build"),
        "--install-base",
        str(root / "install"),
        "--event-handlers",
        "console_direct+",
    ]


def run_colcon(
    project_root: Path, build_root: Path, selection: PackageSelection
) -> int:
    project = Path(project_root).resolve()
    root = Path(build_root).resolve()
    environment = os.environ.copy()
    environment["COLCON_HOME"] = str(root / "colcon-home")
    environment["ROS_LOG_DIR"] = ROS_LOG_DIRECTORY
    environment["PYTHONUNBUFFERED"] = "1"
    completed = subprocess.run(
        colcon_command(project, root, selection),
        cwd=project,
        env=environment,
        check=False,
    )
    return completed.returncode


def _ownership_document() -> dict[str, object]:
    return {
        "schema_version": OWNERSHIP_SCHEMA_VERSION,
        "kind": OWNERSHIP_KIND,
        "created_at": datetime.now(UTC).isoformat(timespec="seconds"),
    }


def _read_ownership(root: Path) -> dict[str, object] | None:
    path = root / OWNERSHIP_NAME
    if not path.exists() and not path.is_symlink():
        return None
    if path.is_symlink():
        raise DevWorkspaceError(
            f"refusing to use build root with a symlinked ownership marker: {path}; "
            "choose a new/empty --build-root"
        )
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise DevWorkspaceError(
            f"build root ownership marker is unreadable: {path}: {error}; "
            "choose a new/empty --build-root or inspect it manually"
        ) from error
    if (
        not isinstance(document, dict)
        or document.get("schema_version") != OWNERSHIP_SCHEMA_VERSION
        or document.get("kind") != OWNERSHIP_KIND
    ):
        raise DevWorkspaceError(
            f"build root ownership marker is not recognised: {path}; "
            "choose a new/empty --build-root or inspect it manually"
        )
    return document


def _ensure_workspace_owned(root: Path) -> None:
    """Claim only a new/empty root; never delete unowned contents first."""
    if root.exists():
        if not root.is_dir():
            raise DevWorkspaceError(f"build root exists but is not a directory: {root}")
        entries = list(root.iterdir())
        if entries:
            if _read_ownership(root) is None:
                raise DevWorkspaceError(
                    f"build root is not empty and has no {OWNERSHIP_NAME} ownership "
                    f"marker: {root}; refusing to modify it. Choose a new/empty "
                    "--build-root or inspect/remove it manually. No files were deleted."
                )
            return
    else:
        root.mkdir(parents=True)
    write_json_atomic(root / OWNERSHIP_NAME, _ownership_document())


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
    chosen = selection or core_selection(shared)
    root = validate_build_root(project, build_root, shared)
    marker_path = root / MARKER_NAME
    build = run_colcon if runner is None else runner
    with build_lock(root):
        _ensure_workspace_owned(root)
        # Record the exact source snapshot this build consumes.
        records = _selection_records(chosen)
        marker_path.unlink(missing_ok=True)
        _clean_outputs(root)
        _prepare_cache_directories(root)
        try:
            code = build(project, root, chosen)
            if code != 0:
                raise DevWorkspaceError(
                    f"colcon build failed with exit code {code}; outputs kept in {root}"
                )
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
            "--scenario", help="select the package set declared by this scenario binding"
        )
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
        explicit = arguments.scenario is not None or bool(arguments.package)
        selection: PackageSelection | None
        if arguments.command in {"check", "status"} and not explicit:
            # A service-side check verifies whatever package set was recorded;
            # it must not silently reinterpret a full SDK as a core build.
            selection = None
        else:
            selection = resolve_selection(
                project,
                shared_root=arguments.shared_root,
                scenario=arguments.scenario,
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
