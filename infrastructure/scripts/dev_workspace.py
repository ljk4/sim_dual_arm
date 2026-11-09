"""Build and verify the host-mounted ROS contract workspace.

The ROS interfaces under ``contracts/ros`` are the only source.  A successful
colcon build writes an atomic completion marker next to the generated
``build``/``install``/``log`` directories.  Services must verify that marker and
the generated modules before sourcing the install tree, so a changed contract
cannot silently keep using an older interface.
"""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import importlib
import json
import os
import platform
import shutil
import subprocess
import sys
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from datetime import UTC, datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent.parent
CONTRACT_SOURCE = Path("contracts/ros")
DEFAULT_BUILD_ROOT = Path(".build/ros-dev")
MARKER_NAME = "contracts.json"
OWNERSHIP_NAME = "workspace.json"
OWNERSHIP_KIND = "sim-dual-arm-dev-ros-workspace"
MARKER_SCHEMA_VERSION = 1
OWNERSHIP_SCHEMA_VERSION = 1
ROS_LOG_DIRECTORY = "/tmp/ros-dev-logs"
BUILD_COMMAND = "run.sh build-ros-sdk inside the runtime container"
REQUIRED_MODULES = (
    "robot_api_msgs",
    "robot_api_msgs.msg",
    "robot_api_msgs.srv",
)
INSPECT_MODULES = (
    "robot_contracts",
    "robot_ros_support",
    "robot_run_logging",
    "robot_api_msgs",
    "dual_arm_runtime",
)


class DevWorkspaceError(RuntimeError):
    """A build or verification failure with an actionable message."""


class ContractCheckError(DevWorkspaceError):
    """The generated contract workspace is missing, stale or incomplete."""


class BuildBusyError(DevWorkspaceError):
    """Another builder already owns this output root."""


def contract_source_root(project_root: Path) -> Path:
    return Path(project_root).resolve() / CONTRACT_SOURCE


def contract_digest(project_root: Path) -> str:
    """Return a stable SHA-256 over contract paths and exact file bytes."""
    source_root = contract_source_root(project_root)
    return source_digest(source_root)


def source_digest(source_root: Path) -> str:
    digest, _ = source_digest_details(Path(source_root))
    return digest


def source_digest_details(source_root: Path) -> tuple[str, int]:
    root = Path(source_root).resolve()
    if not root.is_dir():
        raise DevWorkspaceError(f"contract source directory is missing: {root}")
    digest = hashlib.sha256()
    count = 0
    for path in sorted(root.rglob("*"), key=lambda item: item.relative_to(root).as_posix()):
        relative = path.relative_to(root).as_posix()
        if path.is_symlink():
            raise DevWorkspaceError(f"contract source must not contain symlinks: {relative}")
        if not path.is_file():
            continue
        try:
            body = path.read_bytes()
        except OSError as error:
            raise DevWorkspaceError(f"cannot read contract source {relative}: {error}") from error
        digest.update(b"file\0")
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(len(body)).encode("ascii"))
        digest.update(b"\0")
        digest.update(body)
        count += 1
    if not count:
        raise DevWorkspaceError(f"contract source directory is empty: {root}")
    return digest.hexdigest(), count


def validate_build_root(project_root: Path, build_root: Path) -> Path:
    """Resolve an explicit output root and reject roots unsafe to clean."""
    project = Path(project_root).resolve()
    root = Path(build_root)
    if not root.is_absolute():
        root = project / root
    root = root.resolve()
    if root == Path("/"):
        raise DevWorkspaceError("refusing to use the filesystem root as the build root")
    if root == project or root in project.parents:
        raise DevWorkspaceError(
            f"build root must not be the project root or one of its parents: {root}"
        )
    home = Path.home().resolve()
    if root == home or root in home.parents:
        raise DevWorkspaceError(f"refusing to use the home directory as the build root: {root}")
    source = contract_source_root(project)
    if root == source or source in root.parents:
        raise DevWorkspaceError(f"build root must not be inside the contract sources: {root}")
    if root.exists() and not root.is_dir():
        raise DevWorkspaceError(f"build root exists but is not a directory: {root}")
    return root


@contextmanager
def build_lock(build_root: Path) -> Iterator[None]:
    """Hold an exclusive non-blocking lock on the output root directory.

    Locking the directory itself works across builder containers sharing the
    same host bind mount and does not create a file in the root before its
    ownership has been established.
    """
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


def colcon_command(project_root: Path, build_root: Path) -> list[str]:
    """Use the existing interpreter-policy wrapper, never a host colcon binary."""
    project = Path(project_root).resolve()
    root = Path(build_root).resolve()
    return [
        sys.executable,
        str(project / "infrastructure/scripts/ros_workspace.py"),
        "--log-base",
        str(root / "log"),
        "build",
        "--base-paths",
        str(contract_source_root(project)),
        "--build-base",
        str(root / "build"),
        "--install-base",
        str(root / "install"),
        "--event-handlers",
        "console_direct+",
    ]


def run_colcon(project_root: Path, build_root: Path) -> int:
    project = Path(project_root).resolve()
    root = Path(build_root).resolve()
    environment = os.environ.copy()
    environment["COLCON_HOME"] = str(root / "colcon-home")
    environment["ROS_LOG_DIR"] = ROS_LOG_DIRECTORY
    environment["PYTHONUNBUFFERED"] = "1"
    completed = subprocess.run(
        colcon_command(project, root),
        cwd=project,
        env=environment,
        check=False,
    )
    return completed.returncode


def generated_module_root(install_root: Path) -> Path:
    """Find the generated ament Python module root below one install base."""
    install = Path(install_root)
    matches = sorted(install.glob("robot_api_msgs/lib/python*/site-packages/robot_api_msgs"))
    candidates = [path for path in matches if path.is_dir()]
    if len(candidates) != 1:
        raise DevWorkspaceError(
            f"expected one generated robot_api_msgs module below {install}, found {len(candidates)}"
        )
    package = candidates[0]
    for module in REQUIRED_MODULES:
        parts = module.split(".")[1:]
        directory = package.joinpath(*parts)
        if not (directory / "__init__.py").is_file():
            raise DevWorkspaceError(f"generated module is incomplete: {module} ({directory})")
    if not (install / "setup.bash").is_file():
        raise DevWorkspaceError(f"generated ROS install setup is missing: {install / 'setup.bash'}")
    return package


def _environment() -> dict[str, object]:
    distro = os.environ.get("ROS_DISTRO") or "unknown"
    if distro == "unknown":
        prefix: object = "unknown"
    else:
        prefix = os.environ.get("ROS_PREFIX") or f"/opt/ros/{distro}"
    return {
        "python": {
            "implementation": platform.python_implementation(),
            "version": platform.python_version(),
            "executable": sys.executable,
        },
        "ros": {"distro": distro, "prefix": prefix},
        "image": os.environ.get("PROJECT_IMAGE") or "unknown",
        "workspace_version": os.environ.get("PROJECT_WORKSPACE_VERSION") or "unknown",
    }


def _marker_document(build_root: Path, *, digest: str, files: int) -> dict[str, object]:
    root = Path(build_root).resolve()
    package = generated_module_root(root / "install")
    return {
        "schema_version": MARKER_SCHEMA_VERSION,
        "contracts": {
            "source": CONTRACT_SOURCE.as_posix(),
            "files": files,
            "sha256": digest,
        },
        "environment": _environment(),
        "install": {
            "directory": "install",
            "setup": "install/setup.bash",
            "module_root": package.relative_to(root).as_posix(),
        },
        "finished_at": datetime.now(UTC).isoformat(timespec="seconds"),
    }


def _write_json_atomic(path: Path, document: dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    body = (json.dumps(document, indent=2, sort_keys=True) + "\n").encode("utf-8")
    try:
        with temporary.open("wb") as stream:
            stream.write(body)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


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
    _write_json_atomic(root / OWNERSHIP_NAME, _ownership_document())


def build_workspace(
    project_root: Path,
    build_root: Path,
    *,
    runner: Callable[[Path, Path], int] | None = None,
) -> dict[str, object]:
    """Build contracts and publish the marker only after verifying artifacts."""
    project = Path(project_root).resolve()
    root = validate_build_root(project, build_root)
    source = contract_source_root(project)
    marker_path = root / MARKER_NAME
    build = run_colcon if runner is None else runner
    with build_lock(root):
        # Establish or validate ownership before deleting anything.
        _ensure_workspace_owned(root)
        # Record the source snapshot this build is about to consume.
        digest, files = source_digest_details(source)
        # A killed or failed build must never leave the previous marker usable.
        marker_path.unlink(missing_ok=True)
        _clean_outputs(root)
        _prepare_cache_directories(root)
        try:
            code = build(project, root)
            if code != 0:
                raise DevWorkspaceError(
                    f"colcon build failed with exit code {code}; outputs kept in {root}"
                )
            # Record the digest observed before the build.  If sources change
            # while colcon runs, verification compares against the new digest and
            # rejects the result instead of blessing a mixed tree.
            document = _marker_document(root, digest=digest, files=files)
            _write_json_atomic(marker_path, document)
            return document
        except BaseException:
            marker_path.unlink(missing_ok=True)
            raise


def _load_marker(marker_path: Path) -> dict[str, object]:
    try:
        document = json.loads(marker_path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise ContractCheckError(
            f"completion marker is missing: {marker_path}; build with: {BUILD_COMMAND}"
        ) from error
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ContractCheckError(
            f"completion marker is unreadable: {marker_path}: {error}; "
            f"rebuild with: {BUILD_COMMAND}"
        ) from error
    if not isinstance(document, dict):
        raise ContractCheckError(
            f"completion marker has the wrong shape: {marker_path}; rebuild with: {BUILD_COMMAND}"
        )
    return document


def _relative_install_path(root: Path, value: object, label: str) -> Path:
    if not isinstance(value, str) or not value or Path(value).is_absolute():
        raise ContractCheckError(f"completion marker has an invalid {label}: {value!r}")
    relative = Path(value)
    if ".." in relative.parts:
        raise ContractCheckError(f"completion marker {label} escapes the build root: {value!r}")
    path = (root / relative).resolve()
    if path != root and root not in path.parents:
        raise ContractCheckError(f"completion marker {label} escapes the build root: {value!r}")
    return path


def verify_workspace(project_root: Path, build_root: Path) -> dict[str, object]:
    """Check marker freshness and required generated modules without importing ROS."""
    project = Path(project_root).resolve()
    root = validate_build_root(project, build_root)
    marker_path = root / MARKER_NAME
    document = _load_marker(marker_path)
    if document.get("schema_version") != MARKER_SCHEMA_VERSION:
        raise ContractCheckError(
            f"completion marker schema is not {MARKER_SCHEMA_VERSION}: {marker_path}; "
            f"rebuild with: {BUILD_COMMAND}"
        )
    contracts = document.get("contracts")
    if not isinstance(contracts, dict):
        raise ContractCheckError(
            f"completion marker is missing contract provenance: {marker_path}; "
            f"rebuild with: {BUILD_COMMAND}"
        )
    if contracts.get("source") != CONTRACT_SOURCE.as_posix():
        raise ContractCheckError(
            f"completion marker names an unexpected contract source: {contracts.get('source')!r}; "
            f"rebuild with: {BUILD_COMMAND}"
        )
    digest, count = source_digest_details(contract_source_root(project))
    if contracts.get("sha256") != digest:
        raise ContractCheckError(
            "generated ROS contracts are stale: sources changed since the last build "
            f"(source {digest}, marker {contracts.get('sha256')}); stop the services, "
            f"then rebuild with: {BUILD_COMMAND}"
        )
    if contracts.get("files") != count:
        raise ContractCheckError(
            "generated ROS contracts are stale: source file count changed; "
            f"rebuild with: {BUILD_COMMAND}"
        )
    environment = document.get("environment")
    if not isinstance(environment, dict) or not isinstance(environment.get("python"), dict):
        raise ContractCheckError(
            f"completion marker is missing environment provenance: {marker_path}; "
            f"rebuild with: {BUILD_COMMAND}"
        )
    marker_python = environment["python"]
    if not isinstance(marker_python.get("version"), str):
        raise ContractCheckError(
            f"completion marker is missing the Python version: {marker_path}; "
            f"rebuild with: {BUILD_COMMAND}"
        )
    if marker_python.get("implementation") != platform.python_implementation():
        raise ContractCheckError(
            "generated ROS contracts were built for a different Python implementation; "
            f"rebuild with: {BUILD_COMMAND}"
        )
    marker_version = str(marker_python.get("version", "")).split(".")[:2]
    current_version = platform.python_version().split(".")[:2]
    if marker_version != current_version:
        raise ContractCheckError(
            "generated ROS contracts were built for Python "
            f"{'.'.join(marker_version) or 'unknown'}, current is "
            f"{'.'.join(current_version)}; rebuild with: {BUILD_COMMAND}"
        )
    ros = environment.get("ros")
    if (
        not isinstance(ros, dict)
        or not isinstance(ros.get("distro"), str)
        or not isinstance(ros.get("prefix"), str)
    ):
        raise ContractCheckError(
            f"completion marker is missing ROS provenance: {marker_path}; "
            f"rebuild with: {BUILD_COMMAND}"
        )
    for key in ("image", "workspace_version"):
        if not isinstance(environment.get(key), str):
            raise ContractCheckError(
                f"completion marker is missing {key} provenance: {marker_path}; "
                f"rebuild with: {BUILD_COMMAND}"
            )
    current_distro = os.environ.get("ROS_DISTRO")
    if current_distro and ros["distro"] != current_distro:
        raise ContractCheckError(
            f"generated ROS contracts were built for {ros['distro']}, current is "
            f"{current_distro}; rebuild with: {BUILD_COMMAND}"
        )
    install = document.get("install")
    if not isinstance(install, dict):
        raise ContractCheckError(
            f"completion marker is missing install provenance: {marker_path}; "
            f"rebuild with: {BUILD_COMMAND}"
        )
    setup = _relative_install_path(root, install.get("setup"), "setup path")
    module_root = _relative_install_path(root, install.get("module_root"), "module root")
    if not setup.is_file():
        raise ContractCheckError(
            f"generated ROS install setup is missing: {setup}; rebuild with: {BUILD_COMMAND}"
        )
    for module in REQUIRED_MODULES:
        parts = module.split(".")[1:]
        if not (module_root.joinpath(*parts) / "__init__.py").is_file():
            raise ContractCheckError(
                f"generated ROS module is missing or incomplete: {module} ({module_root}); "
                f"rebuild with: {BUILD_COMMAND}"
            )
    return document


def _import_paths(project_root: Path, build_root: Path) -> dict[str, str]:
    expected = {
        "robot_contracts": project_root / "contracts/python/src",
        "robot_ros_support": project_root / "libraries/ros_support/src",
        "robot_run_logging": project_root / "libraries/run_logging/src",
        "dual_arm_runtime": project_root / "runtime/ros2/ros/dual_arm_runtime",
        "robot_api_msgs": build_root / "install",
    }
    paths: dict[str, str] = {}
    for name in INSPECT_MODULES:
        try:
            module = importlib.import_module(name)
        except Exception as error:
            raise DevWorkspaceError(
                f"cannot import {name}: {type(error).__name__}: {error}"
            ) from error
        filename = getattr(module, "__file__", None)
        path = Path(filename).resolve() if filename else None
        if path is None:
            raise DevWorkspaceError(
                f"{name} has no source path; a stale installed copy may shadow it"
            )
        root = expected[name].resolve()
        if not path.is_relative_to(root):
            raise DevWorkspaceError(
                f"source shadowing detected for {name}: imported {path}, expected under {root}; "
                "check the mounted source paths and PYTHONPATH order"
            )
        paths[name] = str(path)
    return paths


def _print_status(project_root: Path, build_root: Path, document: dict[str, object]) -> None:
    contracts = document["contracts"]
    environment = document["environment"]
    install = document["install"]
    assert isinstance(contracts, dict)
    assert isinstance(environment, dict)
    assert isinstance(install, dict)
    print(f"contracts_source: {contract_source_root(project_root)}")
    print(
        "contracts_digest: "
        f"{contracts['sha256']} ({contracts['files']} files, schema {document['schema_version']})"
    )
    print(f"workspace_version: {environment['workspace_version']}")
    print(
        "generated_install: "
        f"{Path(build_root).resolve() / install['directory']} (module {install['module_root']})"
    )
    print(
        "ros: "
        f"distro={environment['ros']['distro']} prefix={environment['ros']['prefix']} "
        f"image={environment['image']}"
    )
    print(
        "python: "
        f"{environment['python']['implementation']} {environment['python']['version']} "
        f"({environment['python']['executable']})"
    )
    for name, path in _import_paths(project_root, build_root).items():
        print(f"import {name}: {path}")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    digest = subparsers.add_parser("digest", help="print the contract source SHA-256")
    digest.add_argument("--project-root", type=Path, default=ROOT)

    build = subparsers.add_parser("build", help="build contracts into an output root")
    build.add_argument("--project-root", type=Path, default=ROOT)
    build.add_argument("--build-root", type=Path, required=True)

    check = subparsers.add_parser("check", help="verify marker, digest and modules")
    check.add_argument("--project-root", type=Path, default=ROOT)
    check.add_argument("--build-root", type=Path, required=True)

    status = subparsers.add_parser("status", help="verify and print actual import paths")
    status.add_argument("--project-root", type=Path, default=ROOT)
    status.add_argument("--build-root", type=Path, required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    project = Path(arguments.project_root).resolve()
    try:
        if arguments.command == "digest":
            digest, count = source_digest_details(contract_source_root(project))
            print(json.dumps({"sha256": digest, "files": count}, sort_keys=True))
        elif arguments.command == "build":
            document = build_workspace(project, arguments.build_root)
            print(
                f"built {document['contracts']['files']} contract files: "
                f"{document['contracts']['sha256']} -> {Path(arguments.build_root).resolve()}"
            )
        elif arguments.command == "check":
            document = verify_workspace(project, arguments.build_root)
            print(
                f"contracts ok: {document['contracts']['sha256']} "
                f"({document['contracts']['files']} files)"
            )
        else:
            document = verify_workspace(project, arguments.build_root)
            _print_status(project, Path(arguments.build_root), document)
    except (DevWorkspaceError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
