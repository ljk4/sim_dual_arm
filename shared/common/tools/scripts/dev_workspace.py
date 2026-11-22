"""Build and verify the host-mounted ROS contract workspace."""

from __future__ import annotations

import argparse
import fcntl
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
MARKER_NAME = "contracts.json"
BUILD_COMMAND = "scripts/build_sdk.sh inside the runtime container"
ROS_LOG_DIRECTORY = "/tmp/ros-dev-logs"


class DevWorkspaceError(RuntimeError):
    """A ROS SDK build or contract verification failure."""


class ContractCheckError(DevWorkspaceError):
    """The generated contract workspace is missing, stale or incomplete."""


class BuildBusyError(DevWorkspaceError):
    """Another builder already owns this output root."""


@dataclass(frozen=True)
class PackageSource:
    name: str
    directory: Path
    role: str
    source: str


def shared_root_path(project_root: Path, explicit: Path | None = None) -> Path:
    if explicit is not None:
        return Path(explicit).resolve()
    env = os.environ.get("SHARED_ROOT")
    return Path(env).resolve() if env else (Path(project_root).resolve() / "shared").resolve()


def validate_build_root(project_root: Path, build_root: Path, shared_root: Path) -> Path:
    project = Path(project_root).resolve()
    root = Path(build_root)
    if not root.is_absolute():
        root = project / root
    root = root.resolve()
    home = Path.home().resolve()
    shared = Path(shared_root).resolve()
    if root == Path("/") or root == project or root in project.parents:
        raise DevWorkspaceError(f"unsafe build root: {root}")
    if root == home or root in home.parents:
        raise DevWorkspaceError(f"unsafe build root inside home: {root}")
    if root == shared or shared in root.parents:
        raise DevWorkspaceError(f"build root must not be inside shared sources: {root}")
    if root.exists() and not root.is_dir():
        raise DevWorkspaceError(f"build root exists but is not a directory: {root}")
    return root


def discover_packages(shared_root: Path) -> tuple[PackageSource, ...]:
    shared = Path(shared_root).resolve()
    common = shared / "common/interfaces/ros"
    if not (common / "robot_api_msgs/package.xml").is_file():
        raise DevWorkspaceError(f"core package robot_api_msgs missing under {common}")
    packages: list[PackageSource] = []
    for directory in sorted(common.iterdir()):
        if (directory / "package.xml").is_file():
            role = "core" if directory.name == "robot_api_msgs" else "common"
            packages.append(
                PackageSource(
                    directory.name, directory.resolve(), role,
                    directory.resolve().relative_to(shared).as_posix(),
                )
            )
    for pattern in ("scenarios/*/extensions/ros/*", "scenarios/*/*/extensions/ros/*"):
        for directory in sorted(shared.glob(pattern)):
            if (directory / "package.xml").is_file():
                packages.append(
                    PackageSource(
                        directory.name, directory.resolve(), "extension",
                        directory.resolve().relative_to(shared).as_posix(),
                    )
                )
    seen: dict[str, Path] = {}
    for pkg in packages:
        if pkg.name in seen:
            raise DevWorkspaceError(f"duplicate package {pkg.name!r}: {seen[pkg.name]} and {pkg.directory}")
        seen[pkg.name] = pkg.directory
    return tuple(packages)


def _package_digest(directory: Path) -> tuple[str, int]:
    root = directory.resolve()
    digest = hashlib.sha256()
    count = 0
    for path in sorted(root.rglob("*"), key=lambda item: item.relative_to(root).as_posix()):
        rel = path.relative_to(root).as_posix()
        if path.is_symlink():
            raise DevWorkspaceError(f"package source must not contain symlinks: {rel}")
        if not path.is_file() or "__pycache__" in path.parts or path.suffix.lower() == ".md":
            continue
        body = path.read_bytes()
        digest.update(f"file\0{rel}\0{len(body)}\0".encode("utf-8"))
        digest.update(body)
        count += 1
    if not count:
        raise DevWorkspaceError(f"package source directory has no digestible files: {root}")
    return digest.hexdigest(), count


def _package_records(packages: tuple[PackageSource, ...]) -> tuple[dict[str, dict[str, object]], str]:
    records: dict[str, dict[str, object]] = {}
    overall = hashlib.sha256()
    for pkg in sorted(packages, key=lambda item: item.name):
        digest, files = _package_digest(pkg.directory)
        records[pkg.name] = {
            "role": pkg.role,
            "source": pkg.source,
            "files": files,
            "sha256": digest,
        }
        overall.update(f"{pkg.name}\0{pkg.source}\0{digest}\n".encode("utf-8"))
    return records, overall.hexdigest()


@contextmanager
def build_lock(build_root: Path) -> Iterator[None]:
    root = Path(build_root).resolve()
    root.mkdir(parents=True, exist_ok=True)
    fd = os.open(root, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise BuildBusyError(f"another builder is already writing {root}") from error
        yield
    finally:
        os.close(fd)


def _clean_outputs(root: Path) -> None:
    for name in ("build", "install", "log", "colcon-home", "workspace.json"):
        target = root / name
        if target.is_symlink() or target.is_file():
            target.unlink()
        elif target.is_dir():
            shutil.rmtree(target)


def run_colcon(project_root: Path, build_root: Path, packages: tuple[PackageSource, ...]) -> int:
    project = Path(project_root).resolve()
    root = Path(build_root).resolve()
    with tempfile.TemporaryDirectory(prefix="sim-dual-arm-ros-") as scratch:
        scratch_path = Path(scratch)
        colcon_home = scratch_path / "colcon-home"
        colcon_home.mkdir()
        env = os.environ.copy()
        env["COLCON_HOME"] = str(colcon_home)
        env["ROS_LOG_DIR"] = ROS_LOG_DIRECTORY
        env["PYTHONUNBUFFERED"] = "1"
        command = [
            sys.executable,
            "-c",
            "from colcon_core.command import main; raise SystemExit(main())",
            "--log-base", str(scratch_path / "log"),
            "build",
            "--base-paths", *(str(pkg.directory) for pkg in packages),
            "--build-base", str(scratch_path / "build"),
            "--install-base", str(root / "install"),
            "--event-handlers", "console_direct+",
        ]
        return subprocess.run(command, cwd=project, env=env, check=False).returncode


def _check_modules(install_root: Path, pkg: PackageSource) -> str:
    candidates = [
        p.resolve() for p in sorted(install_root.glob(f"{pkg.name}/lib/python*/site-packages/{pkg.name}"))
        if p.is_dir()
    ]
    if len(candidates) != 1:
        raise ContractCheckError(
            f"expected one generated {pkg.name} module under {install_root}, found {len(candidates)}; "
            f"rebuild with: {BUILD_COMMAND}"
        )
    mod = candidates[0]
    if not (mod / "__init__.py").is_file():
        raise ContractCheckError(f"generated module incomplete: {pkg.name}; rebuild with: {BUILD_COMMAND}")
    for sub in ("msg", "srv", "action"):
        if (pkg.directory / sub).is_dir() and not (mod / sub / "__init__.py").is_file():
            raise ContractCheckError(
                f"generated ROS module incomplete: {pkg.name}.{sub}; rebuild with: {BUILD_COMMAND}"
            )
    return mod.relative_to(install_root.parent).as_posix()


def build_workspace(
    project_root: Path,
    build_root: Path,
    *,
    shared_root: Path | None = None,
    runner: Callable[[Path, Path, tuple[PackageSource, ...]], int] | None = None,
) -> dict[str, object]:
    project = Path(project_root).resolve()
    shared = shared_root_path(project, shared_root)
    packages = discover_packages(shared)
    root = validate_build_root(project, build_root, shared)
    marker_path = root / MARKER_NAME
    build = run_colcon if runner is None else runner
    with build_lock(root):
        records, set_digest = _package_records(packages)
        marker_path.unlink(missing_ok=True)
        _clean_outputs(root)
        try:
            code = build(project, root, packages)
            if code != 0:
                raise DevWorkspaceError(f"colcon build failed with exit code {code}")
            install_root = root / "install"
            if not (install_root / "setup.bash").is_file():
                raise DevWorkspaceError(f"generated ROS install setup missing: {install_root / 'setup.bash'}")
            for pkg in packages:
                records[pkg.name]["module_root"] = _check_modules(install_root, pkg)
            document: dict[str, object] = {
                "kind": "sim-dual-arm-ros-sdk",
                "packages": records,
                "package_set_sha256": set_digest,
                "install": {"directory": "install", "setup": "install/setup.bash"},
            }
            temp = marker_path.with_name(f".{MARKER_NAME}.{os.getpid()}.tmp")
            try:
                temp.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")
                os.replace(temp, marker_path)
            finally:
                temp.unlink(missing_ok=True)
            return document
        except BaseException:
            marker_path.unlink(missing_ok=True)
            raise


def verify_workspace(
    project_root: Path,
    build_root: Path,
    *,
    shared_root: Path | None = None,
) -> dict[str, object]:
    project = Path(project_root).resolve()
    shared = shared_root_path(project, shared_root)
    root = validate_build_root(project, build_root, shared)
    marker_path = root / MARKER_NAME
    try:
        document = json.loads(marker_path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise ContractCheckError(f"contracts marker missing: {marker_path}; build with: {BUILD_COMMAND}") from error
    except (OSError, json.JSONDecodeError) as error:
        raise ContractCheckError(f"contracts marker unreadable: {marker_path}: {error}; rebuild with: {BUILD_COMMAND}") from error
    if not isinstance(document, dict) or document.get("kind") != "sim-dual-arm-ros-sdk":
        raise ContractCheckError(f"invalid contracts marker: {marker_path}; rebuild with: {BUILD_COMMAND}")
    packages = discover_packages(shared)
    _, current_digest = _package_records(packages)
    if document.get("package_set_sha256") != current_digest:
        raise ContractCheckError(
            f"generated ROS contracts are stale (sources changed); rebuild with: {BUILD_COMMAND}"
        )
    install_root = root / "install"
    if not (install_root / "setup.bash").is_file():
        raise ContractCheckError(f"generated ROS install setup missing; rebuild with: {BUILD_COMMAND}")
    for pkg in packages:
        _check_modules(install_root, pkg)
    return document


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    for name in ("build", "check"):
        sub = subparsers.add_parser(name)
        sub.add_argument("--project-root", type=Path, default=ROOT)
        sub.add_argument("--shared-root", type=Path)
        sub.add_argument("--build-root", type=Path, required=True)
        # Accepted for caller compatibility; SDK always covers every package.
        sub.add_argument("--scenario")
        sub.add_argument("--robot")
        sub.add_argument("--task")
    args = parser.parse_args(argv)
    try:
        if args.command == "build":
            document = build_workspace(args.project_root, args.build_root, shared_root=args.shared_root)
            names = ", ".join(sorted(document["packages"]))  # type: ignore[arg-type]
            print(f"built {len(document['packages'])} packages ({names}): {document['package_set_sha256']} -> {args.build_root.resolve()}")
        else:
            document = verify_workspace(args.project_root, args.build_root, shared_root=args.shared_root)
            names = ", ".join(sorted(document["packages"]))  # type: ignore[arg-type]
            print(f"contracts ok: {names} ({document['package_set_sha256']})")
    except (DevWorkspaceError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
