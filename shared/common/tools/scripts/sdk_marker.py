"""Completion-marker creation and verification for the selected ROS SDK.

The marker records the exact package set, each package's source provenance and
non-README digest, the generated module origin and the build environment.  A
package-set mismatch, changed IDL/build metadata, missing/duplicated generated
modules or an old marker schema is rejected with a rebuild instruction.
"""

from __future__ import annotations

import json
import os
import platform
import sys
from datetime import UTC, datetime
from pathlib import Path

from sdk_packages import (
    PackageSelection,
    PackageSelectionError,
    PackageSource,
    SdkCheckError,
    SdkError,
    expected_submodules,
    package_digest_details,
    package_set_sha256,
    selection_from_marker,
    shared_root_path,
    validate_build_root,
    write_json_atomic,
)

MARKER_NAME = "contracts.json"
MARKER_SCHEMA_VERSION = 2
BUILD_COMMAND = "scripts/build_sdk.sh inside the runtime container"


def environment_record() -> dict[str, object]:
    distro = os.environ.get("ROS_DISTRO") or "unknown"
    if distro == "unknown":
        prefix: object = "unknown"
    else:
        prefix = os.environ.get("ROS_PREFIX") or f"/opt/ros/{distro}"
    return {
        "python": {
            "implementation": platform.python_implementation(),
            "version": platform.python_version(),
            "executable": str(Path(sys.executable).resolve()),
        },
        "ros": {"distro": distro, "prefix": prefix},
        "image": os.environ.get("PROJECT_IMAGE") or "unknown",
        "workspace_version": os.environ.get("PROJECT_WORKSPACE_VERSION") or "unknown",
    }


def generated_module_root(
    install_root: Path, name: str, source: Path, *, required: bool = True
) -> Path | None:
    """Find the single generated ament Python module below one install base."""
    install = Path(install_root)
    candidates = [
        path for path in sorted(install.glob(f"{name}/lib/python*/site-packages/{name}"))
        if path.is_dir()
    ]
    if len(candidates) != 1:
        if not required and not candidates:
            return None
        raise SdkError(
            f"expected one generated {name} module below {install}, found {len(candidates)}"
        )
    package = candidates[0]
    if not (package / "__init__.py").is_file():
        raise SdkError(f"generated module is incomplete: {name} ({package})")
    for submodule in expected_submodules(source):
        if not (package / submodule / "__init__.py").is_file():
            raise SdkError(
                f"generated module is incomplete: {name}.{submodule} ({package / submodule})"
            )
    if not (install / "setup.bash").is_file():
        raise SdkError(f"generated ROS install setup is missing: {install / 'setup.bash'}")
    return package


def build_marker_document(
    build_root: Path,
    selection: PackageSelection,
    records: dict[str, dict[str, object]],
) -> dict[str, object]:
    root = Path(build_root).resolve()
    packages: dict[str, dict[str, object]] = {}
    for package in selection.packages:
        module = generated_module_root(root / "install", package.name, package.directory)
        assert module is not None
        record = dict(records[package.name])
        record["module_root"] = module.relative_to(root).as_posix()
        packages[package.name] = record
    return {
        "schema_version": MARKER_SCHEMA_VERSION,
        "kind": "sim-dual-arm-ros-sdk",
        "selection": selection.summary(),
        "packages": packages,
        "package_set_sha256": package_set_sha256(records),
        "environment": environment_record(),
        "install": {"directory": "install", "setup": "install/setup.bash"},
        "finished_at": datetime.now(UTC).isoformat(timespec="seconds"),
    }


def write_marker(
    build_root: Path,
    selection: PackageSelection,
    records: dict[str, dict[str, object]],
) -> dict[str, object]:
    document = build_marker_document(build_root, selection, records)
    write_json_atomic(Path(build_root).resolve() / MARKER_NAME, document)
    return document


def _load_marker(marker_path: Path) -> dict[str, object]:
    try:
        document = json.loads(marker_path.read_text(encoding="utf-8"))
    except FileNotFoundError as error:
        raise SdkCheckError(
            f"completion marker is missing: {marker_path}; build with: {BUILD_COMMAND}"
        ) from error
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise SdkCheckError(
            f"completion marker is unreadable: {marker_path}: {error}; "
            f"rebuild with: {BUILD_COMMAND}"
        ) from error
    if not isinstance(document, dict):
        raise SdkCheckError(
            f"completion marker has the wrong shape: {marker_path}; "
            f"rebuild with: {BUILD_COMMAND}"
        )
    return document


def _relative_install_path(root: Path, value: object, label: str) -> Path:
    if not isinstance(value, str) or not value or Path(value).is_absolute():
        raise SdkCheckError(f"completion marker has an invalid {label}: {value!r}")
    relative = Path(value)
    if ".." in relative.parts:
        raise SdkCheckError(f"completion marker {label} escapes the build root: {value!r}")
    path = (root / relative).resolve()
    if path != root and root not in path.parents:
        raise SdkCheckError(f"completion marker {label} escapes the build root: {value!r}")
    return path


def _check_environment(document: dict[str, object], marker_path: Path) -> None:
    environment = document.get("environment")
    if not isinstance(environment, dict) or not isinstance(environment.get("python"), dict):
        raise SdkCheckError(
            f"completion marker is missing environment provenance: {marker_path}; "
            f"rebuild with: {BUILD_COMMAND}"
        )
    marker_python = environment["python"]
    if not isinstance(marker_python.get("version"), str):
        raise SdkCheckError(
            f"completion marker is missing the Python version: {marker_path}; "
            f"rebuild with: {BUILD_COMMAND}"
        )
    if marker_python.get("implementation") != platform.python_implementation():
        raise SdkCheckError(
            "generated ROS contracts were built for a different Python implementation; "
            f"rebuild with: {BUILD_COMMAND}"
        )
    marker_version = str(marker_python.get("version", "")).split(".")[:2]
    current_version = platform.python_version().split(".")[:2]
    if marker_version != current_version:
        raise SdkCheckError(
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
        raise SdkCheckError(
            f"completion marker is missing ROS provenance: {marker_path}; "
            f"rebuild with: {BUILD_COMMAND}"
        )
    for key in ("image", "workspace_version"):
        if not isinstance(environment.get(key), str):
            raise SdkCheckError(
                f"completion marker is missing {key} provenance: {marker_path}; "
                f"rebuild with: {BUILD_COMMAND}"
            )
    current_distro = os.environ.get("ROS_DISTRO")
    if current_distro and ros["distro"] != current_distro:
        raise SdkCheckError(
            f"generated ROS contracts were built for {ros['distro']}, current is "
            f"{current_distro}; rebuild with: {BUILD_COMMAND}"
        )


def _check_package_set(
    recorded: dict[str, dict[str, object]],
    expected: PackageSelection | None,
) -> None:
    """A full build may cover more packages than one scenario requires."""
    if expected is None:
        return
    missing = sorted(set(expected.names) - set(recorded))
    if missing:
        raise SdkCheckError(
            "completion marker is missing required packages: "
            f"{missing}; built {sorted(recorded)}; "
            f"rebuild with: {BUILD_COMMAND}"
        )
    expected_by_name = expected.by_name()
    for name in expected.names:
        record = recorded[name]
        package = expected_by_name[name]
        if record.get("role") != package.role:
            raise SdkCheckError(
                f"completion marker role for {name!r} is {record.get('role')!r}, "
                f"expected {package.role!r}; rebuild with: {BUILD_COMMAND}"
            )
        if record.get("source") != package.source:
            raise SdkCheckError(
                f"completion marker source for {name!r} is {record.get('source')!r}, "
                f"expected {package.source!r}; rebuild with: {BUILD_COMMAND}"
            )


def _check_modules(
    root: Path,
    record: dict[str, object],
    name: str,
    source: PackageSource,
) -> Path:
    module_root = _relative_install_path(root, record.get("module_root"), "module root")
    candidates = sorted((root / "install").glob(f"{name}/lib/python*/site-packages/{name}"))
    directories = [path.resolve() for path in candidates if path.is_dir()]
    if len(directories) != 1 or directories[0] != module_root:
        raise SdkCheckError(
            f"generated module origin for {name!r} is missing, duplicated or unexpected: "
            f"marker {module_root}, found {[str(path) for path in directories]}; "
            f"rebuild with: {BUILD_COMMAND}"
        )
    if not (module_root / "__init__.py").is_file():
        raise SdkCheckError(
            f"generated module is missing or incomplete: {name} ({module_root}); "
            f"rebuild with: {BUILD_COMMAND}"
        )
    for submodule in expected_submodules(source.directory):
        if not (module_root / submodule / "__init__.py").is_file():
            raise SdkCheckError(
                f"generated ROS module is missing or incomplete: {name}.{submodule} "
                f"({module_root}); rebuild with: {BUILD_COMMAND}"
            )
    return module_root


def verify_workspace(
    project_root: Path,
    build_root: Path,
    *,
    selection: PackageSelection | None = None,
    shared_root: Path | None = None,
) -> dict[str, object]:
    """Verify marker freshness, package set, digests and generated modules."""
    project = Path(project_root).resolve()
    shared = shared_root_path(project, shared_root)
    root = validate_build_root(project, build_root, shared)
    marker_path = root / MARKER_NAME
    document = _load_marker(marker_path)
    if document.get("schema_version") != MARKER_SCHEMA_VERSION:
        raise SdkCheckError(
            f"completion marker schema is not {MARKER_SCHEMA_VERSION}: {marker_path}; "
            f"rebuild with: {BUILD_COMMAND}"
        )
    if document.get("kind") != "sim-dual-arm-ros-sdk":
        raise SdkCheckError(
            f"completion marker kind is not a ROS SDK: {document.get('kind')!r}; "
            f"rebuild with: {BUILD_COMMAND}"
        )
    recorded = document.get("packages")
    if not isinstance(recorded, dict) or not recorded:
        raise SdkCheckError(
            f"completion marker records no packages: {marker_path}; "
            f"rebuild with: {BUILD_COMMAND}"
        )
    try:
        packages = selection_from_marker(recorded, shared)
    except PackageSelectionError as error:
        raise SdkCheckError(f"{error}; rebuild with: {BUILD_COMMAND}") from error
    if tuple(sorted(package.name for package in packages)) != tuple(sorted(recorded)):
        raise SdkCheckError(
            f"completion marker package names are inconsistent: {marker_path}; "
            f"rebuild with: {BUILD_COMMAND}"
        )
    _check_package_set(recorded, selection)
    by_name = {package.name: package for package in packages}
    digests: dict[str, dict[str, object]] = {}
    for name, record in recorded.items():
        package = by_name[name]
        try:
            digest, files = package_digest_details(package.directory)
        except PackageSelectionError as error:
            raise SdkCheckError(f"{error}; rebuild with: {BUILD_COMMAND}") from error
        if record.get("sha256") != digest or record.get("files") != files:
            raise SdkCheckError(
                f"generated ROS contracts are stale: package {name!r} sources changed since "
                f"the last build (source {digest}, marker {record.get('sha256')}); "
                f"rebuild with: {BUILD_COMMAND}"
            )
        digests[name] = {"source": record.get("source"), "sha256": digest}
    if document.get("package_set_sha256") != package_set_sha256(digests):
        raise SdkCheckError(
            f"completion marker package-set digest is inconsistent: {marker_path}; "
            f"rebuild with: {BUILD_COMMAND}"
        )
    _check_environment(document, marker_path)
    install = document.get("install")
    if not isinstance(install, dict):
        raise SdkCheckError(
            f"completion marker is missing install provenance: {marker_path}; "
            f"rebuild with: {BUILD_COMMAND}"
        )
    setup = _relative_install_path(root, install.get("setup"), "setup path")
    if not setup.is_file():
        raise SdkCheckError(
            f"generated ROS install setup is missing: {setup}; rebuild with: {BUILD_COMMAND}"
        )
    for name, record in recorded.items():
        assert isinstance(record, dict)
        _check_modules(root, record, name, by_name[name])
    return document


def print_status(document: dict[str, object], build_root: Path) -> None:
    install = document["install"]
    environment = document["environment"]
    packages = document["packages"]
    assert isinstance(install, dict)
    assert isinstance(environment, dict)
    assert isinstance(packages, dict)
    print("sdk_kind: sim-dual-arm-ros-sdk")
    print(f"package_set: {', '.join(sorted(packages))}")
    print(f"package_set_sha256: {document['package_set_sha256']}")
    print(f"workspace_version: {environment['workspace_version']}")
    print(f"generated_install: {Path(build_root).resolve() / str(install['directory'])}")
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
    for name in sorted(packages):
        record = packages[name]
        assert isinstance(record, dict)
        print(
            f"package {name}: role={record['role']} source={record['source']} "
            f"sha256={record['sha256']} module={record['module_root']}"
        )
