"""Package selection, dependency closure and source digests for the ROS SDK.

The core set is always ``robot_api_msgs``.  A scenario build adds only the
packages chosen by its binding: ``interfaces.common_ros`` and each
``interfaces.extension_ros`` directory.  README files are deliberately excluded
from digests so documentation edits do not invalidate generated code, while
IDL files and package build metadata do.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

CORE_PACKAGE = "robot_api_msgs"
COMMON_ROS_ROUTE = Path("common/interfaces/ros")
EXTENSION_FIELD_PREFIX = "interfaces.extension_ros["
SCENARIO_ID = re.compile(r"^[A-Za-z_][A-Za-z0-9_.:-]{0,127}$")
DEPENDENCY_TAGS = (
    "depend",
    "build_depend",
    "build_export_depend",
    "buildtool_depend",
    "exec_depend",
)
_ID_CONSTANT = re.compile(r"^\s*string\s+ID\s*=\s*(\S+)\s*$")


class SdkError(RuntimeError):
    """A build or verification failure with an actionable message."""


class SdkCheckError(SdkError):
    """The generated contract workspace is missing, stale or incomplete."""


class PackageSelectionError(SdkError):
    """A selection is missing, ambiguous or inconsistent with package metadata."""


@dataclass(frozen=True)
class PackageSource:
    name: str
    directory: Path
    role: str
    source: str
    dependencies: tuple[str, ...] = ()

    def summary(self) -> dict[str, object]:
        return {
            "name": self.name,
            "role": self.role,
            "source": self.source,
        }


@dataclass(frozen=True)
class PackageSelection:
    shared_root: Path
    packages: tuple[PackageSource, ...]
    kind: str
    scenario_id: str | None = None
    binding: Path | None = None
    binding_sha256: str | None = None

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(package.name for package in self.packages)

    def by_name(self) -> dict[str, PackageSource]:
        return {package.name: package for package in self.packages}

    def summary(self) -> dict[str, object]:
        record: dict[str, object] = {
            "kind": self.kind,
            "packages": [package.name for package in self.packages],
        }
        if self.scenario_id is not None:
            record["scenario_id"] = self.scenario_id
        if self.binding is not None:
            try:
                binding_path = self.binding.relative_to(self.shared_root).as_posix()
            except ValueError:
                binding_path = self.binding.as_posix()
            record["binding"] = {
                "path": binding_path,
                "sha256": self.binding_sha256,
            }
        return record


def _manifest(directory: Path) -> ET.Element:
    path = Path(directory) / "package.xml"
    try:
        return ET.parse(path).getroot()
    except (OSError, ET.ParseError) as error:
        raise PackageSelectionError(f"cannot read package manifest {path}: {error}") from error


def _package_name(root: ET.Element, manifest: Path) -> str:
    name = root.findtext("name")
    if not name or not name.strip():
        raise PackageSelectionError(f"package manifest has no name: {manifest}")
    return name.strip()


def _dependencies(root: ET.Element) -> tuple[str, ...]:
    dependencies = []
    for tag in DEPENDENCY_TAGS:
        dependencies.extend(
            element.text.strip()
            for element in root.findall(tag)
            if element.text and element.text.strip()
        )
    return tuple(dict.fromkeys(dependencies))


def teacher_capability(directory: Path) -> str | None:
    path = Path(directory) / "msg" / "TeacherCapability.msg"
    if not path.is_file():
        return None
    for line in path.read_text(encoding="utf-8").splitlines():
        match = _ID_CONSTANT.match(line.split("#", 1)[0])
        if match:
            return match.group(1)
    return None


def _relative_source(directory: Path, shared_root: Path) -> str:
    shared = shared_root.resolve()
    try:
        relative = directory.relative_to(shared)
    except ValueError as error:
        raise PackageSelectionError(
            f"package directory is outside the shared root and would not be mounted: "
            f"{directory} (shared {shared})"
        ) from error
    if ".." in relative.parts:
        raise PackageSelectionError(f"package source escapes the shared root: {relative}")
    return relative.as_posix()


def package_source(
    directory: Path,
    shared_root: Path,
    role: str,
    *,
    expected_name: str | None = None,
) -> PackageSource:
    directory = Path(directory)
    if not directory.is_dir():
        raise PackageSelectionError(f"package directory is missing: {directory}")
    resolved = directory.resolve()
    root = _manifest(directory)
    name = _package_name(root, Path(directory) / "package.xml")
    if expected_name is not None and name != expected_name:
        raise PackageSelectionError(
            f"package manifest name {name!r} differs from selected name {expected_name!r}: "
            f"{directory}"
        )
    if directory.name != name:
        raise PackageSelectionError(
            f"package directory {directory.name!r} does not match manifest name {name!r}"
        )
    return PackageSource(
        name, resolved, role, _relative_source(resolved, shared_root), _dependencies(root)
    )


def core_package_source(shared_root: Path) -> PackageSource:
    directory = Path(shared_root) / COMMON_ROS_ROUTE / CORE_PACKAGE
    return package_source(directory, shared_root, "core", expected_name=CORE_PACKAGE)


def core_selection(shared_root: Path) -> PackageSelection:
    return PackageSelection(
        Path(shared_root).resolve(), (core_package_source(shared_root),), "core"
    )


def _deduplicate(packages: list[PackageSource]) -> tuple[PackageSource, ...]:
    seen: dict[str, PackageSource] = {}
    for package in packages:
        existing = seen.get(package.name)
        if existing is not None:
            raise PackageSelectionError(
                f"duplicate package name {package.name!r}: {existing.directory} and "
                f"{package.directory}"
            )
        seen[package.name] = package
    return tuple(packages)


def _workspace_package_directory(name: str, shared_root: Path) -> Path | None:
    common = Path(shared_root) / COMMON_ROS_ROUTE / name
    if common.is_dir():
        return common
    for candidate in sorted(Path(shared_root).glob(f"scenarios/*/extensions/ros/{name}")):
        if candidate.is_dir():
            return candidate
    return None


def check_dependency_closure(packages: tuple[PackageSource, ...], shared_root: Path) -> None:
    selected = {package.name for package in packages}
    for package in packages:
        for dependency in package.dependencies:
            if dependency in selected:
                continue
            workspace = _workspace_package_directory(dependency, shared_root)
            if workspace is not None:
                raise PackageSelectionError(
                    f"package {package.name!r} depends on workspace package {dependency!r} "
                    f"({workspace}) but the selected set does not include it"
                )


def binding_selection(shared_root: Path, scenario: str) -> PackageSelection:
    if SCENARIO_ID.fullmatch(scenario) is None or ".." in scenario:
        raise PackageSelectionError(
            "scenario id must contain 1..128 safe characters and no path traversal"
        )
    shared = Path(shared_root).resolve()
    binding = shared / "scenarios" / scenario / "binding.yaml"
    if not binding.is_file():
        raise PackageSelectionError(f"scenario {scenario!r} has no binding file: {binding}")
    try:
        from robot_contracts.binding import BindingError, load_binding
    except ImportError as error:
        raise PackageSelectionError(
            "scenario SDK selection requires robot_contracts on PYTHONPATH"
        ) from error
    try:
        resolved = load_binding(binding, shared_root=shared)
    except BindingError as error:
        raise PackageSelectionError(str(error)) from error

    packages: list[PackageSource] = [core_package_source(shared)]
    for name in resolved.binding.interfaces.common_ros:
        if name == CORE_PACKAGE:
            continue
        directory = shared / COMMON_ROS_ROUTE / name
        packages.append(package_source(directory, shared, "common", expected_name=name))
    for index, _declared in enumerate(resolved.binding.interfaces.extension_ros):
        field = f"{EXTENSION_FIELD_PREFIX}{index}]"
        directory = resolved.references[field].path
        packages.append(package_source(directory, shared, "extension"))
    selection_packages = _deduplicate(packages)
    check_dependency_closure(selection_packages, shared)
    teacher_api = resolved.binding.interfaces.teacher_api
    if teacher_api:
        capabilities = [
            capability
            for package in selection_packages
            if (capability := teacher_capability(package.directory)) is not None
        ]
        if teacher_api not in capabilities:
            raise PackageSelectionError(
                f"binding declares teacher_api {teacher_api!r}, but no selected extension "
                f"implements it (found {capabilities})"
            )
    return PackageSelection(
        shared,
        selection_packages,
        "binding",
        scenario_id=resolved.binding.scenario_id,
        binding=resolved.path,
        binding_sha256=resolved.sha256,
    )


def explicit_selection(
    project_root: Path, shared_root: Path, values: list[str]
) -> PackageSelection:
    """Test/operator selection: common names or package directories plus core."""
    shared = Path(shared_root).resolve()
    project = Path(project_root).resolve()
    packages: list[PackageSource] = [core_package_source(shared)]
    for value in values:
        candidate = shared / COMMON_ROS_ROUTE / value
        if not Path(value).is_absolute() and "/" not in value and candidate.is_dir():
            role = "core" if value == CORE_PACKAGE else "common"
            packages.append(package_source(candidate, shared, role, expected_name=value))
            continue
        path = Path(value)
        if not path.is_absolute():
            path = project / path
        if not path.is_dir():
            raise PackageSelectionError(
                f"selected package is neither a common name nor a directory: {value}"
            )
        packages.append(package_source(path, shared, "extension"))
    selection_packages = _deduplicate(packages)
    check_dependency_closure(selection_packages, shared)
    return PackageSelection(shared, selection_packages, "explicit")


def package_digest_details(directory: Path) -> tuple[str, int]:
    """Hash IDL and build metadata; README-only edits do not invalidate a build."""
    root = Path(directory).resolve()
    if not root.is_dir():
        raise PackageSelectionError(f"package source directory is missing: {root}")
    digest = hashlib.sha256()
    count = 0
    for path in sorted(root.rglob("*"), key=lambda item: item.relative_to(root).as_posix()):
        relative = path.relative_to(root).as_posix()
        if path.is_symlink():
            raise PackageSelectionError(f"package source must not contain symlinks: {relative}")
        if not path.is_file() or path.suffix.lower() == ".md":
            continue
        try:
            body = path.read_bytes()
        except OSError as error:
            raise PackageSelectionError(
                f"cannot read package source {relative}: {error}"
            ) from error
        digest.update(b"file\0")
        digest.update(relative.encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(len(body)).encode("ascii"))
        digest.update(b"\0")
        digest.update(body)
        count += 1
    if not count:
        raise PackageSelectionError(f"package source directory has no digestible files: {root}")
    return digest.hexdigest(), count


def package_record(package: PackageSource) -> dict[str, object]:
    digest, files = package_digest_details(package.directory)
    return {
        "role": package.role,
        "source": package.source,
        "files": files,
        "sha256": digest,
    }


def package_set_sha256(records: dict[str, dict[str, object]]) -> str:
    digest = hashlib.sha256()
    for name in sorted(records):
        record = records[name]
        digest.update(name.encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(record["source"]).encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(record["sha256"]).encode("ascii"))
        digest.update(b"\n")
    return digest.hexdigest()


def selection_from_marker(
    packages: dict[str, dict[str, object]], shared_root: Path
) -> tuple[PackageSource, ...]:
    """Reconstruct the recorded package set without trusting absolute paths."""
    shared = Path(shared_root).resolve()
    sources: list[PackageSource] = []
    for name, record in packages.items():
        source = record.get("source")
        role = record.get("role")
        if not isinstance(source, str) or Path(source).is_absolute() or ".." in Path(source).parts:
            raise PackageSelectionError(f"marker package {name!r} has an unsafe source: {source!r}")
        if not isinstance(role, str):
            raise PackageSelectionError(f"marker package {name!r} has no role")
        directory = (shared / source).resolve()
        if directory != shared and shared not in directory.parents:
            raise PackageSelectionError(
                f"marker package {name!r} escapes the shared root: {source}"
            )
        sources.append(package_source(directory, shared, role, expected_name=name))
    if not sources:
        raise PackageSelectionError("marker records no packages")
    return tuple(sources)


def expected_submodules(source: Path) -> tuple[str, ...]:
    return tuple(
        name for name in ("msg", "srv", "action") if (Path(source) / name).is_dir()
    )


def shared_root_path(project_root: Path, explicit: Path | None = None) -> Path:
    if explicit is not None:
        return Path(explicit).resolve()
    environment = os.environ.get("SHARED_ROOT")
    if environment:
        return Path(environment).resolve()
    return (Path(project_root).resolve() / "shared").resolve()


def validate_build_root(project_root: Path, build_root: Path, shared_root: Path) -> Path:
    """Resolve an explicit output root and reject roots unsafe to clean."""
    project = Path(project_root).resolve()
    root = Path(build_root)
    if not root.is_absolute():
        root = project / root
    root = root.resolve()
    if root == Path("/"):
        raise SdkError("refusing to use the filesystem root as the build root")
    if root == project or root in project.parents:
        raise SdkError(
            f"build root must not be the project root or one of its parents: {root}"
        )
    home = Path.home().resolve()
    if root == home or root in home.parents:
        raise SdkError(f"refusing to use the home directory as the build root: {root}")
    shared = Path(shared_root).resolve()
    if root == shared or shared in root.parents:
        raise SdkError(f"build root must not be inside the shared sources: {root}")
    if root.exists() and not root.is_dir():
        raise SdkError(f"build root exists but is not a directory: {root}")
    return root


def write_json_atomic(path: Path, document: dict[str, object]) -> None:
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
