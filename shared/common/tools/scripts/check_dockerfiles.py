"""Reject business source/task startup embedded in Dockerfiles and Compose.

Each implementation has exactly one Dockerfile, one docker-compose.yml, one
deployment.yaml requirement declaration and single-purpose scripts/*.sh
entrypoints (including scripts/test.sh).  Dockerfiles may
copy pinned dependency files or generated wheels, but not application source or
an entrypoint wrapper. All services inherit the image's idle command and are
entered with Compose exec.  Requirements that differ per implementation (GPU,
source mount, networks, entry commands and required mounts) live in the
implementation's deployment.yaml; the shared checks stay implementation-agnostic.
"""

from __future__ import annotations

import argparse
import json
import re
import shlex
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[4]
APPLICATION_ROOTS = {
    "simulation",
    "runtime",
    "learning",
    "hardware",
    "shared",
    "tests",
}
END_ROOTS = ("simulation", "hardware", "runtime", "learning")
CURRENT_ENDS = ("simulation", "runtime", "learning")
DEPENDENCY_SUFFIXES = {
    ".cfg",
    ".in",
    ".json",
    ".lock",
    ".sha256",
    ".toml",
    ".txt",
    ".whl",
    ".xml",
}
IDLE_COMMANDS = (["sleep", "infinity"], ["idle"])
COMPOSE_NAME = "docker-compose.yml"
DECLARATION_NAME = "deployment.yaml"
TEST_SCRIPT_NAME = "test.sh"
MOUNT_MODES = {"read-only": True, "writable": False}
SOURCE_SCOPES = {"implementation", "repository"}


def _copy_sources(body: str) -> list[str]:
    try:
        arguments = json.loads(body) if body.startswith("[") else shlex.split(body)
    except (ValueError, json.JSONDecodeError):
        return []
    return [
        argument
        for argument in arguments
        if not argument.startswith("--") and not argument.startswith("-")
    ][:-1]


def _copy_source_allowed(source: str) -> bool:
    path = Path(source)
    parts = path.parts
    if source in {".", "./", "/", "*", "./*"} or ".." in parts:
        return False
    suffix = path.suffix.lower()
    if suffix not in DEPENDENCY_SUFFIXES:
        return False
    if not parts or parts[0] not in APPLICATION_ROOTS:
        # Infrastructure dependency files are the only non-application COPY
        # sources allowed into an image.
        return True
    # Application-owned files are allowed only as dependency metadata under a
    # docker/ directory, never as source or an entrypoint copied into the image.
    return "docker" in parts and path.name != "Dockerfile"


def _interpreter_arguments(body: str) -> list[str]:
    body = body.strip()
    try:
        return json.loads(body) if body.startswith("[") else shlex.split(body)
    except (ValueError, json.JSONDecodeError):
        return []


def check_text(text: str) -> list[tuple[int, str]]:
    issues: list[tuple[int, str]] = []
    instruction = ""
    start = 0
    lines = 0
    for number, line in enumerate(text.splitlines(), 1):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        if not instruction:
            start = number
            lines = 0
        lines += 1
        instruction += line.strip().removesuffix("\\") + " "
        if line.rstrip().endswith("\\"):
            continue
        command, _, body = instruction.strip().partition(" ")
        command = command.upper()
        if command == "RUN":
            if "<<" in body:
                issues.append(
                    (start, "embedded heredoc; put the program in shared/common/tools/scripts")
                )
            if re.search(r"\b(?:python[\d.]*|node|bash|sh)\s+-[ce]\b", body):
                issues.append((start, "inline interpreter program; call a named script"))
            if re.search(r"(?:^|[;&])\s*(?:for|while|if|case)\s", body):
                issues.append((start, "shell control flow; put it in a named build script"))
            if lines > 30:
                issues.append((start, "large RUN program; put build logic in a named script"))
        if command == "VOLUME":
            issues.append((start, "do not declare anonymous volumes; use explicit Compose binds"))
        if command == "WORKDIR":
            issues.append((start, "working_dir belongs in Compose"))
        if command == "ENV":
            try:
                settings = shlex.split(body)
            except ValueError:
                settings = []
            if not settings or any(setting.split("=", 1)[0] != "PATH" for setting in settings):
                issues.append((start, "runtime ENV belongs in configs/container.env"))
        if command == "FROM" and "${" in body:
            issues.append(
                (start, "base image must be concrete; do not depend on another project image")
            )
        if command in {"COPY", "ADD"} and "--from" not in body:
            for source in _copy_sources(body):
                if not _copy_source_allowed(source):
                    issues.append(
                        (start, f"application source {source!r} must be a read-only runtime mount")
                    )
                    break
        if command in {"CMD", "ENTRYPOINT"}:
            arguments = _interpreter_arguments(body)
            if command == "CMD" and arguments not in IDLE_COMMANDS:
                issues.append((start, 'runtime CMD must be idle: ["sleep", "infinity"]'))
            if command == "ENTRYPOINT" and arguments:
                issues.append(
                    (start, "ENTRYPOINT must be empty; scripts run after container startup")
                )
        instruction = ""
    return issues


def _mounts(service: dict[str, Any]) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    for volume in service.get("volumes", []):
        if isinstance(volume, dict) and isinstance(volume.get("target"), str):
            result[volume["target"]] = volume
        elif isinstance(volume, str):
            parts = volume.split(":")
            if len(parts) in (2, 3):
                result[parts[1]] = {
                    "source": parts[0],
                    "target": parts[1],
                    "read_only": len(parts) == 3 and parts[2] == "ro",
                }
    return result


def _check_build(service_name: str, service: dict[str, Any], name: str) -> list[str]:
    issues: list[str] = []
    image = service.get("image")
    if not isinstance(image, str) or ":current" not in image:
        issues.append(
            f"{name}: service {service_name} image must default to a fixed :current tag"
        )
    build = service.get("build")
    if build == ".":
        return issues
    if not isinstance(build, dict):
        issues.append(f"{name}: service {service_name} must declare its Dockerfile via build")
        return issues
    if build.get("context") != "." or build.get("dockerfile") != "Dockerfile":
        issues.append(
            f"{name}: service {service_name} build must use the local Dockerfile"
        )
    if build.get("target"):
        issues.append(
            f"{name}: service {service_name} must not select a build target; "
            "each end has one runtime environment"
        )
    return issues


def _check_requirements(
    document: dict[str, Any],
    requirements: dict[str, Any],
    *,
    name: str,
    declaration_name: str,
) -> list[str]:
    services = document.get("services")
    if not isinstance(services, dict):
        return []
    service_name = requirements["service"]
    service = services.get(service_name)
    if not isinstance(service, dict):
        return [f"{declaration_name}: service {service_name!r} is not defined in {name}"]
    source_target = requirements["source"]["mount"]
    mounts = _mounts(service)
    expected: dict[str, bool] = {source_target: True}
    for target, mode in requirements["mounts"].items():
        expected[target] = MOUNT_MODES[mode]
    issues: list[str] = []
    for target, read_only in expected.items():
        mount = mounts.get(target)
        if mount is None or mount.get("read_only") is not read_only:
            field = "source.mount" if target == source_target else f"mounts.{target}"
            label = "read-only" if read_only else "writable"
            issues.append(
                f"{name}: service {service_name} must mount {target} {label} "
                f"as declared in {declaration_name}: {field}"
            )
    for target in sorted(set(mounts) - set(expected) - {"/shared"}):
        issues.append(
            f"{name}: service {service_name} has undeclared mount {target}; "
            f"add it to {declaration_name}: mounts or remove it from Compose"
        )
    if service.get("working_dir") != source_target:
        issues.append(
            f"{name}: service {service_name} must use {source_target} as its working "
            f"directory, as declared in {declaration_name}: source.mount"
        )
    networks = service.get("networks", [])
    expected_networks = requirements["networks"]
    if not isinstance(networks, list) or set(networks) != set(expected_networks):
        issues.append(
            f"{name}: service {service_name} must join networks "
            f"{sorted(expected_networks)} as declared in {declaration_name}: networks"
        )
    environment = service.get("environment", {})
    if not isinstance(environment, dict):
        environment = {}
    if requirements["gpu"]:
        if service.get("runtime") != "nvidia" or service.get("gpus") != "all":
            issues.append(
                f"{name}: service {service_name} must use runtime: nvidia and gpus: all "
                f"as declared in {declaration_name}: gpu"
            )
    elif service.get("gpus") is not None or {
        "NVIDIA_VISIBLE_DEVICES",
        "NVIDIA_DRIVER_CAPABILITIES",
    } & set(environment):
        issues.append(
            f"{name}: service {service_name} must not request GPUs, "
            f"as declared in {declaration_name}: gpu"
        )
    return issues


def check_compose_document(
    document: object,
    *,
    name: str,
    requirements: dict[str, Any] | None = None,
    declaration_name: str = DECLARATION_NAME,
) -> list[str]:
    issues: list[str] = []
    if not isinstance(document, dict):
        return [f"{name}: Compose document must be an object"]
    services = document.get("services")
    if not isinstance(services, dict):
        return [f"{name}: services must be an object"]
    if document.get("name") != "sim-dual-arm":
        issues.append(f"{name}: standalone services must share the sim-dual-arm project")
    if set(services) - {"simulation", "hardware", "runtime", "learning"}:
        unknown = sorted(set(services) - {"simulation", "hardware", "runtime", "learning"})
        issues.append(f"{name}: unexpected service name(s): {unknown}")
    for service_name, service in services.items():
        if not isinstance(service, dict):
            issues.append(f"{name}: service {service_name} must be an object")
            continue
        if "command" in service or "entrypoint" in service:
            issues.append(
                f"{name}: service {service_name} must inherit the image's idle CMD "
                "and empty ENTRYPOINT"
            )
        if service.get("env_file") != "../configs/container.env":
            issues.append(f"{name}: service {service_name} must load ../configs/container.env")
        if service.get("healthcheck"):
            issues.append(f"{name}: service {service_name} must not define a healthcheck")
        if service.get("privileged"):
            issues.append(f"{name}: service {service_name} must not be privileged")
        if service.get("network_mode") == "host":
            issues.append(f"{name}: service {service_name} must not use host networking")
        if any("${" in str(value) for value in service.values()):
            issues.append(f"{name}: service {service_name} must not interpolate host variables")
        issues.extend(_check_build(service_name, service, name))
        environment = service.get("environment", {})
        if not isinstance(environment, dict):
            environment = {}
        for key in environment:
            issues.append(
                f"{name}: service {service_name} must move {key} to configs/container.env"
            )
        for legacy in ("LEARNING_VENV", "DEV_IMAGE"):
            if legacy in environment:
                issues.append(f"{name}: service {service_name} must not define {legacy}")
        volumes = _mounts(service)
        for mount in volumes.values():
            source = mount.get("source")
            if not isinstance(source, str) or "${" in source:
                issues.append(
                    f"{name}: service {service_name} bind source must be a direct path"
                )
        if "/opt/host-learning-venv" in volumes:
            issues.append(f"{name}: service {service_name} must not mount a host virtualenv")
        shared_mount = volumes.get("/shared")
        if not shared_mount or shared_mount.get("read_only") is not True:
            issues.append(f"{name}: service {service_name} must mount /shared read-only")
        if service.get("init") is not True:
            issues.append(f"{name}: service {service_name} must enable init: true")
        if any("docker.sock" in str(volume.get("source", "")) for volume in volumes.values()):
            issues.append(f"{name}: service {service_name} must not mount the Docker socket")
    if requirements is not None:
        issues.extend(
            _check_requirements(
                document,
                requirements,
                name=name,
                declaration_name=declaration_name,
            )
        )
    return issues


def _declaration_issues(declaration: object, path: Path) -> list[str]:
    if not isinstance(declaration, dict):
        return [f"{path}: deployment requirements must be an object"]
    issues: list[str] = []
    service = declaration.get("service")
    if not isinstance(service, str) or not service:
        issues.append(f"{path}: service must name the Compose service")
    source = declaration.get("source")
    if not isinstance(source, dict):
        issues.append(f"{path}: source must declare mount and scope")
    else:
        mount = source.get("mount")
        if not isinstance(mount, str) or not mount.startswith("/"):
            issues.append(f"{path}: source.mount must be an absolute container path")
        if source.get("scope") not in SOURCE_SCOPES:
            issues.append(f"{path}: source.scope must be one of {sorted(SOURCE_SCOPES)}")
    mounts = declaration.get("mounts")
    if not isinstance(mounts, dict):
        issues.append(f"{path}: mounts must map container paths to read-only or writable")
    else:
        for target, mode in mounts.items():
            if not isinstance(target, str) or not target.startswith("/"):
                issues.append(f"{path}: mounts keys must be absolute container paths")
            if mode not in MOUNT_MODES:
                issues.append(f"{path}: mounts.{target} must be read-only or writable")
    networks = declaration.get("networks")
    if (
        not isinstance(networks, list)
        or not networks
        or any(not isinstance(network, str) or not network for network in networks)
    ):
        issues.append(f"{path}: networks must list the required Compose networks")
    if not isinstance(declaration.get("gpu"), bool):
        issues.append(f"{path}: gpu must be true or false")
    commands = declaration.get("commands")
    if (
        not isinstance(commands, list)
        or not commands
        or any(not isinstance(command, str) or not command for command in commands)
    ):
        issues.append(f"{path}: commands must list the required scripts/<command>.sh entrypoints")
    return issues


def _check_single_entry(
    project: Path, declarations: dict[Path, dict[str, Any]]
) -> list[str]:
    issues: list[str] = []
    for end in CURRENT_ENDS:
        dockerfiles = sorted((project / end).rglob("Dockerfile"))
        composes = sorted(
            path
            for path in (project / end).rglob(COMPOSE_NAME)
            if path.parent.name == "docker"
        )
        test_scripts = sorted((project / end).rglob(f"scripts/{TEST_SCRIPT_NAME}"))
        if len(dockerfiles) != 1:
            issues.append(f"{project / end}: expected exactly one Dockerfile")
        if len(composes) != 1:
            issues.append(f"{project / end}: expected exactly one docker/{COMPOSE_NAME}")
        if len(test_scripts) != 1:
            issues.append(f"{project / end}: expected exactly one scripts/{TEST_SCRIPT_NAME}")
        if len(dockerfiles) == 1 and len(composes) == 1:
            dockerfile = dockerfiles[0]
            compose = composes[0]
            if compose.parent != dockerfile.parent:
                issues.append(
                    f"{compose}: must sit next to the single {end} Dockerfile"
                )
        if len(composes) == 1 and len(test_scripts) == 1:
            expected_scripts_dir = composes[0].parent.parent / "scripts"
            if test_scripts[0] != expected_scripts_dir / TEST_SCRIPT_NAME:
                issues.append(f"{test_scripts[0]}: unexpected {TEST_SCRIPT_NAME} location")
                continue
            declaration_path = composes[0].parent / DECLARATION_NAME
            commands = declarations.get(composes[0], {}).get("commands", [])
            for command in commands:
                script_name = command if command.endswith(".sh") else f"{command}.sh"
                if not (expected_scripts_dir / script_name).is_file():
                    issues.append(
                        f"{expected_scripts_dir}: missing script {script_name!r}, "
                        f"as declared in {declaration_path}: commands"
                    )
    return issues


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=ROOT)
    arguments = parser.parse_args()
    project = arguments.project_root.resolve()
    found: list[str] = []
    dockerfiles = sorted(
        path
        for end in END_ROOTS
        for path in (project / end).rglob("Dockerfile")
        if path.is_file()
    )
    for path in dockerfiles:
        text = path.read_text(encoding="utf-8")
        for line, message in check_text(text):
            found.append(f"{path}:{line}: {message}")
        if not re.search(r"(?mi)^\s*ENTRYPOINT\s*\[\s*\]\s*$", text):
            found.append(
                f"{path}: must clear the inherited ENTRYPOINT with ENTRYPOINT []"
            )
        if not re.search(
            r'(?mi)^\s*CMD\s*\["sleep",\s*"infinity"\]\s*$', text
        ):
            found.append(f"{path}: image must default to sleep infinity")
    legacy_entrypoints = project / "shared/common/tools/docker/entrypoints"
    if legacy_entrypoints.exists():
        found.append(
            f"{legacy_entrypoints}: legacy entrypoint wrappers must be removed; "
            "source shared/common/tools/docker/setup.sh instead"
        )
    for end in END_ROOTS:
        for path in (project / end).rglob("compose.yaml"):
            found.append(f"{path}: legacy Compose name; use docker/{COMPOSE_NAME}")
    for root_compose in ("compose.yaml", COMPOSE_NAME):
        if (project / root_compose).exists():
            found.append(f"{project / root_compose}: root Compose wrapper is unnecessary")
    environments = sorted(
        path
        for end in END_ROOTS
        for path in (project / end).rglob(COMPOSE_NAME)
        if path.parent.name == "docker"
    )
    declarations: dict[Path, dict[str, Any]] = {}
    for path in environments:
        if not path.is_file():
            found.append(f"{path}: missing")
            continue
        try:
            document = yaml.safe_load(path.read_text(encoding="utf-8"))
        except yaml.YAMLError as error:
            found.append(f"{path}: invalid YAML: {error}")
            continue
        declaration_path = path.parent / DECLARATION_NAME
        declaration: dict[str, Any] | None = None
        if not declaration_path.is_file():
            found.append(f"{declaration_path}: missing deployment requirements")
        else:
            try:
                loaded = yaml.safe_load(declaration_path.read_text(encoding="utf-8"))
            except yaml.YAMLError as error:
                found.append(f"{declaration_path}: invalid YAML: {error}")
            else:
                declaration_issues = _declaration_issues(loaded, declaration_path)
                if declaration_issues:
                    found.extend(declaration_issues)
                else:
                    declaration = loaded
                    declarations[path] = loaded
        for message in check_compose_document(
            document,
            name=str(path),
            requirements=declaration,
            declaration_name=str(declaration_path),
        ):
            found.append(message)
        if not isinstance(document, dict) or not isinstance(document.get("services"), dict):
            continue
        implementation = path.parent.parent.relative_to(project).as_posix()
        if declaration is not None:
            service = document["services"].get(declaration["service"])
            if isinstance(service, dict):
                source_target = declaration["source"]["mount"]
                scope = declaration["source"]["scope"]
                source = _mounts(service).get(source_target, {}).get("source", "")
                expected_root = (
                    project if scope == "repository" else project / implementation
                )
                if not isinstance(source, str) or (path.parent / source).resolve() != (
                    expected_root.resolve()
                ):
                    allowed = (
                        "the repository root"
                        if scope == "repository"
                        else implementation
                    )
                    found.append(
                        f"{declaration_path}: source.mount {source_target} "
                        f"must bind only {allowed}"
                    )
        for service_name, service in document["services"].items():
            if not isinstance(service, dict):
                continue
            env_path = path.parent.parent / "configs/container.env"
            if not env_path.is_file():
                found.append(f"{env_path}: missing container runtime settings")
            else:
                settings = dict(
                    line.split("=", 1)
                    for line in env_path.read_text(encoding="utf-8").splitlines()
                    if line and not line.lstrip().startswith("#") and "=" in line
                )
                expected = {"PROJECT_CONTAINER": "1"}
                if declaration is not None and declaration["service"] == service_name:
                    expected["PROJECT_ROOT"] = declaration["source"]["mount"]
                for key, value in expected.items():
                    if settings.get(key) != value:
                        found.append(f"{env_path}: {key} must be {value}")
    found.extend(_check_single_entry(project, declarations))
    if found:
        print("\n".join(found))
        return 1
    print(
        f"Dockerfile/Compose boundaries passed: {len(dockerfiles)} Dockerfiles, "
        f"{len(environments)} idle environment files"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
