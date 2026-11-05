"""Reject business source/task startup embedded in Dockerfiles and Compose.

Each implementation has exactly one Dockerfile, one docker-compose.yml and one
scripts/run.sh.  Dockerfiles may copy pinned dependency files or generated
wheels, but not application source or an entrypoint wrapper. All services
inherit the image's idle command and are entered with Compose exec.
"""

from __future__ import annotations

import argparse
import json
import re
import shlex
from pathlib import Path
from typing import Any

import yaml

ROOT = Path(__file__).resolve().parents[2]
APPLICATION_ROOTS = {
    "simulation",
    "runtime",
    "learning",
    "hardware",
    "libraries",
    "contracts",
    "profiles",
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
RUN_SCRIPT_NAME = "run.sh"


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
                    (start, "embedded heredoc; put the program in infrastructure/scripts")
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


def check_compose_document(document: object, *, name: str) -> list[str]:
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
        allowed_environment: set[str] = set()
        for key in set(environment) - allowed_environment:
            issues.append(
                f"{name}: service {service_name} must move {key} to configs/container.env"
            )
        for legacy in ("LEARNING_VENV", "DEV_IMAGE", "DEV_WORKSPACE_VERSION"):
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
        workspace_target = "/app" if service_name == "simulation" else "/workspace"
        workspace = volumes.get(workspace_target)
        if not workspace or workspace.get("read_only") is not True:
            issues.append(
                f"{name}: service {service_name} must mount {workspace_target} read-only"
            )
        if service.get("init") is not True:
            issues.append(f"{name}: service {service_name} must enable init: true")
        if service_name == "simulation":
            for target in ("/data",):
                mount = volumes.get(target)
                if not mount or mount.get("read_only") is True:
                    issues.append(f"{name}: service {service_name} must mount {target} writable")
            readonly = {
                "/app", "/contracts", "/profiles", "/deps/backend_core",
                "/deps/ros_support", "/deps/run_logging", "/config/setup.sh",
                "/config/check_ros_sdk.py", "/config/fastdds.xml",
                "/opt/ros-dev",
            }
            for target in sorted(readonly):
                if target not in volumes or volumes[target].get("read_only") is not True:
                    issues.append(f"{name}: simulation must mount {target} read-only")
            extra = set(volumes) - readonly - {"/data"}
            if extra:
                issues.append(
                    f"{name}: simulation must mount only its project, dependencies and data; "
                    f"extra={sorted(extra)}"
                )
            if service.get("working_dir") != "/app":
                issues.append(f"{name}: simulation must use /app as its working directory")
            if service.get("runtime") != "nvidia" or service.get("gpus") != "all":
                issues.append(
                    f"{name}: simulation must use runtime: nvidia and gpus: all "
                    "for NVIDIA rendering"
                )
        else:
            for target in ("/build", "/data", "/workspace/var", "/workspace/.build"):
                mount = volumes.get(target)
                if not mount or mount.get("read_only") is True:
                    issues.append(f"{name}: service {service_name} must mount {target} writable")
        if any("docker.sock" in str(volume.get("source", "")) for volume in volumes.values()):
            issues.append(f"{name}: service {service_name} must not mount the Docker socket")
        ros_mount = volumes.get("/opt/ros-dev")
        if service_name == "learning":
            if ros_mount:
                issues.append(f"{name}: learning must not mount /opt/ros-dev")
            for key in (
                "ROS_DOMAIN_ID",
                "RMW_IMPLEMENTATION",
                "ROS_LOG_DIR",
                "ROS_LOCALHOST_ONLY",
                "FASTRTPS_DEFAULT_PROFILES_FILE",
            ):
                if key in environment:
                    issues.append(f"{name}: learning must not define {key}")
            if "ros" in service.get("networks", []):
                issues.append(f"{name}: learning must not join the ROS network")
        else:
            if not ros_mount or ros_mount.get("read_only") is not True:
                issues.append(f"{name}: service {service_name} must mount /opt/ros-dev read-only")
        if service_name in {"runtime", "learning"}:
            policy = volumes.get("/opt/policy-sdk")
            if not policy or policy.get("read_only") is not True:
                issues.append(
                    f"{name}: service {service_name} must mount /opt/policy-sdk read-only"
                )
        if service_name == "simulation" and volumes.get("/opt/policy-sdk"):
            issues.append(f"{name}: simulation must not mount the Policy SDK")
        networks = service.get("networks", [])
        if not isinstance(networks, list):
            networks = []
        if service_name == "simulation" and "learning" in networks:
            issues.append(f"{name}: simulation must not join the learning network")
        if service_name == "runtime" and not {"ros", "learning"}.issubset(networks):
            issues.append(f"{name}: runtime must join the ros and learning networks")
        if service_name == "learning" and networks != ["learning"]:
            issues.append(f"{name}: learning must join only its own network")
        if service_name == "runtime" and (
            service.get("gpus") is not None
            or "NVIDIA_VISIBLE_DEVICES" in environment
            or "NVIDIA_DRIVER_CAPABILITIES" in environment
        ):
            issues.append(f"{name}: runtime must not require GPUs/NVIDIA")
    return issues


def _check_single_entry(project: Path) -> list[str]:
    issues: list[str] = []
    required_commands = {
        "simulation": (
            "backend",
            "environment",
            "scene",
            "kinematics",
            "prepare",
        ),
        "runtime": ("build-ros-sdk", "build-policy-sdk", "test"),
        "learning": ("prepare-policy", "policy", "test"),
    }
    for end in CURRENT_ENDS:
        dockerfiles = sorted((project / end).rglob("Dockerfile"))
        composes = sorted(
            path
            for path in (project / end).rglob(COMPOSE_NAME)
            if path.parent.name == "docker"
        )
        run_scripts = sorted((project / end).rglob(f"scripts/{RUN_SCRIPT_NAME}"))
        if len(dockerfiles) != 1:
            issues.append(f"{project / end}: expected exactly one Dockerfile")
        if len(composes) != 1:
            issues.append(f"{project / end}: expected exactly one docker/{COMPOSE_NAME}")
        if len(run_scripts) != 1:
            issues.append(f"{project / end}: expected exactly one scripts/{RUN_SCRIPT_NAME}")
        if len(dockerfiles) == 1 and len(composes) == 1:
            dockerfile = dockerfiles[0]
            compose = composes[0]
            if compose.parent != dockerfile.parent:
                issues.append(
                    f"{compose}: must sit next to the single {end} Dockerfile"
                )
        if len(composes) == 1 and len(run_scripts) == 1:
            expected = composes[0].parent.parent / "scripts" / RUN_SCRIPT_NAME
            if run_scripts[0] != expected:
                issues.append(f"{run_scripts[0]}: unexpected run.sh location")
                continue
            text = run_scripts[0].read_text(encoding="utf-8")
            for redundant in ("idle", "shell", "exec"):
                if re.search(rf"(?m)^\s*{redundant}\)", text):
                    issues.append(
                        f"{run_scripts[0]}: {redundant} belongs to Compose or the image CMD"
                    )
            for command in required_commands[end]:
                if not re.search(rf"(?m)^\s*{re.escape(command)}\)", text):
                    issues.append(
                        f"{run_scripts[0]}: missing named command {command!r}"
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
    legacy_entrypoints = project / "infrastructure/docker/entrypoints"
    if legacy_entrypoints.exists():
        found.append(
            f"{legacy_entrypoints}: legacy entrypoint wrappers must be removed; "
            "source infrastructure/docker/setup.sh instead"
        )
    for end in END_ROOTS:
        for path in (project / end).rglob("compose.yaml"):
            found.append(f"{path}: legacy Compose name; use docker/{COMPOSE_NAME}")
    for root_compose in ("compose.yaml", COMPOSE_NAME):
        if (project / root_compose).exists():
            found.append(f"{project / root_compose}: root Compose wrapper is unnecessary")
    found.extend(_check_single_entry(project))

    environments = sorted(
        path
        for end in END_ROOTS
        for path in (project / end).rglob(COMPOSE_NAME)
        if path.parent.name == "docker"
    )
    for path in environments:
        if not path.is_file():
            found.append(f"{path}: missing")
            continue
        try:
            document = yaml.safe_load(path.read_text(encoding="utf-8"))
        except yaml.YAMLError as error:
            found.append(f"{path}: invalid YAML: {error}")
            continue
        for message in check_compose_document(document, name=str(path)):
            found.append(message)
        if isinstance(document, dict) and isinstance(document.get("services"), dict):
            implementation = path.parent.parent.relative_to(project).as_posix()
            for service_name, service in document["services"].items():
                if not isinstance(service, dict):
                    continue
                if service_name == "simulation":
                    source = _mounts(service).get("/app", {}).get("source", "")
                    if not isinstance(source, str) or (path.parent / source).resolve() != (
                        project / implementation
                    ).resolve():
                        found.append(
                            f"{path}: /app must bind only {implementation}, not the repository"
                        )
                env_path = path.parent.parent / "configs/container.env"
                if not env_path.is_file():
                    found.append(f"{env_path}: missing container runtime settings")
                else:
                    settings = dict(
                        line.split("=", 1)
                        for line in env_path.read_text(encoding="utf-8").splitlines()
                        if line and not line.lstrip().startswith("#") and "=" in line
                    )
                    expected_root = "/app" if service_name == "simulation" else "/workspace"
                    for key, value in {
                        "PROJECT_ROOT": expected_root,
                        "PROJECT_CONTAINER": "1",
                    }.items():
                        if settings.get(key) != value:
                            found.append(f"{env_path}: {key} must be {value}")
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
