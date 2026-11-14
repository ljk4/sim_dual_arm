"""Collection-side scenario selection on top of the public binding API.

This module runs inside the runtime container, where ``robot_contracts`` (and
therefore Pydantic) is installed.  The host-side parallel launcher never
imports it: it asks the container for a resolved, read-only plan instead.

Selection only reads the binding and the referenced recipes.  It never writes
a ledger, reserves a seed, creates a worker directory or starts a backend.
"""

from __future__ import annotations

import hashlib
import os
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import yaml
from robot_contracts.binding import BindingError, ResolvedBinding, load_binding
from robot_contracts.models import ContractError
from robot_contracts.project_paths import project_path

from dual_arm_runtime.offline.campaign_ledger import CampaignError
from dual_arm_runtime.offline.campaign_ledger import load as load_campaign
from dual_arm_runtime.offline.collect_attempt import REFERENCES as RUNTIME_REFERENCES

SCENARIO_ID = re.compile(r"[A-Za-z_][A-Za-z0-9_.:-]{0,127}")
COLLECTION_RECIPES = ("runtime_teacher", "simulation_teacher", "campaign")
PROFILE_FIELDS = ("robot", "observation", "action")
RUNTIME_PROFILE_FIELDS = {
    "robot": "robot_profile",
    "observation": "observation_schema",
    "action": "action_schema",
}
SIMULATION_REFERENCE_FIELDS = (
    "profile",
    "schemas",
    "scene",
    "control",
    "stream",
    "follower",
    "task",
)
# The current collection implementation calls these common services directly.
REQUIRED_COMMON_ROS = ("robot_api_msgs", "sim_api_msgs")
# A Teacher extension package publishes these files; the capability id inside
# TeacherCapability.msg is the explicit wire/version identity.
TEACHER_SERVICE = "srv/GetTeacherChunk.srv"
TEACHER_CAPABILITY_FILE = "msg/TeacherCapability.msg"
_CAPABILITY_ID = re.compile(r"^\s*string\s+ID\s*=\s*(\S+)\s*$")


def shared_root_path(explicit: Path | None, project_root: Path) -> Path:
    if explicit is not None:
        return Path(explicit)
    environment = os.environ.get("SHARED_ROOT")
    if environment:
        return Path(environment)
    candidate = Path(project_root) / "shared"
    if candidate.is_dir():
        return candidate
    return Path("/shared")


def scenario_binding_path(shared_root: Path, scenario: str) -> Path:
    if SCENARIO_ID.fullmatch(scenario) is None or ".." in scenario:
        raise CampaignError("scenario id must contain 1..128 safe characters and no path traversal")
    path = Path(shared_root) / "scenarios" / scenario / "binding.yaml"
    if not path.is_file():
        raise CampaignError(f"scenario {scenario!r} has no binding file: {path}")
    return path


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_yaml(path: Path, label: str) -> dict[str, Any]:
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise CampaignError(f"cannot read {label} {path}: {error}") from error
    if not isinstance(document, dict):
        raise CampaignError(f"{label} must be a YAML object: {path}")
    return document


def _resolve(base: Path, value: str, label: str) -> Path:
    path = (base / value).resolve()
    if not path.is_file():
        raise CampaignError(f"{label} is missing: {path}")
    return path


def _package_name(package: Path) -> str:
    manifest = package / "package.xml"
    try:
        root = ET.parse(manifest).getroot()
    except (OSError, ET.ParseError) as error:
        raise CampaignError(
            f"cannot read extension package manifest {manifest}: {error}"
        ) from error
    name = root.findtext("name")
    if not name:
        raise CampaignError(f"extension package manifest has no name: {manifest}")
    name = name.strip()
    if package.name != name:
        raise CampaignError(
            f"extension directory {package.name!r} does not match package name {name!r}"
        )
    return name


def _teacher_capability(package: Path) -> str:
    path = package / TEACHER_CAPABILITY_FILE
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except OSError as error:
        raise CampaignError(f"Teacher capability file is missing: {path}: {error}") from error
    for line in lines:
        match = _CAPABILITY_ID.match(line.split("#", 1)[0])
        if match:
            return match.group(1)
    raise CampaignError(f"Teacher capability file has no string ID constant: {path}")


def _check_capabilities(resolved: ResolvedBinding, shared_root: Path) -> dict[str, str]:
    backend = resolved.binding.backend
    runtime = resolved.binding.runtime
    if backend.kind != "simulation":
        raise CampaignError(
            f"unsupported backend.kind {backend.kind!r} for collection; "
            "only simulation is implemented"
        )
    if backend.implementation != "mujoco":
        raise CampaignError(
            f"unsupported backend.implementation {backend.implementation!r} for "
            "collection; only mujoco is implemented"
        )
    if runtime.implementation != "ros2":
        raise CampaignError(
            f"unsupported runtime.implementation {runtime.implementation!r} for "
            "collection; only ros2 is implemented"
        )
    declared = list(resolved.binding.interfaces.common_ros)
    missing = [name for name in REQUIRED_COMMON_ROS if name not in declared]
    if missing:
        raise CampaignError(
            "unsupported Teacher capability: interfaces.common_ros must declare "
            f"{', '.join(REQUIRED_COMMON_ROS)}; missing {', '.join(missing)}"
        )
    for name in declared:
        package = Path(shared_root) / "common" / "interfaces" / "ros" / name
        if not package.is_dir():
            raise CampaignError(f"common ROS interface package is missing: {package}")

    teacher_api = resolved.binding.interfaces.teacher_api
    if not teacher_api:
        raise CampaignError(
            "unsupported Teacher capability: interfaces.teacher_api must name the "
            "selected extension's capability/version"
        )
    matches: list[tuple[str, Path]] = []
    for index, _declared in enumerate(resolved.binding.interfaces.extension_ros):
        field = f"interfaces.extension_ros[{index}]"
        package = resolved.references[field].path
        if not (package / TEACHER_SERVICE).is_file():
            continue
        if not (package / TEACHER_CAPABILITY_FILE).is_file():
            continue
        if _teacher_capability(package) == teacher_api:
            matches.append((_package_name(package), package))
    if not matches:
        raise CampaignError(
            "unsupported Teacher capability: no selected extension declares "
            f"{teacher_api!r} with {TEACHER_SERVICE} and {TEACHER_CAPABILITY_FILE}"
        )
    if len(matches) > 1:
        names = ", ".join(name for name, _path in matches)
        raise CampaignError(
            f"ambiguous Teacher capability {teacher_api!r}: multiple extensions match: {names}"
        )
    name, package = matches[0]
    return {
        "teacher": teacher_api,
        "teacher_package": name,
        "teacher_package_path": str(package),
    }


def _check_recipe_profiles(
    runtime_path: Path,
    runtime_document: dict[str, Any],
    backend_path: Path,
    backend_document: dict[str, Any],
    profiles: dict[str, dict[str, Any]],
) -> None:
    for name, field in RUNTIME_PROFILE_FIELDS.items():
        declared = runtime_document.get(field)
        if isinstance(declared, str):
            actual = (runtime_path.parent / declared).resolve()
            expected = Path(profiles[name]["path"])
            if actual != expected:
                raise CampaignError(
                    f"runtime teacher recipe {field} {actual} differs from "
                    f"scenario profile {expected}"
                )
    robot = backend_document.get("robot")
    if not isinstance(robot, dict):
        raise CampaignError(f"simulation teacher recipe has no robot object: {backend_path}")
    declared = robot.get("profile")
    if isinstance(declared, str):
        actual = (backend_path.parent / declared).resolve()
        expected = Path(profiles["robot"]["path"])
        if actual != expected:
            raise CampaignError(
                f"simulation teacher recipe robot.profile {actual} differs from "
                f"scenario profile {expected}"
            )


def _check_runtime_references(path: Path, document: dict[str, Any]) -> None:
    for name in RUNTIME_REFERENCES:
        declared = document.get(name)
        if isinstance(declared, str):
            _resolve(path.parent, declared, f"runtime teacher recipe {name}")


def _check_teacher_templates(
    runtime_path: Path,
    runtime_document: dict[str, Any],
    backend_path: Path,
    backend_document: dict[str, Any],
    *,
    require_parallel: bool,
    expected_teacher_api: str | None = None,
) -> None:
    if runtime_document.get("action_source") != "teacher":
        raise CampaignError(
            f"runtime teacher recipe must declare action_source: teacher ({runtime_path})"
        )
    declared_api = runtime_document.get("teacher_api")
    if not isinstance(declared_api, str) or not declared_api:
        raise CampaignError(f"runtime teacher recipe must declare teacher_api ({runtime_path})")
    if expected_teacher_api is not None and declared_api != expected_teacher_api:
        raise CampaignError(
            f"runtime teacher recipe teacher_api {declared_api!r} differs from "
            f"binding {expected_teacher_api!r}"
        )
    robot = backend_document.get("robot")
    if not isinstance(robot, dict):
        raise CampaignError(f"simulation teacher recipe has no robot object: {backend_path}")
    if robot.get("task_mode") != "teacher":
        raise CampaignError(
            f"simulation teacher recipe must declare robot.task_mode: teacher ({backend_path})"
        )
    for name in SIMULATION_REFERENCE_FIELDS:
        declared = robot.get(name)
        if isinstance(declared, str):
            path = (backend_path.parent / declared).resolve()
            if not path.exists():
                raise CampaignError(f"simulation teacher recipe robot.{name} is missing: {path}")
    if require_parallel and robot.get("render_quality", {}).get("enabled") is not True:
        raise CampaignError(
            "parallel recording requires simulation teacher recipe robot.render_quality.enabled"
        )


def _check_teacher_budget(
    runtime_document: dict[str, Any],
    backend_document: dict[str, Any],
    backend_path: Path,
) -> None:
    """Runtime stream clock and budget must equal the simulation task judge."""
    teacher = runtime_document.get("teacher")
    robot = backend_document.get("robot")
    if not isinstance(teacher, dict) or "task_sim_budget_ns" not in teacher:
        # Minimal synthetic recipes are validated by RuntimeRecipe at use.
        return
    if not isinstance(robot, dict):
        raise CampaignError("teacher recipes have no robot section")
    declared_task = robot.get("task")
    if not isinstance(declared_task, str):
        raise CampaignError(f"simulation teacher recipe must declare robot.task ({backend_path})")
    task_path = (backend_path.parent / declared_task).resolve()
    if not task_path.is_file():
        raise CampaignError(f"simulation task recipe is missing: {task_path}")
    task_document = _load_yaml(task_path, "simulation task recipe")
    success = task_document.get("success")
    if not isinstance(success, dict) or "max_task_sim_duration_s" not in success:
        return
    task_teacher = task_document.get("teacher")
    if not isinstance(task_teacher, dict):
        raise CampaignError(f"simulation task recipe is incomplete: {task_path}")
    try:
        budget_ns = round(float(success["max_task_sim_duration_s"]) * 1e9)
        chunk_size = int(task_teacher["chunk_size"])
        period_ns = int(task_teacher["action_period_ns"])
        runtime_budget_ns = int(teacher["task_sim_budget_ns"])
    except (KeyError, TypeError, ValueError) as error:
        raise CampaignError(f"teacher budget fields are not numeric: {error}") from error
    if runtime_budget_ns != budget_ns:
        raise CampaignError(
            f"runtime teacher.task_sim_budget_ns {runtime_budget_ns} differs from "
            f"simulation success.max_task_sim_duration_s ({budget_ns})"
        )
    if int(teacher.get("chunk_size", -1)) != chunk_size:
        raise CampaignError("runtime teacher.chunk_size differs from the simulation task")
    if int(teacher.get("action_period_ns", -1)) != period_ns:
        raise CampaignError("runtime teacher.action_period_ns differs from the simulation task")


def _profile_records(resolved: ResolvedBinding) -> dict[str, dict[str, Any]]:
    records: dict[str, dict[str, Any]] = {}
    for name in PROFILE_FIELDS:
        record = resolved.references[f"profiles.{name}"].record()
        record.pop("field")
        records[name] = record
    return records


def select_collection(
    *,
    scenario: str,
    shared_root: Path | None,
    project_root: Path,
    binding_sha256: str | None = None,
    require_parallel: bool = False,
) -> dict[str, Any]:
    """Resolve the collection recipes, profiles and capabilities from a binding."""
    root = shared_root_path(shared_root, project_root)
    binding_path = scenario_binding_path(root, scenario)
    try:
        resolved = load_binding(binding_path, shared_root=root)
    except (BindingError, ContractError, OSError) as error:
        raise CampaignError(str(error)) from error
    if binding_sha256 is not None and resolved.sha256 != binding_sha256:
        raise CampaignError(
            f"binding changed after preflight: {resolved.path} "
            f"sha256 {resolved.sha256} != {binding_sha256}"
        )
    try:
        resolved.validate_stage("collection")
    except BindingError as error:
        raise CampaignError(str(error)) from error
    teacher_capability = _check_capabilities(resolved, root)

    recipes: dict[str, dict[str, Any]] = {}
    for field in COLLECTION_RECIPES:
        declared = getattr(resolved.binding.recipes, field)
        if declared is None:
            raise CampaignError(
                f"scenario {resolved.binding.scenario_id!r} declares no recipes.{field}"
            )
        recipes[field] = resolved.references[f"recipes.{field}"].record()
    profiles = _profile_records(resolved)

    runtime_path = Path(recipes["runtime_teacher"]["path"])
    backend_path = Path(recipes["simulation_teacher"]["path"])
    runtime_document = _load_yaml(runtime_path, "runtime teacher recipe")
    backend_document = _load_yaml(backend_path, "simulation teacher recipe")
    _check_runtime_references(runtime_path, runtime_document)
    _check_recipe_profiles(runtime_path, runtime_document, backend_path, backend_document, profiles)
    _check_teacher_templates(
        runtime_path,
        runtime_document,
        backend_path,
        backend_document,
        require_parallel=require_parallel,
        expected_teacher_api=teacher_capability["teacher"],
    )
    _check_teacher_budget(runtime_document, backend_document, backend_path)
    campaign_path = Path(recipes["campaign"]["path"])
    load_campaign(campaign_path)

    return {
        "mode": "scenario",
        "scenario_id": resolved.binding.scenario_id,
        "binding": {"path": str(resolved.path), "sha256": resolved.sha256},
        "recipes": recipes,
        "profiles": profiles,
        "interfaces": {
            "common_ros": list(resolved.binding.interfaces.common_ros),
            "extension_ros": list(resolved.binding.interfaces.extension_ros),
            "teacher_api": resolved.binding.interfaces.teacher_api,
            "policy_api": resolved.binding.interfaces.policy_api,
        },
        "capabilities": {
            "backend": f"{resolved.binding.backend.kind}/{resolved.binding.backend.implementation}",
            "runtime": resolved.binding.runtime.implementation,
            **teacher_capability,
        },
        "overrides": [],
    }


def direct_selection(
    *,
    campaign: Path,
    runtime_teacher: Path,
    project_root: Path,
    simulation_teacher: Path | None = None,
) -> dict[str, Any]:
    """Describe explicit direct-mode inputs with real file identities."""
    project_root = Path(project_root).resolve()

    def record(field: str, value: Path) -> dict[str, Any]:
        path = project_path(value, project_root).resolve()
        if not path.is_file():
            raise CampaignError(f"direct collection recipe is missing: {path}")
        return {
            "field": f"direct.{field}",
            "declared": None,
            "path": str(path),
            "sha256": _sha256(path),
        }

    recipes = {
        "campaign": record("campaign", campaign),
        "runtime_teacher": record("runtime_teacher", runtime_teacher),
    }
    if simulation_teacher is not None:
        recipes["simulation_teacher"] = record("simulation_teacher", simulation_teacher)
    return {
        "mode": "direct",
        "scenario_id": None,
        "binding": None,
        "recipes": recipes,
        "profiles": None,
        "interfaces": None,
        "capabilities": None,
        "overrides": ["direct recipes supplied explicitly"],
    }


def campaign_config_path(
    *,
    campaign_config: Path | None,
    scenario: str | None,
    shared_root: Path | None,
    project_root: Path,
    default: Path | None = None,
) -> Path:
    """Resolve the campaign recipe for export, evaluation and campaign commands."""
    if scenario is not None:
        if campaign_config is not None:
            raise CampaignError("--scenario cannot be combined with --campaign-config")
        selection = select_collection(
            scenario=scenario, shared_root=shared_root, project_root=project_root
        )
        return Path(selection["recipes"]["campaign"]["path"])
    if campaign_config is None:
        if default is None:
            raise CampaignError("a campaign recipe is required")
        campaign_config = default
    path = project_path(campaign_config, project_root).resolve()
    if not path.is_file():
        raise CampaignError(f"campaign config is missing: {path}")
    return path
