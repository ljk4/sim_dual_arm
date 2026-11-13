"""Resolve a scenario binding into one in-memory runtime configuration.

The runtime consumes the same public binding as the simulation and learning
ends.  A teacher selection injects ``interfaces.teacher_api`` and checks the
stream budget against the simulation task judge; a policy selection reads the
verified bundle manifest directly, so no ``policy_binding.json`` file and no
prepared directory exist between the bundle and the runtime process.
"""

from __future__ import annotations

import hashlib
import json
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from robot_contracts.binding import BindingError, ResolvedBinding, load_binding
from robot_contracts.models import ContractError
from robot_contracts.profiles import load_document
from robot_contracts.validation import UniqueKeyError, read_unique_yaml

SCENARIO_ID = re.compile(r"^[A-Za-z_][A-Za-z0-9_.:-]{0,127}$")
ROLE_RECIPES = {
    "teacher": ("runtime_teacher", "collection"),
    "policy": ("runtime_policy", "inference"),
}
BUNDLE_MANIFEST_NAMES = ("bundle_manifest.json", "manifest.json")
BUNDLE_FORMAT = "dual-arm-act-bundle-v1"
POLICY_FIELDS = (
    ("robot_profile", "robot"),
    ("observation_schema", "observation"),
    ("action_schema", "action"),
)


class ScenarioRuntimeError(ValueError):
    """The requested scenario cannot produce one deployable runtime recipe."""


@dataclass(frozen=True)
class ScenarioRuntime:
    document: dict[str, Any]
    base_dir: Path
    selection: dict[str, Any]
    overrides: tuple[str, ...]

    @property
    def source(self) -> str:
        return f"scenario:{self.selection['scenario_id']}/{self.selection['role']}"


def resolve_shared_root(explicit: Path | None, project_root: Path) -> Path:
    if explicit is not None:
        return Path(explicit)
    environment = os.environ.get("SHARED_ROOT")
    if environment:
        return Path(environment)
    candidate = Path(project_root) / "shared"
    if candidate.is_dir():
        return candidate
    return Path("/shared")


def _binding_path(shared_root: Path, scenario: str) -> Path:
    if SCENARIO_ID.fullmatch(scenario) is None or ".." in scenario:
        raise ScenarioRuntimeError(
            "scenario id must contain 1..128 safe characters and no path traversal"
        )
    path = Path(shared_root) / "scenarios" / scenario / "binding.yaml"
    if not path.is_file():
        raise ScenarioRuntimeError(f"scenario {scenario!r} has no binding file: {path}")
    return path


def _load_recipe(path: Path) -> dict[str, Any]:
    try:
        document = read_unique_yaml(path)
    except (UniqueKeyError, OSError) as error:
        raise ScenarioRuntimeError(f"cannot read runtime recipe {path}: {error}") from error
    if not isinstance(document, dict):
        raise ScenarioRuntimeError(f"runtime recipe must be one YAML object: {path}")
    return document


def _text(value: object, where: str) -> str:
    if not isinstance(value, str) or not value:
        raise ScenarioRuntimeError(f"{where} must be a non-empty string")
    return value


def _bundle_manifest(bundle: Path) -> dict[str, Any]:
    directory = Path(bundle)
    if not directory.is_dir():
        raise ScenarioRuntimeError(f"bundle directory is missing: {directory}")
    manifest_path = next(
        (directory / name for name in BUNDLE_MANIFEST_NAMES if (directory / name).is_file()),
        None,
    )
    if manifest_path is None:
        raise ScenarioRuntimeError(
            f"bundle has no {' or '.join(BUNDLE_MANIFEST_NAMES)}: {directory}"
        )
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ScenarioRuntimeError(
            f"cannot read bundle manifest {manifest_path}: {error}"
        ) from error
    if not isinstance(manifest, dict) or manifest.get("format") != BUNDLE_FORMAT:
        raise ScenarioRuntimeError(f"unsupported bundle format in {manifest_path}")
    stored = manifest.get("bundle_digest")
    payload = {key: value for key, value in manifest.items() if key != "bundle_digest"}
    expected = hashlib.sha256(
        json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()
    if stored != expected:
        raise ScenarioRuntimeError("bundle manifest digest does not match its contents")
    return manifest


def _bundle_policy_identity(
    resolved: ResolvedBinding, document: dict[str, Any]
) -> dict[str, Any]:
    bundle = resolved.bundle.path
    if bundle is None:
        raise ScenarioRuntimeError("scenario policy selection has no effective model bundle")
    manifest = _bundle_manifest(bundle)
    processors = manifest.get("processors")
    if not isinstance(processors, dict) or not processors.get("digest"):
        raise ScenarioRuntimeError("bundle manifest has no processors.digest")
    profile_record = manifest.get("profile")
    if not isinstance(profile_record, dict):
        raise ScenarioRuntimeError("bundle manifest has no profile record")
    scenario_record = resolved.references["profiles.robot"]
    if profile_record.get("semantic_sha256") != scenario_record.semantic_sha256:
        raise ScenarioRuntimeError(
            "bundle profile semantic hash differs from the scenario robot profile"
        )
    shipped = load_document(scenario_record.path)
    bundled_profile = bundle / "metadata/profile.json"
    if not bundled_profile.is_file():
        raise ScenarioRuntimeError(f"bundle profile is missing: {bundled_profile}")
    try:
        bundled = json.loads(bundled_profile.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ScenarioRuntimeError(
            f"cannot read bundle profile {bundled_profile}: {error}"
        ) from error
    if bundled != shipped:
        raise ScenarioRuntimeError("shipped scenario profile differs from the bundle profile")
    names = [joint.get("name") for joint in shipped.get("joints", [])]
    cameras = [camera.get("name") for camera in shipped.get("cameras", [])]
    observation = manifest.get("observation")
    action = manifest.get("action")
    if not isinstance(observation, dict) or not isinstance(action, dict):
        raise ScenarioRuntimeError("bundle manifest has no observation/action schema")
    if list(observation.get("state_names", [])) != names:
        raise ScenarioRuntimeError("bundle observation state names differ from the profile")
    if list(observation.get("cameras", [])) != cameras:
        raise ScenarioRuntimeError("bundle observation cameras differ from the profile")
    if list(action.get("names", [])) != names:
        raise ScenarioRuntimeError("bundle action names differ from the profile")
    policy = document.get("policy")
    if not isinstance(policy, dict):
        raise ScenarioRuntimeError("runtime policy recipe has no policy object")
    if int(action.get("chunk_size", 0)) != int(policy.get("chunk_size", 0)):
        raise ScenarioRuntimeError("bundle action chunk_size differs from the runtime recipe")
    endpoint = policy.get("endpoint")
    _text(endpoint, "policy.endpoint")
    policy["model_bundle_digest"] = _text(manifest.get("bundle_digest"), "bundle_digest")
    policy["processor_digest"] = _text(processors.get("digest"), "processors.digest")
    policy["robot_profile_hash"] = scenario_record.semantic_sha256
    policy["observation_schema_hash"] = resolved.references[
        "profiles.observation"
    ].semantic_sha256
    policy["action_schema_hash"] = resolved.references["profiles.action"].semantic_sha256
    policy["allow_test_policy"] = False
    policy["expected_test_only"] = False
    return {
        "path": str(bundle),
        "bundle_digest": policy["model_bundle_digest"],
        "processor_digest": policy["processor_digest"],
    }


def _check_teacher_capability(
    document: dict[str, Any], resolved: ResolvedBinding
) -> str:
    teacher_api = resolved.binding.interfaces.teacher_api
    if not teacher_api:
        raise ScenarioRuntimeError(
            "scenario teacher selection requires interfaces.teacher_api"
        )
    declared = document.get("teacher_api")
    if declared is not None and declared != teacher_api:
        raise ScenarioRuntimeError(
            f"runtime recipe teacher_api {declared!r} differs from binding {teacher_api!r}"
        )
    document["teacher_api"] = teacher_api
    return teacher_api


def _check_teacher_budget(document: dict[str, Any], resolved: ResolvedBinding) -> None:
    teacher = document.get("teacher")
    if not isinstance(teacher, dict) or "task_sim_budget_ns" not in teacher:
        return
    reference = resolved.references.get("recipes.simulation_teacher")
    if reference is None:
        raise ScenarioRuntimeError(
            "scenario declares no simulation_teacher recipe for teacher selection"
        )
    backend_path = reference.path
    try:
        backend = yaml.safe_load(backend_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ScenarioRuntimeError(f"cannot read simulation teacher recipe: {error}") from error
    robot = backend.get("robot") if isinstance(backend, dict) else None
    if not isinstance(robot, dict) or not isinstance(robot.get("task"), str):
        raise ScenarioRuntimeError(f"simulation teacher recipe has no robot.task ({backend_path})")
    task_path = (backend_path.parent / robot["task"]).resolve()
    if not task_path.is_file():
        raise ScenarioRuntimeError(f"simulation task recipe is missing: {task_path}")
    try:
        task = yaml.safe_load(task_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as error:
        raise ScenarioRuntimeError(f"cannot read simulation task recipe: {error}") from error
    success = task.get("success") if isinstance(task, dict) else None
    task_teacher = task.get("teacher") if isinstance(task, dict) else None
    if not isinstance(success, dict) or "max_task_sim_duration_s" not in success:
        return
    if not isinstance(task_teacher, dict):
        raise ScenarioRuntimeError(f"simulation task recipe is incomplete: {task_path}")
    try:
        budget_ns = round(float(success["max_task_sim_duration_s"]) * 1e9)
        chunk_size = int(task_teacher["chunk_size"])
        period_ns = int(task_teacher["action_period_ns"])
        runtime_budget_ns = int(teacher["task_sim_budget_ns"])
    except (KeyError, TypeError, ValueError) as error:
        raise ScenarioRuntimeError(f"teacher budget fields are not numeric: {error}") from error
    if runtime_budget_ns != budget_ns:
        raise ScenarioRuntimeError(
            f"runtime teacher.task_sim_budget_ns {runtime_budget_ns} differs from "
            f"simulation success.max_task_sim_duration_s ({budget_ns})"
        )
    if int(teacher.get("chunk_size", -1)) != chunk_size:
        raise ScenarioRuntimeError("runtime teacher.chunk_size differs from the simulation task")
    if int(teacher.get("action_period_ns", -1)) != period_ns:
        raise ScenarioRuntimeError(
            "runtime teacher.action_period_ns differs from the simulation task"
        )


def _override_profile_paths(document: dict[str, Any], resolved: ResolvedBinding) -> None:
    for field, name in POLICY_FIELDS:
        document[field] = str(resolved.references[f"profiles.{name}"].path)


def resolve_scenario_runtime(
    *,
    scenario: str,
    role: str = "policy",
    shared_root: Path | None = None,
    project_root: Path,
    bundle: Path | None = None,
    seed: int | None = None,
    overrides: tuple[str, ...] | list[str] = (),
) -> ScenarioRuntime:
    """Resolve one runtime role into a document ready for strict validation."""
    if role not in ROLE_RECIPES:
        raise ScenarioRuntimeError("role must be 'teacher' or 'policy'")
    recipe_field, stage = ROLE_RECIPES[role]
    if bundle is not None and role != "policy":
        raise ScenarioRuntimeError("--bundle applies to policy selection only")
    if seed is not None and (not isinstance(seed, int) or isinstance(seed, bool)):
        raise ScenarioRuntimeError("seed must be an integer")
    root = resolve_shared_root(shared_root, project_root)
    binding_path = _binding_path(root, scenario)
    try:
        resolved = load_binding(binding_path, shared_root=root, bundle=bundle)
    except (BindingError, ContractError, OSError) as error:
        raise ScenarioRuntimeError(str(error)) from error
    if getattr(resolved.binding.recipes, recipe_field) is None:
        raise ScenarioRuntimeError(
            f"scenario {resolved.binding.scenario_id!r} declares no recipes.{recipe_field}"
        )
    reference = resolved.references[f"recipes.{recipe_field}"]
    try:
        resolved.validate_stage(stage)
    except BindingError as error:
        hint = " (pass --bundle for inference)" if stage == "inference" else ""
        raise ScenarioRuntimeError(f"{error}{hint}") from error
    document = _load_recipe(reference.path)
    selection: dict[str, Any] = {
        "mode": "scenario",
        "scenario_id": resolved.binding.scenario_id,
        "role": role,
        "stage": stage,
        "binding": {"path": str(resolved.path), "sha256": resolved.sha256},
        "recipe": reference.record(),
    }
    _override_profile_paths(document, resolved)
    if role == "teacher":
        selection["teacher_api"] = _check_teacher_capability(document, resolved)
        _check_teacher_budget(document, resolved)
    else:
        selection["bundle"] = _bundle_policy_identity(resolved, document)
    reset = document.get("reset")
    if not isinstance(reset, dict):
        raise ScenarioRuntimeError("runtime recipe has no reset object")
    if seed is not None:
        reset["seed"] = seed
    return ScenarioRuntime(
        document=document,
        base_dir=reference.path.parent,
        selection=selection,
        overrides=tuple(overrides),
    )
