"""Resolve a ``(robot, task)`` binding into one in-memory runtime configuration.

The runtime consumes the same public binding as the simulation and learning
ends.  A teacher selection injects ``interfaces.teacher_api`` and checks the
stream budget against the simulation task judge.  A policy selection only
needs the runtime recipe and profiles: model identity comes from the Policy
Service handshake, so the runtime never reads a bundle or any model file.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from robot_contracts.binding import (
    BindingError,
    ResolvedBinding,
    load_binding,
    resolve_binding_path,
)
from robot_contracts.models import ContractError
from robot_contracts.validation import UniqueKeyError, read_unique_yaml

ROLE_RECIPES = {
    "teacher": ("runtime_teacher", "collection"),
    "policy": ("runtime_policy", "inference"),
}
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
        return (
            f"scenario:{self.selection['robot']}/{self.selection['task']}"
            f"/{self.selection['role']}"
        )


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


def _binding_path(shared_root: Path, robot: str, task: str) -> Path:
    try:
        path = resolve_binding_path(shared_root, robot, task)
    except BindingError as error:
        raise ScenarioRuntimeError(str(error)) from error
    if not path.is_file():
        raise ScenarioRuntimeError(f"scenario {robot}/{task} has no binding file: {path}")
    return path


def _load_recipe(path: Path) -> dict[str, Any]:
    try:
        document = read_unique_yaml(path)
    except (UniqueKeyError, OSError) as error:
        raise ScenarioRuntimeError(f"cannot read runtime recipe {path}: {error}") from error
    if not isinstance(document, dict):
        raise ScenarioRuntimeError(f"runtime recipe must be one YAML object: {path}")
    return document


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
    robot: str,
    task: str,
    role: str = "policy",
    shared_root: Path | None = None,
    project_root: Path,
    seed: int | None = None,
    overrides: tuple[str, ...] | list[str] = (),
) -> ScenarioRuntime:
    """Resolve one runtime role into a document ready for strict validation."""
    if role not in ROLE_RECIPES:
        raise ScenarioRuntimeError("role must be 'teacher' or 'policy'")
    recipe_field, stage = ROLE_RECIPES[role]
    if seed is not None and (not isinstance(seed, int) or isinstance(seed, bool)):
        raise ScenarioRuntimeError("seed must be an integer")
    root = resolve_shared_root(shared_root, project_root)
    binding_path = _binding_path(root, robot, task)
    try:
        resolved = load_binding(binding_path, shared_root=root)
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
        raise ScenarioRuntimeError(str(error)) from error
    document = _load_recipe(reference.path)
    selection: dict[str, Any] = {
        "mode": "scenario",
        "robot": robot,
        "task": task,
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
