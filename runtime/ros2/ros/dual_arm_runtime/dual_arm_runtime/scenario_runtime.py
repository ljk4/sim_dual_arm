"""Resolve a ``(robot, backend, task)`` binding into one in-memory runtime configuration."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robot_contracts.binding import (
    BindingError,
    ResolvedBinding,
    load_binding,
    resolve_binding_path,
)
from robot_contracts.io import read_yaml_unique_mapping
from robot_contracts.models import ContractError
from robot_contracts.project_paths import resolve_shared_root as _contracts_shared_root

from dual_arm_runtime.simulation.scenario import configure_simulation_scenario

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

    @property
    def source(self) -> str:
        return (
            f"scenario:{self.selection['robot']}/{self.selection['backend']}"
            f"/{self.selection['task']}/{self.selection['role']}"
        )


def resolve_shared_root(explicit: Path | None, project_root: Path) -> Path:
    return _contracts_shared_root(explicit, project_root)


def _binding_path(shared_root: Path, robot: str, backend: str, task: str) -> Path:
    try:
        path = resolve_binding_path(shared_root, robot, backend, task)
    except BindingError as error:
        raise ScenarioRuntimeError(str(error)) from error
    if not path.is_file():
        raise ScenarioRuntimeError(
            f"scenario {robot}/{backend}/{task} has no binding file: {path}"
        )
    return path


def _load_recipe(path: Path) -> dict[str, Any]:
    return read_yaml_unique_mapping(
        path, label="runtime recipe", error_cls=ScenarioRuntimeError
    )


def _override_profile_paths(document: dict[str, Any], resolved: ResolvedBinding) -> None:
    for field, name in POLICY_FIELDS:
        document[field] = str(resolved.references[f"profiles.{name}"].path)


def resolve_scenario_runtime(
    *,
    robot: str,
    backend: str,
    task: str,
    role: str = "policy",
    shared_root: Path | None = None,
    project_root: Path,
    seed: int | None = None,
) -> ScenarioRuntime:
    """Resolve one runtime role into a document ready for strict validation."""
    if role not in ROLE_RECIPES:
        raise ScenarioRuntimeError("role must be 'teacher' or 'policy'")
    recipe_field, stage = ROLE_RECIPES[role]
    if seed is not None and (not isinstance(seed, int) or isinstance(seed, bool)):
        raise ScenarioRuntimeError("seed must be an integer")
    root = resolve_shared_root(shared_root, project_root)
    binding_path = _binding_path(root, robot, backend, task)
    try:
        resolved = load_binding(binding_path, shared_root=root)
    except (BindingError, ContractError, OSError) as error:
        raise ScenarioRuntimeError(str(error)) from error
    if (
        resolved.binding.robot_id != robot
        or resolved.binding.backend.implementation != backend
        or resolved.binding.task_id != task
    ):
        raise ScenarioRuntimeError(
            f"binding {binding_path} declares {resolved.binding.scenario_id!r}, "
            f"expected {robot}/{backend}/{task!r}"
        )
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
        "backend": backend,
        "task": task,
        "scenario_id": resolved.binding.scenario_id,
        "role": role,
        "stage": stage,
        "binding": {"path": str(resolved.path), "sha256": resolved.sha256},
        "recipe": reference.record(),
    }
    _override_profile_paths(document, resolved)
    try:
        teacher_api = configure_simulation_scenario(document, resolved, role=role)
    except ValueError as error:
        raise ScenarioRuntimeError(str(error)) from error
    if teacher_api is not None:
        selection["teacher_api"] = teacher_api
    if seed is not None:
        reset = document.get("reset")
        if not isinstance(reset, dict):
            raise ScenarioRuntimeError("--reset-seed requires a reset block")
        reset["seed"] = seed
    return ScenarioRuntime(
        document=document,
        base_dir=reference.path.parent,
        selection=selection,
    )
