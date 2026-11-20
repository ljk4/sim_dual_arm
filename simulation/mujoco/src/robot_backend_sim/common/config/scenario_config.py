"""Resolve a robot/task binding into one in-memory simulation backend document.

The selection is the same strict :mod:`robot_contracts.binding` load used by the
other ends.  Paths are expanded against the selected recipe directory, the
recipe's robot profile must match the binding profile. The caller can supply the
run seed and realtime speed before strict :class:`BackendConfig` validation.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robot_contracts.binding import (
    BindingError,
    ResolvedBinding,
    load_binding,
    resolve_binding_path,
)
from robot_contracts.models import ContractError
from robot_contracts.validation import UniqueKeyError, read_unique_yaml

ROLE_RECIPES = {
    "teacher": ("simulation_teacher", "collection"),
    "policy": ("simulation_policy", "inference"),
}
ROBOT_REFERENCE_FIELDS = ("profile", "schemas", "scene", "control", "stream", "follower", "task")


class ScenarioError(ValueError):
    """The robot/task selection cannot produce one valid backend document."""


@dataclass(frozen=True)
class ScenarioBackend:
    """The effective backend document and its provenance."""

    document: dict[str, Any]
    base_dir: Path
    selection: dict[str, Any]

    @property
    def source(self) -> str:
        selection = self.selection
        return f"scenario:{selection['robot']}/{selection['task']}/{selection['role']}"


def resolve_shared_root(explicit: Path | None) -> Path:
    if explicit is not None:
        return Path(explicit)
    environment = os.environ.get("SHARED_ROOT")
    if environment:
        return Path(environment)
    here = Path(__file__).resolve()
    for candidate in (here.parents[6] / "shared", here.parents[5] / "shared", Path("/shared")):
        if candidate.is_dir():
            return candidate
    return Path("/shared")


def _binding_path(shared_root: Path, robot: str, task: str) -> Path:
    try:
        path = resolve_binding_path(shared_root, robot, task)
    except BindingError as error:
        raise ScenarioError(str(error)) from error
    if not path.is_file():
        raise ScenarioError(f"scenario {robot}/{task} has no binding file: {path}")
    return path


def _load_recipe(path: Path) -> dict[str, Any]:
    try:
        document = read_unique_yaml(path)
    except (UniqueKeyError, OSError) as error:
        raise ScenarioError(f"cannot read backend recipe {path}: {error}") from error
    if not isinstance(document, dict):
        raise ScenarioError(f"backend recipe must be one YAML object: {path}")
    return document


def _select(
    *, robot: str, task: str, role: str, shared_root: Path | None
) -> tuple[dict[str, Any], Path, dict[str, Any], ResolvedBinding]:
    if role not in ROLE_RECIPES:
        raise ScenarioError("role must be 'teacher' or 'policy'")
    recipe_field, stage = ROLE_RECIPES[role]
    root = resolve_shared_root(shared_root)
    binding_path = _binding_path(root, robot, task)
    try:
        resolved = load_binding(binding_path, shared_root=root)
    except (BindingError, ContractError, OSError) as error:
        raise ScenarioError(str(error)) from error
    if getattr(resolved.binding.recipes, recipe_field) is None:
        raise ScenarioError(
            f"scenario {robot}/{task} declares no recipes.{recipe_field}"
        )
    reference = resolved.references[f"recipes.{recipe_field}"]
    selection = {
        "mode": "scenario",
        "robot": robot,
        "task": task,
        "role": role,
        "stage": stage,
        "binding": {"path": str(resolved.path), "sha256": resolved.sha256},
        "recipe": reference.record(),
    }
    return _load_recipe(reference.path), reference.path.parent, selection, resolved


def resolve_scenario_backend(
    *,
    robot: str | None = None,
    task: str | None = None,
    role: str = "teacher",
    shared_root: Path | None = None,
    seed: int | None = None,
    realtime_speed: float | None = None,
    scenario: str | None = None,
) -> ScenarioBackend:
    """Load a robot/task recipe and return the effective backend document.

    ``scenario`` is a convenience alternative to ``robot`` and ``task``: pass
    ``"<robot>/<task>"`` instead of the two separate values.  Relative
    ``robot.*`` references are expanded to absolute paths so the document can be
    snapshot to the run directory and validated from there.
    """
    if scenario is not None:
        if robot is not None or task is not None:
            raise ScenarioError("scenario cannot be combined with robot/task")
        robot, separator, task = scenario.partition("/")
        if not separator or not robot or not task or "/" in task:
            raise ScenarioError("scenario must be '<robot>/<task>'")
    if robot is None or task is None:
        raise ScenarioError("robot and task are required")
    if seed is not None and (not isinstance(seed, int) or isinstance(seed, bool)):
        raise ScenarioError("seed must be an integer")
    document, base_dir, selection, resolved = _select(
        robot=robot, task=task, role=role, shared_root=shared_root
    )
    expected_profile = resolved.references["profiles.robot"].path
    _check_binding_profile(document, base_dir, expected_profile)
    robot_config = document.get("robot")
    if not isinstance(robot_config, dict):
        raise ScenarioError("backend recipe has no robot object")
    for field in ROBOT_REFERENCE_FIELDS:
        value = robot_config.get(field)
        if value is None:
            continue
        if not isinstance(value, str):
            raise ScenarioError(f"robot.{field} must be a path string")
        source = (base_dir / value).resolve()
        if not source.exists():
            raise ScenarioError(f"robot.{field} is missing: {source}")
        robot_config[field] = str(source)
    run = document.get("run")
    if not isinstance(run, dict):
        run = {}
        document["run"] = run
    if seed is not None:
        run["seed"] = seed
    if realtime_speed is not None:
        run["realtime_speed"] = realtime_speed
    return ScenarioBackend(document=document, base_dir=base_dir, selection=selection)


def _check_binding_profile(document: dict[str, Any], base_dir: Path, expected: Path) -> None:
    robot = document.get("robot")
    declared = None if not isinstance(robot, dict) else robot.get("profile")
    if not isinstance(declared, str):
        return
    actual = (base_dir / declared).resolve()
    if actual != expected:
        raise ScenarioError(
            f"recipe robot profile {actual} differs from binding profile {expected}"
        )
