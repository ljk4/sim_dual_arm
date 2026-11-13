"""Resolve a scenario binding into one in-memory simulation backend document.

The selection is the same strict :mod:`robot_contracts.binding` load used by the
other ends.  Paths are expanded against the selected recipe directory, the
recipe's robot profile must be the scenario profile, and the caller applies the
seed and ``--set`` overrides before strict :class:`BackendConfig` validation.
No prepared directory and no manifest file are produced here.
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robot_contracts.binding import BindingError, ResolvedBinding, load_binding
from robot_contracts.models import ContractError
from robot_contracts.overrides import apply_overrides
from robot_contracts.validation import UniqueKeyError, read_unique_yaml

SCENARIO_ID = re.compile(r"^[A-Za-z_][A-Za-z0-9_.:-]{0,127}$")
ROLE_RECIPES = {
    "teacher": ("simulation_teacher", "collection"),
    "policy": ("simulation_policy", "inference"),
}
ROBOT_REFERENCE_FIELDS = ("profile", "schemas", "scene", "control", "stream", "follower", "task")


class ScenarioError(ValueError):
    """The scenario cannot produce one valid backend selection."""


@dataclass(frozen=True)
class ScenarioBackend:
    """The effective backend document and its provenance."""

    document: dict[str, Any]
    base_dir: Path
    selection: dict[str, Any]

    @property
    def source(self) -> str:
        return f"scenario:{self.selection['scenario_id']}/{self.selection['role']}"


def resolve_shared_root(explicit: Path | None) -> Path:
    if explicit is not None:
        return Path(explicit)
    environment = os.environ.get("SHARED_ROOT")
    if environment:
        return Path(environment)
    here = Path(__file__).resolve()
    for candidate in (here.parents[5] / "shared", here.parents[4] / "shared", Path("/shared")):
        if candidate.is_dir():
            return candidate
    return Path("/shared")


def _binding_path(shared_root: Path, scenario: str) -> Path:
    if SCENARIO_ID.fullmatch(scenario) is None or ".." in scenario:
        raise ScenarioError(
            "scenario id must contain 1..128 safe characters and no path traversal"
        )
    path = Path(shared_root) / "scenarios" / scenario / "binding.yaml"
    if not path.is_file():
        raise ScenarioError(f"scenario {scenario!r} has no binding file: {path}")
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
    *, scenario: str, role: str, shared_root: Path | None, bundle: Path | None
) -> tuple[dict[str, Any], Path, dict[str, Any], ResolvedBinding]:
    if role not in ROLE_RECIPES:
        raise ScenarioError("--role must be 'teacher' or 'policy'")
    recipe_field, stage = ROLE_RECIPES[role]
    if bundle is not None and role != "policy":
        raise ScenarioError("--bundle applies to policy backend selection only")
    root = resolve_shared_root(shared_root)
    binding_path = _binding_path(root, scenario)
    try:
        resolved = load_binding(binding_path, shared_root=root, bundle=bundle)
    except (BindingError, ContractError, OSError) as error:
        raise ScenarioError(str(error)) from error
    if getattr(resolved.binding.recipes, recipe_field) is None:
        raise ScenarioError(
            f"scenario {resolved.binding.scenario_id!r} declares no recipes.{recipe_field}"
        )
    reference = resolved.references[f"recipes.{recipe_field}"]
    try:
        resolved.validate_stage(stage)
    except BindingError as error:
        hint = " (pass --bundle for inference)" if stage == "inference" else ""
        raise ScenarioError(f"{error}{hint}") from error
    selection = {
        "mode": "scenario",
        "scenario_id": resolved.binding.scenario_id,
        "role": role,
        "stage": stage,
        "binding": {"path": str(resolved.path), "sha256": resolved.sha256},
        "recipe": reference.record(),
    }
    if role == "policy":
        selection["bundle"] = resolved.effective_bundle()
    return _load_recipe(reference.path), reference.path.parent, selection, resolved


def resolve_scenario_backend(
    *,
    scenario: str,
    role: str = "teacher",
    shared_root: Path | None = None,
    bundle: Path | None = None,
    seed: int | None = None,
    overrides: tuple[str, ...] | list[str] = (),
) -> ScenarioBackend:
    """Load a scenario recipe and return the effective backend document.

    Relative ``robot.*`` references are expanded to absolute paths so the
    document can be snapshot to the run directory and validated from there.
    """
    if seed is not None and (not isinstance(seed, int) or isinstance(seed, bool)):
        raise ScenarioError("seed must be an integer")
    document, base_dir, selection, resolved = _select(
        scenario=scenario, role=role, shared_root=shared_root, bundle=bundle
    )
    expected_profile = resolved.references["profiles.robot"].path
    _check_scenario_profile(document, base_dir, expected_profile)
    robot = document.get("robot")
    if not isinstance(robot, dict):
        raise ScenarioError("backend recipe has no robot object")
    for field in ROBOT_REFERENCE_FIELDS:
        value = robot.get(field)
        if value is None:
            continue
        if not isinstance(value, str):
            raise ScenarioError(f"robot.{field} must be a path string")
        source = (base_dir / value).resolve()
        if not source.exists():
            raise ScenarioError(f"robot.{field} is missing: {source}")
        robot[field] = str(source)
    run = document.get("run")
    if not isinstance(run, dict):
        run = {}
        document["run"] = run
    if seed is not None:
        run["seed"] = seed
    try:
        document = apply_overrides(document, list(overrides))
    except ValueError as error:
        raise ScenarioError(str(error)) from error
    return ScenarioBackend(document=document, base_dir=base_dir, selection=selection)


def _check_scenario_profile(document: dict[str, Any], base_dir: Path, expected: Path) -> None:
    robot = document.get("robot")
    declared = None if not isinstance(robot, dict) else robot.get("profile")
    if not isinstance(declared, str):
        return
    actual = (base_dir / declared).resolve()
    if actual != expected:
        raise ScenarioError(
            f"recipe robot profile {actual} differs from scenario profile {expected}"
        )
