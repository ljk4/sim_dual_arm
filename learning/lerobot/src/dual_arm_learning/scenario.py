"""Scenario binding selection for the learning entry points.

This module is the only place where learning resolves ``--scenario``.  It uses
the public :mod:`robot_contracts.binding` loader, so path resolution, schema
checks and stage requirements stay identical to the simulation and runtime
side.  It never imports ROS, MuJoCo, Torch or a scenario implementation, never
loads model weights and never writes a file.

The selected recipe is the *actual* recipe handed to the existing learning
configuration loaders.  A binding only selects files; numeric values remain in
the recipe that the producing end owns.
"""

from __future__ import annotations

import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robot_contracts.binding import (
    BindingError,
    BindingStage,
    ResolvedBinding,
    ResolvedReference,
    load_binding,
)
from robot_contracts.models import ContractError

SCENARIO_ID = re.compile(r"^[A-Za-z_][A-Za-z0-9_.:-]{0,127}$")
SUPPORTED_IMPLEMENTATION = "lerobot"
SUPPORTED_ADAPTER = "act"


class ScenarioSelectionError(ValueError):
    """The requested scenario cannot be used by this learning component."""


@dataclass(frozen=True)
class LearningScenario:
    """One validated scenario selection with its real recipe reference."""

    resolved: ResolvedBinding
    recipe_field: str
    stage: BindingStage

    @property
    def scenario_id(self) -> str:
        return self.resolved.binding.scenario_id

    def recipe_reference(self, field: str) -> ResolvedReference:
        try:
            return self.resolved.references[f"recipes.{field}"]
        except KeyError as error:
            raise ScenarioSelectionError(
                f"scenario {self.scenario_id!r} declares no recipes.{field}"
            ) from error

    @property
    def recipe_path(self) -> Path:
        return self.recipe_reference(self.recipe_field).path

    def profile_path(self, name: str) -> Path:
        return self.resolved.references[f"profiles.{name}"].path

    def recipe_record(self, field: str) -> dict[str, Any]:
        return self.recipe_reference(field).record()

    def summary(
        self,
        *,
        recipe_field: str | None = None,
        include_dataset_recipe: bool = False,
        include_bundle: bool = False,
    ) -> dict[str, Any]:
        """Deterministic provenance record for manifests and snapshots."""
        field = recipe_field or self.recipe_field
        recipe = self.recipe_record(field)
        record: dict[str, Any] = {
            "mode": "scenario",
            "scenario_id": self.scenario_id,
            "stage": self.stage.value,
            "binding": {
                "path": str(self.resolved.path),
                "sha256": self.resolved.sha256,
            },
            "recipe": recipe,
        }
        if include_dataset_recipe and field != "dataset":
            record["dataset_recipe"] = self.recipe_record("dataset")
        if include_bundle:
            record["bundle"] = self.resolved.effective_bundle()
        return record


def resolve_shared_root(explicit: Path | None, project_root: Path) -> Path:
    """Use the explicit root, then ``SHARED_ROOT``, then the project layout."""
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
        raise ScenarioSelectionError(
            "scenario id must contain 1..128 safe characters and no path traversal"
        )
    path = Path(shared_root) / "scenarios" / scenario / "binding.yaml"
    if not path.is_file():
        raise ScenarioSelectionError(f"scenario {scenario!r} has no binding file: {path}")
    return path


def select_scenario(
    *,
    scenario: str,
    recipe_field: str,
    stage: BindingStage | str,
    project_root: Path,
    shared_root: Path | None = None,
    bundle: Path | None = None,
) -> LearningScenario:
    """Load one binding, verify this end supports it and select a recipe.

    ``stage`` controls the binding requirements: conversion/training/export use
    ``learning`` (no model needed), inference preparation and serving use
    ``inference`` (a real model source is mandatory).  ``bundle`` is an explicit
    model override; it is resolved by the binding loader against the declared
    roots and recorded as the effective source.
    """
    stage_value = BindingStage(stage)
    root = resolve_shared_root(shared_root, project_root)
    binding_path = _binding_path(root, scenario)
    try:
        resolved = load_binding(binding_path, shared_root=root, bundle=bundle)
    except (BindingError, ContractError, OSError) as error:
        raise ScenarioSelectionError(str(error)) from error

    implementation = resolved.binding.learning.implementation
    if implementation != SUPPORTED_IMPLEMENTATION:
        raise ScenarioSelectionError(
            f"unsupported learning.implementation {implementation!r} in scenario "
            f"{resolved.binding.scenario_id!r}; this learning component only "
            f"implements {SUPPORTED_IMPLEMENTATION!r}"
        )
    declared = getattr(resolved.binding.recipes, recipe_field)
    if declared is None:
        raise ScenarioSelectionError(
            f"scenario {resolved.binding.scenario_id!r} declares no recipes.{recipe_field}"
        )
    try:
        resolved.validate_stage(stage_value)
    except BindingError as error:
        hint = (
            " (pass --bundle with a verified model directory)"
            if stage_value is BindingStage.inference
            else ""
        )
        raise ScenarioSelectionError(f"{error}{hint}") from error
    adapter = resolved.binding.learning.adapter
    if adapter != SUPPORTED_ADAPTER:
        raise ScenarioSelectionError(
            f"unsupported learning.adapter {adapter!r} in scenario "
            f"{resolved.binding.scenario_id!r}; this learning component only "
            f"implements {SUPPORTED_ADAPTER!r}"
        )
    return LearningScenario(resolved=resolved, recipe_field=recipe_field, stage=stage_value)


def _same_file(left: Path, right: Path) -> bool:
    try:
        return os.path.samefile(left.resolve(strict=True), right.resolve(strict=True))
    except OSError:
        return False


def check_dataset_recipe_consistency(
    selection: LearningScenario, training_dataset_path: Path
) -> None:
    """The training recipe must use the dataset recipe the binding selected."""
    selected = selection.recipe_reference("dataset").path
    if not _same_file(Path(training_dataset_path), selected):
        raise ScenarioSelectionError(
            f"training recipe data.config {Path(training_dataset_path).resolve()} differs "
            f"from the selected dataset recipe {selected}; refusing to train on another "
            "data configuration"
        )


def check_profile_consistency(selection: LearningScenario, profile_path: Path) -> None:
    """The learning recipe profile must be the robot profile the binding selects."""
    selected = selection.profile_path("robot")
    if not _same_file(Path(profile_path), selected):
        raise ScenarioSelectionError(
            f"learning recipe profile {Path(profile_path).resolve()} differs from the "
            f"scenario robot profile {selected}"
        )
