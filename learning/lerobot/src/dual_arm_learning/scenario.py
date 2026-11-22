"""Robot/task binding selection for the learning entry points."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robot_contracts.binding import (
    BindingError,
    BindingStage,
    ResolvedBinding,
    ResolvedReference,
    load_binding,
    resolve_binding_path,
)
from robot_contracts.models import ContractError
from robot_contracts.project_paths import (
    project_path,
    resolve_shared_root as _resolve_shared_root,
)

SUPPORTED_IMPLEMENTATION = "lerobot"
SUPPORTED_ADAPTER = "act"


class ScenarioSelectionError(ValueError):
    """The requested robot/task cannot be used by this learning component."""


@dataclass(frozen=True)
class LearningScenario:
    """One validated robot/task selection with its real recipe reference."""

    resolved: ResolvedBinding
    recipe_field: str
    stage: BindingStage

    @property
    def robot(self) -> str:
        return self.resolved.binding.robot_id

    @property
    def task(self) -> str:
        return self.resolved.binding.task_id

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
    ) -> dict[str, Any]:
        """Deterministic provenance record for manifests and snapshots."""
        field = recipe_field or self.recipe_field
        recipe = self.recipe_record(field)
        record: dict[str, Any] = {
            "mode": "scenario",
            "robot": self.robot,
            "task": self.task,
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
        return record


def resolve_shared_root(explicit: Path | None, project_root: Path) -> Path:
    """Use the explicit root, then ``SHARED_ROOT``, then the project layout."""
    return _resolve_shared_root(explicit, project_root=project_root)


def artifact_paths(robot: str, task: str, project_root: Path) -> dict[str, Path]:
    """Canonical runtime artifact roots for one robot/task pair."""
    base = project_path(Path("var") / robot / task, project_root)
    return {
        "select": base / "datasets" / "raw" / "formal-select.json",
        "dataset": base / "datasets" / "lerobot",
        "cache": base / "debug" / "learning-cache",
        "models": base / "models",
        "logs": base / "logs",
        "training_logs": base / "logs" / "training",
    }


def latest_model(robot: str, task: str, project_root: Path) -> Path | None:
    """Return the newest ``act_<timestamp>`` model directory, if any."""
    models = artifact_paths(robot, task, project_root)["models"]
    if not models.is_dir():
        return None
    candidates = sorted(
        (path for path in models.iterdir() if path.is_dir() and path.name.startswith("act_")),
        key=lambda path: path.name,
        reverse=True,
    )
    return candidates[0] if candidates else None


def select_scenario(
    *,
    robot: str,
    task: str,
    recipe_field: str,
    stage: BindingStage | str,
    project_root: Path,
    shared_root: Path | None = None,
    model: Path | None = None,
) -> LearningScenario:
    """Load one binding, verify this end supports it and select a recipe."""
    root = resolve_shared_root(shared_root, project_root)
    try:
        binding_path = resolve_binding_path(root, robot, task)
    except BindingError as error:
        raise ScenarioSelectionError(str(error)) from error
    if not binding_path.is_file():
        raise ScenarioSelectionError(
            f"scenario {robot}/{task} has no binding file: {binding_path}"
        )
    stage_value = BindingStage(stage)
    try:
        resolved = load_binding(binding_path, shared_root=root, model=model)
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
    adapter = resolved.binding.learning.adapter
    if adapter != SUPPORTED_ADAPTER:
        raise ScenarioSelectionError(
            f"unsupported learning.adapter {adapter!r} in scenario "
            f"{resolved.binding.scenario_id!r}; this learning component only "
            f"implements {SUPPORTED_ADAPTER!r}"
        )
    try:
        resolved.validate_stage(stage_value)
    except BindingError as error:
        hint = (
            " (pass --model with a trained model directory)"
            if stage_value is BindingStage.inference
            else ""
        )
        raise ScenarioSelectionError(f"{error}{hint}") from error
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
