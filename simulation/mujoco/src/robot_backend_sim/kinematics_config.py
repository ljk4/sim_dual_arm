"""Strict, frozen configuration for the S2 kinematic key-pose check."""

from __future__ import annotations

from pathlib import Path
from typing import Annotated

from pydantic import Field, model_validator
from robot_contracts.validation import (
    StrictMappingError,
    StrictMappingModel,
    UniqueKeyError,
    contract_error,
    read_unique_yaml,
    validate_config_model,
)

from .kinematics_config_sections import (
    CAMERA_NAMES,
    PHASES,
    BudgetConfig,
    ClearanceConfig,
    IKConfig,
    TargetConfig,
    VisibilityRule,
    VisibilitySection,
)
from .scene_config import SceneConfig

__all__ = [
    "PHASES", "BudgetConfig", "ClearanceConfig", "IKConfig", "KinematicsCheckConfig",
    "KinematicsConfigError", "TargetConfig", "VisibilityRule", "load_kinematics_config",
]

SCHEMA_VERSION = 1
KinematicsConfigError = StrictMappingError


class KinematicsCheckConfig(StrictMappingModel):
    source: str = ""
    schema_version: Annotated[int, Field(strict=True, ge=SCHEMA_VERSION, le=SCHEMA_VERSION)]
    ik: IKConfig
    clearance: ClearanceConfig
    targets: TargetConfig
    visibility: VisibilitySection
    budgets: BudgetConfig
    camera_names: tuple[str, ...] = ()
    object_names: tuple[str, ...] = ()
    camera_resolutions: dict[str, tuple[int, int]] = {}

    @property
    def visibility_rules(self) -> tuple[VisibilityRule, ...]:
        return self.visibility.rules

    def thresholds(self, camera: str, obj: str, phase: str) -> VisibilityRule:
        for rule in self.visibility.rules:
            if rule.camera == camera and rule.object == obj and phase in rule.phases:
                return rule
        raise KinematicsConfigError(
            self.source, f"visibility.{camera}.{obj}", f"missing phase {phase!r}")

    @model_validator(mode="after")
    def _coverage(self) -> KinematicsCheckConfig:
        if not self.camera_names:
            return self
        if self.camera_names != CAMERA_NAMES:
            raise contract_error("visibility.rules",
                                 f"scene cameras must be exactly {CAMERA_NAMES}")
        expected = {
            (camera, obj, phase)
            for camera in self.camera_names
            for obj in self.object_names + (("table",) if camera == "left_wrist" else ())
            for phase in PHASES
        }
        seen: set[tuple[str, str, str]] = set()
        for index, rule in enumerate(self.visibility.rules):
            field = f"visibility.rules[{index}]"
            if rule.camera not in self.camera_names:
                raise contract_error(f"{field}.camera", "unknown camera")
            if len(set(rule.phases)) != len(rule.phases):
                raise contract_error(f"{field}.phases", "duplicate phase")
            for phase in rule.phases:
                if phase not in PHASES:
                    raise contract_error(f"{field}.phases", f"must be one of {PHASES}")
                key = (rule.camera, rule.object, phase)
                if key not in expected:
                    raise contract_error(field, "unexpected camera/object pair")
                if key in seen:
                    raise contract_error(f"{field}.phases", "duplicate rule coverage")
                seen.add(key)
            if rule.mode == "required":
                if rule.min_pixels is None:
                    raise contract_error(f"{field}.min_pixels", "field is required")
                width, height = self.camera_resolutions[rule.camera]
                if rule.min_pixels > width * height:
                    raise contract_error(f"{field}.min_pixels",
                                         "must not exceed the camera resolution")
            elif rule.min_pixels is not None:
                raise contract_error(f"{field}.min_pixels",
                                     "record_only rules must not set a threshold")
        missing = expected - seen
        if missing:
            raise contract_error("visibility.rules", f"missing coverage for {sorted(missing)[0]}")
        return self


def load_kinematics_config(path: Path | str, scene_config: SceneConfig) -> KinematicsCheckConfig:
    """Load and cross-check one kinematics YAML file before any native work."""
    source_path = Path(path)
    try:
        document = read_unique_yaml(source_path)
    except UniqueKeyError as error:
        raise KinematicsConfigError(error.source, error.field, error.message) from error
    if not isinstance(document, dict):
        raise KinematicsConfigError(source_path, "<root>", "must be a mapping")
    camera_names = tuple(camera.name for camera in scene_config.cameras)
    object_names = tuple(obj.name for obj in scene_config.objects)
    resolutions = {camera.name: (camera.width, camera.height)
                   for camera in scene_config.cameras}
    return validate_config_model(
        KinematicsCheckConfig,
        {**document, "source": str(source_path), "camera_names": camera_names,
         "object_names": object_names, "camera_resolutions": resolutions},
        source_path, KinematicsConfigError,
    )
