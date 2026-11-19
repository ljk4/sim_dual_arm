"""Strict, frozen configuration for the S2 kinematic key-pose check."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Annotated, Literal

from pydantic import BeforeValidator, Field, model_validator
from robot_contracts.validation import (
    NonEmptyStr,
    PositiveFloat,
    PositiveInt,
    StrictFloat,
    StrictMappingError,
    StrictMappingModel,
    UniqueKeyError,
    as_tuple,
    contract_error,
    read_unique_yaml,
    validate_config_model,
)

from robot_backend_sim.panda_sim.robot_config import PANDA_JOINT_RANGES

if TYPE_CHECKING:
    from robot_backend_sim.panda_sim.stack.scene_config import SceneConfig

__all__ = [
    "PHASES", "BudgetConfig", "ClearanceConfig", "IKConfig", "KinematicsCheckConfig",
    "KinematicsConfigError", "TargetConfig", "VisibilityRule", "load_kinematics_config",
]

SCHEMA_VERSION = 1
KinematicsConfigError = StrictMappingError
PHASES = ("pregrasp", "grasp_pose", "lift_pose", "place_hover", "place_pose", "retreat")
CAMERA_NAMES = ("global", "left_wrist", "right_wrist")
MAX_LAYOUTS = 100
MAX_ITERATIONS = 250
MAX_SAMPLES_PER_SEGMENT = 2000
MAX_PNG_BYTES = 128 * 1024 * 1024
MAX_REPORT_BYTES = 8 * 1024 * 1024


class IKConfig(StrictMappingModel):
    dt_s: Annotated[PositiveFloat, Field(gt=1e-4, le=0.05)]
    max_iterations: Annotated[PositiveInt, Field(le=MAX_ITERATIONS)]
    position_tolerance_m: Annotated[PositiveFloat, Field(gt=1e-6, le=0.05)]
    orientation_tolerance_rad: Annotated[PositiveFloat, Field(gt=1e-6, le=0.5)]
    posture_cost: Annotated[StrictFloat, Field(ge=0.0, le=1.0)]
    position_cost: Annotated[PositiveFloat, Field(gt=0.0, le=100.0)]
    orientation_cost: Annotated[PositiveFloat, Field(gt=0.0, le=100.0)]
    lm_damping: Annotated[StrictFloat, Field(ge=0.0, le=1.0)]
    # Interior margin (rad) reserved by the IK configuration limit on every
    # solved arm hinge, so a key pose can never land on a hard joint limit.
    limit_margin_rad: Annotated[StrictFloat, Field(ge=0.0, le=0.2)]
    solver: Literal["daqp"]
    initial_right_joint_positions: Annotated[
        tuple[StrictFloat, ...], BeforeValidator(as_tuple), Field(min_length=7, max_length=7)]

    @model_validator(mode="after")
    def _ranges(self) -> IKConfig:
        for index, angle in enumerate(self.initial_right_joint_positions):
            lower, upper = PANDA_JOINT_RANGES[index]
            if not lower <= angle <= upper:
                raise contract_error(f"initial_right_joint_positions[{index}]",
                                     f"must be within right_joint{index + 1} range")
        return self


class ClearanceConfig(StrictMappingModel):
    minimum_clearance_m: Annotated[PositiveFloat, Field(gt=0.0001, le=0.05)]
    collision_detection_distance_m: Annotated[PositiveFloat, Field(le=0.2)]
    sample_step_rad: Annotated[PositiveFloat, Field(gt=1e-5, le=0.01)]
    max_samples_per_segment: Annotated[PositiveInt, Field(ge=2, le=MAX_SAMPLES_PER_SEGMENT)]
    max_total_samples: Annotated[PositiveInt, Field(le=1_000_000)]

    @model_validator(mode="after")
    def _relations(self) -> ClearanceConfig:
        if self.collision_detection_distance_m < self.minimum_clearance_m:
            raise contract_error("collision_detection_distance_m",
                                 "must be at least minimum_clearance_m")
        if self.max_total_samples < self.max_samples_per_segment:
            raise contract_error("max_total_samples",
                                 "must be at least max_samples_per_segment")
        return self


class TargetConfig(StrictMappingModel):
    pregrasp_height_m: Annotated[StrictFloat, Field(ge=-0.2, le=0.5)]
    grasp_height_m: Annotated[StrictFloat, Field(ge=-0.2, le=0.5)]
    lift_height_m: Annotated[StrictFloat, Field(ge=-0.2, le=0.5)]
    place_hover_height_m: Annotated[StrictFloat, Field(ge=-0.2, le=0.5)]
    place_clearance_m: Annotated[StrictFloat, Field(ge=0.0, le=0.2)]
    retreat_height_m: Annotated[StrictFloat, Field(ge=-0.2, le=0.5)]


class VisibilityRule(StrictMappingModel):
    camera: NonEmptyStr
    object: NonEmptyStr
    phases: Annotated[tuple[NonEmptyStr, ...], BeforeValidator(as_tuple), Field(min_length=1)]
    mode: Literal["required", "record_only"]
    min_pixels: PositiveInt | None = None

    @property
    def record_only(self) -> bool:
        return self.mode == "record_only"


class BudgetConfig(StrictMappingModel):
    max_layouts: Annotated[PositiveInt, Field(le=MAX_LAYOUTS)]
    max_png_bytes: Annotated[PositiveInt, Field(le=MAX_PNG_BYTES)]
    max_report_bytes: Annotated[PositiveInt, Field(ge=1024, le=MAX_REPORT_BYTES)]


class VisibilitySection(StrictMappingModel):
    rules: Annotated[tuple[VisibilityRule, ...], BeforeValidator(as_tuple), Field(min_length=1)]


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
