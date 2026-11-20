"""Strict planning/kinematics configuration for the Marvin stack task."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from types import MappingProxyType
from typing import Annotated, Literal

from pydantic import BeforeValidator, Field, StrictInt, model_validator
from robot_contracts.validation import (
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

from robot_backend_sim.common.config.camera_config import CAMERA_NAMES
from robot_backend_sim.marvin_sim.robot_config import Seven
from robot_backend_sim.marvin_sim.stack.scene_config import MarvinStackSceneConfig

__all__ = [
    "BudgetConfig",
    "ClearanceConfig",
    "IKConfig",
    "KinematicsCheckConfig",
    "KinematicsConfigError",
    "MARVIN_ARM_JOINT_RANGES",
    "STACK_KEY_POSES",
    "TargetConfig",
    "VisibilityRule",
    "VisibilitySection",
    "load_kinematics_config",
]

KinematicsConfigError = StrictMappingError

MARVIN_ARM_JOINT_RANGES = (
    (-3.106686, 3.106686),
    (-2.094395, 2.094395),
    (-3.106686, 3.106686),
    (-2.530727, 1.047198),
    (-3.106686, 3.106686),
    (-1.047198, 1.047198),
    (-1.570796, 1.570796),
)

STACK_KEY_POSES = (
    "pregrasp",
    "grasp",
    "lift",
    "place_hover",
    "place",
    "retreat",
)
_OBJECT_NAMES = ("source", "target")


def _camera_objects(camera: str) -> tuple[str, ...]:
    if camera == "left_wrist":
        return ("source", "target", "table")
    return ("source", "target")


class IKConfig(StrictMappingModel):
    dt_s: Annotated[PositiveFloat, Field(gt=1e-4, le=0.05)]
    max_iterations: Annotated[PositiveInt, Field(le=5000)]
    position_tolerance_m: Annotated[PositiveFloat, Field(gt=1e-6, le=0.05)]
    orientation_tolerance_rad: Annotated[PositiveFloat, Field(gt=1e-6, le=0.5)]
    posture_cost: Annotated[StrictFloat, Field(ge=0.0, le=1.0)]
    position_cost: Annotated[PositiveFloat, Field(gt=0.0, le=100.0)]
    orientation_cost: Annotated[PositiveFloat, Field(gt=0.0, le=100.0)]
    lm_damping: Annotated[StrictFloat, Field(ge=0.0, le=1.0)]
    limit_margin_rad: Annotated[StrictFloat, Field(ge=0.0, le=0.2)] = 0.05
    solver: Literal["daqp"]
    initial_right_joint_positions: Seven

    @model_validator(mode="after")
    def _limits(self) -> IKConfig:
        for index, (value, (lower, upper)) in enumerate(
            zip(self.initial_right_joint_positions, MARVIN_ARM_JOINT_RANGES, strict=True),
            start=1,
        ):
            margin = self.limit_margin_rad
            if not (lower + margin <= value <= upper - margin):
                raise contract_error(
                    f"initial_right_joint_positions[{index - 1}]",
                    f"must stay within [{lower + margin:.4f}, {upper - margin:.4f}]",
                )
        return self


class ClearanceConfig(StrictMappingModel):
    minimum_clearance_m: Annotated[PositiveFloat, Field(gt=1e-5, le=0.05)]
    collision_detection_distance_m: Annotated[PositiveFloat, Field(le=0.2)]
    sample_step_rad: Annotated[PositiveFloat, Field(gt=1e-5, le=0.05)]
    max_samples_per_segment: Annotated[PositiveInt, Field(ge=2, le=5000)]

    @model_validator(mode="after")
    def _clearance(self) -> ClearanceConfig:
        if self.collision_detection_distance_m < self.minimum_clearance_m:
            raise contract_error(
                "collision_detection_distance_m",
                "must be at least minimum_clearance_m",
            )
        return self


class TargetConfig(StrictMappingModel):
    pregrasp_height_m: Annotated[PositiveFloat, Field(gt=0.01, le=0.5)]
    grasp_height_m: Annotated[PositiveFloat, Field(gt=0.001, le=0.1)]
    lift_height_m: Annotated[PositiveFloat, Field(gt=0.02, le=0.5)]
    place_hover_height_m: Annotated[PositiveFloat, Field(gt=0.02, le=0.5)]
    place_clearance_m: Annotated[PositiveFloat, Field(gt=0.001, le=0.1)]
    retreat_height_m: Annotated[PositiveFloat, Field(gt=0.02, le=0.5)]
    approach_tilt_deg: Annotated[StrictFloat, Field(ge=-60.0, le=60.0)] = 0.0
    approach_yaw_deg: Annotated[StrictFloat, Field(ge=-180.0, le=180.0)] = 0.0
    place_yaw_deg: Annotated[StrictFloat, Field(ge=-180.0, le=180.0)] = 0.0


class VisibilityRule(StrictMappingModel):
    camera: Literal["global", "left_wrist", "right_wrist"]
    object: Literal["source", "target", "table"]
    phase: Literal["pregrasp", "grasp", "lift", "place_hover", "place", "retreat"]
    mode: Literal["min_pixels", "record_only"]
    min_pixels: Annotated[PositiveInt, Field(le=4096 * 4096)] | None = None

    @model_validator(mode="after")
    def _rule(self) -> VisibilityRule:
        if self.mode == "min_pixels" and self.min_pixels is None:
            raise contract_error("min_pixels", "is required when mode is 'min_pixels'")
        if self.mode == "record_only" and self.min_pixels is not None:
            raise contract_error("min_pixels", "must be omitted when mode is 'record_only'")
        return self

    @property
    def record_only(self) -> bool:
        return self.mode == "record_only"


class VisibilitySection(StrictMappingModel):
    rules: Annotated[tuple[VisibilityRule, ...], BeforeValidator(as_tuple)]

    @model_validator(mode="after")
    def _rules(self) -> VisibilitySection:
        expected = {
            (camera, obj, phase)
            for camera in CAMERA_NAMES
            for obj in _camera_objects(camera)
            for phase in STACK_KEY_POSES
        }
        seen: set[tuple[str, str, str]] = set()
        for index, rule in enumerate(self.rules):
            key = (rule.camera, rule.object, rule.phase)
            if key in seen:
                raise contract_error(f"rules[{index}]", f"duplicate rule for {key}")
            if key not in expected:
                raise contract_error(f"rules[{index}]", f"unexpected rule for {key}")
            seen.add(key)
        missing = sorted(expected - seen)
        if missing:
            raise contract_error("rules", f"missing visibility rules: {missing[0]}")
        return self


class BudgetConfig(StrictMappingModel):
    max_total_samples: Annotated[StrictInt, Field(ge=1000, le=5_000_000)]
    max_elapsed_s: Annotated[PositiveFloat, Field(gt=1.0, le=3600.0)]
    max_png_count: Annotated[StrictInt, Field(ge=0, le=10_000)]
    max_png_total_bytes: Annotated[StrictInt, Field(ge=0, le=1_000_000_000)]
    max_report_bytes: Annotated[StrictInt, Field(ge=1024, le=100_000_000)]


class KinematicsCheckConfig(StrictMappingModel):
    source: str = ""
    ik: IKConfig
    clearance: ClearanceConfig
    targets: TargetConfig
    visibility: VisibilitySection
    budgets: BudgetConfig
    camera_resolutions: Mapping[str, tuple[int, int]] = Field(default_factory=dict)

    @model_validator(mode="after")
    def _freeze_maps(self) -> KinematicsCheckConfig:
        object.__setattr__(
            self, "camera_resolutions", MappingProxyType(dict(self.camera_resolutions))
        )
        return self

    @property
    def camera_names(self) -> tuple[str, ...]:
        return CAMERA_NAMES

    @property
    def object_names(self) -> tuple[str, ...]:
        return _OBJECT_NAMES

    @property
    def visibility_rules(self) -> tuple[VisibilityRule, ...]:
        return self.visibility.rules

    def thresholds(self, camera: str, object_name: str, phase: str) -> VisibilityRule:
        for rule in self.visibility.rules:
            if rule.camera == camera and rule.object == object_name and rule.phase == phase:
                return rule
        raise KeyError((camera, object_name, phase))


def load_kinematics_config(
    path: Path | str, scene_config: MarvinStackSceneConfig
) -> KinematicsCheckConfig:
    source_path = Path(path).resolve()
    try:
        document = read_unique_yaml(source_path)
    except OSError as error:
        raise KinematicsConfigError(source_path, "<file>", str(error)) from error
    except UniqueKeyError as error:
        raise KinematicsConfigError(error.source, error.field, error.message) from error
    if not isinstance(document, dict):
        raise KinematicsConfigError(source_path, "<root>", "must be a mapping")
    resolutions = {
        camera.name: (camera.width, camera.height) for camera in scene_config.cameras
    }
    config = validate_config_model(
        KinematicsCheckConfig,
        {**document, "source": str(source_path), "camera_resolutions": resolutions},
        source_path,
        KinematicsConfigError,
    )
    for rule in config.visibility.rules:
        if rule.min_pixels is not None:
            width, height = resolutions[rule.camera]
            if rule.min_pixels > width * height:
                raise KinematicsConfigError(
                    source_path,
                    "visibility.rules",
                    f"min_pixels {rule.min_pixels} exceeds resolution for {rule.camera}",
                )
    return config
