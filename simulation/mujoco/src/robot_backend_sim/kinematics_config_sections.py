"""Nested Pydantic sections for the kinematic key-pose check configuration."""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import BeforeValidator, Field, model_validator
from robot_contracts.validation import (
    NonEmptyStr,
    PositiveFloat,
    PositiveInt,
    StrictFloat,
    StrictMappingModel,
    as_tuple,
    contract_error,
)

from .scene_config_sections import PANDA_JOINT_RANGES

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
