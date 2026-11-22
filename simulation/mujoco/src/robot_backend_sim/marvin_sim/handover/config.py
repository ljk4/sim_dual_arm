"""Strict planning recipe for the Marvin handover candidate paths.

The recipe stores task geometry relative to the block and box and the
execution/acceptance values measured in P02B.  Robot limits, TCP names and
collision groups come from the compiled robot assembly, not from this file.
"""

from __future__ import annotations

from pathlib import Path
from typing import Annotated, Literal

from pydantic import Field, model_validator
from robot_contracts.validation import (
    PositiveFloat,
    PositiveInt,
    StrictFloat,
    StrictMappingError,
    StrictMappingModel,
    UniqueKeyError,
    contract_error,
    read_unique_yaml,
    validate_config_model,
)

from robot_backend_sim.common.tasks.config import (
    ClearanceSettings,
    IKSettingsBase,
    Vec3,
)

__all__ = [
    "ApproachSettings",
    "ClearanceSettings",
    "DiagnosticsSettings",
    "ExecutionSettings",
    "GraspSettings",
    "GraspSide",
    "HandoverPlanConfig",
    "HandoverPlanConfigError",
    "HandoverSettings",
    "IKSettings",
    "RetreatSettings",
    "TransportSettings",
    "load_handover_plan_config",
]

HandoverPlanConfigError = StrictMappingError


class IKSettings(IKSettingsBase):
    # Interior margin (rad) reserved by the IK configuration limit on every
    # solved arm hinge, so no target can land on a hard joint limit.
    limit_margin_rad: Annotated[StrictFloat, Field(ge=0.0, le=0.2)]


class GraspSide(StrictMappingModel):
    along_block_y_m: Annotated[StrictFloat, Field(ge=-0.15, le=0.15)]
    approach_tilt_deg: Annotated[StrictFloat, Field(ge=-90.0, le=90.0)]
    approach_yaw_deg: Annotated[StrictFloat, Field(ge=-180.0, le=180.0)]


class GraspSettings(StrictMappingModel):
    tcp_above_block_center_m: Annotated[StrictFloat, Field(ge=0.0, le=0.2)]
    pregrasp_height_m: Annotated[PositiveFloat, Field(gt=0.0, le=0.3)]
    right_initial_pregrasp_height_m: Annotated[PositiveFloat, Field(gt=0.0, le=0.3)]
    lift_height_m: Annotated[PositiveFloat, Field(gt=0.0, le=0.3)]
    right_initial_lift_height_m: Annotated[PositiveFloat, Field(gt=0.0, le=0.3)]
    cohold_lift_height_m: Annotated[PositiveFloat, Field(gt=0.0, le=0.3)]
    closed_opening_m: Annotated[PositiveFloat, Field(gt=0.0, le=0.2)]
    left: GraspSide
    right_center: GraspSide
    left_cohold: GraspSide
    right_cohold: GraspSide

    @model_validator(mode="after")
    def _sides(self) -> GraspSettings:
        return self


class ApproachSettings(StrictMappingModel):
    left_ready_tcp_m: Vec3
    right_wait_tcp_m: Vec3
    right_wait_tilt_deg: Annotated[StrictFloat, Field(ge=-90.0, le=90.0)]


class HandoverSettings(StrictMappingModel):
    block_center_offset_m: Vec3


class RetreatSettings(StrictMappingModel):
    left_tcp_position_m: Vec3


class TransportSettings(StrictMappingModel):
    place_yaw_deg: Annotated[StrictFloat, Field(ge=-180.0, le=180.0)]
    preplace_block_center_above_rim_m: Annotated[PositiveFloat, Field(gt=0.0, le=0.3)]


class ExecutionSettings(StrictMappingModel):
    period_ns: Annotated[PositiveInt, Field(ge=1_000_000, le=200_000_000)]
    duration_peak_factor: Annotated[PositiveFloat, Field(ge=1.0, le=5.0)]
    minimum_segment_duration_s: Annotated[PositiveFloat, Field(gt=0.0, le=2.0)]


class DiagnosticsSettings(StrictMappingModel):
    contact_force_min_n: Annotated[PositiveFloat, Field(gt=0.0)]
    hold_duration_s: Annotated[PositiveFloat, Field(gt=0.0, le=5.0)]
    hold_sample_interval_s: Annotated[PositiveFloat, Field(gt=0.0, le=1.0)]
    max_object_linear_speed_m_s: Annotated[PositiveFloat, Field(gt=0.0)]
    max_object_angular_speed_rad_s: Annotated[PositiveFloat, Field(gt=0.0)]
    relative_position_tolerance_m: Annotated[PositiveFloat, Field(gt=0.0, le=0.05)]
    relative_orientation_tolerance_rad: Annotated[PositiveFloat, Field(gt=0.0, le=0.5)]
    stop_gripper_tolerance_m: Annotated[PositiveFloat, Field(gt=0.0, le=0.05)]


class HandoverPlanConfig(StrictMappingModel):
    source: str = ""
    frame: Literal["robot_base"]
    ik: IKSettings
    clearance: ClearanceSettings
    grasp: GraspSettings
    approach: ApproachSettings
    handover: HandoverSettings
    retreat: RetreatSettings
    transport: TransportSettings
    execution: ExecutionSettings
    diagnostics: DiagnosticsSettings


def load_handover_plan_config(path: Path | str) -> HandoverPlanConfig:
    source = Path(path)
    try:
        document = read_unique_yaml(source)
    except UniqueKeyError as error:
        raise HandoverPlanConfigError(error.source, error.field, error.message) from error
    if not isinstance(document, dict):
        raise HandoverPlanConfigError(source, "<root>", "must be a mapping")
    return validate_config_model(
        HandoverPlanConfig,
        {**document, "source": str(source)},
        source,
        HandoverPlanConfigError,
    )
