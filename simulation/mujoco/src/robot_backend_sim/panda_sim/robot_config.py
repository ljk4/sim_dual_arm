"""Frozen robot-level configuration for the dual-Panda assembly."""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import BeforeValidator, Field, model_validator
from robot_contracts.validation import (
    PositiveFloat,
    StrictFloat,
    StrictMappingModel,
    as_tuple,
    contract_error,
)

from robot_backend_sim.common.config.camera_config import Unit, Vec3

__all__ = [
    "FINGER_JOINT_LIMIT",
    "MountsConfig",
    "PANDA_JOINT_RANGES",
    "ResetConfig",
    "RobotSelection",
]

PANDA_JOINT_RANGES = (
    (-2.8973, 2.8973), (-1.7628, 1.7628), (-2.8973, 2.8973), (-3.0718, -0.0698),
    (-2.8973, 2.8973), (-0.0175, 3.7525), (-2.8973, 2.8973),
)
FINGER_JOINT_LIMIT = 0.04

Rgba = Annotated[tuple[Unit, Unit, Unit, Unit], BeforeValidator(as_tuple)]


class MountsConfig(StrictMappingModel):
    base_x: Annotated[PositiveFloat, Field(ge=0.05, le=1.0)]
    base_y: StrictFloat
    base_z: Annotated[StrictFloat, Field(ge=0.0, le=3.0)]
    yaw_deg: Annotated[StrictFloat, Field(ge=-180.0, le=180.0)]
    bracket_length: Annotated[PositiveFloat, Field(ge=0.01, le=1.0)]
    bracket_width: Annotated[PositiveFloat, Field(ge=0.01, le=0.5)]
    bracket_height: Annotated[PositiveFloat, Field(ge=0.01, le=0.5)]
    tcp_position: Vec3
    rgba: Rgba


class ResetConfig(StrictMappingModel):
    home_joint_positions: Annotated[
        tuple[StrictFloat, ...], BeforeValidator(as_tuple), Field(min_length=7, max_length=7)]
    gripper_total_opening: Annotated[StrictFloat, Field(ge=0.0, le=2.0 * FINGER_JOINT_LIMIT)]

    @model_validator(mode="after")
    def _ranges(self) -> ResetConfig:
        for index, angle in enumerate(self.home_joint_positions):
            lower, upper = PANDA_JOINT_RANGES[index]
            if not lower <= angle <= upper:
                raise contract_error(f"home_joint_positions[{index}]",
                                     f"must be within joint{index + 1} range")
        return self


class RobotSelection(StrictMappingModel):
    """Explicit robot assembly selection for the Panda scene recipe."""

    assembly: Literal["panda"]
