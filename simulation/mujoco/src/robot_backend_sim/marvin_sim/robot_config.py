"""Strict robot-level configuration for the fixed-base Marvin assembly."""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import BeforeValidator, Field, StrictBool, model_validator
from robot_contracts.validation import (
    PositiveFloat,
    StrictFloat,
    StrictMappingModel,
    as_tuple,
    contract_error,
)

from robot_backend_sim.common.config.camera_config import Vec3

__all__ = [
    "MarvinRobotConfig",
    "Seven",
]

Seven = Annotated[
    tuple[StrictFloat, ...], BeforeValidator(as_tuple), Field(min_length=7, max_length=7)
]


class MarvinRobotConfig(StrictMappingModel):
    assembly: Literal["marvin"]
    base_position: Vec3
    base_yaw_deg: Annotated[StrictFloat, Field(ge=-180.0, le=180.0)]
    home_joint_positions: Seven
    gravity_compensation: StrictBool
    gripper_open_total: Annotated[PositiveFloat, Field(gt=0.0, le=0.2)]
    gripper_closed_total: Annotated[StrictFloat, Field(ge=0.0, le=0.2)]
    arm_kp: Seven
    arm_kv: Seven
    joint_damping: Annotated[StrictFloat, Field(ge=0.0, le=100.0)]
    joint_armature: Annotated[StrictFloat, Field(ge=0.0, le=1.0)]
    gripper_kp: Annotated[PositiveFloat, Field(le=100000.0)]
    gripper_kv: Annotated[PositiveFloat, Field(le=10000.0)]
    gripper_force: Annotated[PositiveFloat, Field(le=10000.0)]
    finger_damping: Annotated[PositiveFloat, Field(le=10000.0)]

    @model_validator(mode="after")
    def _robot(self) -> MarvinRobotConfig:
        if not self.gripper_closed_total < self.gripper_open_total:
            raise contract_error(
                "gripper_closed_total", "must be smaller than gripper_open_total"
            )
        for name in ("arm_kp", "arm_kv"):
            if any(value <= 0.0 for value in getattr(self, name)):
                raise contract_error(name, "all seven values must be positive")
        return self
