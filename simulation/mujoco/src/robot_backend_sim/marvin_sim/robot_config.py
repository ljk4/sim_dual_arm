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
    "MARVIN_ARM_JOINT_RANGES",
    "MARVIN_CAMERA_NAMES",
    "MARVIN_CONTROL_JOINT_NAMES",
    "MARVIN_TEACHER_CAPABILITY",
    "MarvinRobotConfig",
    "Seven",
    "public_dual_arm_positions",
]


def public_dual_arm_positions(
    qpos: object,
    arm_qpos_left: tuple[int, ...],
    arm_qpos_right: tuple[int, ...],
    left_opening: float,
    right_opening: float,
) -> tuple[float, ...]:
    """Assemble the 16-DOF Marvin public joint vector from MuJoCo qpos and gripper openings."""
    left = tuple(float(qpos[a]) for a in arm_qpos_left)  # type: ignore[index]
    right = tuple(float(qpos[a]) for a in arm_qpos_right)  # type: ignore[index]
    return (*left, float(left_opening), *right, float(right_opening))

MARVIN_TEACHER_CAPABILITY = "marvin_teacher_api_msgs.teacher_chunk"

MARVIN_CONTROL_JOINT_NAMES = (
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "left_joint7",
    "left_gripper_width",
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_joint4",
    "right_joint5",
    "right_joint6",
    "right_joint7",
    "right_gripper_width",
)

MARVIN_CAMERA_NAMES = ("global", "left_wrist", "right_wrist")

MARVIN_ARM_JOINT_RANGES = (
    (-3.106686, 3.106686),
    (-2.094395, 2.094395),
    (-3.106686, 3.106686),
    (-2.530727, 1.047198),
    (-3.106686, 3.106686),
    (-1.047198, 1.047198),
    (-1.570796, 1.570796),
)

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
