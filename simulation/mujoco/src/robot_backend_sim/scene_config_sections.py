"""Nested Pydantic sections for the dual-Panda stack scene configuration."""

from __future__ import annotations

import json
import math
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

CAMERA_NAMES = ("global", "left_wrist", "right_wrist")
CAMERA_PARENTS = {"global": "column", "left_wrist": "left_hand", "right_wrist": "right_hand"}
INTEGRATORS = ("euler", "implicit", "implicitfast", "rk4")
PANDA_JOINT_RANGES = (
    (-2.8973, 2.8973), (-1.7628, 1.7628), (-2.8973, 2.8973), (-3.0718, -0.0698),
    (-2.8973, 2.8973), (-0.0175, 3.7525), (-2.8973, 2.8973),
)
FINGER_JOINT_LIMIT = 0.04
MAX_RANDOM_XY = 0.03
MAX_RANDOM_YAW_DEG = 10.0
_OBJECT_NAME_PATTERN = r"^[A-Za-z][A-Za-z0-9_]*$"
_RESERVED_BODY_NAMES = frozenset({"world", "column", "table"})
CAMERA_FIELDS = ("name", "parent", "position", "quaternion_wxyz", "fovy", "width", "height")

Unit = Annotated[StrictFloat, Field(ge=0.0, le=1.0)]
Vec2 = Annotated[tuple[StrictFloat, StrictFloat], BeforeValidator(as_tuple)]
Vec3 = Annotated[tuple[StrictFloat, StrictFloat, StrictFloat], BeforeValidator(as_tuple)]
Rgba = Annotated[tuple[Unit, Unit, Unit, Unit], BeforeValidator(as_tuple)]
Quaternion = Annotated[tuple[StrictFloat, StrictFloat, StrictFloat, StrictFloat],
                       BeforeValidator(as_tuple)]


def _declaration(data: object) -> object:
    if not isinstance(data, dict) or "declared_description_json" in data:
        return data
    if not all(name in data for name in CAMERA_FIELDS):
        return data
    declaration = {name: data[name] for name in CAMERA_FIELDS}
    try:
        encoded = json.dumps(declaration, sort_keys=True, separators=(",", ":"), allow_nan=False)
    except (TypeError, ValueError):
        return data
    return {**data, "declared_description_json": encoded}


class PhysicsConfig(StrictMappingModel):
    timestep: Annotated[PositiveFloat, Field(ge=1e-5, le=0.01)]
    gravity: Vec3
    integrator: Literal["euler", "implicit", "implicitfast", "rk4"]
    object_friction: Vec3

    @model_validator(mode="after")
    def _physics(self) -> PhysicsConfig:
        if self.gravity[2] >= 0.0:
            raise contract_error("gravity[2]", "must point downward")
        for index, value in enumerate(self.object_friction):
            if value < 0.0:
                raise contract_error(f"object_friction[{index}]", "must be non-negative")
        if self.object_friction[0] == 0.0:
            raise contract_error("object_friction[0]", "sliding friction must be positive")
        return self


class ColumnConfig(StrictMappingModel):
    radius: Annotated[PositiveFloat, Field(ge=0.01, le=0.5)]
    height: Annotated[PositiveFloat, Field(ge=0.1, le=3.0)]
    rgba: Rgba


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


class TableConfig(StrictMappingModel):
    center_xy: Vec2
    size_xy: Vec2
    top_z: Annotated[PositiveFloat, Field(ge=0.01, le=3.0)]
    thickness: Annotated[PositiveFloat, Field(ge=0.001, le=0.5)]
    rgba: Rgba

    @model_validator(mode="after")
    def _table(self) -> TableConfig:
        for index, value in enumerate(self.size_xy):
            if not 0.05 <= value <= 5.0:
                raise contract_error(f"size_xy[{index}]", "must be between 0.05 and 5.0")
        if self.thickness > self.top_z:
            raise contract_error("thickness", "must not exceed top_z")
        return self


class RandomizationConfig(StrictMappingModel):
    xy_m: Annotated[StrictFloat, Field(ge=0.0, le=MAX_RANDOM_XY)]
    yaw_deg: Annotated[StrictFloat, Field(ge=0.0, le=MAX_RANDOM_YAW_DEG)]


class ObjectConfig(StrictMappingModel):
    name: Annotated[NonEmptyStr, Field(pattern=_OBJECT_NAME_PATTERN)]
    half_extents: Vec3
    nominal_xy: Vec2
    mass: Annotated[PositiveFloat, Field(ge=0.001, le=10.0)]
    rgba: Rgba
    randomization: RandomizationConfig

    @model_validator(mode="after")
    def _object(self) -> ObjectConfig:
        for index, value in enumerate(self.half_extents):
            if not 0.005 <= value <= 0.25:
                raise contract_error(f"half_extents[{index}]",
                                     "must be between 0.005 and 0.25")
        if self.name in _RESERVED_BODY_NAMES:
            raise contract_error("name", "is a reserved scene body")
        return self


class CameraConfig(StrictMappingModel):
    name: NonEmptyStr
    parent: NonEmptyStr
    position: Vec3
    quaternion_wxyz: Quaternion
    fovy: Annotated[PositiveFloat, Field(ge=5.0, le=170.0)]
    width: Annotated[PositiveInt, Field(le=4096)]
    height: Annotated[PositiveInt, Field(le=4096)]
    declared_description_json: str = ""

    @model_validator(mode="before")
    @classmethod
    def _declaration(cls, data: object) -> object:
        return _declaration(data)

    @model_validator(mode="after")
    def _camera(self) -> CameraConfig:
        if self.name not in CAMERA_NAMES:
            raise contract_error("name", f"must be one of {CAMERA_NAMES}")
        if self.parent != CAMERA_PARENTS[self.name]:
            raise contract_error("parent", f"must be {CAMERA_PARENTS[self.name]!r}")
        norm = math.sqrt(sum(component * component for component in self.quaternion_wxyz))
        if not math.isfinite(norm) or norm < 1e-9:
            raise contract_error("quaternion_wxyz", "must be a non-zero finite quaternion")
        if abs(norm - 1.0) > 1e-3:
            raise contract_error("quaternion_wxyz", "must be a unit quaternion")
        object.__setattr__(self, "quaternion_wxyz",
                           tuple(component / norm for component in self.quaternion_wxyz))
        return self


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
