"""Frozen configuration for the dual-Panda stack scene."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Annotated, Literal

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

__all__ = [
    "CameraConfig", "camera_declared_description", "ColumnConfig", "MountsConfig",
    "ObjectConfig", "PhysicsConfig", "RandomizationConfig", "ResetConfig", "SceneConfig",
    "SceneConfigError", "TableConfig", "CAMERA_NAMES", "CAMERA_PARENTS",
    "FINGER_JOINT_LIMIT", "INTEGRATORS", "MAX_RANDOM_XY", "MAX_RANDOM_YAW_DEG",
    "PANDA_JOINT_RANGES", "load_scene_config",
]

SceneConfigError = StrictMappingError
TOP_LEVEL_KEYS = ("physics", "column", "mounts", "table", "objects", "cameras", "reset")
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


class SceneConfig(StrictMappingModel):
    source: str = ""
    physics: PhysicsConfig
    column: ColumnConfig
    mounts: MountsConfig
    table: TableConfig
    objects: Annotated[tuple[ObjectConfig, ...], BeforeValidator(as_tuple),
                       Field(min_length=2, max_length=2)]
    cameras: Annotated[tuple[CameraConfig, ...], BeforeValidator(as_tuple),
                        Field(min_length=3, max_length=3)]
    reset: ResetConfig

    @model_validator(mode="before")
    @classmethod
    def _cameras_are_named(cls, data: object) -> object:
        if isinstance(data, dict) and isinstance(data.get("cameras"), dict):
            cameras = data["cameras"]
            for name in cameras:
                if name not in CAMERA_NAMES:
                    raise contract_error(f"cameras.{name}", "unknown camera")
            entries = []
            for name in CAMERA_NAMES:
                if name not in cameras:
                    raise contract_error(f"cameras.{name}", "field is required")
                entries.append({**cameras[name], "name": name})
            data = {**data, "cameras": entries}
        return data

    @model_validator(mode="after")
    def _scene(self) -> SceneConfig:
        if tuple(camera.name for camera in self.cameras) != CAMERA_NAMES:
            raise contract_error("cameras", f"must be exactly {CAMERA_NAMES}")
        if (len({camera.width for camera in self.cameras}) != 1
                or len({camera.height for camera in self.cameras}) != 1):
            raise contract_error("cameras", "all cameras must share one resolution")
        names: set[str] = set()
        for index, obj in enumerate(self.objects):
            if obj.name in names:
                raise contract_error(f"objects[{index}].name", "duplicate object name")
            names.add(obj.name)
        half_x, half_y = self.table.size_xy[0] / 2.0, self.table.size_xy[1] / 2.0
        for index, obj in enumerate(self.objects):
            reach = math.hypot(obj.half_extents[0], obj.half_extents[1])
            reach += obj.randomization.xy_m
            for axis, (nominal, half) in enumerate((
                (obj.nominal_xy[0] - self.table.center_xy[0], half_x),
                (obj.nominal_xy[1] - self.table.center_xy[1], half_y),
            )):
                if abs(nominal) + reach > half + 1e-9:
                    raise contract_error(f"objects[{index}].nominal_xy[{axis}]",
                                         "randomized footprint crosses the table edge")
        for first in range(len(self.objects)):
            for second in range(first + 1, len(self.objects)):
                left, right = self.objects[first], self.objects[second]
                dx = max(0.0, abs(left.nominal_xy[0] - right.nominal_xy[0])
                         - left.randomization.xy_m - right.randomization.xy_m)
                dy = max(0.0, abs(left.nominal_xy[1] - right.nominal_xy[1])
                         - left.randomization.xy_m - right.randomization.xy_m)
                if math.hypot(dx, dy) <= (math.hypot(*left.half_extents[:2])
                                          + math.hypot(*right.half_extents[:2])):
                    raise contract_error(f"objects[{second}].nominal_xy",
                                         f"randomized footprint can overlap objects[{first}]")
        return self


def camera_declared_description(camera: CameraConfig) -> dict[str, object]:
    """Return a fresh mutable camera declaration with the original YAML values."""
    source = "<camera declaration>"
    if not isinstance(camera, CameraConfig):
        raise SceneConfigError(source, "<camera>", "must be a CameraConfig")
    field = f"cameras.{camera.name}"
    try:
        declaration = json.loads(camera.declared_description_json)
    except (TypeError, ValueError) as error:
        raise SceneConfigError(source, f"{field}.declared_description_json",
                               "must be a canonical JSON object") from error
    if not isinstance(declaration, dict):
        raise SceneConfigError(source, f"{field}.declared_description_json",
                               "must be a canonical JSON object")
    try:
        rebuilt = validate_config_model(CameraConfig, declaration, source, SceneConfigError)
    except SceneConfigError as error:
        raise SceneConfigError(source, f"{field}.{error.field}", error.message) from error
    if rebuilt != camera:
        mismatch = next(name for name in CameraConfig.model_fields
                        if getattr(rebuilt, name) != getattr(camera, name))
        raise SceneConfigError(source, f"{field}.{mismatch}",
                               "declared camera description does not match typed fields")
    return declaration


def load_scene_config(path: Path | str) -> SceneConfig:
    """Load and fully validate one scene YAML file."""
    source_path = Path(path)
    try:
        document = read_unique_yaml(source_path)
    except UniqueKeyError as error:
        raise SceneConfigError(error.source, error.field, error.message) from error
    if not isinstance(document, dict):
        raise SceneConfigError(source_path, "<root>", "must be a mapping")
    return validate_config_model(
        SceneConfig, {**document, "source": str(source_path)},
        source_path, SceneConfigError,
    )
