"""Strict configuration for the fixed-base Marvin stack task scene."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Annotated, Literal

from pydantic import BeforeValidator, Field, model_validator
from robot_contracts.validation import (
    NonEmptyStr,
    PositiveFloat,
    StrictFloat,
    StrictMappingError,
    StrictMappingModel,
    UniqueKeyError,
    as_tuple,
    contract_error,
    read_unique_yaml,
    validate_config_model,
)

from robot_backend_sim.common.config.camera_config import (
    CameraConfig,
    Vec3,
    camera_declared_description,
)
from robot_backend_sim.common.physics.assets import robot_model_path
from robot_backend_sim.marvin.robot_config import (
    MARVIN_CAMERA_NAMES as CAMERA_NAMES,
    MarvinRobotConfig,
)
from robot_backend_sim.marvin.scene_base import (
    CAMERA_PARENTS,
    CameraMountConfig,
    DomainRandomizationConfig,
    PhysicsConfig,
    Rgba,
    TableConfig,
    Vec2,
)

__all__ = [
    "CAMERA_PARENTS",
    "MarvinStackSceneConfig",
    "ObjectConfig",
    "PhysicsConfig",
    "StackEnvironmentConfig",
    "StackSceneConfigError",
    "TableConfig",
    "camera_declared_description",
    "load_marvin_stack_scene_config",
]

StackSceneConfigError = StrictMappingError
_OBJECT_NAME_PATTERN = r"^[A-Za-z][A-Za-z0-9_]*$"


class ObjectConfig(StrictMappingModel):
    name: Annotated[NonEmptyStr, Field(pattern=_OBJECT_NAME_PATTERN)]
    half_extents: Vec3
    nominal_xy: Vec2
    nominal_yaw_deg: Annotated[StrictFloat, Field(ge=-180.0, le=180.0)]
    mass: Annotated[PositiveFloat, Field(ge=0.001, le=10.0)]
    rgba: Rgba

    @model_validator(mode="after")
    def _object(self) -> ObjectConfig:
        for index, value in enumerate(self.half_extents):
            if not 0.005 <= value <= 0.25:
                raise contract_error(
                    f"half_extents[{index}]", "must be between 0.005 and 0.25"
                )
        return self


class StackEnvironmentConfig(StrictMappingModel):
    environment: Literal["stack"]
    frame: Literal["robot_base"]
    table: TableConfig
    source: ObjectConfig
    target: ObjectConfig
    camera_mount: CameraMountConfig

    @model_validator(mode="after")
    def _environment(self) -> StackEnvironmentConfig:
        if self.source.name != "source":
            raise contract_error("source.name", "must be 'source'")
        if self.target.name != "target":
            raise contract_error("target.name", "must be 'target'")
        table_half_x = self.table.size_xy[0] / 2.0
        table_half_y = self.table.size_xy[1] / 2.0
        for field_name, obj in (("source", self.source), ("target", self.target)):
            radius = math.hypot(*obj.half_extents[:2])
            reach_x = abs(obj.nominal_xy[0] - self.table.center_xy[0]) + radius
            reach_y = abs(obj.nominal_xy[1] - self.table.center_xy[1]) + radius
            if reach_x > table_half_x + 1e-9 or reach_y > table_half_y + 1e-9:
                raise contract_error(
                    f"{field_name}.nominal_xy",
                    "footprint crosses the table edge",
                )
        center_distance = math.dist(self.source.nominal_xy, self.target.nominal_xy)
        min_required = (
            math.hypot(*self.source.half_extents[:2])
            + math.hypot(*self.target.half_extents[:2])
            + 0.005
        )
        if center_distance < min_required:
            raise contract_error(
                "source.nominal_xy",
                "source and target footprints can overlap or lack clearance",
            )
        return self


class MarvinStackSceneConfig(StrictMappingModel):
    source: str = ""
    physics: PhysicsConfig
    robot: MarvinRobotConfig
    task: StackEnvironmentConfig
    cameras: Annotated[
        tuple[CameraConfig, ...],
        BeforeValidator(as_tuple),
        Field(min_length=3, max_length=3),
    ]
    domain_randomization: DomainRandomizationConfig = Field(
        default_factory=DomainRandomizationConfig
    )
    urdf_path: Path | None = Field(default=None, exclude=True)

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
    def _scene(self) -> MarvinStackSceneConfig:
        if tuple(camera.name for camera in self.cameras) != CAMERA_NAMES:
            raise contract_error("cameras", f"must be exactly {CAMERA_NAMES}")
        for index, camera in enumerate(self.cameras):
            expected = CAMERA_PARENTS[camera.name]
            if camera.parent != expected:
                raise contract_error(
                    f"cameras[{index}].parent", f"must be {expected!r}"
                )
        if (
            len({camera.width for camera in self.cameras}) != 1
            or len({camera.height for camera in self.cameras}) != 1
        ):
            raise contract_error("cameras", "all cameras must share one resolution")
        return self

    @property
    def table(self) -> TableConfig:
        return self.task.table

    @property
    def objects(self) -> tuple[ObjectConfig, ObjectConfig]:
        return (self.task.source, self.task.target)

    @property
    def reset(self) -> MarvinRobotConfig:
        return self.robot


def load_marvin_stack_scene_config(path: Path | str) -> MarvinStackSceneConfig:
    """Load one Marvin stack scene YAML and resolve the selected URDF path."""
    source_path = Path(path).resolve()
    try:
        document = read_unique_yaml(source_path)
    except OSError as error:
        raise StackSceneConfigError(source_path, "<file>", str(error)) from error
    except UniqueKeyError as error:
        raise StackSceneConfigError(error.source, error.field, error.message) from error
    if not isinstance(document, dict):
        raise StackSceneConfigError(source_path, "<root>", "must be a mapping")
    config = validate_config_model(
        MarvinStackSceneConfig,
        {**document, "source": str(source_path)},
        source_path,
        StackSceneConfigError,
    )
    try:
        urdf_path = robot_model_path(config.robot.assembly)
    except KeyError as error:
        raise StackSceneConfigError(
            source_path, "robot.assembly", str(error)
        ) from error
    if not urdf_path.is_file():
        raise StackSceneConfigError(
            source_path, "robot.assembly", f"model asset is missing: {urdf_path}"
        )
    return config.model_copy(update={"urdf_path": urdf_path})
