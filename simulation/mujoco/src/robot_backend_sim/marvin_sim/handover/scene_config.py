"""Strict configuration for the fixed-base Marvin handover task scene.

The file selects one robot assembly and one task environment independently.
Numeric robot values are executable ranges and servo gains; geometry, mass and
joint limits stay in the source URDF and are verified against the compiled
model.  Task values describe the table, the carried block and the open box.
"""

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
    CAMERA_NAMES,
    CameraConfig,
    Vec3,
)
from robot_backend_sim.common.physics.assets import robot_model_path
from robot_backend_sim.marvin_sim.robot_config import MarvinRobotConfig
from robot_backend_sim.marvin_sim.scene_base import (
    CAMERA_PARENTS,
    CameraMountConfig,
    PhysicsConfig,
    Rgba,
    TableConfig,
    Vec2,
)

__all__ = [
    "CAMERA_PARENTS",
    "BoxConfig",
    "BlockConfig",
    "MarvinSceneConfig",
    "MarvinSceneConfigError",
    "PhysicsConfig",
    "TableConfig",
    "TaskConfig",
    "load_marvin_scene_config",
]

MarvinSceneConfigError = StrictMappingError
_OBJECT_NAME_PATTERN = r"^[A-Za-z][A-Za-z0-9_]*$"
_RESERVED_BODY_NAMES = frozenset(
    {"world", "table", "box", "camera_mount", "ZJ_Robot_link"}
)


class BlockConfig(StrictMappingModel):
    name: Annotated[NonEmptyStr, Field(pattern=_OBJECT_NAME_PATTERN)]
    half_extents: Vec3
    initial_xy: Vec2
    initial_yaw_deg: Annotated[StrictFloat, Field(ge=-180.0, le=180.0)]
    mass: Annotated[PositiveFloat, Field(ge=0.001, le=10.0)]
    rgba: Rgba

    @model_validator(mode="after")
    def _block(self) -> BlockConfig:
        if self.name in _RESERVED_BODY_NAMES:
            raise contract_error("name", "is a reserved scene body")
        for index, value in enumerate(self.half_extents):
            if not 0.005 <= value <= 0.25:
                raise contract_error(
                    f"half_extents[{index}]", "must be between 0.005 and 0.25"
                )
        return self


class BoxConfig(StrictMappingModel):
    name: Annotated[NonEmptyStr, Field(pattern=_OBJECT_NAME_PATTERN)]
    center_xy: Vec2
    outer_xy: Vec2
    height: Annotated[PositiveFloat, Field(ge=0.02, le=1.0)]
    bottom_thickness: Annotated[PositiveFloat, Field(ge=0.001, le=0.2)]
    wall_thickness: Annotated[PositiveFloat, Field(ge=0.001, le=0.2)]
    yaw_deg: Annotated[StrictFloat, Field(ge=-180.0, le=180.0)]
    rgba: Rgba

    @model_validator(mode="after")
    def _box(self) -> BoxConfig:
        for index, value in enumerate(self.outer_xy):
            if not 0.05 <= value <= 2.0:
                raise contract_error(f"outer_xy[{index}]", "must be between 0.05 and 2.0")
            if not 2.0 * self.wall_thickness < value:
                raise contract_error(
                    f"outer_xy[{index}]", "must exceed twice the wall thickness"
                )
        if not self.bottom_thickness < self.height:
            raise contract_error("bottom_thickness", "must be smaller than height")
        return self

    @property
    def inner_xy(self) -> tuple[float, float]:
        return (
            self.outer_xy[0] - 2.0 * self.wall_thickness,
            self.outer_xy[1] - 2.0 * self.wall_thickness,
        )

    @property
    def inner_height(self) -> float:
        return self.height - self.bottom_thickness


class TaskConfig(StrictMappingModel):
    environment: Literal["handover"]
    frame: Literal["robot_base"]
    table: TableConfig
    block: BlockConfig
    box: BoxConfig
    camera_mount: CameraMountConfig

    @model_validator(mode="after")
    def _task(self) -> TaskConfig:
        inner_x, inner_y = self.box.inner_xy
        block_x, block_y, block_z = (2.0 * value for value in self.block.half_extents)
        if block_x > inner_x + 1e-9 or block_y > inner_y + 1e-9:
            raise contract_error(
                "box.outer_xy", "open box interior cannot contain the block footprint"
            )
        if block_z > self.box.inner_height + 1e-9:
            raise contract_error(
                "box.height", "open box interior cannot contain the block height"
            )
        table_half_x = self.table.size_xy[0] / 2.0
        table_half_y = self.table.size_xy[1] / 2.0
        for name, center, half in (
            (
                "block.initial_xy",
                self.block.initial_xy,
                math.hypot(*self.block.half_extents[:2]),
            ),
            (
                "box.center_xy",
                self.box.center_xy,
                math.hypot(self.box.outer_xy[0], self.box.outer_xy[1]) / 2.0,
            ),
        ):
            if (
                abs(center[0] - self.table.center_xy[0]) + half > table_half_x + 1e-9
                or abs(center[1] - self.table.center_xy[1]) + half > table_half_y + 1e-9
            ):
                raise contract_error(name, "footprint crosses the table edge")
        return self


class MarvinSceneConfig(StrictMappingModel):
    source: str = ""
    physics: PhysicsConfig
    robot: MarvinRobotConfig
    task: TaskConfig
    cameras: Annotated[
        tuple[CameraConfig, ...], BeforeValidator(as_tuple), Field(min_length=3, max_length=3)
    ]
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
    def _scene(self) -> MarvinSceneConfig:
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


def load_marvin_scene_config(path: Path | str) -> MarvinSceneConfig:
    """Load one scene YAML and resolve the model selected by ``robot.assembly``."""
    source_path = Path(path).resolve()
    try:
        document = read_unique_yaml(source_path)
    except UniqueKeyError as error:
        raise MarvinSceneConfigError(error.source, error.field, error.message) from error
    if not isinstance(document, dict):
        raise MarvinSceneConfigError(source_path, "<root>", "must be a mapping")
    config = validate_config_model(
        MarvinSceneConfig,
        {**document, "source": str(source_path)},
        source_path,
        MarvinSceneConfigError,
    )
    try:
        urdf_path = robot_model_path(config.robot.assembly)
    except KeyError as error:
        raise MarvinSceneConfigError(
            source_path, "robot.assembly", str(error)
        ) from error
    if not urdf_path.is_file():
        raise MarvinSceneConfigError(
            source_path, "robot.assembly", f"model asset is missing: {urdf_path}"
        )
    return config.model_copy(update={"urdf_path": urdf_path})
