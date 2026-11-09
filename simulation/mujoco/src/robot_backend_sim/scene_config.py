"""Frozen configuration for the dual-Panda stack scene."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Annotated

from pydantic import BeforeValidator, Field, model_validator
from robot_contracts.validation import (
    StrictMappingError,
    StrictMappingModel,
    UniqueKeyError,
    as_tuple,
    contract_error,
    read_unique_yaml,
    validate_config_model,
)

from .scene_config_sections import (
    CAMERA_NAMES,
    CAMERA_PARENTS,
    FINGER_JOINT_LIMIT,
    INTEGRATORS,
    MAX_RANDOM_XY,
    MAX_RANDOM_YAW_DEG,
    PANDA_JOINT_RANGES,
    CameraConfig,
    ColumnConfig,
    MountsConfig,
    ObjectConfig,
    PhysicsConfig,
    RandomizationConfig,
    ResetConfig,
    TableConfig,
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
