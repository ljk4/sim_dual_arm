"""Fixed extrinsic RGB camera declarations shared by simulation scenes.

The declaration is the calibration identity: the profile stores the SHA-256 of
the exact name/parent/pose/lens/resolution fields, and every scene hashes the
same way.  Scene-specific configuration decides which body each camera is
attached to.
"""

from __future__ import annotations

import json
import math
from typing import Annotated

from pydantic import BeforeValidator, Field, model_validator
from robot_contracts.validation import (
    NonEmptyStr,
    PositiveFloat,
    PositiveInt,
    StrictFloat,
    StrictMappingError,
    StrictMappingModel,
    as_tuple,
    contract_error,
    validate_config_model,
)

__all__ = [
    "CAMERA_FIELDS",
    "CameraConfig",
    "CameraConfigError",
    "camera_declared_description",
]

CAMERA_FIELDS = ("name", "parent", "position", "quaternion_wxyz", "fovy", "width", "height")
CameraConfigError = StrictMappingError

Unit = Annotated[StrictFloat, Field(ge=0.0, le=1.0)]
Vec3 = Annotated[tuple[StrictFloat, StrictFloat, StrictFloat], BeforeValidator(as_tuple)]
Quaternion = Annotated[
    tuple[StrictFloat, StrictFloat, StrictFloat, StrictFloat], BeforeValidator(as_tuple)
]


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
        norm = math.sqrt(sum(component * component for component in self.quaternion_wxyz))
        if not math.isfinite(norm) or norm < 1e-9:
            raise contract_error("quaternion_wxyz", "must be a non-zero finite quaternion")
        if abs(norm - 1.0) > 1e-3:
            raise contract_error("quaternion_wxyz", "must be a unit quaternion")
        object.__setattr__(
            self,
            "quaternion_wxyz",
            tuple(component / norm for component in self.quaternion_wxyz),
        )
        return self


def camera_declared_description(camera: CameraConfig) -> dict[str, object]:
    """Return a fresh mutable camera declaration with the original YAML values."""
    source = "<camera declaration>"
    if not isinstance(camera, CameraConfig):
        raise CameraConfigError(source, "<camera>", "must be a CameraConfig")
    field = f"cameras.{camera.name}"
    try:
        declaration = json.loads(camera.declared_description_json)
    except (TypeError, ValueError) as error:
        raise CameraConfigError(
            source, f"{field}.declared_description_json", "must be a canonical JSON object"
        ) from error
    if not isinstance(declaration, dict):
        raise CameraConfigError(
            source, f"{field}.declared_description_json", "must be a canonical JSON object"
        )
    try:
        rebuilt = validate_config_model(CameraConfig, declaration, source, CameraConfigError)
    except CameraConfigError as error:
        raise CameraConfigError(source, f"{field}.{error.field}", error.message) from error
    if rebuilt != camera:
        mismatch = next(
            name
            for name in CameraConfig.model_fields
            if getattr(rebuilt, name) != getattr(camera, name)
        )
        raise CameraConfigError(
            source, f"{field}.{mismatch}", "declared camera description does not match typed fields"
        )
    return declaration
