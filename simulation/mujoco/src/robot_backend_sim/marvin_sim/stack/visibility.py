"""Segmentation visibility checks for the nominal Marvin stack layout."""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from types import MappingProxyType
from typing import Annotated

from pydantic import BaseModel, ConfigDict, Field, model_validator
from robot_contracts.validation import (
    PositiveInt,
    StrictFloat,
    StrictMappingError,
    UniqueKeyError,
    contract_error,
    read_unique_yaml,
    validate_config_model,
)

from robot_backend_sim.common.config.camera_config import CAMERA_NAMES
from robot_backend_sim.common.render.visibility import count_visible_pixels
from robot_backend_sim.marvin_sim.stack.scene_config import MarvinStackSceneConfig

__all__ = [
    "NOMINAL_LAYOUT",
    "CameraVisibility",
    "LayoutSpec",
    "LayoutVisibility",
    "ObjectVisibility",
    "SceneCheckConfig",
    "SceneCheckConfigError",
    "ThroughputConfig",
    "evaluate_layout_visibility",
    "load_check_config",
]

SceneCheckConfigError = StrictMappingError
_OBJECT_NAMES = ("source", "target")


class _FrozenModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class ThroughputConfig(_FrozenModel):
    warmup_groups: Annotated[int, Field(strict=True, ge=1, le=50)]
    measure_groups: Annotated[int, Field(strict=True, ge=5, le=200)]
    max_elapsed_s: Annotated[StrictFloat, Field(gt=0.0, le=300.0)]
    min_fps: Annotated[StrictFloat, Field(gt=0.0, le=10000.0)]


class SceneCheckConfig(_FrozenModel):
    source: str = ""
    min_visible_pixels: Mapping[str, Mapping[str, PositiveInt]]
    throughput: ThroughputConfig
    camera_resolutions: Mapping[str, tuple[int, int]] = Field(default_factory=dict)

    @model_validator(mode="after")
    def _validate(self) -> SceneCheckConfig:
        if tuple(self.min_visible_pixels.keys()) != CAMERA_NAMES:
            raise contract_error("min_visible_pixels", f"must contain {CAMERA_NAMES}")
        frozen_pixels: dict[str, Mapping[str, int]] = {}
        for camera in CAMERA_NAMES:
            section = self.min_visible_pixels[camera]
            if tuple(section.keys()) != _OBJECT_NAMES:
                raise contract_error(
                    f"min_visible_pixels.{camera}", f"must contain {_OBJECT_NAMES}"
                )
            frozen_pixels[camera] = MappingProxyType(dict(section))
        object.__setattr__(self, "min_visible_pixels", MappingProxyType(frozen_pixels))
        object.__setattr__(
            self, "camera_resolutions", MappingProxyType(dict(self.camera_resolutions))
        )
        return self

    @property
    def camera_names(self) -> tuple[str, ...]:
        return CAMERA_NAMES

    @property
    def object_names(self) -> tuple[str, ...]:
        return _OBJECT_NAMES

    def threshold(self, camera: str, object_name: str) -> int:
        return int(self.min_visible_pixels[camera][object_name])


def load_check_config(
    path: Path | str, scene_config: MarvinStackSceneConfig
) -> SceneCheckConfig:
    source_path = Path(path).resolve()
    try:
        document = read_unique_yaml(source_path)
    except OSError as error:
        raise SceneCheckConfigError(source_path, "<file>", str(error)) from error
    except UniqueKeyError as error:
        raise SceneCheckConfigError(error.source, error.field, error.message) from error
    if not isinstance(document, dict):
        raise SceneCheckConfigError(source_path, "<root>", "must be a mapping")
    resolutions = {
        camera.name: (camera.width, camera.height) for camera in scene_config.cameras
    }
    config = validate_config_model(
        SceneCheckConfig,
        {**document, "source": str(source_path), "camera_resolutions": resolutions},
        source_path,
        SceneCheckConfigError,
    )
    for camera, (width, height) in resolutions.items():
        limit = width * height
        for obj in _OBJECT_NAMES:
            threshold = config.threshold(camera, obj)
            if threshold > limit:
                raise SceneCheckConfigError(
                    source_path,
                    f"min_visible_pixels.{camera}.{obj}",
                    f"threshold {threshold} exceeds pixel count {limit}",
                )
    return config


class LayoutSpec(_FrozenModel):
    layout_id: str
    kind: str
    seed: int


NOMINAL_LAYOUT = LayoutSpec(layout_id="nominal", kind="nominal", seed=0)


class ObjectVisibility(_FrozenModel):
    object: str
    geom_id: int
    pixels: int
    threshold: int
    passed: bool


class CameraVisibility(_FrozenModel):
    camera: str
    width: int
    height: int
    total_pixels: int
    passed: bool
    objects: tuple[ObjectVisibility, ...]


class LayoutVisibility(_FrozenModel):
    layout_id: str
    cameras: tuple[CameraVisibility, ...]
    total_pixels: int
    minimum_margin: int
    passed: bool


def evaluate_layout_visibility(
    layout_id: str,
    segmentations: Mapping[str, object],
    geom_ids: Mapping[str, int],
    check_config: SceneCheckConfig,
) -> LayoutVisibility:
    cameras: list[CameraVisibility] = []
    total_pixels = 0
    minimum_margin: int | None = None
    for camera in check_config.camera_names:
        seg = segmentations[camera]
        width, height = check_config.camera_resolutions[camera]
        object_items: list[ObjectVisibility] = []
        camera_total = 0
        for obj in check_config.object_names:
            geom_id = int(geom_ids[obj])
            pixels = count_visible_pixels(seg, geom_id)
            threshold = check_config.threshold(camera, obj)
            margin = pixels - threshold
            minimum_margin = margin if minimum_margin is None else min(minimum_margin, margin)
            camera_total += pixels
            object_items.append(
                ObjectVisibility(
                    object=obj,
                    geom_id=geom_id,
                    pixels=pixels,
                    threshold=threshold,
                    passed=pixels >= threshold,
                )
            )
        total_pixels += camera_total
        cameras.append(
            CameraVisibility(
                camera=camera,
                width=width,
                height=height,
                total_pixels=camera_total,
                passed=all(item.passed for item in object_items),
                objects=tuple(object_items),
            )
        )
    assert minimum_margin is not None
    return LayoutVisibility(
        layout_id=layout_id,
        cameras=tuple(cameras),
        total_pixels=total_pixels,
        minimum_margin=minimum_margin,
        passed=all(camera.passed for camera in cameras),
    )
