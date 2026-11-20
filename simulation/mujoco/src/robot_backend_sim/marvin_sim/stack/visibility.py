"""Layout enumeration and segmentation visibility checks for Marvin stack."""

from __future__ import annotations

import itertools
from collections.abc import Mapping
from pathlib import Path
from types import MappingProxyType
from typing import Annotated

import numpy as np
from pydantic import BaseModel, BeforeValidator, ConfigDict, Field, model_validator
from robot_contracts.validation import (
    PositiveInt,
    StrictFloat,
    StrictMappingError,
    UniqueKeyError,
    as_tuple,
    contract_error,
    read_unique_yaml,
    validate_config_model,
)

from robot_backend_sim.common.config.camera_config import CAMERA_NAMES
from robot_backend_sim.common.render.visibility import count_visible_pixels
from robot_backend_sim.marvin_sim.stack.scene_config import MarvinStackSceneConfig

__all__ = [
    "CameraVisibility",
    "LayoutSpec",
    "LayoutVisibility",
    "ObjectPlacement",
    "ObjectVisibility",
    "SceneCheckConfig",
    "SceneCheckConfigError",
    "ThroughputConfig",
    "build_layouts",
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
    schema_version: Annotated[int, Field(strict=True, ge=1, le=1)]
    seeds: Annotated[
        tuple[int, ...], BeforeValidator(as_tuple), Field(min_length=3, max_length=16)
    ]
    min_visible_pixels: Mapping[str, Mapping[str, PositiveInt]]
    throughput: ThroughputConfig
    camera_resolutions: Mapping[str, tuple[int, int]] = Field(default_factory=dict)

    @model_validator(mode="after")
    def _validate(self) -> SceneCheckConfig:
        if len(set(self.seeds)) != len(self.seeds):
            raise contract_error("seeds", "must not contain duplicates")
        if any(isinstance(seed, bool) or seed < 0 for seed in self.seeds):
            raise contract_error("seeds", "must contain non-negative integers")
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

    @property
    def boundary_layout_count(self) -> int:
        return 64

    @property
    def boundary_xy_corners(self) -> tuple[tuple[int, int], ...]:
        return ((-1, -1), (-1, 1), (1, -1), (1, 1))

    @property
    def boundary_yaw_signs(self) -> tuple[int, ...]:
        return (-1, 1)

    @property
    def max_layouts(self) -> int:
        return self.layout_count

    @property
    def layout_count(self) -> int:
        return len(self.seeds) + self.boundary_layout_count

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


class ObjectPlacement(_FrozenModel):
    name: str
    x: float
    y: float
    yaw_deg: float


class LayoutSpec(_FrozenModel):
    layout_id: str
    kind: str
    seed: int | None
    placements: tuple[ObjectPlacement, ...]


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


def _seed_placements(
    seed: int, scene_config: MarvinStackSceneConfig
) -> tuple[ObjectPlacement, ...]:
    rng = np.random.default_rng(seed)
    placements: list[ObjectPlacement] = []
    for obj in scene_config.objects:
        dx = (
            float(rng.uniform(-obj.randomization.xy_m, obj.randomization.xy_m))
            if obj.randomization.xy_m > 0.0
            else 0.0
        )
        dy = (
            float(rng.uniform(-obj.randomization.xy_m, obj.randomization.xy_m))
            if obj.randomization.xy_m > 0.0
            else 0.0
        )
        dyaw = (
            float(rng.uniform(-obj.randomization.yaw_deg, obj.randomization.yaw_deg))
            if obj.randomization.yaw_deg > 0.0
            else 0.0
        )
        placements.append(
            ObjectPlacement(
                name=obj.name,
                x=obj.nominal_xy[0] + dx,
                y=obj.nominal_xy[1] + dy,
                yaw_deg=obj.nominal_yaw_deg + dyaw,
            )
        )
    return tuple(placements)


def build_layouts(
    check_config: SceneCheckConfig, scene_config: MarvinStackSceneConfig
) -> tuple[LayoutSpec, ...]:
    layouts: list[LayoutSpec] = [
        LayoutSpec(
            layout_id=f"seed-{seed}",
            kind="seed",
            seed=seed,
            placements=_seed_placements(seed, scene_config),
        )
        for seed in check_config.seeds
    ]
    source_obj, target_obj = scene_config.objects
    source_corners = list(
        itertools.product(
            (-source_obj.randomization.xy_m, source_obj.randomization.xy_m),
            (-source_obj.randomization.xy_m, source_obj.randomization.xy_m),
            (-source_obj.randomization.yaw_deg, source_obj.randomization.yaw_deg),
        )
    )
    target_corners = list(
        itertools.product(
            (-target_obj.randomization.xy_m, target_obj.randomization.xy_m),
            (-target_obj.randomization.xy_m, target_obj.randomization.xy_m),
            (-target_obj.randomization.yaw_deg, target_obj.randomization.yaw_deg),
        )
    )
    index = 0
    for s_dx, s_dy, s_dyaw in source_corners:
        for t_dx, t_dy, t_dyaw in target_corners:
            layouts.append(
                LayoutSpec(
                    layout_id=f"boundary-{index:03d}",
                    kind="boundary",
                    seed=None,
                    placements=(
                        ObjectPlacement(
                            name=source_obj.name,
                            x=source_obj.nominal_xy[0] + s_dx,
                            y=source_obj.nominal_xy[1] + s_dy,
                            yaw_deg=source_obj.nominal_yaw_deg + s_dyaw,
                        ),
                        ObjectPlacement(
                            name=target_obj.name,
                            x=target_obj.nominal_xy[0] + t_dx,
                            y=target_obj.nominal_xy[1] + t_dy,
                            yaw_deg=target_obj.nominal_yaw_deg + t_dyaw,
                        ),
                    ),
                )
            )
            index += 1
    return tuple(layouts)


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
