"""Typed configuration, boundary layouts and segmentation visibility counts.

This module deliberately has no MuJoCo dependency: the check configuration and
the 64 joint boundary layouts can be rejected or inspected before an EGL
context exists.  Rendering and applying a layout to a snapshot stay in
``rendering``/``check_scene``; nothing here moves objects in the live scene.
"""

from __future__ import annotations

import itertools
import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any

import yaml

from .scene_config import ObjectConfig, SceneConfig

__all__ = [
    "GEOM_OBJECT_TYPE",
    "CameraVisibility",
    "LayoutSpec",
    "LayoutVisibility",
    "ObjectPlacement",
    "ObjectVisibility",
    "SceneCheckConfig",
    "SceneCheckConfigError",
    "ThroughputConfig",
    "build_layouts",
    "count_visible_pixels",
    "load_check_config",
    "summarize_layout",
    "yaw_quaternion",
]

# mjtObj.mjOBJ_GEOM is 5 in the fixed MuJoCo SDK.  Keeping the value here lets
# the pure-python checks run on a host without MuJoCo.
GEOM_OBJECT_TYPE = 5
# MuJoCo's Renderer returns a segmentation image whose last axis is
# (objid, objtype); column order is part of the SDK contract, not a guess.
SEGMENTATION_OBJID_CHANNEL = 0
SEGMENTATION_OBJTYPE_CHANNEL = 1
MAX_LAYOUTS = 100
MAX_PIXELS = 4096 * 4096
_TOP_LEVEL_KEYS = (
    "schema_version",
    "seeds",
    "boundary_xy_corners",
    "boundary_yaw_signs",
    "min_visible_pixels",
    "max_layouts",
    "throughput",
)
_BOUNDARY_CORNERS = 4
_BOUNDARY_YAW_SIGNS = 2
_PER_OBJECT_BOUNDARY_LAYOUTS = _BOUNDARY_CORNERS * _BOUNDARY_YAW_SIGNS
_SCHEMA_VERSION = 1


class SceneCheckConfigError(ValueError):
    """A scene-check configuration problem with source file and field attached."""

    def __init__(self, source: str | Path, field: str, message: str) -> None:
        self.source = str(source)
        self.field = field
        self.message = message
        super().__init__(f"{self.source}: {field}: {message}")


class VisibilityError(ValueError):
    """A segmentation image or visibility input is not usable."""


class _UniqueKeyLoader(yaml.SafeLoader):
    source_name = "<scene check config>"


def _construct_unique_mapping(
    loader: _UniqueKeyLoader, node: yaml.MappingNode, deep: bool = False
) -> dict[Any, Any]:
    loader.flatten_mapping(node)
    mapping: dict[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            duplicate = key in mapping
        except TypeError as error:
            raise SceneCheckConfigError(
                loader.source_name, "<mapping>", "keys must be hashable"
            ) from error
        if duplicate:
            raise SceneCheckConfigError(loader.source_name, str(key), "duplicate key")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping
)


@dataclass(frozen=True, slots=True)
class ThroughputConfig:
    warmup_groups: int
    measure_groups: int


@dataclass(frozen=True, slots=True)
class SceneCheckConfig:
    source: str
    schema_version: int
    seeds: tuple[int, ...]
    boundary_xy_corners: tuple[tuple[int, int], ...]
    boundary_yaw_signs: tuple[int, ...]
    min_visible_pixels: Mapping[str, Mapping[str, int]]
    max_layouts: int
    throughput: ThroughputConfig
    camera_names: tuple[str, ...]
    object_names: tuple[str, ...]
    camera_resolutions: Mapping[str, tuple[int, int]]

    @property
    def boundary_layout_count(self) -> int:
        return _PER_OBJECT_BOUNDARY_LAYOUTS ** len(self.object_names)

    @property
    def layout_count(self) -> int:
        return len(self.seeds) + self.boundary_layout_count

    def threshold(self, camera: str, obj: str) -> int:
        return self.min_visible_pixels[camera][obj]


@dataclass(frozen=True, slots=True)
class ObjectPlacement:
    name: str
    x: float
    y: float
    yaw_deg: float


@dataclass(frozen=True, slots=True)
class LayoutSpec:
    layout_id: str
    kind: str
    seed: int
    placements: tuple[ObjectPlacement, ...]


@dataclass(frozen=True, slots=True)
class ObjectVisibility:
    object: str
    geom_id: int
    pixels: int
    threshold: int
    passed: bool


@dataclass(frozen=True, slots=True)
class CameraVisibility:
    camera: str
    width: int
    height: int
    objects: tuple[ObjectVisibility, ...]
    total_pixels: int
    passed: bool


@dataclass(frozen=True, slots=True)
class LayoutVisibility:
    layout_id: str
    kind: str
    cameras: tuple[CameraVisibility, ...]
    total_pixels: int
    minimum_margin: int
    passed: bool


def _field(parent: str, child: str) -> str:
    return f"{parent}.{child}" if parent else child


def _mapping(value: object, source: str, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise SceneCheckConfigError(source, field, "must be a mapping")
    for key in value:
        if not isinstance(key, str):
            raise SceneCheckConfigError(source, field, "keys must be strings")
    return value


def _check_keys(
    document: dict[str, Any],
    source: str,
    field: str,
    required: Sequence[str],
) -> None:
    allowed = set(required)
    for key in document:
        if key not in allowed:
            raise SceneCheckConfigError(source, _field(field, key), "unknown key")
    for key in required:
        if key not in document:
            raise SceneCheckConfigError(source, _field(field, key), "missing required key")


def _integer(
    value: object,
    source: str,
    field: str,
    *,
    minimum: int,
    maximum: int,
) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise SceneCheckConfigError(source, field, "must be an integer; booleans are not accepted")
    if value < minimum or value > maximum:
        raise SceneCheckConfigError(source, field, f"must be between {minimum} and {maximum}")
    return value


def _list(value: object, source: str, field: str) -> list[Any]:
    if not isinstance(value, list):
        raise SceneCheckConfigError(source, field, "must be a list")
    return value


def _parse_seeds(value: object, source: str) -> tuple[int, ...]:
    field = "seeds"
    entries = _list(value, source, field)
    if not entries:
        raise SceneCheckConfigError(source, field, "must contain at least one seed")
    seeds: list[int] = []
    for index, entry in enumerate(entries):
        seed = _integer(entry, source, f"{field}[{index}]", minimum=0, maximum=2_147_483_647)
        if seed in seeds:
            raise SceneCheckConfigError(source, f"{field}[{index}]", "duplicate seed")
        seeds.append(seed)
    return tuple(seeds)


def _parse_corners(value: object, source: str) -> tuple[tuple[int, int], ...]:
    field = "boundary_xy_corners"
    entries = _list(value, source, field)
    if len(entries) != _BOUNDARY_CORNERS:
        raise SceneCheckConfigError(
            source, field, f"must contain exactly {_BOUNDARY_CORNERS} XY corners"
        )
    corners: list[tuple[int, int]] = []
    for index, entry in enumerate(entries):
        item_field = f"{field}[{index}]"
        vector = _list(entry, source, item_field)
        if len(vector) != 2:
            raise SceneCheckConfigError(source, item_field, "must be a list of two signs")
        pair = tuple(
            _integer(component, source, f"{item_field}[{axis}]", minimum=-1, maximum=1)
            for axis, component in enumerate(vector)
        )
        if pair[0] not in (-1, 1) or pair[1] not in (-1, 1):
            raise SceneCheckConfigError(source, item_field, "each sign must be -1 or 1")
        if pair in corners:
            raise SceneCheckConfigError(source, item_field, "duplicate XY corner")
        corners.append(pair)  # type: ignore[arg-type]
    return tuple(corners)


def _parse_yaw_signs(value: object, source: str) -> tuple[int, ...]:
    field = "boundary_yaw_signs"
    entries = _list(value, source, field)
    if len(entries) != _BOUNDARY_YAW_SIGNS:
        raise SceneCheckConfigError(
            source, field, f"must contain exactly {_BOUNDARY_YAW_SIGNS} signs"
        )
    signs: list[int] = []
    for index, entry in enumerate(entries):
        sign = _integer(entry, source, f"{field}[{index}]", minimum=-1, maximum=1)
        if sign not in (-1, 1):
            raise SceneCheckConfigError(source, f"{field}[{index}]", "must be -1 or 1")
        if sign in signs:
            raise SceneCheckConfigError(source, f"{field}[{index}]", "duplicate yaw sign")
        signs.append(sign)
    return tuple(signs)


def _parse_throughput(value: object, source: str) -> ThroughputConfig:
    field = "throughput"
    document = _mapping(value, source, field)
    _check_keys(document, source, field, ("warmup_groups", "measure_groups"))
    warmup = _integer(
        document["warmup_groups"],
        source,
        f"{field}.warmup_groups",
        minimum=0,
        maximum=1000,
    )
    measure = _integer(
        document["measure_groups"],
        source,
        f"{field}.measure_groups",
        minimum=1,
        maximum=10000,
    )
    return ThroughputConfig(warmup, measure)


def _parse_thresholds(
    value: object,
    source: str,
    scene_config: SceneConfig,
) -> Mapping[str, Mapping[str, int]]:
    field = "min_visible_pixels"
    document = _mapping(value, source, field)
    camera_names = tuple(camera.name for camera in scene_config.cameras)
    object_names = tuple(obj.name for obj in scene_config.objects)
    if set(document) != set(camera_names):
        missing = sorted(set(camera_names) - set(document))
        unknown = sorted(set(document) - set(camera_names))
        raise SceneCheckConfigError(
            source,
            field,
            f"camera keys must be exactly {list(camera_names)}; "
            f"missing={missing}, unknown={unknown}",
        )
    resolutions = {camera.name: (camera.width, camera.height) for camera in scene_config.cameras}
    result: dict[str, Mapping[str, int]] = {}
    for camera in camera_names:
        item_field = f"{field}.{camera}"
        entries = _mapping(document[camera], source, item_field)
        if set(entries) != set(object_names):
            missing = sorted(set(object_names) - set(entries))
            unknown = sorted(set(entries) - set(object_names))
            raise SceneCheckConfigError(
                source,
                item_field,
                f"object keys must be exactly {list(object_names)}; "
                f"missing={missing}, unknown={unknown}",
            )
        width, height = resolutions[camera]
        upper = width * height
        thresholds: dict[str, int] = {}
        for obj in object_names:
            threshold = _integer(
                entries[obj],
                source,
                f"{item_field}.{obj}",
                minimum=1,
                maximum=upper,
            )
            thresholds[obj] = threshold
        result[camera] = MappingProxyType(thresholds)
    return MappingProxyType(result)


def load_check_config(path: Path | str, scene_config: SceneConfig) -> SceneCheckConfig:
    """Load, cross-check against the scene and freeze one check configuration."""
    source_path = Path(path)
    source = str(source_path)
    try:
        text = source_path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise SceneCheckConfigError(
            source, "<file>", f"cannot read configuration: {error}"
        ) from error
    loader = _UniqueKeyLoader(text)
    loader.source_name = source
    try:
        raw = loader.get_single_data()
    except SceneCheckConfigError:
        raise
    except yaml.YAMLError as error:
        raise SceneCheckConfigError(source, "<yaml>", f"invalid YAML: {error}") from error
    finally:
        loader.dispose()
    root = _mapping(raw, source, "<root>")
    _check_keys(root, source, "", _TOP_LEVEL_KEYS)
    schema_version = _integer(
        root["schema_version"], source, "schema_version", minimum=1, maximum=1
    )
    seeds = _parse_seeds(root["seeds"], source)
    corners = _parse_corners(root["boundary_xy_corners"], source)
    yaw_signs = _parse_yaw_signs(root["boundary_yaw_signs"], source)
    thresholds = _parse_thresholds(root["min_visible_pixels"], source, scene_config)
    max_layouts = _integer(
        root["max_layouts"], source, "max_layouts", minimum=1, maximum=MAX_LAYOUTS
    )
    throughput = _parse_throughput(root["throughput"], source)
    camera_names = tuple(camera.name for camera in scene_config.cameras)
    object_names = tuple(obj.name for obj in scene_config.objects)
    resolutions = MappingProxyType(
        {camera.name: (camera.width, camera.height) for camera in scene_config.cameras}
    )
    config = SceneCheckConfig(
        source=source,
        schema_version=schema_version,
        seeds=seeds,
        boundary_xy_corners=corners,
        boundary_yaw_signs=yaw_signs,
        min_visible_pixels=thresholds,
        max_layouts=max_layouts,
        throughput=throughput,
        camera_names=camera_names,
        object_names=object_names,
        camera_resolutions=resolutions,
    )
    if config.boundary_layout_count != _PER_OBJECT_BOUNDARY_LAYOUTS ** len(object_names):
        raise SceneCheckConfigError(source, "boundary_xy_corners", "invalid boundary layout count")
    if config.layout_count > config.max_layouts:
        raise SceneCheckConfigError(
            source,
            "max_layouts",
            f"{config.layout_count} configured layouts exceed max_layouts={config.max_layouts}",
        )
    return config


def _object_combinations(
    config: SceneCheckConfig, obj: ObjectConfig
) -> tuple[ObjectPlacement, ...]:
    combinations: list[ObjectPlacement] = []
    for sign_x, sign_y in config.boundary_xy_corners:
        for yaw_sign in config.boundary_yaw_signs:
            combinations.append(
                ObjectPlacement(
                    name=obj.name,
                    x=obj.nominal_xy[0] + sign_x * obj.randomization.xy_m,
                    y=obj.nominal_xy[1] + sign_y * obj.randomization.xy_m,
                    yaw_deg=yaw_sign * obj.randomization.yaw_deg,
                )
            )
    return tuple(combinations)


def build_layouts(config: SceneCheckConfig, scene_config: SceneConfig) -> tuple[LayoutSpec, ...]:
    """Return normal seed layouts followed by the joint 8x8 boundary layouts."""
    objects = tuple(scene_config.objects)
    if tuple(obj.name for obj in objects) != config.object_names:
        raise SceneCheckConfigError(
            config.source, "min_visible_pixels", "object names do not match the scene config"
        )
    layouts: list[LayoutSpec] = [
        LayoutSpec(layout_id=f"seed-{seed}", kind="seed", seed=seed, placements=())
        for seed in config.seeds
    ]
    per_object = [_object_combinations(config, obj) for obj in objects]
    boundary_base_seed = config.seeds[0]
    for index, joint in enumerate(itertools.product(*per_object)):
        layouts.append(
            LayoutSpec(
                layout_id=f"boundary-{index:03d}",
                kind="boundary",
                seed=boundary_base_seed,
                placements=tuple(joint),
            )
        )
    if len(layouts) > config.max_layouts:
        raise SceneCheckConfigError(
            config.source,
            "max_layouts",
            f"{len(layouts)} layouts exceed max_layouts={config.max_layouts}",
        )
    return tuple(layouts)


def yaw_quaternion(yaw_deg: float) -> tuple[float, float, float, float]:
    """Return the world Z yaw quaternion in MuJoCo's wxyz order."""
    if isinstance(yaw_deg, bool) or not isinstance(yaw_deg, int | float):
        raise VisibilityError("yaw_deg must be a finite number")
    yaw = math.radians(float(yaw_deg))
    if not math.isfinite(yaw):
        raise VisibilityError("yaw_deg must be finite")
    half = yaw / 2.0
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def _shape(segmentation: object) -> tuple[int, int, int]:
    shape = getattr(segmentation, "shape", None)
    if shape is not None:
        try:
            dimensions = tuple(int(value) for value in shape)
        except (TypeError, ValueError) as error:
            raise VisibilityError("segmentation shape is not readable") from error
    else:
        try:
            dimensions = (
                len(segmentation),  # type: ignore[arg-type]
                len(segmentation[0]),  # type: ignore[index]
                len(segmentation[0][0]),  # type: ignore[index]
            )
        except (TypeError, IndexError) as error:
            raise VisibilityError("segmentation must be a 3-dimensional image") from error
    if len(dimensions) != 3 or dimensions[2] != 2:
        raise VisibilityError(
            "segmentation must have shape (height, width, 2) with (objid, objtype); "
            f"got {dimensions}"
        )
    height, width, _channels = dimensions
    if height <= 0 or width <= 0 or height * width > MAX_PIXELS:
        raise VisibilityError(
            f"segmentation resolution is outside the supported bound: {dimensions}"
        )
    return height, width, 2


def _is_integer(value: object) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def count_visible_pixels(
    segmentation: object,
    geom_id: int,
    *,
    object_type: int = GEOM_OBJECT_TYPE,
) -> int:
    """Count pixels whose segmentation pair is exactly (objid, objtype)."""
    if isinstance(geom_id, bool) or not isinstance(geom_id, int) or geom_id < 0:
        raise VisibilityError("geom_id must be a non-negative integer")
    if isinstance(object_type, bool) or not isinstance(object_type, int) or object_type < 0:
        raise VisibilityError("object_type must be a non-negative integer")
    _shape(segmentation)
    try:
        import numpy as np
    except ImportError:
        np = None  # type: ignore[assignment]
    if np is not None and isinstance(segmentation, np.ndarray):
        if segmentation.ndim != 3 or segmentation.shape[2] != 2:
            raise VisibilityError("segmentation must be a 2-channel image")
        if segmentation.dtype.kind not in ("i", "u"):
            raise VisibilityError(f"segmentation dtype must be integer, got {segmentation.dtype}")
        mask = (segmentation[:, :, SEGMENTATION_OBJTYPE_CHANNEL] == object_type) & (
            segmentation[:, :, SEGMENTATION_OBJID_CHANNEL] == geom_id
        )
        return int(np.count_nonzero(mask))
    height, width, _channels = _shape(segmentation)
    count = 0
    for row in range(height):
        row_values = segmentation[row]  # type: ignore[index]
        if len(row_values) != width:
            raise VisibilityError("segmentation rows have inconsistent widths")
        for column in range(width):
            cell = row_values[column]
            if len(cell) != 2:
                raise VisibilityError("segmentation pixels must contain (objid, objtype)")
            obj_id = cell[SEGMENTATION_OBJID_CHANNEL]
            obj_type = cell[SEGMENTATION_OBJTYPE_CHANNEL]
            if not _is_integer(obj_id) or not _is_integer(obj_type):
                raise VisibilityError("segmentation ids and types must be integers")
            if obj_id == geom_id and obj_type == object_type:
                count += 1
    return count


def _summarize_camera(
    camera: str,
    segmentation: object,
    object_geom_ids: Mapping[str, int],
    thresholds: Mapping[str, int],
    resolution: tuple[int, int],
) -> CameraVisibility:
    height, width, _channels = _shape(segmentation)
    expected_width, expected_height = resolution
    if (width, height) != (expected_width, expected_height):
        raise VisibilityError(
            f"camera {camera!r} segmentation is {width}x{height}, "
            f"expected {expected_width}x{expected_height}"
        )
    if set(object_geom_ids) != set(thresholds):
        raise VisibilityError(f"camera {camera!r} object sets do not match thresholds")
    objects: list[ObjectVisibility] = []
    for obj, geom_id in object_geom_ids.items():
        pixels = count_visible_pixels(segmentation, geom_id)
        threshold = thresholds[obj]
        objects.append(
            ObjectVisibility(
                object=obj,
                geom_id=geom_id,
                pixels=pixels,
                threshold=threshold,
                passed=pixels >= threshold,
            )
        )
    total = sum(item.pixels for item in objects)
    return CameraVisibility(
        camera=camera,
        width=width,
        height=height,
        objects=tuple(objects),
        total_pixels=total,
        passed=all(item.passed for item in objects),
    )


def summarize_layout(
    layout: LayoutSpec,
    segmentations: Mapping[str, object],
    object_geom_ids: Mapping[str, int],
    thresholds: Mapping[str, Mapping[str, int]],
    resolutions: Mapping[str, tuple[int, int]],
) -> LayoutVisibility:
    """Count all configured objects for every camera of one layout."""
    if set(segmentations) != set(thresholds):
        raise VisibilityError("segmentation cameras do not match the threshold cameras")
    if set(resolutions) != set(thresholds):
        raise VisibilityError("resolutions do not match the threshold cameras")
    cameras: list[CameraVisibility] = []
    for camera in thresholds:
        cameras.append(
            _summarize_camera(
                camera,
                segmentations[camera],
                object_geom_ids,
                thresholds[camera],
                resolutions[camera],
            )
        )
    total = sum(camera.total_pixels for camera in cameras)
    margins = [item.pixels - item.threshold for camera in cameras for item in camera.objects]
    minimum_margin = min(margins) if margins else 0
    return LayoutVisibility(
        layout_id=layout.layout_id,
        kind=layout.kind,
        cameras=tuple(cameras),
        total_pixels=total,
        minimum_margin=minimum_margin,
        passed=all(camera.passed for camera in cameras),
    )
