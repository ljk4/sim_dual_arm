"""Frozen configuration for the dual-Panda stack scene.

The YAML document is the only hand-editable scene parameter source.  Lengths
are metres, angles are degrees in the file and radians in the model, and world
coordinates use +Y toward the table and +Z up.  This module deliberately has
no MuJoCo dependency so configuration errors are rejected before a model is
built.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any

import yaml

__all__ = [
    "CameraConfig",
    "camera_declared_description",
    "ColumnConfig",
    "MountsConfig",
    "ObjectConfig",
    "PhysicsConfig",
    "RandomizationConfig",
    "ResetConfig",
    "SceneConfig",
    "SceneConfigError",
    "TableConfig",
    "load_scene_config",
]

TOP_LEVEL_KEYS = ("physics", "column", "mounts", "table", "objects", "cameras", "reset")
CAMERA_NAMES = ("global", "left_wrist", "right_wrist")
CAMERA_PARENTS = {
    "global": "column",
    "left_wrist": "left_hand",
    "right_wrist": "right_hand",
}
INTEGRATORS = ("euler", "implicit", "implicitfast", "rk4")
# Fixed upstream franka_emika_panda/panda.xml joint ranges, in joint1..joint7
# order.  These are the actual <joint range="..."> values, not the wider
# default class range used by joints without an override.
PANDA_JOINT_RANGES = (
    (-2.8973, 2.8973),
    (-1.7628, 1.7628),
    (-2.8973, 2.8973),
    (-3.0718, -0.0698),
    (-2.8973, 2.8973),
    (-0.0175, 3.7525),
    (-2.8973, 2.8973),
)
FINGER_JOINT_LIMIT = 0.04
MAX_RANDOM_XY = 0.03
MAX_RANDOM_YAW_DEG = 10.0
_OBJECT_NAME_PATTERN = r"^[A-Za-z][A-Za-z0-9_]*$"
_RESERVED_BODY_NAMES = frozenset({"world", "column", "table"})


class SceneConfigError(ValueError):
    """A configuration problem with the source file and field path attached."""

    def __init__(self, source: str | Path, field: str, message: str) -> None:
        self.source = str(source)
        self.field = field
        self.message = message
        super().__init__(f"{self.source}: {field}: {message}")


class _UniqueKeyLoader(yaml.SafeLoader):
    """Safe loader that rejects duplicate mapping keys instead of dropping them."""

    source_name = "<scene config>"


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
            raise SceneConfigError(
                loader.source_name, "<mapping>", "keys must be hashable"
            ) from error
        if duplicate:
            raise SceneConfigError(loader.source_name, str(key), "duplicate key")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping
)


@dataclass(frozen=True, slots=True)
class PhysicsConfig:
    timestep: float
    gravity: tuple[float, float, float]
    integrator: str
    object_friction: tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class ColumnConfig:
    radius: float
    height: float
    rgba: tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class MountsConfig:
    base_x: float
    base_y: float
    base_z: float
    yaw_deg: float
    bracket_length: float
    bracket_width: float
    bracket_height: float
    tcp_position: tuple[float, float, float]
    rgba: tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class TableConfig:
    center_xy: tuple[float, float]
    size_xy: tuple[float, float]
    top_z: float
    thickness: float
    rgba: tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class RandomizationConfig:
    xy_m: float
    yaw_deg: float


@dataclass(frozen=True, slots=True)
class ObjectConfig:
    name: str
    half_extents: tuple[float, float, float]
    nominal_xy: tuple[float, float]
    mass: float
    rgba: tuple[float, float, float, float]
    randomization: RandomizationConfig


@dataclass(frozen=True, slots=True)
class CameraConfig:
    name: str
    parent: str
    position: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]
    fovy: float
    width: int
    height: int
    declared_description_json: str


@dataclass(frozen=True, slots=True)
class ResetConfig:
    home_joint_positions: tuple[float, ...]
    gripper_total_opening: float


@dataclass(frozen=True, slots=True)
class SceneConfig:
    source: str
    physics: PhysicsConfig
    column: ColumnConfig
    mounts: MountsConfig
    table: TableConfig
    objects: tuple[ObjectConfig, ...]
    cameras: tuple[CameraConfig, ...]
    reset: ResetConfig


def _field(parent: str, child: str) -> str:
    return f"{parent}.{child}" if parent else child


def _mapping(value: object, source: str, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise SceneConfigError(source, field, "must be a mapping")
    for key in value:
        if not isinstance(key, str):
            raise SceneConfigError(source, field, "keys must be strings")
    return value


def _check_keys(
    document: dict[str, Any],
    source: str,
    field: str,
    required: tuple[str, ...],
    optional: tuple[str, ...] = (),
) -> None:
    allowed = set(required) | set(optional)
    for key in document:
        if key not in allowed:
            raise SceneConfigError(source, _field(field, key), "unknown key")
    for key in required:
        if key not in document:
            raise SceneConfigError(source, _field(field, key), "missing required key")


def _number(
    value: object,
    source: str,
    field: str,
    *,
    minimum: float | None = None,
    maximum: float | None = None,
    strict_minimum: bool = False,
) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise SceneConfigError(source, field, "must be a number; booleans are not accepted")
    try:
        result = float(value)
    except OverflowError as error:
        raise SceneConfigError(source, field, "must be finite and fit a float") from error
    if not math.isfinite(result):
        raise SceneConfigError(source, field, "must be finite")
    if minimum is not None:
        if strict_minimum and result <= minimum:
            raise SceneConfigError(source, field, f"must be greater than {minimum}")
        if not strict_minimum and result < minimum:
            raise SceneConfigError(source, field, f"must be at least {minimum}")
    if maximum is not None and result > maximum:
        raise SceneConfigError(source, field, f"must be at most {maximum}")
    return result


def _integer(
    value: object,
    source: str,
    field: str,
    *,
    minimum: int,
    maximum: int,
) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise SceneConfigError(source, field, "must be an integer; booleans are not accepted")
    if value < minimum or value > maximum:
        raise SceneConfigError(source, field, f"must be between {minimum} and {maximum}")
    return value


def _text(value: object, source: str, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise SceneConfigError(source, field, "must be a non-empty string")
    return value


def _name(value: object, source: str, field: str) -> str:
    name = _text(value, source, field)
    if not re.fullmatch(_OBJECT_NAME_PATTERN, name):
        raise SceneConfigError(source, field, "must match letters, digits and underscores")
    return name


def _vector(value: object, source: str, field: str, length: int) -> tuple[float, ...]:
    if not isinstance(value, list | tuple) or len(value) != length:
        raise SceneConfigError(source, field, f"must be a list of {length} numbers")
    return tuple(_number(item, source, f"{field}[{index}]") for index, item in enumerate(value))


def _vec2(value: object, source: str, field: str) -> tuple[float, float]:
    vector = _vector(value, source, field, 2)
    return (vector[0], vector[1])


def _vec3(value: object, source: str, field: str) -> tuple[float, float, float]:
    vector = _vector(value, source, field, 3)
    return (vector[0], vector[1], vector[2])


def _rgba(value: object, source: str, field: str) -> tuple[float, float, float, float]:
    vector = _vector(value, source, field, 4)
    for index, component in enumerate(vector):
        if component < 0.0 or component > 1.0:
            raise SceneConfigError(
                source, f"{field}[{index}]", "color components must be between 0 and 1"
            )
    return (vector[0], vector[1], vector[2], vector[3])


def _quaternion(value: object, source: str, field: str) -> tuple[float, float, float, float]:
    vector = _vector(value, source, field, 4)
    norm = math.sqrt(sum(component * component for component in vector))
    if not math.isfinite(norm) or norm < 1e-9:
        raise SceneConfigError(source, field, "must be a non-zero finite quaternion")
    if abs(norm - 1.0) > 1e-3:
        raise SceneConfigError(source, field, "must be a unit quaternion")
    return (vector[0] / norm, vector[1] / norm, vector[2] / norm, vector[3] / norm)


def _parse_physics(value: object, source: str) -> PhysicsConfig:
    field = "physics"
    document = _mapping(value, source, field)
    _check_keys(
        document,
        source,
        field,
        ("timestep", "gravity", "integrator", "object_friction"),
    )
    timestep = _number(
        document["timestep"], source, f"{field}.timestep", minimum=1e-5, maximum=0.01
    )
    gravity = _vec3(document["gravity"], source, f"{field}.gravity")
    if gravity[2] >= 0.0:
        raise SceneConfigError(
            source, f"{field}.gravity[2]", "must point downward for free-object physics"
        )
    integrator = _text(document["integrator"], source, f"{field}.integrator")
    if integrator not in INTEGRATORS:
        raise SceneConfigError(
            source,
            f"{field}.integrator",
            f"must be one of {', '.join(INTEGRATORS)}",
        )
    friction = _vec3(document["object_friction"], source, f"{field}.object_friction")
    for index, component in enumerate(friction):
        if component < 0.0:
            raise SceneConfigError(
                source, f"{field}.object_friction[{index}]", "friction must be non-negative"
            )
    if friction[0] == 0.0:
        raise SceneConfigError(
            source, f"{field}.object_friction[0]", "sliding friction must be positive"
        )
    return PhysicsConfig(timestep, gravity, integrator, friction)


def _parse_column(value: object, source: str) -> ColumnConfig:
    field = "column"
    document = _mapping(value, source, field)
    _check_keys(document, source, field, ("radius", "height", "rgba"))
    radius = _number(document["radius"], source, f"{field}.radius", minimum=0.01, maximum=0.5)
    height = _number(document["height"], source, f"{field}.height", minimum=0.1, maximum=3.0)
    rgba = _rgba(document["rgba"], source, f"{field}.rgba")
    return ColumnConfig(radius, height, rgba)


def _parse_mounts(value: object, source: str) -> MountsConfig:
    field = "mounts"
    document = _mapping(value, source, field)
    _check_keys(
        document,
        source,
        field,
        (
            "base_x",
            "base_y",
            "base_z",
            "yaw_deg",
            "bracket_length",
            "bracket_width",
            "bracket_height",
            "tcp_position",
            "rgba",
        ),
    )
    base_x = _number(document["base_x"], source, f"{field}.base_x", minimum=0.05, maximum=1.0)
    base_y = _number(document["base_y"], source, f"{field}.base_y")
    base_z = _number(document["base_z"], source, f"{field}.base_z", minimum=0.0, maximum=3.0)
    yaw_deg = _number(
        document["yaw_deg"], source, f"{field}.yaw_deg", minimum=-180.0, maximum=180.0
    )
    bracket_length = _number(
        document["bracket_length"],
        source,
        f"{field}.bracket_length",
        minimum=0.01,
        maximum=1.0,
    )
    bracket_width = _number(
        document["bracket_width"],
        source,
        f"{field}.bracket_width",
        minimum=0.01,
        maximum=0.5,
    )
    bracket_height = _number(
        document["bracket_height"],
        source,
        f"{field}.bracket_height",
        minimum=0.01,
        maximum=0.5,
    )
    tcp_position = _vec3(document["tcp_position"], source, f"{field}.tcp_position")
    rgba = _rgba(document["rgba"], source, f"{field}.rgba")
    return MountsConfig(
        base_x,
        base_y,
        base_z,
        yaw_deg,
        bracket_length,
        bracket_width,
        bracket_height,
        tcp_position,
        rgba,
    )


def _parse_table(value: object, source: str) -> TableConfig:
    field = "table"
    document = _mapping(value, source, field)
    _check_keys(document, source, field, ("center_xy", "size_xy", "top_z", "thickness", "rgba"))
    center_xy = _vec2(document["center_xy"], source, f"{field}.center_xy")
    size_xy = _vec2(document["size_xy"], source, f"{field}.size_xy")
    for index, size in enumerate(size_xy):
        if size < 0.05 or size > 5.0:
            raise SceneConfigError(
                source, f"{field}.size_xy[{index}]", "must be between 0.05 and 5.0"
            )
    top_z = _number(document["top_z"], source, f"{field}.top_z", minimum=0.01, maximum=3.0)
    thickness = _number(
        document["thickness"],
        source,
        f"{field}.thickness",
        minimum=0.001,
        maximum=min(0.5, top_z),
    )
    rgba = _rgba(document["rgba"], source, f"{field}.rgba")
    return TableConfig(center_xy, size_xy, top_z, thickness, rgba)


def _parse_randomization(value: object, source: str, field: str) -> RandomizationConfig:
    document = _mapping(value, source, field)
    _check_keys(document, source, field, ("xy_m", "yaw_deg"))
    xy_m = _number(document["xy_m"], source, f"{field}.xy_m", minimum=0.0, maximum=MAX_RANDOM_XY)
    yaw_deg = _number(
        document["yaw_deg"],
        source,
        f"{field}.yaw_deg",
        minimum=0.0,
        maximum=MAX_RANDOM_YAW_DEG,
    )
    return RandomizationConfig(xy_m, yaw_deg)


def _parse_objects(value: object, source: str) -> tuple[ObjectConfig, ...]:
    field = "objects"
    if not isinstance(value, list | tuple):
        raise SceneConfigError(source, field, "must be a list of two objects")
    if len(value) != 2:
        raise SceneConfigError(source, field, "must contain exactly two objects")
    objects: list[ObjectConfig] = []
    names: set[str] = set()
    for index, entry in enumerate(value):
        item_field = f"{field}[{index}]"
        document = _mapping(entry, source, item_field)
        _check_keys(
            document,
            source,
            item_field,
            ("name", "half_extents", "nominal_xy", "mass", "rgba", "randomization"),
        )
        name = _name(document["name"], source, f"{item_field}.name")
        if name in names:
            raise SceneConfigError(source, f"{item_field}.name", f"duplicate object name {name!r}")
        if name in _RESERVED_BODY_NAMES:
            raise SceneConfigError(source, f"{item_field}.name", f"reserved scene body {name!r}")
        names.add(name)
        half_extents = _vec3(document["half_extents"], source, f"{item_field}.half_extents")
        for axis, extent in enumerate(half_extents):
            if extent < 0.005 or extent > 0.25:
                raise SceneConfigError(
                    source,
                    f"{item_field}.half_extents[{axis}]",
                    "must be between 0.005 and 0.25",
                )
        nominal_xy = _vec2(document["nominal_xy"], source, f"{item_field}.nominal_xy")
        mass = _number(document["mass"], source, f"{item_field}.mass", minimum=0.001, maximum=10.0)
        rgba = _rgba(document["rgba"], source, f"{item_field}.rgba")
        randomization = _parse_randomization(
            document["randomization"], source, f"{item_field}.randomization"
        )
        objects.append(ObjectConfig(name, half_extents, nominal_xy, mass, rgba, randomization))
    return tuple(objects)


def _parse_cameras(value: object, source: str) -> tuple[CameraConfig, ...]:
    field = "cameras"
    document = _mapping(value, source, field)
    _check_keys(document, source, field, CAMERA_NAMES)
    cameras = []
    for name in CAMERA_NAMES:
        item_field = f"{field}.{name}"
        entry = _mapping(document[name], source, item_field)
        cameras.append(_parse_camera(name, entry, source, item_field))
    widths = {camera.width for camera in cameras}
    heights = {camera.height for camera in cameras}
    if len(widths) != 1 or len(heights) != 1:
        raise SceneConfigError(
            source, f"{field}.width", "all scene cameras must share one resolution"
        )
    return tuple(cameras)


_CAMERA_FIELDS = ("parent", "position", "quaternion_wxyz", "fovy", "width", "height")


def _parse_camera(
    name: str,
    entry: object,
    source: str,
    item_field: str,
    *,
    declaration: bool = False,
) -> CameraConfig:
    document = _mapping(entry, source, item_field)
    keys = _CAMERA_FIELDS if not declaration else ("name", *_CAMERA_FIELDS)
    _check_keys(document, source, item_field, keys)
    if declaration:
        declared_name = _text(document["name"], source, f"{item_field}.name")
        if declared_name != name:
            raise SceneConfigError(source, f"{item_field}.name", f"must be {name!r}")
    parent = _text(document["parent"], source, f"{item_field}.parent")
    expected_parent = CAMERA_PARENTS[name]
    if parent != expected_parent:
        raise SceneConfigError(
            source,
            f"{item_field}.parent",
            f"must be {expected_parent!r} for fixed {name} camera",
        )
    position = _vec3(document["position"], source, f"{item_field}.position")
    quaternion = _quaternion(document["quaternion_wxyz"], source, f"{item_field}.quaternion_wxyz")
    fovy = _number(document["fovy"], source, f"{item_field}.fovy", minimum=5.0, maximum=170.0)
    width = _integer(document["width"], source, f"{item_field}.width", minimum=1, maximum=4096)
    height = _integer(document["height"], source, f"{item_field}.height", minimum=1, maximum=4096)
    description = {
        "name": name,
        "parent": parent,
        "position": list(document["position"]),
        "quaternion_wxyz": list(document["quaternion_wxyz"]),
        "fovy": document["fovy"],
        "width": document["width"],
        "height": document["height"],
    }
    declared_description_json = json.dumps(
        description,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    )
    return CameraConfig(
        name,
        parent,
        position,
        quaternion,
        fovy,
        width,
        height,
        declared_description_json,
    )


def camera_declared_description(camera: CameraConfig) -> dict[str, Any]:
    """Return a fresh mutable camera declaration with the original YAML values.

    ``CameraConfig`` keeps the validated/normalized typed values used by the
    scene and a canonical JSON copy of the original declaration.  This helper
    decodes that immutable copy, re-parses it through the same camera validator,
    and requires the rebuilt typed camera to equal ``camera`` exactly.  A
    ``dataclasses.replace`` that changes geometry but leaves the old declaration
    behind is therefore rejected instead of silently keeping an old digest.
    """
    source = "<camera declaration>"
    if not isinstance(camera, CameraConfig):
        raise SceneConfigError(source, "<camera>", "must be a CameraConfig")
    field = f"cameras.{camera.name}"
    if camera.name not in CAMERA_NAMES:
        raise SceneConfigError(source, f"{field}.name", f"must be one of {CAMERA_NAMES}")
    try:
        declaration = json.loads(camera.declared_description_json)
    except (TypeError, ValueError) as error:
        raise SceneConfigError(
            source,
            f"{field}.declared_description_json",
            "must be a canonical JSON object",
        ) from error
    if not isinstance(declaration, dict):
        raise SceneConfigError(
            source,
            f"{field}.declared_description_json",
            "must be a canonical JSON object",
        )
    rebuilt = _parse_camera(camera.name, declaration, source, field, declaration=True)
    if rebuilt != camera:
        mismatch = next(
            item.name
            for item in fields(CameraConfig)
            if getattr(rebuilt, item.name) != getattr(camera, item.name)
        )
        raise SceneConfigError(
            source,
            f"{field}.{mismatch}",
            "declared camera description does not match the typed camera fields",
        )
    return declaration


def _parse_reset(value: object, source: str) -> ResetConfig:
    field = "reset"
    document = _mapping(value, source, field)
    _check_keys(document, source, field, ("home_joint_positions", "gripper_total_opening"))
    positions = document["home_joint_positions"]
    if not isinstance(positions, list | tuple) or len(positions) != 7:
        raise SceneConfigError(
            source, f"{field}.home_joint_positions", "must be a list of seven joint angles"
        )
    home: list[float] = []
    for index, value in enumerate(positions):
        angle = _number(value, source, f"{field}.home_joint_positions[{index}]")
        lower, upper = PANDA_JOINT_RANGES[index]
        if angle < lower or angle > upper:
            raise SceneConfigError(
                source,
                f"{field}.home_joint_positions[{index}]",
                f"must be within fixed upstream joint{index + 1} range [{lower}, {upper}] rad",
            )
        home.append(angle)
    opening = _number(
        document["gripper_total_opening"],
        source,
        f"{field}.gripper_total_opening",
        minimum=0.0,
        maximum=2.0 * FINGER_JOINT_LIMIT,
    )
    return ResetConfig(tuple(home), opening)


def _validate_table_and_objects(config: SceneConfig) -> None:
    table = config.table
    half_x = table.size_xy[0] / 2.0
    half_y = table.size_xy[1] / 2.0
    for index, obj in enumerate(config.objects):
        radius = math.hypot(obj.half_extents[0], obj.half_extents[1])
        reach = radius + obj.randomization.xy_m
        for axis, (nominal, table_half) in enumerate(
            (
                (obj.nominal_xy[0] - table.center_xy[0], half_x),
                (obj.nominal_xy[1] - table.center_xy[1], half_y),
            )
        ):
            if abs(nominal) + reach > table_half + 1e-9:
                raise SceneConfigError(
                    config.source,
                    f"objects[{index}].nominal_xy[{axis}]",
                    "randomized footprint crosses the table edge",
                )
    for first in range(len(config.objects)):
        for second in range(first + 1, len(config.objects)):
            left = config.objects[first]
            right = config.objects[second]
            # Each object's XY jitter is independent per axis.  Collapse each
            # axis to the smallest possible centre separation after consuming
            # both jitter radii, then compare that worst-case centre distance
            # with the conservative circular footprints of the yawed boxes.
            axis_dx = max(
                0.0,
                abs(left.nominal_xy[0] - right.nominal_xy[0])
                - left.randomization.xy_m
                - right.randomization.xy_m,
            )
            axis_dy = max(
                0.0,
                abs(left.nominal_xy[1] - right.nominal_xy[1])
                - left.randomization.xy_m
                - right.randomization.xy_m,
            )
            min_centre_distance = math.hypot(axis_dx, axis_dy)
            left_footprint = math.hypot(*left.half_extents[:2])
            right_footprint = math.hypot(*right.half_extents[:2])
            if min_centre_distance <= left_footprint + right_footprint:
                raise SceneConfigError(
                    config.source,
                    f"objects[{second}].nominal_xy",
                    f"randomized footprint can overlap objects[{first}]",
                )


def load_scene_config(path: Path | str) -> SceneConfig:
    """Load and fully validate one scene YAML file.

    Unknown keys, duplicate YAML keys, non-finite or out-of-range numbers,
    invalid quaternions and unsafe object randomization are rejected here,
    before MuJoCo or any robot asset is opened.
    """
    source_path = Path(path)
    source = str(source_path)
    try:
        text = source_path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise SceneConfigError(source, "<file>", f"cannot read configuration: {error}") from error
    loader = _UniqueKeyLoader(text)
    loader.source_name = source
    try:
        raw = loader.get_single_data()
    except SceneConfigError:
        raise
    except yaml.YAMLError as error:
        raise SceneConfigError(source, "<yaml>", f"invalid YAML: {error}") from error
    finally:
        loader.dispose()
    root = _mapping(raw, source, "<root>")
    _check_keys(root, source, "", TOP_LEVEL_KEYS)
    config = SceneConfig(
        source=source,
        physics=_parse_physics(root["physics"], source),
        column=_parse_column(root["column"], source),
        mounts=_parse_mounts(root["mounts"], source),
        table=_parse_table(root["table"], source),
        objects=_parse_objects(root["objects"], source),
        cameras=_parse_cameras(root["cameras"], source),
        reset=_parse_reset(root["reset"], source),
    )
    _validate_table_and_objects(config)
    return config
