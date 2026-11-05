"""Strict, frozen configuration for the S2 kinematic key-pose check.

The module has no MuJoCo dependency so invalid parameters are rejected before
an EGL context, a robot model or a large IK run exists.  Every value is
explicit; this check never derives a threshold or a target height in code.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any

import yaml

from .scene_config import PANDA_JOINT_RANGES, SceneConfig

__all__ = [
    "PHASES",
    "BudgetConfig",
    "ClearanceConfig",
    "IKConfig",
    "KinematicsCheckConfig",
    "KinematicsConfigError",
    "TargetConfig",
    "VisibilityRule",
    "load_kinematics_config",
]

SCHEMA_VERSION = 1
PHASES = (
    "pregrasp",
    "grasp_pose",
    "lift_pose",
    "place_hover",
    "place_pose",
    "retreat",
)
CAMERA_NAMES = ("global", "left_wrist", "right_wrist")
MAX_LAYOUTS = 100
MAX_ITERATIONS = 250
MAX_SAMPLES_PER_SEGMENT = 2000
MAX_PNG_BYTES = 128 * 1024 * 1024
MAX_REPORT_BYTES = 8 * 1024 * 1024
_TOP_LEVEL_KEYS = ("schema_version", "ik", "clearance", "targets", "visibility", "budgets")


class KinematicsConfigError(ValueError):
    """A configuration problem with source file and field path attached."""

    def __init__(self, source: str | Path, field: str, message: str) -> None:
        self.source = str(source)
        self.field = field
        self.message = message
        super().__init__(f"{self.source}: {field}: {message}")


class _UniqueKeyLoader(yaml.SafeLoader):
    source_name = "<kinematics check config>"


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
            raise KinematicsConfigError(
                loader.source_name, "<mapping>", "keys must be hashable"
            ) from error
        if duplicate:
            raise KinematicsConfigError(loader.source_name, str(key), "duplicate key")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping
)


@dataclass(frozen=True, slots=True)
class IKConfig:
    dt_s: float
    max_iterations: int
    position_tolerance_m: float
    orientation_tolerance_rad: float
    posture_cost: float
    position_cost: float
    orientation_cost: float
    lm_damping: float
    solver: str
    initial_right_joint_positions: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class ClearanceConfig:
    minimum_clearance_m: float
    collision_detection_distance_m: float
    sample_step_rad: float
    max_samples_per_segment: int
    max_total_samples: int


@dataclass(frozen=True, slots=True)
class TargetConfig:
    pregrasp_height_m: float
    grasp_height_m: float
    lift_height_m: float
    place_hover_height_m: float
    place_clearance_m: float
    retreat_height_m: float


@dataclass(frozen=True, slots=True)
class VisibilityRule:
    camera: str
    object: str
    phases: tuple[str, ...]
    mode: str
    min_pixels: int | None

    @property
    def record_only(self) -> bool:
        return self.mode == "record_only"


@dataclass(frozen=True, slots=True)
class BudgetConfig:
    max_layouts: int
    max_png_bytes: int
    max_report_bytes: int


@dataclass(frozen=True, slots=True)
class KinematicsCheckConfig:
    source: str
    schema_version: int
    ik: IKConfig
    clearance: ClearanceConfig
    targets: TargetConfig
    visibility_rules: tuple[VisibilityRule, ...]
    budgets: BudgetConfig
    camera_names: tuple[str, ...]
    object_names: tuple[str, ...]
    camera_resolutions: MappingProxyType[str, tuple[int, int]]

    def thresholds(self, camera: str, obj: str, phase: str) -> VisibilityRule:
        for rule in self.visibility_rules:
            if rule.camera == camera and rule.object == obj and phase in rule.phases:
                return rule
        raise KinematicsConfigError(
            self.source, f"visibility.{camera}.{obj}", f"missing phase {phase!r}"
        )


def _field(parent: str, child: str) -> str:
    return f"{parent}.{child}" if parent else child


def _mapping(value: object, source: str, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise KinematicsConfigError(source, field, "must be a mapping")
    for key in value:
        if not isinstance(key, str):
            raise KinematicsConfigError(source, field, "keys must be strings")
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
            raise KinematicsConfigError(source, _field(field, key), "unknown key")
    for key in required:
        if key not in document:
            raise KinematicsConfigError(source, _field(field, key), "missing required key")


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
        raise KinematicsConfigError(source, field, "must be a number; booleans are not accepted")
    try:
        result = float(value)
    except OverflowError as error:
        raise KinematicsConfigError(source, field, "must be finite and fit a float") from error
    if not math.isfinite(result):
        raise KinematicsConfigError(source, field, "must be finite")
    if minimum is not None:
        if strict_minimum and result <= minimum:
            raise KinematicsConfigError(source, field, f"must be greater than {minimum}")
        if not strict_minimum and result < minimum:
            raise KinematicsConfigError(source, field, f"must be at least {minimum}")
    if maximum is not None and result > maximum:
        raise KinematicsConfigError(source, field, f"must be at most {maximum}")
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
        raise KinematicsConfigError(source, field, "must be an integer; booleans are not accepted")
    if value < minimum or value > maximum:
        raise KinematicsConfigError(source, field, f"must be between {minimum} and {maximum}")
    return value


def _text(value: object, source: str, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise KinematicsConfigError(source, field, "must be a non-empty string")
    return value


def _parse_ik(value: object, source: str) -> IKConfig:
    field = "ik"
    document = _mapping(value, source, field)
    _check_keys(
        document,
        source,
        field,
        (
            "dt_s",
            "max_iterations",
            "position_tolerance_m",
            "orientation_tolerance_rad",
            "posture_cost",
            "position_cost",
            "orientation_cost",
            "lm_damping",
            "solver",
            "initial_right_joint_positions",
        ),
    )
    solver = _text(document["solver"], source, f"{field}.solver")
    if solver != "daqp":
        raise KinematicsConfigError(source, f"{field}.solver", "must be 'daqp'")
    positions = document["initial_right_joint_positions"]
    if not isinstance(positions, list) or len(positions) != len(PANDA_JOINT_RANGES):
        raise KinematicsConfigError(
            source,
            f"{field}.initial_right_joint_positions",
            "must be a list of seven joint angles",
        )
    initial_positions: list[float] = []
    for index, value in enumerate(positions):
        angle = _number(value, source, f"{field}.initial_right_joint_positions[{index}]")
        lower, upper = PANDA_JOINT_RANGES[index]
        if angle < lower or angle > upper:
            raise KinematicsConfigError(
                source,
                f"{field}.initial_right_joint_positions[{index}]",
                f"must be within right_joint{index + 1} range [{lower}, {upper}] rad",
            )
        initial_positions.append(angle)
    return IKConfig(
        dt_s=_number(
            document["dt_s"],
            source,
            f"{field}.dt_s",
            minimum=0.0001,
            maximum=0.05,
            strict_minimum=True,
        ),
        max_iterations=_integer(
            document["max_iterations"],
            source,
            f"{field}.max_iterations",
            minimum=1,
            maximum=MAX_ITERATIONS,
        ),
        position_tolerance_m=_number(
            document["position_tolerance_m"],
            source,
            f"{field}.position_tolerance_m",
            minimum=1e-6,
            maximum=0.05,
            strict_minimum=True,
        ),
        orientation_tolerance_rad=_number(
            document["orientation_tolerance_rad"],
            source,
            f"{field}.orientation_tolerance_rad",
            minimum=1e-6,
            maximum=0.5,
            strict_minimum=True,
        ),
        posture_cost=_number(
            document["posture_cost"],
            source,
            f"{field}.posture_cost",
            minimum=0.0,
            maximum=1.0,
        ),
        position_cost=_number(
            document["position_cost"],
            source,
            f"{field}.position_cost",
            minimum=0.0,
            maximum=100.0,
            strict_minimum=True,
        ),
        orientation_cost=_number(
            document["orientation_cost"],
            source,
            f"{field}.orientation_cost",
            minimum=0.0,
            maximum=100.0,
            strict_minimum=True,
        ),
        lm_damping=_number(
            document["lm_damping"],
            source,
            f"{field}.lm_damping",
            minimum=0.0,
            maximum=1.0,
        ),
        solver=solver,
        initial_right_joint_positions=tuple(initial_positions),
    )


def _parse_clearance(value: object, source: str) -> ClearanceConfig:
    field = "clearance"
    document = _mapping(value, source, field)
    _check_keys(
        document,
        source,
        field,
        (
            "minimum_clearance_m",
            "collision_detection_distance_m",
            "sample_step_rad",
            "max_samples_per_segment",
            "max_total_samples",
        ),
    )
    minimum = _number(
        document["minimum_clearance_m"],
        source,
        f"{field}.minimum_clearance_m",
        minimum=0.0001,
        maximum=0.05,
        strict_minimum=True,
    )
    detection = _number(
        document["collision_detection_distance_m"],
        source,
        f"{field}.collision_detection_distance_m",
        minimum=minimum,
        maximum=0.2,
    )
    sample_step = _number(
        document["sample_step_rad"],
        source,
        f"{field}.sample_step_rad",
        minimum=1e-5,
        maximum=0.01,
        strict_minimum=True,
    )
    per_segment = _integer(
        document["max_samples_per_segment"],
        source,
        f"{field}.max_samples_per_segment",
        minimum=2,
        maximum=MAX_SAMPLES_PER_SEGMENT,
    )
    total = _integer(
        document["max_total_samples"],
        source,
        f"{field}.max_total_samples",
        minimum=per_segment,
        maximum=1_000_000,
    )
    return ClearanceConfig(minimum, detection, sample_step, per_segment, total)


def _parse_targets(value: object, source: str) -> TargetConfig:
    field = "targets"
    document = _mapping(value, source, field)
    _check_keys(
        document,
        source,
        field,
        (
            "pregrasp_height_m",
            "grasp_height_m",
            "lift_height_m",
            "place_hover_height_m",
            "place_clearance_m",
            "retreat_height_m",
        ),
    )
    return TargetConfig(
        pregrasp_height_m=_number(
            document["pregrasp_height_m"],
            source,
            f"{field}.pregrasp_height_m",
            minimum=-0.2,
            maximum=0.5,
        ),
        grasp_height_m=_number(
            document["grasp_height_m"],
            source,
            f"{field}.grasp_height_m",
            minimum=-0.2,
            maximum=0.5,
        ),
        lift_height_m=_number(
            document["lift_height_m"],
            source,
            f"{field}.lift_height_m",
            minimum=-0.2,
            maximum=0.5,
        ),
        place_hover_height_m=_number(
            document["place_hover_height_m"],
            source,
            f"{field}.place_hover_height_m",
            minimum=-0.2,
            maximum=0.5,
        ),
        place_clearance_m=_number(
            document["place_clearance_m"],
            source,
            f"{field}.place_clearance_m",
            minimum=0.0,
            maximum=0.2,
        ),
        retreat_height_m=_number(
            document["retreat_height_m"],
            source,
            f"{field}.retreat_height_m",
            minimum=-0.2,
            maximum=0.5,
        ),
    )


def _parse_visibility(
    value: object,
    source: str,
    camera_names: tuple[str, ...],
    object_names: tuple[str, ...],
    resolutions: dict[str, tuple[int, int]],
) -> tuple[VisibilityRule, ...]:
    field = "visibility"
    document = _mapping(value, source, field)
    _check_keys(document, source, field, ("rules",))
    entries = document["rules"]
    if not isinstance(entries, list) or not entries:
        raise KinematicsConfigError(source, f"{field}.rules", "must be a non-empty list")
    expected = _expected_visibility_keys(camera_names, object_names)
    rules: list[VisibilityRule] = []
    seen: set[tuple[str, str, str]] = set()
    for index, entry in enumerate(entries):
        item_field = f"{field}.rules[{index}]"
        rule_document = _mapping(entry, source, item_field)
        _check_keys(
            rule_document,
            source,
            item_field,
            ("camera", "object", "phases", "mode"),
            ("min_pixels",),
        )
        camera = _text(rule_document["camera"], source, f"{item_field}.camera")
        obj = _text(rule_document["object"], source, f"{item_field}.object")
        if camera not in camera_names:
            raise KinematicsConfigError(
                source, f"{item_field}.camera", f"must be one of {list(camera_names)}"
            )
        phases_value = rule_document["phases"]
        if not isinstance(phases_value, list) or not phases_value:
            raise KinematicsConfigError(source, f"{item_field}.phases", "must be a non-empty list")
        phases: list[str] = []
        for phase_index, phase_value in enumerate(phases_value):
            phase = _text(phase_value, source, f"{item_field}.phases[{phase_index}]")
            if phase not in PHASES:
                raise KinematicsConfigError(
                    source,
                    f"{item_field}.phases[{phase_index}]",
                    f"must be one of {list(PHASES)}",
                )
            if phase in phases:
                raise KinematicsConfigError(
                    source, f"{item_field}.phases[{phase_index}]", "duplicate phase"
                )
            key = (camera, obj, phase)
            if key not in expected:
                raise KinematicsConfigError(
                    source,
                    item_field,
                    f"unexpected camera/object pair ({camera!r}, {obj!r})",
                )
            if key in seen:
                raise KinematicsConfigError(
                    source, f"{item_field}.phases[{phase_index}]", "duplicate rule coverage"
                )
            seen.add(key)
            phases.append(phase)
        mode = _text(rule_document["mode"], source, f"{item_field}.mode")
        if mode not in ("required", "record_only"):
            raise KinematicsConfigError(
                source, f"{item_field}.mode", "must be 'required' or 'record_only'"
            )
        if mode == "required":
            if "min_pixels" not in rule_document:
                raise KinematicsConfigError(
                    source, f"{item_field}.min_pixels", "missing required key"
                )
            width, height = resolutions[camera]
            min_pixels = _integer(
                rule_document["min_pixels"],
                source,
                f"{item_field}.min_pixels",
                minimum=1,
                maximum=width * height,
            )
        else:
            if "min_pixels" in rule_document:
                raise KinematicsConfigError(
                    source,
                    f"{item_field}.min_pixels",
                    "record_only rules must not set a threshold",
                )
            min_pixels = None
        rules.append(VisibilityRule(camera, obj, tuple(phases), mode, min_pixels))
    missing = expected - seen
    if missing:
        first = sorted(missing)[0]
        raise KinematicsConfigError(
            source,
            f"{field}.rules",
            f"missing coverage for camera={first[0]!r} object={first[1]!r} phase={first[2]!r}",
        )
    return tuple(rules)


def _expected_visibility_keys(
    camera_names: tuple[str, ...], object_names: tuple[str, ...]
) -> set[tuple[str, str, str]]:
    keys: set[tuple[str, str, str]] = set()
    for camera in camera_names:
        objects = object_names + (("table",) if camera == "left_wrist" else ())
        for obj in objects:
            for phase in PHASES:
                keys.add((camera, obj, phase))
    return keys


def _parse_budgets(value: object, source: str) -> BudgetConfig:
    field = "budgets"
    document = _mapping(value, source, field)
    _check_keys(document, source, field, ("max_layouts", "max_png_bytes", "max_report_bytes"))
    return BudgetConfig(
        max_layouts=_integer(
            document["max_layouts"],
            source,
            f"{field}.max_layouts",
            minimum=1,
            maximum=MAX_LAYOUTS,
        ),
        max_png_bytes=_integer(
            document["max_png_bytes"],
            source,
            f"{field}.max_png_bytes",
            minimum=1,
            maximum=MAX_PNG_BYTES,
        ),
        max_report_bytes=_integer(
            document["max_report_bytes"],
            source,
            f"{field}.max_report_bytes",
            minimum=1024,
            maximum=MAX_REPORT_BYTES,
        ),
    )


def load_kinematics_config(path: Path | str, scene_config: SceneConfig) -> KinematicsCheckConfig:
    """Load and cross-check one kinematics YAML file before any native work."""
    source_path = Path(path)
    source = str(source_path)
    try:
        text = source_path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise KinematicsConfigError(
            source, "<file>", f"cannot read configuration: {error}"
        ) from error
    loader = _UniqueKeyLoader(text)
    loader.source_name = source
    try:
        raw = loader.get_single_data()
    except KinematicsConfigError:
        raise
    except yaml.YAMLError as error:
        raise KinematicsConfigError(source, "<yaml>", f"invalid YAML: {error}") from error
    finally:
        loader.dispose()
    root = _mapping(raw, source, "<root>")
    _check_keys(root, source, "", _TOP_LEVEL_KEYS)
    camera_names = tuple(camera.name for camera in scene_config.cameras)
    object_names = tuple(obj.name for obj in scene_config.objects)
    if camera_names != CAMERA_NAMES:
        raise KinematicsConfigError(
            source,
            "visibility.rules",
            f"scene cameras must be exactly {list(CAMERA_NAMES)}, got {list(camera_names)}",
        )
    resolutions = {camera.name: (camera.width, camera.height) for camera in scene_config.cameras}
    return KinematicsCheckConfig(
        source=source,
        schema_version=_integer(
            root["schema_version"],
            source,
            "schema_version",
            minimum=SCHEMA_VERSION,
            maximum=SCHEMA_VERSION,
        ),
        ik=_parse_ik(root["ik"], source),
        clearance=_parse_clearance(root["clearance"], source),
        targets=_parse_targets(root["targets"], source),
        visibility_rules=_parse_visibility(
            root["visibility"], source, camera_names, object_names, resolutions
        ),
        budgets=_parse_budgets(root["budgets"], source),
        camera_names=camera_names,
        object_names=object_names,
        camera_resolutions=MappingProxyType(resolutions),
    )
