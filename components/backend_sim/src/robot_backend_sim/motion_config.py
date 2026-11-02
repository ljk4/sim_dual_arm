"""Strict immutable configuration for the native motion core.

The YAML file contains only controller parameters.  Joint position and
velocity hard limits stay in the accepted ``RobotProfile``; they are not
repeated here.  Units are explicit SI units and every array follows the fixed
16-dimensional joint order used by :mod:`robot_backend_sim.panda_io`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

__all__ = [
    "CONTROL_JOINT_NAMES",
    "PandaControlConfig",
    "MotionConfigError",
    "load_control_config",
    "parse_control_config",
]

# Kept local so loading the pure configuration does not import MuJoCo.  The
# order is exactly PandaIO.JOINT_NAMES and the accepted panda_stack_v1 profile.
CONTROL_JOINT_NAMES = (
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "left_joint7",
    "left_gripper_width",
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_joint4",
    "right_joint5",
    "right_joint6",
    "right_joint7",
    "right_gripper_width",
)

_TOP_LEVEL_KEYS = (
    "joint_names",
    "max_accelerations",
    "position_tolerances",
    "velocity_tolerances",
    "stable_duration_s",
    "settle_timeout_s",
    "motion_wall_timeout_s",
    "stop_wall_timeout_s",
    "contact_force_min_n",
    "finger_velocity_max_m_s",
)
_ARRAY_KEYS = ("max_accelerations", "position_tolerances", "velocity_tolerances")
_SCALAR_KEYS = _TOP_LEVEL_KEYS[4:]
_MAX_NS = 2**63 - 1


class MotionConfigError(ValueError):
    """A strict configuration rejection with source and field attached."""

    def __init__(self, source: str | Path, field: str, message: str) -> None:
        self.source = str(source)
        self.field = field
        self.message = message
        super().__init__(f"{self.source}: {field}: {message}")


class _UniqueKeyLoader(yaml.SafeLoader):
    """Safe loader that rejects duplicate mapping keys instead of dropping them."""

    source_name = "<control config>"


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
            raise MotionConfigError(
                loader.source_name, "<mapping>", "keys must be hashable"
            ) from error
        if duplicate:
            raise MotionConfigError(loader.source_name, str(key), "duplicate key")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping
)


def _mapping(value: object, source: str | Path, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise MotionConfigError(source, field, "must be a mapping")
    for key in value:
        if not isinstance(key, str):
            raise MotionConfigError(source, field, "keys must be strings")
    return value


def _check_keys(document: dict[str, Any], source: str | Path) -> None:
    for key in document:
        if key not in _TOP_LEVEL_KEYS:
            raise MotionConfigError(source, key, "unknown key")
    for key in _TOP_LEVEL_KEYS:
        if key not in document:
            raise MotionConfigError(source, key, "missing required key")


def _number(value: object, source: str | Path, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise MotionConfigError(source, field, "must be a finite number; booleans are not accepted")
    try:
        result = float(value)
    except (OverflowError, TypeError, ValueError) as error:
        raise MotionConfigError(source, field, "must be finite and fit a float") from error
    if not math.isfinite(result):
        raise MotionConfigError(source, field, "must be finite")
    if result <= 0.0:
        raise MotionConfigError(source, field, "must be positive")
    return result


def _array(value: object, source: str | Path, field: str) -> tuple[float, ...]:
    if isinstance(value, str | bytes):
        raise MotionConfigError(source, field, "must be a list of 16 numbers")
    if not isinstance(value, list | tuple):
        raise MotionConfigError(source, field, "must be a list of 16 numbers")
    if len(value) != len(CONTROL_JOINT_NAMES):
        raise MotionConfigError(
            source,
            field,
            f"must contain {len(CONTROL_JOINT_NAMES)} values, got {len(value)}",
        )
    return tuple(_number(item, source, f"{field}[{index}]") for index, item in enumerate(value))


def _joint_names(value: object, source: str | Path) -> tuple[str, ...]:
    field = "joint_names"
    if not isinstance(value, list | tuple):
        raise MotionConfigError(source, field, "must be a list of 16 names")
    if len(value) != len(CONTROL_JOINT_NAMES):
        raise MotionConfigError(
            source,
            field,
            f"must contain {len(CONTROL_JOINT_NAMES)} names, got {len(value)}",
        )
    result: list[str] = []
    for index, name in enumerate(value):
        if not isinstance(name, str) or not name:
            raise MotionConfigError(source, f"{field}[{index}]", "must be a non-empty string")
        result.append(name)
    if tuple(result) != CONTROL_JOINT_NAMES:
        raise MotionConfigError(
            source,
            field,
            "must match the fixed 16-joint order exactly",
        )
    return tuple(result)


def _seconds_to_ns(seconds: float, source: str | Path, field: str) -> int:
    try:
        nanoseconds = round(seconds * 1_000_000_000)
    except (OverflowError, ValueError) as error:
        raise MotionConfigError(
            source,
            field,
            "must convert to a positive integer nanosecond duration without overflow",
        ) from error
    if nanoseconds <= 0 or nanoseconds > _MAX_NS:
        raise MotionConfigError(
            source,
            field,
            "must convert to a positive integer nanosecond duration without overflow",
        )
    return nanoseconds


def _duration_ns(value: object, source: str | Path, field: str) -> tuple[float, int]:
    seconds = _number(value, source, field)
    return seconds, _seconds_to_ns(seconds, source, field)


@dataclass(frozen=True, slots=True)
class PandaControlConfig:
    """Validated controller limits and deadlines in explicit SI units.

    ``max_accelerations`` are rad/s² for the fourteen arm joints and m/s² for
    each total gripper opening.  ``position_tolerances`` are rad/m,
    ``velocity_tolerances`` rad/s or m/s.  Durations and wall deadlines are
    seconds in the file and integer nanoseconds through the ``*_ns``
    properties.  The object is deeply immutable and ``to_document()`` produces
    a JSON/YAML-serialisable snapshot.
    """

    joint_names: tuple[str, ...]
    max_accelerations: tuple[float, ...]
    position_tolerances: tuple[float, ...]
    velocity_tolerances: tuple[float, ...]
    stable_duration_s: float
    settle_timeout_s: float
    motion_wall_timeout_s: float
    stop_wall_timeout_s: float
    contact_force_min_n: float
    finger_velocity_max_m_s: float

    def __post_init__(self) -> None:
        source = "<control config>"
        if not isinstance(self.joint_names, tuple) or self.joint_names != CONTROL_JOINT_NAMES:
            raise MotionConfigError(source, "joint_names", "must be the fixed 16-joint order")
        for field in _ARRAY_KEYS:
            values = getattr(self, field)
            if not isinstance(values, tuple) or len(values) != len(CONTROL_JOINT_NAMES):
                raise MotionConfigError(source, field, "must contain exactly 16 immutable values")
            for index, value in enumerate(values):
                _number(value, source, f"{field}[{index}]")
        for field in _SCALAR_KEYS:
            _number(getattr(self, field), source, field)
        for field in (
            "stable_duration_ns",
            "settle_timeout_ns",
            "motion_wall_timeout_ns",
            "stop_wall_timeout_ns",
        ):
            getattr(self, field)

    @property
    def stable_duration_ns(self) -> int:
        return _checked_ns(self.stable_duration_s, "stable_duration_s")

    @property
    def settle_timeout_ns(self) -> int:
        return _checked_ns(self.settle_timeout_s, "settle_timeout_s")

    @property
    def motion_wall_timeout_ns(self) -> int:
        return _checked_ns(self.motion_wall_timeout_s, "motion_wall_timeout_s")

    @property
    def stop_wall_timeout_ns(self) -> int:
        return _checked_ns(self.stop_wall_timeout_s, "stop_wall_timeout_s")

    def to_document(self) -> dict[str, object]:
        """Return a fresh JSON/YAML-serialisable snapshot of every field."""
        return {
            "joint_names": list(self.joint_names),
            "max_accelerations": list(self.max_accelerations),
            "position_tolerances": list(self.position_tolerances),
            "velocity_tolerances": list(self.velocity_tolerances),
            "stable_duration_s": self.stable_duration_s,
            "settle_timeout_s": self.settle_timeout_s,
            "motion_wall_timeout_s": self.motion_wall_timeout_s,
            "stop_wall_timeout_s": self.stop_wall_timeout_s,
            "contact_force_min_n": self.contact_force_min_n,
            "finger_velocity_max_m_s": self.finger_velocity_max_m_s,
        }


def _checked_ns(seconds: object, field: str) -> int:
    value = _number(seconds, "<control config>", field)
    return _seconds_to_ns(value, "<control config>", field)


def parse_control_config(
    document: object, source: str | Path = "<control config>"
) -> PandaControlConfig:
    """Validate one mapping and return an immutable controller configuration."""
    mapping = _mapping(document, source, "<root>")
    _check_keys(mapping, source)
    joint_names = _joint_names(mapping["joint_names"], source)
    arrays = {field: _array(mapping[field], source, field) for field in _ARRAY_KEYS}
    durations = {
        field: _duration_ns(mapping[field], source, field)[0] for field in _SCALAR_KEYS[:4]
    }
    scalars = {field: _number(mapping[field], source, field) for field in _SCALAR_KEYS[4:]}
    return PandaControlConfig(
        joint_names=joint_names,
        max_accelerations=arrays["max_accelerations"],
        position_tolerances=arrays["position_tolerances"],
        velocity_tolerances=arrays["velocity_tolerances"],
        stable_duration_s=durations["stable_duration_s"],
        settle_timeout_s=durations["settle_timeout_s"],
        motion_wall_timeout_s=durations["motion_wall_timeout_s"],
        stop_wall_timeout_s=durations["stop_wall_timeout_s"],
        contact_force_min_n=scalars["contact_force_min_n"],
        finger_velocity_max_m_s=scalars["finger_velocity_max_m_s"],
    )


def load_control_config(path: Path | str) -> PandaControlConfig:
    """Load one YAML control file, rejecting duplicate/unknown/bool/non-finite data."""
    source = Path(path)
    try:
        text = source.read_text()
    except OSError as error:
        raise MotionConfigError(source, "<file>", f"cannot read file: {error}") from error
    try:
        loader = _UniqueKeyLoader(text)
        loader.source_name = str(source)
        try:
            document = loader.get_single_data()
        finally:
            loader.dispose()
    except MotionConfigError:
        raise
    except yaml.YAMLError as error:
        raise MotionConfigError(source, "<yaml>", f"invalid YAML: {error}") from error
    return parse_control_config(document, source)
