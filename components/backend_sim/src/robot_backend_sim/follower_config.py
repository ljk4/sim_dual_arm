"""Strict immutable configuration for bounded target following.

The file contains only follower-specific parameters.  Joint hard limits and
maximum velocities remain in the accepted robot profile; maximum accelerations
remain in :class:`robot_backend_sim.motion_config.PandaControlConfig`.  Units are
explicit SI units and every array follows the fixed 16-joint control order.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from robot_contracts.action_chunks import MAX_TARGET_COUNT
from robot_contracts.reference import MAX_HORIZON_NS

from .motion_config import CONTROL_JOINT_NAMES

__all__ = [
    "FollowerConfigError",
    "PandaFollowerConfig",
    "load_follower_config",
    "parse_follower_config",
]

_TOP_LEVEL_KEYS = (
    "joint_names",
    "target_count",
    "max_target_deviations",
    "max_terminal_tail_duration_s",
)


class FollowerConfigError(ValueError):
    """A strict configuration rejection with source and field attached."""

    def __init__(self, source: str | Path, field: str, message: str) -> None:
        self.source = str(source)
        self.field = field
        self.message = message
        super().__init__(f"{self.source}: {field}: {message}")


class _UniqueKeyLoader(yaml.SafeLoader):
    """Safe loader that rejects duplicate mapping keys instead of dropping them."""

    source_name = "<follower config>"


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
            raise FollowerConfigError(
                loader.source_name, "<mapping>", "keys must be hashable"
            ) from error
        if duplicate:
            raise FollowerConfigError(loader.source_name, str(key), "duplicate key")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping
)


def _mapping(value: object, source: str | Path) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise FollowerConfigError(source, "<root>", "must be a mapping")
    for key in value:
        if not isinstance(key, str):
            raise FollowerConfigError(source, "<root>", "keys must be strings")
    return value


def _check_keys(document: dict[str, Any], source: str | Path) -> None:
    for key in document:
        if key not in _TOP_LEVEL_KEYS:
            raise FollowerConfigError(source, key, "unknown key")
    for key in _TOP_LEVEL_KEYS:
        if key not in document:
            raise FollowerConfigError(source, key, "missing required key")


def _number(value: object, source: str | Path, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise FollowerConfigError(
            source,
            field,
            "must be a finite number; booleans are not accepted",
        )
    try:
        result = float(value)
    except (OverflowError, TypeError, ValueError) as error:
        raise FollowerConfigError(source, field, "must be finite and fit a float") from error
    if not math.isfinite(result):
        raise FollowerConfigError(source, field, "must be finite")
    return result


def _positive_number(value: object, source: str | Path, field: str) -> float:
    result = _number(value, source, field)
    if result <= 0.0:
        raise FollowerConfigError(source, field, "must be positive")
    return result


def _array(value: object, source: str | Path, field: str) -> tuple[float, ...]:
    if isinstance(value, str | bytes) or not isinstance(value, list | tuple):
        raise FollowerConfigError(source, field, "must be a list of 16 numbers")
    if len(value) != len(CONTROL_JOINT_NAMES):
        raise FollowerConfigError(
            source,
            field,
            f"must contain {len(CONTROL_JOINT_NAMES)} values, got {len(value)}",
        )
    return tuple(
        _positive_number(item, source, f"{field}[{index}]")
        for index, item in enumerate(value)
    )


def _joint_names(value: object, source: str | Path) -> tuple[str, ...]:
    field = "joint_names"
    if not isinstance(value, list | tuple) or len(value) != len(CONTROL_JOINT_NAMES):
        raise FollowerConfigError(
            source,
            field,
            f"must contain {len(CONTROL_JOINT_NAMES)} names",
        )
    for index, name in enumerate(value):
        if not isinstance(name, str) or not name:
            raise FollowerConfigError(source, f"{field}[{index}]", "must be a non-empty string")
    result = tuple(value)
    if result != CONTROL_JOINT_NAMES:
        raise FollowerConfigError(source, field, "must match the fixed 16-joint order exactly")
    return result


def _target_count(value: object, source: str | Path) -> int:
    field = "target_count"
    if isinstance(value, bool) or not isinstance(value, int):
        raise FollowerConfigError(source, field, "must be an integer")
    if not 2 <= value <= MAX_TARGET_COUNT:
        raise FollowerConfigError(source, field, f"must be in 2..{MAX_TARGET_COUNT}")
    return value


def _duration_ns(value: object, source: str | Path, field: str) -> tuple[float, int]:
    seconds = _positive_number(value, source, field)
    try:
        nanoseconds = round(seconds * 1_000_000_000)
    except (OverflowError, ValueError) as error:
        raise FollowerConfigError(
            source,
            field,
            "must convert to an integer nanosecond duration without overflow",
        ) from error
    if nanoseconds <= 0 or nanoseconds > MAX_HORIZON_NS:
        raise FollowerConfigError(
            source,
            field,
            f"must convert to a positive duration not exceeding {MAX_HORIZON_NS}ns",
        )
    return seconds, nanoseconds


@dataclass(frozen=True, slots=True)
class PandaFollowerConfig:
    """Validated bounded-follower selection, tail and deviation parameters.

    ``max_target_deviations`` is the per-axis continuous maximum allowed
    distance between the actual reference and the piecewise-linear raw target.
    It is a following budget, not a relaxation of the robot's joint hard limits.
    """

    joint_names: tuple[str, ...]
    target_count: int
    max_target_deviations: tuple[float, ...]
    max_terminal_tail_duration_s: float

    def __post_init__(self) -> None:
        source = "<follower config>"
        if not isinstance(self.joint_names, tuple) or self.joint_names != CONTROL_JOINT_NAMES:
            raise FollowerConfigError(source, "joint_names", "must be the fixed 16-joint order")
        if (
            isinstance(self.target_count, bool)
            or not isinstance(self.target_count, int)
            or not 2 <= self.target_count <= MAX_TARGET_COUNT
        ):
            raise FollowerConfigError(
                source,
                "target_count",
                f"must be an integer in 2..{MAX_TARGET_COUNT}",
            )
        if (
            not isinstance(self.max_target_deviations, tuple)
            or len(self.max_target_deviations) != len(CONTROL_JOINT_NAMES)
        ):
            raise FollowerConfigError(
                source,
                "max_target_deviations",
                "must contain exactly 16 immutable values",
            )
        deviations = tuple(
            _positive_number(value, source, f"max_target_deviations[{index}]")
            for index, value in enumerate(self.max_target_deviations)
        )
        seconds, _ = _duration_ns(
            self.max_terminal_tail_duration_s,
            source,
            "max_terminal_tail_duration_s",
        )
        object.__setattr__(self, "max_target_deviations", deviations)
        object.__setattr__(self, "max_terminal_tail_duration_s", seconds)

    @property
    def max_terminal_tail_duration_ns(self) -> int:
        return _duration_ns(
            self.max_terminal_tail_duration_s,
            "<follower config>",
            "max_terminal_tail_duration_s",
        )[1]

    def to_document(self) -> dict[str, object]:
        """Return a fresh JSON/YAML-serialisable snapshot of every field."""
        return {
            "joint_names": list(self.joint_names),
            "target_count": self.target_count,
            "max_target_deviations": list(self.max_target_deviations),
            "max_terminal_tail_duration_s": self.max_terminal_tail_duration_s,
        }


def parse_follower_config(
    document: object, source: str | Path = "<follower config>"
) -> PandaFollowerConfig:
    """Validate one mapping and return an immutable follower configuration."""
    mapping = _mapping(document, source)
    _check_keys(mapping, source)
    joint_names = _joint_names(mapping["joint_names"], source)
    target_count = _target_count(mapping["target_count"], source)
    deviations = _array(mapping["max_target_deviations"], source, "max_target_deviations")
    seconds, _ = _duration_ns(
        mapping["max_terminal_tail_duration_s"],
        source,
        "max_terminal_tail_duration_s",
    )
    return PandaFollowerConfig(
        joint_names=joint_names,
        target_count=target_count,
        max_target_deviations=deviations,
        max_terminal_tail_duration_s=seconds,
    )


def load_follower_config(path: Path | str) -> PandaFollowerConfig:
    """Load one YAML follower file, rejecting duplicate/unknown/bool/non-finite data."""
    source = Path(path)
    try:
        text = source.read_text()
    except OSError as error:
        raise FollowerConfigError(source, "<file>", f"cannot read file: {error}") from error
    try:
        loader = _UniqueKeyLoader(text)
        loader.source_name = str(source)
        try:
            document = loader.get_single_data()
        finally:
            loader.dispose()
    except FollowerConfigError:
        raise
    except yaml.YAMLError as error:
        raise FollowerConfigError(source, "<yaml>", f"invalid YAML: {error}") from error
    return parse_follower_config(document, source)
