"""Strict immutable configuration for bounded continuous chunk streams.

This module is pure Python/PyYAML and never imports MuJoCo.  It is deliberately
separate from :mod:`motion_config`: the accepted per-motion controller file does
not gain new required keys, and callers opt into continuous streaming by
loading this file and passing the resulting object to ``NativeMotion``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

__all__ = [
    "STREAM_CONFIG_FIELDS",
    "StreamConfig",
    "StreamConfigError",
    "load_stream_config",
    "parse_stream_config",
]

STREAM_CONFIG_FIELDS = (
    "initial_lead_s",
    "min_lead_s",
    "max_observation_age_s",
    "stream_wall_timeout_s",
    "progress_wall_timeout_s",
    "tracking_timeout_s",
)
_MAX_NS = 2**63 - 1


class StreamConfigError(ValueError):
    """A strict configuration rejection with source path and field attached."""

    def __init__(self, source: str | Path, field: str, message: str) -> None:
        self.source = str(source)
        self.field = field
        self.message = message
        super().__init__(f"{self.source}: {field}: {message}")


class _UniqueKeyLoader(yaml.SafeLoader):
    source_name = "<stream config>"


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
            raise StreamConfigError(
                loader.source_name, "<mapping>", "keys must be hashable"
            ) from error
        if duplicate:
            raise StreamConfigError(loader.source_name, str(key), "duplicate key")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping
)


def _mapping(value: object, source: str | Path) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise StreamConfigError(source, "<root>", "must be a mapping")
    for key in value:
        if not isinstance(key, str):
            raise StreamConfigError(source, "<root>", "keys must be strings")
    return value


def _number(value: object, source: str | Path, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise StreamConfigError(source, field, "must be a finite number; booleans are not accepted")
    try:
        result = float(value)
    except (OverflowError, TypeError, ValueError) as error:
        raise StreamConfigError(source, field, "must be finite and fit a float") from error
    if not math.isfinite(result):
        raise StreamConfigError(source, field, "must be finite")
    if result <= 0.0:
        raise StreamConfigError(source, field, "must be positive")
    return result


def _seconds_to_ns(seconds: float, source: str | Path, field: str) -> int:
    try:
        nanoseconds = round(seconds * 1_000_000_000)
    except (OverflowError, ValueError) as error:
        raise StreamConfigError(
            source,
            field,
            "must convert to a positive integer nanosecond duration without overflow",
        ) from error
    if nanoseconds <= 0 or nanoseconds > _MAX_NS:
        raise StreamConfigError(
            source,
            field,
            "must convert to a positive integer nanosecond duration without overflow",
        )
    return nanoseconds


@dataclass(frozen=True, slots=True)
class StreamConfig:
    """Validated stream lead/age/deadline parameters in explicit SI units."""

    initial_lead_s: float
    min_lead_s: float
    max_observation_age_s: float
    stream_wall_timeout_s: float
    progress_wall_timeout_s: float
    tracking_timeout_s: float
    contact_unload_timeout_s: float = 0.0

    def __post_init__(self) -> None:
        source = "<stream config>"
        value = self.contact_unload_timeout_s
        if (
            isinstance(value, bool)
            or not isinstance(value, int | float)
            or not math.isfinite(value)
            or not 0 <= value <= 2.0
        ):
            raise StreamConfigError(
                source, "contact_unload_timeout_s", "must be finite in [0, 2] seconds"
            )
        for field in STREAM_CONFIG_FIELDS:
            _number(getattr(self, field), source, field)
        if self.initial_lead_s < self.min_lead_s:
            raise StreamConfigError(
                source,
                "initial_lead_s",
                "must be greater than or equal to min_lead_s",
            )
        for field in STREAM_CONFIG_FIELDS:
            self.nanoseconds(field)

    def nanoseconds(self, field: str) -> int:
        if field not in STREAM_CONFIG_FIELDS:
            raise StreamConfigError("<stream config>", field, "unknown field")
        return _seconds_to_ns(
            _number(getattr(self, field), "<stream config>", field),
            "<stream config>",
            field,
        )

    @property
    def initial_lead_ns(self) -> int:
        return self.nanoseconds("initial_lead_s")

    @property
    def min_lead_ns(self) -> int:
        return self.nanoseconds("min_lead_s")

    @property
    def max_observation_age_ns(self) -> int:
        return self.nanoseconds("max_observation_age_s")

    @property
    def stream_wall_timeout_ns(self) -> int:
        return self.nanoseconds("stream_wall_timeout_s")

    @property
    def progress_wall_timeout_ns(self) -> int:
        return self.nanoseconds("progress_wall_timeout_s")

    @property
    def tracking_timeout_ns(self) -> int:
        return self.nanoseconds("tracking_timeout_s")

    def to_document(self) -> dict[str, float]:
        """Return a fresh JSON/YAML-serialisable snapshot of every field."""
        document = {field: float(getattr(self, field)) for field in STREAM_CONFIG_FIELDS}
        if self.contact_unload_timeout_s:
            document["contact_unload_timeout_s"] = self.contact_unload_timeout_s
        return document


def parse_stream_config(document: object, source: str | Path = "<stream config>") -> StreamConfig:
    """Validate one mapping, rejecting unknown/missing/duplicate keys."""
    mapping = _mapping(document, source)
    for key in mapping:
        if key not in (*STREAM_CONFIG_FIELDS, "contact_unload_timeout_s"):
            raise StreamConfigError(source, key, "unknown key")
    values: dict[str, float] = {}
    for field in STREAM_CONFIG_FIELDS:
        if field not in mapping:
            raise StreamConfigError(source, field, "missing required key")
        values[field] = _number(mapping[field], source, field)
    if values["initial_lead_s"] < values["min_lead_s"]:
        raise StreamConfigError(
            source,
            "initial_lead_s",
            "must be greater than or equal to min_lead_s",
        )
    for field in STREAM_CONFIG_FIELDS:
        _seconds_to_ns(values[field], source, field)
    try:
        return StreamConfig(
            **values, contact_unload_timeout_s=mapping.get("contact_unload_timeout_s", 0.0)
        )
    except StreamConfigError as error:
        raise StreamConfigError(source, error.field, error.message) from error
    except TypeError as error:  # pragma: no cover - guarded by the field loop
        raise StreamConfigError(source, "<root>", str(error)) from error


def load_stream_config(path: Path | str) -> StreamConfig:
    """Load one YAML stream file with the same strictness as the parser."""
    source = Path(path)
    try:
        text = source.read_text()
    except OSError as error:
        raise StreamConfigError(source, "<file>", f"cannot read file: {error}") from error
    try:
        loader = _UniqueKeyLoader(text)
        loader.source_name = str(source)
        try:
            document = loader.get_single_data()
        finally:
            loader.dispose()
    except StreamConfigError:
        raise
    except yaml.YAMLError as error:
        raise StreamConfigError(source, "<yaml>", f"invalid YAML: {error}") from error
    return parse_stream_config(document, source)
