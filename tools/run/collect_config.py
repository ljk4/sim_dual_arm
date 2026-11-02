"""Load and validate the host-only recipe for one automatic mock collection."""

from __future__ import annotations

import hashlib
import json
import math
import os
import stat
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any

CONFIG_SCHEMA_VERSION = 1
MAX_JSON_BYTES = 256 * 1024


class ConfigError(ValueError):
    """The recipe is missing, unreadable or does not satisfy the explicit schema."""


@dataclass(frozen=True)
class CollectConfig:
    """Validated, immutable launcher values and their canonical digest."""

    values: Mapping[str, Any]
    sha256: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "values", MappingProxyType(dict(self.values)))

    @property
    def work_deadline_s(self) -> float:
        return float(self.values["work_deadline_s"])

    @property
    def command_timeout_s(self) -> float:
        return float(self.values["command_timeout_s"])

    @property
    def poll_interval_s(self) -> float:
        return float(self.values["poll_interval_s"])

    @property
    def audit_timeout_s(self) -> float:
        return float(self.values["audit_timeout_s"])

    @property
    def stop_grace_s(self) -> float:
        return float(self.values["stop_grace_s"])

    @property
    def cleanup_timeout_s(self) -> float:
        return float(self.values["cleanup_timeout_s"])

    @property
    def diagnostic_max_bytes(self) -> int:
        return int(self.values["diagnostic_max_bytes"])

    @property
    def min_mock_motion_rad(self) -> float:
        return float(self.values["min_mock_motion_rad"])

    def document(self) -> dict[str, Any]:
        return dict(self.values)


_NUMBER_LIMITS = {
    "work_deadline_s": (0.0, 3600.0, False),
    "command_timeout_s": (0.0, 3600.0, False),
    "poll_interval_s": (0.01, 5.0, False),
    "audit_timeout_s": (0.0, 3600.0, False),
    "stop_grace_s": (0.0, 300.0, False),
    "cleanup_timeout_s": (0.0, 300.0, False),
    "launcher_log_close_timeout_s": (0.0, 60.0, False),
    "min_mock_motion_rad": (0.0, 1.0, False),
}
_INTEGER_LIMITS = {
    "diagnostic_max_bytes": (1024, 64 * 1024 * 1024),
    "launcher_log_max_bytes": (4096, 64 * 1024 * 1024),
    "launcher_log_max_event_bytes": (1024, 64 * 1024 * 1024),
    "launcher_log_queue_capacity": (8, 10000),
}
_REQUIRED = {
    "schema_version",
    *_NUMBER_LIMITS,
    *_INTEGER_LIMITS,
}


def _reject_constant(value: str) -> None:
    raise ConfigError(f"non-finite JSON number is not allowed: {value}")


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ConfigError(f"duplicate configuration field: {key!r}")
        result[key] = value
    return result


def _number(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ConfigError(f"{name} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise ConfigError(f"{name} must be a finite number")
    low, high, _inclusive = _NUMBER_LIMITS[name]
    if number <= low or number > high:
        raise ConfigError(f"{name} must be greater than {low} and at most {high}")
    return number


def _integer(name: str, value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError(f"{name} must be a positive integer")
    low, high = _INTEGER_LIMITS[name]
    if value < low or value > high:
        raise ConfigError(f"{name} must be between {low} and {high}")
    return value


def parse_config(document: object) -> CollectConfig:
    """Validate one fully specified recipe; there are no hidden defaults."""
    if not isinstance(document, dict):
        raise ConfigError("configuration must be one JSON object")
    unknown = sorted(set(document) - _REQUIRED)
    missing = sorted(_REQUIRED - set(document))
    if unknown:
        raise ConfigError("unknown configuration fields: " + ", ".join(unknown))
    if missing:
        raise ConfigError("missing configuration fields: " + ", ".join(missing))
    if document["schema_version"] != CONFIG_SCHEMA_VERSION or isinstance(
        document["schema_version"], bool
    ):
        raise ConfigError(f"schema_version must be {CONFIG_SCHEMA_VERSION}")

    values: dict[str, Any] = {"schema_version": CONFIG_SCHEMA_VERSION}
    for name in _NUMBER_LIMITS:
        values[name] = _number(name, document[name])
    for name in _INTEGER_LIMITS:
        values[name] = _integer(name, document[name])

    if values["command_timeout_s"] > values["work_deadline_s"]:
        raise ConfigError("command_timeout_s must not exceed work_deadline_s")
    if values["audit_timeout_s"] > values["work_deadline_s"]:
        raise ConfigError("audit_timeout_s must not exceed work_deadline_s")
    if values["poll_interval_s"] * 2 > values["command_timeout_s"]:
        raise ConfigError("poll_interval_s must be small relative to command_timeout_s")
    if values["launcher_log_max_event_bytes"] > values["launcher_log_max_bytes"]:
        raise ConfigError("launcher_log_max_event_bytes must not exceed launcher_log_max_bytes")
    if values["diagnostic_max_bytes"] < values["launcher_log_max_event_bytes"]:
        raise ConfigError("diagnostic_max_bytes must be at least launcher_log_max_event_bytes")
    return CollectConfig(values=values, sha256=config_sha256(values))


def config_sha256(values: Mapping[str, Any]) -> str:
    canonical = json.dumps(values, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def load_config(path: Path) -> CollectConfig:
    """Read a bounded JSON file and reject duplicate/non-finite/unknown values."""
    try:
        descriptor = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        with os.fdopen(descriptor, "rb") as stream:
            if not stat.S_ISREG(os.fstat(stream.fileno()).st_mode):
                raise ConfigError(f"configuration must be a regular JSON file: {path}")
            payload = stream.read(MAX_JSON_BYTES + 1)
        if len(payload) > MAX_JSON_BYTES:
            raise ConfigError(f"configuration is larger than {MAX_JSON_BYTES} bytes")
        document = json.loads(
            payload,
            parse_constant=_reject_constant,
            object_pairs_hook=_unique_object,
        )
    except ConfigError:
        raise
    except (OSError, UnicodeError, ValueError) as error:
        raise ConfigError(f"cannot parse configuration {path}: {error}") from error
    return parse_config(document)
