"""Small strict readers for the sample-filter rule document.

The main dataset/training configuration is validated by Pydantic models in
:mod:`dual_arm_learning.config`; this module only serves the independent
sample-filter YAML and keeps its original field-path diagnostics.
"""

from __future__ import annotations

import math
import re
from pathlib import Path
from typing import Any

from robot_contracts.validation import (
    StrictMappingError as ConfigError,
)
from robot_contracts.validation import (
    UniqueKeyError,
    read_unique_yaml,
)

_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.:-]{0,127}$")


def load_yaml(path: str | Path) -> dict[str, Any]:
    source = Path(path)
    try:
        document = read_unique_yaml(source)
    except UniqueKeyError as error:
        raise ConfigError(error.source, error.field, error.message) from error
    return mapping(document, "<root>")


def mapping(value: object, location: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ConfigError(location, "must be an object")
    result: dict[str, Any] = {}
    for key, item in value.items():
        if not isinstance(key, str):
            raise ConfigError(location, "keys must be strings")
        result[key] = item
    return result


def reject_unknown(value: dict[str, Any], known: set[str], location: str) -> None:
    unknown = sorted(set(value) - known)
    if unknown:
        raise ConfigError(location, f"unknown field(s): {', '.join(unknown)}")


def require(value: dict[str, Any], name: str, location: str) -> Any:
    if name not in value:
        raise ConfigError(f"{location}.{name}", "field is required")
    return value[name]


def text(value: object, location: str) -> str:
    if not isinstance(value, str) or not value:
        raise ConfigError(location, "must be a non-empty string")
    return value


def boolean(value: object, location: str) -> bool:
    if not isinstance(value, bool):
        raise ConfigError(location, "must be a boolean")
    return value


def integer(value: object, location: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ConfigError(location, "must be an integer")
    if not minimum <= value <= maximum:
        raise ConfigError(location, f"must be within [{minimum}, {maximum}]")
    return value


def number(
    value: object, location: str, *, minimum: float = 0.0, maximum: float = float("inf")
) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ConfigError(location, "must be a finite number")
    result = float(value)
    if not math.isfinite(result) or not minimum <= result <= maximum:
        raise ConfigError(location, f"must be a finite number within [{minimum}, {maximum}]")
    return result


def path_value(value: object, location: str) -> Path:
    return Path(text(value, location))


def name(value: object, location: str) -> str:
    result = text(value, location)
    if _NAME.fullmatch(result) is None:
        raise ConfigError(location, "contains unsupported characters")
    return result


def sha256(value: object, location: str) -> str:
    result = text(value, location)
    if _SHA256.fullmatch(result) is None:
        raise ConfigError(location, "must be a lowercase SHA-256 digest")
    return result


def names(value: object, location: str, *, count: int | None = None) -> tuple[str, ...]:
    if not isinstance(value, list):
        raise ConfigError(location, "must be an array of names")
    result = tuple(name(item, f"{location}[{index}]") for index, item in enumerate(value))
    if len(set(result)) != len(result):
        raise ConfigError(location, "must be unique")
    if count is not None and len(result) != count:
        raise ConfigError(location, f"must contain exactly {count} names")
    return result
