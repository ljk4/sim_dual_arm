"""Apply ``--set dot.path=value`` overrides to configuration documents.

The tool is deliberately tiny: it only walks existing mapping keys, parses the
value with ``yaml.safe_load`` and never creates new structure.  Every caller
still validates the modified document with its own strict Pydantic model, so an
override can never silently introduce an unknown field or a wrong type.
"""

from __future__ import annotations

import copy
from collections.abc import Mapping, Sequence
from typing import Any

import yaml

__all__ = ["OverrideError", "apply_overrides", "parse_override"]


class OverrideError(ValueError):
    """One ``--set`` expression is malformed or targets an absent path."""


def parse_override(spec: str) -> tuple[tuple[str, ...], Any]:
    """Split ``dot.path=value`` and parse the value as YAML."""
    if not isinstance(spec, str) or "=" not in spec:
        raise OverrideError(f"override must be 'dot.path=value': {spec!r}")
    path_text, raw_value = spec.split("=", 1)
    path = tuple(path_text.split("."))
    if not path_text or any(not part or part == ".." for part in path):
        raise OverrideError(f"override path is empty or unsafe: {path_text!r}")
    try:
        value = yaml.safe_load(raw_value)
    except yaml.YAMLError as error:
        raise OverrideError(f"override {path_text!r} has invalid YAML: {error}") from error
    return path, value


def apply_overrides(
    document: Mapping[str, Any], overrides: Sequence[str]
) -> dict[str, Any]:
    """Return a deep copy of ``document`` with every override applied in order."""
    result = copy.deepcopy(dict(document))
    for spec in overrides:
        path, value = parse_override(spec)
        _set_path(result, path, value)
    return result


def _set_path(document: dict[str, Any], path: tuple[str, ...], value: Any) -> None:
    node: Any = document
    for index, part in enumerate(path):
        if not isinstance(node, dict) or part not in node:
            raise OverrideError(
                "override path does not exist: " + ".".join(path[: index + 1])
            )
        if index == len(path) - 1:
            node[part] = value
            return
        node = node[part]
