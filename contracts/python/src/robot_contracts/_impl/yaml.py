"""Unique-key YAML parsing shared by every configuration module."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from ..validation import UniqueKeyError


class UniqueKeyYamlLoader(yaml.SafeLoader):
    source_name = "<document>"


def _construct_unique_mapping(loader: UniqueKeyYamlLoader, node: yaml.MappingNode,
                              deep: bool = False) -> dict[Any, Any]:
    loader.flatten_mapping(node)
    mapping: dict[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            if key in mapping:
                raise UniqueKeyError(loader.source_name, str(key), "duplicate key")
        except TypeError as error:
            raise UniqueKeyError(loader.source_name, "<mapping>",
                                 "keys must be hashable") from error
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


UniqueKeyYamlLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping)

def parse_unique_yaml(text: str, source: Path | str = "<document>") -> Any:
    loader = UniqueKeyYamlLoader(text)
    loader.source_name = str(source)
    try:
        return loader.get_single_data()
    except yaml.YAMLError as error:
        raise UniqueKeyError(source, "<yaml>", f"invalid YAML: {error}") from error


def read_unique_yaml(path: Path | str) -> Any:
    source = Path(path)
    try:
        return parse_unique_yaml(source.read_text(encoding="utf-8"), source)
    except (OSError, UnicodeError) as error:
        raise UniqueKeyError(source, "<file>", str(error)) from error
