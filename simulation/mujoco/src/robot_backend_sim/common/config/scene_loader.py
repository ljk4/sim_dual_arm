"""Load one simulation scene recipe by its explicit robot assembly selection.

Scene recipes declare ``robot.assembly``.  The value selects the strict
configuration model through :mod:`robot_backend_sim.common.plugins`; a task name
or file location is never used to infer the robot, and ``common/`` never
hardcodes any robot name.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from robot_contracts.validation import StrictMappingError, UniqueKeyError, read_unique_yaml

from robot_backend_sim.common.plugins import get_robot_plugin

__all__ = ["SceneConfigLike", "load_scene_config_for"]

SceneConfigLike = Any


def load_scene_config_for(path: Path | str) -> SceneConfigLike:
    source = Path(path)
    try:
        document = read_unique_yaml(source)
    except UniqueKeyError as error:
        raise StrictMappingError(error.source, error.field, error.message) from error
    if not isinstance(document, dict):
        raise StrictMappingError(source, "<root>", "must be a mapping")
    robot = document.get("robot")
    assembly = robot.get("assembly") if isinstance(robot, dict) else None
    if not isinstance(assembly, str):
        raise StrictMappingError(source, "robot.assembly", "must select a robot assembly")
    try:
        plugin = get_robot_plugin(assembly)
    except KeyError as error:
        raise StrictMappingError(
            source, "robot.assembly", f"unsupported robot assembly {assembly!r}"
        ) from error
    return plugin.load_scene_config(source)
