"""Load one simulation scene recipe by its explicit robot assembly selection.

Both scene recipes declare ``robot.assembly``.  The value selects the strict
configuration model and the scene/IO adapter; a task name or file location is
never used to infer the robot.
"""

from __future__ import annotations

from pathlib import Path

from robot_contracts.validation import StrictMappingError, UniqueKeyError, read_unique_yaml

from robot_backend_sim.config.marvin_scene_config import (
    MarvinSceneConfig,
    load_marvin_scene_config,
)
from robot_backend_sim.config.scene_config import SceneConfig, load_scene_config

__all__ = ["SceneConfigLike", "load_scene_config_for"]

SceneConfigLike = SceneConfig | MarvinSceneConfig
_LOADERS = {
    "panda": load_scene_config,
    "marvin": load_marvin_scene_config,
}


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
        loader = _LOADERS[assembly]
    except KeyError as error:
        raise StrictMappingError(
            source, "robot.assembly", f"unsupported robot assembly {assembly!r}"
        ) from error
    return loader(source)
