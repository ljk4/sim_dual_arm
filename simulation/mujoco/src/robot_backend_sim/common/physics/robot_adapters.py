"""Dynamic robot adaptation dispatch for the simulation backend.

Scene recipes select one assembly (``robot.assembly``).  This module resolves
that selection through :mod:`robot_backend_sim.common.plugins` without
hardcoding any robot name or importing any robot-specific package.
"""

from __future__ import annotations

from typing import Any

from robot_contracts.models import RobotProfile

from robot_backend_sim.common.physics.robot_io import RobotIO
from robot_backend_sim.common.physics.scene_container import CompiledScene, SceneBuildError
from robot_backend_sim.common.plugins import RobotPlugin, get_robot_plugin

__all__ = [
    "SceneConfigLike",
    "build_io_for",
    "build_scene_for",
    "scene_assembly",
]

SceneConfigLike = Any


def _plugin(assembly: str) -> RobotPlugin:
    try:
        return get_robot_plugin(assembly)
    except KeyError as error:
        raise SceneBuildError(f"unsupported robot assembly {assembly!r}") from error


def scene_assembly(config: SceneConfigLike) -> str:
    try:
        assembly = config.robot.assembly
    except AttributeError as error:
        raise SceneBuildError("scene configuration does not select a robot assembly") from error
    _plugin(assembly)
    return assembly


def build_scene_for(config: SceneConfigLike) -> CompiledScene:
    return _plugin(scene_assembly(config)).build_scene(config)


def build_io_for(
    scene: CompiledScene, profile: RobotProfile, *, velocity_feedforward: bool = False
) -> RobotIO:
    assembly = scene_assembly(scene.config)
    return _plugin(assembly).build_io(
        scene, profile, velocity_feedforward=velocity_feedforward
    )
