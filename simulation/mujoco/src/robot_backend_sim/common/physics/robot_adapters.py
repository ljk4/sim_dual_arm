"""Explicit robot adaptation registration for the simulation backend.

Scene recipes select one assembly (``robot.assembly``).  This module maps that
selection to its scene builder and named I/O implementation.  The generic
worker, controller and motion loop consume the returned capabilities and never
branch on a robot name.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import cast

from robot_contracts.models import RobotProfile

from robot_backend_sim.marvin_sim.handover.scene_config import MarvinSceneConfig
from robot_backend_sim.panda_sim.stack.scene_config import SceneConfig
from robot_backend_sim.marvin_sim.io import MarvinIO
from robot_backend_sim.marvin_sim.handover.scene import build_marvin_scene
from robot_backend_sim.panda_sim.io import PandaIO
from robot_backend_sim.panda_sim.stack.scene import build_scene as build_panda_scene
from robot_backend_sim.common.physics.robot_io import RobotIO
from robot_backend_sim.common.physics.scene_container import CompiledScene, SceneBuildError

__all__ = [
    "SceneConfigLike",
    "build_io_for",
    "build_scene_for",
    "scene_assembly",
]

SceneConfigLike = SceneConfig | MarvinSceneConfig


@dataclass(frozen=True)
class _Adapter:
    build_scene: Callable[[object], CompiledScene]
    build_io: Callable[..., RobotIO]


_ADAPTERS: dict[str, _Adapter] = {
    "panda": _Adapter(build_panda_scene, PandaIO),
    "marvin": _Adapter(build_marvin_scene, MarvinIO),
}


def scene_assembly(config: SceneConfigLike) -> str:
    try:
        assembly = config.robot.assembly
    except AttributeError as error:
        raise SceneBuildError("scene configuration does not select a robot assembly") from error
    if assembly not in _ADAPTERS:
        raise SceneBuildError(f"unsupported robot assembly {assembly!r}")
    return assembly


def _adapter(assembly: str) -> _Adapter:
    try:
        return _ADAPTERS[assembly]
    except KeyError as error:
        raise SceneBuildError(f"unsupported robot assembly {assembly!r}") from error


def build_scene_for(config: SceneConfigLike) -> CompiledScene:
    return _adapter(scene_assembly(config)).build_scene(config)


def build_io_for(
    scene: CompiledScene, profile: RobotProfile, *, velocity_feedforward: bool = False
) -> RobotIO:
    assembly = scene_assembly(cast(SceneConfigLike, scene.config))
    return _adapter(assembly).build_io(
        scene, profile, velocity_feedforward=velocity_feedforward
    )
