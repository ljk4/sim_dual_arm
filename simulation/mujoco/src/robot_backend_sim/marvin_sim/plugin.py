"""Robot and task plugin registration and model asset paths for Marvin."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from robot_backend_sim.common.physics.assets import ASSET_ROOT
from robot_backend_sim.common.plugins import RobotPlugin, TaskPluginEntry

__all__ = ["MARVIN_URDF", "ROBOT_PLUGIN"]

MARVIN_URDF = ASSET_ROOT / "marvin/urdf/Marvin_Stand_M6_2026.4.8.urdf"


def _load_scene_config(path: Path) -> Any:
    from robot_backend_sim.marvin_sim.handover.scene_config import load_marvin_scene_config

    return load_marvin_scene_config(path)


def _build_scene(config: Any) -> Any:
    from robot_backend_sim.marvin_sim.handover.scene import build_marvin_scene

    return build_marvin_scene(config)


def _build_io(scene: Any, profile: Any, *, velocity_feedforward: bool = False) -> Any:
    from robot_backend_sim.marvin_sim.io import MarvinIO

    return MarvinIO(scene, profile, velocity_feedforward=velocity_feedforward)


def _create_handover_plugin(
    task_path: object, *, task_mode: str | None, scene_config: object
) -> Any:
    from robot_backend_sim.marvin_sim.handover.plugin import HandoverTaskPlugin

    return HandoverTaskPlugin.from_path(
        task_path, task_mode=task_mode, scene_config=scene_config
    )


ROBOT_PLUGIN = RobotPlugin(
    assembly="marvin",
    model_path=MARVIN_URDF,
    load_scene_config=_load_scene_config,
    build_scene=_build_scene,
    build_io=_build_io,
    tasks=(TaskPluginEntry(task_type="handover", create=_create_handover_plugin),),
)
