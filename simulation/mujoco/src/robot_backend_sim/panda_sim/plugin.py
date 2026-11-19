"""Robot and task plugin registration and model asset paths for Panda."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from robot_backend_sim.common.physics.assets import ASSET_ROOT
from robot_backend_sim.common.plugins import RobotPlugin, TaskPluginEntry

__all__ = ["PANDA_MODEL_DIR", "ROBOT_PLUGIN", "panda_model_paths"]

PANDA_MODEL_DIR = ASSET_ROOT / "franka_emika_panda"


def panda_model_paths() -> tuple[Path, Path]:
    return PANDA_MODEL_DIR / "scene.xml", PANDA_MODEL_DIR / "panda.xml"


def _load_scene_config(path: Path) -> Any:
    from robot_backend_sim.panda_sim.stack.scene_config import load_scene_config

    return load_scene_config(path)


def _build_scene(config: Any) -> Any:
    from robot_backend_sim.panda_sim.stack.scene import build_scene

    return build_scene(config)


def _build_io(scene: Any, profile: Any, *, velocity_feedforward: bool = False) -> Any:
    from robot_backend_sim.panda_sim.io import PandaIO

    return PandaIO(scene, profile, velocity_feedforward=velocity_feedforward)


def _create_stack_plugin(
    task_path: object, *, task_mode: str | None, scene_config: object
) -> Any:
    from robot_backend_sim.panda_sim.stack import StackTaskPlugin

    return StackTaskPlugin.from_path(
        task_path, task_mode=task_mode, scene_config=scene_config
    )


ROBOT_PLUGIN = RobotPlugin(
    assembly="panda",
    model_path=PANDA_MODEL_DIR / "panda.xml",
    load_scene_config=_load_scene_config,
    build_scene=_build_scene,
    build_io=_build_io,
    tasks=(TaskPluginEntry(task_type="stack", create=_create_stack_plugin),),
)
