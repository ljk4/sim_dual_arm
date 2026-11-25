"""Robot and task plugin registration and model asset paths for Marvin."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from robot_contracts.validation import StrictMappingError, UniqueKeyError, read_unique_yaml

from robot_backend_sim.common.physics.assets import robots_asset_root
from robot_backend_sim.common.plugins import RobotPlugin, TaskPluginEntry

__all__ = ["MARVIN_URDF", "ROBOT_PLUGIN"]

_MARVIN_URDF_REL = Path("marvin/urdf/Marvin_Stand_M6_2026.4.8.urdf")
MARVIN_URDF = robots_asset_root() / _MARVIN_URDF_REL


def _load_scene_config(path: Path) -> Any:
    source = Path(path)
    try:
        document = read_unique_yaml(source)
    except UniqueKeyError as error:
        raise StrictMappingError(error.source, error.field, error.message) from error
    if not isinstance(document, dict):
        raise StrictMappingError(source, "<root>", "must be a mapping")
    task = document.get("task")
    environment = task.get("environment") if isinstance(task, dict) else None
    if environment == "handover":
        from robot_backend_sim.marvin.handover.scene_config import load_marvin_scene_config

        return load_marvin_scene_config(source)
    if environment == "stack":
        from robot_backend_sim.marvin.stack.scene_config import load_marvin_stack_scene_config

        return load_marvin_stack_scene_config(source)
    raise StrictMappingError(
        source,
        "task.environment",
        f"unsupported Marvin task environment {environment!r}",
    )


def _build_scene(config: Any) -> Any:
    environment = getattr(getattr(config, "task", None), "environment", None)
    if environment == "handover":
        from robot_backend_sim.marvin.handover.scene import build_marvin_scene

        return build_marvin_scene(config)
    if environment == "stack":
        from robot_backend_sim.marvin.stack.scene import build_marvin_stack_scene

        return build_marvin_stack_scene(config)
    raise ValueError(f"unsupported Marvin task environment: {environment!r}")


def _build_io(scene: Any, profile: Any, *, velocity_feedforward: bool = False) -> Any:
    from robot_backend_sim.marvin.io import MarvinIO

    return MarvinIO(scene, profile, velocity_feedforward=velocity_feedforward)


def _create_handover_plugin(
    task_path: object, *, task_mode: str | None, scene_config: object
) -> Any:
    from robot_backend_sim.marvin.handover.plugin import HandoverTaskPlugin

    return HandoverTaskPlugin.from_path(
        task_path, task_mode=task_mode, scene_config=scene_config
    )


def _create_stack_plugin(
    task_path: object, *, task_mode: str | None, scene_config: object
) -> Any:
    from robot_backend_sim.marvin.stack.plugin import StackTaskPlugin

    return StackTaskPlugin.from_path(
        task_path, task_mode=task_mode, scene_config=scene_config
    )


ROBOT_PLUGIN = RobotPlugin(
    assembly="marvin",
    model_path=_MARVIN_URDF_REL,
    load_scene_config=_load_scene_config,
    build_scene=_build_scene,
    build_io=_build_io,
    tasks=(
        TaskPluginEntry(task_type="handover", create=_create_handover_plugin),
        TaskPluginEntry(task_type="stack", create=_create_stack_plugin),
    ),
)
