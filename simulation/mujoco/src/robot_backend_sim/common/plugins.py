"""Convention-based dynamic discovery for simulation robot and task plugins.

Every robot package under ``robot_backend_sim/<robot-id>/`` exposes a
lightweight ``plugin.py`` defining ``ROBOT_PLUGIN`` (which carries its
``tasks`` descriptors).  ``common/`` discovers robot and task plugins
dynamically without hardcoding any robot or task names and without importing
MuJoCo at discovery time.
"""

from __future__ import annotations

import importlib
from collections.abc import Callable
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path
from typing import Any

__all__ = [
    "RobotPlugin",
    "TaskPluginEntry",
    "get_robot_plugin",
    "get_task_plugin",
    "robot_plugins",
    "supported_task_types",
    "task_plugins",
]

_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
_EXCLUDED_DIRS = frozenset({"common", "tools"})


@dataclass(frozen=True, slots=True)
class TaskPluginEntry:
    """Lazy task plugin factory exposed by a robot plugin."""

    task_type: str
    create: Callable[..., Any]


@dataclass(frozen=True, slots=True)
class RobotPlugin:
    """Registration descriptor exposed by one ``<robot-id>/plugin.py`` module."""

    assembly: str
    model_path: Path
    load_scene_config: Callable[[Path], Any]
    build_scene: Callable[[Any], Any]
    build_io: Callable[..., Any]
    tasks: tuple[TaskPluginEntry, ...] = ()


def _robot_dirs() -> tuple[Path, ...]:
    return tuple(
        sorted(
            entry
            for entry in _PACKAGE_ROOT.iterdir()
            if entry.is_dir()
            and not entry.name.startswith(("_", "."))
            and entry.name not in _EXCLUDED_DIRS
        )
    )


@lru_cache(maxsize=1)
def robot_plugins() -> dict[str, RobotPlugin]:
    discovered: dict[str, RobotPlugin] = {}
    for robot_dir in _robot_dirs():
        if not (robot_dir / "plugin.py").is_file():
            continue
        module = importlib.import_module(f"robot_backend_sim.{robot_dir.name}.plugin")
        plugin = getattr(module, "ROBOT_PLUGIN", None)
        if not isinstance(plugin, RobotPlugin):
            raise RuntimeError(
                f"robot_backend_sim.{robot_dir.name}.plugin must export ROBOT_PLUGIN"
            )
        if plugin.assembly in discovered:
            raise RuntimeError(f"duplicate robot assembly registration: {plugin.assembly!r}")
        discovered[plugin.assembly] = plugin
    return discovered


@lru_cache(maxsize=1)
def task_plugins() -> dict[str, TaskPluginEntry]:
    discovered: dict[str, TaskPluginEntry] = {}
    for plugin in robot_plugins().values():
        for entry in plugin.tasks:
            if entry.task_type in discovered:
                raise RuntimeError(f"duplicate task plugin registration: {entry.task_type!r}")
            discovered[entry.task_type] = entry
    return discovered


def get_robot_plugin(assembly: str) -> RobotPlugin:
    plugins = robot_plugins()
    try:
        return plugins[assembly]
    except KeyError as error:
        raise KeyError(f"unsupported robot assembly: {assembly!r}") from error


def supported_task_types() -> tuple[str, ...]:
    return tuple(sorted(task_plugins()))


def get_task_plugin(task_type: str) -> TaskPluginEntry:
    plugins = task_plugins()
    try:
        return plugins[task_type]
    except KeyError as error:
        raise ValueError(f"unknown simulation task type: {task_type!r}") from error
