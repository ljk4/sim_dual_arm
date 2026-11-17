"""Single source for ``var/<robot>/<task>`` runtime artifact roots.

Every runtime output lives under ``<base>/<robot>/<task>/{datasets,models,logs,
debug}`` (container ``/data/<robot>/<task>/...``).  Direct configuration mode
has no robot/task pair and uses the reserved ``_direct`` segment instead of
silently falling back to a real scenario.

A caller may pass the pair as ``(robot, task)`` or as a single ``"robot/task"``
string; both forms resolve to the same directory.
"""

from __future__ import annotations

import os
import re
from pathlib import Path

DIRECT_SEGMENT = "_direct"
_SEGMENT_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}")


def _safe(segment: str, label: str) -> str:
    if not _SEGMENT_PATTERN.fullmatch(segment) or ".." in segment:
        raise ValueError(
            f"{label} must contain 1..128 safe ASCII characters and no path traversal"
        )
    return segment


def _pair(robot: str | None, task: str | None) -> tuple[str, str | None]:
    if robot is None:
        if task is not None:
            raise ValueError("task requires a robot id")
        return DIRECT_SEGMENT, None
    if task is None:
        if "/" not in robot:
            raise ValueError("task is required unless robot is '<robot>/<task>'")
        robot, task = robot.split("/", 1)
    return _safe(robot, "robot"), _safe(task, "task")


def scenario_segment(robot: str | None, task: str | None = None) -> str:
    """Return the safe ``<robot>/<task>`` directory segment, or ``_direct``."""
    head, tail = _pair(robot, task)
    return head if tail is None else f"{head}/{tail}"


def container_data_root() -> Path:
    return Path(os.environ.get("DATA_ROOT", "/data"))


def scenario_root(base: Path, robot: str | None, task: str | None = None) -> Path:
    return Path(base) / scenario_segment(robot, task)


def datasets_root(base: Path, robot: str | None, task: str | None = None) -> Path:
    return scenario_root(base, robot, task) / "datasets"


def raw_datasets_root(base: Path, robot: str | None, task: str | None = None) -> Path:
    return datasets_root(base, robot, task) / "raw"


def logs_root(base: Path, robot: str | None, task: str | None = None) -> Path:
    return scenario_root(base, robot, task) / "logs"


def debug_root(base: Path, robot: str | None, task: str | None = None) -> Path:
    return scenario_root(base, robot, task) / "debug"


def runs_root(robot: str | None, task: str | None = None) -> Path:
    return logs_root(container_data_root(), robot, task) / "runs"


def recordings_root(robot: str | None, task: str | None = None) -> Path:
    return logs_root(container_data_root(), robot, task) / "recordings"


def default_backend_log(robot: str | None, task: str | None = None) -> Path:
    backend_root = logs_root(container_data_root(), robot, task) / "backend"
    if backend_root.is_dir():
        candidates = [
            child / "log"
            for child in backend_root.iterdir()
            if child.is_dir()
            and (child / "log").is_dir()
            and (child.name.startswith("teacher-backend") or "-teacher" in child.name)
        ]
        if candidates:
            candidates.sort(key=lambda item: item.parent.stat().st_mtime_ns, reverse=True)
            return candidates[0]
    return backend_root / "teacher-backend" / "log"


__all__ = [
    "DIRECT_SEGMENT",
    "container_data_root",
    "datasets_root",
    "debug_root",
    "default_backend_log",
    "logs_root",
    "raw_datasets_root",
    "recordings_root",
    "runs_root",
    "scenario_root",
    "scenario_segment",
]
