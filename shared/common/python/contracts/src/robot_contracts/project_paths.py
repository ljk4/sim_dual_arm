"""Resolve repository-relative inputs and ``output/<robot>/<backend>/<task>`` artifact roots."""

from __future__ import annotations

import os
import re
from pathlib import Path

DIRECT_SEGMENT = "_direct"
RESERVED_ROBOT_SEGMENTS = frozenset({"build"})
SEGMENT_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}")

__all__ = [
    "DIRECT_SEGMENT",
    "RESERVED_ROBOT_SEGMENTS",
    "SEGMENT_PATTERN",
    "container_data_root",
    "debug_root",
    "logs_root",
    "project_path",
    "recordings_root",
    "resolve_shared_root",
    "runs_root",
    "scenario_root",
    "scenario_segment",
]


def _safe(segment: str, label: str) -> str:
    if not SEGMENT_PATTERN.fullmatch(segment) or ".." in segment:
        raise ValueError(
            f"{label} must contain 1..128 safe ASCII characters and no path traversal"
        )
    if label == "robot" and segment in RESERVED_ROBOT_SEGMENTS:
        raise ValueError(f"robot id {segment!r} is reserved")
    return segment


def _triple(
    robot: str | None, backend: str | None, task: str | None
) -> tuple[str, str | None, str | None]:
    if robot is None:
        if backend is not None or task is not None:
            raise ValueError("backend and task require a robot id")
        return DIRECT_SEGMENT, None, None
    if backend is None and task is None:
        if "/" not in robot:
            raise ValueError(
                "backend and task are required unless robot is '<robot>/<backend>/<task>'"
            )
        parts = robot.split("/")
        if len(parts) != 3:
            raise ValueError("scenario string must be '<robot>/<backend>/<task>'")
        robot, backend, task = parts
    elif backend is None or task is None:
        raise ValueError("robot, backend, and task must be passed together")
    return _safe(robot, "robot"), _safe(backend, "backend"), _safe(task, "task")


def scenario_segment(
    robot: str | None, backend: str | None = None, task: str | None = None
) -> str:
    """Return the safe ``<robot>/<backend>/<task>`` directory segment, or ``_direct``."""
    head, middle, tail = _triple(robot, backend, task)
    if middle is None or tail is None:
        return head
    return f"{head}/{middle}/{tail}"


def container_data_root() -> Path:
    return Path(os.environ.get("DATA_ROOT", "/data"))


def scenario_root(
    base: Path, robot: str | None, backend: str | None = None, task: str | None = None
) -> Path:
    return Path(base) / scenario_segment(robot, backend, task)


def logs_root(
    base: Path, robot: str | None, backend: str | None = None, task: str | None = None
) -> Path:
    return scenario_root(base, robot, backend, task) / "logs"


def debug_root(
    base: Path, robot: str | None, backend: str | None = None, task: str | None = None
) -> Path:
    return scenario_root(base, robot, backend, task) / "debug"


def runs_root(
    robot: str | None,
    backend: str | None = None,
    task: str | None = None,
    base: Path | None = None,
) -> Path:
    root = container_data_root() if base is None else Path(base)
    return logs_root(root, robot, backend, task) / "runs"


def recordings_root(
    robot: str | None,
    backend: str | None = None,
    task: str | None = None,
    base: Path | None = None,
) -> Path:
    root = container_data_root() if base is None else Path(base)
    return logs_root(root, robot, backend, task) / "recordings"


def resolve_shared_root(
    explicit: Path | None = None, project_root: Path | None = None
) -> Path:
    """Resolve the ``shared/`` root from an explicit path, ``SHARED_ROOT``, or project layout."""
    if explicit is not None:
        return Path(explicit)
    environment = os.environ.get("SHARED_ROOT")
    if environment:
        return Path(environment)
    if project_root is not None:
        candidate = Path(project_root) / "shared"
        if candidate.is_dir():
            return candidate
    here = Path(__file__).resolve()
    # shared/common/python/contracts/src/robot_contracts/project_paths.py -> shared/
    for candidate in (here.parents[4], here.parents[5] / "shared", Path("/shared")):
        if (candidate / "common/interfaces/schemas").is_dir():
            return candidate
    return Path("/shared")


def project_path(value: str | Path, project_root: Path) -> Path:
    """Use declared mount roots for the active implementation, or the host tree."""
    path = Path(value)
    if path.is_absolute():
        return path
    root = Path(project_root).resolve()
    container_root = os.environ.get("PROJECT_ROOT")
    if (
        os.environ.get("PROJECT_CONTAINER") == "1"
        and container_root
        and (root == Path(container_root).resolve())
    ):
        if path.parts[:2] == ("output", "build"):
            build_root = Path(os.environ.get("BUILD_ROOT", "/build"))
            return (build_root / Path(*path.parts[2:])).resolve()
        mounts = {
            "shared": Path(os.environ.get("SHARED_ROOT", "/shared")),
            "output": Path(os.environ.get("DATA_ROOT", "/data")),
            "third_party": Path(os.environ.get("THIRD_PARTY_ROOT", "/third_party")),
        }
        if path.parts and path.parts[0] in mounts:
            return (mounts[path.parts[0]] / Path(*path.parts[1:])).resolve()
        prefix = Path(os.environ.get("PROJECT_SOURCE_PREFIX", ".")).parts
        if prefix and path.parts[: len(prefix)] == prefix:
            return (root / Path(*path.parts[len(prefix) :])).resolve()
    return (root / path).resolve()
