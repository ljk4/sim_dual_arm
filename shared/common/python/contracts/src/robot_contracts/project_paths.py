"""Resolve repository-relative inputs against the mounted container roots."""

from __future__ import annotations

import os
from pathlib import Path


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
        mounts = {
            "shared": Path(os.environ.get("SHARED_ROOT", "/shared")),
            "var": Path(os.environ.get("DATA_ROOT", "/data")),
            "build": Path(os.environ.get("BUILD_ROOT", "/build")),
            "third_party": Path(os.environ.get("THIRD_PARTY_ROOT", "/third_party")),
        }
        if path.parts and path.parts[0] in mounts:
            return (mounts[path.parts[0]] / Path(*path.parts[1:])).resolve()
        prefix = Path(os.environ.get("PROJECT_SOURCE_PREFIX", ".")).parts
        if prefix and path.parts[: len(prefix)] == prefix:
            return (root / Path(*path.parts[len(prefix) :])).resolve()
    return (root / path).resolve()
