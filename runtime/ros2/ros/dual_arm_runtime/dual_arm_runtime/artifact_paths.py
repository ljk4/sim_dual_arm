"""Runtime-specific default teacher backend log lookup under ``var/<robot>/<task>``."""

from __future__ import annotations

from pathlib import Path

from robot_contracts.project_paths import container_data_root, logs_root


def default_backend_log(robot: str | None, task: str | None = None) -> Path:
    backend_root = logs_root(container_data_root(), robot, task) / "backend"
    if backend_root.is_dir():
        candidates = [
            child / "log"
            for child in backend_root.iterdir()
            if child.is_dir()
            and (child / "log").is_dir()
            and (
                child.name.startswith("teacher-backend")
                or "-teacher" in child.name
                or "_teacher" in child.name
            )
        ]
        if candidates:
            candidates.sort(key=lambda item: item.parent.stat().st_mtime_ns, reverse=True)
            return candidates[0]
    return backend_root / "teacher-backend" / "log"


__all__ = ["default_backend_log"]
