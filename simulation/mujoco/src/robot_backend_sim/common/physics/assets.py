"""Paths to the shared robot models under ``shared/robots/``.

Every robot model is resolved from the shared ``robots/`` tree through the
robot plugin registry, so ``common/`` never hardcodes any robot model path.
"""

from __future__ import annotations

from pathlib import Path

from robot_contracts.project_paths import resolve_shared_root

from robot_backend_sim.common.plugins import get_robot_plugin

__all__ = ["ASSET_ROOT", "robot_model_path", "robots_asset_root"]


def robots_asset_root() -> Path:
    """Return the shared ``robots/`` root containing canonical robot URDFs/meshes."""
    return resolve_shared_root() / "robots"


ASSET_ROOT = robots_asset_root()


def robot_model_path(assembly: str) -> Path:
    """Resolve the model selected by ``robot.assembly``."""
    path = Path(get_robot_plugin(assembly).model_path)
    return path if path.is_absolute() else robots_asset_root() / path
