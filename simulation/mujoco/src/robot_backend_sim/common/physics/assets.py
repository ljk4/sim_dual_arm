"""Paths to the robot models shipped with this simulation project.

Every robot model is resolved from this end's own ``assets/`` root through the
robot plugin registry, so ``common/`` never hardcodes any robot model path.
"""

from __future__ import annotations

from pathlib import Path

from robot_backend_sim.common.plugins import get_robot_plugin

__all__ = ["ASSET_ROOT", "robot_model_path"]

ASSET_ROOT = Path(__file__).resolve().parents[4] / "assets"


def robot_model_path(assembly: str) -> Path:
    """Resolve the fixed-version model selected by ``robot.assembly``."""
    return get_robot_plugin(assembly).model_path
