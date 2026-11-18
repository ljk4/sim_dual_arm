"""Paths to the robot models shipped with this simulation project.

Every robot model is resolved from this end's own ``assets/`` root, so the
host layout (``simulation/mujoco/assets``) and the container mount
(``/app/assets``) both work without scenario-relative paths.
"""

from pathlib import Path

ASSET_ROOT = Path(__file__).resolve().parents[4] / "assets"
MODEL_DIR = ASSET_ROOT / "franka_emika_panda"
MARVIN_URDF = ASSET_ROOT / "marvin/urdf/Marvin_Stand_M6_2026.4.8.urdf"

_ROBOT_MODELS = {
    "panda": MODEL_DIR / "panda.xml",
    "marvin": MARVIN_URDF,
}


def model_paths() -> tuple[Path, Path]:
    return MODEL_DIR / "scene.xml", MODEL_DIR / "panda.xml"


def robot_model_path(assembly: str) -> Path:
    """Resolve the fixed-version model selected by ``robot.assembly``."""
    try:
        return _ROBOT_MODELS[assembly]
    except KeyError as error:
        raise KeyError(f"unsupported robot assembly: {assembly!r}") from error
