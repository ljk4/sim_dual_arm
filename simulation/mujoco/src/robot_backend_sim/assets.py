"""Paths to the Panda model shipped with this simulation project."""

from pathlib import Path

MODEL_DIR = Path(__file__).resolve().parents[2] / "assets/franka_emika_panda"


def model_paths() -> tuple[Path, Path]:
    return MODEL_DIR / "scene.xml", MODEL_DIR / "panda.xml"
