"""Explicit robot assembly selection at the simulation scene entry."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.config.marvin_scene_config import MarvinSceneConfig  # noqa: E402
from robot_backend_sim.config.scene_config import SceneConfig  # noqa: E402
from robot_backend_sim.config.scene_loader import load_scene_config_for  # noqa: E402
from robot_contracts.validation import StrictMappingError  # noqa: E402

PANDA_SCENE = (
    ROOT / "shared/scenarios/panda_stack/configs/simulation/scene/panda_stack.yaml"
)
MARVIN_SCENE = (
    ROOT / "shared/scenarios/marvin_handover/configs/simulation/scene/marvin_handover.yaml"
)


def test_each_scene_recipe_selects_its_own_assembly():
    panda = load_scene_config_for(PANDA_SCENE)
    marvin = load_scene_config_for(MARVIN_SCENE)
    assert isinstance(panda, SceneConfig)
    assert panda.robot.assembly == "panda"
    assert isinstance(marvin, MarvinSceneConfig)
    assert marvin.robot.assembly == "marvin"
    assert marvin.task.environment == "handover"


def test_unknown_or_missing_assembly_is_rejected(tmp_path):
    text = PANDA_SCENE.read_text(encoding="utf-8").replace(
        "  assembly: panda", "  assembly: unknown"
    )
    unknown = tmp_path / "unknown.yaml"
    unknown.write_text(text, encoding="utf-8")
    with pytest.raises(StrictMappingError) as caught:
        load_scene_config_for(unknown)
    assert caught.value.field == "robot.assembly"

    missing = tmp_path / "missing.yaml"
    missing.write_text("physics: {}\n", encoding="utf-8")
    with pytest.raises(StrictMappingError) as caught:
        load_scene_config_for(missing)
    assert caught.value.field == "robot.assembly"
