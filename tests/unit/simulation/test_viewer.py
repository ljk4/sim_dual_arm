"""Unit tests for the simplified live MuJoCo viewer CLI and scenario lookup."""

from __future__ import annotations

import shutil
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.common.config.scenario_config import (  # noqa: E402
    ScenarioError,
    resolve_scenario_viewer,
)
from robot_backend_sim.common.render import viewer  # noqa: E402


def test_parser_accepts_only_robot_and_task():
    args = viewer._parser().parse_args(["--robot", "marvin_sim", "--task", "stack"])
    assert vars(args) == {"robot": "marvin_sim", "task": "stack"}


@pytest.mark.parametrize(
    "argv",
    [
        [],
        ["--robot", "marvin_sim"],
        ["--task", "stack"],
        ["--robot", "marvin_sim", "--task", "stack", "--role", "teacher"],
        ["--robot", "marvin_sim", "--task", "stack", "--config", "foo.yaml"],
        ["--robot", "marvin_sim", "--task", "stack", "--shared-root", "/shared"],
        ["--robot", "marvin_sim", "--task", "stack", "--state-topic", "/x"],
        ["--robot", "marvin_sim", "--task", "stack", "--poll-hz", "30"],
    ],
)
def test_parser_rejects_missing_required_or_removed_arguments(argv: list[str]):
    with pytest.raises(SystemExit) as caught:
        viewer._parser().parse_args(argv)
    assert caught.value.code == 2


@pytest.mark.parametrize("task", ["stack", "handover"])
def test_load_backend_resolves_marvin_scenarios(task: str):
    backend = viewer._load_backend("marvin_sim", task)
    assert backend.visualization_enabled is True
    assert backend.namespace == "/robot"
    assert backend.scene_path.is_file()


def test_resolve_scenario_viewer_falls_back_to_simulation_policy(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
):
    shared = tmp_path / "shared"
    shutil.copytree(ROOT / "shared/common/interfaces/schemas", shared / "common/interfaces/schemas")
    shutil.copytree(ROOT / "shared/scenarios/marvin_sim", shared / "scenarios/marvin_sim")
    binding_path = shared / "scenarios/marvin_sim/stack/binding.yaml"
    document = yaml.safe_load(binding_path.read_text(encoding="utf-8"))
    del document["recipes"]["simulation_teacher"]
    binding_path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")

    monkeypatch.setenv("SHARED_ROOT", str(shared))
    resolved = resolve_scenario_viewer(robot="marvin_sim", task="stack")
    assert resolved.selection["role"] == "policy"

    del document["recipes"]["simulation_policy"]
    binding_path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    with pytest.raises(ScenarioError, match="declares no simulation recipe"):
        resolve_scenario_viewer(robot="marvin_sim", task="stack")
