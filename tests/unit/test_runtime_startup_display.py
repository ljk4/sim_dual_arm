"""Startup output must consume the actual validated configuration interface."""

import argparse
import os
from pathlib import Path

import pytest
from dual_arm_runtime.runtime_cli import _print_configuration_summary
from dual_arm_runtime.runtime_config import load_runtime_configuration_document
from dual_arm_runtime.scenario_runtime import resolve_scenario_runtime


@pytest.mark.parametrize(
    ("robot", "task"), [("marvin_sim", "stack"), ("marvin_sim", "handover")]
)
def test_teacher_startup_prints_real_loaded_configuration(robot, task, capsys):
    root = Path(os.environ.get("PROJECT_ROOT", Path(__file__).resolve().parents[2]))
    resolved = resolve_scenario_runtime(
        robot=robot, task=task, role="teacher", project_root=root
    )
    configuration = load_runtime_configuration_document(
        resolved.document,
        source=resolved.selection["recipe"]["path"],
        base_dir=resolved.base_dir,
        project_root=root,
        selection=resolved.selection,
    )
    _print_configuration_summary(
        configuration,
        argparse.Namespace(output="/tmp/recording"),
        Path("/tmp/runs/run-1"),
    )
    output = capsys.readouterr().out
    assert configuration.configuration_source in output
    assert configuration.recipe.robot_namespace in output
    assert "/tmp/runs/run-1/resolved/runtime.json" in output
    assert '"recipe"' not in output


def test_direct_runtime_config_uses_declared_shared_mount(monkeypatch):
    from dual_arm_runtime.runtime_config import load_runtime_configuration

    root = Path(os.environ.get("PROJECT_ROOT", Path(__file__).resolve().parents[2]))
    monkeypatch.setenv("PROJECT_CONTAINER", "1")
    monkeypatch.setenv("PROJECT_ROOT", str(root))
    resolved = resolve_scenario_runtime(
        robot="marvin_sim", task="stack", role="teacher", project_root=root
    )
    config = load_runtime_configuration(
        Path("shared/scenarios/marvin_sim/stack/configs/runtime/teacher_collect.yaml"), root
    )
    assert config.configuration_source == str(resolved.selection["recipe"]["path"])
    assert config.recipe.action_source == "teacher"
