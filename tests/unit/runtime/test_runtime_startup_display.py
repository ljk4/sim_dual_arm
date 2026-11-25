"""Startup output must consume the actual validated configuration interface."""

import argparse
import os
from pathlib import Path

import pytest
from dual_arm_runtime.runtime_cli import _print_configuration_summary
from dual_arm_runtime.runtime_config import load_runtime_configuration_document
from dual_arm_runtime.scenario_runtime import resolve_scenario_runtime


@pytest.mark.parametrize(
    ("robot", "backend", "task"),
    [("marvin", "mujoco", "stack"), ("marvin", "mujoco", "handover")],
)
def test_teacher_startup_prints_real_loaded_configuration(robot, backend, task, capsys):
    root = Path(os.environ.get("PROJECT_ROOT", Path(__file__).resolve().parents[3]))
    resolved = resolve_scenario_runtime(
        robot=robot, backend=backend, task=task, role="teacher", project_root=root
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

    root = Path(os.environ.get("PROJECT_ROOT", Path(__file__).resolve().parents[3]))
    monkeypatch.setenv("PROJECT_CONTAINER", "1")
    monkeypatch.setenv("PROJECT_ROOT", str(root))
    resolved = resolve_scenario_runtime(
        robot="marvin", backend="mujoco", task="stack", role="teacher", project_root=root
    )
    config = load_runtime_configuration(
        Path("shared/scenarios/marvin/mujoco/stack/configs/runtime/teacher_collect.yaml"), root
    )
    assert config.configuration_source == str(resolved.selection["recipe"]["path"])
    assert config.recipe.action_source == "teacher"


def test_interactive_cli_removes_run_id_and_generates_robot_backend_task_timestamp(
    tmp_path, capsys
):
    from dual_arm_runtime.runtime_cli import (
        _interactive_run_id_base,
        _parse_arguments,
        _unique_run_id,
    )

    with pytest.raises(SystemExit):
        _parse_arguments(
            [
                "--robot",
                "marvin",
                "--backend",
                "mujoco",
                "--task",
                "stack",
                "--once",
                "--confirm",
                "--run-id",
                "manual-id",
            ]
        )

    with pytest.raises(SystemExit):
        _parse_arguments(["--confirm", "--help"])
    help_output = capsys.readouterr().out
    assert "--run-id" not in help_output

    args, _ = _parse_arguments(
        ["--robot", "marvin", "--backend", "mujoco", "--task", "stack", "--once", "--confirm"]
    )
    assert args.run_id is None
    assert args.episode_id == ""

    policy_args, _ = _parse_arguments(
        [
            "--robot",
            "marvin",
            "--backend",
            "mujoco",
            "--task",
            "stack",
            "--once",
            "--run-id",
            "custom-run",
        ]
    )
    assert policy_args.run_id == "custom-run"

    root = Path(os.environ.get("PROJECT_ROOT", Path(__file__).resolve().parents[3]))
    resolved = resolve_scenario_runtime(
        robot="marvin", backend="mujoco", task="stack", role="policy", project_root=root
    )
    configuration = load_runtime_configuration_document(
        resolved.document,
        source=resolved.selection["recipe"]["path"],
        base_dir=resolved.base_dir,
        project_root=root,
        selection=resolved.selection,
    )
    base = _interactive_run_id_base(args, configuration, stamp="20261125_120000")
    assert base == "client_interactive_marvin_mujoco_stack_20261125_120000"

    direct_args, _ = _parse_arguments(
        [
            "--config",
            "shared/scenarios/marvin/mujoco/stack/configs/runtime/policy_rollout.yaml",
            "--once",
            "--confirm",
        ]
    )
    direct_base = _interactive_run_id_base(
        direct_args, configuration, stamp="20261125_120000"
    )
    assert direct_base == "client_interactive_marvin_marvin-stack_20261125_120000"

    out_dir = tmp_path / "recordings"
    runs_dir = tmp_path / "runs"
    (out_dir / f".partial-{base}").mkdir(parents=True)
    (runs_dir / f"{base}_2").mkdir(parents=True)
    assert _unique_run_id(out_dir, runs_dir, base) == f"{base}_3"


def test_runtime_cli_random_default_seed_and_explicit_override(monkeypatch, capsys):
    from dual_arm_runtime import runtime_cli

    root = Path(os.environ.get("PROJECT_ROOT", Path(__file__).resolve().parents[3]))
    monkeypatch.setattr(runtime_cli, "_random_seed", lambda: 654321)

    args_default, _ = runtime_cli._parse_arguments(
        [
            "--project-root",
            str(root),
            "--robot",
            "marvin",
            "--backend",
            "mujoco",
            "--task",
            "stack",
            "--once",
        ]
    )
    config_default = runtime_cli._load_configuration(args_default)
    assert config_default.recipe.reset.seed == 654321

    args_explicit, _ = runtime_cli._parse_arguments(
        [
            "--project-root",
            str(root),
            "--robot",
            "marvin",
            "--backend",
            "mujoco",
            "--task",
            "stack",
            "--once",
            "--reset-seed",
            "77",
        ]
    )
    config_explicit = runtime_cli._load_configuration(args_explicit)
    assert config_explicit.recipe.reset.seed == 77

    _print_configuration_summary(
        config_default,
        argparse.Namespace(output="/tmp/recording"),
        Path("/tmp/runs/run-1"),
    )
    output = capsys.readouterr().out
    assert "seed: 654321" in output
