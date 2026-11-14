"""All learning entry points print the real scenario selection interface."""

import os
from pathlib import Path

import pytest
from dual_arm_learning.cli import _print_effective_config
from dual_arm_learning.config import load_dataset_config, load_training_config
from dual_arm_learning.scenario import select_scenario


@pytest.mark.parametrize("scenario", ["panda_stack", "marvin_handover"])
@pytest.mark.parametrize("field", ["dataset", "training"])
def test_learning_startup_prints_real_loaded_selection(scenario, field, capsys):
    root = Path(os.environ.get("PROJECT_ROOT", Path(__file__).resolve().parents[2]))
    selection = select_scenario(
        scenario=scenario, recipe_field=field, stage="learning", project_root=root
    )
    load = load_dataset_config if field == "dataset" else load_training_config
    config = load(selection.recipe_path, root)
    _print_effective_config("配置", selection.recipe_path, selection, config)
    output = capsys.readouterr().out
    assert scenario in output
    assert str(selection.resolved.path) in output
    assert str(selection.recipe_path) in output


def test_direct_service_config_reaches_loader_through_shared_mount(tmp_path, monkeypatch):
    from dual_arm_learning import policy_service_cli
    from dual_arm_learning.policy_config import PolicyConfigError

    app = tmp_path / "app"
    shared = tmp_path / "mounted-shared"
    app.mkdir()
    shared.mkdir()
    recipe = shared / "policy.yaml"
    recipe.write_text("service: recorded-config\n")
    monkeypatch.setenv("PROJECT_CONTAINER", "1")
    monkeypatch.setenv("PROJECT_ROOT", str(app))
    monkeypatch.setenv("SHARED_ROOT", str(shared))
    captured = {}

    def stop_after_loading(path, **kwargs):
        captured["contents"] = path.read_text()
        captured["path"] = path
        raise PolicyConfigError("configuration probe stops before model loading")

    monkeypatch.setattr(policy_service_cli, "load_policy_config", stop_after_loading)
    monkeypatch.setattr(policy_service_cli, "_fail", lambda *args: 2)
    assert policy_service_cli.main(["--config", "shared/policy.yaml"]) == 2
    assert captured == {"contents": recipe.read_text(), "path": recipe.resolve()}


def test_training_data_root_override_uses_data_mount(monkeypatch):
    import argparse

    from dual_arm_learning import cli

    root = Path(os.environ.get("PROJECT_ROOT", Path(__file__).resolve().parents[2]))
    monkeypatch.setenv("PROJECT_CONTAINER", "1")
    monkeypatch.setenv("PROJECT_ROOT", str(root))
    selection = select_scenario(
        scenario="panda_stack", recipe_field="training", stage="learning", project_root=root
    )
    args = argparse.Namespace(
        project_root=root,
        overrides=[],
        mode=None,
        data_root=Path("var/synthetic_stack/datasets/override"),
    )
    config = cli._training_config(args, selection.recipe_path)
    assert (
        config.data_root
        == Path(os.environ.get("DATA_ROOT", "/data")) / "synthetic_stack/datasets/override"
    )
