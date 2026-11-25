"""Learning-side robot/task selection: provenance, conflicts and real dispatch."""

from __future__ import annotations

import json

import pytest
import yaml
from dual_arm_learning import cli
from dual_arm_learning.serving import cli as policy_service_cli
from learning_scenario_fixtures import (
    convert_args as _convert_args,
)
from learning_scenario_fixtures import (
    fake_convert as _fake_convert,
)
from learning_scenario_fixtures import (
    fake_train as _fake_train,
)
from learning_scenario_fixtures import (
    make_tree as _make_tree,
)
from learning_scenario_fixtures import (
    run_dispatcher as _run_dispatcher,
)
from learning_scenario_fixtures import (
    scenario_args as _scenario_args,
)
from learning_scenario_fixtures import (
    sha as _sha,
)


def test_convert_uses_selected_dataset_recipe(tmp_path, monkeypatch):
    tree = _make_tree(tmp_path)
    captured: dict = {}
    monkeypatch.setattr(cli, "convert", _fake_convert(captured))
    code = cli.main(_convert_args(tree, tmp_path))
    assert code == 0
    assert captured["config_path"] == tree.dataset_recipe.resolve()
    assert captured["config"].task_text == "scenario dataset task"
    assert captured["config"].profile_path == (tree.scenario.root / "profiles/robot.json")
    assert captured["selection"]["mode"] == "scenario"
    assert captured["selection"]["robot"] == "synthetic_sim"
    assert captured["selection"]["task"] == "stack"
    assert captured["selection"]["recipe"]["field"] == "recipes.dataset"
    assert captured["selection"]["recipe"]["path"] == str(tree.dataset_recipe.resolve())
    assert captured["selection"]["binding"]["sha256"] == _sha(tree.scenario.binding_path)
    assert captured["overrides"]["select"] == str(tmp_path / "select.json")
    assert captured["overrides"]["output_root"] == str(tmp_path / "out")
    assert captured["overrides"]["synthetic"] is True


def test_convert_requires_explicit_config_or_robot_task(tmp_path, monkeypatch):
    captured: dict = {}
    monkeypatch.setattr(cli, "convert", _fake_convert(captured))
    assert (
        cli.main(
            [
                "--project-root",
                str(tmp_path),
                "convert",
                "--select",
                str(tmp_path / "select.json"),
                "--output-root",
                str(tmp_path / "out"),
            ]
        )
        == 1
    )
    assert captured == {}

    tree = _make_tree(tmp_path)
    custom = tmp_path / "custom-dataset.yaml"
    document = yaml.safe_load(tree.dataset_recipe.read_text())
    document["task_text"] = "explicit custom task"
    custom.write_text(yaml.safe_dump(document, sort_keys=False))
    captured.clear()
    assert (
        cli.main(
            [
                "--project-root",
                str(tree.repo),
                "convert",
                "--config",
                str(custom),
                "--select",
                str(tmp_path / "select.json"),
                "--output-root",
                str(tmp_path / "out2"),
            ]
        )
        == 0
    )
    assert captured["config_path"] == custom.resolve()
    assert captured["config"].task_text == "explicit custom task"
    assert captured["selection"] is None


def test_train_uses_selected_training_recipe_and_canonical_roots(tmp_path, monkeypatch):
    tree = _make_tree(tmp_path)
    captured: dict = {}
    monkeypatch.setattr(cli, "train", _fake_train(captured))
    code = cli.main(_scenario_args(tree, "train", "--mode", "smoke"))
    assert code == 0
    config = captured["config"]
    assert config.source == tree.training_recipe.resolve()
    assert config.dataset_config_path == tree.dataset_recipe.resolve()
    assert config.dataset.task_text == "scenario dataset task"
    assert config.data_root == (
        tree.repo / "output/synthetic_sim/mujoco/stack/datasets/lerobot"
    ).resolve()
    assert config.checkpoint.root == (tree.repo / "output/synthetic_sim/mujoco/stack/models").resolve()
    assert config.logging.root == (
        tree.repo / "output/synthetic_sim/mujoco/stack/logs/training"
    ).resolve()
    selection = captured["kwargs"]["selection"]
    assert selection["recipe"]["field"] == "recipes.training"
    assert selection["dataset_recipe"]["path"] == str(tree.dataset_recipe.resolve())
    assert captured["kwargs"]["overrides"]["mode"] == "smoke"


def test_train_rejects_training_recipe_with_another_dataset(tmp_path, monkeypatch, capsys):
    tree = _make_tree(tmp_path)
    other = tree.scenario.root / "configs/other_dataset.yaml"
    document = yaml.safe_load(tree.dataset_recipe.read_text())
    document["task_text"] = "another dataset"
    other.write_text(yaml.safe_dump(document, sort_keys=False))
    training = yaml.safe_load(tree.training_recipe.read_text())
    training["data"]["config"] = str(other)
    tree.training_recipe.write_text(yaml.safe_dump(training, sort_keys=False))
    captured: dict = {}
    monkeypatch.setattr(cli, "train", _fake_train(captured))
    code = cli.main(_scenario_args(tree, "train", "--mode", "smoke"))
    assert code == 1
    assert captured == {}
    assert "differs from the selected dataset recipe" in capsys.readouterr().err


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("implementation", "pytorch", "unsupported learning.implementation"),
        ("adapter", "diffusion", "unsupported learning.adapter"),
    ],
)
def test_unsupported_learning_implementation_is_rejected_before_dispatch(
    tmp_path, monkeypatch, capsys, field, value, message
):
    tree = _make_tree(tmp_path)
    tree.scenario.document["learning"][field] = value
    tree.scenario.write()
    captured: dict = {}
    monkeypatch.setattr(cli, "convert", _fake_convert(captured))
    code = cli.main(_convert_args(tree, tmp_path))
    assert code == 1
    assert captured == {}
    assert message in capsys.readouterr().err
    assert not (tmp_path / "out").exists()


def test_selection_conflicts_and_missing_arguments_are_explicit(tmp_path, monkeypatch):
    tree = _make_tree(tmp_path)
    captured: dict = {}
    monkeypatch.setattr(cli, "convert", _fake_convert(captured))
    assert cli.main(_scenario_args(tree, "convert", "--config", str(tree.dataset_recipe))) == 1
    assert captured == {}
    assert (
        cli.main(
            [
                "--project-root",
                str(tree.repo),
                "convert",
                "--shared-root",
                str(tree.shared),
            ]
        )
        == 1
    )
    assert captured == {}
    assert (
        cli.main(
            [
                "--project-root",
                str(tree.repo),
                "train",
                "--robot",
                "missing",
                "--backend",
                "mujoco",
                "--task",
                "stack",
                "--shared-root",
                str(tree.shared),
            ]
        )
        == 1
    )
    assert captured == {}


def test_selection_is_cwd_independent(tmp_path, monkeypatch):
    tree = _make_tree(tmp_path)
    captured: dict = {}
    monkeypatch.setattr(cli, "convert", _fake_convert(captured))
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    monkeypatch.chdir(elsewhere)
    assert cli.main(_convert_args(tree, tmp_path)) == 0
    assert captured["config_path"] == tree.dataset_recipe.resolve()
    assert captured["selection"]["recipe"]["path"] == str(tree.dataset_recipe.resolve())


def test_help_and_parse_do_not_load_selection(tmp_path):
    assert (
        cli.main(["convert", "--robot", "missing", "--backend", "mujoco", "--task", "stack", "--help"])
        == 0
    )
    with pytest.raises(SystemExit) as error:
        policy_service_cli.main(
            [
                "--robot",
                "missing",
                "--backend",
                "mujoco",
                "--task",
                "stack",
                "--shared-root",
                str(tmp_path / "nowhere"),
                "--help",
            ]
        )
    assert error.value.code == 0


def test_scripts_forward_selection_without_implicit_defaults(tmp_path):
    selection = [
        "--robot",
        "synthetic_sim",
        "--backend",
        "mujoco",
        "--task",
        "stack",
        "--shared-root",
        str(tmp_path / "shared"),
    ]
    arguments = _run_dispatcher(tmp_path, "convert.sh", *selection)
    assert "--robot" in arguments
    assert "--backend" in arguments
    assert "--task" in arguments
    assert "--config" not in arguments

    arguments = _run_dispatcher(tmp_path, "convert.sh", "--config", "/custom/dataset.yaml")
    assert arguments.count("--config") == 1
    assert "/custom/dataset.yaml" in arguments

    arguments = _run_dispatcher(
        tmp_path,
        "server_policy.sh",
        "--robot",
        "synthetic_sim",
        "--backend",
        "mujoco",
        "--task",
        "stack",
        "--shared-root",
        str(tmp_path / "shared"),
    )
    assert "--robot" in arguments
    assert "--backend" in arguments
    assert "--config" not in arguments


def test_learning_help_does_not_require_a_container():
    with pytest.raises(SystemExit) as error:
        policy_service_cli.main(["--help"])
    assert error.value.code == 0


@pytest.fixture
def model_fixtures(tmp_path):
    pytest.importorskip("torch")
    pytest.importorskip("lerobot")
    from act_fixtures import make_act_model

    def make(path, *, synthetic=False):
        return make_act_model(path, synthetic=synthetic)

    return make


def _serve_patches(monkeypatch, captured: dict):
    class Adapter:
        def summary(self):
            return {}

    monkeypatch.setattr(policy_service_cli, "load_act_adapter", lambda config: Adapter())
    monkeypatch.setattr(
        policy_service_cli,
        "serve_forever",
        lambda config, adapter, **kwargs: captured.update(
            config=config, snapshot=kwargs["snapshot"]
        )
        or 0,
    )
    return Adapter


def test_policy_service_uses_recipe_and_explicit_model(
    tmp_path, model_fixtures, monkeypatch
):
    tree = _make_tree(tmp_path)
    model = model_fixtures(tmp_path / "model", synthetic=False)
    monkeypatch.setenv("PROJECT_ROOT", str(tree.repo))
    captured: dict = {}
    _serve_patches(monkeypatch, captured)
    code = policy_service_cli.main(
        [
            "--robot",
            "synthetic_sim",
            "--backend",
            "mujoco",
            "--task",
            "stack",
            "--shared-root",
            str(tree.shared),
            "--model",
            str(model),
            "--log-directory",
            str(tmp_path / "logs"),
            "--run-id",
            "scenario-serve",
        ]
    )
    assert code == 0
    config = captured["config"]
    assert config.task_text == "scenario service task"
    assert config.model_path == model.resolve()
    assert config.robot_profile_path == (tree.scenario.root / "profiles/robot.json").resolve()
    snapshot = captured["snapshot"]
    assert snapshot["config"]["path"] == str(tree.service_recipe.resolve())
    assert snapshot["selection"]["mode"] == "scenario"
    assert snapshot["selection"]["recipe"]["path"] == str(tree.service_recipe.resolve())
    assert snapshot["model_source"]["source"] == "cli"
    assert snapshot["source"]["model"] == str(config.model_path)

def test_policy_service_auto_picks_latest_model(
    tmp_path, model_fixtures, monkeypatch
):
    tree = _make_tree(tmp_path)
    monkeypatch.setenv("PROJECT_ROOT", str(tree.repo))
    monkeypatch.setenv("DATA_ROOT", str(tree.repo / "output"))
    models = tree.repo / "output/synthetic_sim/mujoco/stack/models"
    older = model_fixtures(models / "act_20260101_000000", synthetic=False)
    latest = model_fixtures(models / "act_20260102_000000", synthetic=False)
    assert older.is_dir()
    captured: dict = {}
    _serve_patches(monkeypatch, captured)
    code = policy_service_cli.main(
        [
            "--robot",
            "synthetic_sim",
            "--backend",
            "mujoco",
            "--task",
            "stack",
            "--shared-root",
            str(tree.shared),
            "--log-directory",
            str(tmp_path / "logs"),
            "--run-id",
            "auto-model",
        ]
    )
    assert code == 0
    assert captured["config"].model_path == latest.resolve()


def test_policy_service_missing_model_fails_before_serving(tmp_path, monkeypatch):
    tree = _make_tree(tmp_path)
    monkeypatch.setenv("PROJECT_ROOT", str(tree.repo))
    called = {"serve": False}
    monkeypatch.setattr(
        policy_service_cli, "serve_forever", lambda *a, **k: called.update(serve=True)
    )
    log_dir = tmp_path / "logs"
    code = policy_service_cli.main(
        [
            "--robot",
            "synthetic_sim",
            "--backend",
            "mujoco",
            "--task",
            "stack",
            "--shared-root",
            str(tree.shared),
            "--log-directory",
            str(log_dir),
            "--run-id",
            "missing-model",
        ]
    )
    assert code == 1
    assert called["serve"] is False
    failure = json.loads((log_dir / "startup_failure.json").read_text())
    assert failure["stage"] == "config"
    assert "no act_* model found" in failure["error"]


def test_policy_service_rejects_direct_profile_inputs(tmp_path):
    tree = _make_tree(tmp_path)
    with pytest.raises(SystemExit) as error:
        policy_service_cli.main(
            [
                "--robot",
                "synthetic_sim",
                "--backend",
                "mujoco",
                "--task",
                "stack",
                "--shared-root",
                str(tree.shared),
                "--robot-profile",
                str(tree.scenario.root / "profiles/robot.json"),
            ]
        )
    assert error.value.code == 2


def test_train_applies_hyperparameters_from_the_recipe(tmp_path, monkeypatch):
    tree = _make_tree(tmp_path)
    training = yaml.safe_load(tree.training_recipe.read_text())
    training["modes"]["smoke"]["steps"] = 64
    tree.training_recipe.write_text(yaml.safe_dump(training))
    captured: dict = {}
    monkeypatch.setattr(cli, "train", _fake_train(captured))
    assert cli.main(_scenario_args(tree, "train", "--mode", "smoke")) == 0
    assert captured["config"].modes["smoke"].steps == 64


def test_learning_entries_reject_generic_overrides(tmp_path):
    tree = _make_tree(tmp_path)
    for command in ("convert", "train"):
        assert cli.main(_scenario_args(tree, command, "--set", "modes.smoke.steps=64")) == 2
    with pytest.raises(SystemExit) as error:
        policy_service_cli.main(
            [
                "--robot", "synthetic_sim", "--backend", "mujoco", "--task", "stack",
                "--shared-root", str(tree.shared), "--set", "service.task_text=changed",
            ]
        )
    assert error.value.code == 2
