"""Learning-side scenario selection: provenance, conflicts and real dispatch.

Conversion and training are exercised through test doubles that receive the
real configuration object; the full data pipeline is deliberately not rerun.
prepare-policy was removed; policy loading uses a real, verifiable synthetic
ACT bundle instead.
"""

from __future__ import annotations

import json

import pytest
import yaml
from dual_arm_learning import cli, policy_service_cli
from learning_scenario_fixtures import (
    ROOT,
    SHA,
)
from learning_scenario_fixtures import (
    Audit as _Audit,
)
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
    assert captured["selection"]["recipe"]["field"] == "recipes.dataset"
    assert captured["selection"]["recipe"]["path"] == str(tree.dataset_recipe.resolve())
    assert captured["selection"]["binding"]["sha256"] == _sha(tree.scenario.binding_path)
    assert captured["overrides"]["select"]["value"] == str(tmp_path / "select.json")
    assert captured["overrides"]["output_root"]["value"] == str(tmp_path / "out")
    assert captured["overrides"]["synthetic"]["value"] is True


def test_convert_requires_explicit_config_or_scenario(tmp_path, monkeypatch):
    captured: dict = {}
    monkeypatch.setattr(cli, "convert", _fake_convert(captured))
    assert (
        cli.main(
            [
                "--project-root",
                str(ROOT),
                "convert",
                "--select",
                str(tmp_path / "select.json"),
                "--output-root",
                str(tmp_path / "out"),
            ]
        )
        == 2
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


def test_train_uses_selected_training_recipe_and_checkpoint_provenance(tmp_path, monkeypatch):
    tree = _make_tree(tmp_path)
    captured: dict = {}
    monkeypatch.setattr(cli, "train", _fake_train(captured))
    code = cli.main(_scenario_args(tree, "train", "--mode", "smoke"))
    assert code == 0
    config = captured["config"]
    assert config.source == tree.training_recipe.resolve()
    assert config.dataset_config_path == tree.dataset_recipe.resolve()
    assert config.dataset.task_text == "scenario dataset task"
    assert config.data_root == (tmp_path / "data").resolve()
    selection = captured["kwargs"]["selection"]
    assert selection["recipe"]["field"] == "recipes.training"
    assert selection["dataset_recipe"]["path"] == str(tree.dataset_recipe.resolve())
    assert captured["kwargs"]["overrides"]["mode"]["value"] == "smoke"
    # learning stage never requires a pre-existing model
    assert selection.get("bundle") is None


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
    assert not (tmp_path / "runs").exists()


def test_bundle_uses_selected_training_recipe_without_a_model(tmp_path, monkeypatch):
    tree = _make_tree(tmp_path)
    captured: dict = {}

    def export_bundle(**kwargs):
        captured.update(kwargs)
        return {
            "bundle_digest": SHA,
            "files": {},
            "checkpoint": {"step": 1},
            "synthetic": True,
        }

    monkeypatch.setattr(cli, "export_bundle", export_bundle)
    code = cli.main(
        _scenario_args(
            tree,
            "bundle",
            "--run",
            str(tmp_path / "runs/run-scenario"),
            "--output",
            str(tmp_path / "bundle-out"),
            "--mode",
            "overfit",
        )
    )
    assert code == 0
    assert captured["config"].source == tree.training_recipe.resolve()
    assert captured["selection"]["recipe"]["field"] == "recipes.training"
    assert captured["overrides"]["mode"]["value"] == "overfit"


def test_verify_data_and_budget_use_selected_training_recipe(tmp_path, monkeypatch):
    tree = _make_tree(tmp_path)
    captured: dict = {}

    def verify(config, **kwargs):
        captured["config"] = config
        captured["verify"] = kwargs
        return _Audit()

    monkeypatch.setattr(cli, "verify_dataset_manifest", verify)
    assert cli.main(_scenario_args(tree, "verify-data", "--allow-synthetic")) == 0
    assert captured["config"].source == tree.training_recipe.resolve()
    assert captured["verify"]["allow_synthetic"] is True

    assert cli.main(_scenario_args(tree, "budget-status")) == 0
    assert (tmp_path / "budget.jsonl").parent.is_dir()
    assert not (tmp_path / "runs").exists()


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


def test_scenario_conflicts_and_missing_arguments_are_explicit(tmp_path, monkeypatch):
    tree = _make_tree(tmp_path)
    captured: dict = {}
    monkeypatch.setattr(cli, "convert", _fake_convert(captured))
    # argparse mutually exclusive group: exit code 2, no dispatch
    assert cli.main(_scenario_args(tree, "convert", "--config", str(tree.dataset_recipe))) == 2
    assert captured == {}
    # --shared-root without --scenario: the required selection group rejects it
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
        == 2
    )
    assert captured == {}
    # unknown scenario
    assert (
        cli.main(
            [
                "--project-root",
                str(tree.repo),
                "train",
                "--scenario",
                "missing",
                "--shared-root",
                str(tree.shared),
            ]
        )
        == 1
    )
    assert captured == {}


def test_scenario_selection_is_cwd_independent(tmp_path, monkeypatch):
    tree = _make_tree(tmp_path)
    captured: dict = {}
    monkeypatch.setattr(cli, "convert", _fake_convert(captured))
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    monkeypatch.chdir(elsewhere)
    assert cli.main(_convert_args(tree, tmp_path)) == 0
    assert captured["config_path"] == tree.dataset_recipe.resolve()
    assert captured["selection"]["recipe"]["path"] == str(tree.dataset_recipe.resolve())


def test_help_and_parse_do_not_load_scenario(tmp_path):
    # argparse exits before binding selection or model loading.
    assert cli.main(["convert", "--scenario", "missing", "--help"]) == 0
    with pytest.raises(SystemExit) as error:
        policy_service_cli.main(
            [
                "--scenario",
                "missing",
                "--shared-root",
                str(tmp_path / "nowhere"),
                "--help",
            ]
        )
    assert error.value.code == 0


def test_scripts_forward_selection_without_implicit_defaults(tmp_path):
    scenario = [
        "--scenario",
        "synthetic_stack",
        "--shared-root",
        str(tmp_path / "shared"),
    ]
    arguments = _run_dispatcher(tmp_path, "convert.sh", *scenario)
    assert "--scenario" in arguments
    assert "--config" not in arguments

    arguments = _run_dispatcher(tmp_path, "convert.sh", "--config", "/custom/dataset.yaml")
    assert arguments.count("--config") == 1
    assert "/custom/dataset.yaml" in arguments

    arguments = _run_dispatcher(tmp_path, "convert.sh")
    assert "--config" not in arguments

    arguments = _run_dispatcher(
        tmp_path,
        "policy.sh",
        "--scenario",
        "synthetic_stack",
        "--shared-root",
        str(tmp_path / "shared"),
    )
    assert "--scenario" in arguments
    assert "--config" not in arguments


def test_learning_help_does_not_require_a_container():
    with pytest.raises(SystemExit) as error:
        policy_service_cli.main(["--help"])
    assert error.value.code == 0


# --------------------------------------------------------------------------
# policy service: scenario recipe plus a real verified bundle
# --------------------------------------------------------------------------


@pytest.fixture
def torch_fixtures(tmp_path):
    pytest.importorskip("torch")
    from act_fixtures import make_act_bundle

    return make_act_bundle


def test_policy_service_scenario_uses_recipe_and_effective_bundle(
    tmp_path, torch_fixtures, monkeypatch
):
    tree = _make_tree(tmp_path)
    bundle = torch_fixtures(tree.scenario.root / "bundles/verified", synthetic=False)
    tree.scenario.document["models"]["bundle"] = "bundles/verified"
    tree.scenario.write()
    captured: dict = {}

    class Adapter:
        def summary(self):
            return {}

    def fake_serve(config, adapter, **kwargs):
        captured["config"] = config
        captured["snapshot"] = kwargs["snapshot"]
        return 0

    monkeypatch.setattr(policy_service_cli, "load_act_adapter", lambda config: Adapter())
    monkeypatch.setattr(policy_service_cli, "serve_forever", fake_serve)
    log_dir = tmp_path / "logs"
    code = policy_service_cli.main(
        [
            "--scenario",
            "synthetic_stack",
            "--shared-root",
            str(tree.shared),
            "--bundle-dir",
            str(bundle),
            "--log-directory",
            str(log_dir),
            "--run-id",
            "scenario-serve",
        ]
    )
    assert code == 0
    config = captured["config"]
    assert config.task_text == "scenario service task"
    assert config.bundle_dir == bundle.resolve()
    assert config.robot_profile_path == (tree.scenario.root / "profiles/robot.json").resolve()
    snapshot = captured["snapshot"]
    assert snapshot["config"]["path"] == str(tree.service_recipe.resolve())
    assert snapshot["selection"]["mode"] == "scenario"
    assert snapshot["selection"]["recipe"]["path"] == str(tree.service_recipe.resolve())
    assert snapshot["model_source"]["source"] == "override"
    assert snapshot["source"]["bundle_root"] == str(bundle.resolve())


def test_policy_service_scenario_missing_model_fails_before_serving(
    tmp_path, torch_fixtures, monkeypatch
):
    tree = _make_tree(tmp_path)
    tree.scenario.document["models"]["bundle"] = None
    tree.scenario.write()
    called = {"serve": False}
    monkeypatch.setattr(
        policy_service_cli, "serve_forever", lambda *a, **k: called.update(serve=True)
    )
    log_dir = tmp_path / "logs"
    code = policy_service_cli.main(
        [
            "--scenario",
            "synthetic_stack",
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
    assert "models.bundle" in failure["error"]


def test_policy_service_scenario_rejects_direct_profile_inputs(tmp_path):
    tree = _make_tree(tmp_path)
    with pytest.raises(SystemExit) as error:
        policy_service_cli.main(
            [
                "--scenario",
                "synthetic_stack",
                "--shared-root",
                str(tree.shared),
                "--robot-profile",
                str(tree.scenario.root / "profiles/robot.json"),
            ]
        )
    assert error.value.code == 2


def test_train_applies_set_overrides_and_records_them(tmp_path, monkeypatch):
    tree = _make_tree(tmp_path)
    captured: dict = {}
    monkeypatch.setattr(cli, "train", _fake_train(captured))
    assert (
        cli.main(
            _scenario_args(tree, "train", "--mode", "smoke", "--set", "budget.limit_seconds=123")
        )
        == 0
    )
    assert captured["config"].budget.limit_seconds == 123.0
    assert captured["kwargs"]["overrides"]["set"]["value"] == ["budget.limit_seconds=123"]


def test_policy_service_scenario_applies_set_overrides(tmp_path, torch_fixtures, monkeypatch):
    tree = _make_tree(tmp_path)
    torch_fixtures(tree.scenario.root / "bundles/verified", synthetic=False)
    tree.scenario.document["models"]["bundle"] = "bundles/verified"
    tree.scenario.write()
    captured: dict = {}

    class Adapter:
        def summary(self):
            return {}

    monkeypatch.setattr(policy_service_cli, "load_act_adapter", lambda config: Adapter())

    def fake_serve(config, adapter, **kwargs):
        captured["config"] = config
        captured["snapshot"] = kwargs["snapshot"]
        return 0

    monkeypatch.setattr(policy_service_cli, "serve_forever", fake_serve)
    code = policy_service_cli.main(
        [
            "--scenario",
            "synthetic_stack",
            "--shared-root",
            str(tree.shared),
            "--set",
            "service.task_text=overridden task",
            "--log-directory",
            str(tmp_path / "logs"),
            "--run-id",
            "scenario-set",
        ]
    )
    assert code == 0
    assert captured["config"].task_text == "overridden task"
    assert captured["snapshot"]["overrides"]["set"]["value"] == [
        "service.task_text=overridden task"
    ]
