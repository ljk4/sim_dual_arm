"""Binding model, path roots, reference graph and stage conditions."""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import pytest
import yaml
from robot_contracts.binding import (
    BindingError,
    BindingStage,
    load_binding,
    stage_requirements,
)
from scenario_fixtures import ROOT, SHARED, ScenarioTree, base_binding, build_scenario


@pytest.fixture
def scenario(tmp_path: Path) -> ScenarioTree:
    return build_scenario(tmp_path / "scenario")


def _load(tree: ScenarioTree, **kwargs):
    return load_binding(tree.binding_path, shared_root=SHARED, **kwargs)


def _write(tree: ScenarioTree, document: dict) -> Path:
    return tree.write(document)


def test_valid_binding_resolves_every_declared_reference(scenario):
    resolved = _load(scenario)

    assert resolved.binding.scenario_id == "synthetic_sim/stack"
    assert resolved.path == scenario.binding_path.resolve()
    assert resolved.root == scenario.root.resolve()
    assert resolved.bundle.source == "none"
    assert resolved.bundle.path is None
    assert resolved.bundle.declared is None
    assert len(resolved.sha256) == 64
    assert set(resolved.references) == {
        "profiles.robot",
        "profiles.observation",
        "profiles.action",
        "recipes.simulation_teacher",
        "recipes.simulation_policy",
        "recipes.runtime_teacher",
        "recipes.runtime_policy",
        "recipes.campaign",
        "recipes.dataset",
        "recipes.training",
        "recipes.policy_service",
        "interfaces.extension_ros[0]",
    }
    assert resolved.references["profiles.robot"].semantic_sha256 is not None
    assert len(resolved.references["profiles.robot"].sha256) == 64
    assert resolved.references["interfaces.extension_ros[0]"].kind == "directory"
    assert resolved.references["interfaces.extension_ros[0]"].sha256 is None
    for stage in BindingStage:
        resolved.validate_stage(stage)


def test_summary_and_paths_are_cwd_independent(scenario, tmp_path, monkeypatch):
    first = _load(scenario).summary()
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    monkeypatch.chdir(elsewhere)
    second = _load(scenario).summary()

    assert first == second
    assert first["binding"]["path"] == str(scenario.binding_path.resolve())
    assert first["roots"]["binding"] == str(scenario.root.resolve())
    assert first["references"]["profiles.robot"]["path"] == str(
        (scenario.root / "profiles/robot.json").resolve()
    )
    assert first["models"]["digest"] is None
    assert first["models"]["validated_by"] == "learning.training.bundle"
    assert first["stages"]["collection"]["satisfied"] is True
    assert first["stages"]["collection"]["requirements"] == [
        "profiles.robot",
        "profiles.observation",
        "profiles.action",
        "recipes.runtime_teacher",
        "recipes.campaign",
        "recipes.simulation_teacher",
    ]


@pytest.mark.parametrize(
    ("field", "stage"),
    [
        ("recipes.campaign", BindingStage.collection),
        ("recipes.runtime_teacher", BindingStage.collection),
        ("recipes.dataset", BindingStage.learning),
        ("recipes.training", BindingStage.learning),
        ("learning.adapter", BindingStage.learning),
        ("recipes.runtime_policy", BindingStage.inference),
        ("recipes.policy_service", BindingStage.inference),
        ("interfaces.policy_api", BindingStage.inference),
    ],
)
def test_stage_requirements_fail_with_a_clear_field(scenario, field, stage):
    document = scenario.copy()
    section, name = field.split(".", 1)
    document[section][name] = None
    _write(scenario, document)

    with pytest.raises(BindingError) as caught:
        _load(scenario).validate_stage(stage)
    assert caught.value.reason == "BINDING_STAGE"
    assert field in str(caught.value)
    assert str(scenario.binding_path.resolve()) in str(caught.value)


def test_model_bundle_is_optional_for_every_stage(scenario):
    document = scenario.copy()
    document.setdefault("models", {})["bundle"] = None
    _write(scenario, document)
    resolved = _load(scenario)

    assert resolved.bundle.source == "none"
    assert resolved.bundle.path is None
    for stage in BindingStage:
        resolved.validate_stage(stage)
    assert "models.bundle" not in resolved.requirements("inference")


def test_hardware_backend_does_not_require_simulation_recipes(scenario):
    document = scenario.copy()
    document["backend"]["kind"] = "hardware"
    document["recipes"]["simulation_teacher"] = None
    document["recipes"]["simulation_policy"] = None
    _write(scenario, document)
    resolved = _load(scenario)

    resolved.validate_stage("collection")
    resolved.validate_stage("inference")
    assert "recipes.simulation_teacher" not in resolved.requirements("collection")
    assert "recipes.simulation_policy" not in resolved.requirements("inference")

    document["backend"]["kind"] = "simulation"
    _write(scenario, document)
    with pytest.raises(BindingError, match="recipes.simulation_teacher"):
        _load(scenario).validate_stage("collection")
    with pytest.raises(BindingError, match="recipes.simulation_policy"):
        _load(scenario).validate_stage("inference")


def test_bundle_override_records_the_real_source_without_a_fake_digest(scenario):
    override = scenario.root / "override_bundle"
    override.mkdir()
    base = _load(scenario)
    resolved = _load(scenario, bundle=override.resolve())

    assert resolved.bundle.source == "override"
    assert resolved.bundle.declared is None
    assert resolved.bundle.path == override.resolve()
    assert resolved.bundle.effective == str(override.resolve())
    assert resolved.bundle.summary()["digest"] is None
    resolved.validate_stage("inference")
    base_profiles = {
        field: reference.semantic_sha256
        for field, reference in base.references.items()
        if field.startswith("profiles.")
    }
    override_profiles = {
        field: reference.semantic_sha256
        for field, reference in resolved.references.items()
        if field.startswith("profiles.")
    }
    assert base_profiles == override_profiles


def test_bundle_override_is_relative_to_the_caller_cwd(scenario, monkeypatch):
    (scenario.root / "override_bundle").mkdir()
    monkeypatch.chdir(scenario.root)
    resolved = _load(scenario, bundle="override_bundle")

    assert resolved.bundle.source == "override"
    assert resolved.bundle.path == (scenario.root / "override_bundle").resolve()


def test_duplicate_yaml_key_is_rejected(scenario):
    text = yaml.safe_dump(scenario.document, sort_keys=False)
    text += f"robot_id: {scenario.document['robot_id']}\n"
    scenario.binding_path.write_text(text, encoding="utf-8")

    with pytest.raises(BindingError) as caught:
        _load(scenario)
    assert caught.value.reason == "BINDING_SCHEMA"
    assert "duplicate key" in str(caught.value)
    assert "robot_id" in str(caught.value)
    assert str(scenario.binding_path) in str(caught.value)


@pytest.mark.parametrize(
    ("mutate", "location"),
    [
        (lambda document: document["backend"].pop("kind"), "backend: kind"),
        (lambda document: document["profiles"].pop("robot"), "profiles: robot"),
        (lambda document: document.pop("learning"), "learning: Field required"),
    ],
)
def test_missing_required_fields_are_rejected(scenario, mutate, location):
    document = scenario.copy()
    mutate(document)
    _write(scenario, document)

    with pytest.raises(BindingError) as caught:
        _load(scenario)
    assert caught.value.reason == "BINDING_SCHEMA"
    assert location in str(caught.value)
    assert str(scenario.binding_path) in str(caught.value)


@pytest.mark.parametrize(
    ("label", "mutate"),
    [
        ("unused_defaults", lambda document: document.update(unused_defaults={})),
        ("extends", lambda document: document.update(extends="../other/binding.yaml")),
        (
            "defaults",
            lambda document: document.update(defaults={"models": {"bundle": "bundles/act"}}),
        ),
        ("unknown", lambda document: document["backend"].update(unknown=True)),
    ],
)
def test_unknown_fields_and_implicit_inheritance_are_rejected(scenario, label, mutate):
    document = scenario.copy()
    mutate(document)
    _write(scenario, document)

    with pytest.raises(BindingError) as caught:
        _load(scenario)
    assert caught.value.reason == "BINDING_SCHEMA"
    assert label in str(caught.value)


@pytest.mark.parametrize(
    ("field", "remove"),
    [
        ("profiles.robot", lambda tree: (tree.root / "profiles/robot.json").unlink()),
        ("recipes.campaign", lambda tree: (tree.root / "configs/campaign.yaml").unlink()),
        (
            "interfaces.extension_ros[0]",
            lambda tree: shutil.rmtree(tree.root / "extensions/ros/marvin_teacher_api_msgs"),
        ),
    ],
)
def test_missing_reference_is_rejected(scenario, field, remove):
    remove(scenario)

    with pytest.raises(BindingError) as caught:
        _load(scenario)
    assert caught.value.reason == "BINDING_REFERENCE"
    assert field in str(caught.value)
    assert "does not exist" in str(caught.value)


@pytest.mark.parametrize(
    ("field", "replace", "message"),
    [
        (
            "profiles.robot",
            lambda tree: (tree.root / "profiles/robot.json").unlink()
            or (tree.root / "profiles/robot.json").mkdir(),
            "expected a file",
        ),
        (
            "recipes.campaign",
            lambda tree: (tree.root / "configs/campaign.yaml").unlink()
            or (tree.root / "configs/campaign.yaml").mkdir(),
            "expected a file",
        ),
        (
            "interfaces.extension_ros[0]",
            lambda tree: shutil.rmtree(tree.root / "extensions/ros/marvin_teacher_api_msgs")
            or (tree.root / "extensions/ros/marvin_teacher_api_msgs").write_text("file"),
            "expected a directory",
        ),
    ],
)
def test_path_category_errors_are_rejected(scenario, field, replace, message):
    replace(scenario)

    with pytest.raises(BindingError) as caught:
        _load(scenario)
    assert caught.value.reason == "BINDING_REFERENCE"
    assert field in str(caught.value)
    assert message in str(caught.value)


def test_source_references_must_be_relative(scenario):
    document = scenario.copy()
    document["profiles"]["robot"] = str((scenario.root / "profiles/robot.json").resolve())
    _write(scenario, document)

    with pytest.raises(BindingError) as caught:
        _load(scenario)
    assert caught.value.reason == "BINDING_REFERENCE"
    assert "profiles.robot" in str(caught.value)
    assert "relative" in str(caught.value)


def test_reference_outside_declared_source_roots_is_rejected(scenario):
    outside = scenario.root.parent / "outside_profile.json"
    shutil.copyfile(scenario.root / "profiles/robot.json", outside)
    document = scenario.copy()
    document["profiles"]["robot"] = "../outside_profile.json"
    _write(scenario, document)

    with pytest.raises(BindingError) as caught:
        _load(scenario)
    assert caught.value.reason == "BINDING_REFERENCE"
    assert "outside the declared source roots" in str(caught.value)


def test_relative_source_roots_cover_end_and_shared_configs(tmp_path):
    repository = tmp_path / "repo"
    scenario = build_scenario(repository / "shared/scenarios/marvin_sim/stack")
    end_config = repository / "runtime/ros2/configs/existing.yaml"
    end_config.parent.mkdir(parents=True)
    end_config.write_text("{}\n", encoding="utf-8")
    shared_config = repository / "shared/common/configs/shared.yaml"
    shared_config.parent.mkdir(parents=True)
    shared_config.write_text("{}\n", encoding="utf-8")

    document = scenario.copy()
    document["source_roots"] = [
        "../../../../runtime/ros2/configs",
        "../../..",
    ]
    document["recipes"]["runtime_teacher"] = "../../../../runtime/ros2/configs/existing.yaml"
    document["recipes"]["dataset"] = "../../../common/configs/shared.yaml"
    _write(scenario, document)

    resolved = _load(scenario)
    assert resolved.references["recipes.runtime_teacher"].path == end_config.resolve()
    assert resolved.references["recipes.dataset"].path == shared_config.resolve()
    assert resolved.source_roots == (
        end_config.parent.resolve(),
        (repository / "shared").resolve(),
    )
    resolved.validate_stage("collection")
    resolved.validate_stage("learning")


def test_explicit_cross_scenario_reuse_is_allowed(tmp_path):
    scenarios = tmp_path / "scenarios"
    first = build_scenario(scenarios / "a")
    second = build_scenario(scenarios / "b")
    document = first.copy()
    document["source_roots"] = [".."]
    document["profiles"] = {
        name: f"../b/profiles/{name}.json"
        for name in ("robot", "observation", "action")
    }
    first.write(document)

    resolved = _load(first)
    assert resolved.references["profiles.robot"].path == (
        scenarios / "b/profiles/robot.json"
    ).resolve()
    assert resolved.source_roots == (scenarios.resolve(),)
    resolved.validate_stage("collection")

    without_root = first.copy()
    without_root["source_roots"] = []
    first.write(without_root)
    with pytest.raises(BindingError, match="outside the declared source roots"):
        _load(first)
    assert second.binding_path.is_file()


def test_reference_cycle_is_rejected(tmp_path):
    scenarios = tmp_path / "scenarios"
    first = build_scenario(scenarios / "a")
    second = build_scenario(scenarios / "b")
    first_document = first.copy()
    first_document["source_roots"] = [".."]
    first_document["references"] = ["../b/binding.yaml"]
    first.write(first_document)
    second_document = second.copy()
    second_document["source_roots"] = [".."]
    second_document["references"] = ["../a/binding.yaml"]
    second.write(second_document)

    with pytest.raises(BindingError) as caught:
        _load(first)
    assert caught.value.reason == "BINDING_CYCLE"
    assert str(first.binding_path.resolve()) in str(caught.value)
    assert str(second.binding_path.resolve()) in str(caught.value)


def test_self_reference_is_a_cycle(scenario):
    document = scenario.copy()
    document["references"] = ["binding.yaml"]
    _write(scenario, document)

    with pytest.raises(BindingError) as caught:
        _load(scenario)
    assert caught.value.reason == "BINDING_CYCLE"
    assert str(scenario.binding_path.resolve()) in str(caught.value)


def test_explicit_acyclic_reference_is_recorded(tmp_path):
    scenarios = tmp_path / "scenarios"
    first = build_scenario(scenarios / "a")
    second = build_scenario(scenarios / "b")
    second_document = second.copy()
    second_document["robot_id"] = "synthetic_sim"
    second_document["task_id"] = "dependency"
    second.write(second_document)
    first_document = first.copy()
    first_document["source_roots"] = [".."]
    first_document["references"] = ["../b/binding.yaml"]
    first.write(first_document)

    resolved = _load(first)
    assert len(resolved.dependencies) == 1
    assert resolved.dependencies[0].binding.scenario_id == "synthetic_sim/dependency"
    summary = resolved.summary()
    assert summary["dependencies"][0]["path"] == str(second.binding_path.resolve())
    assert summary["references"]["references[0]"]["kind"] == "file"


def test_profile_binding_mismatch_is_rejected(scenario):
    observation_path = scenario.root / "profiles/observation.json"
    observation = json.loads(observation_path.read_text())
    observation["camera_names"].reverse()
    observation_path.write_text(json.dumps(observation), encoding="utf-8")

    with pytest.raises(BindingError) as caught:
        _load(scenario)
    assert caught.value.reason == "BINDING_SCHEMA"
    assert "camera_names" in str(caught.value)


def test_absolute_model_requires_a_declared_artifact_root(scenario, tmp_path):
    artifacts = tmp_path / "artifacts"
    (artifacts / "bundle").mkdir(parents=True)
    document = scenario.copy()
    document["artifact_roots"] = [str(artifacts.resolve())]
    document.setdefault("models", {})["bundle"] = str((artifacts / "bundle").resolve())
    _write(scenario, document)

    resolved = _load(scenario)
    assert resolved.bundle.path == (artifacts / "bundle").resolve()
    assert resolved.bundle.source == "binding"

    outside = tmp_path / "other_bundle"
    outside.mkdir()
    document["models"]["bundle"] = str(outside.resolve())
    _write(scenario, document)
    with pytest.raises(BindingError, match="outside the declared roots"):
        _load(scenario)


def test_invalid_roots_are_rejected(scenario, tmp_path):
    file_root = tmp_path / "root-file"
    file_root.write_text("not a directory")
    document = scenario.copy()
    document["source_roots"] = ["missing-root"]
    _write(scenario, document)
    with pytest.raises(BindingError) as caught:
        _load(scenario)
    assert caught.value.reason == "BINDING_REFERENCE"
    assert "root is not a directory" in str(caught.value)

    document["source_roots"] = [str(tmp_path)]
    _write(scenario, document)
    with pytest.raises(BindingError, match="source roots must be relative"):
        _load(scenario)

    document["source_roots"] = ["../root-file"]
    _write(scenario, document)
    with pytest.raises(BindingError, match="root is not a directory"):
        _load(scenario)

    document["source_roots"] = []
    document["artifact_roots"] = ["missing-artifacts"]
    _write(scenario, document)
    with pytest.raises(BindingError) as caught:
        _load(scenario)
    assert caught.value.reason == "BINDING_REFERENCE"
    assert "missing-artifacts" in str(caught.value)


def test_stage_requirement_helper_is_stable():
    from robot_contracts.binding import ScenarioBinding

    binding = ScenarioBinding.validate_dict(base_binding())
    assert stage_requirements(binding, "collection") == (
        "profiles.robot",
        "profiles.observation",
        "profiles.action",
        "recipes.runtime_teacher",
        "recipes.campaign",
        "recipes.simulation_teacher",
    )
    assert stage_requirements(binding, BindingStage.learning) == (
        "profiles.robot",
        "profiles.observation",
        "profiles.action",
        "learning.adapter",
        "recipes.dataset",
        "recipes.training",
    )
    assert "models.bundle" not in stage_requirements(binding, "inference")

    hardware = base_binding()
    hardware["backend"]["kind"] = "hardware"
    assert "recipes.simulation_teacher" not in stage_requirements(
        ScenarioBinding.validate_dict(hardware), "collection"
    )


def test_unknown_stage_is_rejected(scenario):
    with pytest.raises(BindingError, match="unknown stage"):
        _load(scenario).validate_stage("training")


def test_template_robot_and_task_are_structurally_accepted(scenario):
    document = scenario.copy()
    document["robot_id"] = "_template"
    _write(scenario, document)

    assert _load(scenario).binding.scenario_id == "_template/stack"


def test_template_example_is_unfilled_and_not_runnable():
    template_dir = ROOT / "shared/scenarios/_template"
    example = template_dir / "binding.example.yaml"

    assert example.is_file()
    assert not (template_dir / "binding.yaml").exists()
    document = yaml.safe_load(example.read_text(encoding="utf-8"))
    assert document["robot_id"] == "replace_me"
    assert document["task_id"] == "replace_me"
    with pytest.raises(BindingError) as caught:
        load_binding(example, shared_root=SHARED)
    assert caught.value.reason == "BINDING_REFERENCE"
    assert "profiles.robot" in str(caught.value)
    assert "REPLACE_ME" in str(caught.value)


def test_artifact_root_data_resolves_via_data_root_env(scenario, tmp_path, monkeypatch):
    artifacts = tmp_path / "var"
    bundle = artifacts / "marvin_sim/stack/models/act_20261121_000000"
    bundle.mkdir(parents=True)
    document = scenario.copy()
    document["artifact_roots"] = ["/data"]
    _write(scenario, document)

    monkeypatch.setenv("DATA_ROOT", str(artifacts))
    resolved = _load(scenario, model=bundle)
    assert resolved.artifact_roots == (artifacts.resolve(),)
    assert resolved.bundle.path == bundle.resolve()
    assert resolved.bundle.source == "override"

    resolved_container_model = _load(
        scenario, model="/data/marvin_sim/stack/models/act_20261121_000000"
    )
    assert resolved_container_model.bundle.path == bundle.resolve()
