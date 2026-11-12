"""Scenario selection feeds real prepared inputs and preserves identity checks."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import shutil
from pathlib import Path

import pytest
import yaml
from dual_arm_runtime.offline import prepare_run
from robot_contracts.profiles import load_document, semantic_hash
from scenario_fixtures import build_scenario

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    "prepare_backend", ROOT / "simulation/mujoco/scripts/prepare_backend.py"
)
assert spec is not None and spec.loader is not None
prepare_backend = importlib.util.module_from_spec(spec)
spec.loader.exec_module(prepare_backend)

SHA = "a" * 64


def _shared_root(tmp_path: Path):
    root = tmp_path / "repo/shared"
    scenario = build_scenario(root / "scenarios/synthetic_stack")
    shutil.copytree(
        ROOT / "shared/common/interfaces/schemas", root / "common/interfaces/schemas"
    )
    return root, scenario


def _backend_recipe(
    scenario,
    realtime_speed: float,
    *,
    name: str = "simulation_teacher.yaml",
    chunk_history: int = 256,
) -> Path:
    (scenario.root / "configs/scene.yaml").write_text("{}")
    path = scenario.root / "configs" / name
    path.write_text(
        yaml.safe_dump(
            {
                "robot": {"profile": "../profiles/robot.json", "scene": "scene.yaml"},
                "run": {"seed": 0, "realtime_speed": realtime_speed},
                "limits": {"chunk_history": chunk_history},
            },
            sort_keys=False,
        )
    )
    return path


def _runtime_recipe(scenario, name: str, action_source: str) -> Path:
    (scenario.root / "profiles/control.yaml").write_text("schema_version: 1\n")
    (scenario.root / "profiles/follower.yaml").write_text("schema_version: 1\n")
    document = {
        "schema_version": 1,
        "robot_profile": "../profiles/robot.json",
        "observation_schema": "../profiles/observation.json",
        "action_schema": "../profiles/action.json",
        "control_constraints": "../profiles/control.yaml",
        "follower_config": "../profiles/follower.yaml",
        "robot_namespace": "/robot",
        "action_source": action_source,
        "reset": {"enabled": True, "seed": 0},
    }
    if action_source == "policy":
        document["policy"] = {
            "endpoint": "learning:50051",
            "model_bundle_digest": "FROM_VERIFIED_ACT_BUNDLE",
            "processor_digest": "FROM_VERIFIED_ACT_BUNDLE",
            "task_id": "stack",
            "task_text": "stack",
        }
    else:
        document["teacher"] = {"chunk_size": 60}
    path = scenario.root / "configs" / f"{name}.yaml"
    path.write_text(yaml.safe_dump(document, sort_keys=False))
    return path


def _policy_binding(
    scenario, run_id: str, *, bundle: Path | None = None
) -> Path:
    """Mirror the real learning prepare-policy JSON shape (model.bundle included)."""
    documents = {}
    for key, name in (
        ("profile", "robot.json"),
        ("observation_schema", "observation.json"),
        ("action_schema", "action.json"),
    ):
        path = scenario.root / "profiles" / name
        documents[key] = {
            "path": str(path),
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "semantic_sha256": semantic_hash(load_document(path)),
        }
    bundle_dir = scenario.root / "bundles/act" if bundle is None else bundle
    binding = {
        "schema_version": 1,
        "kind": "policy-binding",
        "run_id": run_id,
        "endpoint": "learning:50051",
        "model": {
            "bundle": str(bundle_dir),
            "bundle_digest": SHA,
            "processor_digest": "b" * 64,
            "model_sha256": "c" * 64,
            "state_dict_sha256": "c" * 64,
            "config_sha256": "d" * 64,
            "checkpoint_step": 1,
            "synthetic": True,
        },
        "service": {
            "port": 50051,
            "task_id": "stack",
            "task_text": "stack",
            "predict_deadline_s": 2.0,
            "open_deadline_s": 2.0,
            "close_deadline_s": 2.0,
            "max_message_bytes": 16777216,
        },
        **documents,
    }
    path = scenario.root / "policy_binding.json"
    path.write_text(json.dumps(binding) + "\n")
    return path


def test_simulation_prepare_scenario_changes_real_template(tmp_path):
    shared_root, scenario = _shared_root(tmp_path)
    recipe = _backend_recipe(scenario, 1.0)
    first = tmp_path / "first"
    manifest = prepare_backend.prepare_backend(
        scenario="synthetic_stack",
        role="teacher",
        shared_root=shared_root,
        seed=3,
        run_id="prepare-1",
        output=first,
    )
    baseline = yaml.safe_load((first / "backend.yaml").read_text())
    assert baseline["run"]["realtime_speed"] == 1.0
    assert baseline["run"]["seed"] == 3
    assert manifest["selection"]["scenario_id"] == "synthetic_stack"
    assert manifest["selection"]["recipe"]["path"] == str(recipe.resolve())
    assert manifest["selection"]["binding"]["sha256"] == hashlib.sha256(
        scenario.binding_path.read_bytes()
    ).hexdigest()

    _backend_recipe(scenario, 0.4)
    second = tmp_path / "second"
    prepare_backend.prepare_backend(
        scenario="synthetic_stack",
        role="teacher",
        shared_root=shared_root,
        seed=3,
        run_id="prepare-2",
        output=second,
    )
    changed = yaml.safe_load((second / "backend.yaml").read_text())
    assert changed["run"]["realtime_speed"] == 0.4
    assert changed != baseline


def test_runtime_prepare_scenario_uses_recipe_and_verified_binding(tmp_path):
    shared_root, scenario = _shared_root(tmp_path)
    _runtime_recipe(scenario, "runtime_policy", "policy")
    binding = _policy_binding(scenario, "run-1")
    output = tmp_path / "prepared"
    manifest = prepare_run.prepare_run(
        scenario="synthetic_stack",
        role="policy",
        shared_root=shared_root,
        binding=binding,
        seed=11,
        run_id="run-1",
        output=output,
        project_root=tmp_path / "repo",
    )
    runtime = yaml.safe_load((output / "runtime.yaml").read_text())
    assert runtime["action_source"] == "policy"
    assert runtime["policy"]["model_bundle_digest"] == SHA
    assert runtime["reset"] == {"enabled": True, "seed": 11}
    assert runtime["robot_profile"] == (
        "/workspace/shared/scenarios/synthetic_stack/profiles/robot.json"
    )
    assert manifest["selection"]["scenario_id"] == "synthetic_stack"
    assert manifest["selection"]["bundle"]["source"] == "binding"
    assert manifest["selection"]["bundle"]["path"] == str(
        (scenario.root / "bundles/act").resolve()
    )
    assert manifest["policy"]["bundle_digest"] == SHA

    document = json.loads(binding.read_text())
    document["profile"]["semantic_sha256"] = "0" * 64
    bad = tmp_path / "bad_binding.json"
    bad.write_text(json.dumps(document))
    with pytest.raises(prepare_run.PrepareError, match="semantic_sha256 differs"):
        prepare_run.prepare_run(
            scenario="synthetic_stack",
            role="policy",
            shared_root=shared_root,
            binding=bad,
            seed=11,
            run_id="run-1",
            output=tmp_path / "rejected",
            project_root=tmp_path / "repo",
        )
    assert not (tmp_path / "rejected").exists()


def _prepare_policy(scenario, shared_root, binding, output):
    return prepare_run.prepare_run(
        scenario="synthetic_stack",
        role="policy",
        shared_root=shared_root,
        binding=binding,
        seed=11,
        run_id="run-1",
        output=output,
        project_root=scenario.root.parents[2],
    )


def test_runtime_prepare_accepts_same_bundle_directory_aliases(tmp_path):
    shared_root, scenario = _shared_root(tmp_path)
    _runtime_recipe(scenario, "runtime_policy", "policy")
    bundle = scenario.root / "bundles/act"
    alias = scenario.root / "bundles/act-alias"
    alias.symlink_to(bundle, target_is_directory=True)

    for label, declared in (("symlink", alias), ("relative", Path("bundles/act"))):
        binding = _policy_binding(scenario, "run-1", bundle=declared)
        output = tmp_path / label
        manifest = _prepare_policy(scenario, shared_root, binding, output)
        assert (output / "runtime.yaml").is_file()
        assert manifest["selection"]["bundle"]["path"] == str(bundle.resolve())


def test_runtime_prepare_rejects_different_bundle_with_same_profile(tmp_path):
    shared_root, scenario = _shared_root(tmp_path)
    _runtime_recipe(scenario, "runtime_policy", "policy")
    other = scenario.root / "bundles/other"
    other.mkdir()
    binding = _policy_binding(scenario, "run-1", bundle=other)
    output = tmp_path / "different-bundle"
    with pytest.raises(prepare_run.PrepareError, match="differs from effective scenario bundle"):
        _prepare_policy(scenario, shared_root, binding, output)
    assert not output.exists()


def test_runtime_prepare_rejects_missing_policy_bundle_information(tmp_path):
    shared_root, scenario = _shared_root(tmp_path)
    _runtime_recipe(scenario, "runtime_policy", "policy")
    binding = _policy_binding(scenario, "run-1")
    document = json.loads(binding.read_text())
    document["model"].pop("bundle")
    binding.write_text(json.dumps(document) + "\n")
    output = tmp_path / "no-policy-bundle"
    with pytest.raises(prepare_run.PrepareError, match="model.bundle is required"):
        _prepare_policy(scenario, shared_root, binding, output)
    assert not output.exists()

    missing = scenario.root / "bundles/missing"
    binding = _policy_binding(scenario, "run-1", bundle=missing)
    output = tmp_path / "missing-bundle-dir"
    with pytest.raises(prepare_run.PrepareError, match="missing or unreadable"):
        _prepare_policy(scenario, shared_root, binding, output)
    assert not output.exists()


def test_runtime_prepare_rejects_scenario_without_effective_bundle(tmp_path):
    shared_root, scenario = _shared_root(tmp_path)
    _runtime_recipe(scenario, "runtime_policy", "policy")
    scenario.document["models"]["bundle"] = None
    scenario.write()
    binding = _policy_binding(scenario, "run-1")
    output = tmp_path / "no-scenario-bundle"
    with pytest.raises(prepare_run.PrepareError, match="models.bundle"):
        _prepare_policy(scenario, shared_root, binding, output)
    assert not output.exists()


def test_runtime_prepare_teacher_scenario_has_no_policy_binding(tmp_path):
    shared_root, scenario = _shared_root(tmp_path)
    _runtime_recipe(scenario, "runtime_teacher", "teacher")
    output = tmp_path / "teacher"
    manifest = prepare_run.prepare_run(
        scenario="synthetic_stack",
        role="teacher",
        shared_root=shared_root,
        seed=5,
        run_id="teacher-run",
        output=output,
        project_root=tmp_path / "repo",
    )
    runtime = yaml.safe_load((output / "runtime.yaml").read_text())
    assert runtime["action_source"] == "teacher"
    assert runtime["reset"]["seed"] == 5
    assert manifest["policy"] is None
    assert manifest["selection"]["scenario_id"] == "synthetic_stack"


def test_second_scenario_selects_another_recipe_without_name_branch(tmp_path):
    shared_root, baseline = _shared_root(tmp_path)
    baseline_recipe = _backend_recipe(
        baseline, 0.5, name="simulation_policy_baseline.yaml", chunk_history=512
    )
    baseline_document = baseline.copy()
    baseline_document["recipes"]["simulation_policy"] = "configs/simulation_policy_baseline.yaml"
    baseline.write(baseline_document)

    replacement = build_scenario(shared_root / "scenarios/test_only_replacement")
    replacement_document = replacement.copy()
    replacement_document["scenario_id"] = "test_only_replacement"
    replacement_document["description"] = "TEST ONLY: second scenario for replacement checks"
    replacement_document["recipes"]["simulation_policy"] = (
        "configs/simulation_policy_replacement.yaml"
    )
    replacement.write(replacement_document)
    replacement_recipe = _backend_recipe(
        replacement, 1.0, name="simulation_policy_replacement.yaml", chunk_history=256
    )

    baseline_output = tmp_path / "baseline-policy"
    baseline_manifest = prepare_backend.prepare_backend(
        scenario="synthetic_stack",
        role="policy",
        shared_root=shared_root,
        seed=7,
        run_id="baseline-policy",
        output=baseline_output,
    )
    replacement_output = tmp_path / "replacement-policy"
    replacement_manifest = prepare_backend.prepare_backend(
        scenario="test_only_replacement",
        role="policy",
        shared_root=shared_root,
        seed=7,
        run_id="replacement-policy",
        output=replacement_output,
    )

    baseline_backend = yaml.safe_load((baseline_output / "backend.yaml").read_text())
    replacement_backend = yaml.safe_load((replacement_output / "backend.yaml").read_text())
    assert baseline_backend["run"] == {"seed": 7, "realtime_speed": 0.5}
    assert baseline_backend["limits"] == {"chunk_history": 512}
    assert replacement_backend["run"] == {"seed": 7, "realtime_speed": 1.0}
    assert replacement_backend["limits"] == {"chunk_history": 256}
    assert baseline_backend != replacement_backend

    assert baseline_manifest["selection"]["scenario_id"] == "synthetic_stack"
    assert replacement_manifest["selection"]["scenario_id"] == "test_only_replacement"
    assert baseline_manifest["selection"]["recipe"]["path"] == str(
        baseline_recipe.resolve()
    )
    assert replacement_manifest["selection"]["recipe"]["path"] == str(
        replacement_recipe.resolve()
    )
    assert baseline_manifest["selection"]["binding"]["path"] == str(
        baseline.binding_path.resolve()
    )
    assert replacement_manifest["selection"]["binding"]["path"] == str(
        replacement.binding_path.resolve()
    )


def test_template_directory_is_not_a_selectable_scenario(tmp_path):
    with pytest.raises(prepare_backend.PrepareError, match="has no binding file"):
        prepare_backend.prepare_backend(
            scenario="_template",
            role="teacher",
            shared_root=ROOT / "shared",
            seed=0,
            run_id="template",
            output=tmp_path / "template",
        )
    assert not (tmp_path / "template").exists()
