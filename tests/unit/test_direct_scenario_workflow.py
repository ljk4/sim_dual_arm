"""Direct ``--robot/--task`` loading for simulation and runtime (no prepared dirs)."""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import pytest
import yaml
from dual_arm_runtime.runtime_config import load_runtime_configuration_document
from dual_arm_runtime.scenario_runtime import (
    ScenarioRuntimeError,
    resolve_scenario_runtime,
)
from robot_backend_sim.config.backend_config import load_backend_document
from robot_backend_sim.config.scenario_config import (
    ScenarioError,
    resolve_scenario_backend,
)
from scenario_fixtures import ROOT, build_scenario

SIM_RECIPE = (
    ROOT / "shared/scenarios/panda_sim/stack/configs/simulation/backend/backend_stack_teacher.yaml"
)
RUNTIME_TEACHER = ROOT / "shared/scenarios/panda_sim/stack/configs/runtime/teacher_collect.yaml"
RUNTIME_POLICY = ROOT / "shared/scenarios/panda_sim/stack/configs/runtime/policy_rollout.yaml"


def _shared(tmp_path: Path):
    repo = tmp_path / "repo"
    shared = repo / "shared"
    scenario = build_scenario(shared / "scenarios/synthetic_sim/stack")
    shutil.copytree(
        ROOT / "shared/common/interfaces/schemas", shared / "common/interfaces/schemas"
    )
    return repo, shared, scenario


def _touch(path: Path, text: str = "schema_version: 1\n") -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    return path


def _simulation_recipe(scenario, shared: Path) -> Path:
    document = yaml.safe_load(SIM_RECIPE.read_text())
    robot = document["robot"]
    robot.pop("task", None)
    robot.pop("task_mode", None)
    robot["render_quality"] = {"enabled": False}
    robot["profile"] = str(scenario.root / "profiles/robot.json")
    robot["schemas"] = str(shared / "common/interfaces/schemas")
    for name in ("scene", "control", "stream", "follower"):
        robot[name] = str(_touch(scenario.root / f"profiles/{name}.yaml", "{}"))
    document["run"].update({"seed": 0, "realtime_speed": 1.0})
    path = scenario.root / "configs/simulation_teacher.yaml"
    path.write_text(yaml.safe_dump(document, sort_keys=False))
    return path


def _follower_document(profile: dict) -> dict:
    names = [joint["name"] for joint in profile["joints"]]
    return {
        "joint_names": names,
        "target_count": 10,
        "max_target_deviations": [0.1] * len(names),
        "max_terminal_tail_duration_s": 0.5,
    }


def _runtime_recipe(scenario, action_source: str, *, budget_ns: int = 30_000_000_000) -> Path:
    profile = json.loads((scenario.root / "profiles/robot.json").read_text())
    control = _touch(scenario.root / "profiles/control.yaml")
    follower = scenario.root / "profiles/follower.yaml"
    follower.write_text(yaml.safe_dump(_follower_document(profile)))
    template = RUNTIME_TEACHER if action_source == "teacher" else RUNTIME_POLICY
    document = yaml.safe_load(template.read_text())
    document["robot_profile"] = str(scenario.root / "profiles/robot.json")
    document["observation_schema"] = str(scenario.root / "profiles/observation.json")
    document["action_schema"] = str(scenario.root / "profiles/action.json")
    document["control_constraints"] = str(control)
    document["follower_config"] = str(follower)
    if action_source == "teacher":
        document["teacher"]["task_sim_budget_ns"] = budget_ns
    path = scenario.root / f"configs/runtime_{action_source}.yaml"
    path.write_text(yaml.safe_dump(document, sort_keys=False))
    return path


def _teacher_task(scenario, *, duration_s: float = 30.0) -> None:
    backend = scenario.root / "configs/simulation_teacher.yaml"
    task = scenario.root / "configs/task.yaml"
    task.write_text(
        yaml.safe_dump(
            {
                "success": {"max_task_sim_duration_s": duration_s},
                "teacher": {"chunk_size": 60, "action_period_ns": 50_000_000},
            }
        )
    )
    document = yaml.safe_load(backend.read_text())
    document["robot"]["task"] = "task.yaml"
    document["robot"]["task_mode"] = "teacher"
    backend.write_text(yaml.safe_dump(document, sort_keys=False))


def test_simulation_scenario_expands_paths_seed_and_overrides(tmp_path):
    repo, shared, scenario = _shared(tmp_path)
    recipe = _simulation_recipe(scenario, shared)
    resolved = resolve_scenario_backend(
        robot="synthetic_sim", task="stack",
        role="teacher",
        shared_root=shared,
        seed=7,
        overrides=["run.realtime_speed=0.4"],
    )
    assert resolved.selection["robot"] == "synthetic_sim"
    assert resolved.selection["task"] == "stack"
    assert resolved.selection["recipe"]["path"] == str(recipe.resolve())
    assert resolved.document["robot"]["profile"] == str(
        (scenario.root / "profiles/robot.json").resolve()
    )
    config = load_backend_document(resolved.document, resolved.source)
    assert config.seed == 7
    assert config.realtime_speed == 0.4
    assert config.source == Path(resolved.source)


def test_simulation_scenario_rejects_a_different_robot_profile(tmp_path):
    _repo, shared, scenario = _shared(tmp_path)
    _simulation_recipe(scenario, shared)
    other = _touch(scenario.root / "profiles/other.json", "{}")
    document = yaml.safe_load((scenario.root / "configs/simulation_teacher.yaml").read_text())
    document["robot"]["profile"] = str(other)
    (scenario.root / "configs/simulation_teacher.yaml").write_text(
        yaml.safe_dump(document, sort_keys=False)
    )
    with pytest.raises(ScenarioError, match="differs from binding profile"):
        resolve_scenario_backend(
            robot="synthetic_sim", task="stack", role="teacher", shared_root=shared
        )


def test_runtime_teacher_scenario_injects_api_and_checks_budget(tmp_path):
    repo, shared, scenario = _shared(tmp_path)
    _simulation_recipe(scenario, shared)
    _teacher_task(scenario)
    _runtime_recipe(scenario, "teacher")
    resolved = resolve_scenario_runtime(
        robot="synthetic_sim", task="stack",
        role="teacher",
        shared_root=shared,
        project_root=repo,
        seed=5,
        overrides=["observation.max_age_ns=250000000"],
    )
    assert resolved.document["teacher_api"] == "panda_stack_api_msgs.teacher_chunk.v1"
    configuration = load_runtime_configuration_document(
        resolved.document,
        source=resolved.source,
        base_dir=resolved.base_dir,
        project_root=repo,
        overrides=resolved.overrides,
        selection=resolved.selection,
    )
    assert configuration.recipe.reset.seed == 5
    assert configuration.recipe.observation.max_age_ns == 250_000_000
    assert configuration.selection["role"] == "teacher"
    assert configuration.snapshot["overrides"] == ["observation.max_age_ns=250000000"]


def test_runtime_teacher_scenario_rejects_budget_mismatch(tmp_path):
    repo, shared, scenario = _shared(tmp_path)
    _simulation_recipe(scenario, shared)
    _teacher_task(scenario)
    _runtime_recipe(scenario, "teacher", budget_ns=31_000_000_000)
    with pytest.raises(ScenarioRuntimeError, match="task_sim_budget_ns"):
        resolve_scenario_runtime(
            robot="synthetic_sim", task="stack",
            role="teacher",
            shared_root=shared,
            project_root=repo,
        )


def test_runtime_policy_scenario_needs_no_bundle_or_model_identity(tmp_path):
    repo, shared, scenario = _shared(tmp_path)
    _runtime_recipe(scenario, "policy")
    resolved = resolve_scenario_runtime(
        robot="synthetic_sim", task="stack",
        role="policy",
        shared_root=shared,
        project_root=repo,
    )
    policy = resolved.document["policy"]
    assert "model_bundle_digest" not in policy
    assert "processor_digest" not in policy
    assert "bundle" not in resolved.selection
    configuration = load_runtime_configuration_document(
        resolved.document,
        source=resolved.source,
        base_dir=resolved.base_dir,
        project_root=repo,
        selection=resolved.selection,
    )
    assert configuration.recipe.action_source == "policy"
    assert not hasattr(configuration.recipe.policy, "model_bundle_digest")


def test_runtime_scenario_rejects_unknown_override_path(tmp_path):
    repo, shared, scenario = _shared(tmp_path)
    _simulation_recipe(scenario, shared)
    _teacher_task(scenario)
    _runtime_recipe(scenario, "teacher")
    resolved = resolve_scenario_runtime(
        robot="synthetic_sim", task="stack",
        role="teacher",
        shared_root=shared,
        project_root=repo,
        overrides=["observation.not_a_field=1"],
    )
    with pytest.raises(Exception, match="does not exist"):
        load_runtime_configuration_document(
            resolved.document,
            source=resolved.source,
            base_dir=resolved.base_dir,
            project_root=repo,
            overrides=resolved.overrides,
            selection=resolved.selection,
        )
