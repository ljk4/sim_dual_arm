"""Marvin short inference recipe and campaign evaluation consistency."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
SCENARIO = ROOT / "shared/scenarios/marvin_handover"
TASK = SCENARIO / "configs/simulation/scene/marvin_handover_task.yaml"
INFERENCE_TASK = SCENARIO / "configs/simulation/scene/marvin_handover_task_inference.yaml"
RUNTIME_POLICY = SCENARIO / "configs/runtime/policy_eval.yaml"


def _load_task(path: Path):
    pytest.importorskip("mujoco")
    from robot_backend_sim.tasks.handover.task_config import load_handover_task_config

    return load_handover_task_config(path)


def test_teacher_plan_budget_only_applies_when_teacher_is_enabled():
    teacher_task = _load_task(TASK)
    assert teacher_task.teacher.enabled is True
    assert teacher_task.teacher.max_plan_duration_s < teacher_task.success.max_task_sim_duration_s

    inference = _load_task(INFERENCE_TASK)
    assert inference.teacher.enabled is False
    assert inference.success.max_task_sim_duration_s == 10.0
    assert inference.success.max_task_wall_duration_s == 60.0
    # The unchanged plan-duration value proves the check follows the switch
    # instead of the literal duration.
    assert inference.teacher.max_plan_duration_s > inference.success.max_task_sim_duration_s


def _runtime_with_verified_placeholders(tmp_path: Path) -> Path:
    document = yaml.safe_load(RUNTIME_POLICY.read_text(encoding="utf-8"))
    for enum in ("model_bundle_digest", "processor_digest"):
        document["policy"][enum] = "0" * 64
    for field in (
        "robot_profile",
        "observation_schema",
        "action_schema",
        "control_constraints",
        "follower_config",
    ):
        document[field] = str((RUNTIME_POLICY.parent / document[field]).resolve())
    target = tmp_path / "policy_eval.yaml"
    target.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return target


def test_runtime_policy_recipe_declares_bounded_task_execution(tmp_path):
    from dual_arm_runtime.runtime_config import load_runtime_configuration

    policy = load_runtime_configuration(_runtime_with_verified_placeholders(tmp_path), ROOT)
    stream = policy.recipe.policy
    assert stream is not None
    assert stream.termination_mode == "task"
    assert stream.task_sim_budget_ns == 10_000_000_000
    assert stream.task_stop_margin_s == 1.0
    assert stream.total_wall_deadline_s == 60.0
    assert stream.startup_settle_sim_ns == 2_000_000_000
    follower = policy.recipe.follower
    assert follower is not None
    covered = stream.max_chunks * follower.target_count * stream.action_period_ns
    assert covered >= stream.task_sim_budget_ns


def test_runtime_and_task_evaluation_budgets_agree(tmp_path):
    from dual_arm_runtime.runtime_config import load_runtime_configuration

    policy = load_runtime_configuration(_runtime_with_verified_placeholders(tmp_path), ROOT)
    stream = policy.recipe.policy
    assert stream is not None
    task = _load_task(INFERENCE_TASK)
    assert stream.task_sim_budget_ns == round(task.success.max_task_sim_duration_s * 1e9)
    assert stream.total_wall_deadline_s == task.success.max_task_wall_duration_s


def test_inference_following_budget_covers_measured_model_output_only():
    inference = yaml.safe_load(
        (
            SCENARIO / "profiles/control/marvin_follower_inference_v1.yaml"
        ).read_text(encoding="utf-8")
    )
    teacher = yaml.safe_load(
        (SCENARIO / "profiles/control/marvin_follower_v1.yaml").read_text(encoding="utf-8")
    )
    measured = [
        0.2675, 1.1691, 0.4657, 0.8825, 0.4904, 0.3937, 1.3685, 0.0030,
        0.0057, 0.0218, 0.0224, 0.0188, 0.0843, 0.0056, 0.0118, 0.0031,
    ]
    budgets = inference["max_target_deviations"]
    assert len(budgets) == 16
    assert inference["joint_names"] == teacher["joint_names"]
    assert inference["target_count"] == teacher["target_count"]
    assert all(budget >= value for budget, value in zip(budgets, measured, strict=True))
    # The Teacher profile keeps its own measured following budget.
    assert teacher["max_target_deviations"][:7] == [0.05] * 7


def test_marvin_binding_declares_learning_and_policy_recipes():
    document = yaml.safe_load((SCENARIO / "binding.yaml").read_text(encoding="utf-8"))
    recipes = document["recipes"]
    for name in (
        "simulation_policy",
        "runtime_policy",
        "dataset",
        "training",
        "policy_service",
    ):
        assert (SCENARIO / recipes[name]).is_file()
    dataset = yaml.safe_load(
        (SCENARIO / recipes["dataset"]).read_text(encoding="utf-8")
    )
    assert dataset["diagnostic"] is True
    assert dataset["expected"] == {
        "formal_episodes": 6,
        "training_episodes": 4,
        "validation_episodes": 2,
        "validation_positions": [4, 5],
    }
    assert document["interfaces"]["policy_api"] == "bimanual.policy.v1"
    policy_service = yaml.safe_load(
        (SCENARIO / recipes["policy_service"]).read_text(encoding="utf-8")
    )
    assert policy_service["service"]["test_only"] is False
    assert policy_service["model"]["gripper_max_correction_m"] == 0.005
    assert policy_service["model"]["temporal_ensemble_coeff"] is None
