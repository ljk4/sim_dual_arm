"""ACT inference recipes bind execution windows while preserving physical limits."""

from __future__ import annotations

from pathlib import Path

import yaml
from dual_arm_runtime.runtime_config import load_runtime_configuration
from robot_backend_sim.config.backend_config import load_backend_config
from robot_backend_sim.config.follower_config import load_follower_config

ROOT = Path(__file__).resolve().parents[2]
SCENARIO = ROOT / "shared/scenarios/panda_stack"
FOLLOWER = SCENARIO / "profiles/control/panda_follower_v1.yaml"
BACKEND_CHUNKED = SCENARIO / "configs/simulation/backend/backend_stack_act_chunked.yaml"
RUNTIME_CHUNKED = SCENARIO / "configs/runtime/policy_rollout.yaml"
RUNTIME_TEACHER = SCENARIO / "configs/runtime/teacher_collect.yaml"

_REFERENCED_FILES = (
    "robot_profile",
    "observation_schema",
    "action_schema",
    "control_constraints",
    "follower_config",
)


def materialize_runtime(template: Path, directory: Path) -> Path:
    """Apply absolute paths and verified-identity placeholders used by prepare_run."""
    document = yaml.safe_load(template.read_text(encoding="utf-8"))
    for field in _REFERENCED_FILES:
        document[field] = str((template.parent / document[field]).resolve())
    policy = document["policy"]
    policy["model_bundle_digest"] = "a" * 64
    policy["processor_digest"] = "b" * 64
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / template.name
    path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    return path


def load_runtime(template: Path, directory: Path):
    path = materialize_runtime(template, directory)
    return load_runtime_configuration(path, ROOT)


def test_single_follower_source_owns_target_count_and_deviation_budget():
    config = load_follower_config(FOLLOWER)
    assert config.target_count == 5
    assert config.target_count * 50_000_000 == 250_000_000
    assert config.max_target_deviations == ((0.30,) * 6 + (0.40, 0.04) + (0.30,) * 6 + (0.40, 0.04))
    assert config.max_terminal_tail_duration_s == 3.0


def test_policy_backend_references_the_single_follower_source():
    chunked = load_backend_config(BACKEND_CHUNKED)
    assert chunked.follower_path == FOLLOWER.resolve()
    assert chunked.task_mode == "inference"
    assert chunked.realtime_speed == 0.5
    assert chunked.chunk_history_capacity == 512


def test_runtime_configs_reference_the_single_follower_source():
    for template in (RUNTIME_CHUNKED, RUNTIME_TEACHER):
        document = yaml.safe_load(template.read_text(encoding="utf-8"))
        assert "target_count" not in document.get("policy", {})
        assert "target_count" not in document.get("teacher", {})
        assert (template.parent / document["follower_config"]).resolve() == FOLLOWER.resolve()


def test_chunked_runtime_keeps_the_60x16_interface_and_covers_the_task(tmp_path):
    runtime = load_runtime(RUNTIME_CHUNKED, tmp_path / "chunked")
    policy = runtime.recipe.policy
    follower = runtime.recipe.follower
    assert policy is not None
    assert follower is not None
    assert runtime.recipe.action_source == "policy"
    assert runtime.recipe.teacher is None

    loaded_follower = load_follower_config(FOLLOWER)
    assert follower.target_count == loaded_follower.target_count == 5
    assert policy.chunk_size == 60
    assert policy.action_period_ns == 50_000_000
    assert follower.duration_grid_ns == 2_000_000

    window_ns = follower.target_count * policy.action_period_ns
    assert window_ns == 250_000_000
    assert policy.max_chunks == 256
    assert policy.max_chunks * window_ns == 64_000_000_000
    assert policy.startup_settle_sim_ns == 2_000_000_000
    assert policy.max_chunks * window_ns >= policy.task_sim_budget_ns
    assert policy.total_sim_deadline_ns == policy.task_sim_budget_ns == 30_000_000_000
    assert policy.total_wall_deadline_s == 120.0

    assert follower.max_target_deviations == loaded_follower.max_target_deviations
    assert follower.max_terminal_tail_duration_ns == loaded_follower.max_terminal_tail_duration_ns
    assert len(runtime.snapshot["action"]["joint_names"]) == 16
    assert policy.task_id == "panda-stack"
    assert policy.task_text.startswith("Pick up the red source cube")
