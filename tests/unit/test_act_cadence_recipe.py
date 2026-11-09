"""ACT inference recipes bind execution windows while preserving physical limits."""

from __future__ import annotations

from pathlib import Path

import yaml
from dual_arm_runtime.native_config import load_native_configuration
from robot_backend_sim.backend_config import BackendConfig, load_backend_config
from robot_backend_sim.follower_config import load_follower_config

ROOT = Path(__file__).resolve().parents[2]
FOLLOWER = ROOT / "simulation/mujoco/configs/control/panda_follower.yaml"
FOLLOWER_ACT = ROOT / "simulation/mujoco/configs/control/panda_follower_act.yaml"
FOLLOWER_ACT_CHUNKED = ROOT / "simulation/mujoco/configs/control/panda_follower_act_chunked.yaml"
BACKEND = ROOT / "simulation/mujoco/configs/backend/backend_stack_inference.yaml"
BACKEND_ACT = ROOT / "tests/fixtures/simulation/backend_stack_act.yaml"
BACKEND_ACT_CHUNKED = ROOT / "simulation/mujoco/configs/backend/backend_stack_act_chunked.yaml"
RUNTIME_CHUNKED = ROOT / "runtime/ros2/configs/native_task_act_chunked.yaml"

_REFERENCED_FILES = (
    "robot_profile",
    "observation_schema",
    "action_schema",
    "control_constraints",
    "follower_constraints",
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
    return load_native_configuration(path, ROOT)


def test_act_follower_keeps_the_accepted_limits_and_selects_two_points():
    old = load_follower_config(FOLLOWER)
    new = load_follower_config(FOLLOWER_ACT)
    assert new.target_count == 2
    assert new.joint_names == old.joint_names
    assert new.max_target_deviations == old.max_target_deviations
    assert new.max_terminal_tail_duration_s == old.max_terminal_tail_duration_s
    changed = {key for key in old.to_document() if old.to_document()[key] != new.to_document()[key]}
    assert changed == {"target_count"}


def test_chunked_follower_uses_the_approved_simulation_reference_budget():
    old = load_follower_config(FOLLOWER_ACT)
    new = load_follower_config(FOLLOWER_ACT_CHUNKED)
    assert new.target_count == 10
    assert new.target_count * 50_000_000 == 500_000_000
    assert new.joint_names == old.joint_names
    assert new.max_target_deviations == tuple(
        0.04 if index in (7, 15) else 0.40 if index in (6, 14) else 0.30
        for index in range(16)
    )
    assert new.max_terminal_tail_duration_s == old.max_terminal_tail_duration_s
    old_document = old.to_document()
    new_document = new.to_document()
    changed = {key for key in old_document if old_document[key] != new_document[key]}
    assert changed == {"target_count", "max_target_deviations"}


def test_chunked_backend_only_redirects_the_follower_and_stays_inference():
    old = load_backend_config(BACKEND_ACT)
    new = load_backend_config(BACKEND_ACT_CHUNKED)
    assert new.follower_path == FOLLOWER_ACT_CHUNKED.resolve()
    assert new.task_mode == old.task_mode == "inference"
    assert new.realtime_speed == 0.5
    assert new.chunk_history_capacity == 512
    # source is the file path itself; every other field except the follower
    # redirect must match the accepted 2-point ACT backend exactly.
    raw_paths = {"profile", "schemas", "scene", "control", "stream", "task", "follower"}
    assert (new.robot.model_dump(exclude=raw_paths)
            == old.robot.model_dump(exclude=raw_paths))
    for name in BackendConfig.model_fields:
        if name in {"source", "robot", "follower_path"}:
            continue
        assert getattr(new, name) == getattr(old, name)


def test_chunked_runtime_keeps_the_60x16_interface_and_covers_the_task(tmp_path):
    runtime = load_runtime(RUNTIME_CHUNKED, tmp_path / "chunked")
    policy = runtime.recipe.policy
    follower = runtime.recipe.follower
    assert policy is not None
    assert follower is not None
    assert runtime.recipe.action_source == "policy"
    assert runtime.recipe.teacher is None
    assert (
        RUNTIME_CHUNKED.parent
        / yaml.safe_load(RUNTIME_CHUNKED.read_text(encoding="utf-8"))["follower_constraints"]
    ).resolve() == FOLLOWER_ACT_CHUNKED.resolve()

    loaded_follower = load_follower_config(FOLLOWER_ACT_CHUNKED)
    assert policy.target_count == follower.target_count == loaded_follower.target_count == 10
    assert policy.chunk_size == 60
    assert policy.action_period_ns == 50_000_000
    assert follower.duration_grid_ns == 2_000_000

    window_ns = policy.target_count * policy.action_period_ns
    assert window_ns == 500_000_000
    assert policy.max_chunks == 64
    assert policy.max_chunks * window_ns == 32_000_000_000
    assert policy.startup_settle_sim_ns == 2_000_000_000
    assert policy.max_chunks * window_ns >= policy.task_sim_budget_ns
    assert policy.total_sim_deadline_ns == policy.task_sim_budget_ns == 30_000_000_000
    assert policy.total_wall_deadline_s == 120.0

    assert follower.max_target_deviations == loaded_follower.max_target_deviations
    assert follower.max_terminal_tail_duration_ns == loaded_follower.max_terminal_tail_duration_ns
    assert len(runtime.snapshot["action"]["joint_names"]) == 16
    assert policy.task_id == "panda-stack"
    assert policy.task_text.startswith("Pick up the red source cube")
