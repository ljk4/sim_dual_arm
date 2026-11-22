from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SCENARIO = ROOT / "shared/scenarios/marvin_sim/stack"
SIM_SRC = ROOT / "simulation/mujoco/src"
CONTRACTS = ROOT / "shared/common/python/contracts/src"
for source in (SIM_SRC, CONTRACTS):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

pytest.importorskip("mujoco")
import mujoco  # noqa: E402
from robot_backend_sim.marvin_sim.stack.config import load_task_config  # noqa: E402
from robot_backend_sim.marvin_sim.stack.evaluator import (  # noqa: E402
    StackTaskRuntime,
    capture_task_observation,
    evaluate_task,
    make_task_binding,
)
from robot_backend_sim.marvin_sim.stack.scene import build_marvin_stack_scene  # noqa: E402
from robot_backend_sim.marvin_sim.stack.scene_config import load_marvin_stack_scene_config  # noqa: E402
from robot_contracts.models import RunContext  # noqa: E402


@pytest.fixture(scope="module")
def scene_fixture():
    recipes = SCENARIO / "configs/simulation/scene"
    scene_config = load_marvin_stack_scene_config(recipes / "marvin_stack.yaml")
    scene = build_marvin_stack_scene(scene_config)
    scene.reset(0)
    task_config = load_task_config(recipes / "marvin_stack_task.yaml")
    binding = make_task_binding(
        scene.model,
        task_config,
        table_top_z_m=scene.config.robot.base_position[2] + scene.config.table.top_z,
    )
    return scene, task_config, binding


def _place_static_stack(scene) -> None:
    """Test-only physical fixture: not a data episode and not teacher success."""
    indices = scene.indices
    target_address = indices.environment.object_free_joint_qpos["target"]
    source_address = indices.environment.object_free_joint_qpos["source"]
    target = scene.data.qpos[target_address : target_address + 7].copy()
    target[3:7] = (1.0, 0.0, 0.0, 0.0)
    scene.data.qpos[target_address : target_address + 7] = target
    z = target[2] + 0.03 + 0.02 + 0.001
    scene.data.qpos[source_address : source_address + 7] = (
        target[0],
        target[1],
        z,
        1.0,
        0.0,
        0.0,
        0.0,
    )
    scene.data.qvel[:] = 0.0
    mujoco.mj_forward(scene.model, scene.data)
    for _ in range(2500):
        mujoco.mj_step(scene.model, scene.data)
        mujoco.mj_forward(scene.model, scene.data)


def test_free_initial_objects_are_not_success(scene_fixture):
    scene, task_config, binding = scene_fixture
    scene.reset(11)
    observation = capture_task_observation(scene, binding, task_config)
    evaluation = evaluate_task(observation, binding, task_config)
    assert not evaluation.all_passed
    assert not evaluation.condition("source_on_target_height").passed
    assert not evaluation.condition("source_target_vertical_contact").passed


def test_static_physical_fixture_passes_and_latches_at_two_seconds(scene_fixture):
    scene, task_config, binding = scene_fixture
    scene.reset(0)
    _place_static_stack(scene)
    observation = capture_task_observation(scene, binding, task_config)
    assert evaluate_task(observation, binding, task_config).all_passed

    runtime = StackTaskRuntime(task_config, binding)
    context = RunContext("robot", "profile", "backend", "time", 1, "episode", 1)
    runtime.commit_prepare(
        request_id="prepare",
        content_digest="prepare-content",
        context=context,
        plan=object(),
        plan_digest="a" * 64,
        plan_duration_ns=20_000_000_000,
    )
    runtime.start(
        request_id="start",
        content_digest="start-content",
        context=context,
        plan_digest="a" * 64,
        task_epoch=runtime.task_epoch,
        sim_time_ns=observation.sim_time_ns,
        wall_ns=0,
    )
    state = runtime.tick(observation, 10)
    assert state.phase == "RUNNING"
    for _ in range(1200):
        mujoco.mj_step(scene.model, scene.data)
        mujoco.mj_forward(scene.model, scene.data)
        state = runtime.tick(capture_task_observation(scene, binding, task_config), 10)
        if state.phase == "SUCCEEDED":
            break
    assert state.phase == "SUCCEEDED"
    assert state.metric("reason") == "SUCCESS"
    assert (
        state.metric("completed_sim_time_ns") - state.metric("task_start_sim_time_ns")
        >= 2_000_000_000
    )
