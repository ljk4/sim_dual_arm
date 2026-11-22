from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest
from model_helpers import replace

ROOT = Path(__file__).resolve().parents[3]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.marvin_sim.stack.config import load_task_config  # noqa: E402
from robot_backend_sim.marvin_sim.stack.evaluator import (  # noqa: E402
    ObjectState,
    StackTaskRuntime,
    TaskBinding,
    TaskObservation,
    evaluate_task,
)
from robot_contracts.models import ContractError, RunContext  # noqa: E402

CONFIG_PATH = (
    ROOT / "shared/scenarios/marvin_sim/stack/configs/simulation/scene/marvin_stack_task.yaml"
)
TABLE_TOP = 0.78
TARGET_Z = TABLE_TOP + 0.03
SOURCE_Z = TARGET_Z + 0.03 + 0.02
SOURCE_Q = (1.0, 0.0, 0.0, 0.0)
TARGET_Q = (1.0, 0.0, 0.0, 0.0)


def binding() -> TaskBinding:
    return TaskBinding(
        source_body_id=3,
        target_body_id=2,
        table_body_id=1,
        finger_body_ids=(20, 21, 30, 31),
        right_finger_body_ids=(30, 31),
        left_finger_body_ids=(20, 21),
        source_qpos_address=20,
        target_qpos_address=13,
        source_dof_address=18,
        target_dof_address=12,
        right_tcp_site_id=0,
        source_half_extents_m=(0.02, 0.02, 0.02),
        target_half_extents_m=(0.03, 0.03, 0.03),
        table_top_z_m=TABLE_TOP,
    )


def observation(**changes) -> TaskObservation:
    source = ObjectState(
        (0.55, -0.30, SOURCE_Z), SOURCE_Q, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    )
    target = ObjectState(
        (0.55, -0.30, TARGET_Z), TARGET_Q, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    )
    base = dict(
        sim_time_ns=0,
        source=source,
        target=target,
        source_target_normal_force_n=0.4,
        target_table_normal_force_n=0.6,
        source_table_normal_force_n=0.0,
        finger_contact_count=0,
        right_gripper_opening_m=0.0814,
        right_tcp_position=(0.55, -0.30, 1.05),
        right_tcp_to_source_m=0.22,
        table_top_z_m=TABLE_TOP,
    )
    base.update(changes)
    return TaskObservation(**base)


@pytest.fixture(scope="module")
def config():
    return load_task_config(CONFIG_PATH)


def test_positive_static_stack_passes_every_condition(config):
    value = observation()
    evaluation = evaluate_task(value, binding(), config)
    assert evaluation.all_passed
    assert [condition.name for condition in evaluation.conditions] == [
        "source_on_target_height",
        "source_xy_inside_target",
        "source_target_vertical_contact",
        "target_on_table",
        "source_not_on_table",
        "source_tilt",
        "target_tilt",
        "source_linear_speed",
        "target_linear_speed",
        "source_angular_speed",
        "target_angular_speed",
        "no_finger_contact",
        "right_gripper_opening",
        "right_tcp_clearance",
    ]


@pytest.mark.parametrize(
    ("changes", "failed"),
    [
        ({"source": replace(observation().source, center=(0.55, -0.30, TARGET_Z + 0.02))},
         "source_on_target_height"),
        ({"source": replace(observation().source, center=(0.59, -0.30, SOURCE_Z))},
         "source_xy_inside_target"),
        ({"source_target_normal_force_n": 0.0}, "source_target_vertical_contact"),
        ({"target_table_normal_force_n": 0.0}, "target_on_table"),
        ({"source_table_normal_force_n": 0.4}, "source_not_on_table"),
        ({"finger_contact_count": 1}, "no_finger_contact"),
        ({"right_gripper_opening_m": 0.04}, "right_gripper_opening"),
        ({"right_tcp_to_source_m": 0.05}, "right_tcp_clearance"),
        ({"source_table_normal_force_n": 0.0, "target": replace(
            observation().target, center=(0.55, -0.30, TARGET_Z + 0.02)
        )}, "target_on_table"),
    ],
)
def test_each_physical_shortcut_is_rejected(config, changes, failed):
    evaluation = evaluate_task(observation(**changes), binding(), config)
    assert not evaluation.all_passed
    assert not evaluation.condition(failed).passed


def test_tilt_and_velocity_are_not_ignored(config):
    tilted = replace(observation().source, quaternion_wxyz=(0.966, 0.259, 0.0, 0.0))
    evaluation = evaluate_task(observation(source=tilted), binding(), config)
    assert not evaluation.condition("source_tilt").passed
    moving = replace(observation().source, linear_velocity=(0.05, 0.0, 0.0))
    evaluation = evaluate_task(observation(source=moving), binding(), config)
    assert not evaluation.condition("source_linear_speed").passed
    spinning = replace(observation().target, angular_velocity=(0.0, 0.0, 0.3))
    evaluation = evaluate_task(observation(target=spinning), binding(), config)
    assert not evaluation.condition("target_angular_speed").passed


def _yaw_quaternion(degrees: float) -> tuple[float, float, float, float]:
    half = math.radians(degrees) / 2.0
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def _world_delta(local: tuple[float, float, float], yaw_degrees: float):
    """Rotate a target-local offset into the world with plain planar geometry."""
    angle = math.radians(yaw_degrees)
    x, y, z = local
    return (
        x * math.cos(angle) - y * math.sin(angle),
        x * math.sin(angle) + y * math.cos(angle),
        z,
    )


def _place_at_target_local(yaw_degrees: float, local) -> TaskObservation:
    rotated = replace(
        observation().target, quaternion_wxyz=_yaw_quaternion(yaw_degrees)
    )
    delta = _world_delta(local, yaw_degrees)
    source = replace(
        observation().source,
        center=tuple(rotated.center[index] + delta[index] for index in range(3)),
    )
    return observation(source=source, target=rotated)


@pytest.mark.parametrize("yaw_degrees", [-10.0, 0.0, 10.0])
def test_xy_support_projection_uses_the_target_local_frame(config, yaw_degrees):
    evaluation = evaluate_task(
        _place_at_target_local(yaw_degrees, (0.027, 0.027, 0.05)),
        binding(),
        config,
    )
    condition = evaluation.condition("source_xy_inside_target")
    assert condition.actual == pytest.approx(-0.001, abs=1e-9)
    assert condition.passed
    assert evaluation.all_passed


@pytest.mark.parametrize("yaw_degrees", [-10.0, 10.0])
@pytest.mark.parametrize(
    ("local", "expected", "passed"),
    [
        ((0.027, 0.027, 0.05), -0.001, True),
        ((0.027999, 0.027, 0.05), -1e-6, True),
        ((0.028001, 0.027, 0.05), 1e-6, False),
    ],
)
def test_xy_support_margin_boundary_rotates_with_the_target(
    config, yaw_degrees, local, expected, passed
):
    evaluation = evaluate_task(
        _place_at_target_local(yaw_degrees, local), binding(), config
    )
    condition = evaluation.condition("source_xy_inside_target")
    assert condition.actual == pytest.approx(expected, abs=1e-9)
    assert condition.passed is passed


@pytest.mark.parametrize("name", ["source_tilt", "target_tilt"])
def test_ninety_degree_block_flip_is_rejected(config, name):
    flipped = (math.cos(math.pi / 4.0), math.sin(math.pi / 4.0), 0.0, 0.0)
    changes = {
        "source": replace(observation().source, quaternion_wxyz=flipped),
        "target": replace(observation().target, quaternion_wxyz=flipped),
    }
    evaluation = evaluate_task(observation(**changes), binding(), config)
    assert not evaluation.condition(name).passed
    assert not evaluation.all_passed


def _runtime(config) -> tuple[StackTaskRuntime, RunContext]:
    runtime = StackTaskRuntime(config, binding())
    context = RunContext("robot", "p", "backend", "time", 1, "episode", 1)
    runtime.commit_prepare(
        request_id="prepare-1",
        content_digest="prepare-content",
        context=context,
        plan=object(),
        plan_digest="a" * 64,
        plan_duration_ns=20_000_000_000,
    )
    runtime.start(
        request_id="start-1",
        content_digest="start-content",
        context=context,
        plan_digest="a" * 64,
        task_epoch=runtime.task_epoch,
        sim_time_ns=0,
        wall_ns=0,
    )
    return runtime, context


def test_stable_window_requires_two_seconds_and_latches(config):
    runtime, _context = _runtime(config)
    state = runtime.tick(observation(sim_time_ns=0), 10)
    assert state.phase == "RUNNING"
    assert state.metric("first_satisfied_sim_time_ns") == 0
    state = runtime.tick(observation(sim_time_ns=1_999_000_000), 10)
    assert state.phase == "RUNNING"
    state = runtime.tick(observation(sim_time_ns=2_000_000_000), 10)
    assert state.phase == "SUCCEEDED"
    assert state.metric("reason") == "SUCCESS"
    assert state.metric("first_satisfied_sim_time_ns") == 0
    assert state.metric("completed_sim_time_ns") == 2_000_000_000
    # Success is latched even if a later sample jitters.
    state = runtime.tick(observation(sim_time_ns=2_100_000_000, finger_contact_count=1), 10)
    assert state.phase == "SUCCEEDED"


def test_any_failed_condition_resets_the_window(config):
    runtime, _context = _runtime(config)
    runtime.tick(observation(sim_time_ns=0), 10)
    runtime.tick(observation(sim_time_ns=1_000_000_000), 10)
    state = runtime.tick(
        observation(sim_time_ns=1_500_000_000, finger_contact_count=1), 10
    )
    assert state.phase == "RUNNING"
    assert state.metric("first_satisfied_sim_time_ns") == 0
    state = runtime.tick(observation(sim_time_ns=2_000_000_000), 10)
    assert state.metric("first_satisfied_sim_time_ns") == 2_000_000_000
    state = runtime.tick(observation(sim_time_ns=3_999_000_000), 10)
    assert state.phase == "RUNNING"
    state = runtime.tick(observation(sim_time_ns=4_000_000_000), 10)
    assert state.phase == "SUCCEEDED"


def test_timeout_and_wall_watchdog_are_independent_reasons(config):
    runtime, _context = _runtime(config)
    state = runtime.tick(observation(sim_time_ns=90_000_000_001), 10)
    assert state.phase == "FAILED"
    assert state.metric("reason").startswith("TIMEOUT")
    runtime, _context = _runtime(config)
    state = runtime.tick(observation(sim_time_ns=1_000_000_000), 300_000_000_001)
    assert state.phase == "FAILED"
    assert state.metric("reason").startswith("WALL_TIMEOUT")


def test_reset_clears_plan_window_epoch_and_rpc(config):
    runtime, context = _runtime(config)
    epoch = runtime.task_epoch
    runtime.tick(observation(sim_time_ns=0), 10)
    state = runtime.reset()
    assert state.phase == "IDLE"
    assert state.task_epoch == epoch + 1
    assert state.plan_digest == ""
    assert runtime.plan is None
    with pytest.raises(ContractError, match="TASK_NOT_PREPARED"):
        runtime.start(
            request_id="start-1",
            content_digest="start-content",
            context=context,
            plan_digest="a" * 64,
            task_epoch=epoch,
            sim_time_ns=0,
            wall_ns=0,
        )


def test_prepare_and_start_idempotency_and_conflict(config):
    runtime = StackTaskRuntime(config, binding())
    context = RunContext("robot", "p", "backend", "time", 1, "episode", 1)
    first = runtime.commit_prepare(
        request_id="prepare-1",
        content_digest="c1",
        context=context,
        plan=object(),
        plan_digest="a" * 64,
        plan_duration_ns=20_000_000_000,
    )
    second = runtime.commit_prepare(
        request_id="prepare-1",
        content_digest="c1",
        context=context,
        plan=object(),
        plan_digest="b" * 64,
        plan_duration_ns=19_000_000_000,
    )
    assert second.idempotent
    assert second.fact.plan_digest == first.fact.plan_digest
    with pytest.raises(ContractError, match="REQUEST_ID_CONFLICT"):
        runtime.lookup_prepare("prepare-1", "different")
    epoch = runtime.task_epoch
    runtime.start(
        request_id="start-1",
        content_digest="s1",
        context=context,
        plan_digest="a" * 64,
        task_epoch=epoch,
        sim_time_ns=0,
        wall_ns=0,
    )
    state, idempotent = runtime.start(
        request_id="start-1",
        content_digest="s1",
        context=context,
        plan_digest="a" * 64,
        task_epoch=epoch,
        sim_time_ns=0,
        wall_ns=0,
    )
    assert idempotent
    assert state.metric("task_start_sim_time_ns") == 0
    with pytest.raises(ContractError, match="START_CONFLICT"):
        runtime.start(
            request_id="start-2",
            content_digest="s2",
            context=context,
            plan_digest="a" * 64,
            task_epoch=epoch,
            sim_time_ns=0,
            wall_ns=0,
        )
