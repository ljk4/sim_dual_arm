"""Teacher history must retain observations and requests for the declared run."""

from collections import deque
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

pytest.importorskip("mujoco")

from robot_backend_sim.common.tasks.base import CAPTURE_HISTORY_LIMIT, TaskChunkView  # noqa: E402
from robot_backend_sim.marvin_sim.handover import plugin as handover_plugin  # noqa: E402
from robot_backend_sim.marvin_sim.handover.task_config import (  # noqa: E402
    load_handover_task_config,
)
from robot_backend_sim.marvin_sim.stack import plugin as stack_plugin  # noqa: E402
from robot_backend_sim.marvin_sim.stack.config import load_task_config  # noqa: E402
from robot_contracts.models import ContractError, RunContext  # noqa: E402
from robot_contracts.task_state import TaskState  # noqa: E402

SCENARIOS = Path(__file__).resolve().parents[2] / "shared/scenarios/marvin_sim"
CONTEXT = RunContext("marvin", "profile", "backend", "time", 1, "episode", 1)
PERIOD_NS = 50_000_000


def _running_state(task_type):
    return TaskState(
        context=CONTEXT,
        state_seq=1,
        sample_time_ns=0,
        task_epoch=1,
        task_type=task_type,
        phase="RUNNING",
    )


@pytest.mark.parametrize("task", ["stack", "handover"])
def test_teacher_retains_previous_capture_when_owner_history_wraps(task, monkeypatch):
    recipes = SCENARIOS / task / "configs/simulation/scene"
    plan = SimpleNamespace(plan_digest="plan")
    if task == "stack":
        plugin = stack_plugin.StackTaskPlugin(
            load_task_config(recipes / "marvin_stack_task.yaml"), task_mode="teacher"
        )
        monkeypatch.setattr(stack_plugin, "StackPlan", SimpleNamespace)
    else:
        plugin = handover_plugin.HandoverTaskPlugin(
            load_handover_task_config(recipes / "marvin_handover_task.yaml"), task_mode="teacher"
        )
    runtime = SimpleNamespace(state=_running_state(task), plan=plan, gate_fact=lambda: capture_seq)
    plugin._runtime = runtime
    observation = SimpleNamespace(
        right_positions=(0.0,) * 7,
        right_velocities=(0.0,) * 7,
        right_gripper_opening_m=0.08,
        right_gripper_velocity_m_s=0.0,
        right_finger_source_contacts=0,
        right_finger_source_min_force_n=0.0,
        right_finger_source_max_speed_m_s=0.0,
        source=SimpleNamespace(center=(0.0, 0.0, 1.0)),
        right_tcp_to_source_m=0.2,
    )
    capture_times = deque(maxlen=CAPTURE_HISTORY_LIMIT)
    latest = CAPTURE_HISTORY_LIMIT + 1
    for capture_seq in range(1, latest + 1):
        stamp = capture_seq * PERIOD_NS
        capture_times.append((capture_seq, stamp))
        plugin.on_capture(SimpleNamespace(capture_seq=capture_seq, sim_time_ns=stamp), observation)
    view = plugin.chunk_view(
        context=CONTEXT,
        task_epoch=1,
        capture_seq=latest - 1,
        observation_time_ns=(latest - 1) * PERIOD_NS,
        latest_capture_seq=latest,
        capture_times=capture_times,
        current_sim_time_ns=latest * PERIOD_NS,
    )
    assert view.feedback.capture_seq == latest - 1
    assert view.progress == latest - 1
    assert len(plugin._teacher_feedback) == CAPTURE_HISTORY_LIMIT


def test_handover_retains_each_teacher_request_through_runtime_chunk_limit(monkeypatch):
    recipes = SCENARIOS / "handover" / "configs"
    config = load_handover_task_config(recipes / "simulation/scene/marvin_handover_task.yaml")
    runtime = yaml.safe_load((recipes / "runtime/teacher_collect.yaml").read_text())
    limit = runtime["teacher"]["max_chunks"]
    plugin = handover_plugin.HandoverTaskPlugin(config, task_mode="teacher")
    view = TaskChunkView(
        plan=SimpleNamespace(plan_digest="plan"),
        task_epoch=1,
        task_start_sim_time_ns=0,
        current_sim_time_ns=PERIOD_NS,
        latest_capture_seq=1,
        state=_running_state("handover"),
        feedback=None,
        progress=SimpleNamespace(step_index=0, active_start_sim_time_ns=0),
    )
    monkeypatch.setattr(handover_plugin, "sample_handover_chunk", lambda *a, **kw: object())

    def sample(request, capture_seq=1):
        return plugin.sample_chunk(
            view,
            request_id=request,
            capture_seq=capture_seq,
            observation_time_ns=PERIOD_NS,
            profile=None,
            control_config=None,
        )

    first = sample("request-0")
    for index in range(1, limit):
        sample(f"request-{index}")
    assert sample("request-0") is first
    with pytest.raises(ContractError, match="same teacher request_id"):
        sample("request-0", capture_seq=2)
