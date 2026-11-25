"""Actual backend queues must cover the selected task's declared capacity."""

import os
from pathlib import Path

import pytest
from robot_backend_sim.common.config.backend_config import load_backend_config
from robot_backend_sim.common.config.stream_config import load_stream_config
from robot_backend_sim.common.runtime.worker import Worker
from robot_backend_sim.marvin.handover.plugin import HandoverTaskPlugin
from robot_backend_sim.marvin.handover.task_config import load_handover_task_config


@pytest.mark.parametrize("role", ["teacher", "policy"])
def test_marvin_backend_covers_real_task_capacity(role):
    shared = Path(os.environ.get("SHARED_ROOT", Path(__file__).resolve().parents[3] / "shared"))
    recipe = shared / (
        f"scenarios/marvin/mujoco/handover/configs/simulation/backend/backend_marvin_{role}.yaml"
    )
    backend = load_backend_config(recipe)
    task = load_handover_task_config(backend.task_path)
    plugin = HandoverTaskPlugin(task, task_mode=backend.task_mode)
    Worker._check_task_capacity(backend, plugin)
    stream = load_stream_config(backend.stream_path)
    minimum_wall_s = task.success.max_task_sim_duration_s / backend.realtime_speed
    assert stream.stream_wall_timeout_s >= minimum_wall_s
    assert task.success.max_task_wall_duration_s >= minimum_wall_s
