"""Verify preparation modes across the generated ROS response and runtime client."""

from __future__ import annotations

import asyncio
import json
from concurrent.futures import Future
from types import SimpleNamespace

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("robot_api_msgs.srv")

from dual_arm_runtime.task_client import TaskClient  # noqa: E402
from rclpy.serialization import deserialize_message, serialize_message  # noqa: E402
from sim_api_msgs.srv import PrepareTask


@pytest.mark.parametrize("prepare_kind", ["teacher", "evaluation_only"])
def test_generated_prepare_response_preserves_mode_on_wire(prepare_kind):
    response = PrepareTask.Response(
        success=True,
        task_epoch=3,
        task_type="stack",
        task_config_digest="config",
        plan_digest="plan",
        prepare_kind=prepare_kind,
        metrics_json='{"planned_duration_s":20.0}',
    )

    received = deserialize_message(serialize_message(response), PrepareTask.Response)
    fact = TaskClient._prepare_fact(received)

    assert fact.prepare_kind == prepare_kind
    assert fact.task_epoch == 3
    assert fact.planned_duration_s == 20.0


@pytest.mark.parametrize("prepare_kind", ["teacher", "evaluation_only"])
def test_backend_adapter_returns_preparation_mode(prepare_kind):
    pytest.importorskip("mujoco")
    from robot_backend_sim.ros.node_services import ServiceAdapter
    from robot_backend_sim.tasks.base import TaskPrepareResult
    from robot_contracts.models import RunContext
    from robot_contracts.task_state import TaskState
    from robot_ros_support.conversions import context_message

    context = RunContext("robot", "profile", "backend", "time", 1, "episode", 1)
    result = TaskPrepareResult(
        task_type="stack",
        task_epoch=3,
        task_config_digest="config",
        plan_digest="plan",
        idempotent=False,
        state=TaskState(
            context=context,
            state_seq=1,
            sample_time_ns=0,
            task_epoch=3,
            task_type="stack",
            phase="PREPARED",
        ),
        prepare_kind=prepare_kind,
        metrics={"planned_duration_s": 20.0},
    )
    future = Future()
    future.set_result(result)

    async def await_result(pending):
        return pending.result()

    def fail_response(response, error):
        raise AssertionError("prepare service failed") from error

    node = SimpleNamespace(
        _await=await_result,
        _emit=lambda *args, **kwargs: None,
        _failure_response=fail_response,
    )
    worker = SimpleNamespace(prepare_task=lambda *args: future)
    adapter = ServiceAdapter(node, worker, None)
    request = PrepareTask.Request(
        context=context_message(context),
        episode_id="episode",
        request_id="prepare",
        lease_id="lease",
        seed=10000,
        task_type="stack",
    )

    response = asyncio.run(adapter.prepare_task(request, PrepareTask.Response()))
    received = deserialize_message(serialize_message(response), PrepareTask.Response)
    fact = TaskClient._prepare_fact(received)

    assert received.success
    assert received.reason == ""
    assert received.prepare_kind == prepare_kind
    assert json.loads(received.metrics_json) == result.metrics
    assert fact.prepare_kind == prepare_kind
