"""Real ROS Future regressions for immediate service-ACK wakeup.

The two runtime entry points wait on the same rclpy Future completion but keep
their own stop, shutdown, timeout, cancellation and rejection rules.  These
tests use real rclpy Future/service objects; a fake rclpy module is never
substituted.
"""

from __future__ import annotations

import threading
import time
import uuid
from dataclasses import dataclass, replace
from pathlib import Path

import pytest

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.task import Future
    from robot_api_msgs.srv import (
        GetNativeAnchor,
        GetStackTaskState,
        PrepareStackTask,
        SubmitNative,
    )
    from robot_contracts.models import ContractError, RunContext
except ImportError as error:  # pragma: no cover - host without ROS
    pytest.skip(f"ROS unavailable: {error}", allow_module_level=True)

from dual_arm_runtime.native_config import load_native_configuration
from dual_arm_runtime.native_control_client import NativeControlClient
from dual_arm_runtime.stack_task_client import StackTaskClient

ROOT = Path(__file__).resolve().parents[2]
NATIVE_CONFIG = ROOT / "configs/runtime/native_teacher.yaml"
POLL_INTERVAL_S = 0.5
ASYNC_ACK_DELAY_S = 0.02


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    started = not rclpy.ok()
    if started:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    yield
    if started and rclpy.ok():
        rclpy.shutdown()


class _RosGraph:
    def __init__(self) -> None:
        self.executor = SingleThreadedExecutor()
        self._nodes: list[Node] = []

    def node(self, prefix: str) -> Node:
        node = Node(f"dsh038_{prefix}_{uuid.uuid4().hex[:8]}")
        self._nodes.append(node)
        self.executor.add_node(node)
        return node

    def close(self) -> None:
        self.executor.shutdown()
        for node in reversed(self._nodes):
            node.destroy_node()
        self._nodes.clear()


@pytest.fixture
def graph():
    value = _RosGraph()
    try:
        yield value
    finally:
        value.close()


@dataclass(slots=True)
class _Outcome:
    value: object = None
    error: BaseException | None = None
    elapsed_s: float = 0.0


class _PendingCall:
    """A real pending/completed Future exposed as a service client."""

    def __init__(self, future: Future) -> None:
        self.future = future

    def wait_for_service(self, *, timeout_sec: float) -> bool:
        return True

    def call_async(self, request: object) -> Future:
        return self.future


class _PrecompletedCall:
    """Make a real service answer, then hand the already-done Future back."""

    def __init__(self, client: object, executor: SingleThreadedExecutor) -> None:
        self._client = client
        self._executor = executor

    def wait_for_service(self, *, timeout_sec: float) -> bool:
        return self._client.wait_for_service(timeout_sec=timeout_sec)  # type: ignore[attr-defined]

    def call_async(self, request: object) -> Future:
        future = self._client.call_async(request)  # type: ignore[attr-defined]
        _spin_until(self._executor, future.done)
        return future


def _root(prefix: str = "ack") -> str:
    return f"/dsh038_{prefix}_{uuid.uuid4().hex[:8]}"


def _spin_until(executor: SingleThreadedExecutor, predicate, timeout: float = 3.0) -> None:
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError("ROS condition was not met")
        executor.spin_once(timeout_sec=0.01)


def _wait_service(executor: SingleThreadedExecutor, client: object, timeout: float = 10.0) -> None:
    deadline = time.monotonic() + timeout
    while not client.service_is_ready():  # type: ignore[attr-defined]
        if time.monotonic() >= deadline:
            raise AssertionError("ROS service was not discovered")
        executor.spin_once(timeout_sec=0.01)


def _run_while_spinning(
    executor: SingleThreadedExecutor, operation, timeout: float = 10.0
) -> _Outcome:
    outcome = _Outcome()

    def run() -> None:
        started = time.monotonic()
        try:
            outcome.value = operation()
        except BaseException as error:  # noqa: BLE001 - asserted by the caller
            outcome.error = error
        finally:
            outcome.elapsed_s = time.monotonic() - started

    thread = threading.Thread(target=run, name="dsh038-ack-wait", daemon=True)
    thread.start()
    deadline = time.monotonic() + timeout
    while thread.is_alive():
        if time.monotonic() >= deadline:
            pytest.fail("blocking ACK wait did not return")
        executor.spin_once(timeout_sec=0.01)
    thread.join()
    return outcome


def _native_client(graph: _RosGraph, root: str, *, poll_interval_s: float) -> NativeControlClient:
    configuration = load_native_configuration(NATIVE_CONFIG, ROOT)
    recipe = replace(
        configuration.recipe,
        robot_namespace=root,
        call_timeout_s=2.0,
        control=replace(configuration.recipe.control, poll_interval_s=poll_interval_s),
    )
    return NativeControlClient(
        graph.node("native_client"),
        recipe,
        configuration.profile,
        (),
        threading.Event(),
        lambda reason: None,
    )


def _stack_client(
    graph: _RosGraph,
    root: str,
    *,
    poll_interval_s: float,
    prepare_timeout_s: float = 2.0,
    call_timeout_s: float = 2.0,
    log: object | None = None,
) -> StackTaskClient:
    return StackTaskClient(
        graph.node("stack_client"),
        root,
        call_timeout_s=call_timeout_s,
        prepare_timeout_s=prepare_timeout_s,
        start_timeout_s=prepare_timeout_s,
        chunk_timeout_s=prepare_timeout_s,
        poll_interval_s=poll_interval_s,
        log=log,  # type: ignore[arg-type]
    )


def _submit_ack(delay_s: float):
    def handler(_request, response):
        if delay_s:
            time.sleep(delay_s)
        response.accepted = True
        response.reason = ""
        return response

    return handler


def _prepare_ack(delay_s: float):
    def handler(_request, response):
        if delay_s:
            time.sleep(delay_s)
        response.success = True
        response.reason = ""
        return response

    return handler


def _prepare_success(response):
    response.success = True
    response.reason = ""
    response.task_epoch = 7
    response.task_config_digest = "config-digest"
    response.plan_digest = "plan-digest"
    response.key_pose_count = 3
    response.collision_segment_count = 2
    response.ik_max_position_error_m = 0.001
    response.ik_max_orientation_error_rad = 0.002
    response.planned_duration_s = 1.5
    response.prepare_kind = "teacher"
    return response


def _context(episode_id: str) -> RunContext:
    return RunContext(
        robot_id="panda",
        robot_profile_hash="profile-hash",
        backend_instance_id="backend-1",
        time_domain_id="sim",
        clock_epoch=1,
        episode_id=episode_id,
    )


class _RecordingLog:
    """Capture emitted events without providing any ROS behavior."""

    def __init__(self) -> None:
        self.events: list[tuple[str, dict[str, object]]] = []

    def emit(self, event: str, **kwargs: object) -> bool:
        self.events.append((event, kwargs))
        return True

    def check_health(self) -> None:
        return None


class _ShutdownOnWaitLog(_RecordingLog):
    """Mark shutdown once the client reports entering the quiescence wait."""

    def __init__(self) -> None:
        super().__init__()
        self.shutdown = False

    def emit(self, event: str, **kwargs: object) -> bool:
        accepted = super().emit(event, **kwargs)
        if event == "native.task.prepare_waiting":
            self.shutdown = True
        return accepted


def test_native_wait_future_returns_for_completed_ack(graph):
    root = _root("native_done")
    server = graph.node("native_server")
    server.create_service(SubmitNative, f"{root}/submit_native", _submit_ack(0.0))
    client = _native_client(graph, root, poll_interval_s=POLL_INTERVAL_S)
    raw_client = client._clients["submit_native"]
    _wait_service(graph.executor, raw_client)

    future = raw_client.call_async(SubmitNative.Request())
    _spin_until(graph.executor, future.done)
    started = time.monotonic()
    response = client._wait_future(future, 1.0)
    elapsed = time.monotonic() - started

    assert response.accepted
    assert elapsed < 0.4


def test_native_call_wakes_on_async_ack(graph):
    root = _root("native_async")
    server = graph.node("native_server")
    server.create_service(SubmitNative, f"{root}/submit_native", _submit_ack(ASYNC_ACK_DELAY_S))
    client = _native_client(graph, root, poll_interval_s=POLL_INTERVAL_S)
    _wait_service(graph.executor, client._clients["submit_native"])

    outcome = _run_while_spinning(
        graph.executor,
        lambda: client._call("submit_native", SubmitNative.Request(), require_success=False),
    )

    assert outcome.error is None
    assert outcome.value.accepted
    assert outcome.elapsed_s < 0.4


def test_stack_call_returns_for_completed_ack(graph):
    root = _root("stack_done")
    server = graph.node("stack_server")
    server.create_service(PrepareStackTask, f"{root}/prepare_stack_task", _prepare_ack(0.0))
    client = _stack_client(graph, root, poll_interval_s=POLL_INTERVAL_S)
    _wait_service(graph.executor, client._clients["prepare"])
    client._clients["prepare"] = _PrecompletedCall(client._clients["prepare"], graph.executor)

    started = time.monotonic()
    response = client._call("prepare", PrepareStackTask.Request())
    elapsed = time.monotonic() - started

    assert response.success
    assert elapsed < 0.4


def test_stack_call_wakes_on_async_ack(graph):
    root = _root("stack_async")
    server = graph.node("stack_server")
    server.create_service(
        PrepareStackTask, f"{root}/prepare_stack_task", _prepare_ack(ASYNC_ACK_DELAY_S)
    )
    client = _stack_client(graph, root, poll_interval_s=POLL_INTERVAL_S)
    _wait_service(graph.executor, client._clients["prepare"])

    outcome = _run_while_spinning(
        graph.executor, lambda: client._call("prepare", PrepareStackTask.Request())
    )

    assert outcome.error is None
    assert outcome.value.success
    assert outcome.elapsed_s < 0.4


def test_native_wait_future_honors_deadline(graph):
    client = _native_client(graph, _root("native_timeout"), poll_interval_s=POLL_INTERVAL_S)
    future = Future()

    started = time.monotonic()
    with pytest.raises(ContractError) as raised:
        client._wait_future(future, 0.1)
    elapsed = time.monotonic() - started

    assert raised.value.reason == "CALL_TIMEOUT"
    assert elapsed < 0.3


def test_native_wait_future_honors_stop(graph):
    client = _native_client(graph, _root("native_stop"), poll_interval_s=0.02)
    client.stop_requested.set()
    future = Future()

    with pytest.raises(ContractError) as raised:
        client._wait_future(future, 2.0)

    assert raised.value.reason == "EPISODE_STOPPED"


def test_native_cleanup_wait_ignores_stop(graph):
    client = _native_client(graph, _root("native_cleanup"), poll_interval_s=0.02)
    client.stop_requested.set()
    future = Future()

    with pytest.raises(ContractError) as raised:
        client._wait_future(future, 0.1, abort_on_stop=False)

    assert raised.value.reason == "CALL_TIMEOUT"


def test_native_wait_future_honors_shutdown(graph, monkeypatch):
    client = _native_client(graph, _root("native_shutdown"), poll_interval_s=0.02)
    future = Future()

    with monkeypatch.context() as patch:
        patch.setattr(rclpy, "ok", lambda: False)
        with pytest.raises(ContractError) as raised:
            client._wait_future(future, 2.0)

    assert raised.value.reason == "EPISODE_STOPPED"


def test_native_wait_future_propagates_ack_exception(graph):
    client = _native_client(graph, _root("native_error"), poll_interval_s=0.02)
    future = Future()
    future.set_exception(RuntimeError("native ack failed"))

    with pytest.raises(RuntimeError, match="native ack failed"):
        client._wait_future(future, 1.0)


def test_native_call_keeps_rejected_ack_reason(graph):
    root = _root("native_reject")
    server = graph.node("native_server")

    def reject(_request, response):
        response.success = False
        response.reason = "NATIVE_ACK_REJECTED"
        return response

    server.create_service(GetNativeAnchor, f"{root}/get_native_anchor", reject)
    client = _native_client(graph, root, poll_interval_s=POLL_INTERVAL_S)
    _wait_service(graph.executor, client._clients["get_native_anchor"])

    outcome = _run_while_spinning(
        graph.executor, lambda: client._call("get_native_anchor", GetNativeAnchor.Request())
    )

    assert isinstance(outcome.error, ContractError)
    assert outcome.error.reason == "NATIVE_ACK_REJECTED"


def test_stack_call_honors_deadline_and_cancels(graph):
    client = _stack_client(
        graph, _root("stack_timeout"), poll_interval_s=POLL_INTERVAL_S, prepare_timeout_s=0.1
    )
    future = Future()
    client._clients["prepare"] = _PendingCall(future)

    started = time.monotonic()
    with pytest.raises(ContractError) as raised:
        client._call("prepare", PrepareStackTask.Request())
    elapsed = time.monotonic() - started

    assert raised.value.reason == "CALL_TIMEOUT"
    assert "stack task prepare" in str(raised.value)
    assert future.cancelled()
    assert elapsed < 0.3


def test_stack_call_honors_stop_and_keeps_state_health(graph):
    root = _root("stack_stop")
    client = _stack_client(
        graph, root, poll_interval_s=0.02, prepare_timeout_s=2.0, call_timeout_s=0.1
    )
    prepare_future = Future()
    client._clients["prepare"] = _PendingCall(prepare_future)
    client._stop.set()

    with pytest.raises(ContractError) as stopped:
        client._call("prepare", PrepareStackTask.Request())
    assert stopped.value.reason == "EPISODE_STOPPED"
    assert "stack task prepare stopped" in str(stopped.value)

    state_future = Future()
    client._clients["state"] = _PendingCall(state_future)
    with pytest.raises(ContractError) as timed_out:
        client._call("state", GetStackTaskState.Request())
    assert timed_out.value.reason == "CALL_TIMEOUT"


def test_stack_call_honors_shutdown(graph, monkeypatch):
    client = _stack_client(
        graph, _root("stack_shutdown"), poll_interval_s=0.02, prepare_timeout_s=2.0
    )
    client._clients["prepare"] = _PendingCall(Future())

    with monkeypatch.context() as patch:
        patch.setattr(rclpy, "ok", lambda: False)
        with pytest.raises(ContractError) as raised:
            client._call("prepare", PrepareStackTask.Request())

    assert raised.value.reason == "EPISODE_STOPPED"
    assert "during shutdown" in str(raised.value)


def test_stack_call_wraps_future_exception(graph):
    client = _stack_client(graph, _root("stack_error"), poll_interval_s=0.02, prepare_timeout_s=2.0)
    future = Future()
    future.set_exception(RuntimeError("stack ack failed"))
    client._clients["prepare"] = _PendingCall(future)

    with pytest.raises(ContractError) as raised:
        client._call("prepare", PrepareStackTask.Request())

    assert raised.value.reason == "TASK_RPC_FAILED"
    assert isinstance(raised.value.__cause__, RuntimeError)


def test_stack_call_keeps_rejected_ack_reason(graph):
    root = _root("stack_reject")
    server = graph.node("stack_server")

    def reject(_request, response):
        response.success = False
        response.reason = "STACK_ACK_REJECTED"
        return response

    server.create_service(PrepareStackTask, f"{root}/prepare_stack_task", reject)
    client = _stack_client(graph, root, poll_interval_s=POLL_INTERVAL_S)
    _wait_service(graph.executor, client._clients["prepare"])

    outcome = _run_while_spinning(
        graph.executor, lambda: client._call("prepare", PrepareStackTask.Request())
    )

    assert isinstance(outcome.error, ContractError)
    assert outcome.error.reason == "STACK_ACK_REJECTED"


def test_stack_prepare_retries_not_quiescent_with_same_request(graph):
    root = _root("prepare_retry")
    server = graph.node("stack_server")
    requests: list[tuple[object, ...]] = []

    def handler(request, response):
        requests.append(
            (
                request.context.robot_id,
                request.context.robot_profile_hash,
                request.context.backend_instance_id,
                request.context.time_domain_id,
                int(request.context.clock_epoch),
                request.context.episode_id,
                request.episode_id,
                request.request_id,
                request.lease_id,
                int(request.seed),
            )
        )
        if len(requests) == 1:
            response.success = False
            response.reason = "TASK_NOT_QUIESCENT"
            return response
        return _prepare_success(response)

    server.create_service(PrepareStackTask, f"{root}/prepare_stack_task", handler)
    log = _RecordingLog()
    client = _stack_client(
        graph, root, poll_interval_s=0.05, prepare_timeout_s=2.0, log=log
    )
    _wait_service(graph.executor, client._clients["prepare"])
    context = _context("retry-episode")

    outcome = _run_while_spinning(
        graph.executor,
        lambda: client.prepare(
            context, "retry-episode", "retry-episode-prepare", "lease-retry", 30400
        ),
    )

    assert outcome.error is None
    assert outcome.value.task_epoch == 7
    assert outcome.value.plan_digest == "plan-digest"
    assert client.prepare_fact is outcome.value
    assert len(requests) == 2
    assert requests[0] == requests[1]
    assert requests[0] == (
        "panda",
        "profile-hash",
        "backend-1",
        "sim",
        1,
        "retry-episode",
        "retry-episode",
        "retry-episode-prepare",
        "lease-retry",
        30400,
    )
    assert [event for event, _ in log.events] == [
        "native.task.prepare_waiting",
        "native.task.prepare_succeeded",
    ]
    assert log.events[0][1]["fields"]["attempt"] == 1
    assert log.events[1][1]["fields"]["attempts"] == 2


def test_stack_prepare_not_quiescent_respects_total_budget(graph):
    root = _root("prepare_timeout")
    server = graph.node("stack_server")
    calls: list[float] = []

    def handler(_request, response):
        calls.append(time.monotonic())
        time.sleep(0.08)
        response.success = False
        response.reason = "TASK_NOT_QUIESCENT"
        return response

    server.create_service(PrepareStackTask, f"{root}/prepare_stack_task", handler)
    log = _RecordingLog()
    client = _stack_client(
        graph, root, poll_interval_s=0.05, prepare_timeout_s=0.3, log=log
    )
    _wait_service(graph.executor, client._clients["prepare"])

    outcome = _run_while_spinning(
        graph.executor,
        lambda: client.prepare(
            _context("budget-episode"),
            "budget-episode",
            "budget-episode-prepare",
            "lease-budget",
            30400,
        ),
        timeout=5.0,
    )

    assert isinstance(outcome.error, ContractError)
    assert outcome.error.reason == "PREPARE_TIMEOUT"
    assert "TASK_NOT_QUIESCENT" in str(outcome.error)
    assert isinstance(outcome.error.__cause__, ContractError)
    assert outcome.error.__cause__.reason == "TASK_NOT_QUIESCENT"
    assert len(calls) >= 2
    assert 0.25 <= outcome.elapsed_s < 1.0
    events = [event for event, _ in log.events]
    assert events.count("native.task.prepare_waiting") == 1
    assert events.count("native.task.prepare_timeout") == 1


def test_stack_prepare_other_rejection_is_not_retried(graph):
    root = _root("prepare_reject")
    server = graph.node("stack_server")
    calls: list[float] = []

    def handler(_request, response):
        calls.append(time.monotonic())
        response.success = False
        response.reason = "LEASE_EXPIRED"
        return response

    server.create_service(PrepareStackTask, f"{root}/prepare_stack_task", handler)
    client = _stack_client(graph, root, poll_interval_s=0.05, prepare_timeout_s=2.0)
    _wait_service(graph.executor, client._clients["prepare"])

    outcome = _run_while_spinning(
        graph.executor,
        lambda: client.prepare(
            _context("reject-episode"),
            "reject-episode",
            "reject-episode-prepare",
            "lease-reject",
            30400,
        ),
    )

    assert isinstance(outcome.error, ContractError)
    assert outcome.error.reason == "LEASE_EXPIRED"
    assert len(calls) == 1
    assert outcome.elapsed_s < 0.5
    assert client.prepare_fact is None


def test_stack_prepare_retry_honors_stop(graph):
    root = _root("prepare_stop")
    server = graph.node("stack_server")
    calls: list[float] = []
    client = _stack_client(graph, root, poll_interval_s=0.05, prepare_timeout_s=2.0)

    def handler(_request, response):
        calls.append(time.monotonic())
        response.success = False
        response.reason = "TASK_NOT_QUIESCENT"
        client._stop.set()
        return response

    server.create_service(PrepareStackTask, f"{root}/prepare_stack_task", handler)
    _wait_service(graph.executor, client._clients["prepare"])

    outcome = _run_while_spinning(
        graph.executor,
        lambda: client.prepare(
            _context("stop-episode"),
            "stop-episode",
            "stop-episode-prepare",
            "lease-stop",
            30400,
        ),
    )

    assert isinstance(outcome.error, ContractError)
    assert outcome.error.reason == "EPISODE_STOPPED"
    assert "stopped" in str(outcome.error)
    assert len(calls) == 1
    assert client.prepare_fact is None


def test_stack_prepare_retry_honors_shutdown(graph, monkeypatch):
    root = _root("prepare_shutdown")
    server = graph.node("stack_server")
    calls: list[float] = []
    log = _ShutdownOnWaitLog()
    client = _stack_client(
        graph, root, poll_interval_s=0.05, prepare_timeout_s=2.0, log=log
    )

    def handler(_request, response):
        calls.append(time.monotonic())
        response.success = False
        response.reason = "TASK_NOT_QUIESCENT"
        return response

    server.create_service(PrepareStackTask, f"{root}/prepare_stack_task", handler)
    _wait_service(graph.executor, client._clients["prepare"])

    with monkeypatch.context() as patch:
        patch.setattr(rclpy, "ok", lambda: not log.shutdown)
        outcome = _run_while_spinning(
            graph.executor,
            lambda: client.prepare(
                _context("shutdown-episode"),
                "shutdown-episode",
                "shutdown-episode-prepare",
                "lease-shutdown",
                30400,
            ),
        )

    assert isinstance(outcome.error, ContractError)
    assert outcome.error.reason == "EPISODE_STOPPED"
    assert "during shutdown" in str(outcome.error)
    assert len(calls) == 1
    assert client.prepare_fact is None
