"""Acquire, execute and renewal tests; run inside the ROS Jazzy test image."""

import threading
import time
from types import SimpleNamespace

import pytest
from dual_arm_runtime import control_client as control_client_module
from dual_arm_runtime.configuration import RuntimeRecipe
from dual_arm_runtime.control_client import ControlClient
from robot_api_msgs.msg import TrajectoryCommand
from robot_contracts.models import ContractError, RunContext

CONTEXT = RunContext("robot", "profile", "backend", "mock", 0, "episode-control", 1)
RECIPE = RuntimeRecipe(
    schema_version=1,
    robot_profile="profiles/robots/mock_dual_arm_v1/profile.json",
    observation_schema="profiles/observations/mock_rgb_joint_v1.json",
    action_schema="profiles/actions/mock_joint_position_v1.json",
    robot_namespace="/cell_01/robot",
    lease_duration_ns=2_000_000_000,
    renew_interval_ns=500_000_000,
    command_acceptance_ns=500_000_000,
    startup_timeout_s=1.0,
    call_timeout_s=0.1,
    command_timeout_s=1.0,
    script_targets=(),
)


class _Future:
    def __init__(self, response=None, *, error=None, done_after=0):
        self._response = response
        self._error = error
        self._done_after = done_after
        self.done_calls = 0
        self.result_calls = 0

    def done(self):
        self.done_calls += 1
        return self.done_calls > self._done_after

    def result(self):
        self.result_calls += 1
        if self._error is not None:
            raise self._error
        return self._response


class _Client:
    def __init__(self, outcome, events, name):
        self._outcome = outcome
        self._events = events
        self._name = name
        self.requests = []

    def wait_for_service(self, timeout_sec):
        assert timeout_sec > 0
        self._events.append(f"{self._name}.wait")
        return True

    def call_async(self, request):
        self._events.append(f"{self._name}.call")
        self.requests.append(request)
        if isinstance(self._outcome, Exception):
            raise self._outcome
        return self._outcome


class _GoalHandle:
    def __init__(self, accepted, result_future):
        self.accepted = accepted
        self._result_future = result_future

    def get_result_async(self):
        return self._result_future


class _Action:
    def __init__(self, *, server_ready=True, accepted=True, goal_future=None, result_future=None):
        self.server_ready = server_ready
        self.accepted = accepted
        self.goal_future = goal_future
        self.result_future = result_future or _Future()
        self.waited = False
        self.goals = []

    def wait_for_server(self, timeout_sec):
        self.waited = timeout_sec > 0
        return self.server_ready

    def send_goal_async(self, goal):
        self.goals.append(goal)
        if self.goal_future is not None:
            return self.goal_future
        return _Future(_GoalHandle(self.accepted, self.result_future))


class _Node:
    """Only the ROS surface ControlClient is allowed to use."""

    def __init__(self):
        self.events = []

    def create_client(self, service_type, name, callback_group=None):
        del service_type, callback_group
        return _Client(_Future(_response()), self.events, name.rsplit("/", 1)[-1])

    def get_fully_qualified_name(self):
        return "runtime-test"


def _response(success=True, reason=""):
    return SimpleNamespace(success=success, reason=reason)


class _Sink:
    def __init__(self):
        self.events = []

    def emit(self, event, **kwargs):
        self.events.append((event, kwargs))
        return True

    def check_health(self):
        pass


def _make_control(monkeypatch, action=None, fault=None, log=None):
    action = action or _Action()
    monkeypatch.setattr(
        control_client_module,
        "ActionClient",
        lambda *args, **kwargs: action,
    )
    node = _Node()
    faults = []
    callback = fault if fault is not None else faults.append
    stop_requested = threading.Event()
    control = ControlClient(node, RECIPE, stop_requested, callback, log=log)
    return control, node, action, faults, stop_requested


def test_acquire_sets_lease_and_enables_renewal(monkeypatch):
    control, node, _action, _faults, _stop = _make_control(monkeypatch)
    response = SimpleNamespace(success=True, reason="", lease_id="lease-1")
    client = _Client(_Future(response), node.events, "acquire")
    control._service_clients["acquire_control"] = client

    control.acquire(CONTEXT)

    assert control.lease_id == "lease-1"
    assert control._lease_context == CONTEXT
    assert control._renew_enabled is True
    assert control._renew_future is None
    assert control._last_renew_ns > 0
    request = client.requests[0]
    assert request.owner_id == "runtime-test"
    assert request.duration.sec == 2
    assert request.duration.nanosec == 0


def test_wait_action_ready_reports_missing_server(monkeypatch):
    action = _Action(server_ready=False)
    control, _node, _action, _faults, _stop = _make_control(monkeypatch, action)

    with pytest.raises(ContractError, match="NOT_READY"):
        control.wait_action_ready()

    assert action.waited is True


def test_renew_sends_request_then_reports_rejection(monkeypatch):
    control, node, _action, faults, _stop = _make_control(monkeypatch)
    control._lease_context = CONTEXT
    control._lease_id = "lease-1"
    control._renew_enabled = True
    control._last_renew_ns = time.monotonic_ns() - RECIPE.renew_interval_ns
    client = _Client(
        _Future(SimpleNamespace(success=False, reason="LEASE_LOST")),
        node.events,
        "renew",
    )
    control._service_clients["renew_control"] = client

    control.renew()
    assert len(client.requests) == 1
    assert client.requests[0].lease_id == "lease-1"
    assert control._renew_future is not None

    control.renew()
    assert faults == ["LEASE_LOST"]
    assert control._renew_future is None


def test_renew_timeout_reports_fault(monkeypatch):
    control, _node, _action, faults, _stop = _make_control(monkeypatch)
    control._lease_context = CONTEXT
    control._lease_id = "lease-1"
    control._renew_enabled = True
    pending = _Future(done_after=10)
    control._renew_future = pending
    control._last_renew_ns = time.monotonic_ns() - RECIPE.lease_duration_ns - 1

    control.renew()

    assert faults == ["LEASE_RENEWAL_TIMEOUT"]
    assert control._renew_future is pending


def test_execute_rejects_unaccepted_goal(monkeypatch):
    action = _Action(accepted=False)
    control, _node, _action, _faults, _stop = _make_control(monkeypatch, action)

    with pytest.raises(ContractError, match="COMMAND_REJECTED"):
        control.execute(TrajectoryCommand())

    assert len(action.goals) == 1
    assert action.goals[0].command is not None


def test_execute_reports_failed_result(monkeypatch):
    result_response = SimpleNamespace(
        result=SimpleNamespace(success=False, reason="MOTION_FAILED", phase="FAILED")
    )
    action = _Action(result_future=_Future(result_response))
    control, _node, _action, _faults, _stop = _make_control(monkeypatch, action)

    with pytest.raises(ContractError, match="MOTION_FAILED"):
        control.execute(TrajectoryCommand())


def test_stop_request_interrupts_goal_wait(monkeypatch):
    monkeypatch.setattr(control_client_module.rclpy, "ok", lambda: True)
    action = _Action(goal_future=_Future(done_after=10))
    control, _node, _action, _faults, stop_requested = _make_control(monkeypatch, action)
    stop_requested.set()

    with pytest.raises(ContractError, match="EPISODE_STOPPED"):
        control.execute(TrajectoryCommand())


def test_controlled_goal_stop_logs_warning_not_action_failure(monkeypatch):
    monkeypatch.setattr(control_client_module.rclpy, "ok", lambda: True)
    action = _Action(goal_future=_Future(done_after=10))
    sink = _Sink()
    control, _node, _action, _faults, stop_requested = _make_control(
        monkeypatch,
        action,
        log=sink,
    )
    original_send = action.send_goal_async

    def stop_after_send(goal):
        stop_requested.set()
        return original_send(goal)

    action.send_goal_async = stop_after_send

    with pytest.raises(ContractError, match="EPISODE_STOPPED"):
        control.execute(TrajectoryCommand())

    names = [event for event, _ in sink.events]
    assert "command.action_stopped" in names
    assert "command.action_failed" not in names
    stopped = next(fields for event, fields in sink.events if event == "command.action_stopped")
    assert stopped["level"] == "WARNING"
    assert stopped["reason"] == "COMMAND_STOPPED"


def test_cleanup_callback_failure_does_not_skip_stop_or_release(monkeypatch):
    def failing_fault(reason):
        raise RuntimeError(f"fault callback failed: {reason}")

    control, node, _action, _faults, _stop = _make_control(
        monkeypatch,
        fault=failing_fault,
    )
    renewal = _Future(error=RuntimeError("renew boom"))
    control._lease_context = CONTEXT
    control._lease_id = "lease-1"
    control._renew_enabled = True
    control._renew_future = renewal
    events = []
    control._service_clients = {
        "stop": _Client(_Future(_response()), events, "stop"),
        "release_control": _Client(_Future(_response()), events, "release"),
    }

    with pytest.raises(RuntimeError, match="fault callback failed"):
        control.cleanup()

    assert events == ["stop.wait", "stop.call", "release.wait", "release.call"]
    assert control.lease_id == ""
    assert control._renew_future is None
    control.cleanup()
    assert events == ["stop.wait", "stop.call", "release.wait", "release.call"]
    assert node.events == []
