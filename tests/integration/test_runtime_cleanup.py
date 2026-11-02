"""Control cleanup tests; run inside the ROS Jazzy test image."""

import threading
from types import SimpleNamespace

import pytest
from dual_arm_runtime import control_client as control_client_module
from dual_arm_runtime.configuration import RuntimeRecipe
from dual_arm_runtime.control_client import ControlClient
from dual_arm_runtime.episode import FailureLog
from robot_contracts.models import RunContext

CONTEXT = RunContext("robot", "profile", "backend", "mock", 0, "episode-cleanup", 1)
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


class _Node:
    """Only the ROS surface ControlClient is allowed to use."""

    def create_client(self, service_type, name, callback_group=None):
        del service_type, callback_group
        return _Client(_Future(_response()), [], name.rsplit("/", 1)[-1])

    def get_fully_qualified_name(self):
        return "runtime-test"


class _ActionClient:
    def __init__(self, *args, **kwargs):
        del args, kwargs

    def wait_for_server(self, timeout_sec):
        return timeout_sec > 0


class _FaultSink:
    """Node-side receiver using the production episode failure aggregation."""

    def __init__(self):
        self._stop_requested = threading.Event()
        self._faults = FailureLog(self._stop_requested, lambda: None)
        self.logged_errors = []

    @property
    def failure_reason(self):
        return self._faults.reason

    @failure_reason.setter
    def failure_reason(self, reason):
        self._faults.reset(reason)

    def get_logger(self):
        return self

    def error(self, message):
        self.logged_errors.append(message)

    def report(self, reason):
        self.error(reason)
        self._faults.append(reason)


def _response(success=True, reason=""):
    return SimpleNamespace(success=success, reason=reason)


def _make_control(monkeypatch):
    monkeypatch.setattr(control_client_module, "ActionClient", _ActionClient)
    sink = _FaultSink()
    control = ControlClient(_Node(), RECIPE, sink._stop_requested, sink.report)
    return control, sink


def _arm(control, events, *, renewal=None, stop=None, release=None):
    control._lease_context = CONTEXT
    control._lease_id = "lease-1"
    control._renew_enabled = True
    control._renew_future = renewal
    control._service_clients = {
        "stop": _Client(stop if stop is not None else _Future(_response()), events, "stop"),
        "release_control": _Client(
            release if release is not None else _Future(_response()),
            events,
            "release",
        ),
    }


def test_cleanup_waits_for_renewal_then_stops_and_releases(monkeypatch):
    control, sink = _make_control(monkeypatch)
    events = []
    renewal = _Future(_response())
    _arm(control, events, renewal=renewal)

    control.cleanup()

    assert renewal.result_calls == 1
    assert events == ["stop.wait", "stop.call", "release.wait", "release.call"]
    assert control._service_clients["stop"].requests[0].lease_id == "lease-1"
    assert control._service_clients["release_control"].requests[0].lease_id == "lease-1"
    assert control.lease_id == ""
    assert control._renew_future is None
    assert control._renew_enabled is False
    assert sink.failure_reason == ""
    assert sink.logged_errors == []


def test_cleanup_continues_after_renewal_wait_error(monkeypatch):
    control, sink = _make_control(monkeypatch)
    events = []
    renewal = _Future(error=RuntimeError("renew boom"))
    _arm(control, events, renewal=renewal)

    control.cleanup()

    assert events == ["stop.wait", "stop.call", "release.wait", "release.call"]
    assert "CONTROL_CLEANUP_FAILED" in sink.failure_reason
    assert "renew: renew boom" in sink.failure_reason
    assert sink._stop_requested.is_set()
    assert control.lease_id == ""


def test_cleanup_treats_rejected_renewal_as_failure(monkeypatch):
    control, sink = _make_control(monkeypatch)
    events = []
    renewal = _Future(_response(success=False, reason="LEASE_LOST"))
    _arm(control, events, renewal=renewal)

    control.cleanup()

    assert events == ["stop.wait", "stop.call", "release.wait", "release.call"]
    assert "renew: LEASE_LOST" in sink.failure_reason
    assert sink.logged_errors[-1].startswith("CONTROL_CLEANUP_FAILED:")


def test_cleanup_still_releases_after_stop_failure(monkeypatch):
    control, sink = _make_control(monkeypatch)
    events = []
    _arm(control, events, stop=_Future(_response(success=False, reason="STOP_REJECTED")))

    control.cleanup()

    assert events == ["stop.wait", "stop.call", "release.wait", "release.call"]
    assert "stop: STOP_REJECTED" in sink.failure_reason
    assert "release:" not in sink.failure_reason
    assert control.lease_id == ""


def test_cleanup_reports_release_failure(monkeypatch):
    control, sink = _make_control(monkeypatch)
    events = []
    _arm(
        control,
        events,
        release=_Future(_response(success=False, reason="RELEASE_REJECTED")),
    )

    control.cleanup()

    assert len(control._service_clients["stop"].requests) == 1
    assert "release: RELEASE_REJECTED" in sink.failure_reason
    assert "stop:" not in sink.failure_reason


def test_cleanup_preserves_primary_failure_and_aggregates_all_steps(monkeypatch):
    control, sink = _make_control(monkeypatch)
    events = []
    sink.failure_reason = "PRIMARY_FAILURE"
    _arm(
        control,
        events,
        renewal=_Future(error=RuntimeError("renew boom")),
        stop=_Future(_response(success=False, reason="STOP_REJECTED")),
        release=RuntimeError("release boom"),
    )

    control.cleanup()

    reason = sink.failure_reason
    assert reason.startswith("PRIMARY_FAILURE; CONTROL_CLEANUP_FAILED:")
    for detail in ("renew: renew boom", "stop: STOP_REJECTED", "release: release boom"):
        assert detail in reason
    assert sink.logged_errors[-1] == reason.removeprefix("PRIMARY_FAILURE; ")
    assert sink._stop_requested.is_set()
    assert control.lease_id == ""
    assert control._renew_future is None


@pytest.mark.parametrize(
    ("has_context", "has_lease"),
    [(False, False), (True, False), (False, True)],
)
def test_cleanup_without_control_is_local_only(monkeypatch, has_context, has_lease):
    control, sink = _make_control(monkeypatch)
    events = []
    pending = _Future(_response())
    control._lease_context = CONTEXT if has_context else None
    control._lease_id = "lease-1" if has_lease else ""
    control._renew_enabled = True
    control._renew_future = pending
    control._service_clients = {
        "stop": _Client(_Future(_response()), events, "stop"),
        "release_control": _Client(_Future(_response()), events, "release"),
    }

    control.cleanup()

    assert events == []
    assert pending.result_calls == 0
    assert control._renew_enabled is False
    assert control._renew_future is None
    assert control.lease_id == ""
    assert sink.failure_reason == ""


def test_second_cleanup_makes_no_service_calls_and_does_not_duplicate_errors(monkeypatch):
    control, sink = _make_control(monkeypatch)
    events = []
    _arm(control, events, stop=_Future(_response(success=False, reason="STOP_REJECTED")))
    control.cleanup()
    reason, logged = sink.failure_reason, list(sink.logged_errors)
    events.clear()

    control.cleanup()

    assert events == []
    assert sink.failure_reason == reason
    assert sink.logged_errors == logged
    assert control.lease_id == ""
    assert control._renew_future is None


def test_cleanup_ignores_stop_flag_while_waiting(monkeypatch):
    monkeypatch.setattr(control_client_module.rclpy, "ok", lambda: True)
    control, sink = _make_control(monkeypatch)
    events = []
    renewal = _Future(_response(), done_after=1)
    _arm(
        control,
        events,
        renewal=renewal,
        stop=_Future(_response(), done_after=1),
        release=_Future(_response(), done_after=1),
    )
    sink._stop_requested.set()

    control.cleanup()

    assert renewal.done_calls >= 2
    assert events == ["stop.wait", "stop.call", "release.wait", "release.call"]
    assert sink.failure_reason == ""
