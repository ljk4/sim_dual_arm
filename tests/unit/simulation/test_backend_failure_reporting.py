"""Preserve the first physical stop even if cleanup arrives during braking."""

import queue
from types import SimpleNamespace

import pytest

pytest.importorskip("mujoco")

from robot_backend_sim.common.runtime.worker_owner import _Owner  # noqa: E402


@pytest.mark.parametrize("operation", ["stop", "release"])
def test_physical_failure_reaches_task_before_cleanup_reason(operation):
    reasons = []
    owner = _Owner.__new__(_Owner)
    owner._events = queue.Queue()
    owner._feedback = queue.Queue()
    owner._events_emitted = 0
    owner._feedback_key = None
    owner._task_plugin = SimpleNamespace(fail_control=reasons.append)
    status = SimpleNamespace(
        command_id="command", phase="STOPPING",
        stop_reason="TRACKING_TIMEOUT", reason="TRACKING_TIMEOUT",
    )
    reply = SimpleNamespace(events=(), status=status)
    controller = SimpleNamespace(stop=lambda *args: reply, release=lambda *args: reply)
    payload = ("context", "lease", "RUNTIME_SHUTDOWN") if operation == "stop" else (
        "context", "lease"
    )
    assert owner._dispatch(controller, operation, payload) is reply
    assert reasons[0] == "TRACKING_TIMEOUT"
    assert reasons[1] in ("CONTROL_STOPPED:RUNTIME_SHUTDOWN", "CONTROL_RELEASED")


def test_first_physical_failure_is_forwarded_without_waiting_for_terminal():
    reasons = []
    owner = _Owner.__new__(_Owner)
    owner._events = queue.Queue()
    owner._feedback = queue.Queue()
    owner._events_emitted = 0
    owner._feedback_key = None
    owner._task_plugin = SimpleNamespace(fail_control=reasons.append)
    status = SimpleNamespace(
        command_id="command", phase="STOPPING",
        stop_reason="TRACKING_TIMEOUT", reason="TRACKING_TIMEOUT",
    )
    owner._record_reply(None, SimpleNamespace(events=(), status=status))
    assert reasons == ["TRACKING_TIMEOUT"]
