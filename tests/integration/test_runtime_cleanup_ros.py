"""Cleanup must distinguish unused command IDs from unresolved submissions."""

import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("robot_api_msgs.srv")

from dual_arm_runtime.runtime_node import RuntimeNode  # noqa: E402
from robot_contracts.models import ContractError  # noqa: E402


def _io(requests, completed=()):
    return SimpleNamespace(
        _condition=threading.Condition(),
        _expected_requests=requests,
        _control_facts_complete=lambda name: name in completed,
        _control=SimpleNamespace(query=Mock()),
        _recipe=SimpleNamespace(control=SimpleNamespace(poll_interval_s=0.001)),
    )


def test_cleanup_ignores_stream_id_when_prediction_failed_before_submission():
    io = _io({})

    RuntimeNode.wait_stop_settled(io, ["never-submitted"], 0.02)

    io._control.query.assert_not_called()


def test_cleanup_handles_unused_id_alongside_completed_submission():
    io = _io({"submitted": object()}, completed={"submitted"})

    RuntimeNode.wait_stop_settled(io, ["never-submitted", "submitted"], 0.02)

    io._control.query.assert_not_called()


def test_cleanup_keeps_submitted_command_with_unknown_outcome():
    request = SimpleNamespace(trajectory=SimpleNamespace(context="context"))
    io = _io({"submitted": request})
    io._control.query.side_effect = ContractError("CALL_TIMEOUT", "outcome still unknown")

    with pytest.raises(ContractError, match="terminal state missing after stop: submitted"):
        RuntimeNode.wait_stop_settled(io, ["never-submitted", "submitted"], 0.02)

    io._control.query.assert_called_with(
        "context", "submitted", request=request, ignore_stop=True
    )
