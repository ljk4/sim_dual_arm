"""Verify the Runtime-generated follower plan at the ROS submission boundary."""

from __future__ import annotations

import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from robot_contracts.models import ContractError
from robot_contracts.trajectory.spline import CubicSegment, ReferencePlan

pytest.importorskip("rclpy")
pytest.importorskip("robot_api_msgs")

from dual_arm_runtime import robot_control_client as control_module  # noqa: E402
from dual_arm_runtime.robot_control_client import FirstFollowerRejected  # noqa: E402
from robot_api_msgs.srv import SubmitFollower  # noqa: E402
from robot_ros_support.follower_conversions import (  # noqa: E402
    follower_reference_message,
    reference_segments_from_messages,
)
from test_follower_reference import (  # noqa: E402
    constraints,
    profile,
    reference,
    request,
    smoothstep_plan,
)


@pytest.fixture
def control():
    raw = request()
    recipe = SimpleNamespace(
        robot_namespace="/robot",
        policy=None,
        control=SimpleNamespace(submit_query_retries=1, poll_interval_s=0.0),
    )
    client = control_module.RobotControlClient(
        SimpleNamespace(create_client=lambda *_args, **_kwargs: object()),
        recipe,
        profile(),
        constraints().max_accelerations,
        threading.Event(),
        lambda _reason: None,
        follower_limits=constraints(),
    )
    client._lease_context = raw.trajectory.context
    client._lease_id = raw.trajectory.lease_id
    return client


def test_transport_retry_reuses_generated_plan_and_packet(control, monkeypatch):
    raw = request()
    plan = smoothstep_plan()
    generate = Mock(return_value=plan)
    monkeypatch.setattr(control_module, "generate_follower_plan", generate)
    submissions = []

    def call(service_name, message, **_kwargs):
        assert service_name == "submit_follower"
        submissions.append(message)
        if len(submissions) == 1:
            raise ContractError("CALL_TIMEOUT", "lost response")
        return SubmitFollower.Response(accepted=True)

    query = Mock()
    accepted = Mock()
    monkeypatch.setattr(control, "_call", call)
    monkeypatch.setattr(control, "query", query)
    monkeypatch.setattr(control, "_follower_accepted", accepted)

    assert control.submit_follower(raw) is accepted.return_value
    assert generate.call_count == 1
    query.assert_called_once_with(
        raw.trajectory.context, raw.trajectory.command_id, request=raw, ignore_stop=True,
    )
    assert len(submissions) == 2 and submissions[0] is submissions[1]
    assert reference_segments_from_messages(
        submissions[0].segments, raw.trajectory.joint_names,
    ) == plan
    assert submissions[0].request.command_id == raw.trajectory.command_id
    assert accepted.call_args.args[2] == plan
    assert accepted.call_args.kwargs == {"queried": True, "attempts": 2}


def test_accepted_receipt_must_echo_the_transmitted_plan(control):
    accepted = reference()
    segment = accepted.plan.segments[0]
    different_plan = ReferencePlan(
        accepted.plan.joint_names,
        (CubicSegment(segment.duration_ns, segment.c0, segment.c1, segment.c2, (0.0, 0.0)),),
    )
    receipt = SubmitFollower.Response(
        accepted=True, has_reference=True, reference=follower_reference_message(accepted),
    )
    with pytest.raises(ContractError, match="accepted follower plan differs"):
        control._follower_accepted(request(), receipt, different_plan, queried=False, attempts=1)


def test_local_first_plan_failure_does_not_create_a_backend_command(control, monkeypatch):
    generate = Mock(side_effect=ContractError("FOLLOWER_INFEASIBLE", "local planning failure"))
    call = Mock()
    monkeypatch.setattr(control_module, "generate_follower_plan", generate)
    monkeypatch.setattr(control, "_call", call)
    with pytest.raises(FirstFollowerRejected, match="FOLLOWER_INFEASIBLE: local planning failure"):
        control.submit_follower(request())
    call.assert_not_called()
