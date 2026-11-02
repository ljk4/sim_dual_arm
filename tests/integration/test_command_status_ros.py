"""Real ROS/MCAP evidence for query, feedback ordering and audit rejection."""

from __future__ import annotations

import time
from dataclasses import replace
from pathlib import Path

import pytest

try:
    import rclpy
    from builtin_interfaces.msg import Duration
    from rclpy.action import ActionClient
    from robot_api_msgs.action import ExecuteTrajectory
    from robot_api_msgs.msg import BackendInfo, CommandStatus, RobotState
    from robot_api_msgs.srv import (
        AcquireControl,
        QueryCommand,
        ReleaseControl,
    )
    from robot_contracts.models import RunContext, Target, Trajectory
    from robot_contracts.status import interpolate_reference
    from robot_ros_support.conversions import (
        command_message,
        command_status_from_message,
        context_from_message,
        context_message,
        duration,
        stamp,
    )
    from robot_ros_support.qos import EVENT_QOS, INFO_QOS, SENSOR_QOS
except ImportError as error:  # pragma: no cover - host without a ROS installation
    pytest.skip(f"ROS unavailable: {error}", allow_module_level=True)

from dual_arm_runtime.audit import inspect_episode
from dual_arm_runtime.configuration import load_configuration
from robot_contracts.models import ContractError
from runtime_cli_harness import RuntimeCliHarness, client_domain, run_cli, wait_ready
from test_recording import ROOT, SCHEMAS, write_episode


def _spin(executor, seconds: float) -> None:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)


def _wait(executor, future, timeout: float = 8.0):
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() >= deadline:
            raise TimeoutError("ROS future timed out")
        executor.spin_once(timeout_sec=0.02)
    return future.result()


def _call(executor, client, request, timeout: float = 8.0):
    assert client.wait_for_service(timeout_sec=timeout)
    return _wait(executor, client.call_async(request), timeout)


def _status_message(
    context: RunContext,
    joint_names: tuple[str, ...],
    *,
    command_id: str = "command-test",
    command_seq: int = 1,
    observation_id: int = 1,
    phase: str = "COMPLETED",
    accepted: int = 100_000_000,
    started: int = 200_000_000,
    sample: int = 300_000_000,
    elapsed: int | None = None,
    initial: tuple[float, ...] | None = None,
    reference: tuple[float, ...] | None = None,
    measured: tuple[float, ...] | None = None,
    tracking: tuple[float, ...] | None = None,
    progress: float = 1.0,
) -> CommandStatus:
    if elapsed is None:
        elapsed = sample - started
    if initial is None:
        initial = (0.0,) * len(joint_names)
    if reference is None:
        reference = (0.2, -0.2, 0.1, -0.1, 0.02, 0.02)
    if measured is None:
        measured = reference
    if tracking is None:
        tracking = tuple(left - right for left, right in zip(reference, measured, strict=True))
    return CommandStatus(
        context=context_message(context),
        command_id=command_id,
        command_seq=command_seq,
        observation_id=observation_id,
        phase=phase,
        reason="",
        sample_time=stamp(sample),
        accepted_at=stamp(accepted),
        started_at=stamp(started),
        elapsed=Duration(sec=0, nanosec=elapsed),
        execution_clock="steady",
        interpolation_version="linear_v1",
        joint_names=list(joint_names),
        initial_positions=list(initial),
        reference_positions=list(reference),
        measured_positions=list(measured),
        tracking_error=list(tracking),
        progress=progress,
    )


def test_public_query_and_feedback_report_actual_phase(tmp_path):
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions

    harness = RuntimeCliHarness(tmp_path)
    harness.start_backend()
    executor = None
    node = None
    try:
        with client_domain(harness):
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            node = Node("dsh019_query_probe")
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            received = {"info": None, "state": None}
            feedback = []
            root = "/cell_01/robot"
            node.create_subscription(
                BackendInfo,
                f"{root}/info",
                lambda message: received.update(info=message),
                INFO_QOS,
            )
            node.create_subscription(
                RobotState,
                f"{root}/state",
                lambda message: received.update(state=message),
                SENSOR_QOS,
            )
            node.create_subscription(
                CommandStatus,
                f"{root}/command_feedback",
                lambda message: feedback.append(message),
                EVENT_QOS,
            )
            query_client = node.create_client(QueryCommand, f"{root}/query_command")
            acquire_client = node.create_client(AcquireControl, f"{root}/acquire_control")
            release_client = node.create_client(ReleaseControl, f"{root}/release_control")
            action = ActionClient(node, ExecuteTrajectory, f"{root}/execute_trajectory")
            deadline = time.monotonic() + 10
            while received["info"] is None or received["state"] is None:
                if time.monotonic() >= deadline:
                    raise TimeoutError("mock backend did not publish info/state")
                executor.spin_once(timeout_sec=0.05)
            info = received["info"]
            assert info.test_only and info.backend_kind == "mock"
            assert {"command_query", "command_feedback_v1"}.issubset(set(info.capabilities))
            context = replace(
                context_from_message(info.context),
                episode_id=f"probe-{time.time_ns()}",
                control_epoch=1,
            )
            acquired = _call(
                executor,
                acquire_client,
                AcquireControl.Request(
                    context=context_message(context),
                    owner_id="dsh019-query-probe",
                    duration=duration(5_000_000_000),
                ),
            )
            assert acquired.success
            lease = acquired.lease_id
            command_id = f"query-{time.time_ns()}"
            initial = tuple(received["state"].joints.position)
            target = (initial[0] + 0.1, *initial[1:])
            command = Trajectory(
                context,
                lease,
                command_id,
                1,
                1,
                node.get_clock().now().nanoseconds + 3_000_000_000,
                tuple(info.joint_names),
                (Target(1_500_000_000, target),),
            )
            assert action.wait_for_server(timeout_sec=5)
            handle = _wait(
                executor,
                action.send_goal_async(ExecuteTrajectory.Goal(command=command_message(command))),
            )
            assert handle.accepted

            active_status = None
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline and active_status is None:
                executor.spin_once(timeout_sec=0.02)
                mine = [message for message in feedback if message.command_id == command_id]
                if mine:
                    response = _call(
                        executor,
                        query_client,
                        QueryCommand.Request(
                            context=context_message(context),
                            command_id=command_id,
                        ),
                    )
                    assert response.success, response.reason
                    active_status = command_status_from_message(response.status)
            assert active_status is not None
            assert active_status.phase in {"STARTED", "COMPLETED"}
            assert active_status.context == context
            assert active_status.accepted_at_ns == active_status.started_at_ns
            assert active_status.sample_time_ns >= active_status.started_at_ns
            assert active_status.execution_clock == "steady"
            assert active_status.interpolation_version == "linear_v1"
            fraction = min(1.0, active_status.elapsed_ns / command.targets[-1].time_ns)
            assert active_status.progress == pytest.approx(fraction)
            assert active_status.reference_positions == pytest.approx(
                tuple(
                    left + fraction * (right - left)
                    for left, right in zip(initial, target, strict=True)
                )
            )
            assert tuple(active_status.joint_names) == tuple(info.joint_names)
            assert active_status.reference_positions == active_status.measured_positions
            assert all(value == 0.0 for value in active_status.tracking_error)
            assert 0.0 <= active_status.progress <= 1.0

            result = _wait(executor, handle.get_result_async(), 10).result
            assert result.success and result.phase == "COMPLETED"
            terminal_response = _call(
                executor,
                query_client,
                QueryCommand.Request(
                    context=context_message(context),
                    command_id=command_id,
                ),
            )
            assert terminal_response.success
            terminal = command_status_from_message(terminal_response.status)
            assert terminal.phase == "COMPLETED"
            assert terminal.progress == 1.0
            assert terminal.reference_positions == target

            stale = _call(
                executor,
                query_client,
                QueryCommand.Request(
                    context=context_message(replace(context, episode_id="other")),
                    command_id=command_id,
                ),
            )
            assert not stale.success and stale.reason == "STALE_CONTEXT"
            assert stale.status.command_id == "" and stale.status.phase == ""
            missing = _call(
                executor,
                query_client,
                QueryCommand.Request(
                    context=context_message(context),
                    command_id="missing-command",
                ),
            )
            assert not missing.success and missing.reason == "RESULT_EXPIRED"
            assert missing.status.command_id == ""

            _spin(executor, 0.3)
            terminals = [
                message
                for message in feedback
                if message.command_id == command_id and message.phase == "COMPLETED"
            ]
            assert len(terminals) == 1
            assert terminals[0].context.episode_id == context.episode_id
            assert all(len(message.tracking_error) == len(info.joint_names) for message in feedback)
            released = _call(
                executor,
                release_client,
                ReleaseControl.Request(
                    context=context_message(context),
                    lease_id=lease,
                ),
            )
            assert released.success
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        harness.stop()


def test_runtime_recording_contains_feedback_and_audit_accepts_it(tmp_path):
    harness = RuntimeCliHarness(tmp_path)
    harness.start_backend()
    harness.start_runtime()
    try:
        wait_ready(harness)
        with client_domain(harness):
            result = run_cli(harness, "start", 60, task_id="mock-script", wait=True)
        assert result["success"] is True, result
        directory = harness.output / result["episode_id"]
        report = inspect_episode(directory, SCHEMAS)
        assert report["recording"]["complete"] is True
        assert report["runtime"]["outcome"] == "completed"
        assert report["feedback_commands"] == report["commands"] > 0
        assert any(topic.endswith("/command_feedback") for topic in report["message_counts"])
    finally:
        harness.stop()


def _recording_with_feedback(tmp_path: Path, feedback: tuple[CommandStatus, ...]) -> Path:
    return write_episode(tmp_path, feedback=feedback)


def _recorded_positions(configuration):
    initial = tuple(configuration.profile.initial_positions)
    target = list(initial)
    target[0] = 0.2
    return initial, tuple(target)


@pytest.fixture
def feedback_context():
    configuration = load_configuration(ROOT, ROOT / "configs/runtime/mock_collect.yaml")
    context = RunContext(
        configuration.profile.robot_id,
        configuration.profile.digest,
        "backend-test",
        "mock-system",
        0,
        "episode-test",
        1,
    )
    return configuration, context


def test_mcap_audit_accepts_matching_feedback(tmp_path, feedback_context):
    configuration, context = feedback_context
    initial, target = _recorded_positions(configuration)
    message = _status_message(
        context,
        configuration.profile.joint_names,
        initial=initial,
        reference=target,
        elapsed=800_000_000,
    )
    directory = _recording_with_feedback(tmp_path, (message,))
    report = inspect_episode(directory, SCHEMAS)
    assert report["recording"]["complete"] is True
    assert report["feedback_commands"] == 1


@pytest.mark.parametrize(
    "case", ["nan", "tracking", "identity", "wrong_reference", "intermediate_reference", "progress"]
)
def test_mcap_audit_rejects_bad_feedback(tmp_path, feedback_context, case):
    configuration, context = feedback_context
    names = configuration.profile.joint_names
    initial, target = _recorded_positions(configuration)
    kwargs = {"initial": initial, "reference": target, "elapsed": 800_000_000}
    if case == "nan":
        kwargs["measured"] = (float("nan"),) + (0.0,) * (len(names) - 1)
        kwargs["tracking"] = (0.0,) * len(names)
    elif case == "tracking":
        kwargs["tracking"] = (0.5,) + (0.0,) * (len(names) - 1)
    elif case == "identity":
        kwargs["command_id"] = "other-command"
    elif case == "wrong_reference":
        kwargs["reference"] = (0.0,) * len(names)
        kwargs["measured"] = (0.0,) * len(names)
        kwargs["tracking"] = (0.0,) * len(names)
    elif case == "intermediate_reference":
        shifted = (target[0] + 0.005, *target[1:])
        kwargs["reference"] = shifted
        kwargs["measured"] = shifted
    elif case == "progress":
        kwargs["progress"] = 0.9
    message = _status_message(context, names, **kwargs)
    directory = _recording_with_feedback(tmp_path, (message,))
    assert directory.name.startswith(".partial-")
    with pytest.raises(ContractError, match="RECORDING_ERROR"):
        inspect_episode(directory, SCHEMAS)


def test_mcap_audit_rejects_feedback_time_and_terminal_regression(tmp_path, feedback_context):
    configuration, context = feedback_context
    names = configuration.profile.joint_names
    initial, target = _recorded_positions(configuration)
    targets = (Target(800_000_000, target),)
    later = _status_message(
        context,
        names,
        phase="STARTED",
        sample=300_000_000,
        elapsed=100_000_000,
        progress=0.125,
        initial=initial,
        reference=interpolate_reference(initial, targets, 100_000_000, "linear_v1"),
    )
    earlier = _status_message(
        context,
        names,
        phase="CANCELED",
        sample=350_000_000,
        elapsed=50_000_000,
        progress=0.0625,
        initial=initial,
        reference=interpolate_reference(initial, targets, 50_000_000, "linear_v1"),
    )
    directory = _recording_with_feedback(tmp_path / "time", (later, earlier))
    assert directory.name.startswith(".partial-")
    with pytest.raises(ContractError, match="RECORDING_ERROR"):
        inspect_episode(directory, SCHEMAS)

    completed = _status_message(
        context,
        names,
        phase="COMPLETED",
        sample=300_000_000,
        elapsed=800_000_000,
        initial=initial,
        reference=target,
    )
    regressed = _status_message(
        context,
        names,
        phase="STARTED",
        sample=350_000_000,
        elapsed=800_000_000,
        progress=1.0,
        initial=initial,
        reference=target,
    )
    directory = _recording_with_feedback(tmp_path / "terminal", (completed, regressed))
    assert directory.name.startswith(".partial-")
    with pytest.raises(ContractError, match="RECORDING_ERROR"):
        inspect_episode(directory, SCHEMAS)


def test_mcap_audit_rejects_missing_terminal_feedback(tmp_path, feedback_context):
    configuration, context = feedback_context
    names = configuration.profile.joint_names
    initial, target = _recorded_positions(configuration)
    targets = (Target(800_000_000, target),)
    active = _status_message(
        context,
        names,
        phase="STARTED",
        sample=250_000_000,
        elapsed=50_000_000,
        progress=0.0625,
        initial=initial,
        reference=interpolate_reference(initial, targets, 50_000_000, "linear_v1"),
    )
    directory = _recording_with_feedback(tmp_path, (active,))
    assert directory.name.startswith(".partial-")
    with pytest.raises(ContractError, match="RECORDING_ERROR"):
        inspect_episode(directory, SCHEMAS)
