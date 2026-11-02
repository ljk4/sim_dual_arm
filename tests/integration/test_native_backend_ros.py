"""End-to-end ROS evidence for the runnable native backend.

Run inside the supervisor's Docker/EGL image:

    python -m pytest tests/integration/test_native_backend_ros.py -q

The test starts the real node process, calls every public service over DDS and
checks that state/frames are one exact capture.  It is skipped on hosts without
ROS; it is not a substitute for the real EGL acceptance run.
"""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
import uuid
from concurrent.futures import Future
from pathlib import Path

import pytest
import yaml

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from robot_api_msgs.msg import (
        BackendInfo,
        CameraFrame,
        NativeControlEvent,
        NativeExecution,
        RobotState,
    )
    from robot_api_msgs.srv import (
        AcquireControl,
        GetNativeAnchor,
        QueryNativeChunk,
        QueryNativeCommand,
        ReleaseControl,
        ResetSimulation,
        Stop,
        SubmitNative,
    )
    from robot_contracts.models import RunContext, Target, Trajectory
    from robot_contracts.native import (
        NATIVE_MOTION,
        NATIVE_STREAM_FIRST,
        NATIVE_STREAM_NEXT,
        NativeRequest,
    )
    from robot_contracts.profiles import load_profile
    from robot_ros_support.conversions import context_from_message, duration, time_ns
    from robot_ros_support.native_conversions import (
        accepted_reference_from_message,
        anchor_from_message,
        native_command_message,
    )
    from robot_ros_support.qos import SENSOR_QOS
except ImportError as error:  # pragma: no cover - host without ROS
    pytest.skip(f"ROS unavailable: {error}", allow_module_level=True)

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "components/backend_sim/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.backend_config import load_backend_config  # noqa: E402
from robot_backend_sim.node import NativeBackendNode  # noqa: E402
from robot_backend_sim.ros_bridge import RosFutureBridge  # noqa: E402
from robot_backend_sim.worker import NativeWorker  # noqa: E402

CONFIG = ROOT / "configs/simulation/backend.yaml"
PROFILE_PATH = ROOT / "profiles/robots/panda_stack_v1/profile.json"
SCHEMAS = ROOT / "contracts/schemas"
ASSETS_ROOT = Path(os.environ.get("SIM_ASSETS_ROOT", ROOT / "third_party/src/mujoco_menagerie"))
SUMMARY = Path(os.environ.get("SIM_ASSET_SUMMARY", ROOT / ".build/assets/panda.json"))
SOURCE = ROOT / "third_party/panda-source.json"


def _config_for(
    tmp_path: Path,
    namespace: str,
    *,
    speed: float = 5.0,
    config_path: Path = CONFIG,
) -> Path:
    document = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    document["robot"].update(
        {
            "namespace": namespace,
            "profile": str(PROFILE_PATH),
            "schemas": str(SCHEMAS),
            "scene": str(ROOT / "configs/simulation/panda_stack.yaml"),
            "control": str(ROOT / "configs/simulation/panda_control.yaml"),
            "stream": str(ROOT / "configs/simulation/panda_stream.yaml"),
        }
    )
    follower = document["robot"].get("follower")
    if follower is not None and not Path(follower).is_absolute():
        document["robot"]["follower"] = str((config_path.parent / Path(follower)).resolve())
    document["run"]["realtime_speed"] = speed
    path = tmp_path / f"backend-{uuid.uuid4().hex[:8]}.yaml"
    path.write_text(yaml.safe_dump(document), encoding="utf-8")
    return path


class Stack:
    def __init__(
        self,
        tmp_path: Path,
        *,
        speed: float = 5.0,
        config_path: Path = CONFIG,
    ) -> None:
        self.namespace = f"/native_test_{uuid.uuid4().hex[:8]}"
        self.run_id = f"ros-{uuid.uuid4().hex[:8]}"
        self.run_root = tmp_path / "runs"
        self.config = _config_for(
            tmp_path, self.namespace, speed=speed, config_path=config_path
        )
        self.log = (tmp_path / f"{self.namespace[1:]}.log").open("wb")
        child_env = os.environ.copy()
        child_env["PYTHONPATH"] = os.pathsep.join(
            [
                str(ROOT / "components/backend_sim/src"),
                str(ROOT / "contracts/python/src"),
                str(ROOT / "libraries/ros_support/src"),
                str(ROOT / "libraries/run_logging/src"),
                child_env.get("PYTHONPATH", ""),
            ]
        )
        self.process = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "robot_backend_sim.node",
                "--config",
                str(self.config),
                "--assets-root",
                str(ASSETS_ROOT),
                "--asset-summary",
                str(SUMMARY),
                "--asset-source",
                str(SOURCE),
                "--run-root",
                str(self.run_root),
                "--run-id",
                self.run_id,
            ],
            cwd=str(ROOT),
            env=child_env,
            stdout=self.log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.node = Node(f"native_test_client_{uuid.uuid4().hex[:8]}")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.states: list[RobotState] = []
        self.frames: dict[str, list[CameraFrame]] = {}
        self.events: list[NativeControlEvent] = []
        self.feedback: list[NativeExecution] = []
        self.info: BackendInfo | None = None
        self.node.create_subscription(BackendInfo, f"{self.namespace}/info", self._on_info, 10)
        self.node.create_subscription(
            RobotState, f"{self.namespace}/state", self._on_state, SENSOR_QOS
        )
        for camera in load_profile(PROFILE_PATH, SCHEMAS).cameras:
            topic = f"{self.namespace}/cameras/{camera.name}/frame"
            self.frames[camera.name] = []
            self.node.create_subscription(
                CameraFrame,
                topic,
                lambda message, name=camera.name: self.frames[name].append(message),
                SENSOR_QOS,
            )
        self.node.create_subscription(
            NativeControlEvent, f"{self.namespace}/native_events", self.events.append, 10
        )
        self.node.create_subscription(
            NativeExecution,
            f"{self.namespace}/native_feedback",
            self.feedback.append,
            10,
        )
        self.clients = {
            name: self.node.create_client(service, f"{self.namespace}/{name}")
            for name, service in (
                ("acquire_control", AcquireControl),
                ("release_control", ReleaseControl),
                ("stop", Stop),
                ("submit_native", SubmitNative),
                ("get_native_anchor", GetNativeAnchor),
                ("query_native_command", QueryNativeCommand),
                ("query_native_chunk", QueryNativeChunk),
                ("reset_simulation", ResetSimulation),
            )
        }

    def _on_info(self, message: BackendInfo) -> None:
        self.info = message

    def _on_state(self, message: RobotState) -> None:
        self.states.append(message)

    def spin(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)

    def wait_ready(self, timeout: float = 20.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin(0.05)
            if self.info is not None and all(
                client.wait_for_service(timeout_sec=0.01) for client in self.clients.values()
            ):
                return
            if self.process.poll() is not None:
                raise AssertionError(f"node exited early: {self._output()}")
        raise AssertionError(f"stack did not become ready: {self._output()}")

    def call(self, name: str, request: object, timeout: float = 10.0) -> object:
        future = self.clients[name].call_async(request)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
            if future.done():
                return future.result()
        raise AssertionError(f"service {name} timed out")

    def wait_for(self, predicate, timeout: float = 10.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin(0.05)
            value = predicate()
            if value:
                return value
        raise AssertionError(f"condition not met: {self._output()}")

    def _output(self) -> str:
        try:
            self.log.flush()
        except ValueError:
            # close() closed the log before a final assertion reads status.json.
            pass
        try:
            return (self.run_root / self.run_id / "status.json").read_text(encoding="utf-8")
        except OSError:
            return "no status.json"

    def close(self, *, signal_number: int = signal.SIGTERM) -> int:
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal_number)
        try:
            code = self.process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid, signal.SIGKILL)
            code = self.process.wait(timeout=10)
        self.executor.shutdown()
        self.node.destroy_node()
        self.log.close()
        return code


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def acquire(stack: Stack, context: RunContext) -> str:
    request = AcquireControl.Request()
    request.context = _context_message(context)
    request.owner_id = "pytest"
    request.duration = duration(10_000_000_000)
    response = stack.call("acquire_control", request)
    assert response.success, response.reason
    return response.lease_id


def _context_message(context: RunContext):
    from robot_ros_support.conversions import context_message

    return context_message(context)


def _profile_context(profile, info: BackendInfo, *, episode: str, control_epoch: int) -> RunContext:
    base = context_from_message(info.context)
    return RunContext(
        base.robot_id,
        base.robot_profile_hash,
        base.backend_instance_id,
        base.time_domain_id,
        base.clock_epoch,
        episode_id=episode,
        control_epoch=control_epoch,
    )


def _motion(profile, context, lease: str, delta: float, command_id: str) -> NativeRequest:
    positions = list(profile.initial_positions)
    positions[0] += delta
    trajectory = Trajectory(
        context,
        lease,
        command_id,
        1,
        1,
        time.monotonic_ns() + 10_000_000_000,
        tuple(profile.joint_names),
        (Target(500_000_000, tuple(positions)),),
    )
    return NativeRequest(trajectory, NATIVE_MOTION)


def _submit(stack: Stack, request: NativeRequest):
    message = SubmitNative.Request()
    message.request = native_command_message(request)
    return stack.call("submit_native", message, timeout=20.0)


def _query(stack: Stack, context: RunContext, command_id: str):
    request = QueryNativeCommand.Request()
    request.context = _context_message(context)
    request.command_id = command_id
    return stack.call("query_native_command", request)


def test_motion_capture_query_retry_and_change(tmp_path):
    stack = Stack(tmp_path, speed=5.0)
    try:
        stack.wait_ready()
        info = stack.info
        assert info.protocol_major == 1
        assert info.backend_kind == "mujoco"
        assert info.test_only is False
        assert "native_motion" in info.capabilities
        assert "continuous_chunks" in info.capabilities
        assert "exact_capture" in info.capabilities
        assert "simulation_reset" in info.capabilities
        assert all("linear" not in capability for capability in info.capabilities)
        profile = load_profile(PROFILE_PATH, SCHEMAS)
        context = _profile_context(profile, info, episode="motion-episode", control_epoch=1)
        lease = acquire(stack, context)
        request = _motion(profile, context, lease, 0.03, "command-motion")
        first = _submit(stack, request)
        assert first.accepted, first.reason
        assert first.has_reference
        digest = accepted_reference_from_message(first.reference).reference_digest

        terminal = stack.wait_for(
            lambda: (
                response
                if (response := _query(stack, context, "command-motion")).success
                and response.execution.phase in ("COMPLETED", "CANCELED", "FAILED")
                else None
            ),
            timeout=20.0,
        )
        assert terminal.execution.phase == "COMPLETED"
        frozen = (
            terminal.execution.finished_sim_time_ns,
            terminal.execution.accepted_sim_time_ns,
            tuple(terminal.execution.measured_positions),
        )
        stack.spin(0.3)
        again = _query(stack, context, "command-motion")
        assert again.success
        assert (
            again.execution.finished_sim_time_ns,
            again.execution.accepted_sim_time_ns,
            tuple(again.execution.measured_positions),
        ) == frozen

        retry = _submit(stack, request)
        assert retry.accepted
        assert accepted_reference_from_message(retry.reference).reference_digest == digest
        assert accepted_reference_from_message(retry.reference).accepted_sim_time_ns == (
            accepted_reference_from_message(first.reference).accepted_sim_time_ns
        )

        changed = _motion(profile, context, lease, 0.04, "command-motion")
        rejected = _submit(stack, changed)
        assert not rejected.accepted
        assert rejected.reason == "COMMAND_ID_CONFLICT"

        # All four published stamps use the capture's simulation time, and the
        # three cameras share state_seq == frame_seq with one context.
        pairs = stack.wait_for(
            lambda: (
                [
                    state
                    for state in stack.states
                    if state.state_seq >= 1
                    and all(
                        any(frame.frame_seq == state.state_seq for frame in frames)
                        for frames in stack.frames.values()
                    )
                ]
                or None
            ),
            timeout=10.0,
        )
        state = pairs[-1]
        capture_time = time_ns(state.stamp)
        assert time_ns(state.joints.header.stamp) == capture_time
        for camera in profile.cameras:
            frame = next(
                item for item in stack.frames[camera.name] if item.frame_seq == state.state_seq
            )
            assert time_ns(frame.image.header.stamp) == capture_time
            assert time_ns(frame.camera_info.header.stamp) == capture_time
            assert frame.image.width == 320 and frame.image.height == 240
            assert frame.image.encoding == "rgb8"
            assert frame.calibration_digest == camera.calibration_digest
            assert frame.context.backend_instance_id == context.backend_instance_id
            assert frame.context.clock_epoch == context.clock_epoch
        assert stack.feedback and stack.events
        stack.call(
            "release_control",
            _release_request(context, lease),
        )
    finally:
        code = stack.close()
    assert code == 0
    status = (stack.run_root / stack.run_id / "status.json").read_text(encoding="utf-8")
    assert '"success": true' in status


def _release_request(context: RunContext, lease: str):
    from robot_api_msgs.srv import ReleaseControl

    request = ReleaseControl.Request()
    request.context = _context_message(context)
    request.lease_id = lease
    return request


def test_stream_chunks_and_reset_rejects_old_context(tmp_path):
    stack = Stack(tmp_path, speed=1.0)
    try:
        stack.wait_ready()
        profile = load_profile(PROFILE_PATH, SCHEMAS)
        context = _profile_context(profile, stack.info, episode="stream-episode", control_epoch=1)
        lease = acquire(stack, context)
        stack.wait_for(
            lambda: stack.states and stack.states[-1].context.episode_id == context.episode_id
        )
        latest = stack.states[-1]
        observation_ns = time_ns(latest.stamp)
        anchor_request = GetNativeAnchor.Request()
        anchor_request.context = _context_message(context)
        anchor_request.lease_id = lease
        anchor_request.has_start = True
        anchor_request.start_sim_time_ns = observation_ns + 500_000_000
        anchor_response = stack.call("get_native_anchor", anchor_request)
        assert anchor_response.success, anchor_response.reason
        anchor = anchor_from_message(anchor_response.anchor)
        hold = list(anchor.reference.positions)
        targets = []
        for index in range(1, 4):
            positions = list(hold)
            u = max(
                0.0,
                min(
                    1.0,
                    (observation_ns + index * 300_000_000 - anchor.start_sim_time_ns)
                    / 1_500_000_000,
                ),
            )
            positions[0] += 0.01 * (3.0 * u * u - 2.0 * u * u * u)
            targets.append(Target(index * 300_000_000, tuple(positions)))
        first_request = NativeRequest(
            Trajectory(
                context,
                lease,
                "command-stream",
                1,
                1,
                time.monotonic_ns() + 10_000_000_000,
                tuple(profile.joint_names),
                tuple(targets),
            ),
            NATIVE_STREAM_FIRST,
            anchor=anchor,
            observation_time_ns=observation_ns,
            target_count=2,
            settle_at_end=False,
        )
        first = _submit(stack, first_request)
        assert first.accepted, first.reason

        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            state = stack.call(
                "query_native_command",
                QueryNativeCommand.Request(
                    context=_context_message(context), command_id="command-stream"
                ),
            )
            if state.success and state.execution.has_started:
                break
            stack.spin(0.05)
        anchor_request.has_start = False
        next_anchor_response = stack.call("get_native_anchor", anchor_request)
        assert next_anchor_response.success, next_anchor_response.reason
        next_anchor = anchor_from_message(next_anchor_response.anchor)
        assert next_anchor.predecessor_seq == 1
        stack.wait_for(lambda: stack.states and stack.states[-1].state_seq >= 1)
        observation_ns = min(
            time_ns(stack.states[-1].stamp), next_anchor.start_sim_time_ns - 50_000_000
        )
        base = list(next_anchor.reference.positions)
        relative_start = next_anchor.start_sim_time_ns - observation_ns
        next_targets = []
        for index in range(1, 4):
            positions = list(base)
            positions[0] += 0.005 * index
            next_targets.append(Target(relative_start + index * 250_000_000, tuple(positions)))
        second_request = NativeRequest(
            Trajectory(
                context,
                lease,
                "command-stream",
                1,
                2,
                time.monotonic_ns() + 10_000_000_000,
                tuple(profile.joint_names),
                tuple(next_targets),
            ),
            NATIVE_STREAM_NEXT,
            anchor=next_anchor,
            observation_time_ns=observation_ns,
            target_count=2,
            settle_at_end=True,
        )
        second = _submit(stack, second_request)
        assert second.accepted, second.reason
        assert second.has_chunk
        chunk_query = QueryNativeChunk.Request()
        chunk_query.context = _context_message(context)
        chunk_query.command_id = "command-stream"
        chunk_query.chunk_seq = 2
        chunk = stack.call("query_native_chunk", chunk_query)
        assert chunk.success, chunk.reason
        assert chunk.chunk.chunk_seq == 2

        # Reset is only safe after a terminal motion; request a real stop first.
        stop = Stop.Request()
        stop.context = _context_message(context)
        stop.lease_id = lease
        stop.reason = "TEST_RESET"
        stack.call("stop", stop)
        stack.wait_for(
            lambda: (
                response
                if (response := _query(stack, context, "command-stream")).success
                and response.execution.phase in ("COMPLETED", "CANCELED", "FAILED")
                else None
            ),
            timeout=20.0,
        )
        reset_request = ResetSimulation.Request()
        reset_request.context = _context_message(context)
        reset_request.lease_id = lease
        reset_request.seed = 7
        reset = stack.call("reset_simulation", reset_request, timeout=20.0)
        assert reset.success, reset.reason
        new_context = context_from_message(reset.new_context)
        assert new_context.clock_epoch == context.clock_epoch + 1
        old = _submit(stack, _motion(profile, context, lease, 0.01, "old-context"))
        assert not old.accepted
        assert old.reason == "STALE_CONTEXT"
    finally:
        stack.close()


def test_node_sigterm_is_clean(tmp_path):
    stack = Stack(tmp_path, speed=5.0)
    try:
        stack.wait_ready()
    finally:
        code = stack.close(signal_number=signal.SIGTERM)
    assert code == 0
    status = (stack.run_root / stack.run_id / "status.json").read_text(encoding="utf-8")
    assert '"success": true' in status
    assert '"renderer_closed": true' in status


def test_bridge_timeout_and_completion_keep_executor_live(tmp_path):
    stack = Stack(tmp_path, speed=5.0)
    try:
        stack.wait_ready()
        group = stack.node.default_callback_group
        timed = Future()
        bridge = RosFutureBridge(
            stack.node,
            timed,
            timeout_s=0.2,
            timeout_reason="TEST_TIMEOUT",
            callback_group=group,
        )
        waiter = stack.executor.create_task(bridge.wait())
        before = len(stack.states)
        while not waiter.done():
            stack.spin(0.02)
        with pytest.raises(TimeoutError):
            waiter.result()
        assert timed.cancelled()
        assert len(stack.states) > before
        bridge.close()

        completed = Future()
        bridge = RosFutureBridge(
            stack.node,
            completed,
            timeout_s=5.0,
            timeout_reason="TEST_TIMEOUT",
            callback_group=group,
        )
        waiter = stack.executor.create_task(bridge.wait())
        completed.set_result(123)
        while not waiter.done():
            stack.spin(0.02)
        assert waiter.result() == 123
        bridge.close()
    finally:
        stack.close()


def test_node_setup_failure_closes_worker_and_renderer(tmp_path, monkeypatch):
    config = load_backend_config(CONFIG)
    closed: list[str] = []
    real_close = NativeWorker.close

    def recording_close(self, reason, timeout_s=None):
        closed.append(reason)
        return real_close(self, reason, timeout_s)

    monkeypatch.setattr(NativeWorker, "close", recording_close)

    real_publisher = NativeBackendNode.create_publisher

    def fail_publisher(self, *args, **kwargs):
        if hasattr(self, "_worker"):
            raise RuntimeError("injected publisher failure")
        return real_publisher(self, *args, **kwargs)

    monkeypatch.setattr(NativeBackendNode, "create_publisher", fail_publisher)
    with pytest.raises(RuntimeError, match="injected publisher failure"):
        NativeBackendNode(
            config,
            assets_root=ASSETS_ROOT,
            summary_path=SUMMARY,
            source_path=SOURCE,
        )
    assert closed == ["NODE_START_FAILED"]


def test_bridge_abort_resolves_pending_waiter(tmp_path):
    stack = Stack(tmp_path, speed=1.0)
    try:
        stack.wait_ready()
        source = Future()
        bridge = RosFutureBridge(
            stack.node,
            source,
            timeout_s=1.0,
            timeout_reason="TEST_TIMEOUT",
            callback_group=stack.node.default_callback_group,
        )
        bridge.abort()
        assert bridge.ros_future.done() and source.cancelled()
        with pytest.raises(RuntimeError, match="BACKEND_CLOSED"):
            bridge.ros_future.result()
    finally:
        stack.close()
