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
        QueryNativeCommand,
        ReleaseControl,
        ResetSimulation,
        Stop,
    )
    from robot_contracts.models import RunContext
    from robot_contracts.profiles import load_profile
    from robot_ros_support.conversions import context_from_message, duration
    from robot_ros_support.qos import SENSOR_QOS
except ImportError as error:  # pragma: no cover - host without ROS
    pytest.skip(f"ROS unavailable: {error}", allow_module_level=True)

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.backend_config import load_backend_config  # noqa: E402
from robot_backend_sim.node import NativeBackendNode  # noqa: E402
from robot_backend_sim.ros_bridge import RosFutureBridge  # noqa: E402
from robot_backend_sim.worker import NativeWorker  # noqa: E402

CONFIG = ROOT / "simulation/mujoco/configs/backend/backend.yaml"
PROFILE_PATH = ROOT / "profiles/robots/panda_stack_v1/profile.json"
SCHEMAS = ROOT / "contracts/schemas"


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
            "scene": str(ROOT / "simulation/mujoco/configs/scene/panda_stack.yaml"),
            "control": str(ROOT / "simulation/mujoco/configs/control/panda_control.yaml"),
            "stream": str(ROOT / "simulation/mujoco/configs/control/panda_stream.yaml"),
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
                str(ROOT / "simulation/mujoco/src"),
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
                ("get_native_anchor", GetNativeAnchor),
                ("query_native_command", QueryNativeCommand),
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
