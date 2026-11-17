"""P02A Robot API evidence for the Marvin handover backend.

Starts the real backend with the direct-prepared Marvin recipe and drives only
the public robot_api_msgs services over DDS.  Every motion is a finite
SubmitFollower window; the recorded evidence contains the raw request, the
accepted reference, measured RobotState samples, execution feedback and the
terminal status.  It is skipped on hosts without ROS.
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path

import pytest
import yaml

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from robot_api_msgs.msg import BackendInfo, ExecutionFeedback, RobotState
    from robot_api_msgs.srv import (
        AcquireControl,
        GetStreamAnchor,
        QueryStreamCommand,
        ReleaseControl,
        RenewControl,
        Stop,
        SubmitFollower,
    )
    from robot_contracts.models import RunContext
    from robot_contracts.stream_request import REQUEST_STREAM_FIRST, StreamRequest, Trajectory
    from robot_contracts.trajectory.follower_reference import (
        FollowerReference,
    )
    from robot_ros_support.conversions import context_from_message, context_message, duration
    from robot_ros_support.follower_conversions import follower_reference_from_message
    from robot_ros_support.qos import SENSOR_QOS
    from robot_ros_support.stream_conversions import (
        anchor_from_message,
        execution_feedback_from_message,
        stream_command_message,
    )
except ImportError as error:  # pragma: no cover - host without ROS
    pytest.skip(f"ROS unavailable: {error}", allow_module_level=True)

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.config.scenario_config import ROBOT_REFERENCE_FIELDS  # noqa: E402
from robot_contracts.models import Target  # noqa: E402
from robot_contracts.profiles import load_profile  # noqa: E402

SCENARIO = ROOT / "shared/scenarios/marvin_sim/handover"
RECIPE = Path(
    os.environ.get(
        "MARVIN_BACKEND_CONFIG",
        str(SCENARIO / "configs/simulation/backend/backend_marvin_control.yaml"),
    )
)
EVIDENCE_DIR = Path(os.environ.get("P02A_EVIDENCE_DIR", "/data/marvin_sim/handover/diagnostics/p02a"))
PROFILE_PATH = SCENARIO / "profiles/robots/marvin_handover_v1/profile.json"
SCHEMAS = ROOT / "shared/common/interfaces/schemas"
PROFILE = load_profile(PROFILE_PATH, SCHEMAS)


def _direct_document(path: Path) -> dict:
    """Load the direct recipe and expand robot paths like ``backend --config``."""
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    robot = document["robot"]
    for field in ROBOT_REFERENCE_FIELDS:
        value = robot.get(field)
        if isinstance(value, str):
            robot[field] = str((path.parent / value).resolve())
    return document

TARGET_COUNT = 20
POINT_NS = 50_000_000
LEASE_NS = 8_000_000_000
LEASE_RENEW_NS = 3_000_000_000
TERMINAL = frozenset({"COMPLETED", "STOPPED", "FAILED", "CANCELED"})



def _stamp_ns(stamp: object) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class Stack:
    def __init__(self, tmp_path: Path) -> None:
        self.namespace = f"/p02a_{uuid.uuid4().hex[:8]}"
        self.run_id = f"p02a-{uuid.uuid4().hex[:8]}"
        self.run_root = EVIDENCE_DIR / "runs"
        self.run_root.mkdir(parents=True, exist_ok=True)
        self.evidence_dir = EVIDENCE_DIR / self.run_id
        self.evidence_dir.mkdir(parents=True, exist_ok=True)
        self.config = self._config(tmp_path)
        self.log = (tmp_path / f"{self.namespace[1:]}.log").open("wb")
        child_env = os.environ.copy()
        child_env["PYTHONPATH"] = os.pathsep.join(
            [
                str(SIM_SRC),
                str(ROOT / "shared/common/python/contracts/src"),
                str(ROOT / "shared/common/python/ros_support/src"),
                str(ROOT / "shared/common/python/run_logging/src"),
                str(ROOT / "shared/common/python/trajectory/src"),
                child_env.get("PYTHONPATH", ""),
            ]
        )
        self.process = subprocess.Popen(
            [
                sys.executable,
                "-m",
                "robot_backend_sim.ros.node",
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
        self.node = Node(f"p02a_client_{uuid.uuid4().hex[:8]}")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.states: list[RobotState] = []
        self.feedback: list[ExecutionFeedback] = []
        self.info: BackendInfo | None = None
        self.node.create_subscription(BackendInfo, f"{self.namespace}/info", self._on_info, 10)
        self.node.create_subscription(
            RobotState, f"{self.namespace}/state", self.states.append, SENSOR_QOS
        )
        self.node.create_subscription(
            ExecutionFeedback,
            f"{self.namespace}/execution_feedback",
            self.feedback.append,
            10,
        )
        self.clients = {
            name: self.node.create_client(service, f"{self.namespace}/{name}")
            for name, service in (
                ("acquire_control", AcquireControl),
                ("renew_control", RenewControl),
                ("release_control", ReleaseControl),
                ("stop", Stop),
                ("submit_follower", SubmitFollower),
                ("get_stream_anchor", GetStreamAnchor),
                ("query_stream_command", QueryStreamCommand),
            )
        }

    def _config(self, tmp_path: Path) -> Path:
        document = _direct_document(RECIPE)
        document["robot"]["namespace"] = self.namespace
        document["run"]["realtime_speed"] = 1.0
        path = tmp_path / "backend.yaml"
        path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
        return path

    def _on_info(self, message: BackendInfo) -> None:
        self.info = message

    def spin(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)

    def wait_ready(self, timeout: float = 30.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin(0.05)
            if self.info is not None and all(
                client.wait_for_service(timeout_sec=0.01) for client in self.clients.values()
            ):
                return
            if self.process.poll() is not None:
                raise AssertionError(f"backend exited early: {self.output()}")
        raise AssertionError(f"backend did not become ready: {self.output()}")

    def call(self, name: str, request: object, timeout: float = 15.0) -> object:
        future = self.clients[name].call_async(request)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)
            if future.done():
                return future.result()
        raise AssertionError(f"service {name} timed out")

    def wait_for(self, predicate, timeout: float = 30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin(0.02)
            value = predicate()
            if value:
                return value
            if self.process.poll() is not None:
                raise AssertionError(f"backend exited: {self.output()}")
        raise AssertionError(f"condition not met: {self.output()}")

    def output(self) -> str:
        try:
            self.log.flush()
        except ValueError:
            pass
        try:
            return (self.run_root / self.run_id / "status.json").read_text(encoding="utf-8")
        except OSError:
            return "no status.json"

    def close(self) -> int:
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGTERM)
        try:
            code = self.process.wait(timeout=30)
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid, signal.SIGKILL)
            code = self.process.wait(timeout=10)
        self.executor.shutdown()
        self.node.destroy_node()
        self.log.close()
        return code

    def latest_state(self) -> RobotState | None:
        return self.states[-1] if self.states else None

    def latest_sim_ns(self) -> int:
        state = self.latest_state()
        assert state is not None
        return _stamp_ns(state.stamp)


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _context(info: BackendInfo, episode: str) -> RunContext:
    base = context_from_message(info.context)
    return RunContext(
        base.robot_id,
        base.robot_profile_hash,
        base.backend_instance_id,
        base.time_domain_id,
        base.clock_epoch,
        episode_id=episode,
        control_epoch=1,
    )


def _smoothstep(t: float) -> float:
    t = min(1.0, max(0.0, t))
    return t * t * (3.0 - 2.0 * t)


def _source_targets(
    hold: tuple[float, ...], target: tuple[float, ...], lead_ns: int
) -> tuple[Target, ...]:
    first = int(lead_ns // POINT_NS)
    total = first + TARGET_COUNT
    values = []
    for index in range(total):
        progress = (index + 1 - first) / TARGET_COUNT
        factor = _smoothstep(progress)
        values.append(
            Target(
                (index + 1) * POINT_NS,
                tuple(h + (v - h) * factor for h, v in zip(hold, target, strict=True)),
            )
        )
    return tuple(values)


class Driver:
    def __init__(self, stack: Stack) -> None:
        self.stack = stack
        self.context = _context(stack.info, "p02a-evidence")
        self.lease = self._acquire()
        self.command_seq = 0
        self.observation_id = 1
        self.windows: list[dict[str, object]] = []
        self.last_wall = 0

    def _acquire(self) -> str:
        response = self.stack.call(
            "acquire_control",
            AcquireControl.Request(
                context=context_message(self.context),
                owner_id="p02a-evidence",
                duration=duration(LEASE_NS),
            ),
        )
        assert response.success, response.reason
        self._renew_at = time.monotonic()
        return response.lease_id

    def renew(self) -> None:
        if time.monotonic() - self._renew_at < LEASE_RENEW_NS / 1e9:
            return
        response = self.stack.call(
            "renew_control",
            RenewControl.Request(
                context=context_message(self.context),
                lease_id=self.lease,
                duration=duration(LEASE_NS),
            ),
        )
        assert response.success, response.reason
        self._renew_at = time.monotonic()

    def anchor(self):
        response = self.stack.call(
            "get_stream_anchor",
            GetStreamAnchor.Request(
                context=context_message(self.context),
                lease_id=self.lease,
                has_start=False,
                start_sim_time_ns=0,
            ),
        )
        assert response.success, response.reason
        return anchor_from_message(response.anchor)

    def query(self, command_id: str):
        response = self.stack.call(
            "query_stream_command",
            QueryStreamCommand.Request(
                context=context_message(self.context), command_id=command_id
            ),
            timeout=10.0,
        )
        assert response.success, response.reason
        return execution_feedback_from_message(response.execution)

    def _request(self, target: tuple[float, ...], command_id: str):
        anchor = self.anchor()
        hold = tuple(float(value) for value in anchor.reference.positions)
        observed = self.stack.latest_sim_ns()
        lead_ns = int(anchor.start_sim_time_ns) - observed
        assert lead_ns > POINT_NS, (
            f"anchor lead {lead_ns} ns leaves no selectable window after the observation"
        )
        self.command_seq += 1
        self.observation_id += 1
        request = StreamRequest(
            trajectory=Trajectory(
                self.context,
                self.lease,
                command_id,
                self.command_seq,
                self.observation_id,
                observed + 10_000_000_000,
                tuple(self.stack.info.joint_names),
                _source_targets(hold, target, lead_ns),
            ),
            kind=REQUEST_STREAM_FIRST,
            anchor=anchor,
            observation_time_ns=observed,
            target_count=TARGET_COUNT,
            settle_at_end=True,
        )
        return request, hold

    def submit(self, target: tuple[float, ...], label: str) -> dict[str, object]:
        self.renew()
        command_id = f"{label}-{uuid.uuid4().hex[:8]}"
        request, hold = self._request(target, command_id)
        states_before = len(self.stack.states)
        response = self.stack.call(
            "submit_follower",
            SubmitFollower.Request(request=stream_command_message(request)),
            timeout=20.0,
        )
        record: dict[str, object] = {
            "label": label,
            "command_id": command_id,
            "request": request.model_dump(mode="json"),
            "accepted": bool(response.accepted),
            "reason": response.reason,
            "hold": hold,
            "target": list(target),
        }
        if not response.accepted:
            record["states"] = self._states(states_before)
            self.windows.append(record)
            return record
        assert response.has_reference, "accepted window has no reference"
        reference = follower_reference_from_message(response.reference)
        assert isinstance(reference, FollowerReference)
        assert response.has_execution and response.has_chunk
        acceptance = execution_feedback_from_message(response.execution)
        record["reference"] = reference.model_dump(mode="json")
        record["acceptance_feedback"] = acceptance.model_dump(mode="json")
        terminal = self._wait_terminal(command_id, states_before, response.reason)
        record.update(terminal)
        self.windows.append(record)
        return record

    def _states(self, start: int) -> list[dict[str, object]]:
        return [
            {
                "state_seq": int(message.state_seq),
                "sim_time_ns": _stamp_ns(message.stamp),
                "status": message.status,
                "joint_names": list(message.joints.name),
                "positions": list(message.joints.position),
                "velocities": list(message.joints.velocity),
            }
            for message in self.stack.states[start:]
        ]

    def _wait_terminal(
        self, command_id: str, states_before: int, submit_reason: str
    ) -> dict[str, object]:
        deadline = time.monotonic() + 30.0
        feedback_start = len(self.stack.feedback)
        while time.monotonic() < deadline:
            self.stack.spin(0.05)
            self.renew()
            execution = self.query(command_id)
            if execution.phase in TERMINAL:
                return {
                    "terminal": execution.model_dump(mode="json"),
                    "states": self._states(states_before),
                    "feedback": [
                        execution_feedback_from_message(item).model_dump(mode="json")
                        for item in self.stack.feedback[feedback_start:]
                        if item.command_id == command_id
                    ],
                    "submit_reason": submit_reason,
                }
        raise AssertionError(f"command {command_id} did not reach a terminal phase")

    def wait_phase(self, command_id: str, phases: set[str], timeout: float = 20.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.stack.spin(0.05)
            self.renew()
            execution = self.query(command_id)
            if execution.phase in phases:
                return execution
        raise AssertionError(f"command {command_id} did not reach {sorted(phases)}")

    def close(self) -> None:
        self.stack.call(
            "release_control",
            ReleaseControl.Request(
                context=context_message(self.context), lease_id=self.lease
            ),
            timeout=10.0,
        )


def _positions(state: RobotState) -> dict[str, float]:
    return dict(zip(state.joints.name, state.joints.position, strict=True))


def _write_evidence(status: str, stack: Stack, payload: dict[str, object]) -> None:
    document = {
        "run_id": stack.run_id,
        "status": status,
        "namespace": stack.namespace,
        "profile": str(PROFILE_PATH),
        "backend_config": str(stack.config),
        **payload,
    }
    (stack.evidence_dir / "evidence.json").write_text(
        json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def test_marvin_robot_api_execution(tmp_path):
    assert RECIPE.is_file(), f"missing direct backend recipe: {RECIPE}"
    stack = Stack(tmp_path)
    payload: dict[str, object] = {}
    try:
        stack.wait_ready()
        assert stack.info is not None
        assert tuple(stack.info.joint_names) == (
            "left_joint1", "left_joint2", "left_joint3", "left_joint4", "left_joint5",
            "left_joint6", "left_joint7", "left_gripper_width",
            "right_joint1", "right_joint2", "right_joint3", "right_joint4", "right_joint5",
            "right_joint6", "right_joint7", "right_gripper_width",
        )
        driver = Driver(stack)
        stack.wait_for(lambda: stack.latest_state() is not None)
        initial = stack.latest_state()
        assert initial is not None
        initial_positions = _positions(initial)
        assert initial_positions["left_gripper_width"] == pytest.approx(0.0814, abs=1e-3)
        assert initial_positions["right_gripper_width"] == pytest.approx(0.0814, abs=1e-3)
        reference = {
            name: float(value)
            for name, value in zip(
                stack.info.joint_names, PROFILE.initial_positions, strict=True
            )
        }

        index_of = {name: position for position, name in enumerate(stack.info.joint_names)}

        def target(**changes: float) -> tuple[float, ...]:
            values = [reference[name] for name in stack.info.joint_names]
            for name, value in changes.items():
                values[index_of[name]] = value
            return tuple(values)

        def accept(record: dict[str, object]) -> None:
            assert record["accepted"], record["reason"]
            reference.update(
                zip(stack.info.joint_names, record["target"], strict=True)
            )

        # 1. Left arm moves; the right arm holds its reference.
        left_target = target(**{
            f"left_joint{index}": 0.05 for index in range(1, 8)
        })
        left = driver.submit(left_target, "left-arm")
        accept(left)
        left_state = stack.latest_state()
        assert left_state is not None
        left_positions = _positions(left_state)
        for index in range(1, 8):
            assert left_positions[f"left_joint{index}"] == pytest.approx(0.05, abs=0.01)
            assert left_positions[f"right_joint{index}"] == pytest.approx(0.0, abs=0.01)
        assert left["terminal"]["phase"] == "COMPLETED"

        # 2. Both arms move together.
        coordinated_target = target(
            left_joint3=0.10,
            right_joint3=0.10,
        )
        coordinated = driver.submit(coordinated_target, "both-arms")
        accept(coordinated)
        coordinated_state = stack.latest_state()
        assert coordinated_state is not None
        coordinated_positions = _positions(coordinated_state)
        assert coordinated_positions["left_joint3"] == pytest.approx(0.10, abs=0.01)
        assert coordinated_positions["right_joint3"] == pytest.approx(0.10, abs=0.01)

        # 3. Left gripper closes then opens; the right gripper stays open.
        closed = driver.submit(target(left_gripper_width=0.0414), "left-close")
        accept(closed)
        closed_state = stack.latest_state()
        assert closed_state is not None
        closed_positions = _positions(closed_state)
        assert closed_positions["left_gripper_width"] == pytest.approx(0.0414, abs=0.002)
        assert closed_positions["right_gripper_width"] == pytest.approx(0.0814, abs=0.002)
        left_close_feedback = closed["terminal"]
        assert left_close_feedback["phase"] == "COMPLETED"

        opened = driver.submit(target(left_gripper_width=0.0814), "left-open")
        accept(opened)
        opened_state = stack.latest_state()
        assert opened_state is not None
        opened_positions = _positions(opened_state)
        assert opened_positions["left_gripper_width"] == pytest.approx(0.0814, abs=0.002)

        # 4. Right gripper closes then opens independently.
        right_closed = driver.submit(target(right_gripper_width=0.0414), "right-close")
        accept(right_closed)
        right_closed_state = stack.latest_state()
        assert right_closed_state is not None
        right_closed_positions = _positions(right_closed_state)
        assert right_closed_positions["right_gripper_width"] == pytest.approx(0.0414, abs=0.002)
        assert right_closed_positions["left_gripper_width"] == pytest.approx(0.0814, abs=0.002)

        right_opened = driver.submit(target(right_gripper_width=0.0814), "right-open")
        accept(right_opened)
        right_opened_state = stack.latest_state()
        assert right_opened_state is not None
        assert _positions(right_opened_state)["right_gripper_width"] == pytest.approx(
            0.0814, abs=0.002
        )

        # 5. A target outside the profile limit is rejected without motion.
        # Let the preceding valid opening finish its small servo relaxation.
        stack.spin(0.5)
        before_limit = stack.latest_state()
        limit = driver.submit(target(left_joint1=4.0), "over-limit")
        assert not limit["accepted"]
        assert limit["reason"]
        stack.spin(0.2)
        after_limit = stack.latest_state()
        assert before_limit is not None and after_limit is not None
        assert _positions(after_limit) == pytest.approx(_positions(before_limit), abs=1e-6)

        # 6. Stop a running window; the grippers are not opened by stopping.
        stop_target = target(**{
            "left_joint2": -0.10,
            "right_joint2": -0.10,
        })
        command_id = f"stop-{uuid.uuid4().hex[:8]}"
        request, _hold = driver._request(stop_target, command_id)
        response = stack.call(
            "submit_follower",
            SubmitFollower.Request(request=stream_command_message(request)),
            timeout=20.0,
        )
        assert response.accepted, response.reason
        driver.wait_phase(command_id, {"STARTED"}, 10.0)
        running = True
        before_stop = stack.latest_state()
        assert before_stop is not None
        stop_response = stack.call(
            "stop",
            Stop.Request(
                context=context_message(driver.context),
                lease_id=driver.lease,
                reason="P02A_OPERATOR_STOP",
            ),
            timeout=10.0,
        )
        assert stop_response.success, stop_response.reason
        terminal = driver.wait_phase(command_id, TERMINAL, 10.0)
        assert terminal.phase in {"STOPPED", "CANCELED"}, terminal.model_dump()
        assert terminal.stop_reason == "P02A_OPERATOR_STOP", terminal.model_dump()
        after_stop = stack.latest_state()
        assert after_stop is not None
        before_positions = _positions(before_stop)
        after_positions = _positions(after_stop)
        assert after_positions["left_gripper_width"] == pytest.approx(
            before_positions["left_gripper_width"], abs=0.002
        )
        assert after_positions["right_gripper_width"] == pytest.approx(
            before_positions["right_gripper_width"], abs=0.002
        )
        stop_record = {
            "label": "running-stop",
            "command_id": command_id,
            "request": request.model_dump(mode="json"),
            "accepted": True,
            "reason": response.reason,
            "running": bool(running),
            "stop_request": {
                "reason": "P02A_OPERATOR_STOP",
                "lease_id": driver.lease,
            },
            "stop_ack": {"success": bool(stop_response.success), "reason": stop_response.reason},
            "before_stop": {
                "sim_time_ns": _stamp_ns(before_stop.stamp),
                "positions": list(before_stop.joints.position),
            },
            "after_stop": {
                "sim_time_ns": _stamp_ns(after_stop.stamp),
                "positions": list(after_stop.joints.position),
            },
            "feedback": [
                execution_feedback_from_message(item).model_dump(mode="json")
                for item in stack.feedback
                if item.command_id == command_id
            ],
            "states": [
                {
                    "state_seq": int(message.state_seq),
                    "sim_time_ns": _stamp_ns(message.stamp),
                    "positions": list(message.joints.position),
                    "velocities": list(message.joints.velocity),
                }
                for message in stack.states[-20:]
            ],
            "terminal": terminal.model_dump(mode="json"),
        }
        driver.windows.append(stop_record)
        driver.close()
        payload = {
            "info": {
                "joint_names": list(stack.info.joint_names),
                "capabilities": list(stack.info.capabilities),
                "profile": stack.info.context.robot_profile_hash,
            },
            "windows": driver.windows,
        }
        status = stack.close()
        assert status == 0, stack.output()
        _write_evidence("completed", stack, payload)
    except BaseException:
        stack.close()
        _write_evidence("failed", stack, payload)
        raise


def test_direct_recipe_selects_marvin_without_task(tmp_path):
    document = _direct_document(RECIPE)
    assert "task" not in document["robot"]
    assert "task_mode" not in document["robot"]
    scene = yaml.safe_load(Path(document["robot"]["scene"]).read_text(encoding="utf-8"))
    assert scene["robot"]["assembly"] == "marvin"
