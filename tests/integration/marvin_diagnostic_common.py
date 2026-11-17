"""Shared P02B Robot API diagnostics: owner snapshots, windows and holds.

The candidate plan comes from the same task module used by the P03 teacher;
this test only converts its joint targets into finite SubmitFollower windows.
Contacts and object motion are read through the simulation contact-snapshot
service, captured on the owner thread.  Everything is skipped on hosts without
the ROS SDK.
"""

from __future__ import annotations

import json
import math
import os
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path

import yaml

try:
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
    from robot_contracts.profiles import load_profile
    from robot_contracts.stream_request import REQUEST_STREAM_FIRST, StreamRequest, Trajectory
    from robot_contracts.trajectory.follower_reference import FollowerReference
    from robot_ros_support.conversions import (
        context_from_message,
        context_message,
        duration,
    )
    from robot_ros_support.follower_conversions import follower_reference_from_message
    from robot_ros_support.qos import SENSOR_QOS
    from robot_ros_support.stream_conversions import (
        anchor_from_message,
        execution_feedback_from_message,
        stream_command_message,
    )
    from sim_api_msgs.srv import GetContactSnapshot, ResetSimulation
    ROS_IMPORT_ERROR: BaseException | None = None
except ImportError as error:  # pragma: no cover - host without ROS
    ROS_IMPORT_ERROR = error

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.config.follower_config import load_follower_config  # noqa: E402
from robot_backend_sim.config.marvin_scene_config import load_marvin_scene_config  # noqa: E402
from robot_backend_sim.config.scenario_config import ROBOT_REFERENCE_FIELDS  # noqa: E402
from robot_backend_sim.physics.marvin.scene import build_marvin_scene  # noqa: E402
from robot_backend_sim.tasks.handover.config import load_handover_plan_config  # noqa: E402
from robot_backend_sim.tasks.handover.plan import plan_handover  # noqa: E402

SCENARIO = ROOT / "shared/scenarios/marvin_sim/handover"
RECIPE = Path(
    os.environ.get(
        "P02B_BACKEND_CONFIG",
        str(SCENARIO / "configs/simulation/backend/backend_marvin_control.yaml"),
    )
)
EVIDENCE_DIR = Path(os.environ.get("P02B_EVIDENCE_DIR", "/data/marvin_sim/handover/diagnostics/p02b"))
SCENE_PATH = SCENARIO / "configs/simulation/scene/marvin_handover.yaml"
PLANNING_PATH = SCENARIO / "configs/simulation/planning/handover_paths.yaml"
PROFILE_PATH = SCENARIO / "profiles/robots/marvin_handover_v1/profile.json"
SCHEMAS = ROOT / "shared/common/interfaces/schemas"
PROFILE = load_profile(PROFILE_PATH, SCHEMAS)
_PLANNING = load_handover_plan_config(PLANNING_PATH)
_FOLLOWER = load_follower_config(SCENARIO / "profiles/control/marvin_follower_v1.yaml")


def _direct_document(path: Path) -> dict:
    """Load the direct recipe and expand robot paths like ``backend --config``."""
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    robot = document["robot"]
    for field in ROBOT_REFERENCE_FIELDS:
        value = robot.get(field)
        if isinstance(value, str):
            robot[field] = str((path.parent / value).resolve())
    return document

# The execution window is the follower's fixed target count at the planning
# recipe's period; the recipe fields are the single source for both values.
POINT_NS = int(_PLANNING.execution.period_ns)
FOLLOWER_TARGET_COUNT = int(_FOLLOWER.target_count)
MAX_WINDOW_S = FOLLOWER_TARGET_COUNT * POINT_NS / 1_000_000_000
LEASE_NS = 8_000_000_000
LEASE_RENEW_NS = 3_000_000_000
TERMINAL = frozenset({"COMPLETED", "STOPPED", "FAILED", "CANCELED"})


def _stamp_ns(stamp: object) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _quaternion_matrix(quaternion):
    import numpy as np

    w, x, y, z = quaternion
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _rotation_angle(first, second) -> float:
    import math

    import numpy as np

    value = (float(np.trace(first.T @ second)) - 1.0) / 2.0
    return math.acos(max(-1.0, min(1.0, value)))


class Stack:
    def __init__(self, tmp_path: Path) -> None:
        self.input_configs = {
            "scene": load_marvin_scene_config(SCENE_PATH).model_dump(mode="json"),
            "planning": _PLANNING.model_dump(mode="json"),
        }
        self.namespace = f"/p02b_{uuid.uuid4().hex[:8]}"
        self.run_id = f"p02b-{uuid.uuid4().hex[:8]}"
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
        self.node = Node(f"p02b_client_{uuid.uuid4().hex[:8]}")
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
                ("reset_simulation", ResetSimulation),
                ("get_contact_snapshot", GetContactSnapshot),
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

    def wait_ready(self, timeout: float = 60.0) -> None:
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

    def call(self, name: str, request: object, timeout: float = 30.0) -> object:
        future = self.clients[name].call_async(request)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)
            if future.done():
                return future.result()
        raise AssertionError(f"service {name} timed out")

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


def _source_targets(hold, target, lead_ns: int, point_count: int):
    from robot_contracts.models import Target

    first = int(lead_ns // POINT_NS)
    values = []
    for index in range(first + point_count):
        progress = (index + 1 - first) / point_count
        factor = _smoothstep(progress)
        values.append(
            Target(
                (index + 1) * POINT_NS,
                tuple(h + (v - h) * factor for h, v in zip(hold, target, strict=True)),
            )
        )
    return tuple(values)


class Driver:
    def __init__(self, stack: Stack, context: RunContext, episode: str) -> None:
        self.stack = stack
        self.context = context
        self.lease = self._acquire(episode)
        self.command_seq = 0
        self.observation_id = 1
        self.reference = tuple(PROFILE.initial_positions)
        self.windows: list[dict[str, object]] = []
        self._renew_at = time.monotonic()

    def _acquire(self, owner: str) -> str:
        response = self.stack.call(
            "acquire_control",
            AcquireControl.Request(
                context=context_message(self.context),
                owner_id=owner,
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
            timeout=15.0,
        )
        assert response.success, response.reason
        return execution_feedback_from_message(response.execution)

    def request(self, target: tuple[float, ...], command_id: str, duration_s: float):
        self.stack.spin(0.1)
        anchor = self.anchor()
        hold = tuple(float(value) for value in anchor.reference.positions)
        observed = self.stack.latest_sim_ns()
        lead_ns = int(anchor.start_sim_time_ns) - observed
        assert lead_ns > POINT_NS, (
            f"anchor lead {lead_ns} ns leaves no selectable window after the observation"
        )
        point_count = FOLLOWER_TARGET_COUNT
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
                _source_targets(hold, target, lead_ns, point_count),
            ),
            kind=REQUEST_STREAM_FIRST,
            anchor=anchor,
            observation_time_ns=observed,
            target_count=point_count,
            settle_at_end=True,
        )
        return request, hold

    def submit(
        self, target: tuple[float, ...], label: str, duration_s: float
    ) -> dict[str, object]:
        """Submit one planned segment, split into bounded follower windows."""
        start = self.reference
        maximum_delta = max(
            abs(last - first) for first, last in zip(start, target, strict=True)
        )
        # The follower fits a 1 s, 20-point curve; keep each window small
        # enough for the 1.0 rad/s^2 arm acceleration and the 0.05 rad
        # reference-deviation budget rather than relying on the nominal
        # segment duration alone.
        count = max(
            1,
            math.ceil(duration_s / MAX_WINDOW_S),
            math.ceil(maximum_delta / 0.10),
        )
        records: list[dict[str, object]] = []
        for index in range(1, count + 1):
            fraction = index / count
            sub_target = tuple(
                first + (last - first) * fraction
                for first, last in zip(start, target, strict=True)
            )
            sub_label = label if count == 1 else f"{label}-{index}"
            record = self._submit_window(sub_target, sub_label, MAX_WINDOW_S)
            records.append(record)
            if not record["accepted"] or record.get("terminal", {}).get("phase") != "COMPLETED":
                break
            self.reference = sub_target
        return records[-1]

    def _submit_window(
        self, target: tuple[float, ...], label: str, duration_s: float
    ) -> dict[str, object]:
        self.renew()
        command_id = f"{label}-{uuid.uuid4().hex[:8]}"
        request, hold = self.request(target, command_id, duration_s)
        states_before = len(self.stack.states)
        response = self.stack.call(
            "submit_follower",
            SubmitFollower.Request(request=stream_command_message(request)),
            timeout=30.0,
        )
        record: dict[str, object] = {
            "label": label,
            "command_id": command_id,
            "duration_s": duration_s,
            "target": list(target),
            "hold": list(hold),
            "accepted": bool(response.accepted),
            "reason": response.reason,
        }
        if not response.accepted:
            record["states"] = self._states(states_before)
            self.windows.append(record)
            return record
        assert response.has_reference, "accepted window has no reference"
        reference = follower_reference_from_message(response.reference)
        assert isinstance(reference, FollowerReference)
        record["reference"] = {
            "accepted_sim_time_ns": reference.accepted_sim_time_ns,
            "curve_duration_ns": reference.curve_duration_ns,
        }
        terminal = self._wait_terminal(command_id, states_before)
        record.update(terminal)
        self.windows.append(record)
        if record["terminal"]["phase"] != "COMPLETED":
            return record
        self.reference = target
        return record

    def _states(self, start: int) -> list[dict[str, object]]:
        return [
            {
                "state_seq": int(message.state_seq),
                "sim_time_ns": _stamp_ns(message.stamp),
                "status": message.status,
                "positions": list(message.joints.position),
            }
            for message in self.stack.states[start:]
        ]

    def _wait_terminal(self, command_id: str, states_before: int) -> dict[str, object]:
        deadline = time.monotonic() + 90.0
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
                }
        raise AssertionError(f"command {command_id} did not reach a terminal phase")

    def wait_phase(self, command_id: str, phases: set[str], timeout: float = 30.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.stack.spin(0.05)
            self.renew()
            execution = self.query(command_id)
            if execution.phase in phases:
                return execution
        raise AssertionError(f"command {command_id} did not reach {sorted(phases)}")

    def stop(self, reason: str) -> dict[str, object]:
        response = self.stack.call(
            "stop",
            Stop.Request(
                context=context_message(self.context),
                lease_id=self.lease,
                reason=reason,
            ),
            timeout=15.0,
        )
        return {"success": bool(response.success), "reason": response.reason}

    def reset(self, seed: int):
        response = self.stack.call(
            "reset_simulation",
            ResetSimulation.Request(
                context=context_message(self.context),
                lease_id=self.lease,
                seed=seed,
            ),
            timeout=30.0,
        )
        assert response.success, response.reason
        return context_from_message(response.new_context)

    def close(self) -> None:
        self.stack.call(
            "release_control",
            ReleaseControl.Request(
                context=context_message(self.context), lease_id=self.lease
            ),
            timeout=15.0,
        )


def _snapshot(stack: Stack, context: RunContext) -> dict[str, object]:
    response = stack.call(
        "get_contact_snapshot",
        GetContactSnapshot.Request(context=context_message(context)),
        timeout=15.0,
    )
    assert response.success, response.reason
    message = response.snapshot
    objects = [
        {
            "name": str(name),
            "center": [
                float(message.object_center[3 * index + 0]),
                float(message.object_center[3 * index + 1]),
                float(message.object_center[3 * index + 2]),
            ],
            "quaternion": [
                float(message.object_quaternion[4 * index + 0]),
                float(message.object_quaternion[4 * index + 1]),
                float(message.object_quaternion[4 * index + 2]),
                float(message.object_quaternion[4 * index + 3]),
            ],
            "linear_velocity": [
                float(message.object_linear_velocity[3 * index + 0]),
                float(message.object_linear_velocity[3 * index + 1]),
                float(message.object_linear_velocity[3 * index + 2]),
            ],
            "angular_velocity": [
                float(message.object_angular_velocity[3 * index + 0]),
                float(message.object_angular_velocity[3 * index + 1]),
                float(message.object_angular_velocity[3 * index + 2]),
            ],
        }
        for index, name in enumerate(message.object_names)
    ]
    contacts = [
        {
            "side": str(message.contact_side[index]),
            "finger": int(message.contact_finger[index]),
            "object": str(message.contact_object[index]),
            "force": float(message.contact_normal_force[index]),
            "count": int(message.contact_count[index]),
        }
        for index in range(len(message.contact_side))
    ]
    supports = [
        {
            "object": str(message.support_object[index]),
            "body": str(message.support_body[index]),
            "force": float(message.support_normal_force[index]),
            "count": int(message.support_contact_count[index]),
        }
        for index in range(len(message.support_object))
    ]
    grippers = [
        {
            "side": str(message.gripper_sides[index]),
            "opening": float(message.gripper_opening[index]),
            "tcp_position": [
                float(message.tcp_position[3 * index + 0]),
                float(message.tcp_position[3 * index + 1]),
                float(message.tcp_position[3 * index + 2]),
            ],
            "tcp_quaternion": [
                float(message.tcp_quaternion[4 * index + 0]),
                float(message.tcp_quaternion[4 * index + 1]),
                float(message.tcp_quaternion[4 * index + 2]),
                float(message.tcp_quaternion[4 * index + 3]),
            ],
        }
        for index in range(len(message.gripper_sides))
    ]
    return {
        "sim_time_ns": _stamp_ns(message.sample_time),
        "capture_seq": int(message.capture_seq),
        "objects": objects,
        "contacts": contacts,
        "supports": supports,
        "grippers": grippers,
    }


def _object(snapshot: dict[str, object], name: str) -> dict[str, object]:
    return next(item for item in snapshot["objects"] if item["name"] == name)


def _gripper(snapshot: dict[str, object], side: str) -> dict[str, object]:
    return next(item for item in snapshot["grippers"] if item["side"] == side)


def _finger_contacts(snapshot: dict[str, object], side: str, object_name: str):
    return [
        item
        for item in snapshot["contacts"]
        if item["side"] == side and item["object"] == object_name
    ]


def _holding(snapshot: dict[str, object], side: str, object_name: str, minimum: float) -> bool:
    contacts = _finger_contacts(snapshot, side, object_name)
    return (
        len(contacts) == 2
        and {item["finger"] for item in contacts} == {1, 2}
        and all(item["count"] > 0 and item["force"] >= minimum for item in contacts)
    )


def _support_force(snapshot: dict[str, object], object_name: str, body: str) -> float:
    return sum(
        item["force"]
        for item in snapshot["supports"]
        if item["object"] == object_name and item["body"] == body
    )


def _relative_pose(snapshot: dict[str, object], side: str, object_name: str):
    import numpy as np

    gripper = _gripper(snapshot, side)
    obj = _object(snapshot, object_name)
    rotation = _quaternion_matrix(tuple(gripper["tcp_quaternion"]))
    position = np.asarray(gripper["tcp_position"], dtype=np.float64)
    relative = rotation.T @ (np.asarray(obj["center"], dtype=np.float64) - position)
    return relative, rotation.T @ _quaternion_matrix(tuple(obj["quaternion"]))


def _hold_snapshots(
    stack: Stack,
    driver: Driver,
    label: str,
    object_name: str,
    sides: tuple[str, ...],
    *,
    minimum_force: float,
    duration_s: float,
    interval_s: float,
    require_off_table: bool = True,
    sink: list[dict[str, object]] | None = None,
) -> list[dict[str, object]]:
    snapshots = []
    deadline = time.monotonic() + duration_s
    while True:
        snapshot = _snapshot(stack, driver.context)
        snapshot["label"] = label
        snapshots.append(snapshot)
        if sink is not None:
            sink.append(snapshot)
        for side in sides:
            assert _holding(snapshot, side, object_name, minimum_force), (
                f"{label}: {side} lost the block",
                snapshot,
            )
        if require_off_table:
            assert _support_force(snapshot, object_name, "table") == 0.0, (
                f"{label}: block is still supported by the table",
                snapshot,
            )
        if time.monotonic() >= deadline:
            break
        stack.spin(interval_s)
    return snapshots


def _assert_relative_stable(
    snapshots: list[dict[str, object]],
    side: str,
    object_name: str,
    *,
    position_tolerance_m: float,
    orientation_tolerance_rad: float,
    object_max_linear_speed: float,
    object_max_angular_speed: float,
) -> None:
    first, first_rotation = _relative_pose(snapshots[0], side, object_name)
    for snapshot in snapshots:
        position, rotation = _relative_pose(snapshot, side, object_name)
        assert abs(position - first).max() <= position_tolerance_m, (
            "relative grasp point moved",
            position.tolist(),
            first.tolist(),
        )
        assert _rotation_angle(first_rotation, rotation) <= orientation_tolerance_rad, (
            "relative block orientation moved"
        )
        obj = _object(snapshot, object_name)
        assert max(abs(value) for value in obj["linear_velocity"]) <= object_max_linear_speed, (
            "block linear speed too high",
            obj["linear_velocity"],
        )
        assert max(abs(value) for value in obj["angular_velocity"]) <= (
            object_max_angular_speed
        ), ("block angular speed too high", obj["angular_velocity"])


def _public_target(
    scene, qpos: tuple[float, ...], joint_names: tuple[str, ...],
    open_width: float, closed_width: float, left_open: bool, right_open: bool,
):
    values = {}
    for side in ("left", "right"):
        chain = next(item for item in scene.robot_geometry.chains if item.side == side)
        for index, name in enumerate(chain.joint_names, start=1):
            address = int(scene.model.jnt_qposadr[scene.model.joint(name).id])
            values[f"{side}_joint{index}"] = float(qpos[address])
    values["left_gripper_width"] = open_width if left_open else closed_width
    values["right_gripper_width"] = open_width if right_open else closed_width
    return tuple(values[name] for name in joint_names)


def _plan_for_execution():
    scene = build_marvin_scene(load_marvin_scene_config(SCENE_PATH))
    planning = _PLANNING
    by_name = {joint.name: joint for joint in PROFILE.joints}
    velocities = {
        side: tuple(
            float(by_name[f"{side}_joint{index}"].max_velocity) for index in range(1, 8)
        )
        for side in ("left", "right")
    }
    plan = plan_handover(scene, planning, max_velocities=velocities)
    assert plan.passed, plan.failures
    targets = {target.name: target for target in plan.targets}
    segments = {segment.name: segment for segment in plan.segments}
    return scene, planning, plan, targets, segments


def _write_evidence(status: str, stack: Stack, payload: dict[str, object]) -> None:
    document = {
        "input_configs": stack.input_configs,
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




def setup_dual_hold(
    stack: Stack,
    driver: Driver,
    scene: object,
    planning: object,
    targets: dict[str, object],
    segments: dict[str, object],
    names: tuple[str, ...],
    *,
    sink: list[dict[str, object]] | None = None,
) -> tuple[dict[str, tuple[object, object]], list[dict[str, object]]]:
    """Grasp the block with both hands on the table and return the baseline."""
    open_width = scene.gripper_open_total
    closed_width = planning.grasp.closed_opening_m

    def target(name: str, *, left_open: bool, right_open: bool):
        return _public_target(
            scene, targets[name].qpos, names, open_width, closed_width,
            left_open, right_open,
        )

    for name, segment, label in (
        ("left_ready", "left_cohold_approach", "cohold-left-ready"),
        ("left_cohold_pregrasp", "left_cohold_pregrasp_path", "cohold-left-pregrasp"),
        ("left_cohold_grasp", "left_cohold_grasp_path", "cohold-left-grasp"),
    ):
        window = driver.submit(
            target(name, left_open=True, right_open=True), label,
            segments[segment].duration_s,
        )
        assert window["accepted"] and window.get("terminal", {}).get("phase") == "COMPLETED", (
            label, window
        )
    window = driver.submit(
        target("left_cohold_grasp", left_open=False, right_open=True),
        "cohold-left-close", 0.5,
    )
    assert window["accepted"] and window.get("terminal", {}).get("phase") == "COMPLETED"
    for name, segment, label in (
        ("right_cohold_wait", "right_cohold_wait_path", "cohold-right-wait"),
        ("right_cohold_pregrasp", "right_cohold_pregrasp_path", "cohold-right-pregrasp"),
        ("right_cohold_grasp", "right_cohold_grasp_path", "cohold-right-grasp"),
    ):
        window = driver.submit(
            target(name, left_open=False, right_open=True), label,
            segments[segment].duration_s,
        )
        assert window["accepted"] and window.get("terminal", {}).get("phase") == "COMPLETED", (
            label, window
        )
    window = driver.submit(
        target("right_cohold_grasp", left_open=False, right_open=False),
        "cohold-right-close", 0.5,
    )
    assert window["accepted"] and window.get("terminal", {}).get("phase") == "COMPLETED"
    held = _hold_snapshots(
        stack, driver, "dual_table_hold", "block", ("left", "right"),
        minimum_force=planning.diagnostics.contact_force_min_n,
        duration_s=planning.diagnostics.hold_duration_s,
        interval_s=planning.diagnostics.hold_sample_interval_s,
        require_off_table=False,
        sink=sink,
    )
    expected = {
        side: _relative_pose(held[0], side, "block") for side in ("left", "right")
    }
    return expected, held
