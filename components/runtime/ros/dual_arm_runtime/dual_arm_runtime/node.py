"""Scripted episode orchestration using only the public Robot API."""

import argparse
import json
import threading
import time
import uuid
from dataclasses import asdict, replace
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from robot_api_msgs.action import ExecuteTrajectory
from robot_api_msgs.msg import (
    BackendInfo,
    CameraFrame,
    CommandEvent,
    ObservationIndex,
    RobotState,
    RuntimeStatus,
    TrajectoryCommand,
)
from robot_api_msgs.srv import (
    AcquireControl,
    EndEpisode,
    ReleaseControl,
    RenewControl,
    StartEpisode,
    Stop,
)
from robot_contracts.models import ContractError, Target, Trajectory
from robot_ros_support.conversions import (
    command_message,
    context_from_message,
    context_message,
    duration,
    stamp,
    time_ns,
)
from robot_ros_support.qos import EVENT_QOS, INFO_QOS, SENSOR_QOS

from dual_arm_runtime.configuration import Configuration, load_configuration
from dual_arm_runtime.observation import ObservationBuffer, ObservationRef
from dual_arm_runtime.recording import EpisodeRecorder


class RuntimeNode(Node):
    """ROS callbacks stay responsive while a worker manages one episode at a time."""

    def __init__(self, configuration: Configuration, output: Path, auto_start: bool) -> None:
        super().__init__("episode_runtime")
        self.config, self.output = configuration, output
        self._recipe = configuration.recipe
        self._root = self._recipe["robot_namespace"].rstrip("/")
        self._group = ReentrantCallbackGroup()
        self._condition = threading.Condition()
        self._start_lock = threading.Lock()
        self._renew_lock = threading.Lock()
        self._renew_enabled = False
        self._stop_requested = threading.Event()
        self.done = threading.Event()
        self.failure_reason = ""
        self.record_directory = ""
        self._info: BackendInfo | None = None
        self._recorder: EpisodeRecorder | None = None
        self._run_context = None
        self._lease_id = ""
        self._renew_future = None
        self._last_renew_ns = 0
        self._last_observation: ObservationRef | None = None
        self._last_index_id = 0
        self._terminal_events: set[str] = set()
        self._worker: threading.Thread | None = None
        self._buffer = ObservationBuffer(
            (configuration.profile.camera.name,),
            configuration.observation["max_skew_ns"],
            configuration.observation["max_age_ns"],
        )
        self._service_clients = {}
        for service_type, name in (
            (AcquireControl, "acquire_control"),
            (RenewControl, "renew_control"),
            (ReleaseControl, "release_control"),
            (Stop, "stop"),
        ):
            self._service_clients[name] = self.create_client(
                service_type,
                f"{self._root}/{name}",
                callback_group=self._group,
            )
        self._action = ActionClient(
            self,
            ExecuteTrajectory,
            f"{self._root}/execute_trajectory",
            callback_group=self._group,
        )
        self._status = self.create_publisher(RuntimeStatus, "/runtime/status", INFO_QOS)
        self._commands = self.create_publisher(
            TrajectoryCommand,
            "/runtime/control/commands",
            EVENT_QOS,
        )
        self._indices = self.create_publisher(
            ObservationIndex,
            "/runtime/observation_index",
            EVENT_QOS,
        )
        self.create_subscription(BackendInfo, f"{self._root}/info", self._on_info, INFO_QOS)
        self.create_subscription(RobotState, f"{self._root}/state", self._on_state, SENSOR_QOS)
        self._camera_topic = f"{self._root}/cameras/{configuration.profile.camera.name}/frame"
        self.create_subscription(CameraFrame, self._camera_topic, self._on_frame, SENSOR_QOS)
        self.create_subscription(
            CommandEvent,
            f"{self._root}/command_events",
            self._on_event,
            EVENT_QOS,
        )
        self.create_service(
            StartEpisode,
            "/runtime/episode/start",
            self._start_service,
            callback_group=self._group,
        )
        self.create_service(
            EndEpisode,
            "/runtime/episode/end",
            self._end_service,
            callback_group=self._group,
        )
        self.create_timer(0.05, self._publish_observation)
        self.create_timer(0.05, self._renew_lease)
        if auto_start:
            self._auto_timer = self.create_timer(0.1, self._auto_start)

    def _set_status(self, phase: str, episode_id: str = "", reason: str = "") -> None:
        self._status.publish(
            RuntimeStatus(
                phase=phase,
                episode_id=episode_id,
                reason=reason,
                record_directory=self.record_directory,
            )
        )

    def _record(self, topic: str, message, timestamp: int) -> None:
        recorder = self._recorder
        if recorder is not None:
            try:
                recorder.append(topic, message, timestamp)
            except Exception as error:
                self._fail(f"RECORDING_ERROR: {error}")

    def _fail(self, reason: str) -> None:
        if not self.failure_reason:
            self.failure_reason = reason
        self._stop_requested.set()
        with self._condition:
            self._condition.notify_all()

    def _on_info(self, message: BackendInfo) -> None:
        if self._run_context is not None and (
            context_from_message(message.context).generation != self._run_context.generation
        ):
            self._fail("STALE_CONTEXT: backend restarted")
        with self._condition:
            self._info = message
            self._condition.notify_all()
        self._record(f"{self._root}/info", message, self.get_clock().now().nanoseconds)

    def _on_state(self, message: RobotState) -> None:
        self._record(f"{self._root}/state", message, time_ns(message.stamp))
        self._buffer.add(
            "state",
            context_from_message(message.context),
            message.state_seq,
            time_ns(message.stamp),
        )

    def _on_frame(self, message: CameraFrame) -> None:
        self._record(self._camera_topic, message, time_ns(message.image.header.stamp))
        self._buffer.add(
            self.config.profile.camera.name,
            context_from_message(message.context),
            message.frame_seq,
            time_ns(message.image.header.stamp),
        )

    def _on_event(self, message: CommandEvent) -> None:
        self._record(f"{self._root}/command_events", message, time_ns(message.stamp))
        if (
            self._run_context is not None
            and context_from_message(message.context) == self._run_context
        ):
            if message.phase in ("COMPLETED", "CANCELED", "FAILED"):
                with self._condition:
                    self._terminal_events.add(message.command_id)
                    self._condition.notify_all()

    def _publish_observation(self) -> None:
        if self._run_context is None or not self._lease_id:
            return
        try:
            observation = self._buffer.assemble(self.get_clock().now().nanoseconds)
        except ContractError:
            if self._last_observation is not None and (
                self.get_clock().now().nanoseconds - self._last_observation.state.stamp_ns
                > self.config.observation["max_age_ns"]
            ):
                self._fail("STALE_OBSERVATION")
            return
        if observation.observation_id != self._last_index_id:
            message = ObservationIndex(
                context=context_message(observation.context),
                observation_id=observation.observation_id,
                state_seq=observation.state.sequence,
                camera_names=list(observation.frames),
                frame_seqs=[sample.sequence for sample in observation.frames.values()],
                reference_time=stamp(observation.state.stamp_ns),
            )
            self._record("/runtime/observation_index", message, observation.state.stamp_ns)
            self._indices.publish(message)
            self._last_index_id = observation.observation_id
        with self._condition:
            self._last_observation = observation
            self._condition.notify_all()

    def _renew_lease(self) -> None:
        with self._renew_lock:
            if not self._renew_enabled:
                return
            if not self._lease_id or self._run_context is None:
                return
            if self._renew_future is not None:
                if self._renew_future.done():
                    try:
                        response = self._renew_future.result()
                        if not response.success:
                            self._fail(response.reason)
                    except Exception as error:
                        self._fail(f"LEASE_RENEWAL_FAILED: {error}")
                    self._renew_future = None
                elif time.monotonic_ns() - self._last_renew_ns > self._recipe["lease_duration_ns"]:
                    self._fail("LEASE_RENEWAL_TIMEOUT")
                return
            if time.monotonic_ns() - self._last_renew_ns >= self._recipe["renew_interval_ns"]:
                self._last_renew_ns = time.monotonic_ns()
                self._renew_future = self._service_clients["renew_control"].call_async(
                    RenewControl.Request(
                        context=context_message(self._run_context),
                        lease_id=self._lease_id,
                        duration=duration(self._recipe["lease_duration_ns"]),
                    )
                )

    def _wait_future(self, future, timeout: float, *, abort_on_stop: bool = True):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise ContractError("CALL_TIMEOUT")
            if not rclpy.ok() or (abort_on_stop and self._stop_requested.is_set()):
                raise ContractError("EPISODE_STOPPED", self.failure_reason)
            time.sleep(0.01)
        return future.result()

    def _call(self, name: str, request, *, cleanup: bool = False):
        client = self._service_clients[name]
        if not client.wait_for_service(timeout_sec=self._recipe["call_timeout_s"]):
            raise ContractError("NOT_READY", name)
        response = self._wait_future(
            client.call_async(request),
            self._recipe["call_timeout_s"],
            abort_on_stop=not cleanup,
        )
        if not response.success:
            raise ContractError(response.reason)
        return response

    def _wait_ready(self) -> BackendInfo:
        deadline = time.monotonic() + self._recipe["startup_timeout_s"]
        with self._condition:
            while self._info is None or not self._info.ready:
                if time.monotonic() >= deadline or self._stop_requested.is_set():
                    raise ContractError("NOT_READY", "Backend info")
                self._condition.wait(timeout=0.1)
            info = self._info
        if not info.test_only or info.backend_kind != "mock":
            raise ContractError("UNSUPPORTED_CAPABILITY", "Stage 2 runtime is test-only")
        if info.context.robot_profile_hash != self.config.profile.digest:
            raise ContractError("SCHEMA_MISMATCH", "Backend profile")
        if tuple(info.joint_names) != self.config.profile.joint_names:
            raise ContractError("SCHEMA_MISMATCH", "Backend joints")
        if list(info.camera_names) != [self.config.profile.camera.name]:
            raise ContractError("SCHEMA_MISMATCH", "Backend cameras")
        if not self._action.wait_for_server(timeout_sec=self._recipe["call_timeout_s"]):
            raise ContractError("NOT_READY", "Trajectory action")
        return info

    def _wait_observation(self, after_id: int = 0) -> ObservationRef:
        deadline = time.monotonic() + self._recipe["call_timeout_s"]
        with self._condition:
            while (
                self._last_observation is None
                or self._last_observation.observation_id <= after_id
                or not 0
                <= self.get_clock().now().nanoseconds - self._last_observation.state.stamp_ns
                <= self.config.observation["max_age_ns"]
            ):
                if self._stop_requested.is_set() or time.monotonic() >= deadline:
                    raise ContractError("NOT_READY", "Fresh synchronized observation")
                self._condition.wait(timeout=0.05)
            return self._last_observation

    def _topics(self) -> dict[str, str]:
        return {
            f"{self._root}/info": "robot_api_msgs/msg/BackendInfo",
            f"{self._root}/state": "robot_api_msgs/msg/RobotState",
            self._camera_topic: "robot_api_msgs/msg/CameraFrame",
            f"{self._root}/command_events": "robot_api_msgs/msg/CommandEvent",
            "/runtime/control/commands": "robot_api_msgs/msg/TrajectoryCommand",
            "/runtime/observation_index": "robot_api_msgs/msg/ObservationIndex",
        }

    def _start_service(self, request, response):
        try:
            response.episode_id = self._start(request.task_id)
            response.success, response.reason = True, "PREPARING"
        except ContractError as error:
            response.success, response.reason = False, error.reason
        return response

    def _end_service(self, request, response):
        if self._worker is None or not self._worker.is_alive():
            response.success, response.reason = False, "NO_ACTIVE_EPISODE"
        else:
            self._fail(request.reason or "EPISODE_ENDED_EARLY")
            response.success, response.reason = True, "ENDING"
        return response

    def _auto_start(self) -> None:
        self._auto_timer.cancel()
        self._start("mock-script")

    def _start(self, task_id: str) -> str:
        with self._start_lock:
            if self._worker is not None and self._worker.is_alive():
                raise ContractError("CONTROL_BUSY")
            episode_id = f"episode-{uuid.uuid4()}"
            self.failure_reason, self.record_directory = "", ""
            self._stop_requested.clear()
            self.done.clear()
            self._terminal_events.clear()
            self._last_observation = None
            self._last_index_id = 0
            self._worker = threading.Thread(
                target=self._run_episode,
                args=(episode_id, task_id),
                daemon=True,
            )
            self._worker.start()
            return episode_id

    def _run_episode(self, episode_id: str, task_id: str) -> None:
        self._set_status("PREPARING", episode_id)
        try:
            info = self._wait_ready()
            self._run_context = replace(
                context_from_message(info.context),
                episode_id=episode_id,
                control_epoch=1,
            )
            self._buffer.set_context(self._run_context)
            snapshot = {
                **self.config.snapshot,
                "task_id": task_id,
                "run_context": asdict(self._run_context),
            }
            self._recorder = EpisodeRecorder(
                self.output,
                episode_id,
                snapshot,
                self.config.schema_root,
                self._topics(),
            )
            self._record(f"{self._root}/info", info, self.get_clock().now().nanoseconds)
            response = self._call(
                "acquire_control",
                AcquireControl.Request(
                    context=context_message(self._run_context),
                    owner_id=self.get_fully_qualified_name(),
                    duration=duration(self._recipe["lease_duration_ns"]),
                ),
            )
            with self._renew_lock:
                self._lease_id = response.lease_id
                self._last_renew_ns = time.monotonic_ns()
                self._renew_enabled = True
            self._set_status("ACTIVE", episode_id)
            command_ids = []
            previous_observation_id = 0
            for sequence, target in enumerate(self._recipe["script_targets"], start=1):
                observation = self._wait_observation(previous_observation_id)
                previous_observation_id = observation.observation_id
                command = Trajectory(
                    self._run_context,
                    self._lease_id,
                    f"{episode_id}-{sequence}",
                    sequence,
                    observation.observation_id,
                    self.get_clock().now().nanoseconds + self._recipe["command_acceptance_ns"],
                    self.config.profile.joint_names,
                    (Target(target["duration_ns"], tuple(float(x) for x in target["positions"])),),
                )
                message = command_message(command)
                self._record(
                    "/runtime/control/commands",
                    message,
                    self.get_clock().now().nanoseconds,
                )
                self._commands.publish(message)
                handle = self._wait_future(
                    self._action.send_goal_async(ExecuteTrajectory.Goal(command=message)),
                    self._recipe["call_timeout_s"],
                )
                if not handle.accepted:
                    raise ContractError("COMMAND_REJECTED")
                result = self._wait_future(
                    handle.get_result_async(),
                    self._recipe["command_timeout_s"],
                ).result
                if not result.success:
                    raise ContractError(result.reason or result.phase)
                command_ids.append(command.command_id)
            deadline = time.monotonic() + self._recipe["call_timeout_s"]
            with self._condition:
                while not set(command_ids).issubset(self._terminal_events):
                    if self._stop_requested.is_set() or time.monotonic() >= deadline:
                        raise ContractError("RECORDING_ERROR", "Missing terminal events")
                    self._condition.wait(timeout=0.05)
        except Exception as error:
            self._fail(str(error))
            self.get_logger().error(self.failure_reason)
        finally:
            self._set_status("ENDING", episode_id)
            self._cleanup_control()
            recorder, self._recorder = self._recorder, None
            if recorder is not None:
                try:
                    path = recorder.finish(self.failure_reason)
                    self.record_directory = str(path)
                    manifest = json.loads((path / "manifest.json").read_text())
                    if not manifest["complete"]:
                        self.failure_reason = manifest["failure_reason"]
                except Exception as error:
                    self.failure_reason = f"RECORDING_ERROR: {error}"
            self._run_context = None
            self._set_status(
                "FAULT" if self.failure_reason else "IDLE", episode_id, self.failure_reason
            )
            print(
                json.dumps(
                    {
                        "episode_id": episode_id,
                        "complete": not self.failure_reason,
                        "directory": self.record_directory,
                        "reason": self.failure_reason,
                    }
                ),
                flush=True,
            )
            self.done.set()

    def _cleanup_control(self) -> None:
        with self._renew_lock:
            self._renew_enabled = False
            pending_renewal, self._renew_future = self._renew_future, None
        if self._run_context is None or not self._lease_id:
            return
        try:
            if pending_renewal is not None:
                self._wait_future(
                    pending_renewal,
                    self._recipe["call_timeout_s"],
                    abort_on_stop=False,
                )
            self._call(
                "stop",
                Stop.Request(
                    context=context_message(self._run_context),
                    lease_id=self._lease_id,
                    reason="EPISODE_END",
                ),
                cleanup=True,
            )
            self._call(
                "release_control",
                ReleaseControl.Request(
                    context=context_message(self._run_context),
                    lease_id=self._lease_id,
                ),
                cleanup=True,
            )
        except Exception as error:
            self._fail(f"CONTROL_CLEANUP_FAILED: {error}")
        finally:
            self._lease_id = ""
            self._renew_future = None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=Path("/workspace"))
    parser.add_argument(
        "--config", type=Path, default=Path("/workspace/configs/runtime/mock_collect.yaml")
    )
    parser.add_argument("--output", type=Path, default=Path("/data/raw"))
    parser.add_argument("--once", action="store_true")
    args, ros_args = parser.parse_known_args()
    configuration = load_configuration(args.project_root, args.config)
    rclpy.init(args=ros_args)
    node = RuntimeNode(configuration, args.output, auto_start=args.once)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not (args.once and node.done.is_set()):
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        node._stop_requested.set()
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if node.failure_reason:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
