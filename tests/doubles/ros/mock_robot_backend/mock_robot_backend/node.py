"""ROS adapter for the deterministic backend fixture."""

import argparse
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.task import Future
from robot_api_msgs.action import ExecuteTrajectory
from robot_api_msgs.msg import BackendInfo, CameraFrame, CommandEvent, CommandStatus, RobotState
from robot_api_msgs.srv import (
    AcquireControl,
    QueryCommand,
    ReleaseControl,
    RenewControl,
    Stop,
)
from robot_backend_core.engine import TERMINAL_PHASES, BackendEngine
from robot_contracts.models import ContractError, Trajectory
from robot_contracts.profiles import load_profile
from robot_ros_support.conversions import (
    command_from_message,
    command_status_message,
    context_from_message,
    context_message,
    stamp,
    time_ns,
    trajectory_message,
)
from robot_ros_support.qos import EVENT_QOS, INFO_QOS, SENSOR_QOS
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Header


@dataclass
class PendingGoal:
    handle: ServerGoalHandle
    command: Trajectory
    future: Future


class MockBackendNode(Node):
    """Publish synthetic measurements and serve the same public control API."""

    def __init__(self, profile_path: Path, schema_root: Path, namespace: str) -> None:
        super().__init__("mock_backend")
        self.profile = load_profile(profile_path, schema_root)
        if not self.profile.test_only:
            raise ContractError(
                "UNSUPPORTED_CAPABILITY",
                "mock backend requires a test_only=true robot profile",
            )
        self.engine = BackendEngine(self.profile)
        self._sequence = 0
        self._goal_waiters: dict[bytes, PendingGoal] = {}
        self._root = namespace.rstrip("/")
        self._group = ReentrantCallbackGroup()
        self._tick_group = MutuallyExclusiveCallbackGroup()
        self._info = self.create_publisher(BackendInfo, f"{self._root}/info", INFO_QOS)
        self._state = self.create_publisher(RobotState, f"{self._root}/state", SENSOR_QOS)
        self._cameras = {
            camera.name: self.create_publisher(
                CameraFrame,
                f"{self._root}/cameras/{camera.name}/frame",
                SENSOR_QOS,
            )
            for camera in self.profile.cameras
        }
        self._events = self.create_publisher(
            CommandEvent,
            f"{self._root}/command_events",
            EVENT_QOS,
        )
        self._feedback = self.create_publisher(
            CommandStatus,
            f"{self._root}/command_feedback",
            EVENT_QOS,
        )
        for service_type, name, callback in (
            (AcquireControl, "acquire_control", self._acquire),
            (RenewControl, "renew_control", self._renew),
            (ReleaseControl, "release_control", self._release),
            (Stop, "stop", self._stop),
            (QueryCommand, "query_command", self._query),
        ):
            self.create_service(
                service_type,
                f"{self._root}/{name}",
                callback,
                callback_group=self._group,
            )
        self._action = ActionServer(
            self,
            ExecuteTrajectory,
            f"{self._root}/execute_trajectory",
            execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self._group,
        )
        self.create_timer(0.05, self._tick, callback_group=self._tick_group)
        self.create_timer(0.5, self._publish_info)
        self._publish_info()

    def _publish_info(self) -> None:
        context, _ = self.engine.snapshot()
        self._info.publish(
            BackendInfo(
                protocol_major=1,
                context=context_message(context),
                backend_kind="mock",
                ready=True,
                test_only=True,
                joint_names=list(self.profile.joint_names),
                joint_units=[joint.unit for joint in self.profile.joints],
                camera_names=list(self.profile.camera_names),
                capabilities=[
                    "joint_position",
                    "lease",
                    "stop",
                    "synthetic_rgb",
                    "command_query",
                    "command_feedback_v1",
                ],
            )
        )

    @staticmethod
    def _service_result(response, operation):
        try:
            operation()
            response.success, response.reason = True, ""
        except ContractError as error:
            response.success, response.reason = False, error.reason
        return response

    def _robot_now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _acquire(self, request, response):
        def operation() -> None:
            response.lease_id = self.engine.acquire(
                context_from_message(request.context),
                request.owner_id,
                time_ns(request.duration),
                time.monotonic_ns(),
                robot_now_ns=self._robot_now_ns(),
            )

        return self._service_result(response, operation)

    def _renew(self, request, response):
        return self._service_result(
            response,
            lambda: self.engine.renew(
                context_from_message(request.context),
                request.lease_id,
                time_ns(request.duration),
                time.monotonic_ns(),
                robot_now_ns=self._robot_now_ns(),
            ),
        )

    def _release(self, request, response):
        return self._service_result(
            response,
            lambda: self.engine.release(
                context_from_message(request.context),
                request.lease_id,
                time.monotonic_ns(),
                robot_now_ns=self._robot_now_ns(),
            ),
        )

    def _stop(self, request, response):
        return self._service_result(
            response,
            lambda: self.engine.stop(
                context_from_message(request.context),
                request.lease_id,
                time.monotonic_ns(),
                request.reason,
                robot_now_ns=self._robot_now_ns(),
            ),
        )

    def _query(self, request, response):
        def operation() -> None:
            response.status = command_status_message(
                self.engine.query_command(
                    context_from_message(request.context),
                    request.command_id,
                )
            )

        return self._service_result(response, operation)

    async def _execute(self, handle):
        result = ExecuteTrajectory.Result(command_id=handle.request.command.command_id)
        try:
            command = command_from_message(handle.request.command)
            self.engine.submit(command, self.get_clock().now().nanoseconds, time.monotonic_ns())
        except ContractError as error:
            result.success, result.phase, result.reason = False, "REJECTED", error.reason
            handle.abort()
            return result
        key = bytes(handle.goal_id.uuid)
        future = Future()
        self._goal_waiters[key] = PendingGoal(handle, command, future)
        try:
            return await future
        finally:
            self._goal_waiters.pop(key, None)

    def _update_goals(self, robot_now_ns: int) -> None:
        """Resolve action futures from the tick; no blocking callbacks or polling threads."""
        for pending in list(self._goal_waiters.values()):
            if pending.future.done():
                continue
            handle, command = pending.handle, pending.command
            try:
                phase, reason = self.engine.execution_status(command.command_id)
                if phase not in TERMINAL_PHASES and handle.is_cancel_requested:
                    self.engine.cancel(
                        command.context,
                        command.lease_id,
                        command.command_id,
                        time.monotonic_ns(),
                        robot_now_ns=robot_now_ns,
                    )
                    phase, reason = self.engine.execution_status(command.command_id)
            except ContractError as error:
                phase, reason = "FAILED", error.reason
            if phase in TERMINAL_PHASES:
                result = ExecuteTrajectory.Result(
                    success=phase == "COMPLETED",
                    command_id=command.command_id,
                    phase=phase,
                    reason=reason,
                )
                if phase == "COMPLETED":
                    handle.succeed()
                elif handle.is_cancel_requested:
                    handle.canceled()
                else:
                    handle.abort()
                pending.future.set_result(result)
            else:
                _, positions = self.engine.snapshot()
                handle.publish_feedback(
                    ExecuteTrajectory.Feedback(
                        phase=phase,
                        state=JointState(
                            name=list(self.profile.joint_names),
                            position=list(positions),
                        ),
                    )
                )

    def _publish_feedback(self, status) -> None:
        self._feedback.publish(command_status_message(status))

    def _tick(self) -> None:
        robot_now_ns = self._robot_now_ns()
        self.engine.tick(time.monotonic_ns(), robot_now_ns)
        self._update_goals(robot_now_ns)
        context, positions = self.engine.snapshot()
        now = self.get_clock().now().to_msg()
        self._sequence += 1
        self._state.publish(
            RobotState(
                context=context_message(context),
                state_seq=self._sequence,
                stamp=now,
                joints=JointState(
                    header=Header(stamp=now),
                    name=list(self.profile.joint_names),
                    position=list(positions),
                ),
                status="READY",
            )
        )
        for camera_index, camera in enumerate(self.profile.cameras):
            red = int((positions[0] + 1) * 100) % 256 if camera_index == 0 else 17 * camera_index
            green = (self._sequence + 31 * camera_index) % 256
            blue = 128 if camera_index == 0 else 43 * camera_index
            image = Image(
                header=Header(stamp=now, frame_id=camera.frame_id),
                height=camera.height,
                width=camera.width,
                encoding="rgb8",
                step=camera.width * 3,
                data=bytes([red, green, blue]) * camera.width * camera.height,
            )
            center_x = (camera.width - 1) / 2.0
            center_y = (camera.height - 1) / 2.0
            info = CameraInfo(
                header=image.header,
                width=camera.width,
                height=camera.height,
                distortion_model="plumb_bob",
                d=[0.0] * 5,
                k=[50.0, 0.0, center_x, 0.0, 50.0, center_y, 0.0, 0.0, 1.0],
            )
            self._cameras[camera.name].publish(
                CameraFrame(
                    context=context_message(context),
                    frame_seq=self._sequence,
                    image=image,
                    camera_info=info,
                    calibration_digest=camera.calibration_digest or "mock-calibration-v1",
                )
            )
        terminals: list[tuple[object, str]] = []
        for event in self.engine.drain_events():
            self._events.publish(
                CommandEvent(
                    context=context_message(event.context),
                    event_seq=event.sequence,
                    stamp=stamp(event.stamp_ns),
                    command_id=event.command.command_id,
                    command_seq=event.command.command_seq,
                    phase=event.phase,
                    reason=event.reason,
                    trajectory=trajectory_message(event.command),
                )
            )
            if event.phase in TERMINAL_PHASES:
                terminals.append((event.context, event.command.command_id))
        active = self.engine.active_status()
        if active is not None:
            self._publish_feedback(active)
        for terminal_context, command_id in terminals:
            self._publish_feedback(self.engine.query_command(terminal_context, command_id))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--schemas", type=Path, required=True)
    parser.add_argument("--namespace", default="/cell_01/robot")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = MockBackendNode(args.profile, args.schemas, args.namespace)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except _rclpy.RCLError:
        # A ROS signal can invalidate the context during a publish callback.
        # Errors from a live context must still propagate as real failures.
        if node.context.ok():
            raise
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
