"""ROS 2 adapter for the runnable MuJoCo backend.

This module owns ROS entities, wire conversions, publishers/services and node
lifecycle only.  Physics, planning and rendering stay on the worker threads;
service callbacks await the worker's explicit ``Future`` through a guard/timer
bridge instead of blocking the single-threaded executor.  Process lifecycle lives
in :mod:`robot_backend_sim.backend_cli`.
"""

from __future__ import annotations

import math
import threading
from concurrent.futures import Future
from dataclasses import replace
from typing import Any

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from robot_api_msgs.msg import (
    BackendInfo,
    CameraFrame,
    ControlEvent,
    ExecutionFeedback,
    FollowerReference,
    RobotState,
)
from robot_api_msgs.msg import TaskState as TaskStateMessage
from robot_api_msgs.srv import (
    AcquireControl,
    GetStreamAnchor,
    GetTaskState,
    GetTeacherChunk,
    PrepareTask,
    QueryStreamCommand,
    ReleaseControl,
    RenewControl,
    ResetSimulation,
    StartTask,
    Stop,
    SubmitFollower,
)
from robot_contracts.execution_feedback import ExecutionFeedback as ExecutionFeedbackRecord
from robot_contracts.models import CameraSpec, ContractError, RunContext
from robot_ros_support.conversions import context_message, stamp
from robot_ros_support.follower_conversions import follower_reference_message
from robot_ros_support.qos import EVENT_QOS, INFO_QOS, RELIABLE_SENSOR_QOS, SENSOR_QOS
from robot_ros_support.stream_conversions import (
    control_event_message,
    execution_feedback_message,
)
from robot_run_logging import EventContext, EventLogger, LogFailure
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Float64MultiArray, Header

from .backend_config import BackendConfig
from .control_state import ControlStatus
from .node_services import ServiceAdapter
from .protocol import control_event, execution_feedback
from .render_worker import RenderedCapture, RendererFactory
from .ros_bridge import RosFutureBridge
from .scene_config import SceneConfig
from .tasks.ros_conv import task_state_message
from .worker import BackendClosed, Worker, WorkerSummary

__all__ = ["BackendNode", "main"]

_CAPABILITIES = (
    "continuous_chunks",
    "exact_capture",
    "simulation_reset",
    "lease",
    "stop",
    "command_query",
    "control_events",
    "follower_reference",
)
_TASK_CAPABILITIES = ("task_plugin", "task_state")
_TEACHER_CAPABILITY = "teacher_chunk"
_FOLLOWER_CAPABILITY = "bounded_target_follower_v1"
_EXIT_OK = 0
_EXIT_FAILURE = 1


def _mount_calibration(
    scene_config: SceneConfig, camera: CameraSpec
) -> tuple[float, float, float, float]:
    """Return (fx, fy, cx, cy) from the scene's declared fovy for this camera."""
    scene_camera = next(
        item
        for item in scene_config.cameras
        if item.name == camera.name  # type: ignore[attr-defined]
    )
    width = int(camera.width)  # type: ignore[attr-defined]
    height = int(camera.height)  # type: ignore[attr-defined]
    fovy = math.radians(float(scene_camera.fovy))
    fy = height / (2.0 * math.tan(fovy / 2.0))
    return fy, fy, (width - 1) / 2.0, (height - 1) / 2.0


class BackendNode(Node):
    """Publish exact captures and expose the shared service/topic API."""

    def __init__(
        self,
        config: BackendConfig,
        *,
        log: EventLogger | None = None,
        renderer_factory: RendererFactory | None = None,
    ) -> None:
        super().__init__("backend")
        self.config = config
        self._log = log
        self._root = config.namespace.rstrip("/")
        self._group = ReentrantCallbackGroup()
        self._bridges: set[RosFutureBridge] = set()
        self._bridges_lock = threading.Lock()
        self._stop_requested = False
        self._exit_reason = ""
        self._exit_code = _EXIT_OK
        self._unload_active = False
        self._last_failed_command = ""
        self._worker = Worker(
            config,
            log=log,
            renderer_factory=renderer_factory,
        )
        try:
            self._worker.start()
            self._setup_ros()
        except BaseException:
            self._close_after_start_failure()
            raise

    def _setup_ros(self) -> None:
        self.profile = self._worker.profile
        self._calibrations = {
            camera.name: _mount_calibration(self._worker.scene_config, camera)
            for camera in self.profile.cameras
        }
        self._info_pub = self.create_publisher(BackendInfo, f"{self._root}/info", INFO_QOS)
        sensor_qos = RELIABLE_SENSOR_QOS if self.config.reliable_sensors else SENSOR_QOS
        self._state_pub = self.create_publisher(RobotState, f"{self._root}/state", sensor_qos)
        self._visual_state_pub = self.create_publisher(
            Float64MultiArray, f"{self._root}/visualization/mujoco_state", SENSOR_QOS
        )
        self._camera_pubs = {
            camera.name: self.create_publisher(
                CameraFrame, f"{self._root}/cameras/{camera.name}/frame", sensor_qos
            )
            for camera in self.profile.cameras
        }
        self._events_pub = self.create_publisher(
            ControlEvent, f"{self._root}/control_events", EVENT_QOS
        )
        self._feedback_pub = self.create_publisher(
            ExecutionFeedback, f"{self._root}/execution_feedback", EVENT_QOS
        )
        self._follower_pub = self.create_publisher(
            FollowerReference, f"{self._root}/follower_references", EVENT_QOS
        )
        self._task_state_pub = self.create_publisher(
            TaskStateMessage, f"{self._root}/task_state", EVENT_QOS
        )
        self._last_task_state_seq = 0
        self._last_task_state_phase = ""
        self._quality_lost_since_ns: dict[tuple[str, str], int] = {}
        self._quality_max_blackout_ns: dict[tuple[str, str], int] = {}
        self._service_adapter = ServiceAdapter(self, self._worker, self.config)
        for service_type, name, callback in (
            (AcquireControl, "acquire_control", self._service_adapter.acquire),
            (RenewControl, "renew_control", self._service_adapter.renew),
            (ReleaseControl, "release_control", self._service_adapter.release),
            (Stop, "stop", self._service_adapter.stop),
            (SubmitFollower, "submit_follower", self._service_adapter.submit_follower),
            (GetStreamAnchor, "get_stream_anchor", self._service_adapter.get_anchor),
            (QueryStreamCommand, "query_stream_command", self._service_adapter.query_command),
            (ResetSimulation, "reset_simulation", self._service_adapter.reset),
            (PrepareTask, "prepare_task", self._service_adapter.prepare_task),
            (StartTask, "start_task", self._service_adapter.start_task),
            (GetTeacherChunk, "get_task_chunk", self._service_adapter.get_task_chunk),
            (GetTaskState, "get_task_state", self._service_adapter.get_task_state),
        ):
            self.create_service(
                service_type,
                f"{self._root}/{name}",
                callback,
                callback_group=self._group,
            )
        self.create_timer(0.02, self._publish_pending, callback_group=self._group)
        self.create_timer(0.5, self._publish_info, callback_group=self._group)
        self._publish_info()

    def _close_after_start_failure(self) -> None:
        """Bounded cleanup after worker.start succeeded but ROS setup failed."""
        with self._bridges_lock:
            bridges = tuple(self._bridges)
            self._bridges.clear()
        for bridge in bridges:
            bridge.abort()
        try:
            self._worker.close("NODE_START_FAILED", self.config.shutdown_timeout_s)
        except Exception:  # noqa: BLE001 - cleanup must not mask the startup error
            pass
        try:
            self.destroy_node()
        except Exception:  # noqa: BLE001 - keep the original startup exception
            pass

    # -------------------------------------------------------------- lifecycle
    @property
    def should_stop(self) -> bool:
        return self._stop_requested

    @property
    def exit_code(self) -> int:
        return self._exit_code

    @property
    def exit_reason(self) -> str:
        return self._exit_reason

    def request_stop(self, reason: str, *, failed: bool = False) -> None:
        if self._stop_requested:
            return
        self._stop_requested = True
        self._exit_reason = reason
        self._exit_code = _EXIT_FAILURE if failed else _EXIT_OK

    def close(self, reason: str) -> tuple[WorkerSummary, str | None]:
        with self._bridges_lock:
            bridges = tuple(self._bridges)
            self._bridges.clear()
        for bridge in bridges:
            bridge.abort()
        summary = self._worker.close(reason, self.config.shutdown_timeout_s)
        # Braking can produce final facts while the ROS loop is no longer
        # pumping. Deliver them before publishers and the logger are closed.
        self._publish_control_pending()
        if self._worker.fault is not None:
            summary = replace(summary, closed=False, fault=self._worker.fault)
        if summary.fault is not None:
            self._exit_code = _EXIT_FAILURE
            self._exit_reason = summary.fault.reason
        elif not summary.closed and self._exit_code == _EXIT_OK:
            self._exit_code = _EXIT_FAILURE
            self._exit_reason = "CLOSE_INCOMPLETE"
        return summary, self._exit_reason

    def _emit(self, event: str, **kwargs: object) -> None:
        if self._log is None:
            return
        try:
            self._log.emit(event, **kwargs)
        except LogFailure as error:
            self._worker.latch_fault("LOGGER_FAILURE", str(error))
            self.request_stop("LOGGER_FAILURE", failed=True)

    def _run_context(self) -> RunContext:
        context = self._worker.current_context
        if not isinstance(context, RunContext):
            raise ContractError("INVALID_CONTEXT", "worker has no RunContext")
        return context

    # ---------------------------------------------------------------- publish
    def _publish_info(self) -> None:
        fault = self._worker.fault
        context = self._run_context()
        self._info_pub.publish(
            BackendInfo(
                protocol_major=1,
                context=context_message(context),
                backend_kind="mujoco",
                ready=fault is None and not self._stop_requested,
                test_only=False,
                joint_names=list(self.profile.joint_names),
                joint_units=[joint.unit for joint in self.profile.joints],
                camera_names=list(self.profile.camera_names),
                capabilities=list(self._capabilities()),
            )
        )

    def _capabilities(self) -> tuple[str, ...]:
        capabilities = list(_CAPABILITIES)
        if self.config.reliable_sensors:
            capabilities.append("reliable_sensor_streams_v1")
        if self._worker.follower_config is None:
            capabilities.remove("follower_reference")
        else:
            capabilities.append(_FOLLOWER_CAPABILITY)
        if self._worker.task_enabled:
            capabilities.extend(_TASK_CAPABILITIES)
        if self._worker.teacher_enabled:
            capabilities.append(_TEACHER_CAPABILITY)
        return tuple(capabilities)

    def _publish_pending(self) -> None:
        fault = self._worker.fault
        if fault is not None:
            if not self._stop_requested:
                self._emit(
                    "backend.fault",
                    level="ERROR",
                    reason=fault.reason,
                    message=fault.detail,
                )
                try:
                    self._worker.system_stop(fault.reason)
                except Exception:
                    pass
            self.request_stop(fault.reason, failed=True)
        for capture in self._worker.captures():
            self._publish_state(capture)
            self._publish_visual_state(capture)
        for rendered in self._worker.rendered():
            self._publish_frames(rendered)
        self._publish_task_state()
        self._publish_control_pending()

    def _publish_control_pending(self) -> None:
        for status in self._worker.feedback():
            self._publish_feedback(status)
        for event in self._worker.control_events():
            try:
                self._events_pub.publish(control_event_message(control_event(event)))
                self._emit(
                    "backend.control",
                    message=event.phase,
                    reason=event.reason or None,
                    context=EventContext(
                        episode_id=event.context.episode_id or None,
                        command_id=event.command_id,
                        clock_epoch=event.context.clock_epoch,
                        sim_time_ns=event.sim_time_ns,
                    ),
                    fields={
                        "command_id": event.command_id,
                        "backend_instance_id": event.context.backend_instance_id,
                        "control_epoch": event.context.control_epoch,
                        "control_event_seq": event.event_seq,
                        "scope": event.scope,
                        "command_seq": event.command_seq,
                        "observation_id": event.observation_id,
                        "chunk_seq": event.chunk_seq,
                    },
                )
            except Exception as error:  # noqa: BLE001 - a control fact must not vanish
                self._worker.latch_fault("EVENT_PUBLISH_FAILED", f"{type(error).__name__}: {error}")
        for follower in self._worker.follower_references():
            try:
                self._follower_pub.publish(follower_reference_message(follower))
            except Exception as error:  # noqa: BLE001 - invalid reference is a hard fault
                self._worker.latch_fault(
                    "FOLLOWER_REFERENCE_PUBLISH_FAILED",
                    f"{type(error).__name__}: {error}",
                )

    def _publish_state(self, capture: object) -> None:
        stamp_msg = stamp(capture.sim_time_ns)  # type: ignore[attr-defined]
        self._state_pub.publish(
            RobotState(
                context=context_message(capture.context),  # type: ignore[attr-defined]
                state_seq=capture.capture_seq,  # type: ignore[attr-defined]
                stamp=stamp_msg,
                joints=JointState(
                    header=Header(stamp=stamp_msg),
                    name=list(capture.joint_names),  # type: ignore[attr-defined]
                    position=list(capture.positions),  # type: ignore[attr-defined]
                    velocity=list(capture.velocities),  # type: ignore[attr-defined]
                ),
                status=capture.phase,  # type: ignore[attr-defined]
                fault_reason="" if self._worker.fault is None else self._worker.fault.reason,
            )
        )

    def _publish_visual_state(self, capture: object) -> None:
        """Publish one read-only full-physics snapshot for the optional viewer.

        The normal RobotState API remains the contract path.  This debug topic is
        best-effort and never feeds control or task evaluation.
        """
        try:
            state = capture.snapshot.state  # type: ignore[attr-defined]
            self._visual_state_pub.publish(Float64MultiArray(data=state.tolist()))
        except Exception as error:  # noqa: BLE001 - visualization must not fail control
            self._emit(
                "backend.visualization.publish_failed",
                level="WARNING",
                reason="VISUALIZATION_PUBLISH_FAILED",
                message=f"{type(error).__name__}: {error}",
            )

    def _publish_task_state(self) -> None:
        state = self._worker.task_state
        if state is None or state.context is None or state.state_seq == self._last_task_state_seq:
            return
        self._last_task_state_seq = state.state_seq
        if state.phase != self._last_task_state_phase:
            self._last_task_state_phase = state.phase
            self._emit(
                "backend.task.state",
                message=state.phase,
                reason=state.failure_reason.partition(":")[0] or None,
                context=EventContext(
                    episode_id=state.context.episode_id,
                    clock_epoch=state.context.clock_epoch,
                    sim_time_ns=state.sample_time_ns,
                ),
                fields={
                    "task_type": state.task_type,
                    "task_epoch": state.task_epoch,
                    "succeeded": state.succeeded,
                    "terminal": state.terminal,
                    "task_reason": state.failure_reason,
                },
            )
        try:
            self._task_state_pub.publish(task_state_message(state))
        except Exception as error:  # noqa: BLE001 - task truth must not vanish silently
            self._worker.latch_fault(
                "TASK_STATE_PUBLISH_FAILED", f"{type(error).__name__}: {error}"
            )

    def _publish_frames(self, rendered: RenderedCapture) -> None:
        capture = rendered.capture
        stamp_msg = stamp(capture.sim_time_ns)
        for camera in self.profile.cameras:
            frame = rendered.frames.get(camera.name)
            if frame is None:
                self._worker.latch_fault(
                    "RENDER_FRAME_MISSING", f"renderer produced no frame for {camera.name}"
                )
                continue
            header = Header(stamp=stamp_msg, frame_id=camera.frame_id)
            image = Image(
                header=header,
                height=camera.height,
                width=camera.width,
                encoding="rgb8",
                is_bigendian=0,
                step=camera.width * 3,
                data=frame.tobytes(order="C"),
            )
            fx, fy, cx, cy = self._calibrations[camera.name]
            info = CameraInfo(
                header=header,
                width=camera.width,
                height=camera.height,
                distortion_model="plumb_bob",
                d=[0.0] * 5,
                k=[fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
                p=[fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
            )
            self._camera_pubs[camera.name].publish(
                CameraFrame(
                    context=context_message(capture.context),
                    frame_seq=capture.capture_seq,
                    image=image,
                    camera_info=info,
                    calibration_digest=camera.calibration_digest,
                )
            )
        self._emit_quality(rendered)

    def _emit_quality(self, rendered: RenderedCapture) -> None:
        quality = rendered.quality
        if not quality:
            return
        capture = rendered.capture
        fields: dict[str, object] = {
            "capture_seq": capture.capture_seq,
            "phase": capture.phase,
        }
        for camera, counts in quality.items():
            for name, pixels in counts.items():
                fields[f"{camera}_{name}_pixels"] = int(pixels)
                key = (camera, name)
                if pixels <= 0:
                    started = self._quality_lost_since_ns.setdefault(key, capture.sim_time_ns)
                    fields[f"{camera}_{name}_blackout_s"] = round(
                        (capture.sim_time_ns - started) / 1e9, 6
                    )
                else:
                    started = self._quality_lost_since_ns.pop(key, None)
                    if started is not None:
                        longest = max(
                            self._quality_max_blackout_ns.get(key, 0),
                            capture.sim_time_ns - started,
                        )
                        self._quality_max_blackout_ns[key] = longest
                        fields[f"{camera}_{name}_max_blackout_s"] = round(longest / 1e9, 6)
        self._emit(
            "backend.render.quality",
            message="segmentation counts",
            context=EventContext(
                episode_id=capture.context.episode_id or None,
                clock_epoch=capture.context.clock_epoch,
                sim_time_ns=capture.sim_time_ns,
            ),
            fields=fields,
        )

    def _publish_feedback(self, status: ControlStatus) -> None:
        if status.snapshot is None:
            return
        try:
            feedback = execution_feedback(status)
            self._feedback_pub.publish(execution_feedback_message(feedback))
            self._log_feedback_transitions(status, feedback)
        except Exception as error:  # noqa: BLE001 - a terminal fact must not vanish
            self._worker.latch_fault("FEEDBACK_PUBLISH_FAILED", f"{type(error).__name__}: {error}")

    def _log_feedback_transitions(
        self, status: ControlStatus, feedback: ExecutionFeedbackRecord
    ) -> None:
        """Log bounded transition evidence; full per-frame facts remain in MCAP."""
        active = feedback.tracking_quality == "CONTACT_UNLOADING"
        context = EventContext(
            episode_id=feedback.context.episode_id,
            command_id=feedback.command_id,
            clock_epoch=feedback.context.clock_epoch,
            sim_time_ns=feedback.measurement_sim_time_ns,
        )
        if active != self._unload_active:
            self._unload_active = active
            fields = {}
            for side, index in (("left", 7), ("right", 15)):
                contact = next(item for item in status.snapshot.contacts if item.side == side)
                fields[f"{side}_object"] = contact.candidate_object
                fields[f"{side}_reference_m"] = feedback.reference_positions[index]
                fields[f"{side}_measured_m"] = feedback.measured_positions[index]
                fields[f"{side}_reference_velocity_m_s"] = feedback.reference_velocities[index]
                for finger in contact.fingers:
                    fields[f"{side}_finger{finger.finger_index}_force_n"] = finger.force_for(
                        contact.candidate_object
                    )
            self._emit(
                "backend.contact_unload.started" if active else "backend.contact_unload.finished",
                context=context,
                fields=fields,
            )
        if feedback.phase == "FAILED" and feedback.command_id != self._last_failed_command:
            self._last_failed_command = feedback.command_id
            self._emit(
                "backend.failed",
                level="WARNING",
                reason=feedback.reason,
                context=context,
                fields={"chunk_seq": feedback.chunk_seq, "stop_reason": feedback.stop_reason},
            )

    # ---------------------------------------------------------------- helpers
    def _new_bridge(self, future: Future, *, submit: bool = False) -> RosFutureBridge:
        try:
            bridge = RosFutureBridge(
                self,
                future,
                timeout_s=self.config.request_timeout_s,
                timeout_reason="OUTCOME_UNKNOWN" if submit else "BACKEND_TIMEOUT",
                callback_group=self._group,
            )
        except BaseException:
            future.cancel()
            raise
        with self._bridges_lock:
            if len(self._bridges) >= 64:
                bridge.close()
                future.cancel()
                raise ContractError("BACKEND_BUSY", "too many service bridges")
            self._bridges.add(bridge)
        return bridge

    def _retire_bridge(self, bridge: RosFutureBridge) -> None:
        with self._bridges_lock:
            self._bridges.discard(bridge)
        bridge.close()

    async def _await(self, future: Future, *, submit: bool = False) -> Any:
        bridge = self._new_bridge(future, submit=submit)
        try:
            return await bridge.wait()
        finally:
            self._retire_bridge(bridge)

    def _failure_response(self, response: object, error: BaseException) -> object:
        if isinstance(error, ContractError):
            response.success = False  # type: ignore[attr-defined]
            response.reason = error.reason  # type: ignore[attr-defined]
        elif isinstance(error, TimeoutError):
            response.success = False  # type: ignore[attr-defined]
            response.reason = str(error)  # type: ignore[attr-defined]
        elif isinstance(error, BackendClosed):
            response.success = False  # type: ignore[attr-defined]
            response.reason = "BACKEND_CLOSED"  # type: ignore[attr-defined]
        else:
            response.success = False  # type: ignore[attr-defined]
            response.reason = "INTERNAL_ERROR"  # type: ignore[attr-defined]
            self._emit(
                "backend.service.error",
                level="ERROR",
                reason="INTERNAL_ERROR",
                message=type(error).__name__,
                exception=error,
            )
        return response


def main() -> None:
    """Thin module entry point; process lifecycle lives in backend_cli."""
    from .backend_cli import main as cli_main

    cli_main()


if __name__ == "__main__":
    main()
