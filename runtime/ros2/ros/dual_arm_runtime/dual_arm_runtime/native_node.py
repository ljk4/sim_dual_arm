"""ROS adapter and process entry point for the diagnostic native runtime.

The node owns ROS entities, exact-capture state, callback validation and the
process/executor lifecycle.  Native episode sequencing, command transport and
MCAP auditing live in their own modules.
"""

from __future__ import annotations

import hashlib
import threading
import time
from collections.abc import Sequence
from pathlib import Path

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from robot_api_msgs.msg import (
    BackendInfo,
    CameraFrame,
    NativeCommand,
    NativeControlEvent,
    NativeExecution,
    ObservationIndex,
    RobotState,
    RuntimeStatus,
)
from robot_api_msgs.msg import FollowerReference as FollowerReferenceMessage
from robot_api_msgs.msg import (
    StackTaskState as StackTaskStateMessage,
)
from robot_api_msgs.srv import EndEpisode, StartEpisode
from robot_contracts.models import ContractError, RunContext
from robot_contracts.native import (
    NativeExecution as DomainNativeExecution,
)
from robot_contracts.native import NativeRequest
from robot_contracts.trajectory import (
    FollowerReference as DomainFollowerReference,
)
from robot_contracts.trajectory import (
    FollowerValidationLimits,
)
from robot_contracts.trajectory import (
    validate as validate_follower_reference,
)
from robot_ros_support.conversions import (
    context_from_message,
    context_message,
    stamp,
    time_ns,
)
from robot_ros_support.follower_conversions import follower_reference_from_message
from robot_ros_support.native_conversions import (
    native_command_message,
    native_control_event_from_message,
    native_execution_from_message,
)
from robot_ros_support.qos import EVENT_QOS, INFO_QOS, RELIABLE_SENSOR_QOS, SENSOR_QOS

from dual_arm_runtime.native_config import NativeConfiguration
from dual_arm_runtime.native_control_client import NativeControlClient
from dual_arm_runtime.native_episode import NativeEpisodeRunner
from dual_arm_runtime.native_stream import follower_limits
from dual_arm_runtime.observation import ExactCaptureBuffer, ExactObservation
from dual_arm_runtime.run_logging import (
    EventSink,
)
from dual_arm_runtime.stack_task_client import StackTaskClient

_TERMINAL = frozenset({"COMPLETED", "CANCELED", "FAILED"})
_MAX_EXPECTED_COMMANDS = 64
_MAX_EXPECTED_CHUNKS = 512
_MAX_NATIVE_EVENTS = 4096


def _terminal(execution: DomainNativeExecution) -> bool:
    return execution.phase in _TERMINAL


class NativeRuntimeNode(Node):
    """Native runtime whose executor stays responsive during command waits."""

    def __init__(
        self,
        configuration: NativeConfiguration,
        output: Path,
        auto_start: bool,
        *,
        log: EventSink | None = None,
        run_id: str = "",
        initial_episode_id: str | None = None,
    ) -> None:
        super().__init__("native_episode_runtime")
        self.config = configuration
        self.output = output
        self._log = log
        self._run_id = run_id
        self._recipe = configuration.recipe
        self._root = self._recipe.robot_namespace.rstrip("/")
        self._group = ReentrantCallbackGroup()
        self._condition = threading.Condition()
        self._info: BackendInfo | None = None
        self._last_observation: ExactObservation | None = None
        self._last_index_id = 0
        self._expected_requests: dict[str, NativeRequest] = {}
        self._expected_chunks: dict[tuple[str, int], NativeRequest] = {}
        self._latest_execution: dict[str, DomainNativeExecution] = {}
        self._execution_signature: dict[str, object] = {}
        self._terminal_execution: dict[str, DomainNativeExecution] = {}
        self._events: dict[int, tuple[object, ...]] = {}
        self._accepted_chunk_events: set[tuple[str, int]] = set()
        self._terminal_chunk_events: set[tuple[str, int]] = set()
        self._overall_terminal_events: set[str] = set()
        self._last_task_state_seq = 0
        self._follower_digests: dict[tuple[str, int], set[str]] = {}
        self._rejected = 0
        self._event_duplicates = 0
        self._execution_duplicates = 0
        self._max_accelerations = self._control_limits()
        self._follower_limits: FollowerValidationLimits | None = (
            follower_limits(configuration, self._max_accelerations)
            if configuration.recipe.follower is not None
            else None
        )
        self._buffer = ExactCaptureBuffer(
            configuration.profile.camera_names,
            max_age_ns=configuration.recipe.observation.max_age_ns,
            wall_arrival_timeout_ns=round(
                configuration.recipe.observation.wall_arrival_timeout_s * 1_000_000_000
            ),
            capacity=configuration.recipe.observation.capacity,
        )
        self._episode = NativeEpisodeRunner(
            configuration,
            output,
            self,
            control_factory=lambda stop, fault: NativeControlClient(
                self,
                self._recipe,
                configuration.profile,
                self._max_accelerations,
                stop,
                fault,
                follower_limits=self._follower_limits,
                log=log,
            ),
            log=log,
            run_id=run_id,
            initial_episode_id=initial_episode_id,
        )
        self._control = self._episode.control
        self._stack_task = (
            StackTaskClient(
                self,
                self._root,
                call_timeout_s=self._recipe.call_timeout_s,
                prepare_timeout_s=self._recipe.teacher.prepare_timeout_s
                if self._recipe.teacher is not None
                else self._recipe.call_timeout_s,
                start_timeout_s=self._recipe.teacher.start_timeout_s
                if self._recipe.teacher is not None
                else self._recipe.call_timeout_s,
                chunk_timeout_s=self._recipe.teacher.chunk_deadline_s
                if self._recipe.teacher is not None
                else self._recipe.call_timeout_s,
                poll_interval_s=self._recipe.teacher.poll_interval_s
                if self._recipe.teacher is not None
                else 0.02,
                log=log,
                stop_requested=self._control.stop_requested,
            )
            if self._task_enabled()
            else None
        )
        self._cameras = {camera.name: camera for camera in configuration.profile.cameras}
        self._status = self.create_publisher(RuntimeStatus, "/runtime/status", INFO_QOS)
        self._native_commands = self.create_publisher(
            NativeCommand, "/runtime/native/commands", EVENT_QOS
        )
        self._indices = self.create_publisher(
            ObservationIndex, "/runtime/observation_index", EVENT_QOS
        )
        self.create_subscription(BackendInfo, f"{self._root}/info", self._on_info, INFO_QOS)
        sensor_qos = RELIABLE_SENSOR_QOS if self._recipe.observation.reliable else SENSOR_QOS
        self.create_subscription(RobotState, f"{self._root}/state", self._on_state, sensor_qos)
        self._camera_topics = {name: f"{self._root}/cameras/{name}/frame" for name in self._cameras}
        for name, topic in self._camera_topics.items():
            self.create_subscription(
                CameraFrame,
                topic,
                lambda message, camera=name: self._on_frame(camera, message),
                sensor_qos,
            )
        if self._stack_task is not None:
            self.create_subscription(
                StackTaskStateMessage,
                f"{self._root}/stack_task_state",
                self._on_stack_task_state,
                EVENT_QOS,
            )
        self.create_subscription(
            NativeControlEvent,
            f"{self._root}/native_events",
            self._on_native_event,
            EVENT_QOS,
        )
        self.create_subscription(
            NativeExecution,
            f"{self._root}/native_feedback",
            self._on_native_feedback,
            EVENT_QOS,
        )
        if self._follower_limits is not None:
            self.create_subscription(
                FollowerReferenceMessage,
                f"{self._root}/follower_references",
                self._on_follower_reference,
                EVENT_QOS,
            )
        self.create_service(
            StartEpisode, "/runtime/episode/start", self._start_service, callback_group=self._group
        )
        self.create_service(
            EndEpisode, "/runtime/episode/end", self._end_service, callback_group=self._group
        )
        self.create_timer(0.05, self._publish_observation)
        self.create_timer(0.05, self._control.renew)
        self.publish_status("IDLE", "", "", "")
        if auto_start:
            self._auto_timer = self.create_timer(0.1, self._auto_start)

    def _task_enabled(self) -> bool:
        recipe = self._recipe
        stream = recipe.policy if recipe.action_source == "policy" else recipe.teacher
        return recipe.action_source == "teacher" or (
            stream is not None and stream.termination_mode == "task"
        )

    def _control_limits(self) -> tuple[float, ...]:
        names = self.config.control_document.get("joint_names")
        limits = self.config.control_document.get("max_accelerations")
        if list(names or ()) != list(self.config.profile.joint_names):
            raise ContractError(
                "SCHEMA_MISMATCH", "control joint_names differ from the robot profile"
            )
        if not isinstance(limits, list) or len(limits) != len(self.config.profile.joints):
            raise ContractError(
                "SCHEMA_MISMATCH", "control max_accelerations must match the profile joints"
            )
        result: list[float] = []
        for index, value in enumerate(limits):
            if isinstance(value, bool) or not isinstance(value, int | float):
                raise ContractError(
                    "SCHEMA_MISMATCH", f"max_accelerations[{index}] must be a number"
                )
            number = float(value)
            if not number >= 0.0:
                raise ContractError(
                    "SCHEMA_MISMATCH", f"max_accelerations[{index}] must be non-negative"
                )
            result.append(number)
        return tuple(result)

    # ---- Shared lifecycle surface -------------------------------------------------

    @property
    def done(self) -> threading.Event:
        return self._episode.done

    @property
    def failure_reason(self) -> str:
        return self._episode.failure_reason

    @property
    def logging_failed(self) -> bool:
        return self._episode.logging_failed

    @property
    def logging_failure(self) -> str:
        return self._episode.logging_failure

    @property
    def record_directory(self) -> str:
        return self._episode.record_directory

    @property
    def shutdown_requested(self) -> bool:
        return self._episode.shutdown_requested

    @property
    def shutdown_reason(self) -> str:
        return self._episode.shutdown_reason

    def request_shutdown(self, reason: str) -> None:
        self._episode.request_shutdown(reason)

    def append_failure(self, reason: str) -> None:
        if hasattr(self._episode, "append_failure"):
            self._episode.append_failure(reason)
        else:
            self._episode.report_control_fault(reason)

    def worker_alive(self) -> bool:
        return self._episode.worker_alive()

    def join_worker(self, timeout: float) -> bool:
        return self._episode.join_worker(timeout)

    def shutdown_details(self) -> dict[str, str]:
        return self._episode.shutdown_details()

    # ---- NativeEpisodeIO ----------------------------------------------------------

    @property
    def info_topic(self) -> str:
        return f"{self._root}/info"

    def wait_backend(
        self, episode_id: str, timeout_s: float, stop: threading.Event
    ) -> tuple[RunContext, BackendInfo]:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self._info is None or not self._info.ready:
                if time.monotonic() >= deadline:
                    raise ContractError("NOT_READY", "native backend info")
                if stop.is_set():
                    raise ContractError("EPISODE_STOPPED", "backend wait stopped")
                self._condition.wait(timeout=0.1)
            info = self._info
        if info.backend_kind != "mujoco":
            raise ContractError("UNSUPPORTED_CAPABILITY", "native backend kind")
        if bool(info.test_only) != self._recipe.test_only:
            raise ContractError("UNSUPPORTED_CAPABILITY", "native backend test_only differs")
        if info.context.robot_profile_hash != self.config.profile.digest:
            raise ContractError("SCHEMA_MISMATCH", "backend profile")
        if tuple(info.joint_names) != self.config.profile.joint_names:
            raise ContractError("SCHEMA_MISMATCH", "backend joints")
        if tuple(info.camera_names) != self.config.profile.camera_names:
            raise ContractError("SCHEMA_MISMATCH", "backend cameras")
        context = context_from_message(info.context).model_copy(update={
            "episode_id": episode_id, "control_epoch": 1})
        return context, info

    def bind_context(self, context: RunContext) -> None:
        with self._condition:
            self._buffer.set_context(context)
            self._last_observation = None
            self._last_index_id = 0
            self._expected_requests.clear()
            self._expected_chunks.clear()
            self._latest_execution.clear()
            self._execution_signature.clear()
            self._terminal_execution.clear()
            self._events.clear()
            self._accepted_chunk_events.clear()
            self._terminal_chunk_events.clear()
            self._overall_terminal_events.clear()
            self._last_task_state_seq = 0
            self._follower_digests.clear()
            self._rejected = 0
            self._event_duplicates = 0
            self._execution_duplicates = 0
            self._condition.notify_all()

    def record_backend_info(self, context: RunContext, timestamp_ns: int = 0) -> None:
        with self._condition:
            info = self._info
        if info is None:
            return
        message = BackendInfo(
            protocol_major=info.protocol_major,
            context=context_message(context),
            backend_kind=info.backend_kind,
            ready=info.ready,
            test_only=info.test_only,
            joint_names=list(info.joint_names),
            joint_units=list(info.joint_units),
            camera_names=list(info.camera_names),
            capabilities=list(info.capabilities),
        )
        # BackendInfo has no simulation timestamp; bag time 0 avoids mixing
        # ROS wall time into an otherwise simulation-time axis.
        self._episode.record(self.info_topic, message, 0)

    def wait_observation(
        self, after_id: int, timeout_s: float, stop: threading.Event
    ) -> ExactObservation:
        deadline = time.monotonic() + timeout_s
        observation: ExactObservation | None = None
        with self._condition:
            while observation is None:
                if stop.is_set():
                    raise ContractError("EPISODE_STOPPED", "observation wait stopped")
                try:
                    candidate = self._buffer.assemble(
                        self._simulation_now_ns(), time.monotonic_ns()
                    )
                except ContractError as error:
                    if error.reason == "STALE_OBSERVATION":
                        raise
                    candidate = None
                if candidate is not None and candidate.observation_id > after_id:
                    observation = candidate
                    break
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise ContractError("WALL_TIMEOUT", "exact complete observation did not arrive")
                self._condition.wait(timeout=min(remaining, 0.05))
        self._publish_index(observation)
        return observation

    def publish_native_command(self, request: NativeRequest, timestamp_ns: int) -> None:
        message = native_command_message(request)
        command_id = request.trajectory.command_id
        chunk_seq = 0 if request.anchor is None else request.anchor.predecessor_seq + 1
        with self._condition:
            if (
                command_id not in self._expected_requests
                and len(self._expected_requests) >= _MAX_EXPECTED_COMMANDS
            ):
                raise ContractError("CONTROL_BUSY", "native expected-command cache is full")
            if command_id not in self._expected_requests:
                # The first request is the command-level source of overall events.
                self._expected_requests[command_id] = request
            key = (command_id, chunk_seq)
            if (
                key not in self._expected_chunks
                and len(self._expected_chunks) >= _MAX_EXPECTED_CHUNKS
            ):
                raise ContractError("CONTROL_BUSY", "native expected-chunk cache is full")
            self._expected_chunks[key] = request
        self._episode.record("/runtime/native/commands", message, timestamp_ns)
        self._native_commands.publish(message)

    def record_task_state(self, fact: object) -> None:
        message = fact
        if not isinstance(message, StackTaskStateMessage):
            raise ContractError("TASK_STATE_MISMATCH", "task fact is not a ROS state message")
        sample_ns = time_ns(message.sample_time)
        with self._condition:
            sequence = int(message.state_seq)
            if sequence <= self._last_task_state_seq:
                return
            self._last_task_state_seq = sequence
        self._episode.record(f"{self._root}/stack_task_state", message, sample_ns)

    def stack_task_client(self) -> object:
        return self._stack_task

    def _on_stack_task_state(self, message: StackTaskStateMessage) -> None:
        context = self._episode_context()
        if context is None or context_from_message(message.context) != context:
            self._rejected += 1
            return
        try:
            self.record_task_state(message)
        except ContractError as error:
            self._episode.report_control_fault(str(error))

    def record_follower_reference(self, reference: DomainFollowerReference) -> None:
        command_id = reference.command_id
        key = (command_id, reference.chunk_seq)
        with self._condition:
            digests = self._follower_digests.setdefault(key, set())
            if reference.reference_digest in digests:
                return
            digests.add(reference.reference_digest or "")
        from robot_ros_support.follower_conversions import follower_reference_message

        self._episode.record(
            f"{self._root}/follower_references",
            follower_reference_message(reference),
            reference.accepted_sim_time_ns,
        )

    def record_execution(self, execution: DomainNativeExecution) -> None:
        signature = self._execution_identity(execution)
        with self._condition:
            frozen = self._terminal_execution.get(execution.command_id)
            if frozen is not None:
                # Query and DDS are independent delivery paths. An older
                # nonterminal reply can arrive after a true terminal fact;
                # it must not replace the frozen fact or its identity.
                if _terminal(execution) and self._execution_identity(frozen) != signature:
                    raise ContractError(
                        "INVALID_NATIVE_EXECUTION",
                        f"terminal execution changed after freeze: {execution.command_id}",
                    )
                self._execution_duplicates += 1
                return
            if self._execution_signature.get(execution.command_id) == signature:
                self._execution_duplicates += 1
                return
            from robot_ros_support.native_conversions import native_execution_message

            # Make recorder enqueue precede the observable completion fact.
            # Otherwise finish can close the writer between these two actions.
            self._episode.record(
                f"{self._root}/native_feedback",
                native_execution_message(execution),
                execution.measurement_sim_time_ns,
            )
            self._execution_signature[execution.command_id] = signature
            self._latest_execution[execution.command_id] = execution
            if _terminal(execution):
                self._terminal_execution[execution.command_id] = execution
            self._condition.notify_all()

    def wait_native_terminal(
        self, command_id: str, timeout_s: float, stop: threading.Event
    ) -> DomainNativeExecution:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while command_id not in self._terminal_execution:
                if stop.is_set():
                    raise ContractError("EPISODE_STOPPED", "native terminal wait stopped")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise ContractError("CALL_TIMEOUT", f"native terminal missing for {command_id}")
                self._condition.wait(timeout=min(remaining, 0.05))
            return self._terminal_execution[command_id]

    def wait_stop_settled(self, command_ids: Sequence[str], timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        missing = set(command_ids)
        with self._condition:
            missing = {name for name in missing if not self._control_facts_complete(name)}
        while missing:
            if time.monotonic() >= deadline:
                raise ContractError(
                    "CALL_TIMEOUT", "terminal missing after stop: " + ", ".join(sorted(missing))
                )
            for command_id in tuple(missing):
                request = self._expected_requests.get(command_id)
                if request is None:
                    missing.discard(command_id)
                    continue
                try:
                    execution = self._control.query(
                        request.trajectory.context, command_id, request=request, ignore_stop=True
                    )
                except ContractError as error:
                    if error.reason in {"RESULT_EXPIRED", "BACKEND_CLOSED", "CALL_TIMEOUT"}:
                        continue
                    raise
                self.record_execution(execution)
                with self._condition:
                    if self._control_facts_complete(command_id):
                        missing.discard(command_id)
            if missing:
                time.sleep(min(self._recipe.control.poll_interval_s, 0.05))
        with self._condition:
            absent = {name for name in command_ids if not self._control_facts_complete(name)}
        if absent:
            raise ContractError(
                "CALL_TIMEOUT", "terminal missing after stop: " + ", ".join(sorted(absent))
            )

    def _control_facts_complete(self, command_id: str) -> bool:
        """Called under the condition lock; DDS topics have independent delivery."""
        if command_id not in self._terminal_execution:
            return False
        if command_id not in self._overall_terminal_events:
            return False
        accepted = {key for key in self._accepted_chunk_events if key[0] == command_id}
        if not accepted <= self._terminal_chunk_events:
            return False
        if self._follower_limits is not None and not accepted <= self._follower_digests.keys():
            return False
        return True

    def robot_time_ns(self) -> int:
        """Current robot simulation time, not the ROS wall clock."""
        return self._simulation_now_ns()

    def publish_status(
        self, phase: str, episode_id: str, reason: str, record_directory: str
    ) -> None:
        self._status.publish(
            RuntimeStatus(
                phase=phase, episode_id=episode_id, reason=reason, record_directory=record_directory
            )
        )

    def record_topics(self) -> dict[str, str]:
        topics = {
            self.info_topic: "robot_api_msgs/msg/BackendInfo",
            f"{self._root}/state": "robot_api_msgs/msg/RobotState",
            f"{self._root}/native_events": "robot_api_msgs/msg/NativeControlEvent",
            f"{self._root}/native_feedback": "robot_api_msgs/msg/NativeExecution",
            "/runtime/native/commands": "robot_api_msgs/msg/NativeCommand",
            "/runtime/observation_index": "robot_api_msgs/msg/ObservationIndex",
        }
        if self._follower_limits is not None:
            topics[f"{self._root}/follower_references"] = "robot_api_msgs/msg/FollowerReference"
        if self._stack_task is not None:
            topics[f"{self._root}/stack_task_state"] = "robot_api_msgs/msg/StackTaskState"
        for topic in self._camera_topics.values():
            topics[topic] = "robot_api_msgs/msg/CameraFrame"
        return topics

    def log_error(self, message: str) -> None:
        self.get_logger().error(message)

    def notify(self) -> None:
        with self._condition:
            self._condition.notify_all()

    # ---- Callbacks ---------------------------------------------------------------

    def _episode_context(self) -> RunContext | None:
        return self._episode.context

    def _matches(self, message: object) -> bool:
        context = self._episode_context()
        if context is None:
            return False
        try:
            return context_from_message(message.context) == context  # type: ignore[attr-defined]
        except Exception:
            return False

    def _on_info(self, message: BackendInfo) -> None:
        context = self._episode_context()
        incoming = context_from_message(message.context)
        if context is not None and incoming.generation != context.generation:
            self._episode.report_control_fault("STALE_CONTEXT: backend restarted")
        with self._condition:
            self._info = message
            self._condition.notify_all()
        if context is not None and incoming == context:
            self._episode.record(self.info_topic, message, 0)

    def _on_state(self, message: RobotState) -> None:
        context = self._episode_context()
        if context is None or context_from_message(message.context) != context:
            self._rejected += 1
            return
        names = list(message.joints.name)
        if names != list(self.config.profile.joint_names):
            self._episode.report_control_fault("STATE_JOINTS_DIFFER")
            return
        if len(message.joints.position) != len(names):
            self._episode.report_control_fault("STATE_POSITION_COUNT_DIFFERS")
            return
        stamp_ns = time_ns(message.stamp)
        if time_ns(message.joints.header.stamp) != stamp_ns:
            self._episode.report_control_fault("STATE_STAMP_MISMATCH")
            return
        self._episode.record(f"{self._root}/state", message, stamp_ns)
        accepted = self._buffer.add(
            "state",
            context,
            int(message.state_seq),
            stamp_ns,
            message,
            wall_arrival_ns=time.monotonic_ns(),
            fingerprint=(stamp_ns, tuple(message.joints.position), tuple(message.joints.velocity)),
        )
        if not accepted and self._buffer.conflict_reason:
            self._episode.report_control_fault(
                f"OBSERVATION_CONFLICT: {self._buffer.conflict_reason}"
            )
        self.notify()

    def _validate_frame(self, camera_name: str, message: CameraFrame) -> None:
        camera = self._cameras[camera_name]
        image = message.image
        info = message.camera_info
        stamp_ns = time_ns(image.header.stamp)
        if (image.width, image.height) != (camera.width, camera.height):
            raise ContractError("SCHEMA_MISMATCH", f"{camera_name}: image dimensions")
        if image.encoding != "rgb8" or image.step != camera.width * 3:
            raise ContractError("SCHEMA_MISMATCH", f"{camera_name}: RGB encoding/step")
        if len(image.data) != camera.width * camera.height * 3:
            raise ContractError("SCHEMA_MISMATCH", f"{camera_name}: RGB payload size")
        if image.header.frame_id != camera.frame_id:
            raise ContractError("SCHEMA_MISMATCH", f"{camera_name}: image frame_id")
        if time_ns(info.header.stamp) != stamp_ns or info.header.frame_id != camera.frame_id:
            raise ContractError("SCHEMA_MISMATCH", f"{camera_name}: CameraInfo header")
        if (info.width, info.height) != (camera.width, camera.height):
            raise ContractError("SCHEMA_MISMATCH", f"{camera_name}: CameraInfo dimensions")
        if message.calibration_digest != camera.calibration_digest:
            raise ContractError("SCHEMA_MISMATCH", f"{camera_name}: calibration digest")

    def _on_frame(self, camera_name: str, message: CameraFrame) -> None:
        context = self._episode_context()
        if context is None or context_from_message(message.context) != context:
            self._rejected += 1
            return
        try:
            self._validate_frame(camera_name, message)
        except ContractError as error:
            self._episode.report_control_fault(str(error))
            return
        stamp_ns = time_ns(message.image.header.stamp)
        self._episode.record(self._camera_topics[camera_name], message, stamp_ns)
        digest = hashlib.sha256(bytes(message.image.data)).digest()
        accepted = self._buffer.add(
            camera_name,
            context,
            int(message.frame_seq),
            stamp_ns,
            message,
            wall_arrival_ns=time.monotonic_ns(),
            fingerprint=(stamp_ns, digest),
        )
        if not accepted and self._buffer.conflict_reason:
            self._episode.report_control_fault(
                f"OBSERVATION_CONFLICT: {self._buffer.conflict_reason}"
            )
        self.notify()

    def _on_native_event(self, message: NativeControlEvent) -> None:
        context = self._episode_context()
        if context is None or context_from_message(message.context) != context:
            self._rejected += 1
            return
        try:
            event = native_control_event_from_message(message)
            if event.is_chunk:
                request = self._expected_chunks.get((event.command_id, event.chunk_seq))
            else:
                request = self._expected_requests.get(event.command_id)
            if request is None:
                raise ContractError(
                    "INVALID_NATIVE_EVENT", f"event for unknown command/chunk {event.command_id}"
                )
            trajectory = request.trajectory
            if (
                event.command_seq != trajectory.command_seq
                or event.observation_id != trajectory.observation_id
            ):
                raise ContractError("INVALID_NATIVE_EVENT", "event command identity differs")
        except ContractError as error:
            self._episode.report_control_fault(str(error))
            return
        signature = (
            event.context,
            event.event_seq,
            event.command_id,
            event.is_chunk,
            event.chunk_seq,
            event.phase,
            event.reason,
            event.sim_time_ns,
        )
        with self._condition:
            previous = self._events.get(event.event_seq)
            if previous == signature:
                self._event_duplicates += 1
                return
            if previous is not None:
                self._episode.report_control_fault(
                    f"NATIVE_EVENT_CONFLICT: sequence {event.event_seq}"
                )
                return
            if len(self._events) >= _MAX_NATIVE_EVENTS:
                self._episode.report_control_fault("NATIVE_EVENT_CACHE_FULL")
                return
            self._events[event.event_seq] = signature
        self._episode.record(f"{self._root}/native_events", message, event.sim_time_ns)
        with self._condition:
            key = (event.command_id, event.chunk_seq)
            if event.is_chunk:
                if event.phase == "ACCEPTED":
                    self._accepted_chunk_events.add(key)
                if event.phase in {"RETIRED", "CANCELED", "REJECTED"}:
                    self._terminal_chunk_events.add(key)
            elif event.phase in _TERMINAL:
                self._overall_terminal_events.add(event.command_id)
            self._condition.notify_all()

    def _execution_identity(self, execution: DomainNativeExecution) -> tuple[object, ...]:
        return (
            execution.phase,
            execution.reason,
            execution.stop_reason,
            execution.accepted_sim_time_ns,
            execution.accepted_wall_time_ns,
            execution.has_started,
            execution.started_sim_time_ns,
            execution.started_wall_time_ns,
            execution.has_finished,
            execution.finished_sim_time_ns,
            execution.finished_wall_time_ns,
            execution.motion_seq,
            execution.stream_id,
            execution.chunk_seq,
            execution.motion_phase,
            execution.reference_application_sim_time_ns,
            execution.measurement_sim_time_ns,
            execution.reference_elapsed_ns,
            execution.stable_for_ns,
            tuple(execution.reference_positions),
            tuple(execution.reference_velocities),
            tuple(execution.reference_accelerations),
            tuple(execution.measured_positions),
            tuple(execution.measured_velocities),
            tuple(execution.position_errors),
            tuple(execution.velocity_errors),
            execution.reference_kind,
            execution.tracking_quality,
            execution.tracking_violation_ns,
            tuple(
                (
                    gripper.side,
                    gripper.mode,
                    gripper.position_ok,
                    gripper.contact_ok,
                    gripper.contact_object,
                    gripper.ambiguous,
                    gripper.target_position_m,
                    gripper.initial_reference_m,
                    gripper.final_reference_m,
                    gripper.measured_position_m,
                    gripper.measured_velocity_m_s,
                    gripper.target_error_m,
                    tuple(
                        (
                            finger.finger_index,
                            finger.body_name,
                            finger.joint_speed_m_s,
                            tuple(finger.object_names),
                            tuple(finger.normal_forces),
                            tuple(finger.contact_counts),
                        )
                        for finger in gripper.fingers
                    ),
                )
                for gripper in execution.grippers
            ),
        )

    def _on_native_feedback(self, message: NativeExecution) -> None:
        context = self._episode_context()
        if context is None or context_from_message(message.context) != context:
            self._rejected += 1
            return
        try:
            execution = native_execution_from_message(message)
            request = self._expected_requests.get(execution.command_id)
            if (
                execution.chunk_seq > 0
                and (execution.command_id, execution.chunk_seq) not in self._expected_chunks
            ):
                raise ContractError(
                    "INVALID_NATIVE_EXECUTION", "feedback names an unknown applied chunk"
                )
            if request is None:
                raise ContractError(
                    "INVALID_NATIVE_EXECUTION",
                    f"feedback for unknown command/chunk {execution.command_id}",
                )
            trajectory = request.trajectory
            if (
                execution.command_seq != trajectory.command_seq
                or execution.observation_id != trajectory.observation_id
                or execution.joint_names != trajectory.joint_names
            ):
                raise ContractError("INVALID_NATIVE_EXECUTION", "feedback identity differs")
        except ContractError as error:
            self._episode.report_control_fault(str(error))
            return
        try:
            self.record_execution(execution)
        except ContractError as error:
            self._episode.report_control_fault(str(error))

    def _on_follower_reference(self, message: FollowerReferenceMessage) -> None:
        context = self._episode_context()
        try:
            reference = follower_reference_from_message(message)
            if context is None or reference.context != context:
                self._rejected += 1
                return
            if self._follower_limits is None:
                raise ContractError("FOLLOWER_DISABLED", "no follower limits configured")
            request = self._expected_chunks.get((reference.command_id, reference.chunk_seq))
            if request is None or reference.request != request:
                raise ContractError(
                    "REFERENCE_REQUEST_MISMATCH", "follower reference request differs"
                )
            validate_follower_reference(reference, self.config.profile, self._follower_limits)
        except Exception as error:
            if isinstance(error, ContractError) and error.reason == "EPISODE_STOPPED":
                return
            self._episode.report_control_fault(
                str(error)
                if isinstance(error, ContractError)
                else f"INVALID_FOLLOWER_REFERENCE: {error}"
            )
            return
        self.record_follower_reference(reference)

    def _simulation_now_ns(self) -> int:
        stamp = self._buffer.latest_state_stamp()
        return 0 if stamp is None else stamp

    def _publish_observation(self) -> None:
        if self._episode.check_recording_health():
            return
        context = self._episode_context()
        if context is None:
            return
        try:
            observation = self._buffer.assemble(self._simulation_now_ns(), time.monotonic_ns())
        except ContractError as error:
            if error.reason == "STALE_OBSERVATION":
                self._episode.report_control_fault("STALE_OBSERVATION")
            return
        self._publish_index(observation)
        with self._condition:
            self._last_observation = observation
            self._condition.notify_all()

    def _publish_index(self, observation: ExactObservation) -> None:
        """Record/publish one observation exactly once, from timer or worker."""
        with self._condition:
            if observation.observation_id == self._last_index_id:
                return
            self._last_index_id = observation.observation_id
        message = ObservationIndex(
            context=context_message(observation.context),
            observation_id=observation.observation_id,
            state_seq=observation.state_seq,
            camera_names=list(observation.frames),
            frame_seqs=[sample.sequence for sample in observation.frames.values()],
            reference_time=stamp(observation.stamp_ns),
        )
        self._episode.record("/runtime/observation_index", message, observation.stamp_ns)
        self._indices.publish(message)

    # ---- Services and start --------------------------------------------------------

    def _start_service(self, request, response):
        try:
            response.episode_id = self._episode.start(request.task_id)
            response.success, response.reason = True, "PREPARING"
        except ContractError as error:
            response.success, response.reason = False, error.reason
        return response

    def _end_service(self, request, response):
        if not self._episode.end_episode(request.reason):
            response.success, response.reason = False, "NO_ACTIVE_EPISODE"
        else:
            response.success, response.reason = True, "ENDING"
        return response

    def _auto_start(self) -> None:
        self._auto_timer.cancel()
        if self.shutdown_requested:
            return
        try:
            task_id = (
                self._recipe.policy.task_id
                if self._recipe.policy is not None
                else "panda-stack"
                if self._recipe.action_source == "teacher"
                else "native-diagnostic"
            )
            self._episode.start(task_id)
        except ContractError as error:
            self.get_logger().error(f"Cannot auto-start native episode: {error}")


def main() -> None:
    """Thin module entry point; process lifecycle lives in native_cli."""
    from dual_arm_runtime.native_cli import main as cli_main

    cli_main()


if __name__ == "__main__":
    main()
