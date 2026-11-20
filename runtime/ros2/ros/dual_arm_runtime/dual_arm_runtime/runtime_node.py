"""ROS adapter and process entry point for the diagnostic runtime.

The node owns ROS entities, exact-capture state, callback validation and the
process/executor lifecycle.  Episode sequencing, command transport and
MCAP auditing live in their own modules.  Subscription callbacks live in
:mod:`runtime_callbacks` and execution identity in :mod:`runtime_identity`.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Sequence
from pathlib import Path

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from robot_api_msgs.msg import (
    BackendInfo,
    CameraFrame,
    ControlEvent,
    ObservationIndex,
    RobotState,
    RuntimeStatus,
    StreamCommand,
)
from robot_api_msgs.msg import ExecutionFeedback as ExecutionFeedbackMessage
from robot_api_msgs.msg import FollowerReference as FollowerReferenceMessage
from robot_api_msgs.srv import EndEpisode, StartEpisode
from robot_contracts.execution_feedback import ExecutionFeedback
from robot_contracts.models import ContractError, RunContext
from robot_contracts.stream_request import StreamRequest
from robot_contracts.trajectory.follower_reference import (
    FollowerReference as DomainFollowerReference,
)
from robot_contracts.trajectory.follower_reference import FollowerValidationLimits
from robot_ros_support.conversions import (
    context_from_message,
    context_message,
    stamp,
    time_ns,
)
from robot_ros_support.follower_conversions import follower_reference_message
from robot_ros_support.qos import EVENT_QOS, INFO_QOS, RELIABLE_SENSOR_QOS, SENSOR_QOS
from robot_ros_support.stream_conversions import (
    execution_feedback_message,
    stream_command_message,
)

from dual_arm_runtime.chunk_stream import follower_limits
from dual_arm_runtime.episode_runner import EpisodeRunner
from dual_arm_runtime.observation import ExactCaptureBuffer, ExactObservation
from dual_arm_runtime.robot_control_client import RobotControlClient
from dual_arm_runtime.run_logging import EventSink
from dual_arm_runtime.runtime_callbacks import RuntimeCallbacks
from dual_arm_runtime.runtime_config import RuntimeConfiguration
from dual_arm_runtime.runtime_identity import execution_identity
from dual_arm_runtime.runtime_limits import MAX_CHUNKS, control_limits
from dual_arm_runtime.simulation.options import task_enabled

_TERMINAL = frozenset({"COMPLETED", "CANCELED", "FAILED"})
_MAX_EXPECTED_COMMANDS = 64
_MAX_EXPECTED_CHUNKS = MAX_CHUNKS


def _terminal(execution: ExecutionFeedback) -> bool:
    return execution.phase in _TERMINAL


class RuntimeNode(Node):
    """Runtime whose executor stays responsive during command waits."""

    def __init__(
        self,
        configuration: RuntimeConfiguration,
        output: Path,
        auto_start: bool,
        *,
        log: EventSink | None = None,
        run_id: str = "",
        initial_episode_id: str | None = None,
        episode_output: Path | None = None,
    ) -> None:
        super().__init__("episode_runtime")
        self.config = configuration
        self.output = output
        self._log = log
        self._run_id = run_id
        self._recipe = configuration.recipe
        self._root = self._recipe.robot_namespace.rstrip("/")
        self._group = ReentrantCallbackGroup()
        self._condition = threading.Condition()
        self._init_stream_state(configuration)
        self._init_episode(
            configuration, output, log, run_id, initial_episode_id, episode_output
        )
        self._callbacks = RuntimeCallbacks(self)
        self._setup_ros(auto_start)

    def _init_stream_state(self, configuration: RuntimeConfiguration) -> None:
        self._info: BackendInfo | None = None
        self._last_observation: ExactObservation | None = None
        self._last_index_id = 0
        self._expected_requests: dict[str, StreamRequest] = {}
        self._expected_chunks: dict[tuple[str, int], StreamRequest] = {}
        self._latest_execution: dict[str, ExecutionFeedback] = {}
        self._execution_signature: dict[str, object] = {}
        self._terminal_execution: dict[str, ExecutionFeedback] = {}
        self._events: dict[int, tuple[object, ...]] = {}
        self._accepted_chunk_events: set[tuple[str, int]] = set()
        self._terminal_chunk_events: set[tuple[str, int]] = set()
        self._overall_terminal_events: set[str] = set()
        self._last_task_state_seq = 0
        self._follower_digests: dict[tuple[str, int], set[str]] = {}
        self._rejected = 0
        self._event_duplicates = 0
        self._execution_duplicates = 0
        self._max_accelerations = control_limits(configuration)
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

    def _init_episode(
        self,
        configuration: RuntimeConfiguration,
        output: Path,
        log: EventSink | None,
        run_id: str,
        initial_episode_id: str | None,
        episode_output: Path | None = None,
    ) -> None:
        self._episode = EpisodeRunner(
            configuration,
            output,
            self,
            control_factory=lambda stop, fault: RobotControlClient(
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
            episode_output=episode_output,
        )
        self._control = self._episode.control
        self._task_client = None
        if task_enabled(self._recipe):
            from dual_arm_runtime.simulation.task_client import create_task_client

            self._task_client = create_task_client(
                self, self._root, self._recipe, self._control.stop_requested, log,
            )
        self._cameras = {camera.name: camera for camera in configuration.profile.cameras}

    def _setup_ros(self, auto_start: bool) -> None:
        self._status = self.create_publisher(RuntimeStatus, "/runtime/status", INFO_QOS)
        self._commands = self.create_publisher(StreamCommand, "/runtime/commands", EVENT_QOS)
        self._indices = self.create_publisher(
            ObservationIndex, "/runtime/observation_index", EVENT_QOS
        )
        self.create_subscription(BackendInfo, f"{self._root}/info", self._on_info, INFO_QOS)
        sensor_qos = RELIABLE_SENSOR_QOS if self._recipe.observation.reliable else SENSOR_QOS
        self.create_subscription(RobotState, f"{self._root}/state", self._on_state, sensor_qos)
        self._camera_topics = {
            name: f"{self._root}/cameras/{name}/frame" for name in self._cameras
        }
        for name, topic in self._camera_topics.items():
            self.create_subscription(
                CameraFrame,
                topic,
                lambda message, camera=name: self._on_frame(camera, message),
                sensor_qos,
            )
        if self._task_client is not None:
            from dual_arm_runtime.simulation.task_client import task_state_message_type

            self.create_subscription(
                task_state_message_type(),
                f"{self._root}/task_state",
                self._on_task_state,
                EVENT_QOS,
            )
        self.create_subscription(
            ControlEvent, f"{self._root}/control_events", self._on_event, EVENT_QOS
        )
        self.create_subscription(
            ExecutionFeedbackMessage,
            f"{self._root}/execution_feedback",
            self._on_feedback,
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
                    raise ContractError("NOT_READY", "backend info")
                if stop.is_set():
                    raise ContractError("EPISODE_STOPPED", "backend wait stopped")
                self._condition.wait(timeout=0.1)
            info = self._info
        if info.context.robot_profile_hash != self.config.profile.digest:
            raise ContractError("SCHEMA_MISMATCH", "backend profile")
        if tuple(info.joint_names) != self.config.profile.joint_names:
            raise ContractError("SCHEMA_MISMATCH", "backend joints")
        if tuple(info.camera_names) != self.config.profile.camera_names:
            raise ContractError("SCHEMA_MISMATCH", "backend cameras")
        context = context_from_message(info.context).model_copy(
            update={"episode_id": episode_id, "control_epoch": 1}
        )
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
                        self._backend_now_ns(), time.monotonic_ns()
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

    def publish_command(self, request: StreamRequest, timestamp_ns: int) -> None:
        message = stream_command_message(request)
        command_id = request.trajectory.command_id
        chunk_seq = 0 if request.anchor is None else request.anchor.predecessor_seq + 1
        with self._condition:
            if (
                command_id not in self._expected_requests
                and len(self._expected_requests) >= _MAX_EXPECTED_COMMANDS
            ):
                raise ContractError("CONTROL_BUSY", "expected-command cache is full")
            if command_id not in self._expected_requests:
                # The first request is the command-level source of overall events.
                self._expected_requests[command_id] = request
            key = (command_id, chunk_seq)
            if (
                key not in self._expected_chunks
                and len(self._expected_chunks) >= _MAX_EXPECTED_CHUNKS
            ):
                raise ContractError("CONTROL_BUSY", "expected-chunk cache is full")
            self._expected_chunks[key] = request
        self._episode.record("/runtime/commands", message, timestamp_ns)
        self._commands.publish(message)

    def record_task_state(self, fact: object) -> None:
        from dual_arm_runtime.simulation.task_client import task_state_message_type

        if not isinstance(fact, task_state_message_type()):
            raise ContractError("TASK_STATE_MISMATCH", "task fact is not a ROS state message")
        sample_ns = time_ns(fact.sample_time)
        with self._condition:
            sequence = int(fact.state_seq)
            if sequence <= self._last_task_state_seq:
                return
            self._last_task_state_seq = sequence
        self._episode.record(f"{self._root}/task_state", fact, sample_ns)

    def task_client(self) -> object:
        return self._task_client

    def record_follower_reference(self, reference: DomainFollowerReference) -> None:
        command_id = reference.command_id
        key = (command_id, reference.chunk_seq)
        with self._condition:
            digests = self._follower_digests.setdefault(key, set())
            if reference.reference_digest in digests:
                return
            digests.add(reference.reference_digest or "")
        self._episode.record(
            f"{self._root}/follower_references",
            follower_reference_message(reference),
            reference.accepted_sim_time_ns,
        )

    def record_execution(self, execution: ExecutionFeedback) -> None:
        signature = execution_identity(execution)
        with self._condition:
            frozen = self._terminal_execution.get(execution.command_id)
            if frozen is not None:
                # Query and DDS are independent delivery paths. An older
                # nonterminal reply can arrive after a true terminal fact;
                # it must not replace the frozen fact or its identity.
                if _terminal(execution) and execution_identity(frozen) != signature:
                    raise ContractError(
                        "INVALID_EXECUTION_FEEDBACK",
                        f"terminal execution changed after freeze: {execution.command_id}",
                    )
                self._execution_duplicates += 1
                return
            if self._execution_signature.get(execution.command_id) == signature:
                self._execution_duplicates += 1
                return
            # Make recorder enqueue precede the observable completion fact.
            # Otherwise finish can close the writer between these two actions.
            self._episode.record(
                f"{self._root}/execution_feedback",
                execution_feedback_message(execution),
                execution.measurement_sim_time_ns,
            )
            self._execution_signature[execution.command_id] = signature
            self._latest_execution[execution.command_id] = execution
            if _terminal(execution):
                self._terminal_execution[execution.command_id] = execution
            self._condition.notify_all()

    def wait_terminal(
        self, command_id: str, timeout_s: float, stop: threading.Event
    ) -> ExecutionFeedback:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while command_id not in self._terminal_execution:
                if stop.is_set():
                    raise ContractError("EPISODE_STOPPED", "terminal wait stopped")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise ContractError("CALL_TIMEOUT", f"terminal state missing for {command_id}")
                self._condition.wait(timeout=min(remaining, 0.05))
            return self._terminal_execution[command_id]

    def wait_stop_settled(self, command_ids: Sequence[str], timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            # Stream IDs exist before prediction. Only published requests can
            # have backend terminal facts, including submissions with lost ACKs.
            command_ids = tuple(name for name in command_ids if name in self._expected_requests)
            missing = {name for name in command_ids if not self._control_facts_complete(name)}
        while missing:
            if time.monotonic() >= deadline:
                raise ContractError(
                    "CALL_TIMEOUT",
                    "terminal state missing after stop: " + ", ".join(sorted(missing)),
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
                "CALL_TIMEOUT", "terminal state missing after stop: " + ", ".join(sorted(absent))
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
        """Current backend clock time used by the recorded observations."""
        return self._backend_now_ns()

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
            f"{self._root}/control_events": "robot_api_msgs/msg/ControlEvent",
            f"{self._root}/execution_feedback": "robot_api_msgs/msg/ExecutionFeedback",
            "/runtime/commands": "robot_api_msgs/msg/StreamCommand",
            "/runtime/observation_index": "robot_api_msgs/msg/ObservationIndex",
        }
        if self._follower_limits is not None:
            topics[f"{self._root}/follower_references"] = "robot_api_msgs/msg/FollowerReference"
        if self._task_client is not None:
            from dual_arm_runtime.simulation.task_client import TASK_STATE_MESSAGE_TYPE

            topics[f"{self._root}/task_state"] = TASK_STATE_MESSAGE_TYPE
        for topic in self._camera_topics.values():
            topics[topic] = "robot_api_msgs/msg/CameraFrame"
        return topics

    def log_error(self, message: str) -> None:
        self.get_logger().error(message)

    def notify(self) -> None:
        with self._condition:
            self._condition.notify_all()

    def _on_info(self, message: BackendInfo) -> None:
        self._callbacks.on_info(message)

    def _on_state(self, message: RobotState) -> None:
        self._callbacks.on_state(message)

    def _on_frame(self, camera_name: str, message: CameraFrame) -> None:
        self._callbacks.on_frame(camera_name, message)

    def _on_event(self, message: object) -> None:
        self._callbacks.on_event(message)

    def _on_feedback(self, message: ExecutionFeedbackMessage) -> None:
        self._callbacks.on_feedback(message)

    def _on_follower_reference(self, message: FollowerReferenceMessage) -> None:
        self._callbacks.on_follower_reference(message)

    def _on_task_state(self, message: object) -> None:
        self._callbacks.on_task_state(message)

    def _backend_now_ns(self) -> int:
        stamp_value = self._buffer.latest_state_stamp()
        return 0 if stamp_value is None else stamp_value

    def _publish_observation(self) -> None:
        if self._episode.check_recording_health():
            return
        context = self._episode.context
        if context is None:
            return
        try:
            observation = self._buffer.assemble(self._backend_now_ns(), time.monotonic_ns())
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
                else self._recipe.teacher.task_id
                if self._recipe.teacher is not None
                else "runtime-diagnostic"
            )
            self._episode.start(task_id)
        except ContractError as error:
            self.get_logger().error(f"Cannot auto-start episode: {error}")


def main() -> None:
    """Thin module entry point; process lifecycle lives in runtime_cli."""
    from dual_arm_runtime.runtime_cli import main as cli_main

    cli_main()


if __name__ == "__main__":
    main()
