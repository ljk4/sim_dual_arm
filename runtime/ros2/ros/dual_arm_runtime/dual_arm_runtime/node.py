"""ROS registration, message conversion and forwarding for scripted episodes.

Episode preparation, script execution, cleanup and recording finalization live in
``dual_arm_runtime.episode``.  This module keeps the ROS entities, the synchronized
backend/observation state and the process/executor lifecycle.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import threading
import time
from collections.abc import Sequence
from dataclasses import replace
from functools import partial
from pathlib import Path

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from robot_api_msgs.msg import (
    BackendInfo,
    CameraFrame,
    CommandEvent,
    CommandStatus,
    ObservationIndex,
    RobotState,
    RuntimeStatus,
    TrajectoryCommand,
)
from robot_api_msgs.srv import EndEpisode, StartEpisode
from robot_contracts.models import CommandStatus as DomainCommandStatus
from robot_contracts.models import ContractError, RunContext, Trajectory
from robot_contracts.status import (
    TERMINAL_PHASES,
    validate_status_against_trajectory,
    validate_status_transition,
)
from robot_ros_support.conversions import (
    command_message,
    command_status_from_message,
    context_from_message,
    context_message,
    stamp,
    time_ns,
)
from robot_ros_support.qos import EVENT_QOS, INFO_QOS, SENSOR_QOS
from robot_run_logging import EventContext, Scalar

from dual_arm_runtime.configuration import Configuration, load_configuration
from dual_arm_runtime.control_client import ControlClient
from dual_arm_runtime.episode import EpisodeRunner
from dual_arm_runtime.observation import ObservationBuffer, ObservationRef
from dual_arm_runtime.run_logging import (
    EventSink,
    check_health,
    emit_best_effort,
    emit_event,
    initialize_run,
    new_run_id,
    runtime_summary,
    validate_run_id,
)
from dual_arm_runtime.shutdown import (
    EXIT_RUNTIME_FAILURE,
    EXIT_SHUTDOWN_TIMEOUT,
    ShutdownSignal,
    install_signal_handlers,
    positive_seconds,
    restore_signal_handlers,
)


class RuntimeNode(Node):
    """ROS callbacks stay responsive while an episode worker manages one run."""

    def __init__(
        self,
        configuration: Configuration,
        output: Path,
        auto_start: bool,
        *,
        log: EventSink | None = None,
        run_id: str = "",
    ) -> None:
        super().__init__("episode_runtime")
        self.config, self.output = configuration, output
        self._log = log
        self._run_id = run_id
        self._recipe = configuration.recipe
        self._root = self._recipe.robot_namespace.rstrip("/")
        self._group = ReentrantCallbackGroup()
        self._condition = threading.Condition()
        self._info: BackendInfo | None = None
        self._last_observation: ObservationRef | None = None
        self._last_index_id = 0
        self._terminal_events: set[str] = set()
        self._accepted_commands: set[str] = set()
        self._expected_commands: dict[str, Trajectory] = {}
        self._latest_feedback: dict[str, DomainCommandStatus] = {}
        self._terminal_feedback: set[str] = set()
        self._buffer = ObservationBuffer(
            configuration.profile.camera_names,
            configuration.observation.max_skew_ns,
            configuration.observation.max_age_ns,
        )
        self._episode = EpisodeRunner(
            configuration,
            output,
            self,
            control_factory=lambda stop, fault: ControlClient(
                self,
                self._recipe,
                stop,
                fault,
                log=log,
            ),
            log=log,
            run_id=run_id,
        )
        self._control = self._episode.control
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
        self._camera_topics = {
            camera.name: f"{self._root}/cameras/{camera.name}/frame"
            for camera in configuration.profile.cameras
        }
        for camera in configuration.profile.cameras:
            self.create_subscription(
                CameraFrame,
                self._camera_topics[camera.name],
                partial(self._on_frame, camera.name, self._camera_topics[camera.name]),
                SENSOR_QOS,
            )
        self.create_subscription(
            CommandEvent,
            f"{self._root}/command_events",
            self._on_event,
            EVENT_QOS,
        )
        self.create_subscription(
            CommandStatus,
            f"{self._root}/command_feedback",
            self._on_feedback,
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
        self.create_timer(0.05, self._control.renew)
        # Give a human client a defined initial state before the first episode.
        self.publish_status("IDLE", "", "", "")
        if auto_start:
            self._auto_timer = self.create_timer(0.1, self._auto_start)

    # ---- Thread/resource ownership -------------------------------------------------
    # The main thread owns rclpy, the executor, RuntimeNode destruction and the
    # shutdown request.  The episode worker owns the run context, recorder and the
    # acquire/execute/cleanup sequence; the executor's renewal timer only calls
    # ControlClient.renew.  EpisodeRunner's start lock guards worker identity and
    # shutdown, its recorder lock guards the context/recorder/directories, its
    # failure lock guards the reason, and this node's condition guards backend info,
    # the latest observation and terminal-event waits.  destroy_node happens only
    # after worker_alive() is false.

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
        self._episode.append_failure(reason)

    def worker_alive(self) -> bool:
        return self._episode.worker_alive()

    def join_worker(self, timeout: float) -> bool:
        return self._episode.join_worker(timeout)

    def shutdown_details(self) -> dict[str, str]:
        return self._episode.shutdown_details()

    # ---- EpisodeIO: ROS-side operations used by the episode worker -----------------

    @property
    def info_topic(self) -> str:
        return f"{self._root}/info"

    def wait_backend(
        self,
        episode_id: str,
        timeout_s: float,
        stop: threading.Event,
    ) -> tuple[RunContext, BackendInfo]:
        info = self._wait_ready(timeout_s, stop)
        context = replace(
            context_from_message(info.context),
            episode_id=episode_id,
            control_epoch=1,
        )
        return context, info

    def bind_context(self, context: RunContext) -> None:
        """Start a fresh observation/terminal window for a new episode."""
        with self._condition:
            self._buffer.set_context(context)
            self._last_observation = None
            self._last_index_id = 0
            self._terminal_events.clear()
            self._accepted_commands.clear()
            self._expected_commands.clear()
            self._latest_feedback.clear()
            self._terminal_feedback.clear()
            self._condition.notify_all()

    def wait_observation(
        self,
        after_id: int,
        timeout_s: float,
        stop: threading.Event,
    ) -> ObservationRef:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while (
                self._last_observation is None
                or self._last_observation.observation_id <= after_id
                or not 0
                <= self.get_clock().now().nanoseconds - self._last_observation.state.stamp_ns
                <= self.config.observation.max_age_ns
            ):
                if time.monotonic() >= deadline:
                    raise ContractError("NOT_READY", "Fresh synchronized observation")
                if stop.is_set():
                    raise ContractError("EPISODE_STOPPED", "Observation wait stopped")
                self._condition.wait(timeout=0.05)
            return self._last_observation

    def wait_terminal_events(
        self,
        command_ids: Sequence[str],
        timeout_s: float,
        stop: threading.Event,
    ) -> None:
        expected = set(command_ids)
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while not expected.issubset(self._terminal_events):
                if time.monotonic() >= deadline:
                    raise ContractError("CALL_TIMEOUT", "Missing terminal events")
                if stop.is_set():
                    raise ContractError("EPISODE_STOPPED", "Terminal event wait stopped")
                self._condition.wait(timeout=0.05)

    def _terminal_feedback_required(self) -> bool:
        info = self._info
        return info is not None and "command_feedback_v1" in info.capabilities

    def wait_terminal_events_after_stop(
        self,
        command_ids: Sequence[str],
        timeout_s: float,
    ) -> None:
        """Wait for real terminal events, and validated feedback when advertised.

        This is deliberately independent of the stop Event: Stop/Release has
        already run, and the backend's CANCELED/FAILED/COMPLETED callback may still
        be travelling through ROS.  An unaccepted intent has no ACK to wait for.
        """
        deadline = time.monotonic() + timeout_s
        with self._condition:
            expected = set(command_ids) & self._accepted_commands
            expected_feedback = set(expected) if self._terminal_feedback_required() else set()
            while not expected.issubset(self._terminal_events) or not (
                expected_feedback.issubset(self._terminal_feedback)
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    missing = sorted(
                        (expected - self._terminal_events)
                        | (expected_feedback - self._terminal_feedback)
                    )
                    raise ContractError(
                        "CALL_TIMEOUT",
                        "Missing terminal events after stop (or required feedback): "
                        + ", ".join(missing),
                    )
                self._condition.wait(timeout=min(remaining, 0.05))

    def robot_time_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def publish_command(
        self,
        command: Trajectory,
        timestamp_ns: int,
    ) -> TrajectoryCommand:
        with self._condition:
            self._expected_commands[command.command_id] = command
        message = command_message(command)
        self._episode.record("/runtime/control/commands", message, timestamp_ns)
        self._commands.publish(message)
        return message

    def publish_status(
        self,
        phase: str,
        episode_id: str,
        reason: str,
        record_directory: str,
    ) -> None:
        self._status.publish(
            RuntimeStatus(
                phase=phase,
                episode_id=episode_id,
                reason=reason,
                record_directory=record_directory,
            )
        )

    def record_topics(self) -> dict[str, str]:
        return self._topics()

    def log_error(self, message: str) -> None:
        self.get_logger().error(message)

    def notify(self) -> None:
        with self._condition:
            self._condition.notify_all()

    # ---- ROS callbacks -------------------------------------------------------------

    def _on_info(self, message: BackendInfo) -> None:
        context = self._episode.context
        if context is not None and (
            context_from_message(message.context).generation != context.generation
        ):
            self._episode.fail("STALE_CONTEXT: backend restarted")
        with self._condition:
            self._info = message
            self._condition.notify_all()
        self._episode.record(self.info_topic, message, self.get_clock().now().nanoseconds)

    def _on_state(self, message: RobotState) -> None:
        self._episode.record(f"{self._root}/state", message, time_ns(message.stamp))
        self._buffer.add(
            "state",
            context_from_message(message.context),
            message.state_seq,
            time_ns(message.stamp),
        )

    def _on_frame(self, camera_name: str, topic: str, message: CameraFrame) -> None:
        self._episode.record(topic, message, time_ns(message.image.header.stamp))
        self._buffer.add(
            camera_name,
            context_from_message(message.context),
            message.frame_seq,
            time_ns(message.image.header.stamp),
        )

    _COMMAND_EVENTS = {
        "ACCEPTED": ("command.accepted", "INFO", "COMMAND_ACCEPTED"),
        "STARTED": ("command.started", "INFO", "COMMAND_STARTED"),
        "COMPLETED": ("command.completed", "INFO", "COMMAND_COMPLETED"),
        "FAILED": ("command.failed", "ERROR", "COMMAND_FAILED"),
        "CANCELED": ("command.canceled", "WARNING", "COMMAND_CANCELED"),
    }

    def _log_runtime_event(
        self,
        event: str,
        *,
        level: str = "INFO",
        reason: str | None = None,
        context: EventContext | None = None,
        fields: dict[str, Scalar] | None = None,
        exception: BaseException | None = None,
    ) -> None:
        """Explicit callback-side logging failure handling; never recurses."""
        try:
            emit_event(
                self._log,
                event,
                level=level,
                reason=reason,
                context=context,
                fields=fields,
                exception=exception,
                critical=True,
            )
        except Exception as error:
            self._episode.report_logging_failure(error)

    def _on_event(self, message: CommandEvent) -> None:
        self._episode.record(f"{self._root}/command_events", message, time_ns(message.stamp))
        event_context = context_from_message(message.context)
        command_event = self._COMMAND_EVENTS.get(message.phase)
        if command_event is not None:
            event_name, level, event_reason = command_event
            self._log_runtime_event(
                event_name,
                level=level,
                reason=event_reason,
                context=EventContext(
                    episode_id=event_context.episode_id or None,
                    command_id=message.command_id or None,
                    clock_epoch=event_context.clock_epoch,
                    sim_time_ns=time_ns(message.stamp),
                ),
                fields={
                    "phase": message.phase,
                    "command_seq": int(message.command_seq),
                    "event_seq": int(message.event_seq),
                    "backend_reason": message.reason or None,
                },
            )
        context = self._episode.context
        if context is not None and event_context == context:
            if message.phase in ("ACCEPTED", "STARTED", "COMPLETED", "CANCELED", "FAILED"):
                with self._condition:
                    if message.phase in ("ACCEPTED", "STARTED"):
                        self._accepted_commands.add(message.command_id)
                    if message.phase in ("COMPLETED", "CANCELED", "FAILED"):
                        self._terminal_events.add(message.command_id)
                    self._condition.notify_all()

    def _on_feedback(self, message: CommandStatus) -> None:
        """Validate and enqueue one known command's immutable execution snapshot."""
        context = self._episode.context
        if context is None or context_from_message(message.context) != context:
            return
        try:
            status = command_status_from_message(message)
            with self._condition:
                command = self._expected_commands.get(status.command_id)
                if command is None:
                    raise ContractError(
                        "INVALID_COMMAND_FEEDBACK", f"Unknown command ID: {status.command_id}"
                    )
                validate_status_against_trajectory(
                    status,
                    command.targets,
                    expected_context=command.context,
                    expected_command_id=command.command_id,
                    expected_command_seq=command.command_seq,
                    expected_observation_id=command.observation_id,
                    expected_joint_names=command.joint_names,
                )
                validate_status_transition(self._latest_feedback.get(status.command_id), status)
                self._latest_feedback[status.command_id] = status
        except ContractError as error:
            self._episode.fail(f"INVALID_COMMAND_FEEDBACK: {error}")
            self._log_runtime_event(
                "command.feedback_rejected",
                level="ERROR",
                reason="INVALID_COMMAND_FEEDBACK",
                context=EventContext(
                    episode_id=context.episode_id,
                    command_id=message.command_id or None,
                    clock_epoch=context.clock_epoch,
                    sim_time_ns=time_ns(message.sample_time),
                ),
                fields={"phase": message.phase, "command_seq": int(message.command_seq)},
                exception=error,
            )
            return
        self._episode.record(f"{self._root}/command_feedback", message, status.sample_time_ns)
        if status.phase in TERMINAL_PHASES:
            # The recorder has accepted the sample before a finalizer may wake.
            with self._condition:
                self._terminal_feedback.add(status.command_id)
                self._condition.notify_all()

    def _publish_observation(self) -> None:
        # Timer path: observe an asynchronous recorder failure even when no new
        # frame arrived.  The check only reads the recorder latch; it does no I/O.
        if self._episode.check_recording_health():
            return
        if self._episode.context is None or not self._control.lease_id:
            return
        try:
            observation = self._buffer.assemble(self.get_clock().now().nanoseconds)
        except ContractError:
            if self._last_observation is not None and (
                self.get_clock().now().nanoseconds - self._last_observation.state.stamp_ns
                > self.config.observation.max_age_ns
            ):
                self._episode.fail("STALE_OBSERVATION")
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
            self._episode.record("/runtime/observation_index", message, observation.state.stamp_ns)
            self._indices.publish(message)
            self._last_index_id = observation.observation_id
        with self._condition:
            self._last_observation = observation
            self._condition.notify_all()

    def _wait_ready(self, timeout_s: float, stop: threading.Event) -> BackendInfo:
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while self._info is None or not self._info.ready:
                if time.monotonic() >= deadline:
                    raise ContractError("NOT_READY", "Backend info")
                if stop.is_set():
                    raise ContractError("EPISODE_STOPPED", "Backend info wait stopped")
                self._condition.wait(timeout=0.1)
            info = self._info
        if not info.test_only or info.backend_kind != "mock":
            raise ContractError("UNSUPPORTED_CAPABILITY", "Stage 2 runtime is test-only")
        if info.context.robot_profile_hash != self.config.profile.digest:
            raise ContractError("SCHEMA_MISMATCH", "Backend profile")
        if tuple(info.joint_names) != self.config.profile.joint_names:
            raise ContractError("SCHEMA_MISMATCH", "Backend joints")
        if tuple(info.camera_names) != self.config.profile.camera_names:
            raise ContractError("SCHEMA_MISMATCH", "Backend cameras")
        self._control.wait_action_ready()
        return info

    def _topics(self) -> dict[str, str]:
        topics = {
            self.info_topic: "robot_api_msgs/msg/BackendInfo",
            f"{self._root}/state": "robot_api_msgs/msg/RobotState",
            f"{self._root}/command_events": "robot_api_msgs/msg/CommandEvent",
            "/runtime/control/commands": "robot_api_msgs/msg/TrajectoryCommand",
            "/runtime/observation_index": "robot_api_msgs/msg/ObservationIndex",
        }
        if self._terminal_feedback_required():
            topics[f"{self._root}/command_feedback"] = "robot_api_msgs/msg/CommandStatus"
        for topic in self._camera_topics.values():
            topics[topic] = "robot_api_msgs/msg/CameraFrame"
        return topics

    # ---- Services and start --------------------------------------------------------

    def _start_service(self, request, response):
        try:
            response.episode_id = self._start(request.task_id)
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
            self._start("mock-script")
        except ContractError as error:
            self.get_logger().error(f"Cannot auto-start episode: {error}")

    def _start(self, task_id: str) -> str:
        return self._episode.start(task_id)


def _forced_shutdown_timeout(node: RuntimeNode | None, timeout_s: float) -> None:
    """Last-resort process exit after the bounded grace expired.

    The worker may still own the recorder and ROS clients, so destroy_node() and an
    unbounded join/flush are forbidden. Exit directly and leave the .partial directory
    untouched; this path does NOT promise a complete manifest.
    """
    details = node.shutdown_details() if node is not None else {}
    print(
        json.dumps({"shutdown": "SHUTDOWN_TIMEOUT", "timeout_s": timeout_s, **details}),
        flush=True,
    )
    print(
        f"SHUTDOWN_TIMEOUT: worker still active after {timeout_s:.3f}s; "
        "forcing process exit without destroying ROS resources",
        file=sys.stderr,
        flush=True,
    )
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(EXIT_SHUTDOWN_TIMEOUT)


def _destroy_ros_resources(
    node: RuntimeNode | None,
    executor: SingleThreadedExecutor | None,
) -> None:
    """Main-thread teardown, called only once the episode worker is no longer alive."""
    if executor is not None:
        if node is not None:
            executor.remove_node(node)
        executor.shutdown()
    if node is not None:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-root", type=Path, default=Path("/workspace"))
    parser.add_argument(
        "--config", type=Path, default=Path("/workspace/runtime/ros2/configs/mock_collect.yaml")
    )
    parser.add_argument("--output", type=Path, default=Path("/data/raw"))
    parser.add_argument(
        "--run-id",
        default=None,
        help="run identity; generated as a UUID when omitted",
    )
    parser.add_argument(
        "--runs-output",
        type=Path,
        default=None,
        help="parent directory for --run-id/<run_id>; defaults to <output parent>/runs",
    )
    parser.add_argument("--once", action="store_true")
    parser.add_argument(
        "--shutdown-timeout",
        type=positive_seconds,
        default=15.0,
        help="bounded seconds to wait for episode worker cleanup after a signal",
    )
    args, ros_args = parser.parse_known_args()
    try:
        run_id = new_run_id() if args.run_id is None else validate_run_id(args.run_id)
    except ValueError as error:
        parser.error(str(error))
    runs_output = args.runs_output if args.runs_output is not None else args.output.parent / "runs"
    try:
        configuration = load_configuration(args.project_root, args.config)
    except Exception as error:
        print(
            json.dumps(
                {
                    "runtime_initialization_error": f"{type(error).__name__}: {error}",
                }
            ),
            file=sys.stderr,
            flush=True,
        )
        raise SystemExit(EXIT_RUNTIME_FAILURE) from error
    try:
        session = initialize_run(
            configuration,
            run_id=run_id,
            runs_output=runs_output,
            output=args.output,
        )
    except Exception as error:
        print(
            json.dumps({"runtime_initialization_error": f"{type(error).__name__}: {error}"}),
            file=sys.stderr,
            flush=True,
        )
        raise SystemExit(EXIT_RUNTIME_FAILURE) from error
    logger = session.logger

    signal_state = ShutdownSignal()
    previous_handlers = install_signal_handlers(signal_state)
    rclpy_initialized = False
    node: RuntimeNode | None = None
    executor: SingleThreadedExecutor | None = None
    exit_code = 0
    fatal_error = False
    fatal_exception: BaseException | None = None
    executor_usable = True
    startup_announced = False
    stopping_logged = False
    health_failure_seen = False
    shutdown_deadline: float | None = None
    record_directory = ""
    episode_id = ""

    def set_fatal(error: BaseException) -> None:
        nonlocal fatal_error, fatal_exception
        fatal_error = True
        if fatal_exception is None:
            fatal_exception = error

    def emit_stopping(code: str, detail: str = "") -> None:
        nonlocal stopping_logged
        if stopping_logged:
            return
        stopping_logged = True
        current_episode = ""
        if node is not None:
            try:
                current_episode = node.shutdown_details().get("episode_id", "")
            except Exception:
                current_episode = ""
        if code == "COMPLETE":
            level = "INFO"
        elif code == signal_state.reason and signal_state.requested:
            level = "WARNING"
        else:
            level = "ERROR"
        error_text = emit_best_effort(
            logger,
            "run.stopping",
            level=level,
            reason=code,
            message=detail,
            context=EventContext(episode_id=current_episode or None),
            fields={
                "signal": signal_state.reason or None,
                "episode_id": current_episode or None,
            },
            critical=True,
        )
        if error_text is not None:
            set_fatal(RuntimeError(error_text))
            if node is not None:
                node.append_failure(f"LOGGING_ERROR: {error_text}")

    def current_failure_code() -> str:
        if signal_state.requested:
            return signal_state.reason
        if fatal_error or (node is not None and node.failure_reason):
            return "RUNTIME_FAILURE"
        return "COMPLETE"

    def request_stop(reason: str, code: str, detail: str = "") -> None:
        nonlocal shutdown_deadline
        if node is not None:
            # Idempotent: repeated signals do not become episode failures, while
            # a genuinely different later fault is still appended by the runner.
            node.request_shutdown(reason)
        if shutdown_deadline is None:
            shutdown_deadline = time.monotonic() + args.shutdown_timeout
        emit_stopping(code, detail)

    try:
        try:
            rclpy.init(
                args=ros_args,
                signal_handler_options=SignalHandlerOptions.NO,
            )
            rclpy_initialized = True
            node = RuntimeNode(
                configuration,
                args.output,
                auto_start=args.once,
                log=logger,
                run_id=run_id,
            )
            executor = SingleThreadedExecutor()
            executor.add_node(node)
        except BaseException as error:
            set_fatal(error)
            print(
                json.dumps({"runtime_error": f"INITIALIZATION: {type(error).__name__}: {error}"}),
                flush=True,
            )
        else:
            try:
                emit_event(
                    logger,
                    "run.started",
                    level="INFO",
                    reason="STARTUP",
                    message="runtime process starting",
                    fields={
                        "config_path": str(args.config),
                        "output_path": str(args.output),
                        "runs_output": str(runs_output),
                        "run_directory": str(session.run_directory),
                        "shutdown_timeout_s": args.shutdown_timeout,
                    },
                    critical=True,
                )
                startup_announced = True
            except Exception as error:
                set_fatal(error)
                request_stop(f"LOGGING_ERROR: {error}", "LOGGING_ERROR", detail=str(error))

            while True:
                if not health_failure_seen:
                    try:
                        check_health(logger)
                    except Exception as error:
                        health_failure_seen = True
                        set_fatal(error)
                        request_stop(
                            f"LOGGING_ERROR: {error}",
                            "LOGGING_ERROR",
                            detail=str(error),
                        )
                if node.logging_failed and not node.shutdown_requested:
                    set_fatal(RuntimeError(node.logging_failure))
                    request_stop(
                        f"LOGGING_ERROR: {node.logging_failure}",
                        "LOGGING_ERROR",
                        detail=node.logging_failure,
                    )
                if signal_state.requested and not node.shutdown_requested:
                    request_stop(
                        signal_state.reason,
                        signal_state.reason,
                        detail="termination signal",
                    )
                if args.once and node.done.is_set() and not node.shutdown_requested:
                    break
                if node.shutdown_requested:
                    if not node.worker_alive():
                        break
                    if shutdown_deadline is None:
                        shutdown_deadline = time.monotonic() + args.shutdown_timeout
                    remaining = shutdown_deadline - time.monotonic()
                    if remaining <= 0:
                        _forced_shutdown_timeout(node, args.shutdown_timeout)
                    if executor_usable:
                        try:
                            executor.spin_once(timeout_sec=min(0.05, remaining))
                        except KeyboardInterrupt:
                            signal_state(signal.SIGINT, None)
                        except Exception as error:
                            set_fatal(error)
                            executor_usable = False
                            reason = f"RUNTIME_ERROR: {type(error).__name__}: {error}"
                            request_stop(reason, "RUNTIME_ERROR", detail=reason)
                            print(json.dumps({"runtime_error": reason}), flush=True)
                    elif not node.join_worker(min(0.05, remaining)):
                        # The ROS executor is no longer safe to spin. Give the
                        # worker bounded wall time to finish its local failure.
                        time.sleep(0.01)
                    continue
                try:
                    executor.spin_once(timeout_sec=0.1)
                except KeyboardInterrupt:
                    # rclpy uses SignalHandlerOptions.NO, so this is only a fallback.
                    signal_state(signal.SIGINT, None)
                except Exception as error:
                    set_fatal(error)
                    executor_usable = False
                    reason = f"RUNTIME_ERROR: {type(error).__name__}: {error}"
                    request_stop(reason, "RUNTIME_ERROR", detail=reason)
                    print(json.dumps({"runtime_error": reason}), flush=True)
                    shutdown_deadline = time.monotonic() + args.shutdown_timeout

            if node.worker_alive():
                if shutdown_deadline is None:
                    shutdown_deadline = time.monotonic() + args.shutdown_timeout
                remaining = shutdown_deadline - time.monotonic()
                if remaining <= 0 or not node.join_worker(remaining):
                    _forced_shutdown_timeout(node, args.shutdown_timeout)
            if signal_state.requested:
                exit_code = signal_state.exit_code
            elif fatal_error or node.failure_reason:
                exit_code = EXIT_RUNTIME_FAILURE
            if not stopping_logged:
                emit_stopping(current_failure_code(), "process leaving main")
    except BaseException as error:
        set_fatal(error)
        print(json.dumps({"runtime_error": f"{type(error).__name__}: {error}"}), flush=True)
        if node is not None:
            request_stop(
                f"RUNTIME_ERROR: {type(error).__name__}: {error}",
                "RUNTIME_ERROR",
                detail=str(error),
            )
    finally:
        teardown_error: BaseException | None = None
        try:
            if node is not None:
                try:
                    details = node.shutdown_details()
                    episode_id = details.get("episode_id", "")
                    record_directory = details.get("directory", "")
                except Exception:
                    pass
                if node.worker_alive():
                    # An unexpected main-thread failure still gives the worker a
                    # bounded chance to preserve its local record before teardown.
                    node.request_shutdown("UNEXPECTED_MAIN_EXIT")
                    if not node.join_worker(args.shutdown_timeout):
                        _forced_shutdown_timeout(node, args.shutdown_timeout)
            try:
                if rclpy_initialized:
                    _destroy_ros_resources(node, executor)
            except BaseException as error:
                teardown_error = error
                set_fatal(error)
                print(
                    json.dumps({"runtime_error": f"TEARDOWN: {type(error).__name__}: {error}"}),
                    flush=True,
                )
            if not stopping_logged:
                emit_stopping(current_failure_code(), "process leaving main")
            if signal_state.requested:
                exit_code = signal_state.exit_code
            elif fatal_error or (node is not None and node.failure_reason):
                exit_code = EXIT_RUNTIME_FAILURE
            elif not startup_announced:
                exit_code = EXIT_RUNTIME_FAILURE
            finalizing_code = current_failure_code()
            if finalizing_code == "COMPLETE":
                # Closing is still pending, so this event must not claim a
                # successful process outcome even though exit_code is still 0.
                finalizing_reason = "CLOSE_PENDING"
                finalizing_level = "INFO"
            elif signal_state.requested:
                finalizing_reason = finalizing_code
                finalizing_level = "WARNING"
            else:
                finalizing_reason = finalizing_code
                finalizing_level = "ERROR"
            failure_detail = ""
            if node is not None:
                failure_detail = node.failure_reason
            if teardown_error is not None:
                failure_detail = (
                    f"{failure_detail}; TEARDOWN: {type(teardown_error).__name__}: {teardown_error}"
                )
            try:
                emit_event(
                    logger,
                    "run.finalizing",
                    level=finalizing_level,
                    reason=finalizing_reason,
                    message=failure_detail,
                    context=EventContext(episode_id=episode_id or None),
                    fields={
                        "exit_code_before_log_close": exit_code,
                        "log_close_pending": True,
                        "episode_id": episode_id or None,
                        "record_directory": record_directory or None,
                        "started": startup_announced,
                        "signal": signal_state.reason or None,
                    },
                    exception=fatal_exception or teardown_error,
                    critical=True,
                )
            except Exception as error:
                # The logger is already broken or the payload is invalid. Close
                # still gets its bounded attempt; a normal exit becomes failure.
                if exit_code == 0:
                    exit_code = EXIT_RUNTIME_FAILURE
                print(
                    json.dumps(
                        {"logging_error": f"run.finalizing: {type(error).__name__}: {error}"}
                    ),
                    file=sys.stderr,
                    flush=True,
                )
            logging_closed = False
            try:
                logger.close()
            except Exception as error:
                if exit_code == 0:
                    exit_code = EXIT_RUNTIME_FAILURE
                print(
                    json.dumps({"logging_error": f"close: {type(error).__name__}: {error}"}),
                    file=sys.stderr,
                    flush=True,
                )
            else:
                logging_closed = True
            if signal_state.requested:
                process_outcome = "interrupted"
            elif exit_code == 0:
                process_outcome = "succeeded"
            else:
                process_outcome = "failed"
        finally:
            restore_signal_handlers(previous_handlers)

    print(
        json.dumps(
            runtime_summary(
                session,
                record_directory=record_directory,
                exit_code=exit_code,
                process_outcome=process_outcome,
                logging_closed=logging_closed,
            )
        ),
        flush=True,
    )
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
