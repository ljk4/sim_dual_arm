"""Lifecycle of one scripted episode: prepare, run, clean up and finish recording.

``RuntimeNode`` owns the ROS entities, message conversion and synchronized
observation buffer.  ``EpisodeRunner`` owns everything that changes from one run
to the next, including the worker, stop/shutdown requests, failure aggregation,
run context, recorder and record directory.  It never imports ``node``; the ROS
side is reached through the typed :class:`EpisodeIO` protocol.
"""

from __future__ import annotations

import json
import threading
import uuid
from collections.abc import Callable, Sequence
from dataclasses import asdict
from pathlib import Path
from typing import TYPE_CHECKING, Protocol

from robot_contracts.episodes import RuntimeOutcome, RuntimeResult, TaskOutcome, TaskResult
from robot_contracts.models import ContractError, RunContext, Target, Trajectory
from robot_run_logging import EventContext, LogFailure, Scalar

from dual_arm_runtime.configuration import Configuration
from dual_arm_runtime.observation import ObservationRef
from dual_arm_runtime.recording_queue import RecordingStats
from dual_arm_runtime.run_logging import (
    EventSink,
    check_health,
    emit_best_effort,
    emit_event,
)

if TYPE_CHECKING:
    from robot_api_msgs.msg import BackendInfo, TrajectoryCommand


class Recorder(Protocol):
    """The small recorder surface owned by one episode."""

    directory: Path

    def append(self, topic: str, message: object, timestamp_ns: int) -> None: ...

    def check_health(self) -> None: ...

    def stats(self) -> RecordingStats: ...

    def finish(
        self,
        runtime: RuntimeResult | None = None,
        *,
        current_runtime: Callable[[], RuntimeResult] | None = None,
        task_result: TaskResult | None = None,
    ) -> Path: ...


class Control(Protocol):
    """The ControlClient operations the episode worker sequences."""

    @property
    def lease_id(self) -> str: ...

    def wait_action_ready(self) -> None: ...

    def acquire(self, context: RunContext) -> None: ...

    def execute(self, command_message: object) -> None: ...

    def renew(self) -> None: ...

    def cleanup(self) -> None: ...


class EpisodeIO(Protocol):
    """ROS-side operations needed by an episode, all on typed values.

    Implementations may block only inside the ``wait_*`` methods.  ``stop`` is the
    episode-owned event, so a stop request is visible to both the worker and the
    callbacks waits.
    """

    @property
    def info_topic(self) -> str: ...

    def wait_backend(
        self,
        episode_id: str,
        timeout_s: float,
        stop: threading.Event,
    ) -> tuple[RunContext, BackendInfo]: ...

    def bind_context(self, context: RunContext) -> None: ...

    def wait_observation(
        self,
        after_id: int,
        timeout_s: float,
        stop: threading.Event,
    ) -> ObservationRef: ...

    def wait_terminal_events(
        self,
        command_ids: Sequence[str],
        timeout_s: float,
        stop: threading.Event,
    ) -> None: ...

    def wait_terminal_events_after_stop(
        self,
        command_ids: Sequence[str],
        timeout_s: float,
    ) -> None: ...

    def robot_time_ns(self) -> int: ...

    def publish_command(self, command: Trajectory, timestamp_ns: int) -> TrajectoryCommand: ...

    def publish_status(
        self,
        phase: str,
        episode_id: str,
        reason: str,
        record_directory: str,
    ) -> None: ...

    def record_topics(self) -> dict[str, str]: ...

    def log_error(self, message: str) -> None: ...

    def notify(self) -> None: ...


class FailureLog:
    """Aggregate the primary fault and later cleanup/recording reasons.

    The lock protects only the string.  Setting the stop event and waking the ROS
    condition happen outside it; no RPC, join or disk operation runs here.
    """

    def __init__(self, stop_requested: threading.Event, notify: Callable[[], None]) -> None:
        self._lock = threading.Lock()
        self._stop_requested = stop_requested
        self._notify = notify
        self._reason = ""

    @property
    def reason(self) -> str:
        with self._lock:
            return self._reason

    def reset(self, reason: str = "") -> None:
        with self._lock:
            self._reason = reason

    def fail(self, reason: str) -> None:
        """Keep the first primary fault and request a stop."""
        with self._lock:
            if not self._reason:
                self._reason = reason
        self._stop_requested.set()
        self._notify()

    def request_stop(self) -> None:
        """Ask the worker to stop without recording a fault.

        Manual EndEpisode and termination signals use this path; they change the
        runtime outcome but are not themselves recording or control failures.
        """
        self._stop_requested.set()
        self._notify()

    def append(self, reason: str) -> None:
        """Append a later cleanup reason without losing the primary fault."""
        with self._lock:
            if not reason or reason in self._reason:
                return
            if self._reason:
                self._reason = f"{self._reason}; {reason}"
            else:
                self._reason = reason
        self._stop_requested.set()
        self._notify()

    def merge(self, reason: str) -> None:
        """Take a recorder's combined reason without duplicating the primary fault."""
        with self._lock:
            if not reason or reason in self._reason:
                return
            if self._reason and self._reason not in reason:
                self._reason = f"{self._reason}; {reason}"
            else:
                self._reason = reason
        self._stop_requested.set()
        self._notify()


RecorderFactory = Callable[[str, dict[str, object], dict[str, str]], Recorder]
ControlFactory = Callable[[threading.Event, Callable[[str], None]], Control]


class EpisodeRunner:
    """Own one runtime's episode lifecycle while the node owns the ROS surface."""

    def __init__(
        self,
        configuration: Configuration,
        output: Path,
        io: EpisodeIO,
        control_factory: ControlFactory,
        recorder_factory: RecorderFactory | None = None,
        log: EventSink | None = None,
        run_id: str = "",
    ) -> None:
        self._configuration = configuration
        self._output = output
        self._io = io
        self._log = log
        self._run_id = run_id
        self._task_id = ""
        self._logging_failure = ""
        self._logging_lock = threading.Lock()
        self._recorder_factory = recorder_factory or self._new_recorder
        self._start_lock = threading.Lock()
        self._recorder_lock = threading.Lock()
        self._stop_requested = threading.Event()
        self._shutdown_requested = threading.Event()
        self._done = threading.Event()
        self._finishing = threading.Event()
        self._end_requested = False
        self._end_reason = ""
        self._shutdown_reason = ""
        self._active_episode_id = ""
        self._sent_command_ids: list[str] = []
        self._record_directory = ""
        self._partial_directory = ""
        self._recorder: Recorder | None = None
        self._recording_fault = ""
        self._run_context: RunContext | None = None
        self._worker: threading.Thread | None = None
        self._faults = FailureLog(self._stop_requested, io.notify)
        # Construction order guarantees that the control client and the episode
        # share this single stop-request event instance.
        self._control = control_factory(self._stop_requested, self.report_control_fault)

    # ---- Wiring and shared state ---------------------------------------------------
    # _start_lock: worker identity, shutdown request/reason, active episode ID and
    #   the start/reject transition.  The lock is never held while joining a worker.
    # _recorder_lock: run context, current recorder and record directories.  The
    #   recorder is detached under this lock but finished outside it.
    # _faults._lock: only the failure string; FailureLog sets the event/notifies
    #   after releasing it.
    # node._condition: backend info, latest observation and terminal-event waits.

    @property
    def control(self) -> Control:
        return self._control

    @property
    def stop_requested(self) -> threading.Event:
        return self._stop_requested

    @property
    def done(self) -> threading.Event:
        return self._done

    @property
    def failure_reason(self) -> str:
        return self._faults.reason

    @property
    def record_directory(self) -> str:
        with self._recorder_lock:
            return self._record_directory

    @property
    def context(self) -> RunContext | None:
        with self._recorder_lock:
            return self._run_context

    @property
    def shutdown_requested(self) -> bool:
        return self._shutdown_requested.is_set()

    @property
    def shutdown_reason(self) -> str:
        return self._shutdown_reason

    def current_runtime_result(self) -> RuntimeResult:
        """Classify the explicit runtime state, with faults taking precedence.

        The runner owns this decision: no caller searches reason text to infer
        whether a stop was a failure.  Priority is real fault, then termination
        signal, then accepted manual end, then normal completion.
        """
        # Lock order is always _start_lock -> FailureLog._lock.  start() also
        # reads the failure string while holding the start lock for rejection.
        with self._start_lock:
            shutdown_requested = self._shutdown_requested.is_set()
            shutdown_reason = self._shutdown_reason
            end_requested = self._end_requested
            end_reason = self._end_reason
            failure = self._faults.reason
        if failure:
            # Keep the primary fault first but do not discard a simultaneous
            # signal or later manual end reason.
            extras = [
                value
                for value in (
                    shutdown_reason if shutdown_requested else "",
                    end_reason if end_requested else "",
                )
                if value and value not in failure
            ]
            reason = "; ".join([failure, *extras]) if extras else failure
            return RuntimeResult(RuntimeOutcome.FAILED, reason)
        if shutdown_requested:
            if shutdown_reason in {"SIGINT", "SIGTERM"}:
                return RuntimeResult(RuntimeOutcome.INTERRUPTED, shutdown_reason)
            return RuntimeResult(
                RuntimeOutcome.FAILED,
                shutdown_reason or "RUNTIME_SHUTDOWN",
            )
        if end_requested:
            return RuntimeResult(
                RuntimeOutcome.STOPPED,
                end_reason or "EPISODE_ENDED_EARLY",
            )
        return RuntimeResult.completed()

    def report_control_fault(self, reason: str) -> None:
        """Log a client fault and add it after the primary episode failure."""
        self._io.log_error(reason)
        self._faults.append(reason)

    def fail(self, reason: str) -> None:
        self._faults.fail(reason)

    def append_failure(self, reason: str) -> None:
        self._faults.append(reason)

    @property
    def logging_failed(self) -> bool:
        with self._logging_lock:
            return bool(self._logging_failure)

    @property
    def logging_failure(self) -> str:
        with self._logging_lock:
            return self._logging_failure

    def report_logging_failure(self, error: BaseException) -> None:
        """Record the first logging fault without emitting through the same logger."""
        with self._logging_lock:
            if not self._logging_failure:
                self._logging_failure = f"{type(error).__name__}: {error}"
        # FailureLog has its own lock; never call it while holding _logging_lock.
        self._faults.append(f"LOGGING_ERROR: {error}")

    def _log_context(
        self,
        episode_id: str = "",
        *,
        clock_epoch: int | None = None,
        command_id: str | None = None,
        request_id: str | None = None,
        sim_time_ns: int | None = None,
    ) -> EventContext:
        return EventContext(
            episode_id=episode_id or None,
            request_id=request_id,
            command_id=command_id,
            clock_epoch=clock_epoch,
            sim_time_ns=sim_time_ns,
        )

    def _emit_or_fail(
        self,
        event: str,
        *,
        level: str = "INFO",
        reason: str | None = None,
        context: EventContext | None = None,
        fields: dict[str, Scalar] | None = None,
        exception: BaseException | None = None,
    ) -> None:
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

    def _emit_teardown(
        self,
        event: str,
        *,
        level: str = "INFO",
        reason: str | None = None,
        context: EventContext | None = None,
        fields: dict[str, Scalar] | None = None,
        exception: BaseException | None = None,
    ) -> None:
        error = emit_best_effort(
            self._log,
            event,
            level=level,
            reason=reason,
            context=context,
            fields=fields,
            exception=exception,
            critical=True,
        )
        if error is not None:
            self.report_logging_failure(LogFailure(error))

    def _is_signal_shutdown(self) -> bool:
        return self._shutdown_requested.is_set() and self._shutdown_reason in {
            "SIGINT",
            "SIGTERM",
        }

    def _failure_level(self, error: BaseException) -> str:
        if self._is_signal_shutdown() and self._failure_code(error) == "EPISODE_STOPPED":
            return "WARNING"
        return "ERROR"

    def _failure_code(self, error: BaseException) -> str:
        if isinstance(error, ContractError) and error.reason:
            return error.reason
        if isinstance(error, LogFailure):
            return "LOGGING_ERROR"
        return "EPISODE_ERROR"

    # ---- Start, shutdown and worker entry points -----------------------------------

    def start(self, task_id: str) -> str:
        """Start one episode or reject shutdown/concurrent starts."""
        with self._start_lock:
            if self._shutdown_requested.is_set():
                raise ContractError("SHUTTING_DOWN")
            if self._worker is not None and self._worker.is_alive():
                raise ContractError("CONTROL_BUSY")
            if self.logging_failed:
                raise ContractError("LOGGING_FAILED", self.logging_failure)
            episode_id = f"episode-{uuid.uuid4()}"
            with self._recorder_lock:
                self._recorder = None
                self._run_context = None
                self._record_directory = ""
                self._partial_directory = ""
                self._recording_fault = ""
            self._faults.reset()
            self._stop_requested.clear()
            self._finishing.clear()
            self._end_requested = False
            self._end_reason = ""
            self._active_episode_id = episode_id
            self._sent_command_ids = []
            self._task_id = task_id
            self._done.clear()
            try:
                check_health(self._log)
            except Exception as error:
                self.report_logging_failure(error)
                raise ContractError("LOGGING_FAILED", self.failure_reason) from error
            self._worker = threading.Thread(
                target=self._run_episode,
                args=(episode_id, task_id),
                daemon=True,
            )
            self._worker.start()
            return episode_id

    def end_episode(self, reason: str) -> bool:
        """Accept one idempotent manual end request for the active episode.

        The request is only an intent: the worker still treats the recording as
        valid if it drains and audits, and the finalizer writes ``runtime=stopped``.
        A request arriving after finalization started is acknowledged as ENDING but
        cannot rewrite a script result that was already being committed.
        """
        with self._start_lock:
            worker = self._worker
            if worker is None or not worker.is_alive():
                return False
            if self._end_requested or self._finishing.is_set():
                return True
            self._end_requested = True
            self._end_reason = reason or "EPISODE_ENDED_EARLY"
        self._faults.request_stop()
        self._emit_teardown(
            "episode.end_requested",
            level="WARNING",
            reason="EPISODE_END_REQUESTED",
            context=self._log_context(self._active_episode_id),
            fields={"reason": self._end_reason},
        )
        return True

    def request_shutdown(self, reason: str) -> None:
        """Stop accepting episodes and ask the current worker to clean up and finish."""
        with self._start_lock:
            first_request = not self._shutdown_requested.is_set()
            if first_request:
                self._shutdown_reason = reason
            self._shutdown_requested.set()
            worker = self._worker
            worker_alive = worker is not None and worker.is_alive()
            already_reported = not first_request and reason == self._shutdown_reason
        if not worker_alive:
            return
        if first_request:
            if reason in {"SIGINT", "SIGTERM"}:
                self._faults.request_stop()
            else:
                self.fail(reason)
        elif not already_reported and reason not in {"SIGINT", "SIGTERM"}:
            self.append_failure(reason)

    def worker_alive(self) -> bool:
        with self._start_lock:
            worker = self._worker
            return worker is not None and worker.is_alive()

    def join_worker(self, timeout: float) -> bool:
        """Join without holding a lifecycle lock; True once the worker has exited."""
        with self._start_lock:
            worker = self._worker
        if worker is None:
            return True
        worker.join(timeout=max(0.0, timeout))
        return not worker.is_alive()

    def shutdown_details(self) -> dict[str, str]:
        """Best-effort diagnostics for the bounded forced-exit path."""
        with self._start_lock:
            episode_id = self._active_episode_id
        with self._recorder_lock:
            recorder = self._recorder
            directory = self._record_directory or self._partial_directory
            if not directory and recorder is not None:
                directory = str(recorder.directory)
        return {
            "episode_id": episode_id,
            "directory": directory,
            "reason": self.failure_reason,
        }

    def record(self, topic: str, message: object, timestamp_ns: int) -> None:
        """Append under the recorder lock so finish cannot race an in-flight append.

        ``append`` serializes and enqueues but never waits for disk or capacity, so
        the lock is only held for bounded CPU work.  Failure reporting happens after
        the lock is released; no logger callback runs while it is held.
        """
        failure = ""
        with self._recorder_lock:
            recorder = self._recorder
            if recorder is None:
                return
            try:
                recorder.append(topic, message, timestamp_ns)
            except Exception as error:
                failure = f"RECORDING_ERROR: {error}"
        if failure:
            self._report_recording_fault(failure)

    def check_recording_health(self) -> str:
        """Check the current recorder; return the newly latched reason, if any."""
        with self._recorder_lock:
            recorder = self._recorder
        if recorder is None:
            return ""
        try:
            recorder.check_health()
        except Exception as error:
            reason = f"RECORDING_ERROR: {error}"
            self._report_recording_fault(reason)
            return reason
        return ""

    def _report_recording_fault(self, reason: str) -> None:
        """Latch and report one recorder fault, never repeating it per control tick."""
        with self._recorder_lock:
            if self._recording_fault:
                return
            self._recording_fault = reason
        self._io.log_error(reason)
        self.fail(reason)

    # ---- Worker body ---------------------------------------------------------------

    def _run_episode(self, episode_id: str, task_id: str) -> None:
        """Always signal completion, even after an unexpected worker error."""
        try:
            self._run_episode_body(episode_id, task_id)
        except LogFailure as error:
            self.report_logging_failure(error)
            self._io.log_error(self.failure_reason)
        except Exception as error:
            self.fail(f"WORKER_ERROR: {error}")
            self._io.log_error(self.failure_reason)
            self._emit_teardown(
                "episode.failed",
                level=self._failure_level(error),
                reason=self._failure_code(error),
                context=self._log_context(
                    episode_id,
                    clock_epoch=self._current_clock_epoch(),
                ),
                fields={"task_id": task_id},
                exception=error,
            )
        finally:
            self._done.set()

    def _current_clock_epoch(self) -> int | None:
        context = self.context
        return None if context is None else context.clock_epoch

    def _current_record_path(self) -> str:
        with self._recorder_lock:
            return self._record_directory or self._partial_directory

    def _run_episode_body(self, episode_id: str, task_id: str) -> None:
        try:
            self._set_status("PREPARING", episode_id)
            self._emit_or_fail(
                "episode.preparing",
                context=self._log_context(episode_id),
                fields={"task_id": task_id},
            )
            context, info = self._io.wait_backend(
                episode_id,
                self._configuration.recipe.startup_timeout_s,
                self._stop_requested,
            )
            self._io.bind_context(context)
            with self._recorder_lock:
                self._run_context = context
            snapshot = {
                **self._configuration.snapshot,
                "run_id": self._run_id,
                "source": str(info.backend_kind),
                "test_only": bool(info.test_only),
                "task_id": task_id,
                "run_context": asdict(context),
            }
            recorder = self._recorder_factory(episode_id, snapshot, self._io.record_topics())
            with self._recorder_lock:
                self._recorder = recorder
                self._partial_directory = str(recorder.directory)
            self.record(self._io.info_topic, info, self._io.robot_time_ns())
            check_health(self._log)
            self._control.acquire(context)
            self._emit_or_fail(
                "episode.active",
                context=self._log_context(episode_id, clock_epoch=context.clock_epoch),
                fields={
                    "task_id": task_id,
                    "record_directory": self._current_record_path() or None,
                },
            )
            self._set_status("ACTIVE", episode_id)
            command_ids: list[str] = []
            previous_observation_id = 0
            for sequence, target in enumerate(self._configuration.recipe.script_targets, start=1):
                # Check before every control send so an asynchronous writer
                # failure cannot silently admit another trajectory.
                check_health(self._log)
                recording_failure = self.check_recording_health()
                if recording_failure:
                    raise ContractError("RECORDING_ERROR", recording_failure)
                # A manual end or signal visible before this point must not send
                # another script segment.
                if self._stop_requested.is_set():
                    break
                observation = self._io.wait_observation(
                    previous_observation_id,
                    self._configuration.recipe.call_timeout_s,
                    self._stop_requested,
                )
                previous_observation_id = observation.observation_id
                if self._stop_requested.is_set():
                    break
                command = Trajectory(
                    context,
                    self._control.lease_id,
                    f"{episode_id}-{sequence}",
                    sequence,
                    observation.observation_id,
                    self._io.robot_time_ns() + self._configuration.recipe.command_acceptance_ns,
                    self._configuration.profile.joint_names,
                    (Target(target.duration_ns, target.positions),),
                )
                self._emit_or_fail(
                    "command.sending",
                    context=self._log_context(
                        episode_id,
                        clock_epoch=context.clock_epoch,
                        command_id=command.command_id,
                        sim_time_ns=observation.state.stamp_ns,
                    ),
                    fields={
                        "task_id": task_id,
                        "command_seq": sequence,
                        "observation_id": observation.observation_id,
                        "lease_id": command.lease_id,
                        "accept_before_ns": command.accept_before_ns,
                        "target_count": len(command.targets),
                    },
                )
                recording_failure = self.check_recording_health()
                if recording_failure:
                    raise ContractError("RECORDING_ERROR", recording_failure)
                message = self._io.publish_command(command, self._io.robot_time_ns())
                self._sent_command_ids.append(message.command_id)
                self._control.execute(message)
                command_ids.append(command.command_id)
            if not self._stop_requested.is_set():
                self._io.wait_terminal_events(
                    command_ids,
                    self._configuration.recipe.call_timeout_s,
                    self._stop_requested,
                )
        except LogFailure as error:
            self.report_logging_failure(error)
            self._io.log_error(self.failure_reason)
        except Exception as error:
            # A controlled stop is a runtime outcome, not a recording or control
            # fault.  Anything else, including real timeouts and I/O errors, is a
            # failure even if an EndEpisode request arrived at the same time.
            controlled_stop = (
                isinstance(error, ContractError)
                and error.reason == "EPISODE_STOPPED"
                and self._stop_requested.is_set()
            )
            if not controlled_stop:
                self.fail(str(error))
                self._io.log_error(self.failure_reason)
                self._emit_teardown(
                    "episode.failed",
                    level=self._failure_level(error),
                    reason=self._failure_code(error),
                    context=self._log_context(
                        episode_id,
                        clock_epoch=self._current_clock_epoch(),
                    ),
                    fields={"task_id": task_id},
                    exception=error,
                )
        finally:
            self._finish(episode_id)

    def _finish(self, episode_id: str) -> None:
        """Clean up control and recording independently of every status failure."""
        self._finishing.set()
        try:
            self._set_status("ENDING", episode_id)
        except Exception as error:
            self._faults.append(f"STATUS_ERROR: {error}")
        clock_epoch = self._current_clock_epoch()
        primary_reason = self.failure_reason
        runtime = self.current_runtime_result()
        if runtime.outcome is RuntimeOutcome.FAILED:
            ending_level, ending_reason = "ERROR", "EPISODE_ENDING_FAILED"
        elif runtime.outcome is RuntimeOutcome.INTERRUPTED:
            ending_level, ending_reason = "WARNING", "EPISODE_ENDING_INTERRUPTED"
        elif runtime.outcome is RuntimeOutcome.STOPPED:
            ending_level, ending_reason = "INFO", "EPISODE_ENDING_STOPPED"
        else:
            ending_level, ending_reason = "INFO", None
        self._emit_teardown(
            "episode.ending",
            level=ending_level,
            reason=ending_reason,
            context=self._log_context(episode_id, clock_epoch=clock_epoch),
            fields={
                "task_id": self._task_id,
                "runtime_outcome": runtime.outcome.value,
                "runtime_reason": runtime.reason or None,
                "primary_reason": primary_reason or None,
                "record_directory": self._current_record_path() or None,
            },
        )
        try:
            self._control.cleanup()
        except Exception as error:
            self._faults.append(f"CLEANUP_ERROR: {error}")
        # Stop/Release can race a CANCELED/FAILED terminal still travelling over
        # ROS.  Keep the recorder attached until this episode's real accepted
        # commands report a terminal, but bound the wait and never turn an
        # unaccepted intent into a fabricated ACK.
        if self._sent_command_ids:
            try:
                self._io.wait_terminal_events_after_stop(
                    tuple(self._sent_command_ids),
                    self._configuration.recipe.call_timeout_s,
                )
            except Exception as error:
                self._faults.append(f"TERMINAL_WAIT_ERROR: {error}")
                self._emit_teardown(
                    "episode.terminal_wait_failed",
                    level="ERROR",
                    reason="TERMINAL_WAIT_FAILED",
                    context=self._log_context(episode_id, clock_epoch=clock_epoch),
                    fields={"command_count": len(self._sent_command_ids)},
                    exception=error,
                )
        with self._recorder_lock:
            recorder = self._recorder
            self._recorder = None
        recording_stats: RecordingStats | None = None
        recording_complete: bool | None = None
        recording_failure = ""
        if recorder is not None:
            try:
                path = recorder.finish(
                    runtime,
                    current_runtime=self.current_runtime_result,
                )
                with self._recorder_lock:
                    self._record_directory = str(path)
                manifest = json.loads((path / "manifest.json").read_text())
                recording_dimension = manifest["recording"]
                recording_complete = bool(recording_dimension["complete"])
                recording_failure = str(recording_dimension["failure_reason"])
                if not recording_complete:
                    self._faults.merge(recording_failure)
                manifest_runtime = RuntimeResult(
                    RuntimeOutcome(manifest["runtime"]["outcome"]),
                    manifest["runtime"]["reason"],
                )
                if manifest_runtime.outcome is RuntimeOutcome.FAILED:
                    # The recorder can discover a close/count/audit fault only
                    # inside finish; surface it as the runtime failure too.
                    self._faults.merge(manifest_runtime.reason)
                try:
                    recording_stats = recorder.stats()
                except Exception:
                    recording_stats = None
                if recording_complete:
                    recording_level, recording_reason = "INFO", "RECORDING_COMPLETED"
                else:
                    recording_level, recording_reason = "ERROR", "RECORDING_INCOMPLETE"
                self._emit_teardown(
                    "recording.completed" if recording_complete else "recording.failed",
                    level=recording_level,
                    reason=recording_reason,
                    context=self._log_context(episode_id, clock_epoch=clock_epoch),
                    fields={
                        "path": str(path),
                        "recording_complete": recording_complete,
                        "recording_failure_reason": recording_failure or None,
                        **self._recording_stat_fields(recording_stats),
                    },
                )
            except Exception as error:
                recording_complete = False
                recording_failure = str(error)
                try:
                    recording_stats = recorder.stats()
                except Exception:
                    recording_stats = None
                self._faults.append(f"RECORDING_ERROR: {error}")
                self._emit_teardown(
                    "recording.failed",
                    level="ERROR",
                    reason="RECORDING_FAILED",
                    context=self._log_context(episode_id, clock_epoch=clock_epoch),
                    fields={
                        "path": self._current_record_path() or None,
                        "recording_complete": False,
                        "recording_failure_reason": recording_failure or None,
                        **self._recording_stat_fields(recording_stats),
                    },
                    exception=error,
                )
        with self._recorder_lock:
            self._run_context = None
        runtime = self.current_runtime_result()
        if runtime.outcome is RuntimeOutcome.FAILED:
            finished_level, finished_reason = "ERROR", "EPISODE_FAILED"
        elif runtime.outcome is RuntimeOutcome.INTERRUPTED:
            finished_level, finished_reason = "WARNING", "EPISODE_INTERRUPTED"
        elif runtime.outcome is RuntimeOutcome.STOPPED:
            finished_level, finished_reason = "INFO", "EPISODE_STOPPED"
        else:
            finished_level, finished_reason = "INFO", "EPISODE_COMPLETED"
        self._emit_teardown(
            "episode.finished",
            level=finished_level,
            reason=finished_reason,
            context=self._log_context(episode_id, clock_epoch=clock_epoch),
            fields={
                "task_id": self._task_id,
                "record_directory": self._current_record_path() or None,
                "recording_complete": recording_complete,
                "recording_failure_reason": recording_failure or None,
                "runtime_outcome": runtime.outcome.value,
                "runtime_reason": runtime.reason or None,
                "task_outcome": TaskOutcome.NOT_EVALUATED.value,
                "task_reason": None,
                "primary_reason": primary_reason or None,
                **self._recording_stat_fields(recording_stats),
            },
        )
        reason = self.failure_reason
        try:
            self._set_status(
                "FAULT" if runtime.outcome is RuntimeOutcome.FAILED else "IDLE",
                episode_id,
                reason if runtime.outcome is RuntimeOutcome.FAILED else "",
            )
        except Exception as error:
            self._faults.append(f"STATUS_ERROR: {error}")
        runtime = self.current_runtime_result()
        print(
            json.dumps(
                {
                    "episode_id": episode_id,
                    "directory": self.record_directory,
                    "recording": {
                        "complete": recording_complete,
                        "failure_reason": recording_failure or None,
                    },
                    "runtime": {
                        "outcome": runtime.outcome.value,
                        "reason": runtime.reason or None,
                    },
                    "task": {
                        "outcome": TaskOutcome.NOT_EVALUATED.value,
                        "reason": None,
                    },
                }
            ),
            flush=True,
        )

    @staticmethod
    def _recording_stat_fields(stats: RecordingStats | None) -> dict[str, Scalar]:
        if stats is None:
            return {}
        return {
            "recording_accepted": stats.accepted,
            "recording_written": stats.written,
            "recording_pending_items": stats.pending_items,
            "recording_pending_bytes": stats.pending_bytes,
            "recording_peak_items": stats.peak_items,
            "recording_peak_bytes": stats.peak_bytes,
            "recording_failure_reason": stats.failure_reason or None,
        }

    def _set_status(self, phase: str, episode_id: str = "", reason: str = "") -> None:
        self._io.publish_status(phase, episode_id, reason, self.record_directory)

    def _new_recorder(
        self,
        episode_id: str,
        snapshot: dict[str, object],
        topics: dict[str, str],
    ) -> Recorder:
        from dual_arm_runtime.recording import EpisodeRecorder

        # Configuration validation still uses the v1 recipe/robot schemas; the
        # episode manifest selects v2 via an explicit mapping from the base dir.
        schema_root = self._configuration.schema_root
        if schema_root.name in {"v1", "v2"}:
            schema_root = schema_root.parent
        return EpisodeRecorder(
            self._output,
            episode_id,
            snapshot,
            schema_root,
            topics,
            settings=self._configuration.recipe.recording,
            run_id=snapshot.get("run_id", self._run_id),
            source=snapshot.get("source", ""),
            test_only=snapshot.get("test_only", False),
        )
