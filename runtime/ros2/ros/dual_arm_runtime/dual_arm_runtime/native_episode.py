"""One native streaming episode: reset, policy/teacher chunks, stop and record.

This module owns only lifecycle and sequencing.  ROS entities and callbacks stay
in ``native_node``; message conversion stays in the shared ROS support layer and
lease/command transport stays in ``native_control_client``.
"""

from __future__ import annotations

import json
import threading
import uuid
from collections.abc import Callable, Sequence
from dataclasses import asdict
from pathlib import Path
from typing import TYPE_CHECKING, Protocol

from robot_contracts.action_chunks import ChunkAnchor
from robot_contracts.episodes import (
    NATIVE_EPISODE_SCHEMA_VERSION,
    POLICY_EPISODE_SCHEMA_VERSION,
    RuntimeOutcome,
    RuntimeResult,
    TaskOutcome,
    TaskResult,
)
from robot_contracts.follower_reference import FollowerReference
from robot_contracts.models import ContractError, RunContext
from robot_contracts.native import NativeExecution, NativeRequest
from robot_run_logging import EventContext, LogFailure, Scalar

from dual_arm_runtime.native_config import NativeConfiguration
from dual_arm_runtime.native_errors import FirstFollowerRejected
from dual_arm_runtime.observation import ExactObservation
from dual_arm_runtime.recording_queue import RecordingStats
from dual_arm_runtime.run_logging import check_health, emit_best_effort
from dual_arm_runtime.task_result import (
    task_fact_document,
    task_result_for_runtime,
    task_result_from_fact,
)

if TYPE_CHECKING:
    from robot_api_msgs.msg import BackendInfo


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
        """Ask the worker to stop without recording a fault."""
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


class NativeControl(Protocol):
    @property
    def lease_id(self) -> str: ...

    def wait_ready(self) -> None: ...

    def acquire(self, context: RunContext) -> str: ...

    def reset(self, context: RunContext, lease_id: str, seed: int) -> RunContext: ...

    def query(
        self, context: RunContext, command_id: str, *, request: NativeRequest
    ) -> NativeExecution: ...

    def get_anchor(
        self, context: RunContext, start_sim_time_ns: int | None = None
    ) -> ChunkAnchor: ...

    def submit_follower(self, request: NativeRequest) -> object: ...

    def stop_episode(self, reason: str) -> None: ...

    def cleanup(self, reason: str = "EPISODE_END") -> None: ...


class NativeEpisodeIO(Protocol):
    """ROS-side operations with bounded waits; every method is worker-thread safe."""

    @property
    def info_topic(self) -> str: ...

    def wait_backend(
        self, episode_id: str, timeout_s: float, stop: threading.Event
    ) -> tuple[RunContext, BackendInfo]: ...

    def bind_context(self, context: RunContext) -> None: ...

    def record_backend_info(self, context: RunContext) -> None: ...

    def wait_observation(
        self, after_id: int, timeout_s: float, stop: threading.Event
    ) -> ExactObservation: ...

    def publish_native_command(self, request: NativeRequest, timestamp_ns: int) -> None: ...

    def record_follower_reference(self, reference: FollowerReference) -> None: ...

    def record_task_state(self, fact: object) -> None: ...

    def stack_task_client(self) -> object: ...

    def record_execution(self, execution: NativeExecution) -> None: ...

    def wait_native_terminal(
        self, command_id: str, timeout_s: float, stop: threading.Event
    ) -> NativeExecution: ...

    def wait_stop_settled(self, command_ids: Sequence[str], timeout_s: float) -> None: ...

    def robot_time_ns(self) -> int: ...

    def publish_status(
        self, phase: str, episode_id: str, reason: str, record_directory: str
    ) -> None: ...

    def record_topics(self) -> dict[str, str]: ...

    def log_error(self, message: str) -> None: ...

    def notify(self) -> None: ...


class NativeEpisodeRunner:
    """Own one worker, recorder, failure latch and native control session."""

    def __init__(
        self,
        configuration: NativeConfiguration,
        output: Path,
        io: NativeEpisodeIO,
        control_factory: Callable[[threading.Event, Callable[[str], None]], NativeControl],
        recorder_factory: Callable[[str, dict[str, object], dict[str, str]], Recorder]
        | None = None,
        log: object | None = None,
        run_id: str = "",
        initial_episode_id: str | None = None,
    ) -> None:
        self._configuration = configuration
        self._output = output
        self._io = io
        self._log = log
        self._run_id = run_id
        self._initial_episode_id = initial_episode_id
        self._recorder_factory = recorder_factory or self._new_recorder
        self._start_lock = threading.Lock()
        self._recorder_lock = threading.Lock()
        self._logging_lock = threading.Lock()
        self._logging_failure = ""
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
        self._task_id = ""
        self._task_result = TaskResult()
        self._managed_runtime: RuntimeResult | None = None
        self._frozen_task_fact: object | None = None
        self._faults = FailureLog(self._stop_requested, io.notify)
        self._control = control_factory(self._stop_requested, self.report_control_fault)

    # ---- Shared lifecycle surface (same names as the mock runner) -----------------

    @property
    def control(self) -> NativeControl:
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

    @property
    def logging_failed(self) -> bool:
        with self._logging_lock:
            return bool(self._logging_failure)

    @property
    def logging_failure(self) -> str:
        with self._logging_lock:
            return self._logging_failure

    def report_control_fault(self, reason: str) -> None:
        self._io.log_error(reason)
        self._faults.append(reason)

    def append_failure(self, reason: str) -> None:
        self._faults.append(reason)

    def report_logging_failure(self, error: BaseException) -> None:
        with self._logging_lock:
            if not self._logging_failure:
                self._logging_failure = f"{type(error).__name__}: {error}"
        self._faults.append(f"LOGGING_ERROR: {error}")

    def current_runtime_result(self) -> RuntimeResult:
        with self._start_lock:
            shutdown_requested = self._shutdown_requested.is_set()
            shutdown_reason = self._shutdown_reason
            end_requested = self._end_requested
            end_reason = self._end_reason
            failure = self._faults.reason
        if failure:
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
            return RuntimeResult(RuntimeOutcome.FAILED, shutdown_reason or "RUNTIME_SHUTDOWN")
        if self._managed_runtime is not None:
            return self._managed_runtime
        if end_requested:
            return RuntimeResult(RuntimeOutcome.STOPPED, end_reason or "EPISODE_ENDED_EARLY")
        return RuntimeResult.completed()

    def start(self, task_id: str) -> str:
        with self._start_lock:
            if self._shutdown_requested.is_set():
                raise ContractError("SHUTTING_DOWN")
            if self._worker is not None and self._worker.is_alive():
                raise ContractError("CONTROL_BUSY")
            if self.logging_failed:
                raise ContractError("LOGGING_FAILED", self.logging_failure)
            episode_id = self._initial_episode_id or f"episode-{uuid.uuid4()}"
            self._initial_episode_id = None
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
            self._task_result = TaskResult()
            self._managed_runtime = None
            self._frozen_task_fact = None
            self._done.clear()
            try:
                check_health(self._log)
            except Exception as error:
                self.report_logging_failure(error)
                raise ContractError("LOGGING_FAILED", self.failure_reason) from error
            self._worker = threading.Thread(
                target=self._run_episode, args=(episode_id, task_id), daemon=True
            )
            self._worker.start()
            return episode_id

    def end_episode(self, reason: str) -> bool:
        with self._start_lock:
            worker = self._worker
            if worker is None or not worker.is_alive():
                return False
            if self._end_requested or self._finishing.is_set():
                return True
            self._end_requested = True
            self._end_reason = reason or "EPISODE_ENDED_EARLY"
        self._faults.request_stop()
        self._emit_best_effort(
            "native.episode.end_requested",
            level="WARNING",
            reason="EPISODE_END_REQUESTED",
            fields={"reason": self._end_reason},
        )
        return True

    def request_shutdown(self, reason: str) -> None:
        with self._start_lock:
            first = not self._shutdown_requested.is_set()
            if first:
                self._shutdown_reason = reason
            self._shutdown_requested.set()
            worker = self._worker
            worker_alive = worker is not None and worker.is_alive()
            already_reported = not first and reason == self._shutdown_reason
        if not worker_alive:
            return
        if first:
            if reason in {"SIGINT", "SIGTERM"}:
                self._faults.request_stop()
            else:
                self._faults.fail(reason)
        elif not already_reported and reason not in {"SIGINT", "SIGTERM"}:
            self._faults.append(reason)

    def worker_alive(self) -> bool:
        with self._start_lock:
            worker = self._worker
            return worker is not None and worker.is_alive()

    def join_worker(self, timeout: float) -> bool:
        with self._start_lock:
            worker = self._worker
        if worker is None:
            return True
        worker.join(timeout=max(0.0, timeout))
        return not worker.is_alive()

    def shutdown_details(self) -> dict[str, str]:
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
        with self._recorder_lock:
            if self._recording_fault:
                return
            self._recording_fault = reason
        self._io.log_error(reason)
        self._faults.fail(reason)

    # ---- Worker -------------------------------------------------------------------

    def _run_episode(self, episode_id: str, task_id: str) -> None:
        try:
            self._run_episode_body(episode_id, task_id)
        except LogFailure as error:
            self.report_logging_failure(error)
            self._io.log_error(self.failure_reason)
        except Exception as error:
            self._faults.fail(f"WORKER_ERROR: {error}")
            self._io.log_error(self.failure_reason)
            self._emit_best_effort(
                "native.episode.failed", level="ERROR", reason="EPISODE_FAILED", exception=error
            )
        finally:
            self._done.set()

    def _emit_best_effort(
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

    def _run_episode_body(self, episode_id: str, task_id: str) -> None:
        try:
            self._set_status("PREPARING", episode_id)
            self._emit_best_effort(
                "native.episode.preparing", context=EventContext(episode_id=episode_id)
            )
            info = self._prepare(episode_id)
            actual_context = self._build_context(episode_id, info)
            if self._configuration.recipe.reset.enabled:
                prep_context = RunContext(
                    robot_id=actual_context.robot_id,
                    robot_profile_hash=actual_context.robot_profile_hash,
                    backend_instance_id=actual_context.backend_instance_id,
                    time_domain_id=actual_context.time_domain_id,
                    clock_epoch=actual_context.clock_epoch,
                    episode_id=f"{episode_id}-reset",
                    control_epoch=1,
                )
                self._io.bind_context(prep_context)
                self._control.acquire(prep_context)
                new_context = self._control.reset(
                    prep_context,
                    self._control.lease_id,
                    self._configuration.recipe.reset.seed,
                )
                actual_context = RunContext(
                    robot_id=new_context.robot_id,
                    robot_profile_hash=new_context.robot_profile_hash,
                    backend_instance_id=new_context.backend_instance_id,
                    time_domain_id=new_context.time_domain_id,
                    clock_epoch=new_context.clock_epoch,
                    episode_id=episode_id,
                    control_epoch=1,
                )
            with self._recorder_lock:
                self._run_context = actual_context
            policy_test_only = bool(
                self._configuration.recipe.policy is not None
                and self._configuration.recipe.policy.expected_test_only
            )
            snapshot = {
                **self._configuration.snapshot,
                "run_id": self._run_id,
                "source": str(info.backend_kind),
                "test_only": bool(info.test_only) or policy_test_only,
                "task_id": task_id,
                "run_context": asdict(actual_context),
                "reset_used": bool(self._configuration.recipe.reset.enabled),
                "reset_seed": self._configuration.recipe.reset.seed,
            }
            recorder = self._recorder_factory(episode_id, snapshot, self._io.record_topics())
            with self._recorder_lock:
                self._recorder = recorder
                self._partial_directory = str(recorder.directory)
            self._io.record_backend_info(actual_context)
            self._io.bind_context(actual_context)
            self._control.acquire(actual_context)
            self._emit_best_effort(
                "native.episode.active",
                context=EventContext(episode_id=episode_id, clock_epoch=actual_context.clock_epoch),
                fields={"task_id": task_id, "record_directory": self._current_path() or None},
            )
            self._set_status("ACTIVE", episode_id)
            if self._configuration.recipe.action_source == "policy":
                self._run_policy(episode_id, actual_context, task_id)
            elif self._configuration.recipe.action_source == "teacher":
                self._run_teacher(episode_id, actual_context, task_id)
            else:
                raise ContractError(
                    "CONFIGURATION",
                    "action_source must be 'policy' or 'teacher'",
                )
        except Exception as error:
            if isinstance(error, LogFailure):
                self.report_logging_failure(error)
            elif not (
                isinstance(error, ContractError)
                and error.reason == "EPISODE_STOPPED"
                and self._stop_requested.is_set()
            ):
                self._faults.fail(str(error))
                self._io.log_error(self.failure_reason)
                self._emit_best_effort(
                    "native.episode.failed",
                    level="ERROR",
                    reason=error.reason
                    if isinstance(error, ContractError) and error.reason
                    else "EPISODE_FAILED",
                    context=EventContext(
                        episode_id=episode_id,
                        clock_epoch=self._current_clock_epoch(),
                    ),
                    exception=error,
                )
        finally:
            self._finish(episode_id)

    def _run_policy(self, episode_id: str, actual_context: RunContext, task_id: str) -> None:
        from dual_arm_runtime.native_stream import PolicyStreamRunner, follower_limits
        from dual_arm_runtime.policy_client import PolicyClient

        policy = self._configuration.recipe.policy
        if policy is None:
            raise ContractError("POLICY_CONFIGURATION", "policy settings are absent")
        snapshot = self._configuration.snapshot
        client = None
        try:
            client = PolicyClient(
                policy,
                self._configuration.profile,
                snapshot["profile"],
                snapshot["observation"],
                snapshot["action"],
                log=self._log,
            )
            client.connect()
            session_id = client.open(actual_context, f"{episode_id}-open")
            self._emit_best_effort(
                "policy.session.ready",
                context=EventContext(episode_id=episode_id, clock_epoch=actual_context.clock_epoch),
                fields={
                    "session_id": session_id,
                    "test_only": policy.expected_test_only,
                    "action_source": self._configuration.recipe.action_source,
                    "chunk_size": policy.chunk_size,
                    "action_period_ns": policy.action_period_ns,
                    "max_chunks": policy.max_chunks,
                    "target_count": policy.target_count,
                    "termination_mode": policy.termination_mode,
                },
            )
            raw_limits = self._configuration.control_document.get("max_accelerations")
            if not isinstance(raw_limits, list):
                raise ContractError("SCHEMA_MISMATCH", "control max_accelerations are absent")
            monitor = None
            if policy.termination_mode == "task":
                monitor = self._io.stack_task_client()
                if monitor is None:
                    raise ContractError(
                        "TASK_MONITOR_MISSING", "task-terminated policy has no task client"
                    )
            runner = PolicyStreamRunner(
                self._configuration,
                actual_context,
                episode_id,
                task_id,
                self._io,
                self._control,
                client,
                follower_limits(self._configuration, raw_limits),
                stop_requested=self._stop_requested,
                log=self._log,
                task_monitor=monitor,
            )
            # The chunk command is the only identity that must be stopped if an
            # unknown ACK leaves it potentially executing.
            self._sent_command_ids.append(runner.command_id)
            try:
                runner.run()
            except FirstFollowerRejected:
                if runner.chunks == 0:
                    self._sent_command_ids.remove(runner.command_id)
                raise
            finally:
                try:
                    client.close()
                except Exception as error:  # noqa: BLE001 - preserve the primary failure
                    self._emit_best_effort(
                        "policy.session.close_failed",
                        level="ERROR",
                        reason="POLICY_CLOSE_FAILED",
                        context=EventContext(
                            episode_id=episode_id,
                            clock_epoch=actual_context.clock_epoch,
                        ),
                        exception=error,
                    )
            if policy.termination_mode == "task":
                self._finish_managed_task(runner, actual_context, policy.task_sim_budget_ns)
        finally:
            if client is not None:
                try:
                    client.shutdown()
                except Exception:
                    pass

    def _run_teacher(self, episode_id: str, actual_context: RunContext, task_id: str) -> None:
        from dual_arm_runtime.native_stream import follower_limits
        from dual_arm_runtime.teacher_source import TeacherStreamRunner, TeacherTargetSource

        teacher = self._configuration.recipe.teacher
        if teacher is None:
            raise ContractError("TEACHER_CONFIGURATION", "teacher settings are absent")
        client = self._io.stack_task_client()
        if client is None:
            raise ContractError("TEACHER_DISABLED", "runtime has no stack-task client")
        raw_limits = self._configuration.control_document.get("max_accelerations")
        if not isinstance(raw_limits, list):
            raise ContractError("SCHEMA_MISMATCH", "control max_accelerations are absent")
        source = TeacherTargetSource(
            client,
            teacher,
            self._configuration.profile,
            task_seed=self._configuration.recipe.reset.seed,
            log=self._log,
        )
        runner = TeacherStreamRunner(
            self._configuration,
            actual_context,
            episode_id,
            task_id,
            self._io,
            self._control,
            source,
            follower_limits(self._configuration, raw_limits),
            stop_requested=self._stop_requested,
            log=self._log,
        )
        self._sent_command_ids.append(runner.command_id)
        try:
            runner.run()
        except FirstFollowerRejected:
            if runner.chunks == 0:
                self._sent_command_ids.remove(runner.command_id)
            raise
        finally:
            source.finish()
        self._finish_managed_task(runner, actual_context, teacher.task_sim_budget_ns)

    def _finish_managed_task(self, runner: object, context: RunContext, budget_ns: int) -> None:
        """Stop once, freeze the judge fact, then classify a normal task ending."""
        runtime_reason, _task_reason, fact = runner.task_end  # type: ignore[attr-defined]
        if not runtime_reason:
            raise ContractError(
                "TASK_NOT_TERMINAL", "managed stream ended without a frozen task fact"
            )
        self._managed_runtime = RuntimeResult(RuntimeOutcome.STOPPED, runtime_reason)
        try:
            self._control.stop_episode(runtime_reason)
        except ContractError as error:
            # A failed stop is a runtime failure, not a normal task ending.
            self._managed_runtime = None
            raise ContractError("STOP_FAILED", str(error)) from error
        if int(getattr(runner, "chunks", 0)) > 0:
            try:
                self._io.wait_native_terminal(
                    runner.command_id,  # type: ignore[attr-defined]
                    self._configuration.recipe.control.stop_timeout_s,
                    self._stop_requested,
                )
            except Exception as error:  # noqa: BLE001 - terminal truth is mandatory
                self._managed_runtime = None
                raise ContractError("TERMINAL_WAIT_FAILED", str(error)) from error
        client = self._io.stack_task_client()
        if client is not None:
            try:
                frozen = client.state(context, 0)  # type: ignore[attr-defined]
            except Exception as error:  # noqa: BLE001 - keep the stop fact, fail honestly
                self._managed_runtime = None
                raise ContractError("TASK_STATE_QUERY_FAILED", str(error)) from error
            if frozen is not None:
                fact = frozen
                self._io.record_task_state(fact)
        frozen_state = str(getattr(fact, "state", ""))
        if frozen_state == "SUCCEEDED":
            runtime_reason = "TASK_SUCCEEDED"
            self._managed_runtime = RuntimeResult(RuntimeOutcome.STOPPED, runtime_reason)
        elif runtime_reason == "TASK_SUCCEEDED":
            self._managed_runtime = None
            raise ContractError(
                "TASK_STATE_CONFLICT", "runtime observed success but the frozen judge did not"
            )
        self._frozen_task_fact = fact
        self._task_result = task_result_for_runtime(runtime_reason, fact)
        self._emit_best_effort(
            "native.task.terminated",
            reason=runtime_reason,
            context=EventContext(episode_id=context.episode_id, clock_epoch=context.clock_epoch),
            fields={
                "task_outcome": self._task_result.outcome.value,
                "task_reason": self._task_result.reason,
                "task_state": None if fact is None else str(getattr(fact, "state", "")),
                "task_state_reason": None if fact is None else str(getattr(fact, "reason", "")),
                "task_epoch": None if fact is None else int(getattr(fact, "task_epoch", 0)),
                "budget_ns": budget_ns,
                "command_id": runner.command_id,  # type: ignore[attr-defined]
            },
        )

    def _prepare(self, episode_id: str):
        _context, info = self._io.wait_backend(
            episode_id,
            self._configuration.recipe.startup_timeout_s,
            self._stop_requested,
        )
        if bool(info.test_only) != self._configuration.recipe.test_only:
            raise ContractError("UNSUPPORTED_CAPABILITY", "backend test_only differs")
        if info.backend_kind != "mujoco":
            raise ContractError("UNSUPPORTED_CAPABILITY", f"backend kind {info.backend_kind!r}")
        required = list(self._configuration.required_capabilities)
        if self._configuration.recipe.observation.reliable:
            required.append("reliable_sensor_streams_v1")
        recipe = self._configuration.recipe
        if recipe.action_source in {"policy", "teacher"}:
            required.append("bounded_target_follower_v1")
        stream = recipe.policy if recipe.action_source == "policy" else recipe.teacher
        if stream is not None and stream.termination_mode == "task":
            required.extend(("stack_task", "stack_task_state"))
        if recipe.action_source == "teacher":
            required.append("teacher_chunk")
        missing = [capability for capability in required if capability not in info.capabilities]
        if missing:
            raise ContractError(
                "UNSUPPORTED_CAPABILITY", f"missing native capabilities: {', '.join(missing)}"
            )
        self._control.wait_ready()
        return info

    def _build_context(self, episode_id: str, info) -> RunContext:
        from robot_ros_support.conversions import context_from_message

        context = context_from_message(info.context)
        control_epoch = 1
        return RunContext(
            robot_id=context.robot_id,
            robot_profile_hash=context.robot_profile_hash,
            backend_instance_id=context.backend_instance_id,
            time_domain_id=context.time_domain_id,
            clock_epoch=context.clock_epoch,
            episode_id=episode_id,
            control_epoch=control_epoch,
        )

    def _finish(self, episode_id: str) -> None:
        self._finishing.set()
        try:
            self._set_status("ENDING", episode_id)
        except Exception as error:
            self._faults.append(f"STATUS_ERROR: {error}")
        clock_epoch = self._current_clock_epoch()
        primary = self.failure_reason
        runtime = self.current_runtime_result()
        if self._managed_runtime is not None:
            cleanup_reason = self._managed_runtime.reason
        else:
            cleanup_reason = "EPISODE_END" if self._end_requested else "RUNTIME_SHUTDOWN"
        try:
            self._control.cleanup(cleanup_reason)
        except Exception as error:
            self._faults.append(f"CLEANUP_ERROR: {error}")
        if self._sent_command_ids:
            try:
                self._io.wait_stop_settled(
                    tuple(self._sent_command_ids),
                    self._configuration.recipe.control.stop_timeout_s,
                )
            except Exception as error:
                self._faults.append(f"TERMINAL_WAIT_ERROR: {error}")
                self._emit_best_effort(
                    "native.terminal_wait_failed",
                    level="ERROR",
                    reason="TERMINAL_WAIT_FAILED",
                    context=EventContext(episode_id=episode_id, clock_epoch=clock_epoch),
                    fields={"command_count": len(self._sent_command_ids)},
                    exception=error,
                )
        self._freeze_task_fact_after_failure(episode_id, clock_epoch)
        with self._recorder_lock:
            recorder = self._recorder
            self._recorder = None
        recording_complete: bool | None = None
        recording_failure = ""
        recording_stats: RecordingStats | None = None
        runtime = self.current_runtime_result()
        if recorder is not None:
            try:
                path = recorder.finish(
                    runtime,
                    current_runtime=self.current_runtime_result,
                    task_result=self._task_result,
                )
                with self._recorder_lock:
                    self._record_directory = str(path)
                manifest = json.loads((path / "manifest.json").read_text())
                try:
                    document = task_fact_document(
                        self._frozen_task_fact,
                        self._task_result,
                        action_source=self._configuration.recipe.action_source,
                        test_only=bool(manifest.get("test_only", False)),
                    )
                    temporary = path / "task_result.json.tmp"
                    temporary.write_text(json.dumps(document, indent=2, allow_nan=False) + "\n")
                    temporary.replace(path / "task_result.json")
                except Exception as error:  # noqa: BLE001 - sidecar must not fake manifest truth
                    self._emit_best_effort(
                        "task.result.write_failed",
                        level="ERROR",
                        reason="TASK_RESULT_WRITE_FAILED",
                        context=EventContext(episode_id=episode_id, clock_epoch=clock_epoch),
                        exception=error,
                    )
                recording_complete = bool(manifest["recording"]["complete"])
                recording_failure = str(manifest["recording"]["failure_reason"])
                if not recording_complete:
                    self._faults.merge(recording_failure)
                manifest_runtime = RuntimeResult(
                    RuntimeOutcome(manifest["runtime"]["outcome"]),
                    manifest["runtime"]["reason"],
                )
                if manifest_runtime.outcome is RuntimeOutcome.FAILED:
                    self._faults.merge(manifest_runtime.reason)
                try:
                    recording_stats = recorder.stats()
                except Exception:
                    recording_stats = None
                self._emit_best_effort(
                    "recording.completed" if recording_complete else "recording.failed",
                    level="INFO" if recording_complete else "ERROR",
                    reason="RECORDING_COMPLETED" if recording_complete else "RECORDING_INCOMPLETE",
                    context=EventContext(episode_id=episode_id, clock_epoch=clock_epoch),
                    fields={
                        "path": str(path),
                        "recording_complete": recording_complete,
                        "recording_failure_reason": recording_failure or None,
                        **self._stat_fields(recording_stats),
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
                self._emit_best_effort(
                    "recording.failed",
                    level="ERROR",
                    reason="RECORDING_FAILED",
                    context=EventContext(episode_id=episode_id, clock_epoch=clock_epoch),
                    fields={
                        "path": self._current_path() or None,
                        "recording_complete": False,
                        "recording_failure_reason": recording_failure or None,
                        **self._stat_fields(recording_stats),
                    },
                    exception=error,
                )
        with self._recorder_lock:
            self._run_context = None
        runtime = self.current_runtime_result()
        self._emit_best_effort(
            "episode.finished",
            level="ERROR"
            if runtime.outcome is RuntimeOutcome.FAILED
            else "WARNING"
            if runtime.outcome in {RuntimeOutcome.INTERRUPTED, RuntimeOutcome.STOPPED}
            else "INFO",
            reason=f"EPISODE_{runtime.outcome.value.upper()}",
            context=EventContext(episode_id=episode_id, clock_epoch=clock_epoch),
            fields={
                "task_id": self._task_id,
                "record_directory": self._current_path() or None,
                "recording_complete": recording_complete,
                "recording_failure_reason": recording_failure or None,
                "runtime_outcome": runtime.outcome.value,
                "runtime_reason": runtime.reason or None,
                "task_outcome": self._task_result.outcome.value,
                "primary_reason": primary or None,
                **self._stat_fields(recording_stats),
            },
        )
        try:
            self._set_status(
                "FAULT" if runtime.outcome is RuntimeOutcome.FAILED else "IDLE",
                episode_id,
                self.failure_reason if runtime.outcome is RuntimeOutcome.FAILED else "",
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
                        "outcome": self._task_result.outcome.value,
                        "reason": self._task_result.reason or None,
                    },
                }
            ),
            flush=True,
        )

    def _freeze_task_fact_after_failure(self, episode_id: str, clock_epoch: int | None) -> None:
        """Best-effort, bounded task truth for a non-managed runtime failure.

        A managed task end already froze its own fact.  This path only covers an
        unexpected runtime failure after Start, so a failed episode can still
        carry the independent judge dimension instead of silently dropping it.
        """
        recipe = self._configuration.recipe
        stream = recipe.policy if recipe.action_source == "policy" else recipe.teacher
        if stream is None or stream.termination_mode != "task":
            return
        if self._task_result.outcome is not TaskOutcome.NOT_EVALUATED:
            return
        context = self.context
        client = self._io.stack_task_client()
        if context is None or client is None:
            return
        try:
            fact = client.state(context, 0)  # type: ignore[attr-defined]
        except Exception as error:  # noqa: BLE001 - preserve the primary runtime failure
            self._emit_best_effort(
                "native.task.state_unavailable",
                level="WARNING",
                reason="TASK_STATE_UNAVAILABLE",
                context=EventContext(episode_id=episode_id, clock_epoch=clock_epoch),
                exception=error,
            )
            fact = None
        if fact is not None:
            self._frozen_task_fact = fact
            self._io.record_task_state(fact)
            judged = task_result_from_fact(fact)
            if judged.outcome is not TaskOutcome.NOT_EVALUATED:
                self._task_result = judged
                return
        # No judge fact means the task dimension is not evaluated; the runtime
        # failure is already recorded independently in the runtime dimension.

    @staticmethod
    def _stat_fields(stats: RecordingStats | None) -> dict[str, Scalar]:
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

    def _current_clock_epoch(self) -> int | None:
        context = self.context
        return None if context is None else context.clock_epoch

    def _current_path(self) -> str:
        with self._recorder_lock:
            return self._record_directory or self._partial_directory

    def _set_status(self, phase: str, episode_id: str = "", reason: str = "") -> None:
        self._io.publish_status(phase, episode_id, reason, self.record_directory)

    def _new_recorder(
        self, episode_id: str, snapshot: dict[str, object], topics: dict[str, str]
    ) -> Recorder:
        from dual_arm_runtime.recording import EpisodeRecorder

        return EpisodeRecorder(
            self._output,
            episode_id,
            snapshot,
            self._configuration.schema_root,
            topics,
            settings=self._configuration.recipe.recording,
            run_id=str(snapshot.get("run_id", self._run_id)),
            source=str(snapshot.get("source", "")),
            test_only=bool(snapshot.get("test_only", False)),
            format_version=POLICY_EPISODE_SCHEMA_VERSION
            if self._configuration.recipe.action_source in {"policy", "teacher"}
            else NATIVE_EPISODE_SCHEMA_VERSION,
        )
