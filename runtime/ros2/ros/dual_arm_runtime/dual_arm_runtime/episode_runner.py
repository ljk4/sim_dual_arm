"""One streaming episode: reset, policy/teacher chunks, stop and record.

This module owns only lifecycle and sequencing.  ROS entities and callbacks stay
in ``runtime_node``; message conversion stays in the shared ROS support layer and
lease/command transport stays in ``robot_control_client``.  Policy/teacher stream
execution lives in :mod:`episode_streams`; recording finalization lives in
:mod:`episode_finalization`.
"""

from __future__ import annotations

import threading
import uuid
from collections.abc import Callable
from pathlib import Path

from robot_contracts.episodes import (
    DIAGNOSTIC_EPISODE_SCHEMA_VERSION,
    POLICY_EPISODE_SCHEMA_VERSION,
    RuntimeOutcome,
    RuntimeResult,
    TaskResult,
)
from robot_contracts.models import ContractError, RunContext
from robot_run_logging import EventContext, LogFailure, Scalar

from dual_arm_runtime.episode_runner_io import (
    ControlPort,
    EpisodeIO,
    FailureLog,
    Recorder,
)
from dual_arm_runtime.run_logging import check_health, emit_best_effort
from dual_arm_runtime.runtime_config import RuntimeConfiguration


class EpisodeRunner:
    """Own one worker, recorder, failure latch and control session."""

    def __init__(
        self,
        configuration: RuntimeConfiguration,
        output: Path,
        io: EpisodeIO,
        control_factory: Callable[[threading.Event, Callable[[str], None]], ControlPort],
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
        self._last_stream_command_id = ""
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

    @property
    def control(self) -> ControlPort:
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
            self._last_stream_command_id = ""
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
            "episode.end_requested",
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
                "episode.failed", level="ERROR", reason="EPISODE_FAILED", exception=error
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
            actual_context = self._start_episode(episode_id, task_id)
            self._dispatch_stream(episode_id, actual_context, task_id)
        except Exception as error:
            self._handle_episode_error(episode_id, error)
        finally:
            self._finish(episode_id)

    def _start_episode(self, episode_id: str, task_id: str) -> RunContext:
        self._set_status("PREPARING", episode_id)
        self._emit_best_effort("episode.preparing", context=EventContext(episode_id=episode_id))
        info = self._prepare(episode_id)
        actual_context = self._build_context(episode_id, info)
        if self._configuration.recipe.reset.enabled:
            actual_context = self._reset_episode(episode_id, actual_context)
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
            "run_context": actual_context.canonical_dict(),
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
            "episode.active",
            context=EventContext(episode_id=episode_id, clock_epoch=actual_context.clock_epoch),
            fields={"task_id": task_id, "record_directory": self._current_path() or None},
        )
        self._set_status("ACTIVE", episode_id)
        return actual_context

    def _reset_episode(self, episode_id: str, actual_context: RunContext) -> RunContext:
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
        return RunContext(
            robot_id=new_context.robot_id,
            robot_profile_hash=new_context.robot_profile_hash,
            backend_instance_id=new_context.backend_instance_id,
            time_domain_id=new_context.time_domain_id,
            clock_epoch=new_context.clock_epoch,
            episode_id=episode_id,
            control_epoch=1,
        )

    def _dispatch_stream(
        self, episode_id: str, actual_context: RunContext, task_id: str
    ) -> None:
        from dual_arm_runtime.episode_streams import run_policy, run_teacher

        action_source = self._configuration.recipe.action_source
        if action_source == "policy":
            run_policy(self, episode_id, actual_context, task_id)
        elif action_source == "teacher":
            run_teacher(self, episode_id, actual_context, task_id)
        else:
            raise ContractError("CONFIGURATION", "action_source must be 'policy' or 'teacher'")

    def _handle_episode_error(self, episode_id: str, error: Exception) -> None:
        if isinstance(error, LogFailure):
            self.report_logging_failure(error)
            return
        if (
            isinstance(error, ContractError)
            and error.reason == "EPISODE_STOPPED"
            and self._stop_requested.is_set()
        ):
            return
        self._faults.fail(str(error))
        self._io.log_error(self.failure_reason)
        self._emit_best_effort(
            "episode.failed",
            level="ERROR",
            reason=(
                error.reason
                if isinstance(error, ContractError) and error.reason
                else "EPISODE_FAILED"
            ),
            context=EventContext(
                episode_id=episode_id,
                clock_epoch=self._current_clock_epoch(),
            ),
            exception=error,
        )

    def _prepare(self, episode_id: str) -> object:
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
            required.extend(("task_plugin", "task_state"))
        if recipe.action_source == "teacher":
            required.append("teacher_chunk")
        missing = [capability for capability in required if capability not in info.capabilities]
        if missing:
            raise ContractError(
                "UNSUPPORTED_CAPABILITY", f"missing backend capabilities: {', '.join(missing)}"
            )
        self._control.wait_ready()
        return info

    def _build_context(self, episode_id: str, info: object) -> RunContext:
        from robot_ros_support.conversions import context_from_message

        context = context_from_message(info.context)
        return RunContext(
            robot_id=context.robot_id,
            robot_profile_hash=context.robot_profile_hash,
            backend_instance_id=context.backend_instance_id,
            time_domain_id=context.time_domain_id,
            clock_epoch=context.clock_epoch,
            episode_id=episode_id,
            control_epoch=1,
        )

    def _finish(self, episode_id: str) -> None:
        from dual_arm_runtime.offline.episode_finalization import finish

        finish(self, episode_id)

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
            else DIAGNOSTIC_EPISODE_SCHEMA_VERSION,
        )


__all__ = ["EpisodeRunner"]
