"""Bounded ROS client for the 033 prepare/start/chunk/state services.

Only wire conversion and service deadlines live here.  Physical task
evaluation and teacher planning stay in the backend; the runtime never imports
MuJoCo or mink and never reads object truth from a live model.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.task import Future
from robot_api_msgs.srv import (
    GetStackTaskState,
    GetTeacherChunk,
    PrepareStackTask,
    StartStackTask,
)
from robot_contracts.models import ContractError, RunContext
from robot_ros_support.conversions import context_message, stamp, time_ns

from dual_arm_runtime.run_logging import EventSink, emit_best_effort

__all__ = ["PrepareFact", "StackTaskClient", "TeacherChunkFact"]

# Upper bound on one interruptible wait slice.  The prepare retry cadence remains
# the configured poll interval; this only bounds stop/shutdown detection latency.
_CANCEL_CHECK_S = 0.05


@dataclass(frozen=True, slots=True)
class PrepareFact:
    task_epoch: int
    task_config_digest: str
    plan_digest: str
    prepare_kind: str
    planned_duration_s: float
    key_pose_count: int
    collision_segment_count: int
    ik_max_position_error_m: float
    ik_max_orientation_error_rad: float


@dataclass(frozen=True, slots=True)
class TeacherChunkFact:
    task_epoch: int
    capture_seq: int
    plan_digest: str
    chunk_digest: str
    observation_time_ns: int
    source_time_ns: int
    robot_profile_hash: str
    phase: str
    times_from_observation_ns: tuple[int, ...]
    positions: tuple[tuple[float, ...], ...]
    phase_codes: tuple[int, ...]
    gate_state: str
    gate_failure: str
    phase_start_sim_time_ns: int
    phase_wait_s: float
    ik_max_position_error_m: float
    ik_max_orientation_error_rad: float


class StackTaskClient:
    """Own the four task service clients and their bounded waits."""

    def __init__(
        self,
        node: Node,
        root: str,
        *,
        call_timeout_s: float,
        prepare_timeout_s: float,
        start_timeout_s: float,
        chunk_timeout_s: float,
        poll_interval_s: float,
        log: EventSink | None = None,
        stop_requested: threading.Event | None = None,
    ) -> None:
        self._node = node
        self._root = root.rstrip("/")
        self._call_timeout_s = call_timeout_s
        self._timeouts = {
            "prepare": prepare_timeout_s,
            "start": start_timeout_s,
            "chunk": chunk_timeout_s,
            "state": call_timeout_s,
        }
        self._poll_interval_s = poll_interval_s
        self._log = log
        self._stop = stop_requested or threading.Event()
        self._group = ReentrantCallbackGroup()
        self._clients = {
            "prepare": node.create_client(
                PrepareStackTask,
                f"{self._root}/prepare_stack_task",
                callback_group=self._group,
            ),
            "start": node.create_client(
                StartStackTask,
                f"{self._root}/start_stack_task",
                callback_group=self._group,
            ),
            "chunk": node.create_client(
                GetTeacherChunk,
                f"{self._root}/get_teacher_chunk",
                callback_group=self._group,
            ),
            "state": node.create_client(
                GetStackTaskState,
                f"{self._root}/get_stack_task_state",
                callback_group=self._group,
            ),
        }
        self._lock = threading.Lock()
        self.prepare_fact: PrepareFact | None = None
        self.start_sim_time_ns = 0
        self.frozen_state: object | None = None

    def wait_ready(self) -> None:
        for name, client in self._clients.items():
            if not client.wait_for_service(timeout_sec=self._call_timeout_s):
                raise ContractError("NOT_READY", f"stack task service {name}")

    def prepare(
        self,
        context: RunContext,
        episode_id: str,
        request_id: str,
        lease_id: str,
        seed: int,
    ) -> PrepareFact:
        started = time.monotonic()
        deadline = started + self._timeouts["prepare"]
        request = PrepareStackTask.Request(
            context=context_message(context),
            episode_id=episode_id,
            request_id=request_id,
            lease_id=lease_id,
            seed=seed,
        )
        attempts = 0
        waiting_logged = False
        last_rejection: ContractError | None = None
        while True:
            attempts += 1
            try:
                response = self._call("prepare", request, deadline=deadline)
            except ContractError as error:
                if error.reason == "TASK_NOT_QUIESCENT":
                    last_rejection = error
                elif error.reason == "CALL_TIMEOUT" and last_rejection is not None:
                    # The shared deadline expired while the retry RPC was in
                    # flight; keep the last quiescence rejection as the cause.
                    self._prepare_timeout(
                        attempts=attempts,
                        started=started,
                        last_rejection=last_rejection,
                        request_id=request_id,
                    )
                else:
                    raise
                if time.monotonic() >= deadline:
                    self._prepare_timeout(
                        attempts=attempts,
                        started=started,
                        last_rejection=last_rejection,
                        request_id=request_id,
                    )
                if not waiting_logged:
                    # One entry per prepare call, not one per poll interval.
                    self.emit(
                        "task.prepare_waiting",
                        reason="TASK_NOT_QUIESCENT",
                        message="stack task prepare waiting for a still robot",
                        fields={
                            "request_id": request_id,
                            "episode_id": episode_id,
                            "attempt": attempts,
                            "prepare_timeout_s": self._timeouts["prepare"],
                            "poll_interval_s": self._poll_interval_s,
                        },
                    )
                    waiting_logged = True
                self._wait_before_retry(deadline)
                if time.monotonic() >= deadline:
                    self._prepare_timeout(
                        attempts=attempts,
                        started=started,
                        last_rejection=last_rejection,
                        request_id=request_id,
                    )
                continue
            fact = PrepareFact(
                int(response.task_epoch),
                str(response.task_config_digest),
                str(response.plan_digest),
                str(response.prepare_kind),
                float(response.planned_duration_s),
                int(response.key_pose_count),
                int(response.collision_segment_count),
                float(response.ik_max_position_error_m),
                float(response.ik_max_orientation_error_rad),
            )
            if not fact.plan_digest or not fact.task_config_digest:
                raise ContractError("INVALID_TASK_PREPARE", "prepare fact has no digest")
            if fact.prepare_kind not in {"teacher", "evaluation_only"}:
                raise ContractError(
                    "INVALID_TASK_PREPARE", f"unknown prepare_kind {fact.prepare_kind!r}"
                )
            if waiting_logged:
                self.emit(
                    "task.prepare_succeeded",
                    message="stack task prepare succeeded after quiescence wait",
                    fields={
                        "request_id": request_id,
                        "attempts": attempts,
                        "elapsed_s": time.monotonic() - started,
                        "last_rejection": (
                            last_rejection.reason if last_rejection is not None else ""
                        ),
                    },
                )
            with self._lock:
                self.prepare_fact = fact
            return fact

    def _wait_before_retry(self, deadline: float) -> None:
        """Sleep one retry interval, bounded by the shared prepare deadline."""
        retry_at = min(time.monotonic() + self._poll_interval_s, deadline)
        while True:
            now = time.monotonic()
            if now >= retry_at:
                return
            if not rclpy.ok():
                raise ContractError(
                    "EPISODE_STOPPED", "stack task prepare during shutdown"
                )
            if self._stop.is_set():
                raise ContractError("EPISODE_STOPPED", "stack task prepare stopped")
            # The retry cadence stays _poll_interval_s; short slices only make
            # stop/shutdown checks responsive when the interval is long.
            pause = min(retry_at - now, _CANCEL_CHECK_S)
            if pause <= 0.0:
                return
            if self._stop.wait(pause):
                raise ContractError("EPISODE_STOPPED", "stack task prepare stopped")

    def _prepare_timeout(
        self,
        *,
        attempts: int,
        started: float,
        last_rejection: ContractError,
        request_id: str,
    ) -> None:
        elapsed = time.monotonic() - started
        self.emit(
            "task.prepare_timeout",
            level="WARNING",
            reason="PREPARE_TIMEOUT",
            message="stack task prepare budget expired while waiting for quiescence",
            fields={
                "request_id": request_id,
                "attempts": attempts,
                "elapsed_s": elapsed,
                "last_rejection": last_rejection.reason,
            },
        )
        raise ContractError(
            "PREPARE_TIMEOUT",
            f"stack task prepare budget expired after {attempts} attempts "
            f"in {elapsed:.3f}s; last rejection: {last_rejection.reason}",
        ) from last_rejection

    def start(
        self,
        context: RunContext,
        episode_id: str,
        request_id: str,
        lease_id: str,
        task_epoch: int,
        plan_digest: str,
    ) -> int:
        response = self._call(
            "start",
            StartStackTask.Request(
                context=context_message(context),
                episode_id=episode_id,
                request_id=request_id,
                lease_id=lease_id,
                task_epoch=task_epoch,
                plan_digest=plan_digest,
            ),
        )
        start_ns = time_ns(response.start_time)
        if start_ns <= 0:
            raise ContractError("INVALID_TASK_START", "start time is not a positive sim time")
        with self._lock:
            self.start_sim_time_ns = start_ns
            self.frozen_state = response.state
        return start_ns

    def chunk(
        self,
        context: RunContext,
        episode_id: str,
        task_epoch: int,
        request_id: str,
        capture_seq: int,
        observation_time_ns: int,
        *,
        robot_profile_hash: str,
        chunk_size: int,
        period_ns: int,
    ) -> TeacherChunkFact:
        response = self._call(
            "chunk",
            GetTeacherChunk.Request(
                context=context_message(context),
                episode_id=episode_id,
                task_epoch=task_epoch,
                request_id=request_id,
                capture_seq=capture_seq,
                observation_time=stamp(observation_time_ns),
            ),
        )
        if int(response.task_epoch) != task_epoch:
            raise ContractError("TEACHER_CONTEXT_MISMATCH", "teacher task epoch changed")
        if int(response.capture_seq) != capture_seq:
            raise ContractError("TEACHER_CAPTURE_MISMATCH", "teacher capture differs")
        if time_ns(response.observation_time) != observation_time_ns:
            raise ContractError(
                "TEACHER_FEEDBACK_MISMATCH", "teacher observation time differs"
            )
        if str(response.robot_profile_hash) != robot_profile_hash:
            raise ContractError("TEACHER_PROFILE_MISMATCH", "teacher profile differs")
        if int(response.point_count) != chunk_size or int(response.joint_count) != 16:
            raise ContractError("TEACHER_CHUNK_SHAPE", "teacher chunk is not 60x16")
        flat = tuple(float(value) for value in response.positions)
        if len(flat) != chunk_size * 16:
            raise ContractError("TEACHER_CHUNK_SHAPE", "teacher positions are not 960-wide")
        times = tuple(int(value) for value in response.time_from_observation_ns)
        if len(times) != chunk_size:
            raise ContractError("TEACHER_CHUNK_CLOCK", "teacher time grid is incomplete")
        for index, value in enumerate(times):
            if value != (index + 1) * period_ns:
                raise ContractError(
                    "TEACHER_CHUNK_CLOCK", "teacher time grid is not a uniform 50 ms"
                )
        positions = tuple(
            flat[point * 16 : point * 16 + 16] for point in range(chunk_size)
        )
        phase_codes = tuple(int(value) for value in response.phase_codes)
        if len(phase_codes) != chunk_size:
            raise ContractError("TEACHER_CHUNK_SHAPE", "teacher phase codes are incomplete")
        return TeacherChunkFact(
            int(response.task_epoch),
            int(response.capture_seq),
            str(response.plan_digest),
            str(response.chunk_digest),
            time_ns(response.observation_time),
            time_ns(response.source_time),
            str(response.robot_profile_hash),
            str(response.phase),
            times,
            positions,
            phase_codes,
            str(response.gate_state),
            str(response.gate_failure),
            time_ns(response.phase_start_time),
            float(response.phase_wait_s),
            float(response.ik_max_position_error_m),
            float(response.ik_max_orientation_error_rad),
        )

    def state(self, context: RunContext, task_epoch: int = 0) -> object:
        response = self._call(
            "state",
            GetStackTaskState.Request(
                context=context_message(context), task_epoch=task_epoch
            ),
        )
        state = response.state
        if context != _state_context(state):
            raise ContractError("TASK_CONTEXT_MISMATCH", "task state context differs")
        with self._lock:
            self.frozen_state = state
        return state

    def _call(self, name: str, request: object, *, deadline: float | None = None):
        client = self._clients[name]
        if deadline is None:
            if not client.wait_for_service(timeout_sec=self._call_timeout_s):
                raise ContractError("NOT_READY", f"stack task service {name}")
            deadline = time.monotonic() + self._timeouts[name]
        else:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise ContractError("CALL_TIMEOUT", f"stack task {name}")
            if not client.wait_for_service(
                timeout_sec=min(remaining, self._call_timeout_s)
            ):
                raise ContractError("NOT_READY", f"stack task service {name}")
            if time.monotonic() >= deadline:
                raise ContractError("CALL_TIMEOUT", f"stack task {name}")
        future: Future = client.call_async(request)
        completed = threading.Event()

        def wake(_future: Future) -> None:
            # Executor callbacks only signal; they never spin, log or clean up.
            completed.set()

        future.add_done_callback(wake)
        if future.done():
            completed.set()
        while not future.done():
            if time.monotonic() > deadline:
                future.cancel()
                raise ContractError("CALL_TIMEOUT", f"stack task {name}")
            if not rclpy.ok():
                raise ContractError("EPISODE_STOPPED", f"stack task {name} during shutdown")
            if self._stop.is_set() and name in {"prepare", "start", "chunk"}:
                future.cancel()
                raise ContractError("EPISODE_STOPPED", f"stack task {name} stopped")
            # poll_interval bounds only stop/deadline checks; an ACK wakes us sooner.
            remaining = deadline - time.monotonic()
            completed.wait(min(self._poll_interval_s, max(0.0, remaining)))
        try:
            response = future.result()
        except Exception as error:
            raise ContractError("TASK_RPC_FAILED", f"{name}: {error}") from error
        if not response.success:
            reason = str(response.reason) or "TASK_REJECTED"
            raise ContractError(reason, f"{name}: {reason}")
        return response

    def emit(self, event: str, **kwargs: object) -> None:
        error = emit_best_effort(self._log, event, critical=True, **kwargs)  # type: ignore[arg-type]
        if error is not None:
            raise ContractError("LOGGING_ERROR", error)


def _state_context(state: object) -> RunContext:
    from robot_ros_support.conversions import context_from_message

    return context_from_message(state.context)  # type: ignore[attr-defined]
