"""Bounded ROS client for the optional prepare/start/state task lifecycle.

Only wire conversion and service deadlines live here.  Physical task evaluation
and teacher planning stay in the backend; the runtime never imports MuJoCo or
mink and never reads object truth from a live model.  A scenario Teacher chunk
client is layered on top by :mod:`dual_arm_runtime.simulation.teacher_client`.
"""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.task import Future
from robot_contracts.models import ContractError, RunContext
from robot_ros_support.conversions import context_message, time_ns

from dual_arm_runtime.run_logging import EventSink, emit_best_effort

if TYPE_CHECKING:
    from dual_arm_runtime.runtime_config import RuntimeRecipe
    from dual_arm_runtime.simulation.teacher_client import TeacherTaskClient

__all__ = ["PrepareFact", "TaskClient"]

_CANCEL_CHECK_S = 0.05
TASK_STATE_MESSAGE_TYPE = "sim_api_msgs/msg/TaskState"


def create_task_client(
    node: Node, root: str, recipe: RuntimeRecipe, stop_requested: threading.Event,
    log: EventSink | None,
) -> TaskClient | TeacherTaskClient:
    teacher = recipe.teacher
    timeout = recipe.call_timeout_s
    client = TaskClient(
        node,
        root,
        call_timeout_s=timeout,
        prepare_timeout_s=timeout if teacher is None else teacher.prepare_timeout_s,
        start_timeout_s=timeout if teacher is None else teacher.start_timeout_s,
        poll_interval_s=0.02 if teacher is None else teacher.poll_interval_s,
        log=log,
        stop_requested=stop_requested,
    )
    if teacher is None:
        return client
    from dual_arm_runtime.simulation.teacher_client import TeacherTaskClient

    assert recipe.teacher_api is not None
    return TeacherTaskClient(
        client, node, root,
        capability=recipe.teacher_api,
        chunk_timeout_s=teacher.chunk_deadline_s,
        call_timeout_s=timeout,
        poll_interval_s=teacher.poll_interval_s,
        log=log,
        stop_requested=stop_requested,
    )


@dataclass(frozen=True, slots=True)
class PrepareFact:
    task_epoch: int
    task_type: str
    task_config_digest: str
    plan_digest: str
    prepare_kind: str
    metrics: dict[str, object] = field(default_factory=dict)

    def metric(self, name: str, default: object = None) -> object:
        return self.metrics.get(name, default)

    @property
    def planned_duration_s(self) -> float:
        return float(self.metrics.get("planned_duration_s", 0.0))

    @property
    def key_pose_count(self) -> int:
        return int(self.metrics.get("key_pose_count", 0))

    @property
    def collision_segment_count(self) -> int:
        return int(self.metrics.get("collision_segment_count", 0))

    @property
    def ik_max_position_error_m(self) -> float:
        return float(self.metrics.get("ik_max_position_error_m", 0.0))

    @property
    def ik_max_orientation_error_rad(self) -> float:
        return float(self.metrics.get("ik_max_orientation_error_rad", 0.0))


class TaskClient:
    """Own the prepare/start/state task services and their bounded waits."""

    def __init__(
        self,
        node: Node,
        root: str,
        *,
        call_timeout_s: float,
        prepare_timeout_s: float,
        start_timeout_s: float,
        poll_interval_s: float,
        log: EventSink | None = None,
        stop_requested: threading.Event | None = None,
    ) -> None:
        from sim_api_msgs.srv import GetTaskState, PrepareTask, StartTask

        self._node = node
        self._root = root.rstrip("/")
        self._call_timeout_s = call_timeout_s
        self._timeouts = {
            "prepare": prepare_timeout_s, "start": start_timeout_s,
            "state": call_timeout_s,
        }
        self._poll_interval_s = poll_interval_s
        self._log = log
        self._stop = stop_requested or threading.Event()
        self._group = ReentrantCallbackGroup()
        self._clients = {
            "prepare": node.create_client(
                PrepareTask, f"{self._root}/prepare_task", callback_group=self._group
            ),
            "start": node.create_client(
                StartTask, f"{self._root}/start_task", callback_group=self._group
            ),
            "state": node.create_client(
                GetTaskState, f"{self._root}/get_task_state", callback_group=self._group
            ),
        }
        self._lock = threading.Lock()
        self.prepare_fact: PrepareFact | None = None
        self.start_sim_time_ns = 0
        self.frozen_state: object | None = None

    def wait_ready(self) -> None:
        for name, client in self._clients.items():
            if not client.wait_for_service(timeout_sec=self._call_timeout_s):
                raise ContractError("NOT_READY", f"task service {name}")

    @staticmethod
    def _prepare_fact(response: object) -> PrepareFact:
        try:
            metrics = json.loads(response.metrics_json or "{}")
        except (TypeError, ValueError) as error:
            raise ContractError("INVALID_TASK_PREPARE", "metrics_json is not JSON") from error
        if not isinstance(metrics, dict):
            raise ContractError("INVALID_TASK_PREPARE", "prepare metrics is not an object")
        fact = PrepareFact(
            int(response.task_epoch),
            str(response.task_type),
            str(response.task_config_digest),
            str(response.plan_digest),
            str(response.prepare_kind),
            metrics,
        )
        if not fact.plan_digest or not fact.task_config_digest:
            raise ContractError("INVALID_TASK_PREPARE", "prepare fact has no digest")
        if not fact.task_type:
            raise ContractError("INVALID_TASK_PREPARE", "prepare fact has no task type")
        if fact.prepare_kind not in {"teacher", "evaluation_only"}:
            raise ContractError(
                "INVALID_TASK_PREPARE", f"unknown prepare_kind {fact.prepare_kind!r}"
            )
        return fact

    def prepare(
        self,
        context: RunContext,
        episode_id: str,
        request_id: str,
        lease_id: str,
        seed: int,
        task_type: str = "",
    ) -> PrepareFact:
        from sim_api_msgs.srv import PrepareTask
        started = time.monotonic()
        deadline = started + self._timeouts["prepare"]
        request = PrepareTask.Request(
            context=context_message(context), episode_id=episode_id, request_id=request_id,
            lease_id=lease_id, seed=seed, task_type=task_type,
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
                    self._prepare_timeout(
                        attempts=attempts, started=started,
                        last_rejection=last_rejection, request_id=request_id,
                    )
                else:
                    raise
                if time.monotonic() >= deadline:
                    self._prepare_timeout(
                        attempts=attempts, started=started,
                        last_rejection=last_rejection, request_id=request_id,
                    )
                if not waiting_logged:
                    self.emit(
                        "task.prepare_waiting", reason="TASK_NOT_QUIESCENT",
                        message="task prepare waiting for a still robot",
                        fields={
                            "request_id": request_id, "episode_id": episode_id,
                            "attempt": attempts,
                            "prepare_timeout_s": self._timeouts["prepare"],
                            "poll_interval_s": self._poll_interval_s,
                        },
                    )
                    waiting_logged = True
                self._wait_before_retry(deadline)
                if time.monotonic() >= deadline:
                    self._prepare_timeout(
                        attempts=attempts, started=started,
                        last_rejection=last_rejection, request_id=request_id,
                    )
                continue
            fact = self._prepare_fact(response)
            if waiting_logged:
                self.emit(
                    "task.prepare_succeeded",
                    message="task prepare succeeded after quiescence wait",
                    fields={
                        "request_id": request_id, "attempts": attempts,
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
                raise ContractError("EPISODE_STOPPED", "task prepare during shutdown")
            if self._stop.is_set():
                raise ContractError("EPISODE_STOPPED", "task prepare stopped")
            pause = min(retry_at - now, _CANCEL_CHECK_S)
            if pause <= 0.0:
                return
            if self._stop.wait(pause):
                raise ContractError("EPISODE_STOPPED", "task prepare stopped")

    def _prepare_timeout(
        self, *, attempts: int, started: float, last_rejection: ContractError, request_id: str,
    ) -> None:
        elapsed = time.monotonic() - started
        self.emit(
            "task.prepare_timeout", level="WARNING", reason="PREPARE_TIMEOUT",
            message="task prepare budget expired while waiting for quiescence",
            fields={
                "request_id": request_id, "attempts": attempts, "elapsed_s": elapsed,
                "last_rejection": last_rejection.reason,
            },
        )
        raise ContractError(
            "PREPARE_TIMEOUT",
            f"task prepare budget expired after {attempts} attempts "
            f"in {elapsed:.3f}s; last rejection: {last_rejection.reason}",
        ) from last_rejection

    def start(
        self, context: RunContext, episode_id: str, request_id: str, lease_id: str,
        task_epoch: int, plan_digest: str,
    ) -> int:
        from sim_api_msgs.srv import StartTask
        response = self._call(
            "start",
            StartTask.Request(
                context=context_message(context), episode_id=episode_id,
                request_id=request_id, lease_id=lease_id, task_epoch=task_epoch,
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

    def state(self, context: RunContext, task_epoch: int = 0) -> object:
        from sim_api_msgs.srv import GetTaskState
        response = self._call(
            "state", GetTaskState.Request(context=context_message(context), task_epoch=task_epoch)
        )
        state = response.state
        if context != _state_context(state):
            raise ContractError("TASK_CONTEXT_MISMATCH", "task state context differs")
        with self._lock:
            self.frozen_state = state
        return state

    def _call(self, name: str, request: object, *, deadline: float | None = None):
        return self.invoke(
            self._clients[name],
            request,
            name=name,
            timeout_s=self._timeouts[name],
            deadline=deadline,
        )

    def invoke(
        self,
        client: object,
        request: object,
        *,
        name: str,
        timeout_s: float,
        deadline: float | None = None,
    ):
        """Bounded call shared with the scenario Teacher extension adapter."""
        if deadline is None:
            if not client.wait_for_service(timeout_sec=self._call_timeout_s):  # type: ignore[attr-defined]
                raise ContractError("NOT_READY", f"task service {name}")
            deadline = time.monotonic() + timeout_s
        else:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise ContractError("CALL_TIMEOUT", f"task {name}")
            if not client.wait_for_service(  # type: ignore[attr-defined]
                timeout_sec=min(remaining, self._call_timeout_s)
            ):
                raise ContractError("NOT_READY", f"task service {name}")
            if time.monotonic() >= deadline:
                raise ContractError("CALL_TIMEOUT", f"task {name}")
        future: Future = client.call_async(request)  # type: ignore[attr-defined]
        completed = threading.Event()

        def wake(_future: Future) -> None:
            completed.set()

        future.add_done_callback(wake)
        if future.done():
            completed.set()
        while not future.done():
            if time.monotonic() > deadline:
                future.cancel()
                raise ContractError("CALL_TIMEOUT", f"task {name}")
            if not rclpy.ok():
                raise ContractError("EPISODE_STOPPED", f"task {name} during shutdown")
            if self._stop.is_set() and name in {"prepare", "start", "chunk"}:
                future.cancel()
                raise ContractError("EPISODE_STOPPED", f"task {name} stopped")
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


def task_state_message_type():
    """Resolve the optional simulation task wire type only on a task path."""
    from sim_api_msgs.msg import TaskState
    return TaskState


def reset_service_type():
    """Resolve the simulation reset type when a reset is explicitly requested."""
    from sim_api_msgs.srv import ResetSimulation
    return ResetSimulation
