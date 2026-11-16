"""ROS service callbacks for the backend node (thin bounded adapters)."""

from __future__ import annotations

import json
import time
from typing import Any

from robot_contracts.models import ContractError
from robot_contracts.task_state import TaskState
from robot_contracts.trajectory.follower_reference import (
    FollowerReference as FollowerReferenceRecord,
)
from robot_ros_support.conversions import context_from_message, context_message, stamp, time_ns
from robot_ros_support.follower_conversions import follower_reference_message
from robot_ros_support.stream_conversions import (
    anchor_message,
    execution_feedback_message,
    stream_chunk_status_message,
    stream_command_from_message,
)
from robot_run_logging import EventContext

from robot_backend_sim.control.control_state import ControlStatus
from robot_backend_sim.runtime.protocol import chunk_feedback, execution_feedback
from robot_backend_sim.runtime.worker import BackendClosed, SubmitOutcome
from robot_backend_sim.tasks import TaskChunk, TaskPrepareResult
from robot_backend_sim.tasks.ros_conv import task_state_message

__all__ = ["ServiceAdapter"]


def _service(method: Any) -> Any:
    """One bounded exception boundary for simple request/response services."""

    async def wrapper(self: Any, request: Any, response: Any) -> Any:
        try:
            return await method(self, request, response)
        except BaseException as error:  # noqa: BLE001 - service boundary
            return self._node._failure_response(response, error)

    return wrapper


class ServiceAdapter:
    """Own every request/response service callback; no physics lives here."""

    def __init__(self, node: Any, worker: Any, config: Any) -> None:
        self._node = node
        self._worker = worker
        self._config = config

    async def _await(self, future: Any, *, submit: bool = False) -> Any:
        return await self._node._await(future, submit=submit)

    def _fail(self, response: Any, error: BaseException) -> Any:
        return self._node._failure_response(response, error)

    def _emit(self, event: str, **kwargs: Any) -> None:
        self._node._emit(event, **kwargs)

    @_service
    async def acquire(self, request: object, response: object) -> object:
        context = context_from_message(request.context)  # type: ignore[attr-defined]
        response.lease_id = await self._node._await(  # type: ignore[attr-defined]
            self._worker.acquire(
                context,
                request.owner_id,  # type: ignore[attr-defined]
                time_ns(request.duration),  # type: ignore[attr-defined]
            )
        )
        response.success = True  # type: ignore[attr-defined]
        response.reason = ""  # type: ignore[attr-defined]
        self._node._publish_info()
        return response

    @_service
    async def renew(self, request: object, response: object) -> object:
        await self._node._await(
            self._worker.renew(
                context_from_message(request.context),  # type: ignore[attr-defined]
                request.lease_id,  # type: ignore[attr-defined]
                time_ns(request.duration),  # type: ignore[attr-defined]
            )
        )
        response.success = True  # type: ignore[attr-defined]
        response.reason = ""  # type: ignore[attr-defined]
        return response

    @_service
    async def release(self, request: object, response: object) -> object:
        await self._node._await(
            self._worker.release(
                context_from_message(request.context),  # type: ignore[attr-defined]
                request.lease_id,  # type: ignore[attr-defined]
            )
        )
        response.success = True  # type: ignore[attr-defined]
        response.reason = ""  # type: ignore[attr-defined]
        return response

    @_service
    async def stop(self, request: object, response: object) -> object:
        await self._node._await(
            self._worker.stop(
                context_from_message(request.context),  # type: ignore[attr-defined]
                request.lease_id,  # type: ignore[attr-defined]
                request.reason,  # type: ignore[attr-defined]
            )
        )
        response.success = True  # type: ignore[attr-defined]
        response.reason = ""  # type: ignore[attr-defined]
        return response

    @staticmethod
    def _reject_follower(response: object, reason: str) -> object:
        response.accepted = False  # type: ignore[attr-defined]
        response.reason = reason  # type: ignore[attr-defined]
        response.has_execution = False  # type: ignore[attr-defined]
        response.has_chunk = False  # type: ignore[attr-defined]
        response.has_reference = False  # type: ignore[attr-defined]
        return response

    def _follower_failure(self, error: BaseException) -> str:
        if isinstance(error, ContractError):
            self._node._emit(
                "backend.submit_follower.rejected",
                level="WARNING",
                reason=error.reason,
                message=str(error),
            )
            return error.reason
        if isinstance(error, BackendClosed):
            return "BACKEND_CLOSED"
        self._node._emit(
            "backend.submit_follower.error",
            level="ERROR",
            reason="INTERNAL_ERROR",
            message=type(error).__name__,
            exception=error,
        )
        return "INTERNAL_ERROR"

    def _fill_follower(self, response: object, outcome: SubmitOutcome) -> object:
        response.accepted = bool(outcome.reply.accepted)  # type: ignore[attr-defined]
        response.reason = outcome.reply.reason  # type: ignore[attr-defined]
        response.has_execution = outcome.execution is not None  # type: ignore[attr-defined]
        if outcome.execution is not None and outcome.execution.snapshot is not None:
            response.execution = execution_feedback_message(  # type: ignore[attr-defined]
                execution_feedback(outcome.execution)
            )
        response.has_chunk = outcome.chunk is not None  # type: ignore[attr-defined]
        if outcome.chunk is not None:
            response.chunk = stream_chunk_status_message(  # type: ignore[attr-defined]
                chunk_feedback(outcome.chunk)
            )
        response.has_reference = outcome.reference is not None  # type: ignore[attr-defined]
        if outcome.reference is not None:
            response.reference = follower_reference_message(  # type: ignore[attr-defined]
                outcome.reference
            )
        return response

    def _emit_follower(self, record, response, outcome: SubmitOutcome) -> None:
        self._node._emit(
            "backend.submit_follower",
            message="accepted" if response.accepted else "rejected",
            reason=response.reason or None,
            context=EventContext(
                episode_id=record.trajectory.context.episode_id or None,
                command_id=record.trajectory.command_id,
                clock_epoch=record.trajectory.context.clock_epoch,
            ),
            fields={
                "command_id": record.trajectory.command_id,
                "kind": record.mode,
                "mode": "FOLLOWER",
                "idempotent": outcome.idempotent,
                "has_reference": response.has_reference,
            },
        )

    async def submit_follower(self, request: object, response: object) -> object:
        try:
            record = stream_command_from_message(request.request)  # type: ignore[attr-defined]
            if record.kind not in (2, 3):
                raise ContractError(
                    "INVALID_FOLLOWER_REQUEST",
                    "SubmitFollower accepts only STREAM_FIRST or STREAM_NEXT",
                )
            outcome = await self._node._await(self._worker.submit_follower(record), submit=True)
        except TimeoutError as error:
            return self._reject_follower(response, str(error))
        except BaseException as error:  # noqa: BLE001 - SubmitFollower has no success field
            return self._reject_follower(response, self._follower_failure(error))
        assert isinstance(outcome, SubmitOutcome)
        if outcome.reference is not None and not isinstance(
            outcome.reference, FollowerReferenceRecord
        ):
            return self._reject_follower(response, "STREAM_MODE_MISMATCH")
        self._fill_follower(response, outcome)
        self._emit_follower(record, response, outcome)
        return response

    @_service
    async def get_anchor(self, request: object, response: object) -> object:
        anchor = await self._node._await(
            self._worker.get_anchor(
                context_from_message(request.context),  # type: ignore[attr-defined]
                request.lease_id,  # type: ignore[attr-defined]
                int(request.start_sim_time_ns) if request.has_start else None,  # type: ignore[attr-defined]
            )
        )
        response.anchor = anchor_message(anchor)  # type: ignore[attr-defined]
        response.success = True  # type: ignore[attr-defined]
        response.reason = ""  # type: ignore[attr-defined]
        return response

    @_service
    async def query_command(self, request: object, response: object) -> object:
        status = await self._node._await(
            self._worker.query_command(
                context_from_message(request.context),  # type: ignore[attr-defined]
                request.command_id,  # type: ignore[attr-defined]
            )
        )
        if not isinstance(status, ControlStatus) or status.snapshot is None:
            raise ContractError("RESULT_EXPIRED", "command has no frozen status")
        response.execution = execution_feedback_message(  # type: ignore[attr-defined]
            execution_feedback(status)
        )
        response.success = True  # type: ignore[attr-defined]
        response.reason = ""  # type: ignore[attr-defined]
        return response

    @_service
    async def contact_snapshot(self, request: object, response: object) -> object:
        context = context_from_message(request.context)  # type: ignore[attr-defined]
        snapshot = await self._node._await(self._worker.contact_snapshot())
        from robot_backend_sim.ros.contact_conv import contact_snapshot_message

        response.snapshot = contact_snapshot_message(snapshot, context)  # type: ignore[attr-defined]
        response.success = True  # type: ignore[attr-defined]
        response.reason = ""  # type: ignore[attr-defined]
        return response

    @_service
    async def reset(self, request: object, response: object) -> object:
        new_context = await self._node._await(
            self._worker.reset(
                context_from_message(request.context),  # type: ignore[attr-defined]
                request.lease_id,  # type: ignore[attr-defined]
                int(request.seed),  # type: ignore[attr-defined]
            )
        )
        self._worker.clear_task_cache()
        self._node._last_task_state_seq = 0
        self._node._last_task_state_phase = ""
        response.new_context = context_message(new_context)  # type: ignore[attr-defined]
        response.success = True  # type: ignore[attr-defined]
        response.reason = ""  # type: ignore[attr-defined]
        self._node._emit(
            "backend.reset",
            message="episode reset",
            context=EventContext(clock_epoch=new_context.clock_epoch),
            fields={
                "seed": int(request.seed),  # type: ignore[attr-defined]
                "episode_id": new_context.episode_id,
            },
        )
        self._node._publish_info()
        return response

    # --------------------------------------------------------------- task API
    async def prepare_task(self, request: object, response: object) -> object:
        started = time.monotonic()
        try:
            context = context_from_message(request.context)  # type: ignore[attr-defined]
            result = await self._node._await(
                self._worker.prepare_task(
                    context,
                    request.episode_id,  # type: ignore[attr-defined]
                    request.request_id,  # type: ignore[attr-defined]
                    request.lease_id,  # type: ignore[attr-defined]
                    int(request.seed),  # type: ignore[attr-defined]
                    request.task_type,  # type: ignore[attr-defined]
                )
            )
        except BaseException as error:  # noqa: BLE001 - service boundary
            self._node._emit(
                "backend.task.prepare_failed",
                level="WARNING",
                reason=error.reason if isinstance(error, ContractError) else "INTERNAL_ERROR",
                message=type(error).__name__,
            )
            return self._node._failure_response(response, error)
        if not isinstance(result, TaskPrepareResult):
            return self._node._failure_response(
                response,
                ContractError("INVALID_TASK_PREPARE", "worker result is not a prepare fact"),
            )
        response.success = True  # type: ignore[attr-defined]
        response.reason = ""  # type: ignore[attr-defined]
        response.task_epoch = result.task_epoch  # type: ignore[attr-defined]
        response.task_type = result.task_type  # type: ignore[attr-defined]
        response.task_config_digest = result.task_config_digest  # type: ignore[attr-defined]
        response.plan_digest = result.plan_digest  # type: ignore[attr-defined]
        response.prepare_kind = result.prepare_kind  # type: ignore[attr-defined]
        response.metrics_json = json.dumps(  # type: ignore[attr-defined]
            result.metrics, sort_keys=True, separators=(",", ":"), allow_nan=False
        )
        self._node._emit(
            "backend.task.prepare",
            message="prepared",
            context=EventContext(
                episode_id=request.episode_id,  # type: ignore[attr-defined]
                clock_epoch=context.clock_epoch,
            ),
            fields={
                "task_type": result.task_type,
                "task_epoch": result.task_epoch,
                "plan_digest": result.plan_digest,
                "task_config_digest": result.task_config_digest,
                "prepare_kind": result.prepare_kind,
                "elapsed_s": time.monotonic() - started,
                "idempotent": result.idempotent,
                **result.metrics,
            },
        )
        return response

    async def start_task(self, request: object, response: object) -> object:
        try:
            context = context_from_message(request.context)  # type: ignore[attr-defined]
            result = await self._node._await(
                self._worker.start_task(
                    context,
                    request.episode_id,  # type: ignore[attr-defined]
                    request.request_id,  # type: ignore[attr-defined]
                    request.lease_id,  # type: ignore[attr-defined]
                    int(request.task_epoch),  # type: ignore[attr-defined]
                    request.plan_digest,  # type: ignore[attr-defined]
                )
            )
        except BaseException as error:  # noqa: BLE001
            return self._node._failure_response(response, error)
        if not isinstance(result, tuple) or len(result) != 2:
            return self._node._failure_response(
                response, ContractError("INVALID_TASK_START", "worker result is not a start fact")
            )
        state, idempotent = result
        if not isinstance(state, TaskState):
            return self._node._failure_response(
                response, ContractError("INVALID_TASK_START", "start fact has no state")
            )
        response.success = True  # type: ignore[attr-defined]
        response.reason = ""  # type: ignore[attr-defined]
        response.task_epoch = state.task_epoch  # type: ignore[attr-defined]
        response.plan_digest = state.plan_digest  # type: ignore[attr-defined]
        response.start_time = stamp(int(state.metric("task_start_sim_time_ns", 0)))  # type: ignore[attr-defined]
        response.state = task_state_message(state)  # type: ignore[attr-defined]
        self._node._emit(
            "backend.task.start",
            message="idempotent" if idempotent else "started",
            context=EventContext(
                episode_id=None if state.context is None else state.context.episode_id or None,
                clock_epoch=0 if state.context is None else state.context.clock_epoch,
                sim_time_ns=int(state.metric("task_start_sim_time_ns", 0)),
            ),
            fields={
                "task_type": state.task_type,
                "task_epoch": state.task_epoch,
                "plan_digest": state.plan_digest,
                "idempotent": idempotent,
            },
        )
        return response

    async def get_task_chunk(self, request: object, response: object) -> object:
        started = time.monotonic()
        try:
            context = context_from_message(request.context)  # type: ignore[attr-defined]
            handoff_time = getattr(request, "handoff_time", None)  # type: ignore[attr-defined]
            chunk = await self._node._await(
                self._worker.get_task_chunk(
                    context,
                    request.episode_id,  # type: ignore[attr-defined]
                    int(request.task_epoch),  # type: ignore[attr-defined]
                    request.request_id,  # type: ignore[attr-defined]
                    int(request.capture_seq),  # type: ignore[attr-defined]
                    time_ns(request.observation_time),  # type: ignore[attr-defined]
                    0 if handoff_time is None else time_ns(handoff_time),
                )
            )
        except BaseException as error:  # noqa: BLE001
            self._node._emit(
                "backend.task.teacher_chunk_failed",
                level="WARNING",
                reason=error.reason if isinstance(error, ContractError) else "INTERNAL_ERROR",
                message=type(error).__name__,
            )
            return self._node._failure_response(response, error)
        if not isinstance(chunk, TaskChunk):
            return self._node._failure_response(
                response, ContractError("INVALID_TEACHER_CHUNK", "worker returned no chunk")
            )
        response.success = True  # type: ignore[attr-defined]
        response.reason = ""  # type: ignore[attr-defined]
        self._worker.fill_task_chunk_response(response, chunk)
        self._node._emit(
            "backend.task.teacher_chunk",
            message=chunk.phase,
            context=EventContext(
                episode_id=context.episode_id,
                request_id=chunk.request_id,
                sim_time_ns=chunk.observation_time_ns,
            ),
            fields={
                "task_epoch": chunk.task_epoch,
                "capture_seq": chunk.capture_seq,
                "chunk_digest": chunk.chunk_digest,
                "phase": chunk.phase,
                "ik_max_position_error_m": chunk.ik_max_position_error_m,
                "ik_max_orientation_error_rad": chunk.ik_max_orientation_error_rad,
                "gate_state": chunk.gate_state,
                "gate_failure": chunk.gate_failure,
                "phase_start_sim_time_ns": chunk.phase_start_sim_time_ns,
                "phase_wait_s": chunk.phase_wait_s,
                "elapsed_s": time.monotonic() - started,
            },
        )
        return response

    async def get_task_state(self, request: object, response: object) -> object:
        try:
            context = context_from_message(request.context)  # type: ignore[attr-defined]
            state = await self._node._await(
                self._worker.get_task_state(
                    context,
                    int(request.task_epoch),  # type: ignore[attr-defined]
                )
            )
        except BaseException as error:  # noqa: BLE001
            return self._node._failure_response(response, error)
        if not isinstance(state, TaskState):
            return self._node._failure_response(
                response, ContractError("INVALID_TASK_STATE", "worker returned no task state")
            )
        if state.context is None:
            return self._node._failure_response(
                response,
                ContractError("TASK_NOT_PREPARED", "no task exists for this episode"),
            )
        response.success = True  # type: ignore[attr-defined]
        response.reason = ""  # type: ignore[attr-defined]
        response.state = task_state_message(state)  # type: ignore[attr-defined]
        return response
