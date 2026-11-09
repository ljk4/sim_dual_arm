"""Lease and native-command client for the diagnostic runtime.

Every call validates the shared native DTO again after ROS conversion.  A
submit timeout or an explicit ``OUTCOME_UNKNOWN`` keeps the same command ID and
queries it; it never retries with a new identity.  Lease renewal is polled by the
node executor timer, so waiting for a command result cannot stop renewal.
"""

from __future__ import annotations

import threading
import time
import uuid
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol, cast

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client
from rclpy.node import Node
from rclpy.task import Future
from robot_api_msgs.srv import (
    AcquireControl,
    GetNativeAnchor,
    QueryNativeCommand,
    ReleaseControl,
    RenewControl,
    ResetSimulation,
    Stop,
    SubmitFollower,
)
from robot_contracts.action_chunks import ChunkAnchor
from robot_contracts.follower_reference import (
    FollowerReference,
    FollowerValidationLimits,
)
from robot_contracts.follower_reference import (
    validate as validate_follower_reference,
)
from robot_contracts.models import ContractError, RobotProfile, RunContext
from robot_contracts.native import (
    NATIVE_STREAM_FIRST,
    NATIVE_STREAM_NEXT,
    NativeExecution,
    NativeRequest,
)
from robot_ros_support.conversions import context_from_message, context_message, duration
from robot_ros_support.follower_conversions import follower_reference_from_message
from robot_ros_support.native_conversions import (
    anchor_from_message,
    native_chunk_status_from_message,
    native_command_message,
    native_execution_from_message,
)
from robot_run_logging import EventContext, Scalar

from dual_arm_runtime.native_config import NativeRecipe
from dual_arm_runtime.native_errors import FirstFollowerRejected
from dual_arm_runtime.run_logging import EventSink, check_health, emit_best_effort, emit_event

_ServiceResponse = (
    AcquireControl.Response
    | RenewControl.Response
    | ReleaseControl.Response
    | Stop.Response
    | QueryNativeCommand.Response
    | ResetSimulation.Response
)


class _SuccessResponse(Protocol):
    success: bool
    reason: str


@dataclass(frozen=True)
class NativeFollowerSubmitResult:
    """One accepted bounded-follower block and its protected raw record."""

    request: NativeRequest
    reference: FollowerReference
    execution: NativeExecution
    queried: bool
    submit_attempts: int


def _unknown_reason(reason: str) -> bool:
    return reason == "OUTCOME_UNKNOWN" or reason.startswith("OUTCOME_UNKNOWN")


class NativeControlClient:
    """Own the native service clients, one lease and command confirmation."""

    def __init__(
        self,
        node: Node,
        recipe: NativeRecipe,
        profile: RobotProfile,
        max_accelerations: tuple[float, ...],
        stop_requested: threading.Event,
        fault: Callable[[str], None],
        *,
        follower_limits: FollowerValidationLimits | None = None,
        log: EventSink | None = None,
    ) -> None:
        self._node = node
        self._recipe = recipe
        self._profile = profile
        self._max_accelerations = max_accelerations
        self._stop_requested = stop_requested
        self._fault = fault
        self._follower_limits = follower_limits
        self._log = log
        self._root = recipe.robot_namespace.rstrip("/")
        self._group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._renew_enabled = False
        self._renew_future: Future | None = None
        self._last_renew_ns = 0
        self._lease_context: RunContext | None = None
        self._lease_id = ""
        self._cleanup_consumed = False
        self._stream_origins: dict[str, NativeRequest] = {}
        # One origin is retained per chunk until the episode ends.  A 30 s task
        # at 20 Hz needs 120; derive the bound from the configured budget and
        # never let a fixed 64-entry cache truncate a valid task.
        max_chunks = 128 if recipe.policy is None else recipe.policy.max_chunks
        self._stream_origin_capacity = min(512, max(128, max_chunks + 16))
        self._clients: dict[str, Client] = {}
        for service, name in (
            (AcquireControl, "acquire_control"),
            (RenewControl, "renew_control"),
            (ReleaseControl, "release_control"),
            (Stop, "stop"),
            (SubmitFollower, "submit_follower"),
            (GetNativeAnchor, "get_native_anchor"),
            (QueryNativeCommand, "query_native_command"),
            (ResetSimulation, "reset_simulation"),
        ):
            self._clients[name] = node.create_client(
                service, f"{self._root}/{name}", callback_group=self._group
            )

    @property
    def lease_id(self) -> str:
        with self._lock:
            return self._lease_id

    @property
    def stop_requested(self) -> threading.Event:
        return self._stop_requested

    @property
    def lease_context(self) -> RunContext | None:
        with self._lock:
            return self._lease_context

    def _event_context(
        self,
        context: RunContext,
        *,
        request_id: str | None = None,
        command_id: str | None = None,
        sim_time_ns: int | None = None,
    ) -> EventContext:
        return EventContext(
            episode_id=context.episode_id or None,
            request_id=request_id,
            command_id=command_id,
            clock_epoch=context.clock_epoch,
            sim_time_ns=sim_time_ns,
        )

    def _safe_log(
        self,
        event: str,
        *,
        level: str = "INFO",
        reason: str | None = None,
        context: EventContext | None = None,
        fields: dict[str, Scalar] | None = None,
        exception: BaseException | None = None,
    ) -> str | None:
        return emit_best_effort(
            self._log,
            event,
            level=level,
            reason=reason,
            context=context,
            fields=fields,
            exception=exception,
            critical=True,
        )

    def _emit(self, event: str, *, reason: str | None = None, **kwargs: object) -> None:
        try:
            emit_event(self._log, event, reason=reason, critical=True, **kwargs)  # type: ignore[arg-type]
        except Exception as error:
            self._fault(f"LOGGING_ERROR: {error}")
            raise

    def wait_ready(self) -> None:
        for name, client in self._clients.items():
            if not client.wait_for_service(timeout_sec=self._recipe.call_timeout_s):
                raise ContractError("NOT_READY", f"native service {name}")

    # ---- Lease -------------------------------------------------------------------

    def acquire(self, context: RunContext) -> str:
        check_health(self._log)
        request_id = f"native-acquire-{uuid.uuid4()}"
        self._emit(
            "native.control.acquire_requested",
            reason="NATIVE_ACQUIRE_REQUESTED",
            context=self._event_context(context, request_id=request_id),
            fields={"lease_duration_ns": self._recipe.lease_duration_ns},
        )
        response = cast(
            AcquireControl.Response,
            self._call(
                "acquire_control",
                AcquireControl.Request(
                    context=context_message(context),
                    owner_id=self._node.get_fully_qualified_name(),
                    duration=duration(self._recipe.lease_duration_ns),
                ),
            ),
        )
        with self._lock:
            self._lease_context = context
            self._lease_id = response.lease_id
            self._last_renew_ns = time.monotonic_ns()
            self._renew_future = None
            self._renew_enabled = True
            self._cleanup_consumed = False
            self._stream_origins.clear()
        self._emit(
            "native.control.acquired",
            reason="NATIVE_ACQUIRED",
            context=self._event_context(context, request_id=request_id),
            fields={"lease_id": response.lease_id},
        )
        return response.lease_id

    def renew(self) -> None:
        failure = ""
        lease_context = None
        with self._lock:
            lease_context = self._lease_context
            if not self._renew_enabled or not self._lease_id or lease_context is None:
                return
            if self._renew_future is not None:
                if not self._renew_future.done():
                    if time.monotonic_ns() - self._last_renew_ns > self._recipe.lease_duration_ns:
                        failure = "LEASE_RENEWAL_TIMEOUT"
                else:
                    future, self._renew_future = self._renew_future, None
                    try:
                        response = cast(_SuccessResponse, future.result())
                    except Exception as error:
                        failure = f"LEASE_RENEWAL_FAILED: {error}"
                    else:
                        if not response.success:
                            failure = response.reason or "LEASE_RENEWAL_REJECTED"
            elif time.monotonic_ns() - self._last_renew_ns >= self._recipe.renew_interval_ns:
                self._last_renew_ns = time.monotonic_ns()
                self._renew_future = self._clients["renew_control"].call_async(
                    RenewControl.Request(
                        context=context_message(lease_context),
                        lease_id=self._lease_id,
                        duration=duration(self._recipe.lease_duration_ns),
                    )
                )
        if failure:
            log_error = self._safe_log(
                "native.control.renew_failed",
                level="ERROR",
                reason="LEASE_RENEWAL_FAILED",
                context=self._event_context(lease_context),
                fields={"failure": failure},
            )
            self._fault(failure)
            if log_error is not None:
                self._fault(f"LOGGING_ERROR: {log_error}")

    def discard_lease(self) -> None:
        """Forget a lease already consumed by a successful reset."""
        with self._lock:
            self._renew_enabled = False
            pending, self._renew_future = self._renew_future, None
            self._lease_context = None
            self._lease_id = ""
            self._cleanup_consumed = True
        if pending is not None:
            pending.cancel()

    # ---- Native commands ----------------------------------------------------------

    def query(
        self,
        context: RunContext,
        command_id: str,
        *,
        request: NativeRequest | None = None,
        ignore_stop: bool = False,
    ) -> NativeExecution:
        if not command_id:
            raise ContractError("INVALID_COMMAND", "command_id must not be empty")
        response = cast(
            QueryNativeCommand.Response,
            self._call(
                "query_native_command",
                QueryNativeCommand.Request(context=context_message(context), command_id=command_id),
                cleanup=ignore_stop,
            ),
        )
        execution = native_execution_from_message(response.execution)
        if request is not None:
            self._validate_execution(request, execution)
        else:
            if execution.context != context or execution.command_id != command_id:
                raise ContractError("STALE_CONTEXT", "native query identity differs")
        return execution

    def _validate_execution(self, request: NativeRequest, execution: NativeExecution) -> None:
        # Overall execution keeps the first block's observation identity even
        # while later blocks are pending or active. Each block is checked below
        # against its own NativeChunkStatus, never against the active sequence.
        if request.kind in (NATIVE_STREAM_FIRST, NATIVE_STREAM_NEXT):
            request = self._stream_origins.get(request.trajectory.command_id, request)
        trajectory = request.trajectory
        if (
            execution.context != trajectory.context
            or execution.command_id != trajectory.command_id
            or execution.command_seq != trajectory.command_seq
            or execution.observation_id != trajectory.observation_id
            or execution.joint_names != trajectory.joint_names
        ):
            raise ContractError("STALE_CONTEXT", "native execution identity differs")

    # ---- Bounded follower ---------------------------------------------------------

    def get_anchor(self, context: RunContext, start_sim_time_ns: int | None = None) -> ChunkAnchor:
        """Ask the backend for its fixed absolute hand-off anchor.

        With no explicit start the backend applies the accepted stream initial
        lead.  A successor may only repeat the currently fixed active end.
        """
        if context != self.lease_context:
            raise ContractError("STALE_CONTEXT", "anchor request does not own the lease")
        response = cast(
            GetNativeAnchor.Response,
            self._call(
                "get_native_anchor",
                GetNativeAnchor.Request(
                    context=context_message(context),
                    lease_id=self.lease_id,
                    has_start=start_sim_time_ns is not None,
                    start_sim_time_ns=0 if start_sim_time_ns is None else start_sim_time_ns,
                ),
            ),
        )
        anchor = anchor_from_message(response.anchor)
        if anchor.robot_profile_hash != self._profile.digest:
            raise ContractError("PROFILE_MISMATCH", "anchor profile differs")
        if context.robot_profile_hash != self._profile.digest:
            raise ContractError("PROFILE_MISMATCH", "anchor request profile differs")
        return anchor

    def submit_follower(self, request: NativeRequest) -> NativeFollowerSubmitResult:
        """Submit one block; an unknown ACK is queried and retried with the same ID.

        The command ID *and* the derived chunk sequence stay unchanged.  A legal
        backend retry returns the original accepted_sim/wall times and digest,
        so the caller never invents a new identity to hide an unknown ACK.
        """
        if not isinstance(request, NativeRequest):
            raise ContractError("INVALID_FOLLOWER_REQUEST", "request must be a NativeRequest")
        if self._follower_limits is None:
            raise ContractError("FOLLOWER_DISABLED", "follower limits are not configured")
        check_health(self._log)
        if self._stop_requested.is_set():
            raise ContractError("EPISODE_STOPPED", "follower submit stopped")
        context = request.trajectory.context
        command_id = request.trajectory.command_id
        if context != self.lease_context or request.trajectory.lease_id != self.lease_id:
            raise ContractError("STALE_CONTEXT", "follower request does not own the current lease")
        origin = self._stream_origins.get(command_id)
        if request.kind == NATIVE_STREAM_FIRST:
            if origin is not None and origin != request:
                raise ContractError("COMMAND_ID_CONFLICT", "stream origin changed")
            if origin is None and len(self._stream_origins) >= self._stream_origin_capacity:
                raise ContractError(
                    "CONTROL_BUSY",
                    f"stream origin cache is full ({self._stream_origin_capacity})",
                )
            self._stream_origins.setdefault(command_id, request)
        elif origin is None:
            raise ContractError("STALE_STREAM", "successor has no local stream origin")
        event_context = self._event_context(context, command_id=command_id)
        self._emit(
            "native.follower.submit_requested",
            reason="NATIVE_FOLLOWER_SUBMIT_REQUESTED",
            context=event_context,
            fields={
                "kind": request.mode,
                "command_seq": request.trajectory.command_seq,
                "chunk_seq": 0 if request.anchor is None else request.anchor.predecessor_seq + 1,
                "observation_id": request.trajectory.observation_id,
                "target_count": request.target_count,
            },
        )
        attempts = 1 + self._recipe.control.submit_query_retries
        unknown_reason = ""
        queried = False
        for attempt in range(1, attempts + 1):
            try:
                response = cast(
                    SubmitFollower.Response,
                    self._call(
                        "submit_follower",
                        SubmitFollower.Request(request=native_command_message(request)),
                        require_success=False,
                    ),
                )
            except ContractError as error:
                if error.reason == "EPISODE_STOPPED":
                    raise
                unknown_reason = f"OUTCOME_UNKNOWN: {error}"
                response = None
            except Exception as error:
                unknown_reason = f"OUTCOME_UNKNOWN: {error}"
                response = None
            if response is not None:
                if response.accepted:
                    return self._follower_accepted(
                        request, response, queried=queried, attempts=attempt
                    )
                if not _unknown_reason(response.reason):
                    rejection = (
                        FirstFollowerRejected
                        if attempt == 1 and not response.has_execution
                        else ContractError
                    )
                    raise rejection(
                        response.reason or "FOLLOWER_REJECTED", "follower submit rejected"
                    )
                unknown_reason = response.reason or "OUTCOME_UNKNOWN"
            # Unknown ACK: confirm the same command/chunk identity exists, then
            # resend the identical request so idempotence returns the original
            # FollowerReference rather than a new acceptance fact.
            try:
                self.query(context, command_id, request=request, ignore_stop=True)
                queried = True
            except ContractError as error:
                if error.reason == "EPISODE_STOPPED":
                    raise
                unknown_reason = f"{unknown_reason}; query: {error}"
            if attempt < attempts:
                time.sleep(self._recipe.control.poll_interval_s)
        raise ContractError(
            "OUTCOME_UNKNOWN",
            f"follower command {command_id} could not be confirmed: {unknown_reason}",
        )

    def _follower_accepted(
        self,
        request: NativeRequest,
        response: SubmitFollower.Response,
        *,
        queried: bool,
        attempts: int,
    ) -> NativeFollowerSubmitResult:
        context = request.trajectory.context
        command_id = request.trajectory.command_id
        if not response.has_reference:
            raise ContractError(
                "INVALID_FOLLOWER_REFERENCE", "accepted follower block has no reference"
            )
        reference = follower_reference_from_message(response.reference)
        self._validate_follower_reference(request, reference)
        if not response.has_execution:
            execution = self.query(context, command_id, request=request, ignore_stop=True)
        else:
            execution = native_execution_from_message(response.execution)
            self._validate_execution(request, execution)
        if not response.has_chunk:
            raise ContractError(
                "INVALID_FOLLOWER_RECEIPT", "accepted follower block has no chunk status"
            )
        chunk = native_chunk_status_from_message(response.chunk)
        trajectory = request.trajectory
        if (
            chunk.context != trajectory.context
            or chunk.command_id != command_id
            or chunk.command_seq != trajectory.command_seq
            or chunk.observation_id != trajectory.observation_id
            or chunk.chunk_seq != reference.chunk_seq
            or chunk.accepted_sim_time_ns != reference.accepted_sim_time_ns
            or chunk.accepted_wall_time_ns != reference.accepted_wall_time_ns
        ):
            raise ContractError(
                "INVALID_FOLLOWER_RECEIPT",
                "per-block acceptance identity or times differ from its reference",
            )
        self._emit(
            "native.follower.accepted",
            reason="NATIVE_FOLLOWER_ACCEPTED",
            context=self._event_context(context, command_id=command_id),
            fields={
                "queried": queried,
                "attempts": attempts,
                "chunk_seq": reference.chunk_seq,
                "reference_digest": reference.reference_digest,
                "phase": execution.phase,
                "accepted_sim_time_ns": reference.accepted_sim_time_ns,
                "accepted_wall_time_ns": reference.accepted_wall_time_ns,
            },
        )
        return NativeFollowerSubmitResult(request, reference, execution, queried, attempts)

    def _validate_follower_reference(
        self, request: NativeRequest, reference: FollowerReference
    ) -> None:
        if self._follower_limits is None:
            raise ContractError("FOLLOWER_DISABLED", "follower limits are not configured")
        if reference.request != request:
            raise ContractError("REFERENCE_REQUEST_MISMATCH", "accepted follower request differs")
        try:
            validate_follower_reference(reference, self._profile, self._follower_limits)
        except ContractError:
            raise
        except Exception as error:
            raise ContractError(
                "INVALID_FOLLOWER_REFERENCE", f"follower validation failed: {error}"
            ) from error

    # ---- Reset and cleanup --------------------------------------------------------

    def reset(self, context: RunContext, lease_id: str, seed: int) -> RunContext:
        if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
            raise ContractError("INVALID_SEED", "seed must be a non-negative integer")
        if context != self.lease_context or lease_id != self.lease_id:
            raise ContractError("STALE_CONTEXT", "reset does not own the current lease")
        response = cast(
            ResetSimulation.Response,
            self._call(
                "reset_simulation",
                ResetSimulation.Request(
                    context=context_message(context), lease_id=lease_id, seed=seed
                ),
                cleanup=True,
            ),
        )
        new_context = context_from_message(response.new_context)
        if new_context.clock_epoch <= context.clock_epoch:
            raise ContractError("STALE_CONTEXT", "reset did not advance clock_epoch")
        if new_context.episode_id or new_context.control_epoch:
            raise ContractError(
                "INVALID_CONTEXT", "reset must return a fresh unbound episode/control identity"
            )
        self.discard_lease()
        self._emit(
            "native.reset.completed",
            reason="NATIVE_RESET_COMPLETED",
            context=EventContext(clock_epoch=new_context.clock_epoch),
            fields={"seed": seed},
        )
        return new_context

    def stop_episode(self, reason: str) -> None:
        """Issue one real Stop for a managed task end without releasing the lease.

        The caller still waits for the command's terminal fact and later calls
        ``cleanup``; this is not an early lease release and never fakes a final
        chunk completion.
        """
        with self._lock:
            context, lease_id = self._lease_context, self._lease_id
        if context is None or not lease_id:
            raise ContractError("STALE_CONTEXT", "stop has no active lease")
        check_health(self._log)
        response = cast(
            _SuccessResponse,
            self._call(
                "stop",
                Stop.Request(
                    context=context_message(context),
                    lease_id=lease_id,
                    reason=reason,
                ),
                cleanup=True,
            ),
        )
        self._emit(
            "native.control.stop_completed",
            reason="NATIVE_TASK_STOP_COMPLETED",
            context=self._event_context(context),
            fields={
                "success": bool(response.success),
                "backend_reason": response.reason,
                "stop_reason": reason,
            },
        )

    def cleanup(self, reason: str = "EPISODE_END") -> None:
        """Stop and release once, waiting for a pending renewal but not for stop facts."""
        with self._lock:
            self._renew_enabled = False
            if self._cleanup_consumed:
                return
            self._cleanup_consumed = True
            pending, self._renew_future = self._renew_future, None
            context, self._lease_context = self._lease_context, None
            lease_id, self._lease_id = self._lease_id, ""
        if context is None or not lease_id:
            return
        failures: list[str] = []
        if pending is not None:
            try:
                cast(
                    _SuccessResponse,
                    self._wait_future(pending, self._recipe.call_timeout_s, abort_on_stop=False),
                )
            except Exception as error:
                failures.append(f"renew: {error}")
        for step, operation in (
            (
                "stop",
                lambda: self._call(
                    "stop",
                    Stop.Request(
                        context=context_message(context), lease_id=lease_id, reason=reason
                    ),
                    cleanup=True,
                ),
            ),
            (
                "release",
                lambda: self._call(
                    "release_control",
                    ReleaseControl.Request(context=context_message(context), lease_id=lease_id),
                    cleanup=True,
                ),
            ),
        ):
            request_id = f"native-{step}-{uuid.uuid4()}"
            try:
                response = cast(_SuccessResponse, operation())
            except Exception as error:
                failures.append(f"{step}: {error}")
                log_error = self._safe_log(
                    f"native.control.{step}_failed",
                    level="ERROR",
                    reason=f"NATIVE_{step.upper()}_FAILED",
                    context=self._event_context(context, request_id=request_id),
                    exception=error,
                )
                if log_error is not None:
                    failures.append(f"logging {step}: {log_error}")
            else:
                log_error = self._safe_log(
                    f"native.control.{step}_completed",
                    reason=f"NATIVE_{step.upper()}_COMPLETED",
                    context=self._event_context(context, request_id=request_id),
                    fields={"success": bool(response.success), "backend_reason": response.reason},
                )
                if log_error is not None:
                    failures.append(f"logging {step}: {log_error}")
        if failures:
            self._fault("CONTROL_CLEANUP_FAILED: " + "; ".join(failures))

    # ---- Transport ----------------------------------------------------------------

    def _wait_future(self, future: Future, timeout: float, *, abort_on_stop: bool = True) -> object:
        deadline = time.monotonic() + timeout
        completed = threading.Event()

        def wake(_future: Future) -> None:
            # Executor callbacks only signal; they never spin, log or clean up.
            completed.set()

        future.add_done_callback(wake)
        if future.done():
            completed.set()
        while not future.done():
            if time.monotonic() > deadline:
                raise ContractError("CALL_TIMEOUT")
            if not rclpy.ok() or (abort_on_stop and self._stop_requested.is_set()):
                raise ContractError("EPISODE_STOPPED")
            # poll_interval bounds only stop/deadline checks; an ACK wakes us sooner.
            remaining = deadline - time.monotonic()
            completed.wait(min(self._recipe.control.poll_interval_s, max(0.0, remaining)))
        return future.result()

    def _call(
        self,
        name: str,
        request: object,
        *,
        cleanup: bool = False,
        require_success: bool = True,
    ) -> _ServiceResponse:
        client = self._clients[name]
        if not client.wait_for_service(timeout_sec=self._recipe.call_timeout_s):
            raise ContractError("NOT_READY", name)
        response = cast(
            _ServiceResponse,
            self._wait_future(
                client.call_async(request),
                self._recipe.call_timeout_s,
                abort_on_stop=not cleanup,
            ),
        )
        if require_success and not response.success:  # type: ignore[union-attr]
            raise ContractError(response.reason or "NATIVE_REJECTED", name)  # type: ignore[union-attr]
        return response
