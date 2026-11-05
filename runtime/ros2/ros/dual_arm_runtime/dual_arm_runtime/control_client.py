"""Lease and trajectory-action client for one runtime process.

The episode worker performs blocking acquire/execute/cleanup operations. The
node's executor timer starts and polls asynchronous renewals; cleanup may consume
an in-flight renewal future without blocking ROS callbacks.
"""

import threading
import time
import uuid
from collections.abc import Callable
from typing import Protocol, cast

import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.client import Client
from rclpy.node import Node
from rclpy.task import Future
from robot_api_msgs.action import ExecuteTrajectory
from robot_api_msgs.msg import TrajectoryCommand
from robot_api_msgs.srv import (
    AcquireControl,
    QueryCommand,
    ReleaseControl,
    RenewControl,
    Stop,
)
from robot_contracts.models import CommandStatus, ContractError, RunContext
from robot_contracts.status import validate_command_status
from robot_ros_support.conversions import (
    command_status_from_message,
    context_from_message,
    context_message,
    duration,
)
from robot_run_logging import EventContext, Scalar

from dual_arm_runtime.configuration import RuntimeRecipe
from dual_arm_runtime.run_logging import (
    EventSink,
    check_health,
    emit_best_effort,
    emit_event,
)

_ServiceRequest = (
    AcquireControl.Request
    | RenewControl.Request
    | ReleaseControl.Request
    | Stop.Request
    | QueryCommand.Request
)


class _ServiceResponse(Protocol):
    success: bool
    reason: str


class ControlClient:
    """Own the backend service/action clients, lease and renewal state.

    ``node`` is used only to create ROS entities and to read its qualified name.
    The injected recipe supplies timeouts and lease durations, ``stop_requested``
    aborts ordinary waits, and ``fault`` reports a reason back to the node.
    """

    def __init__(
        self,
        node: Node,
        recipe: RuntimeRecipe,
        stop_requested: threading.Event,
        fault: Callable[[str], None],
        *,
        log: EventSink | None = None,
    ) -> None:
        self._node = node
        self._recipe = recipe
        self._stop_requested = stop_requested
        self._fault = fault
        self._log = log
        self._root = recipe.robot_namespace.rstrip("/")
        self._group = ReentrantCallbackGroup()
        self._renew_lock = threading.Lock()
        self._renew_enabled = False
        self._renew_future: Future | None = None
        self._last_renew_ns = 0
        self._lease_context: RunContext | None = None
        self._lease_id = ""
        self._cleanup_consumed = False
        self._service_clients: dict[str, Client] = {}
        for service_type, name in (
            (AcquireControl, "acquire_control"),
            (RenewControl, "renew_control"),
            (ReleaseControl, "release_control"),
            (Stop, "stop"),
            (QueryCommand, "query_command"),
        ):
            self._service_clients[name] = node.create_client(
                service_type,
                f"{self._root}/{name}",
                callback_group=self._group,
            )
        self._action = ActionClient(
            node,
            ExecuteTrajectory,
            f"{self._root}/execute_trajectory",
            callback_group=self._group,
        )

    @property
    def lease_id(self) -> str:
        """Return the current lease for trajectory construction."""
        with self._renew_lock:
            return self._lease_id

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

    def _command_context(self, message: TrajectoryCommand) -> EventContext:
        context = context_from_message(message.context)
        return self._event_context(
            context,
            command_id=message.command_id or None,
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
        critical: bool = True,
    ) -> str | None:
        """Fault-path logging: never let a log error replace the original fault."""
        return emit_best_effort(
            self._log,
            event,
            level=level,
            reason=reason,
            context=context,
            fields=fields,
            exception=exception,
            critical=critical,
        )

    def _emit_critical(
        self,
        event: str,
        *,
        level: str = "INFO",
        reason: str | None = None,
        context: EventContext | None = None,
        fields: dict[str, Scalar] | None = None,
        exception: BaseException | None = None,
    ) -> None:
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
            # Latch the failure through the episode owner before propagating;
            # a critical intent/ack may not be silently skipped.
            self._fault(f"LOGGING_ERROR: {error}")
            raise

    def wait_action_ready(self) -> None:
        """Wait for the trajectory action server before the first command."""
        if not self._action.wait_for_server(timeout_sec=self._recipe.call_timeout_s):
            raise ContractError("NOT_READY", "Trajectory action")

    def acquire(self, context: RunContext) -> None:
        """Acquire control and enable renewal for the episode context."""
        check_health(self._log)
        request_id = f"acquire-{uuid.uuid4()}"
        self._emit_critical(
            "control.acquire_requested",
            context=self._event_context(context, request_id=request_id),
            fields={"lease_duration_ns": self._recipe.lease_duration_ns},
        )
        try:
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
        except Exception as error:
            log_error = self._safe_log(
                "control.acquire_failed",
                level="ERROR",
                reason="CONTROL_ACQUIRE_FAILED",
                context=self._event_context(context, request_id=request_id),
                exception=error,
            )
            if log_error is not None:
                self._fault(str(error))
                self._fault(f"LOGGING_ERROR: {log_error}")
            raise
        # Store the lease before emitting success: a failed acquired event must
        # never discard a lease that cleanup still has to stop and release.
        with self._renew_lock:
            self._lease_context = context
            self._lease_id = response.lease_id
            self._last_renew_ns = time.monotonic_ns()
            self._renew_future = None
            self._renew_enabled = True
            self._cleanup_consumed = False
        self._emit_critical(
            "control.acquired",
            context=self._event_context(context, request_id=request_id),
            fields={"lease_id": response.lease_id},
        )

    def _log_command_wait_error(
        self,
        error: BaseException,
        context: EventContext,
    ) -> None:
        """Classify only the explicit controlled stop as a warning."""
        if isinstance(error, ContractError) and error.reason == "EPISODE_STOPPED":
            self._safe_log(
                "command.action_stopped",
                level="WARNING",
                reason="COMMAND_STOPPED",
                context=context,
                exception=error,
            )
            return
        self._safe_log(
            "command.action_failed",
            level="ERROR",
            reason="COMMAND_ACTION_FAILED",
            context=context,
            exception=error,
        )

    def execute(self, command_message: TrajectoryCommand) -> None:
        """Send one trajectory goal and wait for its terminal result."""
        check_health(self._log)
        if self._stop_requested.is_set():
            raise ContractError("EPISODE_STOPPED", "command send stopped")
        context = self._command_context(command_message)
        try:
            handle = cast(
                ClientGoalHandle,
                self._wait_future(
                    self._action.send_goal_async(ExecuteTrajectory.Goal(command=command_message)),
                    self._recipe.call_timeout_s,
                ),
            )
        except Exception as error:
            self._log_command_wait_error(error, context)
            raise
        if not handle.accepted:
            self._emit_critical(
                "command.action_rejected",
                level="ERROR",
                reason="COMMAND_REJECTED",
                context=context,
            )
            raise ContractError("COMMAND_REJECTED")
        self._emit_critical(
            "command.action_accepted",
            reason="COMMAND_ACCEPTED",
            context=context,
        )
        try:
            result_response = cast(
                "ExecuteTrajectory.GetResult.Response",
                self._wait_future(handle.get_result_async(), self._recipe.command_timeout_s),
            )
        except Exception as error:
            self._log_command_wait_error(error, context)
            raise
        result = result_response.result
        self._emit_critical(
            "command.action_result",
            level="INFO" if result.success else "ERROR",
            reason="COMMAND_COMPLETED" if result.success else "COMMAND_FAILED",
            context=context,
            fields={
                "success": bool(result.success),
                "phase": result.phase or None,
                "backend_reason": result.reason or None,
            },
        )
        if not result.success:
            raise ContractError(result.reason or result.phase)

    def query_command(
        self,
        context: RunContext,
        command_id: str,
        *,
        expected_joint_names: tuple[str, ...],
    ) -> CommandStatus:
        """Read one command status; ``expected_joint_names`` is mandatory.

        The caller owns the profile binding and must pass the expected ordered
        joint names explicitly so a missing check is a type/runtime error rather
        than silently skipped validation.
        """
        check_health(self._log)
        event_context = self._event_context(context, command_id=command_id or None)
        self._emit_critical(
            "command.query_requested",
            reason="COMMAND_QUERY_REQUESTED",
            context=event_context,
            fields={"command_id": command_id or None},
        )
        try:
            response = cast(
                QueryCommand.Response,
                self._call(
                    "query_command",
                    QueryCommand.Request(
                        context=context_message(context),
                        command_id=command_id,
                    ),
                ),
            )
            status = command_status_from_message(response.status)
            validate_command_status(
                status,
                expected_context=context,
                expected_command_id=command_id,
                expected_joint_names=expected_joint_names,
            )
        except Exception as error:
            self._safe_log(
                "command.query_failed",
                level="ERROR",
                reason="COMMAND_QUERY_FAILED",
                context=event_context,
                fields={"command_id": command_id or None},
                exception=error,
            )
            raise
        self._emit_critical(
            "command.query_completed",
            reason="COMMAND_QUERY_COMPLETED",
            context=event_context,
            fields={
                "command_id": status.command_id,
                "phase": status.phase,
                "backend_reason": status.reason or None,
                "progress": float(status.progress),
                "command_seq": status.command_seq,
                "observation_id": status.observation_id,
            },
        )
        return status

    def renew(self) -> None:
        """Poll or start one renewal; called by the node timer on the executor thread."""
        failure = ""
        with self._renew_lock:
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
                        response = cast(_ServiceResponse, future.result())
                    except Exception as error:
                        failure = f"LEASE_RENEWAL_FAILED: {error}"
                    else:
                        if not response.success:
                            failure = response.reason or "LEASE_RENEWAL_REJECTED"
            elif time.monotonic_ns() - self._last_renew_ns >= self._recipe.renew_interval_ns:
                self._last_renew_ns = time.monotonic_ns()
                self._renew_future = self._service_clients["renew_control"].call_async(
                    RenewControl.Request(
                        context=context_message(self._lease_context),
                        lease_id=self._lease_id,
                        duration=duration(self._recipe.lease_duration_ns),
                    )
                )
        if failure:
            # Successful renewals are deliberately quiet; a failure must be
            # visible as well as reported through the episode failure path.
            log_error = self._safe_log(
                "control.renew_failed",
                level="ERROR",
                reason="CONTROL_RENEW_FAILED",
                context=self._event_context(lease_context),
                fields={"failure": failure},
            )
            self._fault(failure)
            if log_error is not None:
                self._fault(f"LOGGING_ERROR: {log_error}")

    def cleanup(self) -> None:
        """Stop and release once, independently of the episode stop flag."""
        with self._renew_lock:
            self._renew_enabled = False
            if self._cleanup_consumed:
                return
            self._cleanup_consumed = True
            pending_renewal, self._renew_future = self._renew_future, None
            context, self._lease_context = self._lease_context, None
            lease_id, self._lease_id = self._lease_id, ""
        if context is None or not lease_id:
            return
        failures: list[str] = []

        def log(
            step: str,
            event: str,
            *,
            request_id: str | None = None,
            level: str = "INFO",
            reason: str | None = None,
            fields: dict[str, Scalar] | None = None,
        ) -> None:
            error = self._safe_log(
                event,
                level=level,
                reason=reason,
                context=self._event_context(context, request_id=request_id),
                fields={"step": step, **(fields or {})},
            )
            if error is not None:
                failures.append(f"logging {step}: {error}")

        def service_step(
            step: str,
            event_prefix: str,
            operation: Callable[[], _ServiceResponse],
        ) -> None:
            request_id = f"{step}-{uuid.uuid4()}"
            request_fields = {"lease_id": lease_id}
            if step == "stop":
                request_fields["reason"] = "EPISODE_END"
            log(
                step,
                f"control.{event_prefix}_requested",
                reason=f"CONTROL_{step.upper()}_REQUESTED",
                request_id=request_id,
                fields=request_fields,
            )
            try:
                response = operation()
            except Exception as error:
                failures.append(f"{step}: {error}")
                error_text = self._safe_log(
                    f"control.{event_prefix}_failed",
                    level="ERROR",
                    reason=f"CONTROL_{step.upper()}_FAILED",
                    context=self._event_context(context, request_id=request_id),
                    exception=error,
                )
                if error_text is not None:
                    failures.append(f"logging {step} failure: {error_text}")
            else:
                error_text = self._safe_log(
                    f"control.{event_prefix}_completed",
                    reason=f"CONTROL_{step.upper()}_COMPLETED",
                    context=self._event_context(context, request_id=request_id),
                    fields={
                        "success": bool(response.success),
                        "backend_reason": response.reason or None,
                    },
                )
                if error_text is not None:
                    failures.append(f"logging {step} response: {error_text}")

        if pending_renewal is not None:
            renewal_request_id = f"renew-{uuid.uuid4()}"
            log(
                "renew",
                "control.renew_wait_requested",
                reason="CONTROL_RENEW_REQUESTED",
                request_id=renewal_request_id,
            )

            def wait_for_renewal() -> _ServiceResponse:
                response = cast(
                    _ServiceResponse,
                    self._wait_future(
                        pending_renewal,
                        self._recipe.call_timeout_s,
                        abort_on_stop=False,
                    ),
                )
                if not response.success:
                    raise ContractError(response.reason or "RENEWAL_REJECTED")
                return response

            try:
                wait_for_renewal()
            except Exception as error:
                failures.append(f"renew: {error}")
                error_text = self._safe_log(
                    "control.renew_failed",
                    level="ERROR",
                    reason="CONTROL_RENEW_FAILED",
                    context=self._event_context(context, request_id=renewal_request_id),
                    exception=error,
                )
                if error_text is not None:
                    failures.append(f"logging renew failure: {error_text}")
            else:
                error_text = self._safe_log(
                    "control.renew_completed",
                    reason="CONTROL_RENEW_COMPLETED",
                    context=self._event_context(context, request_id=renewal_request_id),
                )
                if error_text is not None:
                    failures.append(f"logging renew response: {error_text}")

        service_step(
            "stop",
            "stop",
            lambda: cast(
                _ServiceResponse,
                self._call(
                    "stop",
                    Stop.Request(
                        context=context_message(context),
                        lease_id=lease_id,
                        reason="EPISODE_END",
                    ),
                    cleanup=True,
                ),
            ),
        )
        service_step(
            "release",
            "release",
            lambda: cast(
                _ServiceResponse,
                self._call(
                    "release_control",
                    ReleaseControl.Request(
                        context=context_message(context),
                        lease_id=lease_id,
                    ),
                    cleanup=True,
                ),
            ),
        )
        if failures:
            self._fault("CONTROL_CLEANUP_FAILED: " + "; ".join(failures))

    def _wait_future(
        self,
        future: Future,
        timeout: float,
        *,
        abort_on_stop: bool = True,
    ) -> object:
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise ContractError("CALL_TIMEOUT")
            if not rclpy.ok() or (abort_on_stop and self._stop_requested.is_set()):
                raise ContractError("EPISODE_STOPPED")
            time.sleep(0.01)
        return future.result()

    def _call(
        self,
        name: str,
        request: _ServiceRequest,
        *,
        cleanup: bool = False,
    ) -> _ServiceResponse:
        client = self._service_clients[name]
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
        if not response.success:
            raise ContractError(response.reason)
        return response
