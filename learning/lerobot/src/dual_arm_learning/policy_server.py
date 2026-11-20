"""Bounded gRPC transport and process lifecycle for a ROS-free policy service."""

from __future__ import annotations

import json
import re
import signal
import threading
import time
import uuid
from collections.abc import Callable
from concurrent import futures
from pathlib import Path
from typing import Any

import grpc
from robot_run_logging import EventContext, EventLogger, LogFailure

from .adapters.base import PolicyModelAdapter
from .policy_config import PolicyServiceConfig
from .policy_service import PolicyService
from .policy_types import PolicyError, PolicyProvider

_STATUS = {
    "RESOURCE_EXHAUSTED": grpc.StatusCode.RESOURCE_EXHAUSTED,
    "UNKNOWN_SESSION": grpc.StatusCode.NOT_FOUND,
    "NOT_READY": grpc.StatusCode.UNAVAILABLE,
    "PROVIDER_ERROR": grpc.StatusCode.INTERNAL,
    "DEADLINE_EXCEEDED": grpc.StatusCode.DEADLINE_EXCEEDED,
    "MODEL_DIGEST_MISMATCH": grpc.StatusCode.FAILED_PRECONDITION,
    "STALE_CONTEXT": grpc.StatusCode.FAILED_PRECONDITION,
    "INVALID_PREDICTION": grpc.StatusCode.DATA_LOSS,
    "PROVIDER_MISMATCH": grpc.StatusCode.FAILED_PRECONDITION,
}
_SAFE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$")


def _event_context(request: Any) -> EventContext:
    """Invalid wire identifiers must not break the structured log encoder."""
    context = getattr(request, "context", None)
    episode_id = getattr(context, "episode_id", "")
    request_id = getattr(request, "request_id", "") or getattr(request, "open_request_id", "")
    return EventContext(
        episode_id=episode_id if _SAFE_ID.fullmatch(episode_id) else None,
        request_id=request_id if _SAFE_ID.fullmatch(request_id) else None,
        clock_epoch=getattr(context, "clock_epoch", None),
    )


class PolicyServicer:
    """Validate deadlines and log every outcome without logging sensor payloads."""

    def __init__(
        self,
        service: PolicyService,
        config: PolicyServiceConfig,
        log: EventLogger | None = None,
    ) -> None:
        self.service, self.config, self.log = service, config, log
        self.failed = threading.Event()
        self._slots = threading.BoundedSemaphore(config.max_workers)

    def _emit(self, event: str, request: Any, **fields: Any) -> None:
        if self.log is not None:
            self.log.check_health()
            self.log.emit(
                event,
                context=_event_context(request),
                fields=fields,
                critical=True,
            )

    def _invoke(
        self,
        method: str,
        request: Any,
        context: grpc.ServicerContext,
        deadline_s: float,
        handler: Callable[[int], Any],
    ) -> Any:
        if not self._slots.acquire(blocking=False):
            context.abort(grpc.StatusCode.RESOURCE_EXHAUSTED, "service concurrency saturated")
        started = time.monotonic_ns()
        reason = "OK"
        try:
            remaining = context.time_remaining()
            if remaining is not None:
                deadline_s = min(deadline_s, remaining)
            if deadline_s <= 0:
                raise PolicyError("DEADLINE_EXCEEDED", "request already expired")
            self._emit("policy.rpc.started", request, method=method, deadline_s=deadline_s)
            result = handler(round(deadline_s * 1_000_000_000))
            self._emit(
                "policy.rpc.finished",
                request,
                method=method,
                reason="OK",
                duration_ns=time.monotonic_ns() - started,
            )
            return result
        except PolicyError as error:
            reason = error.reason
            self._emit(
                "policy.rpc.rejected",
                request,
                method=method,
                reason=reason,
                duration_ns=time.monotonic_ns() - started,
            )
            context.abort(_STATUS.get(reason, grpc.StatusCode.INVALID_ARGUMENT), str(error))
        except LogFailure:
            self.failed.set()
            context.abort(grpc.StatusCode.INTERNAL, "LOGGING_ERROR")
        except Exception as error:
            self._emit(
                "policy.rpc.failed",
                request,
                method=method,
                reason=type(error).__name__,
                duration_ns=time.monotonic_ns() - started,
            )
            context.abort(grpc.StatusCode.INTERNAL, "POLICY_INTERNAL_ERROR")
        finally:
            self._slots.release()

    def GetPolicyInfo(self, request: Any, context: grpc.ServicerContext) -> Any:
        return self._invoke(
            "GetPolicyInfo",
            request,
            context,
            self.config.open_deadline_s,
            lambda _: self.service.info_reply(str(request.model_bundle_digest)),
        )

    def OpenSession(self, request: Any, context: grpc.ServicerContext) -> Any:
        return self._invoke(
            "OpenSession",
            request,
            context,
            self.config.open_deadline_s,
            lambda limit: self.service.open_session(request, limit),
        )

    def Predict(self, request: Any, context: grpc.ServicerContext) -> Any:
        return self._invoke(
            "Predict",
            request,
            context,
            self.config.predict_deadline_s,
            lambda limit: self.service.predict(request, limit),
        )

    def ResetSession(self, request: Any, context: grpc.ServicerContext) -> Any:
        return self._invoke(
            "ResetSession",
            request,
            context,
            self.config.open_deadline_s,
            lambda _: self.service.reset_session(request),
        )

    def CloseSession(self, request: Any, context: grpc.ServicerContext) -> Any:
        return self._invoke(
            "CloseSession",
            request,
            context,
            self.config.close_deadline_s,
            lambda _: self.service.close_session(request),
        )


class PolicyServer:
    """Own the real server; the optional logger is owned by the caller."""

    def __init__(
        self,
        config: PolicyServiceConfig,
        provider: PolicyProvider | PolicyModelAdapter,
        log: EventLogger | None = None,
        *,
        prediction_directory: Path | None = None,
        prediction_max_bytes: int = 268435456,
    ) -> None:
        from dual_arm_policy import policy_pb2_grpc

        self.config = config
        self.service = PolicyService(
            config,
            provider,
            prediction_directory=prediction_directory,
            prediction_max_bytes=prediction_max_bytes,
        )
        self.log = log
        self.servicer = PolicyServicer(self.service, config, log)
        self._server = grpc.server(
            futures.ThreadPoolExecutor(max_workers=config.max_workers),
            options=[
                ("grpc.max_send_message_length", config.max_message_bytes),
                ("grpc.max_receive_message_length", config.max_message_bytes),
                ("grpc.so_reuseport", 0),
            ],
            maximum_concurrent_rpcs=config.max_workers,
        )
        policy_pb2_grpc.add_PolicyServiceServicer_to_server(self.servicer, self._server)
        self.port = self._server.add_insecure_port(f"{config.bind_address}:{config.port}")
        if not self.port:
            raise RuntimeError("cannot bind Policy service")

    def start(self) -> int:
        self._server.start()
        return self.port

    def wait(self, stop: threading.Event | None = None) -> None:
        if stop is None:
            self._server.wait_for_termination()
            return
        while not stop.wait(0.2):
            if self.log is not None:
                self.log.check_health()
            if self.servicer.failed.is_set():
                raise LogFailure("policy RPC logging failed")

    def stop(self, grace_s: float | None = None) -> None:
        grace = self.config.close_deadline_s if grace_s is None else grace_s
        event = self._server.stop(grace)
        if not event.wait(timeout=grace + 0.5):
            raise TimeoutError("policy server did not stop within its grace period")


def serve_forever(
    config: PolicyServiceConfig,
    provider: PolicyProvider | PolicyModelAdapter,
    stop: threading.Event | None = None,
    *,
    log_directory: Path = Path("var/_direct/logs/policy-service"),
    prediction_directory: Path | None = None,
    prediction_max_bytes: int = 268435456,
    run_id: str | None = None,
    startup_fields: dict[str, Any] | None = None,
    snapshot: dict[str, Any] | None = None,
    snapshot_name: str = "resolved_contract.json",
) -> int:
    run_id = run_id or "policy-" + uuid.uuid4().hex[:12]
    log = EventLogger(log_directory, run_id=run_id, component="policy", settings=config.logging)
    server = None
    fields = dict(startup_fields or {})
    stopping = stop or threading.Event()
    previous = {}
    code = 0
    try:
        (log_directory / "configuration.json").write_text(
            json.dumps(config.to_document(), indent=2, allow_nan=False) + "\n"
        )
        if snapshot is not None:
            (log_directory / snapshot_name).write_text(
                json.dumps(snapshot, indent=2, allow_nan=False) + "\n"
            )
        attach = getattr(provider, "attach_logger", None)
        if callable(attach):
            attach(log)
        server = PolicyServer(
            config,
            provider,
            log,
            prediction_directory=prediction_directory,
            prediction_max_bytes=prediction_max_bytes,
        )
        port = server.start()
        log.emit(
            "policy.started",
            fields={
                "port": port,
                "test_only": config.test_only,
                "model_bundle_digest": config.model_bundle_digest,
                "processor_digest": config.processor_digest,
                **fields,
            },
            critical=True,
        )
        log.emit("policy.ready", fields={"port": port, "ready": True, **fields}, critical=True)

        def request_stop(_signum: int, _frame: object) -> None:
            stopping.set()

        previous = {s: signal.signal(s, request_stop) for s in (signal.SIGINT, signal.SIGTERM)}
        print(f"policy service listening on {config.bind_address}:{port}", flush=True)
        server.wait(stopping)
    except Exception as error:
        code = 1
        try:
            log.emit(
                "policy.failed",
                level="ERROR",
                reason=type(error).__name__,
                exception=error,
                critical=True,
            )
        except Exception:
            pass
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
        if server is not None:
            try:
                server.stop()
            except Exception:
                code = 1
        try:
            log.emit("policy.closed", fields={"exit_code": code}, critical=True)
            log.close()
        except Exception:
            code = 1
    return code


__all__ = ["PolicyServer", "PolicyServicer", "serve_forever"]
