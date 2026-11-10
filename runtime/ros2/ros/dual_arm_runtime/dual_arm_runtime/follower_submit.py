"""Request validation and rejection typing for follower submissions."""

from __future__ import annotations

from robot_contracts.models import ContractError
from robot_contracts.stream_request import REQUEST_STREAM_FIRST, StreamRequest

from dual_arm_runtime.run_logging import check_health
from dual_arm_runtime.runtime_errors import FirstFollowerRejected


def validate_follower_request(client: object, request: StreamRequest) -> tuple[object, str]:
    if not isinstance(request, StreamRequest):
        raise ContractError("INVALID_FOLLOWER_REQUEST", "request must be a StreamRequest")
    if client._follower_limits is None:
        raise ContractError("FOLLOWER_DISABLED", "follower limits are not configured")
    check_health(client._log)
    if client._stop_requested.is_set():
        raise ContractError("EPISODE_STOPPED", "follower submit stopped")
    context = request.trajectory.context
    command_id = request.trajectory.command_id
    if context != client.lease_context or request.trajectory.lease_id != client.lease_id:
        raise ContractError("STALE_CONTEXT", "follower request does not own the current lease")
    origin = client._stream_origins.get(command_id)
    if request.kind == REQUEST_STREAM_FIRST:
        if origin is not None and origin != request:
            raise ContractError("COMMAND_ID_CONFLICT", "stream origin changed")
        if origin is None and len(client._stream_origins) >= client._stream_origin_capacity:
            raise ContractError(
                "CONTROL_BUSY",
                f"stream origin cache is full ({client._stream_origin_capacity})",
            )
        client._stream_origins.setdefault(command_id, request)
    elif origin is None:
        raise ContractError("STALE_STREAM", "successor has no local stream origin")
    return context, command_id


def rejection_type(response: object, attempt: int) -> type[Exception]:
    return (
        FirstFollowerRejected
        if attempt == 1 and not response.has_execution  # type: ignore[attr-defined]
        else ContractError
    )


__all__ = ["rejection_type", "validate_follower_request"]
