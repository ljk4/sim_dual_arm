"""Pure completed-policy audit invariants shared by the MCAP v4 audit."""

from __future__ import annotations

from robot_contracts.models import ContractError


def require_completed_chunks(
    *,
    completed: bool,
    policy_chunks: int,
    configured_chunks: int,
    strict: bool = True,
) -> None:
    """A completed finite diagnostic must contain exactly the configured chunks.

    Task-terminated streams stop on the physical judge, so their executed chunk
    count is not the configured capacity bound; the old v4 diagnostic path keeps
    ``strict=True`` unchanged.
    """
    if not completed or not strict:
        return
    if policy_chunks != configured_chunks:
        raise ContractError(
            "RECORDING_ERROR",
            f"completed policy stream has {policy_chunks} chunks, "
            f"configured {configured_chunks}",
        )


__all__ = ["require_completed_chunks"]
