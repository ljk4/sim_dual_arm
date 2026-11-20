"""Completed-policy checks for the recording audit."""

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

    Finite diagnostics use the configured chunk count as a completion condition.
    Task-terminated streams use the task result and treat that count as capacity.
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
