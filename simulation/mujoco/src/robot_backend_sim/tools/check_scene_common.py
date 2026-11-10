"""Stable failure types for the offline scene check."""

from __future__ import annotations


class SceneCheckUsageError(ValueError):
    """Invalid invocation before the run directory is created."""

class SceneCheckFailure(RuntimeError):
    """A stable scene-check failure with a bounded message."""

    def __init__(self, reason: str, message: str = "") -> None:
        super().__init__(message or reason)
        self.reason = reason
        self.message = message or reason

