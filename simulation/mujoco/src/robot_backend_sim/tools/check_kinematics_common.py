"""Stable failures, artifact budget and event helper for kinematics checks."""

from __future__ import annotations

from dataclasses import dataclass


class KinematicsCheckUsageError(ValueError):
    """Invalid invocation before the run directory is created."""

class KinematicsCheckFailure(RuntimeError):
    """A stable check failure with a bounded reason and message."""

    def __init__(self, reason: str, message: str = "") -> None:
        super().__init__(message or reason)
        self.reason = reason
        self.message = message or reason

@dataclass
class ArtifactBudget:
    limit_bytes: int
    total_bytes: int = 0
    count: int = 0

def _emit(
    logger: object, event: str, message: str, fields: dict[str, object] | None = None
) -> None:
    try:
        logger.emit(event, message=message, fields=fields, critical=True)
    except Exception as error:
        raise KinematicsCheckFailure(
            "event_log_failed", f"{event}: {type(error).__name__}: {error}"
        ) from error

