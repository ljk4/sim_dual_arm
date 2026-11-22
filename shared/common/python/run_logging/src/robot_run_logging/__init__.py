"""Bounded structured event logging for one process instance.

The package owns its files and writer thread. It does not touch ROS, runtime
state, MCAP recording, or the global logging configuration.
"""

from .events import (
    IDENTIFIER_PATTERN,
    EventContext,
    LogFailure,
    LogSettings,
    Scalar,
    is_valid_identifier,
)
from .writer import EventLogger

__all__ = [
    "IDENTIFIER_PATTERN",
    "EventContext",
    "EventLogger",
    "LogFailure",
    "LogSettings",
    "Scalar",
    "is_valid_identifier",
]
