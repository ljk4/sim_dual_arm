"""Bounded structured event logging for one process instance.

The package owns its files and writer thread. It does not touch ROS, runtime
state, MCAP recording, or the global logging configuration.
"""

from .events import EventContext, LogFailure, LogSettings, Scalar
from .writer import EventLogger

__all__ = ["EventContext", "EventLogger", "LogFailure", "LogSettings", "Scalar"]
