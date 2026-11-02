"""Small shutdown helpers: signal capture and exit policy.

The signal handler deliberately does nothing except remember the first signal.
ROS, file and lock work belongs on the normal executor/worker paths.
"""

import argparse
import math
import signal

EXIT_RUNTIME_FAILURE = 1
EXIT_SHUTDOWN_TIMEOUT = 124
SIGNAL_EXIT_BASE = 128

HANDLED_SIGNALS = (signal.SIGINT, signal.SIGTERM)


class ShutdownSignal:
    """Record the first terminating signal; repeated signals never shorten cleanup."""

    def __init__(self) -> None:
        self._signum = 0

    def __call__(self, signum, frame) -> None:
        # Assignment only: no locks, ROS calls, joins or file operations here.
        if not self._signum:
            self._signum = signum

    @property
    def requested(self) -> bool:
        return bool(self._signum)

    @property
    def signum(self) -> int:
        return self._signum

    @property
    def reason(self) -> str:
        if not self._signum:
            return ""
        return signal.Signals(self._signum).name

    @property
    def exit_code(self) -> int:
        return SIGNAL_EXIT_BASE + self._signum if self._signum else 0


def install_signal_handlers(state: ShutdownSignal) -> dict[int, object]:
    """Install only the two documented handlers and return the previous ones."""
    previous = {}
    for signum in HANDLED_SIGNALS:
        previous[signum] = signal.getsignal(signum)
        signal.signal(signum, state)
    return previous


def restore_signal_handlers(previous: dict[int, object]) -> None:
    for signum, handler in previous.items():
        signal.signal(signum, handler)


def positive_seconds(text: str) -> float:
    """argparse type for a positive, finite shutdown grace."""
    try:
        value = float(text)
    except ValueError as error:
        raise argparse.ArgumentTypeError("must be a positive number of seconds") from error
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("must be a positive number of seconds")
    return value
