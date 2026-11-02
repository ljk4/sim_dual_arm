"""Pure Python checks for the shutdown signal policy, no ROS imports."""

import argparse
import signal

import pytest
from dual_arm_runtime.shutdown import (
    EXIT_SHUTDOWN_TIMEOUT,
    ShutdownSignal,
    install_signal_handlers,
    positive_seconds,
    restore_signal_handlers,
)


def test_signal_state_keeps_first_request_and_reason():
    state = ShutdownSignal()
    assert not state.requested
    assert state.exit_code == 0
    assert state.reason == ""

    state(signal.SIGTERM, None)
    state(signal.SIGINT, None)

    assert state.requested
    assert state.reason == "SIGTERM"
    assert state.exit_code == 143


def test_signal_state_sigint_exit_code():
    state = ShutdownSignal()
    state(signal.SIGINT, None)
    assert state.reason == "SIGINT"
    assert state.exit_code == 130


def test_install_and_restore_handlers_are_explicit():
    state = ShutdownSignal()
    previous = install_signal_handlers(state)
    try:
        assert signal.getsignal(signal.SIGINT) is state
        assert signal.getsignal(signal.SIGTERM) is state
    finally:
        restore_signal_handlers(previous)
    assert signal.getsignal(signal.SIGINT) is previous[signal.SIGINT]
    assert signal.getsignal(signal.SIGTERM) is previous[signal.SIGTERM]


@pytest.mark.parametrize("value", ["0", "-1", "nan", "inf", "not-a-number"])
def test_shutdown_timeout_rejects_non_positive_or_non_finite(value):
    with pytest.raises(argparse.ArgumentTypeError):
        positive_seconds(value)


def test_shutdown_timeout_accepts_positive_value():
    assert positive_seconds("0.25") == 0.25


def test_timeout_exit_code_is_nonzero_and_distinct():
    assert EXIT_SHUTDOWN_TIMEOUT != 0
