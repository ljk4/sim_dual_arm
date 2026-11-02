"""Pressure must never erase a terminal/transition feedback fact."""

import queue
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "components/backend_sim/src"))
from robot_backend_sim.feedback_mailbox import FeedbackMailbox  # noqa: E402


def test_pressure_preserves_transition_order():
    box = FeedbackMailbox(3)
    box.put_nowait("accepted", critical=True)
    box.put_nowait("old periodic")
    box.put_nowait("terminal", critical=True)
    assert box.put_nowait("new periodic")
    assert [box.get_nowait() for _ in range(3)] == ["accepted", "terminal", "new periodic"]
    assert box.dropped == 1


def test_all_critical_slots_fail_closed_for_another_transition():
    box = FeedbackMailbox(1)
    box.put_nowait("terminal", critical=True)
    assert not box.put_nowait("periodic")
    with pytest.raises(queue.Full):
        box.put_nowait("next accepted", critical=True)
    assert box.get_nowait() == "terminal"
    with pytest.raises(queue.Empty):
        box.get_nowait()
