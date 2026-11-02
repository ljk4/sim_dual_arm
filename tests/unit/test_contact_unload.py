"""Contact release grace requires evidence and cannot renew its own deadline."""

import sys
from dataclasses import replace
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "components/backend_sim/src"))
from robot_backend_sim.contact_unload import ContactUnload  # noqa: E402
from robot_backend_sim.contacts import (  # noqa: E402
    FingerContactEvidence,
    GripperContactEvidence,
)
from robot_backend_sim.stream_config import StreamConfig, StreamConfigError  # noqa: E402


def evidence(object_name="source"):
    return GripperContactEvidence(
        "right",
        tuple(
            FingerContactEvidence("right", i, f"finger{i}", ((object_name, 5.0, 1),), 0.0)
            for i in (1, 2)
        ),
        object_name,
        False,
    )


def inputs():
    return dict(
        now_ns=0,
        timeout_ns=2_000_000_000,
        reference_position=0.0,
        reference_velocity=0.0,
        measured_position=0.04,
        position_tolerance=0.002,
        closing_contact_ok=True,
        contact=evidence(),
    )


def test_unload_needs_prior_contact_current_same_object_and_opening():
    tracker = ContactUnload()
    args = inputs()
    assert not tracker.update(**{**args, "reference_velocity": 0.03, "closing_contact_ok": False})
    tracker = ContactUnload()
    assert not tracker.update(**args)
    args.update(now_ns=100, reference_velocity=0.03, closing_contact_ok=False)
    assert tracker.update(**args)
    assert not tracker.update(**{**args, "contact": evidence("target")})
    assert not tracker.update(**{**args, "contact": replace(evidence(), ambiguous=True)})
    assert not tracker.update(**{**args, "reference_position": 0.039})
    assert not tracker.update(**{**args, "reference_velocity": 0.0})


def test_timeout_cannot_be_rearmed_by_pausing_but_new_closing_can_arm():
    tracker = ContactUnload()
    args = inputs()
    tracker.update(**args)
    args.update(reference_velocity=0.03, closing_contact_ok=False)
    assert tracker.update(**args)
    args["now_ns"] = 2_000_000_000
    assert not tracker.update(**args)
    tracker.update(**{**args, "reference_velocity": 0.0, "closing_contact_ok": True})
    assert not tracker.update(**args)
    tracker.update(**{**args, "reference_velocity": -0.01, "closing_contact_ok": True})
    assert tracker.update(**args)
    assert not tracker.update(**{**args, "timeout_ns": 0})


@pytest.mark.parametrize("value", [True, -1.0, 2.01, float("inf"), float("nan"), "2"])
def test_unload_budget_rejects_invalid_or_unbounded_values(value):
    with pytest.raises(StreamConfigError, match="contact_unload_timeout_s"):
        StreamConfig(0.25, 0.05, 0.5, 120.0, 3.0, 2.0, value)
