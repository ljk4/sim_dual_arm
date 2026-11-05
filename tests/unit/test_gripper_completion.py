from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.contacts import (  # noqa: E402
    FingerContactEvidence,
    GripperContactEvidence,
    evaluate_gripper_completion,
)


def _finger(
    index: int, object_forces: tuple[tuple[str, float, int], ...], speed: float = 0.0
) -> FingerContactEvidence:
    return FingerContactEvidence(
        side="right",
        finger_index=index,
        body_name=f"right_finger{index}",
        object_forces=object_forces,
        joint_speed_m_s=speed,
    )


def _contact(
    finger1: FingerContactEvidence,
    finger2: FingerContactEvidence,
    candidate: str | None,
    ambiguous: bool = False,
) -> GripperContactEvidence:
    return GripperContactEvidence("right", (finger1, finger2), candidate, ambiguous)


def _completion(contact: GripperContactEvidence, **changes):
    values = {
        "side": "right",
        "initial_reference_m": 0.08,
        "final_reference_m": 0.0,
        "target_position_m": 0.0,
        "measured_position_m": 0.03985,
        "measured_velocity_m_s": 0.0,
        "position_tolerance_m": 0.002,
        "velocity_tolerance_m_s": 0.004,
        "force_min_n": 0.5,
        "finger_speed_max_m_s": 0.003,
        "contact": contact,
    }
    values.update(changes)
    return evaluate_gripper_completion(**values)


def test_contact_completion_accepts_two_named_fingers_on_one_object():
    contact = _contact(
        _finger(1, (("source", 1.0, 3),), 0.001),
        _finger(2, (("source", 0.9, 2),), 0.002),
        "source",
    )
    completion = _completion(contact)
    assert completion.mode == "CONTACT"
    assert completion.contact_ok is True
    assert completion.contact_object == "source"
    assert completion.target_error_m == pytest.approx(-0.03985)
    assert completion.final_reference_m == 0.0


def test_position_completion_takes_precedence_and_uses_total_speed_only_there():
    contact = _contact(_finger(1, ()), _finger(2, ()), None)
    completion = _completion(
        contact,
        measured_position_m=0.001,
        measured_velocity_m_s=0.0039,
    )
    assert completion.mode == "POSITION"
    assert completion.position_ok is True


@pytest.mark.parametrize(
    ("contact", "changes", "mode"),
    (
        (_contact(_finger(1, ()), _finger(2, ()), None), {}, "NONE"),
        (
            _contact(
                _finger(1, (("source", 1.0, 1),)),
                _finger(2, ()),
                None,
            ),
            {},
            "NONE",
        ),
        (
            _contact(
                _finger(1, (("source", 1.0, 1),)),
                _finger(2, (("target", 1.0, 1),)),
                None,
            ),
            {},
            "NONE",
        ),
        (
            _contact(
                _finger(1, (("source", 0.4, 1),)),
                _finger(2, (("source", 1.0, 1),)),
                "source",
            ),
            {},
            "NONE",
        ),
        (
            _contact(
                _finger(1, (("source", 1.0, 1),), 0.004),
                _finger(2, (("source", 1.0, 1),), 0.001),
                "source",
            ),
            {},
            "NONE",
        ),
        (
            _contact(
                _finger(1, (("source", 1.0, 1), ("target", 1.0, 1))),
                _finger(2, (("source", 1.0, 1), ("target", 1.0, 1))),
                None,
                True,
            ),
            {},
            "NONE",
        ),
    ),
)
def test_contact_negative_cases_do_not_complete(contact, changes, mode):
    completion = _completion(contact, **changes)
    assert completion.mode == mode
    assert completion.contact_ok is False


def test_opening_reference_cannot_use_contact_completion_even_with_contact():
    contact = _contact(
        _finger(1, (("source", 1.0, 1),)),
        _finger(2, (("source", 1.0, 1),)),
        "source",
    )
    completion = _completion(
        contact,
        initial_reference_m=0.04,
        final_reference_m=0.08,
        target_position_m=0.08,
        measured_position_m=0.05,
    )
    assert completion.mode == "NONE"
    assert completion.contact_ok is False


def test_contact_is_not_used_when_the_position_threshold_is_already_met():
    contact = _contact(
        _finger(1, (("source", 1.0, 1),)),
        _finger(2, (("source", 1.0, 1),)),
        "source",
    )
    completion = _completion(contact, measured_position_m=0.001)
    assert completion.mode == "POSITION"
    assert completion.contact_ok is False
