from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.common.physics.contacts import (  # noqa: E402
    FingerContactEvidence,
    GripperCompletion,
    GripperContactEvidence,
    evaluate_gripper_completion,
    validated_gripper_completions,
)
from robot_contracts.models import ContractError  # noqa: E402


def _finger_body_name(side: str, index: int) -> str:
    prefix = "L" if side == "left" else "R"
    return f"Arm_{prefix}{9 + index}_Link"


def _finger(
    index: int, object_forces: tuple[tuple[str, float, int], ...], speed: float = 0.0
) -> FingerContactEvidence:
    return FingerContactEvidence(
        side="right",
        finger_index=index,
        body_name=_finger_body_name("right", index),
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
        "initial_reference_m": 0.0814,
        "final_reference_m": 0.0014,
        "target_position_m": 0.0014,
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


def _marvin_completion(side: str, indices: tuple[int, ...] = (1, 2)) -> GripperCompletion:
    fingers = tuple(
        FingerContactEvidence(side, index, _finger_body_name(side, index), (), 0.0)
        for index in indices
    )
    return GripperCompletion(
        side=side,
        mode="NONE",
        position_ok=False,
        contact_ok=False,
        contact_object=None,
        ambiguous=False,
        target_error_m=0.0,
        measured_position_m=0.0,
        measured_velocity_m_s=0.0,
        target_position_m=0.0,
        initial_reference_m=0.0,
        final_reference_m=0.0,
        fingers=fingers,
    )


def test_marvin_completion_evidence_keeps_two_sides_and_two_fingers():
    both = (_marvin_completion("left"), _marvin_completion("right"))
    assert validated_gripper_completions(both) == both
    with pytest.raises(ContractError, match="left then right"):
        validated_gripper_completions(
            (_marvin_completion("right"), _marvin_completion("left"))
        )
    with pytest.raises(ContractError, match="left then right"):
        validated_gripper_completions((_marvin_completion("left"),))
    with pytest.raises(ContractError, match="finger indices 1 then 2"):
        validated_gripper_completions(
            (_marvin_completion("left", (1, 3)), _marvin_completion("right"))
        )
    with pytest.raises(ContractError, match="side does not match"):
        validated_gripper_completions(
            (
                _marvin_completion("left"),
                GripperCompletion(
                    side="right",
                    mode="NONE",
                    position_ok=False,
                    contact_ok=False,
                    contact_object=None,
                    ambiguous=False,
                    target_error_m=0.0,
                    measured_position_m=0.0,
                    measured_velocity_m_s=0.0,
                    target_position_m=0.0,
                    initial_reference_m=0.0,
                    final_reference_m=0.0,
                    fingers=(
                        FingerContactEvidence("left", 1, "Arm_L10_Link", (), 0.0),
                        FingerContactEvidence("right", 2, "Arm_R11_Link", (), 0.0),
                    ),
                ),
            )
        )


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
    assert completion.target_error_m == pytest.approx(0.0014 - 0.03985)
    assert completion.final_reference_m == 0.0014


def test_position_completion_takes_precedence_and_uses_total_speed_only_there():
    contact = _contact(_finger(1, ()), _finger(2, ()), None)
    completion = _completion(
        contact,
        measured_position_m=0.0024,
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
        final_reference_m=0.0814,
        target_position_m=0.0814,
        measured_position_m=0.05,
    )
    assert completion.mode == "NONE"
    assert completion.contact_ok is False


def test_pressure_relief_below_the_measured_aperture_keeps_contact_completion():
    contact = _contact(
        _finger(1, (("source", 5.0, 2),)),
        _finger(2, (("source", 5.0, 2),)),
        "source",
    )
    completion = _completion(
        contact,
        initial_reference_m=0.05,
        final_reference_m=0.05628,
        target_position_m=0.05628,
        measured_position_m=0.05846,
        position_tolerance_m=0.001,
    )
    assert completion.mode == "CONTACT"


def test_contact_is_not_used_when_the_position_threshold_is_already_met():
    contact = _contact(
        _finger(1, (("source", 1.0, 1),)),
        _finger(2, (("source", 1.0, 1),)),
        "source",
    )
    completion = _completion(contact, measured_position_m=0.0024)
    assert completion.mode == "POSITION"
    assert completion.contact_ok is False
