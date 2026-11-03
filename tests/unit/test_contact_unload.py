"""Contact release grace requires evidence and cannot renew its own deadline."""

import sys
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest
from robot_contracts.reference import ReferenceSample

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "components/backend_sim/src"))
from robot_backend_sim.contact_unload import ContactUnload  # noqa: E402
from robot_backend_sim.contacts import (  # noqa: E402
    FingerContactEvidence,
    GripperContactEvidence,
)
from robot_backend_sim.motion import NativeMotion  # noqa: E402
from robot_backend_sim.motion_config import CONTROL_JOINT_NAMES  # noqa: E402
from robot_backend_sim.panda_io import Measurement  # noqa: E402
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


def gripper_evidence(
    side="right",
    *,
    objects=("source", "source"),
    forces=(1.0, 1.0),
    speeds=(0.0, 0.0),
    candidate="source",
    ambiguous=False,
):
    fingers = []
    for index in (1, 2):
        object_name = objects[index - 1]
        object_forces = (
            () if object_name is None else ((object_name, forces[index - 1], 1),)
        )
        fingers.append(
            FingerContactEvidence(
                side, index, f"finger{index}", object_forces, speeds[index - 1]
            )
        )
    return GripperContactEvidence(side, tuple(fingers), candidate, ambiguous)


def captured_contacts(right=None):
    left = gripper_evidence("left", objects=(None, None), candidate=None)
    return (left, gripper_evidence() if right is None else right)


def captured_reference(right_velocity):
    positions = [0.0] * len(CONTROL_JOINT_NAMES)
    velocities = [0.0] * len(CONTROL_JOINT_NAMES)
    velocities[15] = right_velocity
    zeros = (0.0,) * len(CONTROL_JOINT_NAMES)
    return ReferenceSample(tuple(positions), tuple(velocities), zeros)


def captured_measurement():
    positions = [0.0] * len(CONTROL_JOINT_NAMES)
    positions[15] = 0.0398
    velocities = [0.0] * len(CONTROL_JOINT_NAMES)
    return Measurement(1_000_000, CONTROL_JOINT_NAMES, tuple(positions), tuple(velocities))


def constrained_motion():
    # Construct only the state _constrained_axes reads; the rest of NativeMotion
    # requires a live MuJoCo scene and is unrelated to this classification.
    motion = NativeMotion.__new__(NativeMotion)
    motion._config = SimpleNamespace(
        position_tolerances=(0.012,) * 7 + (0.002,) + (0.012,) * 7 + (0.002,),
        velocity_tolerances=(0.025,) * 7 + (0.004,) + (0.025,) * 7 + (0.004,),
        contact_force_min_n=0.5,
        finger_velocity_max_m_s=0.003,
    )
    motion._stream_config = StreamConfig(0.25, 0.05, 0.5, 120.0, 3.0, 2.0, 2.0)
    motion._contact_unload = {side: ContactUnload() for side in ("left", "right")}
    motion._unloading_axes = frozenset()
    return motion


@pytest.mark.parametrize(
    "reference_velocity",
    [5.339220363198457e-14, 0.0, -5.339220363198457e-14],
)
def test_numerical_standstill_contact_is_constrained(reference_velocity):
    motion = constrained_motion()
    constrained = motion._constrained_axes(
        captured_reference(reference_velocity),
        captured_measurement(),
        captured_contacts(),
    )
    assert constrained == frozenset({15})
    assert motion._unloading_axes == frozenset()


def test_real_opening_contact_is_not_exempt_without_prior_hold():
    motion = constrained_motion()
    constrained = motion._constrained_axes(
        captured_reference(0.01),
        captured_measurement(),
        captured_contacts(),
    )
    assert constrained == frozenset()


@pytest.mark.parametrize(
    ("case", "right"),
    (
        ("missing_finger", gripper_evidence(objects=("source", None))),
        ("foreign_object", gripper_evidence(objects=("source", "other"))),
        ("ambiguous", gripper_evidence(ambiguous=True)),
        ("low_force", gripper_evidence(forces=(1.0, 0.1))),
        ("fast_finger", gripper_evidence(speeds=(0.0, 0.0031))),
    ),
)
def test_contact_exemption_still_requires_full_valid_evidence(case, right):
    motion = constrained_motion()
    constrained = motion._constrained_axes(
        captured_reference(5.339220363198457e-14),
        captured_measurement(),
        captured_contacts(right),
    )
    assert constrained == frozenset(), case


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
