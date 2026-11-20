"""Contact release grace requires evidence and cannot renew its own deadline."""

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

pytest.importorskip("mujoco")
from model_helpers import replace
from robot_contracts.trajectory.spline import ReferenceSample

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "simulation/mujoco/src"))
from robot_backend_sim.common.config.motion_config import CONTROL_JOINT_NAMES
from robot_backend_sim.common.config.stream_config import StreamConfig, StreamConfigError
from robot_backend_sim.common.physics.contacts import FingerContactEvidence, GripperContactEvidence
from robot_backend_sim.common.physics.gripper_contact import ContactUnloader, GripperContactTracker
from robot_backend_sim.common.physics.robot_io import Measurement


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


def constrained_tracker():
    # Construct only the state constrained_axes reads; MuJoCo is unrelated.
    tracker = GripperContactTracker.__new__(GripperContactTracker)
    tracker._config = SimpleNamespace(
        position_tolerances=(0.012,) * 7 + (0.002,) + (0.012,) * 7 + (0.002,),
        velocity_tolerances=(0.025,) * 7 + (0.004,) + (0.025,) * 7 + (0.004,),
        contact_force_min_n=0.5,
        finger_velocity_max_m_s=0.003,
    )
    tracker._contact_unload_timeout_ns = round(
        StreamConfig(0.25, 0.05, 0.5, 120.0, 3.0, 2.0, 2.0).contact_unload_timeout_s * 1e9
    )
    tracker._unloaders = {side: ContactUnloader() for side in ("left", "right")}
    tracker._unloading_axes = frozenset()
    tracker._gripper_index = {"left": 7, "right": 15}
    tracker._arm_indices = tuple(index for index in range(16) if index not in (7, 15))
    return tracker


@pytest.mark.parametrize(
    "reference_velocity",
    [5.339220363198457e-14, 0.0, -5.339220363198457e-14, 0.005113313932863591],
)
def test_loaded_contact_below_measured_aperture_is_constrained(reference_velocity):
    tracker = constrained_tracker()
    constrained = tracker.constrained_axes(
        captured_reference(reference_velocity),
        captured_measurement(),
        captured_contacts(),
    )
    assert constrained == frozenset({15})
    assert tracker.unloading_axes == frozenset()


def test_reference_reaching_measured_aperture_is_not_contact_exempt():
    tracker = constrained_tracker()
    reference = captured_reference(0.01)
    positions = list(reference.positions)
    positions[15] = 0.04
    reference = replace(reference, positions=tuple(positions))
    constrained = tracker.constrained_axes(
        reference,
        captured_measurement(),
        captured_contacts(),
    )
    assert constrained == frozenset()


def test_loaded_grip_does_not_timeout_on_repeated_reference_opening_below_object():
    tracker = constrained_tracker()
    # Previously the derivative started a 2 s release budget despite a stable
    # two-finger grip and a reference narrower than the source cube (~4 cm).
    for now_ns in range(0, 5_000_000_000, 50_000_000):
        reference = captured_reference(0.005 if now_ns % 100_000_000 else -0.005)
        measurement = captured_measurement()._replace(sim_time_ns=now_ns)
        assert tracker.constrained_axes(reference, measurement, captured_contacts()) == {15}

    # Losing one finger still removes the exception immediately.
    assert tracker.constrained_axes(
        reference, measurement,
        captured_contacts(gripper_evidence(objects=("source", None), candidate=None)),
    ) == frozenset()


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
    tracker = constrained_tracker()
    constrained = tracker.constrained_axes(
        captured_reference(5.339220363198457e-14),
        captured_measurement(),
        captured_contacts(right),
    )
    assert constrained == frozenset(), case


def test_unload_needs_prior_contact_current_same_object_and_opening():
    tracker = ContactUnloader()
    args = inputs()
    assert not tracker.update(**{**args, "reference_velocity": 0.03, "closing_contact_ok": False})
    tracker = ContactUnloader()
    assert not tracker.update(**args)
    args.update(now_ns=100, reference_velocity=0.03, closing_contact_ok=False)
    assert tracker.update(**args)
    assert not tracker.update(**{**args, "contact": evidence("target")})
    assert not tracker.update(**{**args, "contact": replace(evidence(), ambiguous=True)})
    assert not tracker.update(**{**args, "reference_position": 0.039})
    assert not tracker.update(**{**args, "reference_velocity": 0.0})


def test_timeout_cannot_be_rearmed_by_pausing_but_new_closing_can_arm():
    tracker = ContactUnloader()
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
