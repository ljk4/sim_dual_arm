"""Named dual-finger contact evidence and contact-mode completion rules.

This module reads only the current ``MjModel``/``MjData``.  It never writes
qpos/qvel/time, never steps physics and never selects an arbitrary object when
both fingers touch more than one common free body.  Contact forces are summed
per named finger body and per allowed free-joint object; tangential components
are not treated as normal force.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from robot_contracts.models import ContractError

__all__ = [
    "ContactBinding",
    "FingerBinding",
    "FingerContactEvidence",
    "GripperCompletion",
    "GripperContactEvidence",
    "evaluate_gripper_completion",
    "empty_contact_evidence",
    "measure_contact_evidence",
    "resolve_contact_binding",
    "validated_gripper_completions",
]

_RESERVED_BODY_NAMES = frozenset({"world", "column", "table"})
_SIDES = ("left", "right")
_FINGER_COUNT = 2


@dataclass(frozen=True, slots=True)
class FingerBinding:
    side: str
    finger_index: int
    body_name: str
    body_id: int
    dof_address: int


@dataclass(frozen=True, slots=True)
class ContactBinding:
    """Resolved names/indices for configured free objects and both grippers."""

    object_bodies: tuple[tuple[str, int], ...]
    fingers: tuple[FingerBinding, ...]

    def object_name(self, body_id: int) -> str | None:
        for name, identifier in self.object_bodies:
            if identifier == body_id:
                return name
        return None

    def finger(self, side: str, finger_index: int) -> FingerBinding:
        for item in self.fingers:
            if item.side == side and item.finger_index == finger_index:
                return item
        raise KeyError((side, finger_index))


def _body_id(model: Any, name: str) -> int:
    try:
        return int(model.body(name).id)
    except KeyError as error:
        raise ContractError("MODEL_MISMATCH", f"contact body {name!r} is missing") from error


def _free_joint_count(model: Any, body_id: int) -> int:
    start = int(model.body_jntadr[body_id])
    count = int(model.body_jntnum[body_id])
    return sum(
        1
        for joint_id in range(start, start + count)
        if int(model.jnt_type[joint_id]) == 0  # mjJNT_FREE
    )


def resolve_contact_binding(
    model: Any,
    object_names: tuple[str, ...],
    finger_bodies: Mapping[str, tuple[tuple[str, str], ...]],
) -> ContactBinding:
    """Validate the configured free objects and the robot's named finger pairs.

    Only the configured scene objects are admitted as contact targets.  A
    configured object must have exactly one free joint and must not be a
    reserved world/table/robot body.  Each side must name exactly two
    ``(body_name, joint_name)`` fingers; the robot adapter supplies those names.
    """
    if not isinstance(object_names, tuple) or not object_names:
        raise ContractError("MODEL_MISMATCH", "scene must configure at least one free object")
    if not isinstance(finger_bodies, Mapping):
        raise ContractError("MODEL_MISMATCH", "finger bindings must be a mapping")
    objects: list[tuple[str, int]] = []
    seen: set[str] = set()
    for name in object_names:
        if not isinstance(name, str) or not name or name in seen:
            raise ContractError("MODEL_MISMATCH", "configured object names must be unique strings")
        if name in _RESERVED_BODY_NAMES:
            raise ContractError("MODEL_MISMATCH", f"reserved body {name!r} is not a free object")
        body_id = _body_id(model, name)
        if body_id == 0:
            raise ContractError("MODEL_MISMATCH", f"object {name!r} must not be the world body")
        if _free_joint_count(model, body_id) != 1:
            raise ContractError(
                "MODEL_MISMATCH",
                f"object {name!r} must own exactly one free joint",
            )
        seen.add(name)
        objects.append((name, body_id))

    fingers: list[FingerBinding] = []
    for side in _SIDES:
        specs = finger_bodies.get(side)
        if not isinstance(specs, tuple) or len(specs) != _FINGER_COUNT:
            raise ContractError(
                "MODEL_MISMATCH",
                f"finger bindings for {side!r} must name exactly two fingers",
            )
        for finger_index, spec in enumerate(specs, start=1):
            if not isinstance(spec, tuple) or len(spec) != 2:
                raise ContractError(
                    "MODEL_MISMATCH",
                    f"finger {side!r}/{finger_index} must be a (body, joint) pair",
                )
            body_name, joint_name = spec
            body_id = _body_id(model, body_name)
            try:
                dof_address = int(model.joint(joint_name).dofadr[0])
            except KeyError as error:
                raise ContractError(
                    "MODEL_MISMATCH", f"finger joint {joint_name!r} is missing"
                ) from error
            if dof_address < 0 or dof_address >= model.nv:
                raise ContractError(
                    "MODEL_MISMATCH", f"finger joint {joint_name!r} has an invalid dof address"
                )
            fingers.append(FingerBinding(side, finger_index, body_name, body_id, dof_address))
    return ContactBinding(tuple(objects), tuple(fingers))


@dataclass(frozen=True, slots=True)
class FingerContactEvidence:
    """One named finger's summed normal forces per allowed object plus speed."""

    side: str
    finger_index: int
    body_name: str
    object_forces: tuple[tuple[str, float, int], ...]
    joint_speed_m_s: float

    def force_for(self, object_name: str | None) -> float:
        if object_name is None:
            return 0.0
        for name, force, _count in self.object_forces:
            if name == object_name:
                return force
        return 0.0

    def count_for(self, object_name: str | None) -> int:
        if object_name is None:
            return 0
        for name, _force, count in self.object_forces:
            if name == object_name:
                return count
        return 0

    @property
    def object_names(self) -> tuple[str, ...]:
        return tuple(name for name, _force, _count in self.object_forces)


@dataclass(frozen=True, slots=True)
class GripperContactEvidence:
    """Immutable contact snapshot for one gripper at one measurement time."""

    side: str
    fingers: tuple[FingerContactEvidence, FingerContactEvidence]
    candidate_object: str | None
    ambiguous: bool

    def finger(self, index: int) -> FingerContactEvidence:
        return self.fingers[index - 1]


@dataclass(frozen=True, slots=True)
class GripperCompletion:
    """Recorded POSITION/CONTACT/NONE decision for one gripper."""

    side: str
    mode: str
    position_ok: bool
    contact_ok: bool
    contact_object: str | None
    ambiguous: bool
    target_error_m: float
    measured_position_m: float
    measured_velocity_m_s: float
    target_position_m: float
    initial_reference_m: float
    final_reference_m: float
    fingers: tuple[FingerContactEvidence, ...]


def _normal_force(model: Any, data: Any, contact_index: int) -> float:
    try:
        import mujoco
        import numpy as np
    except ImportError as error:  # pragma: no cover - native path only
        raise ContractError(
            "MUJOCO_UNAVAILABLE", "MuJoCo is required for contact forces"
        ) from error
    value = np.zeros(6, dtype=float)
    mujoco.mj_contactForce(model, data, contact_index, value)
    result = float(value[0])
    if not math.isfinite(result):
        raise ContractError("INVALID_CONTACT", "contact normal force is not finite")
    return abs(result)


def _collect_object_forces(
    model: Any,
    data: Any,
    binding: ContactBinding,
    finger: FingerBinding,
) -> tuple[tuple[str, float, int], ...]:
    totals: dict[str, list[float | int]] = {}
    finger_body = finger.body_id
    for contact_index in range(int(data.ncon)):
        contact = data.contact[contact_index]
        geom1 = int(contact.geom1)
        geom2 = int(contact.geom2)
        body1 = int(model.geom_bodyid[geom1])
        body2 = int(model.geom_bodyid[geom2])
        if body1 == finger_body:
            object_name = binding.object_name(body2)
        elif body2 == finger_body:
            object_name = binding.object_name(body1)
        else:
            continue
        if object_name is None:
            continue
        total = totals.setdefault(object_name, [0.0, 0])
        total[0] = float(total[0]) + _normal_force(model, data, contact_index)
        total[1] = int(total[1]) + 1
    return tuple((name, float(value[0]), int(value[1])) for name, value in totals.items())


def measure_contact_evidence(
    model: Any,
    data: Any,
    binding: ContactBinding,
    side: str,
) -> GripperContactEvidence:
    """Read one gripper's current contacts and per-finger slide speeds.

    Callers must refresh forces with ``mj_forward`` after the last step before
    calling this function; the returned values are then evidence for the same
    measurement time as the corresponding qpos/qvel read.
    """
    if side not in _SIDES:
        raise ContractError("INVALID_CONTACT", f"unknown gripper side {side!r}")
    fingerprints: list[FingerContactEvidence] = []
    for finger in sorted(
        (item for item in binding.fingers if item.side == side),
        key=lambda item: item.finger_index,
    ):
        if finger.finger_index > _FINGER_COUNT:
            raise ContractError("MODEL_MISMATCH", "finger binding index exceeds two")
        speed = abs(float(data.qvel[finger.dof_address]))
        if not math.isfinite(speed):
            raise ContractError("INVALID_CONTACT", f"{finger.body_name}.speed is not finite")
        fingerprints.append(
            FingerContactEvidence(
                side=side,
                finger_index=finger.finger_index,
                body_name=finger.body_name,
                object_forces=_collect_object_forces(model, data, binding, finger),
                joint_speed_m_s=speed,
            )
        )
    if len(fingerprints) != _FINGER_COUNT:
        raise ContractError("MODEL_MISMATCH", f"{side} gripper must bind two fingers")

    common: set[str] = set(fingerprints[0].object_names)
    common.intersection_update(fingerprints[1].object_names)
    if len(common) == 1:
        candidate = next(iter(common))
        ambiguous = False
    elif len(common) > 1:
        candidate = None
        ambiguous = True
    else:
        candidate = None
        ambiguous = False
    return GripperContactEvidence(side, (fingerprints[0], fingerprints[1]), candidate, ambiguous)


def empty_contact_evidence(data: Any, binding: ContactBinding, side: str) -> GripperContactEvidence:
    """Build a no-contact snapshot with current per-finger speeds.

    Used before the first step, where reading post-step contact forces would
    mislabel a previous state.  It deliberately records no force evidence.
    """
    fingerprints = []
    for finger_index in range(1, _FINGER_COUNT + 1):
        finger = binding.finger(side, finger_index)
        speed = abs(float(data.qvel[finger.dof_address]))
        fingerprints.append(
            FingerContactEvidence(side, finger_index, finger.body_name, (), speed)
        )
    return GripperContactEvidence(side, (fingerprints[0], fingerprints[1]), None, False)


def evaluate_gripper_completion(
    *,
    side: str,
    initial_reference_m: float,
    final_reference_m: float,
    target_position_m: float,
    measured_position_m: float,
    measured_velocity_m_s: float,
    position_tolerance_m: float,
    velocity_tolerance_m_s: float,
    force_min_n: float,
    finger_speed_max_m_s: float,
    contact: GripperContactEvidence,
) -> GripperCompletion:
    """Apply the accepted POSITION or contact completion rule for one gripper.

    Position mode requires both total-opening error and total-opening speed to
    meet their thresholds.  Contact mode is allowed only when the measured
    opening is wider than the final reference and the position threshold is
    still missed, and both named finger bodies touch the same configured free
    object with enough per-finger normal force and slow enough per-finger
    motion. A rising reference below that aperture still compresses the object;
    an opening beyond it must complete in position mode. The contact rule does
    not use the summed finger velocity.
    """
    if contact.side != side:
        raise ContractError("INVALID_CONTACT", "contact evidence side does not match")
    error = target_position_m - measured_position_m
    position_ok = abs(error) <= position_tolerance_m and (
        abs(measured_velocity_m_s) <= velocity_tolerance_m_s
    )
    still_above = measured_position_m > final_reference_m
    threshold_missed = abs(error) > position_tolerance_m
    contact_ready = still_above and threshold_missed
    contact_ok = False
    object_name = contact.candidate_object
    if contact_ready and object_name is not None and not contact.ambiguous:
        contact_ok = all(
            finger.count_for(object_name) > 0
            and finger.force_for(object_name) >= force_min_n
            and abs(finger.joint_speed_m_s) <= finger_speed_max_m_s
            for finger in contact.fingers
        )
    if position_ok:
        mode = "POSITION"
    elif contact_ok:
        mode = "CONTACT"
    else:
        mode = "NONE"
    return GripperCompletion(
        side=side,
        mode=mode,
        position_ok=position_ok,
        contact_ok=contact_ok,
        contact_object=object_name,
        ambiguous=contact.ambiguous,
        target_error_m=error,
        measured_position_m=measured_position_m,
        measured_velocity_m_s=measured_velocity_m_s,
        target_position_m=target_position_m,
        initial_reference_m=initial_reference_m,
        final_reference_m=final_reference_m,
        fingers=contact.fingers,
    )


def validated_gripper_completions(
    completions: tuple[GripperCompletion, ...],
) -> tuple[GripperCompletion, ...]:
    """Keep the two-sided, two-finger evidence rule explicit.

    The shared feedback container accepts any bounded, named gripper/finger
    set.  This backend still requires exactly the left then right grippers, each
    with finger indices 1 and 2, because its completion and same-object contact
    rules are defined for that named pair on both robots.
    """
    sides = tuple(completion.side for completion in completions)
    if sides != _SIDES:
        raise ContractError(
            "INVALID_GRIPPER_FEEDBACK",
            "requires exactly left then right gripper completions",
        )
    for completion in completions:
        indices = tuple(finger.finger_index for finger in completion.fingers)
        if indices != (1, 2):
            raise ContractError(
                "INVALID_GRIPPER_FEEDBACK",
                f"{completion.side} gripper requires finger indices 1 then 2",
            )
        if any(finger.side != completion.side for finger in completion.fingers):
            raise ContractError(
                "INVALID_GRIPPER_FEEDBACK",
                f"{completion.side} finger evidence side does not match",
            )
    return completions
