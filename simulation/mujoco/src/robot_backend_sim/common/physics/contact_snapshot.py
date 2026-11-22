"""Consistent per-finger contact and free-object facts for one simulation step.

The builder reads one ``MjModel``/``MjData`` pair and returns immutable tuples.
It never steps physics or writes qpos/qvel/time.  The owner thread calls it
after refreshing forces with ``mj_forward`` so contact forces and object motion
belong to the same measurement time.  Robot-side interpretation (which finger
belongs to which gripper) and task-side interpretation (hold, support) stay in
their adapters.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from robot_contracts.models import ContractError

from robot_backend_sim.common.physics.scene_container import CompiledScene

__all__ = [
    "ContactSnapshot",
    "FingerObjectContact",
    "GripperPose",
    "ObjectMotion",
    "SupportContact",
    "SupportGeomContact",
    "build_contact_snapshot",
]


@dataclass(frozen=True, slots=True)
class ObjectMotion:
    name: str
    center: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]
    linear_velocity: tuple[float, float, float]
    angular_velocity: tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class FingerObjectContact:
    side: str
    finger_index: int
    finger_body: str
    object_name: str
    normal_force_n: float
    contact_count: int


@dataclass(frozen=True, slots=True)
class SupportContact:
    """Contact between a configured object and a non-finger body."""

    object_name: str
    body_name: str
    normal_force_n: float
    contact_count: int


@dataclass(frozen=True, slots=True)
class SupportGeomContact:
    """Active contact between a configured object and one non-finger geom.

    ``object_upward_force_n`` is the world-Z force the support geom applies to
    the object.  It resolves the contact normal direction and the geom1/geom2
    order, so a vertical wall contact contributes a lateral force while a real
    floor contributes positive upward support.
    """

    object_name: str
    body_name: str
    geom_name: str
    normal_force_n: float
    object_upward_force_n: float
    contact_count: int


@dataclass(frozen=True, slots=True)
class GripperPose:
    side: str
    opening_m: float
    tcp_position: tuple[float, float, float]
    tcp_quaternion_wxyz: tuple[float, float, float, float]


@dataclass(frozen=True, slots=True)
class ContactSnapshot:
    sim_time_ns: int
    capture_seq: int
    objects: tuple[ObjectMotion, ...]
    finger_contacts: tuple[FingerObjectContact, ...]
    support_contacts: tuple[SupportContact, ...]
    support_geom_contacts: tuple[SupportGeomContact, ...]
    grippers: tuple[GripperPose, ...]


def _finite(value: Any, field: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ContractError("INVALID_CONTACT", f"{field} is not finite")
    return number


def _object_motion(model: Any, data: Any, name: str, qpos_address: int) -> ObjectMotion:
    if len(data.qpos) < qpos_address + 7:
        raise ContractError("INVALID_CONTACT", f"{name} qpos slice is outside the model")
    values = tuple(
        _finite(data.qpos[qpos_address + index], f"{name}.qpos[{index}]") for index in range(7)
    )
    dof_address = None
    for joint_id in range(int(model.njnt)):
        if int(model.jnt_qposadr[joint_id]) == qpos_address:
            dof_address = int(model.jnt_dofadr[joint_id])
            break
    if dof_address is None or dof_address + 6 > int(model.nv):
        raise ContractError("INVALID_CONTACT", f"{name} has no readable free-joint dof")
    velocity = tuple(
        _finite(data.qvel[dof_address + index], f"{name}.qvel[{index}]") for index in range(6)
    )
    return ObjectMotion(
        name=name,
        center=(values[0], values[1], values[2]),
        quaternion_wxyz=(values[3], values[4], values[5], values[6]),
        linear_velocity=(velocity[0], velocity[1], velocity[2]),
        angular_velocity=(velocity[3], velocity[4], velocity[5]),
    )


def _normal_force(model: Any, data: Any, contact_index: int) -> float:
    import mujoco
    import numpy as np

    value = np.zeros(6, dtype=float)
    mujoco.mj_contactForce(model, data, contact_index, value)
    return _finite(value[0], "contact normal force")


def _active(contact: Any) -> bool:
    """A contact only constrains the model when it owns an equation row.

    Contacts inside ``margin`` are listed even when they are separated by a
    positive gap; those have ``efc_address == -1`` and exert no force.  They
    must not count as holding, support or an unreleased finger.
    """
    return int(contact.efc_address) >= 0


def _tcp_pose(model: Any, data: Any, site_name: str) -> tuple[tuple[float, ...], tuple[float, ...]]:
    import mujoco
    import numpy as np

    try:
        site_id = int(model.site(site_name).id)
    except KeyError as error:
        raise ContractError("MODEL_MISMATCH", f"TCP site {site_name!r} is missing") from error
    position = tuple(_finite(value, f"{site_name}.position") for value in data.site_xpos[site_id])
    quaternion = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(
        quaternion, np.asarray(data.site_xmat[site_id], dtype=np.float64).reshape(9)
    )
    return position, tuple(_finite(value, f"{site_name}.quaternion") for value in quaternion)


def build_contact_snapshot(
    scene: CompiledScene,
    *,
    gripper_openings: Mapping[str, float],
    capture_seq: int,
) -> ContactSnapshot:
    """Capture one immutable snapshot; callers refresh forces first."""
    model = scene.model
    data = scene.data
    object_names = scene.contact_object_names
    geometry = scene.robot_geometry
    object_addresses = scene.object_free_joint_qpos
    object_bodies: dict[int, str] = {}
    for name in object_names:
        if name not in object_addresses:
            raise ContractError("MODEL_MISMATCH", f"object {name!r} has no qpos address")
        try:
            object_bodies[int(model.body(name).id)] = name
        except KeyError as error:
            raise ContractError("MODEL_MISMATCH", f"object body {name!r} is missing") from error

    finger_bodies: dict[int, tuple[str, int, str]] = {}
    finger_names: dict[tuple[str, int], str] = {}
    for side, specs in geometry.finger_bodies.items():
        if not specs:
            raise ContractError("MODEL_MISMATCH", f"{side} must bind at least one finger")
        for finger_index, (body_name, _joint_name) in enumerate(specs, start=1):
            try:
                finger_bodies[int(model.body(body_name).id)] = (side, finger_index, body_name)
            except KeyError as error:
                raise ContractError(
                    "MODEL_MISMATCH", f"finger body {body_name!r} is missing"
                ) from error
            finger_names[(side, finger_index)] = body_name

    finger_totals: dict[tuple[str, int, str], list[float | int]] = {}
    support_geom_totals: dict[tuple[str, str, str], list[float | int]] = {}
    body_names = scene.body_names
    geom_names = scene.geom_names
    for contact_index in range(int(data.ncon)):
        contact = data.contact[contact_index]
        if not _active(contact):
            continue
        body1 = int(model.geom_bodyid[int(contact.geom1)])
        body2 = int(model.geom_bodyid[int(contact.geom2)])
        force = _normal_force(model, data, contact_index)
        finger1 = finger_bodies.get(body1)
        finger2 = finger_bodies.get(body2)
        if finger1 is not None and body2 in object_bodies:
            key = (finger1[0], finger1[1], object_bodies[body2])
            total = finger_totals.setdefault(key, [0.0, 0])
            total[0] = float(total[0]) + abs(force)
            total[1] = int(total[1]) + 1
        elif finger2 is not None and body1 in object_bodies:
            key = (finger2[0], finger2[1], object_bodies[body1])
            total = finger_totals.setdefault(key, [0.0, 0])
            total[0] = float(total[0]) + abs(force)
            total[1] = int(total[1]) + 1
        elif body1 in object_bodies and finger2 is None:
            key = (
                object_bodies[body1],
                body_names[body2],
                geom_names[int(contact.geom2)],
            )
            total = support_geom_totals.setdefault(key, [0.0, 0, 0.0])
            total[0] = float(total[0]) + abs(force)
            total[1] = int(total[1]) + 1
            # The normal points from geom1 to geom2 and mj_contactForce
            # reports the force on geom2, so the object's upward force flips
            # sign when the object owns geom1.
            total[2] = float(total[2]) - force * float(contact.frame[2])
        elif body2 in object_bodies and finger1 is None:
            key = (
                object_bodies[body2],
                body_names[body1],
                geom_names[int(contact.geom1)],
            )
            total = support_geom_totals.setdefault(key, [0.0, 0, 0.0])
            total[0] = float(total[0]) + abs(force)
            total[1] = int(total[1]) + 1
            total[2] = float(total[2]) + force * float(contact.frame[2])

    objects = tuple(
        _object_motion(model, data, name, int(object_addresses[name])) for name in object_names
    )
    finger_contacts = tuple(
        FingerObjectContact(
            side=side,
            finger_index=finger_index,
            finger_body=finger_names[(side, finger_index)],
            object_name=object_name,
            normal_force_n=float(total[0]),
            contact_count=int(total[1]),
        )
        for (side, finger_index, object_name), total in sorted(finger_totals.items())
    )
    support_geom_contacts = tuple(
        SupportGeomContact(
            object_name=object_name,
            body_name=body_name,
            geom_name=geom_name,
            normal_force_n=float(total[0]),
            object_upward_force_n=float(total[2]),
            contact_count=int(total[1]),
        )
        for (object_name, body_name, geom_name), total in sorted(
            support_geom_totals.items()
        )
    )
    support_body_totals: dict[tuple[str, str], list[float | int]] = {}
    for contact in support_geom_contacts:
        key = (contact.object_name, contact.body_name)
        total = support_body_totals.setdefault(key, [0.0, 0])
        total[0] = float(total[0]) + contact.normal_force_n
        total[1] = int(total[1]) + contact.contact_count
    support_contacts = tuple(
        SupportContact(
            object_name=object_name,
            body_name=body_name,
            normal_force_n=float(total[0]),
            contact_count=int(total[1]),
        )
        for (object_name, body_name), total in sorted(support_body_totals.items())
    )
    grippers: list[GripperPose] = []
    for chain in geometry.chains:
        side = chain.side
        position, quaternion = _tcp_pose(model, data, chain.tcp_site)
        grippers.append(
            GripperPose(
                side=side,
                opening_m=_finite(gripper_openings[side], f"{side}_gripper_width"),
                tcp_position=position,
                tcp_quaternion_wxyz=quaternion,
            )
        )
    from robot_backend_sim.common.physics.robot_io import sim_time_ns

    return ContactSnapshot(
        sim_time_ns=sim_time_ns(data.time),
        capture_seq=int(capture_seq),
        objects=objects,
        finger_contacts=finger_contacts,
        support_contacts=support_contacts,
        support_geom_contacts=support_geom_contacts,
        grippers=tuple(grippers),
    )
