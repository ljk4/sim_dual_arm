"""One immutable owner-thread measurement of the handover task facts.

The builder reads one compiled ``MarvinScene`` and returns named finger, object,
support and robot-collision evidence.  It never steps physics or writes state.
Task-side interpretation (holding, lift, support, placement) stays in the
handover evaluator; robot naming comes from the simulation robot geometry.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from robot_contracts.models import ContractError

from robot_backend_sim.physics.contact_snapshot import build_contact_snapshot
from robot_backend_sim.physics.marvin.assembly import MarvinBinding
from robot_backend_sim.physics.marvin.scene import gripper_opening

__all__ = [
    "FingerContact",
    "HandoverBinding",
    "HandoverObservation",
    "SupportContact",
    "capture_handover_observation",
    "make_handover_binding",
]


@dataclass(frozen=True, slots=True)
class FingerContact:
    finger_index: int
    normal_force_n: float
    contact_count: int


@dataclass(frozen=True, slots=True)
class SupportContact:
    body_name: str
    normal_force_n: float
    contact_count: int


@dataclass(frozen=True, slots=True)
class HandoverBinding:
    """Fixed model names and robot groups resolved once after scene binding."""

    block_name: str
    block_body_id: int
    block_geom_name: str
    block_qpos_address: int
    block_dof_address: int
    table_body_id: int
    box_body_id: int
    support_body_names: tuple[str, ...]
    left_finger_dof_addresses: tuple[int, int]
    right_finger_dof_addresses: tuple[int, int]
    left_body_ids: frozenset[int]
    right_body_ids: frozenset[int]
    robot_body_ids: frozenset[int]
    table_top_z_m: float
    block_half_extents_m: tuple[float, float, float]
    box_inner_bottom_z_m: float
    box_center_xy: tuple[float, float]
    box_inner_xy: tuple[float, float]
    box_yaw_rad: float
    block_initial_center: tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class HandoverObservation:
    sim_time_ns: int
    block_center: tuple[float, float, float]
    block_quaternion_wxyz: tuple[float, float, float, float]
    block_linear_velocity: tuple[float, float, float]
    block_angular_velocity: tuple[float, float, float]
    left_positions: tuple[float, ...]
    right_positions: tuple[float, ...]
    left_velocities: tuple[float, ...]
    right_velocities: tuple[float, ...]
    left_opening_m: float
    right_opening_m: float
    left_finger_speed_m_s: float
    right_finger_speed_m_s: float
    left_contacts: tuple[FingerContact, ...]
    right_contacts: tuple[FingerContact, ...]
    support_contacts: tuple[SupportContact, ...]
    left_tcp_position: tuple[float, float, float]
    left_tcp_quaternion_wxyz: tuple[float, float, float, float]
    right_tcp_position: tuple[float, float, float]
    right_tcp_quaternion_wxyz: tuple[float, float, float, float]
    robot_robot_force_n: float
    robot_box_force_n: float

    @property
    def block_height_above_initial_m(self) -> float:
        return self.block_center[2] - self._initial_center[2]

    _initial_center: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def finger_contacts(self, side: str) -> tuple[FingerContact, ...]:
        return self.left_contacts if side == "left" else self.right_contacts

    def holding(self, side: str, minimum_force_n: float) -> bool:
        contacts = self.finger_contacts(side)
        indices = {contact.finger_index for contact in contacts if contact.contact_count > 0}
        return (
            len(contacts) == 2
            and indices == {1, 2}
            and all(
                contact.contact_count > 0 and contact.normal_force_n >= minimum_force_n
                for contact in contacts
            )
        )

    def support_force(self, body_name: str) -> float:
        return sum(
            contact.normal_force_n
            for contact in self.support_contacts
            if contact.body_name == body_name
        )

    def external_support_force(self, allowed: tuple[str, ...]) -> float:
        return sum(
            contact.normal_force_n
            for contact in self.support_contacts
            if contact.body_name not in allowed
        )


def _robot_groups(scene: Any) -> tuple[frozenset[int], frozenset[int], frozenset[int]]:
    geometry = scene.robot_geometry
    prefixes = tuple(geometry.body_prefixes)
    sides = {
        side: tuple(prefixes_side) for side, prefixes_side in geometry.side_prefixes.items()
    }
    left: set[int] = set()
    right: set[int] = set()
    everything: set[int] = set()
    for body_id in range(int(scene.model.nbody)):
        name = str(scene.model.body(body_id).name) or ""
        if not name.startswith(prefixes):
            continue
        everything.add(body_id)
        if name.startswith(sides["left"]):
            left.add(body_id)
        elif name.startswith(sides["right"]):
            right.add(body_id)
    return frozenset(left), frozenset(right), frozenset(everything)


def make_handover_binding(
    scene: Any,
    robot: MarvinBinding,
    *,
    layout: object,
) -> HandoverBinding:
    """Resolve task and robot names once on the owner thread."""
    model = scene.model
    config = scene.config
    block = config.task.block
    box = config.task.box
    try:
        block_body_id = int(model.body(block.name).id)
        table_body_id = int(model.body("table").id)
        box_body_id = int(model.body(box.name).id)
    except KeyError as error:
        raise ContractError("MODEL_MISMATCH", f"task body is missing: {error}") from error
    qpos_address = int(scene.object_free_joint_qpos[block.name])
    joint_id = int(model.body_jntadr[block_body_id])
    if int(model.body_jntnum[block_body_id]) != 1 or int(model.jnt_type[joint_id]) != 0:
        raise ContractError("MODEL_MISMATCH", "the task block must own one free joint")
    left, right, everything = _robot_groups(scene)
    if not left or not right:
        raise ContractError("MODEL_MISMATCH", "robot body prefixes do not split into sides")
    return HandoverBinding(
        block_name=block.name,
        block_body_id=block_body_id,
        block_geom_name=f"{block.name}_geom",
        block_qpos_address=qpos_address,
        block_dof_address=int(model.jnt_dofadr[joint_id]),
        table_body_id=table_body_id,
        box_body_id=box_body_id,
        support_body_names=tuple(scene.support_body_names),
        left_finger_dof_addresses=tuple(
            int(value) for value in robot.finger_joint_dof["left"]
        ),
        right_finger_dof_addresses=tuple(
            int(value) for value in robot.finger_joint_dof["right"]
        ),
        left_body_ids=left,
        right_body_ids=right,
        robot_body_ids=everything,
        table_top_z_m=float(layout.table_top_z),
        block_half_extents_m=tuple(layout.block_half_extents),
        box_inner_bottom_z_m=float(layout.box_inner_bottom_z),
        box_center_xy=(float(layout.box_center[0]), float(layout.box_center[1])),
        box_inner_xy=tuple(layout.box_inner_xy),
        box_yaw_rad=float(layout.box_yaw_rad),
        block_initial_center=tuple(layout.block_initial_center),
    )


def _opening(scene: Any, side: str) -> float:
    return float(
        gripper_opening(scene.model, scene.data, scene.indices.robot, side)
    )


def _normal_force(model: Any, data: Any, contact_index: int) -> float:
    import mujoco
    import numpy as np

    value = np.zeros(6, dtype=float)
    mujoco.mj_contactForce(model, data, contact_index, value)
    return abs(float(value[0]))


def _forbidden_robot_contacts(
    scene: Any, binding: HandoverBinding
) -> tuple[float, float]:
    """Return the worst robot-robot and robot-box normal force in this step."""
    model, data = scene.model, scene.data
    body_of_geom = [int(value) for value in model.geom_bodyid]
    robot_robot = 0.0
    robot_box = 0.0
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        body1 = body_of_geom[int(contact.geom1)]
        body2 = body_of_geom[int(contact.geom2)]
        if body1 in binding.robot_body_ids and body2 in binding.robot_body_ids:
            robot_robot = max(robot_robot, _normal_force(model, data, index))
        elif (body1 == binding.box_body_id and body2 in binding.robot_body_ids) or (
            body2 == binding.box_body_id and body1 in binding.robot_body_ids
        ):
            robot_box = max(robot_box, _normal_force(model, data, index))
    return robot_robot, robot_box


def capture_handover_observation(
    scene: Any, binding: HandoverBinding
) -> HandoverObservation:
    """Copy every task fact from one live ``MjData`` on the owner thread."""
    data = scene.data
    measurements = {
        side: _opening(scene, side) for side in ("left", "right")
    }
    snapshot = build_contact_snapshot(
        scene, gripper_openings=measurements, capture_seq=0
    )
    if not snapshot.objects:
        raise ContractError("INVALID_TASK_STATE", "the task block is not readable")
    block = snapshot.objects[0]
    left_contacts: list[FingerContact] = []
    right_contacts: list[FingerContact] = []
    for contact in snapshot.finger_contacts:
        if contact.object_name != binding.block_name:
            continue
        item = FingerContact(
            contact.finger_index, contact.normal_force_n, contact.contact_count
        )
        (left_contacts if contact.side == "left" else right_contacts).append(item)
    supports = tuple(
        SupportContact(contact.body_name, contact.normal_force_n, contact.contact_count)
        for contact in snapshot.support_contacts
        if contact.object_name == binding.block_name
    )
    grippers = {gripper.side: gripper for gripper in snapshot.grippers}
    robot = scene.indices.robot
    left_positions = tuple(
        float(data.qpos[address]) for address in robot.arm_joint_qpos["left"]
    )
    right_positions = tuple(
        float(data.qpos[address]) for address in robot.arm_joint_qpos["right"]
    )
    left_velocities = tuple(
        float(data.qvel[address]) for address in robot.arm_joint_dof["left"]
    )
    right_velocities = tuple(
        float(data.qvel[address]) for address in robot.arm_joint_dof["right"]
    )
    robot_robot, robot_box = _forbidden_robot_contacts(scene, binding)
    return HandoverObservation(
        sim_time_ns=round(float(data.time) * 1_000_000_000),
        block_center=block.center,
        block_quaternion_wxyz=block.quaternion_wxyz,
        block_linear_velocity=block.linear_velocity,
        block_angular_velocity=block.angular_velocity,
        left_positions=left_positions,
        right_positions=right_positions,
        left_velocities=left_velocities,
        right_velocities=right_velocities,
        left_opening_m=measurements["left"],
        right_opening_m=measurements["right"],
        left_finger_speed_m_s=max(
            abs(float(data.qvel[address])) for address in binding.left_finger_dof_addresses
        ),
        right_finger_speed_m_s=max(
            abs(float(data.qvel[address])) for address in binding.right_finger_dof_addresses
        ),
        left_contacts=tuple(left_contacts),
        right_contacts=tuple(right_contacts),
        support_contacts=supports,
        left_tcp_position=grippers["left"].tcp_position,
        left_tcp_quaternion_wxyz=grippers["left"].tcp_quaternion_wxyz,
        right_tcp_position=grippers["right"].tcp_position,
        right_tcp_quaternion_wxyz=grippers["right"].tcp_quaternion_wxyz,
        robot_robot_force_n=robot_robot,
        robot_box_force_n=robot_box,
        _initial_center=binding.block_initial_center,
    )


def quaternion_matrix(quaternion: tuple[float, float, float, float]) -> tuple[
    tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]
]:
    w, x, y, z = quaternion
    return (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
        (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
        (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
    )


def relative_block_pose(
    observation: HandoverObservation, side: str
) -> tuple[tuple[float, float, float], tuple[tuple[float, ...], ...]]:
    """Block pose expressed in the named TCP frame."""
    if side == "left":
        tcp_position = observation.left_tcp_position
        tcp_rotation = quaternion_matrix(observation.left_tcp_quaternion_wxyz)
    else:
        tcp_position = observation.right_tcp_position
        tcp_rotation = quaternion_matrix(observation.right_tcp_quaternion_wxyz)
    block_rotation = quaternion_matrix(observation.block_quaternion_wxyz)
    delta = tuple(
        observation.block_center[index] - tcp_position[index] for index in range(3)
    )
    position = tuple(
        sum(tcp_rotation[column][row] * delta[column] for column in range(3))
        for row in range(3)
    )
    rotation = tuple(
        tuple(
            sum(tcp_rotation[column][row] * block_rotation[column][other]
                for column in range(3))
            for other in range(3)
        )
        for row in range(3)
    )
    return position, rotation


def rotation_angle(first: tuple[tuple[float, ...], ...],
                   second: tuple[tuple[float, ...], ...]) -> float:
    trace = sum(
        sum(first[column][row] * second[column][row] for column in range(3))
        for row in range(3)
    )
    return math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0)))
