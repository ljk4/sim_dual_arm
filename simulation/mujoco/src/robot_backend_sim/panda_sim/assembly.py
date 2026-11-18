"""Attach and bind the dual-Panda robot assembly.

The robot bodies come from two attached copies of the pinned Menagerie Panda.
Only the mounts belong to the robot; the column, table, objects, cameras and
lights are the task environment.  This module implements no ROS, IK, rendering
or task execution mapping.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType

import mujoco

from robot_backend_sim.common.physics.scene_container import SceneBuildError
from robot_backend_sim.panda_sim.robot_config import MountsConfig

__all__ = [
    "FINGER_OPENING_RANGE",
    "GRIPPER_CTRL_RANGE",
    "PandaBinding",
    "SIDES",
    "SIDE_SIGNS",
    "attach_panda_robots",
    "bind_panda_robot",
    "gripper_ctrl",
]

SIDES = ("left", "right")
SIDE_SIGNS = {"left": -1.0, "right": 1.0}
GRIPPER_CTRL_RANGE = 255.0
FINGER_OPENING_RANGE = 0.08


@dataclass(frozen=True, slots=True)
class PandaBinding:
    """Named MuJoCo addresses for the dual-Panda robot contract."""

    arm_joint_qpos: Mapping[str, tuple[int, ...]]
    finger_joint_qpos: Mapping[str, tuple[int, int]]
    arm_actuator_ids: Mapping[str, tuple[int, ...]]
    gripper_actuator_ids: Mapping[str, int]
    tcp_site_ids: Mapping[str, int]


def gripper_ctrl(total_opening: float) -> float:
    """Map metre total finger opening to the vendor's raw 0..255 command.

    The two finger slides each span 0..0.04 m, so a fully open gripper is
    0.08 m total.  The Menagerie actuator8 command is dimensionless in
    0..255, not metres; the upstream home pose is 0.04 m per finger with
    command 255.  Keep the conversion continuous: 0.04 m total maps to 127.5.
    Callers validate the configured or requested bounds before using it.
    """
    return total_opening / FINGER_OPENING_RANGE * GRIPPER_CTRL_RANGE


def _mount_quaternion(yaw_deg: float) -> tuple[float, float, float, float]:
    half = math.radians(yaw_deg) / 2.0
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def _qpos_address(model: mujoco.MjModel, joint_name: str) -> int:
    try:
        return int(model.joint(joint_name).qposadr[0])
    except KeyError as error:
        raise SceneBuildError(f"expected joint {joint_name!r} is missing") from error


def _actuator_id(model: mujoco.MjModel, actuator_name: str) -> int:
    try:
        return int(model.actuator(actuator_name).id)
    except KeyError as error:
        raise SceneBuildError(f"expected actuator {actuator_name!r} is missing") from error


def _site_id(model: mujoco.MjModel, site_name: str) -> int:
    try:
        return int(model.site(site_name).id)
    except KeyError as error:
        raise SceneBuildError(f"expected site {site_name!r} is missing") from error


def attach_panda_robots(
    spec: mujoco.MjSpec, mounts: MountsConfig, panda_xml: Path
) -> None:
    yaw_quaternion = _mount_quaternion(mounts.yaw_deg)
    for side in SIDES:
        child = mujoco.MjSpec.from_file(str(panda_xml))
        # Vendor keyframes have 9 qpos entries for one Panda.  Delete them
        # before attachment so they are not expanded into a mixed 32-entry
        # parent keyframe; reset writes named positions explicitly below.
        for key in list(child.keys):
            child.delete(key)
        for light in list(child.lights):
            child.delete(light)
        frame = spec.worldbody.add_frame(
            name=f"{side}_mount",
            pos=[
                SIDE_SIGNS[side] * mounts.base_x,
                mounts.base_y,
                mounts.base_z,
            ],
            quat=list(yaw_quaternion),
        )
        spec.attach(child, prefix=f"{side}_", frame=frame)
        spec.body(f"{side}_hand").add_site(
            name=f"{side}_tcp", pos=list(mounts.tcp_position), size=[0.005, 0.0, 0.0]
        )


def bind_panda_robot(model: mujoco.MjModel) -> PandaBinding:
    """Resolve every name/address used by the dual-Panda robot contract."""
    arm_joints = {
        side: tuple(_qpos_address(model, f"{side}_joint{joint}") for joint in range(1, 8))
        for side in SIDES
    }
    finger_joints = {
        side: (
            _qpos_address(model, f"{side}_finger_joint1"),
            _qpos_address(model, f"{side}_finger_joint2"),
        )
        for side in SIDES
    }
    arm_actuators = {
        side: tuple(_actuator_id(model, f"{side}_actuator{joint}") for joint in range(1, 8))
        for side in SIDES
    }
    gripper_actuators = {side: _actuator_id(model, f"{side}_actuator8") for side in SIDES}
    sites = {f"{side}_tcp": _site_id(model, f"{side}_tcp") for side in SIDES}
    return PandaBinding(
        arm_joint_qpos=MappingProxyType(arm_joints),
        finger_joint_qpos=MappingProxyType(finger_joints),
        arm_actuator_ids=MappingProxyType(arm_actuators),
        gripper_actuator_ids=MappingProxyType(gripper_actuators),
        tcp_site_ids=MappingProxyType(sites),
    )
