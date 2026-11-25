"""Handover task environment: fixed table, free block and open-top box.

This module owns only task geometry and layout.  It does not know the robot
model, how the block is grasped or which arm moves first.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

import mujoco

from robot_backend_sim.common.physics.scene_container import SceneBuildError
from robot_backend_sim.marvin.handover.scene_config import TaskConfig
from robot_backend_sim.marvin.scene_base import (
    add_marvin_camera_mount,
    add_marvin_lights,
    add_marvin_table,
    base_world_point,
    combined_yaw_quaternion,
)

__all__ = [
    "BOX_BOTTOM_GEOM",
    "BOX_GEOM_NAMES",
    "HandoverBinding",
    "add_handover_environment",
    "bind_handover_environment",
]

BOX_BOTTOM_GEOM = "box_bottom"
BOX_GEOM_NAMES = (
    BOX_BOTTOM_GEOM,
    "box_wall_x_pos",
    "box_wall_x_neg",
    "box_wall_y_pos",
    "box_wall_y_neg",
)


@dataclass(frozen=True, slots=True)
class HandoverBinding:
    block_free_joint_qpos: int
    block_geom_id: int
    table_geom_id: int
    box_geom_ids: Mapping[str, int]
    box_body_id: int
    camera_mount_body_id: int


def add_handover_environment(
    spec: mujoco.MjSpec,
    task: TaskConfig,
    friction: tuple[float, float, float],
    base_position: tuple[float, float, float],
    base_yaw_deg: float,
) -> None:
    """Add the table, open box, free block, camera mount and lights.

    Task coordinates are expressed in the robot base frame and are transformed
    by the declared base pose.  The environment does not read robot I/O.
    """
    surface_friction = list(friction)
    add_marvin_lights(spec)
    add_marvin_table(spec, task.table, friction, base_position, base_yaw_deg)
    table = task.table

    box = task.box
    outer_x, outer_y = box.outer_xy
    wall = box.wall_thickness
    bottom = box.bottom_thickness
    box_body = spec.worldbody.add_body(
        name=box.name,
        pos=list(
            base_world_point(
                (box.center_xy[0], box.center_xy[1], table.top_z),
                base_position,
                base_yaw_deg,
            )
        ),
        quat=list(combined_yaw_quaternion(base_yaw_deg, box.yaw_deg)),
    )
    box_body.add_geom(
        name="box_bottom",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[outer_x / 2.0, outer_y / 2.0, bottom / 2.0],
        pos=[0.0, 0.0, bottom / 2.0],
        rgba=list(box.rgba),
        contype=1,
        conaffinity=1,
        friction=surface_friction,
    )
    wall_height = (box.height - bottom) / 2.0
    wall_z = (box.height + bottom) / 2.0
    box_body.add_geom(
        name="box_wall_x_pos",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[wall / 2.0, outer_y / 2.0, wall_height],
        pos=[outer_x / 2.0 - wall / 2.0, 0.0, wall_z],
        rgba=list(box.rgba),
        contype=1,
        conaffinity=1,
        friction=surface_friction,
    )
    box_body.add_geom(
        name="box_wall_x_neg",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[wall / 2.0, outer_y / 2.0, wall_height],
        pos=[-outer_x / 2.0 + wall / 2.0, 0.0, wall_z],
        rgba=list(box.rgba),
        contype=1,
        conaffinity=1,
        friction=surface_friction,
    )
    box_body.add_geom(
        name="box_wall_y_pos",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[outer_x / 2.0 - wall, wall / 2.0, wall_height],
        pos=[0.0, outer_y / 2.0 - wall / 2.0, wall_z],
        rgba=list(box.rgba),
        contype=1,
        conaffinity=1,
        friction=surface_friction,
    )
    box_body.add_geom(
        name="box_wall_y_neg",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[outer_x / 2.0 - wall, wall / 2.0, wall_height],
        pos=[0.0, -outer_y / 2.0 + wall / 2.0, wall_z],
        rgba=list(box.rgba),
        contype=1,
        conaffinity=1,
        friction=surface_friction,
    )

    block = task.block
    block_body = spec.worldbody.add_body(
        name=block.name,
        pos=list(
            base_world_point(
                (
                    block.initial_xy[0],
                    block.initial_xy[1],
                    table.top_z + block.half_extents[2],
                ),
                base_position,
                base_yaw_deg,
            )
        ),
        quat=list(combined_yaw_quaternion(base_yaw_deg, block.initial_yaw_deg)),
    )
    block_body.add_freejoint(name=f"{block.name}_free")
    block_body.add_geom(
        name=f"{block.name}_geom",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=list(block.half_extents),
        mass=block.mass,
        rgba=list(block.rgba),
        contype=1,
        conaffinity=1,
        friction=surface_friction,
    )

    add_marvin_camera_mount(spec, task.camera_mount, base_position, base_yaw_deg)


def bind_handover_environment(model: mujoco.MjModel, task: TaskConfig) -> HandoverBinding:
    block = task.block
    try:
        free_joint = model.joint(f"{block.name}_free")
        block_geom = model.geom(f"{block.name}_geom")
        table_geom = model.geom("table_top")
        box_body = model.body(task.box.name)
        camera_body = model.body("camera_mount")
    except KeyError as error:
        raise SceneBuildError(f"handover environment is missing {error}") from error
    if int(model.jnt_type[free_joint.id]) != int(mujoco.mjtJoint.mjJNT_FREE):
        raise SceneBuildError(f"block joint {block.name!r} is not free")
    box_geoms: dict[str, int] = {}
    for name in BOX_GEOM_NAMES:
        try:
            box_geoms[name] = int(model.geom(name).id)
        except KeyError as error:
            raise SceneBuildError(f"open box is missing {name!r}") from error
    if len(box_geoms) != 5:
        raise SceneBuildError("open box must consist of a bottom and four walls")
    return HandoverBinding(
        block_free_joint_qpos=int(model.jnt_qposadr[free_joint.id]),
        block_geom_id=int(block_geom.id),
        table_geom_id=int(table_geom.id),
        box_geom_ids=MappingProxyType(box_geoms),
        box_body_id=int(box_body.id),
        camera_mount_body_id=int(camera_body.id),
    )
