"""Stack task environment: column, mounts, table, free objects and lights.

This module owns only task geometry and layout.  It does not know how the
robot is attached, how objects are grasped or which arm moves first.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

import mujoco

from robot_backend_sim.panda_sim.assembly import SIDES, SIDE_SIGNS, _qpos_address
from robot_backend_sim.panda_sim.stack.scene_config import SceneConfig

__all__ = ["StackBinding", "add_stack_environment", "bind_stack_environment"]


@dataclass(frozen=True, slots=True)
class StackBinding:
    """Named MuJoCo addresses for the stack task environment."""

    object_free_joint_qpos: Mapping[str, int]


def add_stack_environment(spec: mujoco.MjSpec, config: SceneConfig) -> None:
    """Add the column, mounts, table, free objects and lights."""
    column = spec.worldbody.add_body(name="column")
    column.add_geom(
        name="column_geom",
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        size=[config.column.radius, config.column.height / 2.0, 0.0],
        pos=[0.0, 0.0, config.column.height / 2.0],
        rgba=list(config.column.rgba),
        contype=1,
        conaffinity=1,
    )
    for side in SIDES:
        sign = SIDE_SIGNS[side]
        bracket_x = sign * (config.mounts.base_x - config.mounts.bracket_length / 2.0)
        spec.worldbody.add_geom(
            name=f"{side}_mount_bracket",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[
                config.mounts.bracket_length / 2.0,
                config.mounts.bracket_width / 2.0,
                config.mounts.bracket_height / 2.0,
            ],
            pos=[bracket_x, config.mounts.base_y, config.mounts.base_z],
            rgba=list(config.mounts.rgba),
            contype=1,
            conaffinity=1,
        )

    table = spec.worldbody.add_body(name="table")
    table.add_geom(
        name="table_top",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[
            config.table.size_xy[0] / 2.0,
            config.table.size_xy[1] / 2.0,
            config.table.thickness / 2.0,
        ],
        pos=[
            config.table.center_xy[0],
            config.table.center_xy[1],
            config.table.top_z - config.table.thickness / 2.0,
        ],
        rgba=list(config.table.rgba),
        contype=1,
        conaffinity=1,
    )

    for obj in config.objects:
        z = config.table.top_z + obj.half_extents[2]
        body = spec.worldbody.add_body(name=obj.name, pos=[obj.nominal_xy[0], obj.nominal_xy[1], z])
        body.add_freejoint(name=f"{obj.name}_free")
        body.add_geom(
            name=f"{obj.name}_geom",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=list(obj.half_extents),
            mass=obj.mass,
            rgba=list(obj.rgba),
            friction=list(config.physics.object_friction),
            condim=4 if config.physics.object_friction[1] > 0.0 else 3,
            contype=1,
            conaffinity=1,
        )

    spec.worldbody.add_light(
        name="key_light",
        pos=[0.25, 0.35, 2.4],
        dir=[-0.15, -0.25, -1.0],
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
        diffuse=[0.8, 0.8, 0.8],
        specular=[0.2, 0.2, 0.2],
        castshadow=True,
    )
    spec.worldbody.add_light(
        name="fill_light",
        pos=[-0.60, 0.90, 1.80],
        dir=[0.30, -0.35, -1.0],
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
        diffuse=[0.35, 0.35, 0.35],
    )


def bind_stack_environment(model: mujoco.MjModel, config: SceneConfig) -> StackBinding:
    object_joints = {obj.name: _qpos_address(model, f"{obj.name}_free") for obj in config.objects}
    return StackBinding(object_free_joint_qpos=MappingProxyType(object_joints))
