"""Stack task environment: table, cubes, camera mount and lights."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

import mujoco

from robot_backend_sim.common.physics.scene_container import SceneBuildError
from robot_backend_sim.marvin.scene_base import (
    add_marvin_camera_mount,
    add_marvin_lights,
    add_marvin_table,
    base_world_point,
    combined_yaw_quaternion,
)
from robot_backend_sim.marvin.stack.scene_config import StackEnvironmentConfig

__all__ = [
    "StackBinding",
    "add_stack_environment",
    "bind_stack_environment",
]


@dataclass(frozen=True, slots=True)
class StackBinding:
    object_free_joint_qpos: Mapping[str, int]
    object_body_ids: Mapping[str, int]
    object_geom_ids: Mapping[str, int]
    table_body_id: int
    table_geom_id: int
    camera_mount_body_id: int


def add_stack_environment(
    spec: mujoco.MjSpec,
    task: StackEnvironmentConfig,
    friction: tuple[float, float, float],
    base_position: tuple[float, float, float],
    base_yaw_deg: float,
) -> None:
    """Add lights, table, source/target cubes and camera mount to an MjSpec."""
    surface_friction = list(friction)
    add_marvin_lights(spec)
    add_marvin_table(spec, task.table, friction, base_position, base_yaw_deg)
    for obj in (task.source, task.target):
        body = spec.worldbody.add_body(
            name=obj.name,
            pos=list(
                base_world_point(
                    (
                        obj.nominal_xy[0],
                        obj.nominal_xy[1],
                        task.table.top_z + obj.half_extents[2],
                    ),
                    base_position,
                    base_yaw_deg,
                )
            ),
            quat=list(combined_yaw_quaternion(base_yaw_deg, obj.nominal_yaw_deg)),
        )
        body.add_freejoint(name=f"{obj.name}_free")
        body.add_geom(
            name=f"{obj.name}_geom",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=list(obj.half_extents),
            mass=obj.mass,
            rgba=list(obj.rgba),
            contype=1,
            conaffinity=1,
            friction=surface_friction,
        )
    add_marvin_camera_mount(spec, task.camera_mount, base_position, base_yaw_deg)


def bind_stack_environment(
    model: mujoco.MjModel, task: StackEnvironmentConfig
) -> StackBinding:
    try:
        table_body = model.body("table")
        table_geom = model.geom("table_top")
        camera_body = model.body("camera_mount")
    except KeyError as error:
        raise SceneBuildError(f"stack environment is missing {error}") from error
    qpos_map: dict[str, int] = {}
    body_map: dict[str, int] = {}
    geom_map: dict[str, int] = {}
    for obj in (task.source, task.target):
        try:
            joint = model.joint(f"{obj.name}_free")
            body = model.body(obj.name)
            geom = model.geom(f"{obj.name}_geom")
        except KeyError as error:
            raise SceneBuildError(f"stack environment is missing {error}") from error
        if int(model.jnt_type[joint.id]) != int(mujoco.mjtJoint.mjJNT_FREE):
            raise SceneBuildError(f"object joint {obj.name!r} is not free")
        qpos_map[obj.name] = int(model.jnt_qposadr[joint.id])
        body_map[obj.name] = int(body.id)
        geom_map[obj.name] = int(geom.id)
    return StackBinding(
        object_free_joint_qpos=MappingProxyType(qpos_map),
        object_body_ids=MappingProxyType(body_map),
        object_geom_ids=MappingProxyType(geom_map),
        table_body_id=int(table_body.id),
        table_geom_id=int(table_geom.id),
        camera_mount_body_id=int(camera_body.id),
    )
