"""Diagnostic report helpers for Marvin stack offline checks."""

from __future__ import annotations

from robot_backend_sim.common.config.camera_config import camera_declared_description
from robot_backend_sim.marvin_sim.stack.scene_config import MarvinStackSceneConfig

__all__ = ["scene_config_document"]


def scene_config_document(scene_config: MarvinStackSceneConfig) -> dict[str, object]:
    """Serialize one MarvinStackSceneConfig into an auditable JSON dictionary."""
    return {
        "physics": {
            "timestep": scene_config.physics.timestep,
            "gravity": list(scene_config.physics.gravity),
            "integrator": scene_config.physics.integrator,
            "cone": scene_config.physics.cone,
            "impratio": scene_config.physics.impratio,
            "noslip_iterations": scene_config.physics.noslip_iterations,
            "object_contact_dim": scene_config.physics.object_contact_dim,
            "object_friction": list(scene_config.physics.object_friction),
        },
        "robot": {
            "assembly": scene_config.robot.assembly,
            "base_position": list(scene_config.robot.base_position),
            "base_yaw_deg": scene_config.robot.base_yaw_deg,
            "home_joint_positions": list(scene_config.robot.home_joint_positions),
            "gravity_compensation": scene_config.robot.gravity_compensation,
            "gripper_open_total": scene_config.robot.gripper_open_total,
            "gripper_closed_total": scene_config.robot.gripper_closed_total,
        },
        "task": {
            "environment": scene_config.task.environment,
            "frame": scene_config.task.frame,
            "table": {
                "center_xy": list(scene_config.task.table.center_xy),
                "size_xy": list(scene_config.task.table.size_xy),
                "top_z": scene_config.task.table.top_z,
                "thickness": scene_config.task.table.thickness,
                "rgba": list(scene_config.task.table.rgba),
            },
            "objects": [
                {
                    "name": obj.name,
                    "half_extents": list(obj.half_extents),
                    "nominal_xy": list(obj.nominal_xy),
                    "nominal_yaw_deg": obj.nominal_yaw_deg,
                    "mass": obj.mass,
                    "rgba": list(obj.rgba),
                }
                for obj in scene_config.objects
            ],
            "camera_mount": {
                "position": list(scene_config.task.camera_mount.position),
            },
        },
        "cameras": [camera_declared_description(camera) for camera in scene_config.cameras],
    }
