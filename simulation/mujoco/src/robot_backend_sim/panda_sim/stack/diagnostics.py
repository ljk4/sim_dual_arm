"""Stack scene document projection for check reports."""

from __future__ import annotations

from robot_backend_sim.panda_sim.stack.scene_config import SceneConfig

__all__ = ["scene_config_document"]


def scene_config_document(config: SceneConfig) -> dict[str, object]:
    return {
        "source": config.source,
        "physics": {
            "timestep": config.physics.timestep,
            "gravity": list(config.physics.gravity),
            "integrator": config.physics.integrator,
            "object_friction": list(config.physics.object_friction),
        },
        "column": {
            "radius": config.column.radius,
            "height": config.column.height,
            "rgba": list(config.column.rgba),
        },
        "mounts": {
            "base_x": config.mounts.base_x,
            "base_y": config.mounts.base_y,
            "base_z": config.mounts.base_z,
            "yaw_deg": config.mounts.yaw_deg,
            "bracket_length": config.mounts.bracket_length,
            "bracket_width": config.mounts.bracket_width,
            "bracket_height": config.mounts.bracket_height,
            "tcp_position": list(config.mounts.tcp_position),
            "rgba": list(config.mounts.rgba),
        },
        "table": {
            "center_xy": list(config.table.center_xy),
            "size_xy": list(config.table.size_xy),
            "top_z": config.table.top_z,
            "thickness": config.table.thickness,
            "rgba": list(config.table.rgba),
        },
        "objects": [
            {
                "name": obj.name,
                "half_extents": list(obj.half_extents),
                "nominal_xy": list(obj.nominal_xy),
                "mass": obj.mass,
                "rgba": list(obj.rgba),
                "randomization": {
                    "xy_m": obj.randomization.xy_m,
                    "yaw_deg": obj.randomization.yaw_deg,
                },
            }
            for obj in config.objects
        ],
        "cameras": [
            {
                "name": camera.name,
                "parent": camera.parent,
                "position": list(camera.position),
                "quaternion_wxyz": list(camera.quaternion_wxyz),
                "fovy": camera.fovy,
                "width": camera.width,
                "height": camera.height,
            }
            for camera in config.cameras
        ],
        "reset": {
            "home_joint_positions": list(config.reset.home_joint_positions),
            "gripper_total_opening": config.reset.gripper_total_opening,
        },
    }
