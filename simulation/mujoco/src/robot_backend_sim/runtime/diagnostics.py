"""Small shared audit helpers used by the one-shot S2 checks.

These functions have no scene, rendering or ROS dependency.  Keeping them in
one place avoids the new kinematics entry copying the accepted scene-check
provenance and atomic-write logic.
"""

from __future__ import annotations

import hashlib
import json
import os
import tempfile
from datetime import UTC, datetime
from pathlib import Path

from robot_backend_sim.config.scene_config import SceneConfig

__all__ = [
    "code_provenance",
    "python_source_manifest",
    "scene_config_document",
    "sha256_file",
    "utc_now",
    "write_json_atomic",
]


def utc_now() -> str:
    return datetime.now(UTC).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json_atomic(path: Path, document: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    body = (json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()
    handle = tempfile.NamedTemporaryFile(
        mode="wb", dir=path.parent, prefix=f".{path.name}.", delete=False
    )
    temporary = Path(handle.name)
    try:
        with handle:
            handle.write(body)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise


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


def python_source_manifest(directory: Path) -> dict[str, object]:
    """Hash a flat source package independently of its mount location."""
    files = {path.name: sha256_file(path) for path in sorted(directory.glob("*.py"))}
    if not files:
        raise OSError(f"no Python sources in {directory}")
    encoded = json.dumps(files, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return {
        "directory": str(directory),
        "files": files,
        "sha256": hashlib.sha256(encoded).hexdigest(),
    }


def code_provenance() -> dict[str, object]:
    import robot_run_logging

    return {
        "robot_backend_sim": python_source_manifest(Path(__file__).resolve().parent.parent),
        "robot_run_logging": python_source_manifest(
            Path(robot_run_logging.__file__).resolve().parent
        ),
    }
