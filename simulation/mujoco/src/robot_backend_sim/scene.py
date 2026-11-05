"""Build and deterministically reset the dual-Panda stack scene.

The robot bodies come from two attached copies of the pinned Menagerie Panda;
only the column, mounts, table, objects, cameras and lights are custom.  This
module does not implement ROS, IK, rendering or the 16-dimensional execution
mapping.
"""

from __future__ import annotations

import math
import random
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType

import mujoco

from . import assets
from .scene_config import SceneConfig

__all__ = ["Scene", "SceneBuildError", "SceneIndices", "build_scene", "gripper_ctrl"]

SIDES = ("left", "right")
SIDE_SIGNS = {"left": -1.0, "right": 1.0}
GRIPPER_CTRL_RANGE = 255.0
FINGER_OPENING_RANGE = 0.08
_INTEGRATORS = {
    "euler": mujoco.mjtIntegrator.mjINT_EULER,
    "implicit": mujoco.mjtIntegrator.mjINT_IMPLICIT,
    "implicitfast": mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
    "rk4": mujoco.mjtIntegrator.mjINT_RK4,
}


class SceneBuildError(RuntimeError):
    """The fixed assets or the assembled MuJoCo model are not usable."""


@dataclass(frozen=True, slots=True)
class SceneIndices:
    """Model indices looked up by stable element name, never by adjacency."""

    arm_joint_qpos: Mapping[str, tuple[int, ...]]
    finger_joint_qpos: Mapping[str, tuple[int, int]]
    arm_actuator_ids: Mapping[str, tuple[int, ...]]
    gripper_actuator_ids: Mapping[str, int]
    object_free_joint_qpos: Mapping[str, int]
    camera_ids: Mapping[str, int]
    tcp_site_ids: Mapping[str, int]


class Scene:
    """One compiled scene and its writable MuJoCo data."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        config: SceneConfig,
        indices: SceneIndices,
    ) -> None:
        self.model = model
        self.data = data
        self.config = config
        self.indices = indices
        self._gripper_ctrl = gripper_ctrl(config.reset.gripper_total_opening)

    def reset(self, seed: int) -> None:
        """Restart one episode from the configured initial state.

        The local generator makes the same seed reproducible without touching
        Python's global random state.  This is an episode-boundary operation,
        not an interface for moving objects while an episode is running.
        """
        if isinstance(seed, bool) or not isinstance(seed, int):
            raise TypeError("seed must be an integer")
        generator = random.Random(seed)
        model = self.model
        data = self.data
        mujoco.mj_resetData(model, data)

        # Clear every writable run-state field explicitly; mj_resetData also
        # handles fields added by newer MuJoCo releases.
        data.time = 0.0
        data.qpos[:] = model.qpos0
        data.qvel[:] = 0.0
        data.act[:] = 0.0
        data.ctrl[:] = 0.0
        data.qfrc_applied[:] = 0.0
        data.xfrc_applied[:] = 0.0
        data.qacc_warmstart[:] = 0.0

        half_opening = self.config.reset.gripper_total_opening / 2.0
        for side in SIDES:
            for address, value in zip(
                self.indices.arm_joint_qpos[side],
                self.config.reset.home_joint_positions,
                strict=True,
            ):
                data.qpos[address] = value
            for address in self.indices.finger_joint_qpos[side]:
                data.qpos[address] = half_opening
            for actuator, value in zip(
                self.indices.arm_actuator_ids[side],
                self.config.reset.home_joint_positions,
                strict=True,
            ):
                data.ctrl[actuator] = value
            data.ctrl[self.indices.gripper_actuator_ids[side]] = self._gripper_ctrl

        for obj in self.config.objects:
            address = self.indices.object_free_joint_qpos[obj.name]
            x = obj.nominal_xy[0] + generator.uniform(
                -obj.randomization.xy_m, obj.randomization.xy_m
            )
            y = obj.nominal_xy[1] + generator.uniform(
                -obj.randomization.xy_m, obj.randomization.xy_m
            )
            yaw = math.radians(
                generator.uniform(-obj.randomization.yaw_deg, obj.randomization.yaw_deg)
            )
            z = self.config.table.top_z + obj.half_extents[2]
            data.qpos[address : address + 7] = (x, y, z, *_yaw_quaternion(yaw))

        mujoco.mj_forward(model, data)


def gripper_ctrl(total_opening: float) -> float:
    """Map metre total finger opening to the vendor's raw 0..255 command.

    The two finger slides each span 0..0.04 m, so a fully open gripper is
    0.08 m total.  The Menagerie actuator8 command is dimensionless in
    0..255, not metres; the upstream home pose is 0.04 m per finger with
    command 255.  Keep the conversion continuous: 0.04 m total maps to 127.5.
    Callers validate the configured or requested bounds before using it.
    """
    return total_opening / FINGER_OPENING_RANGE * GRIPPER_CTRL_RANGE


def _yaw_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw / 2.0
    quaternion = (math.cos(half), 0.0, 0.0, math.sin(half))
    norm = math.sqrt(sum(component * component for component in quaternion))
    return (
        quaternion[0] / norm,
        quaternion[1] / norm,
        quaternion[2] / norm,
        quaternion[3] / norm,
    )


def _mount_quaternion(yaw_deg: float) -> tuple[float, float, float, float]:
    half = math.radians(yaw_deg) / 2.0
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def _add_static_geometry(spec: mujoco.MjSpec, config: SceneConfig) -> None:
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


def _add_objects(spec: mujoco.MjSpec, config: SceneConfig) -> None:
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
            contype=1,
            conaffinity=1,
        )


def _add_lights(spec: mujoco.MjSpec) -> None:
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


def _attach_robots(spec: mujoco.MjSpec, config: SceneConfig, panda_xml: Path) -> None:
    yaw_quaternion = _mount_quaternion(config.mounts.yaw_deg)
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
                SIDE_SIGNS[side] * config.mounts.base_x,
                config.mounts.base_y,
                config.mounts.base_z,
            ],
            quat=list(yaw_quaternion),
        )
        spec.attach(child, prefix=f"{side}_", frame=frame)
        spec.body(f"{side}_hand").add_site(
            name=f"{side}_tcp", pos=list(config.mounts.tcp_position), size=[0.005, 0.0, 0.0]
        )


def _add_cameras(spec: mujoco.MjSpec, config: SceneConfig) -> None:
    for camera in config.cameras:
        try:
            parent = spec.body(camera.parent)
        except KeyError as error:
            raise SceneBuildError(
                f"camera {camera.name!r} parent {camera.parent!r} is missing"
            ) from error
        camera_spec = parent.add_camera(
            name=camera.name,
            pos=list(camera.position),
            quat=list(camera.quaternion_wxyz),
            fovy=camera.fovy,
        )
        # Keep the MJCF camera and offscreen framebuffer aligned with the one
        # configured resolution; rendering is never silently downscaled.
        camera_spec.resolution = [camera.width, camera.height]


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


def _camera_id(model: mujoco.MjModel, camera_name: str) -> int:
    try:
        return int(model.camera(camera_name).id)
    except KeyError as error:
        raise SceneBuildError(f"expected camera {camera_name!r} is missing") from error


def _site_id(model: mujoco.MjModel, site_name: str) -> int:
    try:
        return int(model.site(site_name).id)
    except KeyError as error:
        raise SceneBuildError(f"expected site {site_name!r} is missing") from error


def _build_indices(model: mujoco.MjModel, config: SceneConfig) -> SceneIndices:
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
    object_joints = {obj.name: _qpos_address(model, f"{obj.name}_free") for obj in config.objects}
    cameras = {camera.name: _camera_id(model, camera.name) for camera in config.cameras}
    sites = {f"{side}_tcp": _site_id(model, f"{side}_tcp") for side in SIDES}
    return SceneIndices(
        arm_joint_qpos=MappingProxyType(arm_joints),
        finger_joint_qpos=MappingProxyType(finger_joints),
        arm_actuator_ids=MappingProxyType(arm_actuators),
        gripper_actuator_ids=MappingProxyType(gripper_actuators),
        object_free_joint_qpos=MappingProxyType(object_joints),
        camera_ids=MappingProxyType(cameras),
        tcp_site_ids=MappingProxyType(sites),
    )


def _check_model_shape(model: mujoco.MjModel, config: SceneConfig) -> None:
    expected = {
        "nq": 32,
        "nv": 30,
        "nu": 16,
        "ncam": 3,
        "nkey": 0,
    }
    actual = {
        "nq": model.nq,
        "nv": model.nv,
        "nu": model.nu,
        "ncam": model.ncam,
        "nkey": model.nkey,
    }
    if actual != expected:
        raise SceneBuildError(f"assembled model shape {actual} does not match {expected}")
    for camera in config.cameras:
        body_id = int(model.camera(camera.name).bodyid[0])
        actual_parent = model.body(body_id).name
        if actual_parent != camera.parent:
            raise SceneBuildError(
                f"camera {camera.name!r} is attached to {actual_parent!r}, "
                f"expected {camera.parent!r}"
            )
    if not (model.opt.gravity[2] < 0.0):
        raise SceneBuildError("scene gravity must point down for the free objects")


def build_scene(config: SceneConfig) -> Scene:
    """Attach two local Pandas and compile the scene once."""
    _, panda_xml = assets.model_paths()
    if not panda_xml.is_file():
        raise SceneBuildError(f"Panda model is missing: {panda_xml}")

    spec = mujoco.MjSpec()
    spec.modelname = "panda_stack"
    spec.option.timestep = config.physics.timestep
    spec.option.gravity = list(config.physics.gravity)
    spec.option.integrator = _INTEGRATORS[config.physics.integrator]
    _add_static_geometry(spec, config)
    _add_objects(spec, config)
    _attach_robots(spec, config, panda_xml)
    _add_cameras(spec, config)
    _add_lights(spec)
    spec.visual.global_.offwidth = max(camera.width for camera in config.cameras)
    spec.visual.global_.offheight = max(camera.height for camera in config.cameras)

    try:
        model = spec.compile()
    except Exception as error:
        raise SceneBuildError(f"MuJoCo scene compilation failed: {error}") from error
    _check_model_shape(model, config)
    data = mujoco.MjData(model)
    scene = Scene(model, data, config, _build_indices(model, config))
    scene.reset(0)
    return scene
