"""Compose and deterministically reset the dual-Panda stack scene.

The robot assembly, stack task environment and the generic MuJoCo
compile/reset container are composed here by explicit configuration selection.
The task name does not choose the robot, and the robot assembly does not know
the stack stages.
"""

from __future__ import annotations

import math
import random
from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

import mujoco

from robot_backend_sim.common.physics import assets
from robot_backend_sim.common.physics.robot_geometry import ArmChain, RobotGeometry
from robot_backend_sim.common.physics.scene_container import (
    CompiledScene,
    SceneBuildError,
    compile_spec,
)
from robot_backend_sim.panda_sim.assembly import (
    SIDES,
    PandaBinding,
    attach_panda_robots,
    bind_panda_robot,
    gripper_ctrl,
)
from robot_backend_sim.panda_sim.stack.environment import (
    StackBinding,
    add_stack_environment,
    bind_stack_environment,
)
from robot_backend_sim.panda_sim.stack.scene_config import SceneConfig

__all__ = ["PandaSceneIndices", "Scene", "SceneBuildError", "build_scene"]

_INTEGRATORS = {
    "euler": mujoco.mjtIntegrator.mjINT_EULER,
    "implicit": mujoco.mjtIntegrator.mjINT_IMPLICIT,
    "implicitfast": mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
    "rk4": mujoco.mjtIntegrator.mjINT_RK4,
}


@dataclass(frozen=True, slots=True)
class PandaSceneIndices:
    """Model indices looked up by stable element name, never by adjacency."""

    robot: PandaBinding
    environment: StackBinding
    camera_ids: Mapping[str, int]


class Scene(CompiledScene):
    """One compiled scene and its writable MuJoCo data."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        config: SceneConfig,
        indices: PandaSceneIndices,
    ) -> None:
        super().__init__(model, data)
        self.config = config
        self.indices = indices
        self._gripper_ctrl = gripper_ctrl(config.reset.gripper_total_opening)

    @property
    def contact_object_names(self) -> tuple[str, ...]:
        return tuple(obj.name for obj in self.config.objects)

    @property
    def robot_geometry(self) -> RobotGeometry:
        return RobotGeometry(
            chains=tuple(
                ArmChain(
                    side=side,
                    joint_names=tuple(f"{side}_joint{index}" for index in range(1, 8)),
                    tcp_site=f"{side}_tcp",
                )
                for side in SIDES
            ),
            body_prefixes=("left_", "right_"),
            side_prefixes={"left": ("left_",), "right": ("right_",)},
            finger_bodies={
                "left": (
                    ("left_left_finger", "left_finger_joint1"),
                    ("left_right_finger", "left_finger_joint2"),
                ),
                "right": (
                    ("right_left_finger", "right_finger_joint1"),
                    ("right_right_finger", "right_finger_joint2"),
                ),
            },
        )

    @property
    def object_free_joint_qpos(self) -> Mapping[str, int]:
        return self.indices.environment.object_free_joint_qpos

    @property
    def support_body_names(self) -> tuple[str, ...]:
        return ("column", "table")

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
        self.reset_run_state()

        half_opening = self.config.reset.gripper_total_opening / 2.0
        robot = self.indices.robot
        for side in SIDES:
            for address, value in zip(
                robot.arm_joint_qpos[side],
                self.config.reset.home_joint_positions,
                strict=True,
            ):
                data.qpos[address] = value
            for address in robot.finger_joint_qpos[side]:
                data.qpos[address] = half_opening
            for actuator, value in zip(
                robot.arm_actuator_ids[side],
                self.config.reset.home_joint_positions,
                strict=True,
            ):
                data.ctrl[actuator] = value
            data.ctrl[robot.gripper_actuator_ids[side]] = self._gripper_ctrl

        for obj in self.config.objects:
            address = self.indices.environment.object_free_joint_qpos[obj.name]
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


def _camera_id(model: mujoco.MjModel, camera_name: str) -> int:
    try:
        return int(model.camera(camera_name).id)
    except KeyError as error:
        raise SceneBuildError(f"expected camera {camera_name!r} is missing") from error


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
    panda_xml = assets.robot_model_path(config.robot.assembly)
    if not panda_xml.is_file():
        raise SceneBuildError(f"Panda model is missing: {panda_xml}")

    spec = mujoco.MjSpec()
    spec.option.timestep = config.physics.timestep
    spec.option.gravity = list(config.physics.gravity)
    spec.option.integrator = _INTEGRATORS[config.physics.integrator]
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.impratio = 10.0
    add_stack_environment(spec, config)
    attach_panda_robots(spec, config.mounts, panda_xml)
    _add_cameras(spec, config)
    model, data = compile_spec(spec, config.cameras)
    _check_model_shape(model, config)
    indices = PandaSceneIndices(
        robot=bind_panda_robot(model),
        environment=bind_stack_environment(model, config),
        camera_ids=MappingProxyType(
            {camera.name: _camera_id(model, camera.name) for camera in config.cameras}
        ),
    )
    scene = Scene(model, data, config, indices)
    scene.reset(0)
    return scene
