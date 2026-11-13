"""Compose the fixed-base Marvin handover scene.

Robot assembly, task environment and the generic MuJoCo compile/reset
container are composed here by explicit configuration selection.  The task
name does not choose the robot, and the robot assembly does not know the
handover stages.
"""

from __future__ import annotations

import math
import warnings
from collections.abc import Mapping
from dataclasses import dataclass
from types import MappingProxyType

import mujoco

from robot_backend_sim.config.marvin_scene_config import (
    MarvinSceneConfig,
    load_marvin_scene_config,
)
from robot_backend_sim.physics.marvin.assembly import (
    MarvinBinding,
    SIDES,
    UrdfSemantics,
    add_marvin_robot,
    bind_marvin_robot,
    read_urdf_semantics,
)
from robot_backend_sim.physics.marvin.environment import (
    HandoverBinding,
    add_handover_environment,
    base_world_point,
    bind_handover_environment,
    combined_yaw_quaternion,
)
from robot_backend_sim.physics.robot_geometry import ArmChain, RobotGeometry
from robot_backend_sim.physics.scene_container import (
    CompiledScene,
    SceneBuildError,
    compile_spec,
)

__all__ = [
    "MarvinScene",
    "MarvinSceneIndices",
    "build_marvin_scene",
    "gripper_opening",
]

_INTEGRATORS = {
    "euler": mujoco.mjtIntegrator.mjINT_EULER,
    "implicit": mujoco.mjtIntegrator.mjINT_IMPLICIT,
    "implicitfast": mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
    "rk4": mujoco.mjtIntegrator.mjINT_RK4,
}
_ASSEMBLIES = {"marvin": add_marvin_robot}
_ENVIRONMENTS = {"handover": add_handover_environment}


@dataclass(frozen=True, slots=True)
class MarvinSceneIndices:
    robot: MarvinBinding
    environment: HandoverBinding
    camera_ids: Mapping[str, int]


class MarvinScene(CompiledScene):
    """One compiled Marvin handover scene with deterministic reset."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        config: MarvinSceneConfig,
        indices: MarvinSceneIndices,
    ) -> None:
        super().__init__(model, data)
        self.config = config
        self.indices = indices

    @property
    def gripper_open_total(self) -> float:
        return self.config.robot.gripper_open_total

    @property
    def contact_object_names(self) -> tuple[str, ...]:
        return (self.config.task.block.name,)

    @property
    def robot_geometry(self) -> RobotGeometry:
        return RobotGeometry(
            chains=tuple(
                ArmChain(
                    side=side,
                    joint_names=tuple(
                        f"Arm_{prefix}{index}_Joint" for index in range(1, 8)
                    ),
                    tcp_site=f"Arm_{prefix}_tcp",
                )
                for side, prefix in SIDES
            ),
            body_prefixes=("Arm_",),
            side_prefixes={"left": ("Arm_L",), "right": ("Arm_R",)},
            finger_bodies={
                side: (
                    (f"Arm_{prefix}10_Link", f"Arm_{prefix}10_Joint"),
                    (f"Arm_{prefix}11_Link", f"Arm_{prefix}11_Joint"),
                )
                for side, prefix in SIDES
            },
        )

    @property
    def object_free_joint_qpos(self) -> Mapping[str, int]:
        return {
            self.config.task.block.name: self.indices.environment.block_free_joint_qpos
        }

    @property
    def support_body_names(self) -> tuple[str, ...]:
        return ("table", self.config.task.box.name)

    @property
    def gripper_closed_total(self) -> float:
        return self.config.robot.gripper_closed_total

    def gripper_ctrl(self, total_opening: float) -> float:
        """Map measured finger-face distance (m) to the tendon closure command."""
        opening = float(total_opening)
        if not math.isfinite(opening):
            raise ValueError("gripper opening must be finite")
        if not self.gripper_closed_total <= opening <= self.gripper_open_total:
            raise ValueError(
                f"gripper opening {opening} is outside "
                f"[{self.gripper_closed_total}, {self.gripper_open_total}]"
            )
        return self.gripper_open_total - opening

    def reset(self, seed: int) -> None:
        if isinstance(seed, bool) or not isinstance(seed, int):
            raise TypeError("seed must be an integer")
        del seed  # the first handover layout is fixed; the seed identifies a run
        model = self.model
        data = self.data
        self.reset_run_state()
        robot = self.indices.robot
        home = self.config.robot.home_joint_positions
        for public_side, _ in SIDES:
            for address, value in zip(
                robot.arm_joint_qpos[public_side], home, strict=True
            ):
                data.qpos[address] = value
            for address in robot.finger_joint_qpos[public_side]:
                data.qpos[address] = 0.0
            for actuator, value in zip(
                robot.arm_actuator_ids[public_side], home, strict=True
            ):
                data.ctrl[actuator] = value
            data.ctrl[robot.gripper_actuator_ids[public_side]] = 0.0

        block = self.config.task.block
        address = self.indices.environment.block_free_joint_qpos
        base = self.config.robot
        world = base_world_point(
            (
                block.initial_xy[0],
                block.initial_xy[1],
                self.config.task.table.top_z + block.half_extents[2],
            ),
            base.base_position,
            base.base_yaw_deg,
        )
        quaternion = combined_yaw_quaternion(base.base_yaw_deg, block.initial_yaw_deg)
        data.qpos[address : address + 7] = (*world, *quaternion)
        mujoco.mj_forward(model, data)


def gripper_opening(
    model: mujoco.MjModel, data: mujoco.MjData, robot: MarvinBinding, side: str
) -> float:
    """Minimum tip aperture projected onto the hand's closing axis.

    The two source finger bodies are laterally offset, so the aperture is the
    projection of the pad separation onto the hand's local closing axis, not
    the straight-line distance between two arbitrary surface points. Tapered
    mesh surfaces can contact an object at a wider section than this aperture.
    """
    del model
    pads = robot.pad_site_ids[side]
    first = data.site_xpos[pads["finger10"]]
    second = data.site_xpos[pads["finger11"]]
    hand = robot.hand_body_ids[side]
    axis = data.xmat[hand].reshape(3, 3)[:, 2]
    delta = (first[0] - second[0], first[1] - second[1], first[2] - second[2])
    return abs(sum(component * direction for component, direction in zip(delta, axis, strict=True)))


def _close(left: float, right: float) -> bool:
    return math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=1e-9)


def _add_cameras(spec: mujoco.MjSpec, config: MarvinSceneConfig) -> None:
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
        camera_spec.resolution = [camera.width, camera.height]


def _validate_cameras(model: mujoco.MjModel, config: MarvinSceneConfig) -> dict[str, int]:
    ids: dict[str, int] = {}
    if model.ncam != len(config.cameras):
        raise SceneBuildError(
            f"compiled model has {model.ncam} cameras, expected {len(config.cameras)}"
        )
    for camera in config.cameras:
        try:
            camera_id = int(model.camera(camera.name).id)
            parent_id = int(model.body(camera.parent).id)
        except KeyError as error:
            raise SceneBuildError(f"compiled camera {camera.name!r} is missing") from error
        ids[camera.name] = camera_id
        if int(model.cam_bodyid[camera_id]) != parent_id:
            raise SceneBuildError(f"camera {camera.name!r} is attached to another body")
        if int(model.cam_mode[camera_id]) != int(mujoco.mjtCamLight.mjCAMLIGHT_FIXED):
            raise SceneBuildError(f"camera {camera.name!r} must use a fixed local pose")
        if tuple(int(value) for value in model.cam_resolution[camera_id]) != (
            camera.width,
            camera.height,
        ):
            raise SceneBuildError(f"camera {camera.name!r} resolution differs")
        if not _close(model.cam_fovy[camera_id], camera.fovy):
            raise SceneBuildError(f"camera {camera.name!r} field of view differs")
        if any(
            not _close(a, b)
            for a, b in zip(model.cam_pos[camera_id], camera.position, strict=True)
        ):
            raise SceneBuildError(f"camera {camera.name!r} position differs")
        actual = tuple(float(value) for value in model.cam_quat[camera_id])
        expected = camera.quaternion_wxyz
        same_rotation = all(
            _close(a, b) for a, b in zip(actual, expected, strict=True)
        ) or all(_close(a, -b) for a, b in zip(actual, expected, strict=True))
        if not same_rotation:
            raise SceneBuildError(f"camera {camera.name!r} orientation differs")
    return ids


def build_marvin_scene(config: MarvinSceneConfig) -> MarvinScene:
    """Assemble, compile and bind one Marvin handover scene."""
    if config.urdf_path is None:
        raise SceneBuildError("scene config has no resolved URDF path")
    spec = mujoco.MjSpec()
    spec.option.timestep = config.physics.timestep
    spec.option.gravity = list(config.physics.gravity)
    spec.option.integrator = _INTEGRATORS[config.physics.integrator]
    spec.option.cone = (
        mujoco.mjtCone.mjCONE_ELLIPTIC
        if config.physics.cone == "elliptic"
        else mujoco.mjtCone.mjCONE_PYRAMIDAL
    )
    spec.option.impratio = config.physics.impratio
    spec.option.noslip_iterations = config.physics.noslip_iterations
    assembly = _ASSEMBLIES.get(config.robot.assembly)
    if assembly is None:
        raise SceneBuildError(f"unsupported robot assembly {config.robot.assembly!r}")
    environment = _ENVIRONMENTS.get(config.task.environment)
    if environment is None:
        raise SceneBuildError(
            f"unsupported task environment {config.task.environment!r}"
        )
    assembly(spec, config.robot, config.urdf_path)
    environment(
        spec,
        config.task,
        config.physics.object_friction,
        config.robot.base_position,
        config.robot.base_yaw_deg,
    )
    spec.geom(f"{config.task.block.name}_geom").condim = config.physics.object_contact_dim
    _add_cameras(spec, config)
    # The source names must stay unprefixed; the model is compiled in memory
    # and never saved, so MuJoCo's empty-prefix save warning does not apply.
    with warnings.catch_warnings():
        warnings.filterwarnings(
            "ignore",
            message="Attaching a spec with an empty prefix.*",
            category=UserWarning,
        )
        model, data = compile_spec(spec, config.cameras, modelname="marvin_handover")

    semantics: UrdfSemantics = read_urdf_semantics(config.urdf_path)
    robot_binding = bind_marvin_robot(model, config.robot, semantics)
    environment_binding = bind_handover_environment(model, config.task)
    indices = MarvinSceneIndices(
        robot=robot_binding,
        environment=environment_binding,
        camera_ids=MappingProxyType(_validate_cameras(model, config)),
    )
    scene = MarvinScene(model, data, config, indices)
    scene.reset(0)
    return scene


def load_and_build(path: str) -> MarvinScene:
    return build_marvin_scene(load_marvin_scene_config(path))
