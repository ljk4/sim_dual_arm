"""Shared robot-level scene assembly, geometry, camera and reset helpers for Marvin."""

from __future__ import annotations

import math
import warnings
from typing import Annotated, Any, Literal

import mujoco
from pydantic import BeforeValidator, Field, StrictInt, model_validator
from robot_contracts.validation import (
    PositiveFloat,
    StrictFloat,
    StrictMappingModel,
    as_tuple,
    contract_error,
)

from robot_backend_sim.common.config.camera_config import Unit, Vec3
from robot_backend_sim.common.physics.robot_geometry import ArmChain, RobotGeometry
from robot_backend_sim.common.physics.scene_container import (
    CompiledScene,
    SceneBuildError,
    compile_spec,
)
from robot_backend_sim.marvin_sim.assembly import SIDES, add_marvin_robot
from robot_backend_sim.marvin_sim.robot_config import MarvinRobotConfig

__all__ = [
    "CAMERA_PARENTS",
    "CameraMountConfig",
    "MARVIN_INTEGRATORS",
    "MarvinSceneBase",
    "PhysicsConfig",
    "Rgba",
    "TableConfig",
    "Vec2",
    "add_marvin_camera_mount",
    "add_marvin_cameras",
    "add_marvin_lights",
    "add_marvin_table",
    "base_world_point",
    "combined_yaw_quaternion",
    "compile_marvin_spec",
    "create_marvin_spec",
    "marvin_robot_geometry",
    "validate_marvin_cameras",
    "yaw_quaternion",
]

CAMERA_PARENTS = {
    "global": "camera_mount",
    "left_wrist": "Arm_L9_Link",
    "right_wrist": "Arm_R9_Link",
}

MARVIN_INTEGRATORS = {
    "euler": mujoco.mjtIntegrator.mjINT_EULER,
    "implicit": mujoco.mjtIntegrator.mjINT_IMPLICIT,
    "implicitfast": mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
    "rk4": mujoco.mjtIntegrator.mjINT_RK4,
}

Vec2 = Annotated[tuple[StrictFloat, StrictFloat], BeforeValidator(as_tuple)]
Rgba = Annotated[tuple[Unit, Unit, Unit, Unit], BeforeValidator(as_tuple)]


class PhysicsConfig(StrictMappingModel):
    timestep: Annotated[PositiveFloat, Field(ge=1e-5, le=0.01)]
    gravity: Vec3
    integrator: Literal["euler", "implicit", "implicitfast", "rk4"]
    cone: Literal["pyramidal", "elliptic"]
    impratio: PositiveFloat
    noslip_iterations: Annotated[StrictInt, Field(ge=0)]
    object_contact_dim: Literal[3, 4, 6]
    object_friction: Vec3

    @model_validator(mode="after")
    def _physics(self) -> PhysicsConfig:
        if self.gravity[2] >= 0.0:
            raise contract_error("gravity[2]", "must point downward")
        if self.object_friction[0] <= 0.0:
            raise contract_error("object_friction[0]", "sliding friction must be positive")
        for index, value in enumerate(self.object_friction):
            if value < 0.0:
                raise contract_error(f"object_friction[{index}]", "must be non-negative")
        return self


class TableConfig(StrictMappingModel):
    center_xy: Vec2
    size_xy: Vec2
    top_z: Annotated[PositiveFloat, Field(ge=0.01, le=3.0)]
    thickness: Annotated[PositiveFloat, Field(ge=0.001, le=0.5)]
    rgba: Rgba

    @model_validator(mode="after")
    def _table(self) -> TableConfig:
        for index, value in enumerate(self.size_xy):
            if not 0.05 <= value <= 5.0:
                raise contract_error(f"size_xy[{index}]", "must be between 0.05 and 5.0")
        if self.thickness > self.top_z:
            raise contract_error("thickness", "must not exceed top_z")
        return self


class CameraMountConfig(StrictMappingModel):
    position: Vec3


def yaw_quaternion(yaw_deg: float) -> tuple[float, float, float, float]:
    half = math.radians(yaw_deg) / 2.0
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def base_world_point(
    local: tuple[float, float, float],
    base_position: tuple[float, float, float],
    base_yaw_deg: float,
) -> tuple[float, float, float]:
    """Transform one task-frame point into world using the robot base pose."""
    angle = math.radians(base_yaw_deg)
    cosine, sine = math.cos(angle), math.sin(angle)
    x, y, z = local
    return (
        base_position[0] + cosine * x - sine * y,
        base_position[1] + sine * x + cosine * y,
        base_position[2] + z,
    )


def combined_yaw_quaternion(
    base_yaw_deg: float, local_yaw_deg: float
) -> tuple[float, float, float, float]:
    return yaw_quaternion(base_yaw_deg + local_yaw_deg)


def add_marvin_lights(spec: mujoco.MjSpec) -> None:
    spec.worldbody.add_light(
        name="key_light",
        pos=[0.55, 0.35, 2.4],
        dir=[-0.15, -0.25, -1.0],
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
        diffuse=[0.8, 0.8, 0.8],
        specular=[0.2, 0.2, 0.2],
        castshadow=True,
    )
    spec.worldbody.add_light(
        name="fill_light",
        pos=[-0.20, 0.90, 1.80],
        dir=[0.30, -0.35, -1.0],
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
        diffuse=[0.35, 0.35, 0.35],
    )


def add_marvin_table(
    spec: mujoco.MjSpec,
    table: TableConfig,
    friction: tuple[float, float, float],
    base_position: tuple[float, float, float],
    base_yaw_deg: float,
) -> None:
    table_center = base_world_point(
        (table.center_xy[0], table.center_xy[1], table.top_z - table.thickness / 2.0),
        base_position,
        base_yaw_deg,
    )
    table_body = spec.worldbody.add_body(
        name="table",
        pos=list(table_center),
        quat=list(yaw_quaternion(base_yaw_deg)),
    )
    table_body.add_geom(
        name="table_top",
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[table.size_xy[0] / 2.0, table.size_xy[1] / 2.0, table.thickness / 2.0],
        rgba=list(table.rgba),
        contype=1,
        conaffinity=1,
        friction=list(friction),
    )


def add_marvin_camera_mount(
    spec: mujoco.MjSpec,
    camera_mount: CameraMountConfig,
    base_position: tuple[float, float, float],
    base_yaw_deg: float,
) -> None:
    spec.worldbody.add_body(
        name="camera_mount",
        pos=list(
            base_world_point(tuple(camera_mount.position), base_position, base_yaw_deg)
        ),
        quat=list(yaw_quaternion(base_yaw_deg)),
    )


def add_marvin_cameras(spec: mujoco.MjSpec, cameras: tuple[Any, ...]) -> None:
    for camera in cameras:
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


def _close(left: float, right: float) -> bool:
    return math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=1e-9)


def validate_marvin_cameras(model: mujoco.MjModel, cameras: tuple[Any, ...]) -> dict[str, int]:
    ids: dict[str, int] = {}
    if model.ncam != len(cameras):
        raise SceneBuildError(
            f"compiled model has {model.ncam} cameras, expected {len(cameras)}"
        )
    for camera in cameras:
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


def marvin_robot_geometry() -> RobotGeometry:
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


def create_marvin_spec(
    physics: PhysicsConfig, robot: MarvinRobotConfig, urdf_path: Any
) -> mujoco.MjSpec:
    spec = mujoco.MjSpec()
    spec.option.timestep = physics.timestep
    spec.option.gravity = list(physics.gravity)
    spec.option.integrator = MARVIN_INTEGRATORS[physics.integrator]
    spec.option.cone = (
        mujoco.mjtCone.mjCONE_ELLIPTIC
        if physics.cone == "elliptic"
        else mujoco.mjtCone.mjCONE_PYRAMIDAL
    )
    spec.option.impratio = physics.impratio
    spec.option.noslip_iterations = physics.noslip_iterations
    add_marvin_robot(spec, robot, urdf_path)
    return spec


def compile_marvin_spec(
    spec: mujoco.MjSpec, cameras: tuple[Any, ...], *, modelname: str
) -> tuple[mujoco.MjModel, mujoco.MjData]:
    # The source names must stay unprefixed; the model is compiled in memory
    # and never saved, so MuJoCo's empty-prefix save warning does not apply.
    with warnings.catch_warnings():
        warnings.filterwarnings(
            "ignore",
            message="Attaching a spec with an empty prefix.*",
            category=UserWarning,
        )
        return compile_spec(spec, cameras, modelname=modelname)


class MarvinSceneBase(CompiledScene):
    """Shared compiled Marvin scene base for task-specific subclasses."""

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        config: Any,
        indices: Any,
    ) -> None:
        super().__init__(model, data)
        self.config = config
        self.indices = indices

    @property
    def gripper_open_total(self) -> float:
        return self.config.robot.gripper_open_total

    @property
    def gripper_closed_total(self) -> float:
        return self.config.robot.gripper_closed_total

    @property
    def robot_geometry(self) -> RobotGeometry:
        return marvin_robot_geometry()

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

    def _reset_robot_joints(self) -> None:
        data = self.data
        robot = self.indices.robot
        for public_side, _ in SIDES:
            home = self.config.robot.home_joint_positions.for_side(public_side)
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
