"""Shared robot-level scene assembly, geometry, camera, DR and reset helpers for Marvin."""

from __future__ import annotations

import math
import warnings
from typing import Annotated, Any, Literal

import mujoco
import numpy as np
from pydantic import BeforeValidator, Field, StrictBool, StrictInt, model_validator
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
from robot_backend_sim.marvin.assembly import SIDES, add_marvin_robot
from robot_backend_sim.marvin.robot_config import MarvinRobotConfig

__all__ = [
    "CAMERA_PARENTS",
    "CameraMountConfig",
    "DomainDynamicsConfig",
    "DomainGeometryConfig",
    "DomainRandomizationConfig",
    "DomainVisualConfig",
    "MARVIN_INTEGRATORS",
    "MarvinSceneBase",
    "PhysicsConfig",
    "Rgba",
    "ScaleRange",
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
    "cam_high": "camera_mount",
    "cam_left_wrist": "Arm_L_gripper_base",
    "cam_right_wrist": "Arm_R_gripper_base",
}

MARVIN_INTEGRATORS = {
    "euler": mujoco.mjtIntegrator.mjINT_EULER,
    "implicit": mujoco.mjtIntegrator.mjINT_IMPLICIT,
    "implicitfast": mujoco.mjtIntegrator.mjINT_IMPLICITFAST,
    "rk4": mujoco.mjtIntegrator.mjINT_RK4,
}

Vec2 = Annotated[tuple[StrictFloat, StrictFloat], BeforeValidator(as_tuple)]
Rgba = Annotated[tuple[Unit, Unit, Unit, Unit], BeforeValidator(as_tuple)]
ScaleRange = Annotated[tuple[PositiveFloat, PositiveFloat], BeforeValidator(as_tuple)]


class DomainGeometryConfig(StrictMappingModel):
    camera_pos_jitter_m: Annotated[StrictFloat, Field(ge=0.0, le=0.02)] = 0.0
    camera_rot_jitter_deg: Annotated[StrictFloat, Field(ge=0.0, le=5.0)] = 0.0
    table_top_z_jitter_m: Annotated[StrictFloat, Field(ge=0.0, le=0.02)] = 0.0


class DomainDynamicsConfig(StrictMappingModel):
    friction_scale_range: ScaleRange = (1.0, 1.0)
    object_mass_scale_range: ScaleRange = (1.0, 1.0)
    joint_damping_scale_range: ScaleRange = (1.0, 1.0)

    @model_validator(mode="after")
    def _ranges(self) -> DomainDynamicsConfig:
        for name in (
            "friction_scale_range",
            "object_mass_scale_range",
            "joint_damping_scale_range",
        ):
            low, high = getattr(self, name)
            if low > high:
                raise contract_error(name, "lower bound must not exceed upper bound")
        return self


class DomainVisualConfig(StrictMappingModel):
    light_diffuse_jitter: Annotated[StrictFloat, Field(ge=0.0, le=0.5)] = 0.0
    object_rgba_jitter: Annotated[StrictFloat, Field(ge=0.0, le=0.3)] = 0.0


class DomainRandomizationConfig(StrictMappingModel):
    """Three-layer (geometry/dynamics/visual) seed-deterministic DR configuration."""

    enabled: StrictBool = False
    geometry: DomainGeometryConfig = Field(default_factory=DomainGeometryConfig)
    dynamics: DomainDynamicsConfig = Field(default_factory=DomainDynamicsConfig)
    visual: DomainVisualConfig = Field(default_factory=DomainVisualConfig)


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


def _quat_mul(
    q1: np.ndarray, q2: np.ndarray
) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


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
                (f"Arm_{prefix}_finger1", f"Arm_{prefix}_finger1_Joint"),
                (f"Arm_{prefix}_finger2", f"Arm_{prefix}_finger2_Joint"),
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
        self._reset_epoch = 0
        self._nominal_cam_pos = model.cam_pos.copy()
        self._nominal_cam_quat = model.cam_quat.copy()
        self._nominal_body_pos = model.body_pos.copy()
        self._nominal_body_mass = model.body_mass.copy()
        self._nominal_geom_friction = model.geom_friction.copy()
        self._nominal_geom_rgba = model.geom_rgba.copy()
        self._nominal_dof_damping = model.dof_damping.copy()
        self._nominal_light_diffuse = model.light_diffuse.copy()

    @property
    def reset_epoch(self) -> int:
        return self._reset_epoch

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
        """Map public stroke opening (m) to tendon closure command."""
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
            for address in robot.axial_joint_qpos[public_side]:
                data.qpos[address] = 0.0
            for actuator, value in zip(
                robot.arm_actuator_ids[public_side], home, strict=True
            ):
                data.ctrl[actuator] = value
            data.ctrl[robot.gripper_actuator_ids[public_side]] = 0.0
        self._reset_epoch += 1

    def _restore_nominal_model(self) -> None:
        model = self.model
        model.cam_pos[:] = self._nominal_cam_pos
        model.cam_quat[:] = self._nominal_cam_quat
        model.body_pos[:] = self._nominal_body_pos
        model.body_mass[:] = self._nominal_body_mass
        model.geom_friction[:] = self._nominal_geom_friction
        model.geom_rgba[:] = self._nominal_geom_rgba
        model.dof_damping[:] = self._nominal_dof_damping
        model.light_diffuse[:] = self._nominal_light_diffuse

    def _apply_domain_randomization(self, seed: int) -> None:
        """Restore nominal model arrays and apply deterministic 3-layer DR when enabled."""
        self._restore_nominal_model()
        dr: DomainRandomizationConfig | None = getattr(self.config, "domain_randomization", None)
        if dr is None or not dr.enabled:
            return

        rng = np.random.Generator(np.random.PCG64(int(seed)))
        model = self.model
        data = self.data

        # 1. Geometry layer: camera extrinsics & table height (shift free objects by same dz).
        pos_j = float(dr.geometry.camera_pos_jitter_m)
        rot_j = math.radians(float(dr.geometry.camera_rot_jitter_deg))
        for cam_id in range(int(model.ncam)):
            if pos_j > 0.0:
                model.cam_pos[cam_id] += rng.uniform(-pos_j, pos_j, size=3)
            if rot_j > 0.0:
                axis = rng.normal(size=3)
                norm = float(np.linalg.norm(axis))
                if norm > 1e-9:
                    axis /= norm
                    angle = float(rng.uniform(-rot_j, rot_j))
                    delta_q = np.array(
                        [math.cos(angle / 2.0), *(math.sin(angle / 2.0) * axis)],
                        dtype=np.float64,
                    )
                    updated = _quat_mul(model.cam_quat[cam_id], delta_q)
                    model.cam_quat[cam_id] = updated / float(np.linalg.norm(updated))

        table_j = float(dr.geometry.table_top_z_jitter_m)
        if table_j > 0.0:
            dz = float(rng.uniform(-table_j, table_j))
            try:
                table_id = int(model.body("table").id)
                model.body_pos[table_id, 2] += dz
                for qpos_addr in self.object_free_joint_qpos.values():
                    data.qpos[int(qpos_addr) + 2] += dz
            except KeyError:
                pass

        # 2. Dynamics layer: friction, object masses, and arm joint damping.
        fric_lo, fric_hi = dr.dynamics.friction_scale_range
        if fric_lo != 1.0 or fric_hi != 1.0:
            scale = float(rng.uniform(fric_lo, fric_hi))
            model.geom_friction[:, 0] *= scale

        mass_lo, mass_hi = dr.dynamics.object_mass_scale_range
        if mass_lo != 1.0 or mass_hi != 1.0:
            for obj_name in self.contact_object_names:
                try:
                    bid = int(model.body(obj_name).id)
                    model.body_mass[bid] *= float(rng.uniform(mass_lo, mass_hi))
                except KeyError:
                    pass
            mujoco.mj_setConst(model, data)

        damp_lo, damp_hi = dr.dynamics.joint_damping_scale_range
        if damp_lo != 1.0 or damp_hi != 1.0:
            robot = self.indices.robot
            for side, _ in SIDES:
                for dof_addr in robot.arm_joint_dof[side]:
                    model.dof_damping[dof_addr] *= float(rng.uniform(damp_lo, damp_hi))

        # 3. Visual layer: directional light intensity and free object albedo.
        light_j = float(dr.visual.light_diffuse_jitter)
        if light_j > 0.0:
            for lid in range(int(model.nlight)):
                delta = rng.uniform(-light_j, light_j, size=3)
                model.light_diffuse[lid] = np.clip(
                    model.light_diffuse[lid] + delta, 0.05, 1.0
                )

        rgba_j = float(dr.visual.object_rgba_jitter)
        if rgba_j > 0.0:
            obj_bids = {int(model.body(name).id) for name in self.contact_object_names}
            for gid in range(int(model.ngeom)):
                if int(model.geom_bodyid[gid]) in obj_bids:
                    delta = rng.uniform(-rgba_j, rgba_j, size=3)
                    model.geom_rgba[gid, :3] = np.clip(
                        model.geom_rgba[gid, :3] + delta, 0.02, 0.98
                    )
