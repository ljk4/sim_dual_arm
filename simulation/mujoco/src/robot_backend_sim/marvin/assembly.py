"""Fixed-base Marvin dual-arm assembly calibrated to the TianJi hardware rig.

The 7-DOF arms and pedestal come from the source URDF unchanged.  The wrist
and gripper assembly attaches an L-bracket plate + ChangingTek CTAG2F90C
90 mm parallel 4-bar linkage + 3D-printed adapters + recessed DeepTouch
visuotactile housings + compliant silicone pads cleanly in-memory, with a
quartic joint equality modelling the exact 55.00 mm 4-bar axial extension.
"""

from __future__ import annotations

import math
import struct
import sys
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType

import mujoco

from robot_backend_sim.common.physics.scene_container import SceneBuildError
from robot_backend_sim.marvin.robot_config import MarvinRobotConfig

__all__ = [
    "ARM_JOINT_COUNT",
    "FOURBAR_POLYCOEF",
    "MarvinBinding",
    "UrdfSemantics",
    "add_marvin_robot",
    "bind_marvin_robot",
    "fourbar_axial_extension_m",
    "gripper_opening",
    "pad_aperture_m",
    "read_urdf_semantics",
]

SIDES = (("left", "L"), ("right", "R"))
ARM_JOINT_COUNT = 7
FINGER_TRAVEL = 0.045
FINGER_ARMATURE = 0.001
# Quartic polynomial fit (max error 0.028 mm over u in [0, 0.045] m) to the
# exact CTAG2F90C 55.00 mm 4-bar axial extension:
#   dy(u) = sqrt(0.055^2 - (0.04036 - u)^2) - 0.03737
FOURBAR_POLYCOEF = (0.0, 1.060315, -23.743821, 298.892686, -2306.472156)
COUPLING_SOLREF = (0.005, 1.0)
COUPLING_SOLIMP = (0.95, 0.99, 0.001, 0.5, 2.0)
# Soft silicone gel pad contact parameters matching DeepTouch visuotactile gel.
PAD_CONTACT_SOLREF = (0.006, 1.0)
PAD_CONTACT_SOLIMP = (0.90, 0.99, 0.0015, 0.5, 2.0)
PAD_CONTACT_FRICTION = (1.2, 0.02, 0.0002)
# Arm_*5 / Arm_*7 source convex hulls overlap structurally around Arm_*6 wrist;
# same-hand finger1 / finger2 are excluded so empty close to 0.0 m settles in
# POSITION mode even when pad_inward_offset_m > 0.
SELF_EXCLUSION_PAIRS = ((5, 7),)
_MUJOCO_STL_FACE_LIMIT = 200_000
TCP_Y_OFFSET_M = -0.236
PAD_INNER_Z_OPEN_M = 0.045


@dataclass(frozen=True, slots=True)
class UrdfJoint:
    kind: str
    lower: float
    upper: float
    effort: float
    velocity: float


@dataclass(frozen=True, slots=True)
class UrdfSemantics:
    joints: Mapping[str, UrdfJoint]
    link_mass: Mapping[str, float]
    link_inertia: Mapping[str, tuple[tuple[float, float, float], ...]]
    mesh_files: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class MarvinBinding:
    """Named MuJoCo addresses and semantic handles for the 16-value Marvin contract."""

    arm_joint_qpos: Mapping[str, tuple[int, ...]]
    arm_joint_dof: Mapping[str, tuple[int, ...]]
    finger_joint_qpos: Mapping[str, tuple[int, int]]
    finger_joint_dof: Mapping[str, tuple[int, int]]
    axial_joint_qpos: Mapping[str, tuple[int, int]]
    axial_joint_dof: Mapping[str, tuple[int, int]]
    arm_actuator_ids: Mapping[str, tuple[int, ...]]
    gripper_actuator_ids: Mapping[str, int]
    tcp_site_ids: Mapping[str, int]
    pad_site_ids: Mapping[str, Mapping[str, int]]
    finger_pad_body_ids: Mapping[str, tuple[int, int]]
    arm_collision_body_ids: Mapping[str, tuple[int, ...]]
    hand_body_ids: Mapping[str, int]
    base_body_id: int
    finger_travel: float
    pad_inward_offset_m: float = 0.0


def fourbar_axial_extension_m(finger_slide_m: float) -> float:
    """Evaluate the 4-bar axial extension (m) from lateral closing slide ``u`` (m)."""
    u = float(finger_slide_m)
    c0, c1, c2, c3, c4 = FOURBAR_POLYCOEF
    return (((c4 * u + c3) * u + c2) * u + c1) * u + c0


def _yaw_quaternion(yaw_deg: float) -> tuple[float, float, float, float]:
    half = math.radians(yaw_deg) / 2.0
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def read_urdf_semantics(path: Path) -> UrdfSemantics:
    """Read joint limits, link masses/inertias and mesh references from URDF."""
    root = ET.parse(path).getroot()
    joints: dict[str, UrdfJoint] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        kind = joint.get("type")
        if name is None or kind is None:
            raise SceneBuildError("URDF joint without name or type")
        limit = joint.find("limit")
        if kind in {"revolute", "prismatic"}:
            if limit is None:
                raise SceneBuildError(f"URDF joint {name!r} has no limit")
            joints[name] = UrdfJoint(
                kind=kind,
                lower=float(limit.get("lower", "0")),
                upper=float(limit.get("upper", "0")),
                effort=float(limit.get("effort", "0")),
                velocity=float(limit.get("velocity", "0")),
            )
    link_mass: dict[str, float] = {}
    link_inertia: dict[str, tuple[tuple[float, float, float], ...]] = {}
    for link in root.findall("link"):
        name = link.get("name")
        inertial = link.find("inertial")
        if name is None or inertial is None:
            continue
        mass = inertial.find("mass")
        inertia = inertial.find("inertia")
        if mass is None:
            raise SceneBuildError(f"URDF link {name!r} has no mass")
        link_mass[name] = float(mass.get("value", "0"))
        if inertia is not None:
            ixx = float(inertia.get("ixx", "0"))
            iyy = float(inertia.get("iyy", "0"))
            izz = float(inertia.get("izz", "0"))
            ixy = float(inertia.get("ixy", "0"))
            ixz = float(inertia.get("ixz", "0"))
            iyz = float(inertia.get("iyz", "0"))
            link_inertia[name] = (
                (ixx, ixy, ixz),
                (ixy, iyy, iyz),
                (ixz, iyz, izz),
            )
    mesh_files = tuple(
        sorted(
            {
                mesh.get("filename")
                for mesh in root.iter("mesh")
                if mesh.get("filename") is not None
            }
        )
    )
    return UrdfSemantics(
        joints=MappingProxyType(joints),
        link_mass=MappingProxyType(link_mass),
        link_inertia=MappingProxyType(link_inertia),
        mesh_files=mesh_files,
    )


def _read_binary_stl(path: Path) -> list[list[tuple[float, float, float]]]:
    try:
        data = path.read_bytes()
    except OSError as error:
        raise SceneBuildError(f"cannot read mesh {path}: {error}") from error
    if len(data) < 84:
        raise SceneBuildError(f"mesh {path} is not a binary STL")
    count = struct.unpack_from("<I", data, 80)[0]
    expected = 84 + count * 50
    if expected != len(data):
        raise SceneBuildError(
            f"mesh {path} is not a binary STL ({len(data)} bytes for {count} faces)"
        )
    triangles = []
    for index in range(count):
        offset = 84 + index * 50 + 12
        values = struct.unpack_from("<9f", data, offset)
        triangles.append(
            [
                (values[0], values[1], values[2]),
                (values[3], values[4], values[5]),
                (values[6], values[7], values[8]),
            ]
        )
    return triangles


def _mesh_dir(urdf_path: Path) -> Path:
    return urdf_path.parent.parent / "meshes"


def _prepare_meshes(spec: mujoco.MjSpec, urdf_path: Path) -> Path:
    mesh_dir = _mesh_dir(urdf_path)
    spec.meshdir = str(mesh_dir)
    for mesh in spec.meshes:
        name = Path(mesh.file).name
        if not (mesh_dir / name).is_file():
            raise SceneBuildError(f"source mesh is missing: {mesh_dir / name}")
        mesh.file = name
    return mesh_dir


def _replace_oversized_stand(spec: mujoco.MjSpec, mesh_dir: Path) -> None:
    """Split the stand mesh in memory because the file decoder caps at 200k faces."""
    mesh = spec.mesh("ZJ_Robot_link")
    triangles = _read_binary_stl(mesh_dir / "ZJ_Robot_link.STL")
    if len(triangles) <= _MUJOCO_STL_FACE_LIMIT:
        return
    half = (len(triangles) + 1) // 2
    parts = (triangles[:half], triangles[half:])
    body = spec.body("ZJ_Robot_link")
    geoms = list(body.geoms)
    if len(geoms) != 1:
        raise SceneBuildError(
            f"expected one stand mesh geom, found {len(geoms)}"
        )
    mesh.name = "ZJ_Robot_link_0"
    _set_user_mesh(mesh, parts[0])
    second = spec.add_mesh(name="ZJ_Robot_link_1")
    _set_user_mesh(second, parts[1])
    geoms[0].meshname = "ZJ_Robot_link_0"
    geoms[0].contype = 0
    geoms[0].conaffinity = 0
    second_geom = body.add_geom(
        name="ZJ_Robot_link_geom_1", type=mujoco.mjtGeom.mjGEOM_MESH, meshname=second.name
    )
    second_geom.contype = 0
    second_geom.conaffinity = 0


def _set_user_mesh(mesh: mujoco.MjsMesh, triangles: list[list[tuple[float, float, float]]]) -> None:
    vertices = [component for triangle in triangles for vertex in triangle for component in vertex]
    mesh.uservert = vertices
    mesh.userface = list(range(len(vertices) // 3))
    mesh.file = ""


def _add_gripper_assembly(
    spec: mujoco.MjSpec, config: MarvinRobotConfig, mesh_dir: Path
) -> None:
    """Populate analytic CTAG2F90C 4-bar, adapter, DeepTouch shell, and silicone pad geoms."""
    inward = float(config.pad_inward_offset_m)
    black = [0.12, 0.12, 0.12, 1.0]
    dark_grey = [0.24, 0.25, 0.27, 1.0]
    gel_black = [0.08, 0.08, 0.08, 1.0]
    rot_y_cyl = [math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0]
    optional_stl = mesh_dir / "gripper" / "ctag2f90c_housing.stl"
    if optional_stl.is_file():
        spec.add_mesh(name="ctag2f90c_housing_vis", file="gripper/ctag2f90c_housing.stl")

    for _, prefix in SIDES:
        base_body = spec.body(f"Arm_{prefix}_gripper_base")
        base_body.add_geom(
            name=f"Arm_{prefix}_cam_plate",
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            pos=[0.0, -0.003, 0.0],
            quat=rot_y_cyl,
            size=[0.038, 0.003, 0.0],
            rgba=black,
            contype=1,
            conaffinity=1,
        )
        base_body.add_geom(
            name=f"Arm_{prefix}_90c_collar",
            type=mujoco.mjtGeom.mjGEOM_CYLINDER,
            pos=[0.0, -0.016, 0.0],
            quat=rot_y_cyl,
            size=[0.0375, 0.010, 0.0],
            rgba=black,
            contype=1,
            conaffinity=1,
        )
        base_body.add_geom(
            name=f"Arm_{prefix}_90c_housing",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=[0.0, -0.0795, 0.0],
            size=[0.0375, 0.0535, 0.0375],
            rgba=black,
            contype=1,
            conaffinity=1,
        )
        if optional_stl.is_file():
            vis_geom = base_body.add_geom(
                name=f"Arm_{prefix}_90c_housing_vis",
                type=mujoco.mjtGeom.mjGEOM_MESH,
                meshname="ctag2f90c_housing_vis",
                rgba=black,
            )
            vis_geom.contype = 0
            vis_geom.conaffinity = 0

        for idx in (1, 2):
            sign = 1.0 if idx == 1 else -1.0
            finger_body = spec.body(f"Arm_{prefix}_finger{idx}")
            axial_joint = finger_body.add_joint(
                name=f"Arm_{prefix}_axial{idx}_Joint",
                type=mujoco.mjtJoint.mjJNT_SLIDE,
                pos=[0.0, 0.0, 0.0],
                axis=[0.0, -1.0, 0.0],
                limited=True,
                range=[-0.002, 0.020],
            )
            axial_joint.damping[:] = config.finger_damping
            axial_joint.armature = FINGER_ARMATURE

            eq = spec.add_equality(
                name=f"Arm_{prefix}_axial{idx}_arc",
                type=mujoco.mjtEq.mjEQ_JOINT,
                name1=axial_joint.name,
                name2=f"Arm_{prefix}_finger{idx}_Joint",
                data=[*FOURBAR_POLYCOEF, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            )
            eq.solref = list(COUPLING_SOLREF)
            eq.solimp = list(COUPLING_SOLIMP)

            vis_link = finger_body.add_geom(
                name=f"Arm_{prefix}_linkage{idx}_vis",
                type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=[0.0, -0.155, sign * 0.032],
                size=[0.010, 0.022, 0.006],
                rgba=black,
            )
            vis_link.contype = 0
            vis_link.conaffinity = 0

            adapter_rgba = [0.18, 0.35, 0.78, 1.0] if idx == 1 else [0.90, 0.90, 0.90, 1.0]
            finger_body.add_geom(
                name=f"Arm_{prefix}_adapter{idx}",
                type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=[0.0, -0.185, sign * (0.0525 - inward)],
                size=[0.011, 0.013, 0.006],
                rgba=adapter_rgba,
                contype=1,
                conaffinity=1,
            )
            finger_body.add_geom(
                name=f"Arm_{prefix}_tactile_shell{idx}",
                type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=[0.0, -0.217, sign * (0.0555 - inward)],
                size=[0.011, 0.019, 0.008],
                rgba=dark_grey,
                contype=1,
                conaffinity=1,
            )
            pad_geom = finger_body.add_geom(
                name=f"Arm_{prefix}_finger{idx}_pad",
                type=mujoco.mjtGeom.mjGEOM_BOX,
                pos=[0.0, -0.223, sign * (0.047 - inward)],
                size=[0.012, 0.012, 0.002],
                rgba=gel_black,
                contype=1,
                conaffinity=1,
            )
            pad_geom.condim = 4
            pad_geom.friction = list(PAD_CONTACT_FRICTION)
            pad_geom.solref = list(PAD_CONTACT_SOLREF)
            pad_geom.solimp = list(PAD_CONTACT_SOLIMP)


def _configure_finger_couplings(spec: mujoco.MjSpec) -> None:
    """Name and tune the two URDF-imported lateral finger mimic equalities."""
    couplings: dict[str, mujoco.MjsEquality] = {}
    for equality in spec.equalities:
        if equality.type != mujoco.mjtEq.mjEQ_JOINT:
            raise SceneBuildError(
                f"source equality {equality.name1!r}/{equality.name2!r} is not a joint equality"
            )
        pair = frozenset((equality.name1, equality.name2))
        matched = None
        for _, prefix in SIDES:
            expected = frozenset(
                (f"Arm_{prefix}_finger1_Joint", f"Arm_{prefix}_finger2_Joint")
            )
            if pair == expected:
                matched = prefix
                break
        if matched is None:
            raise SceneBuildError(
                f"source equality {sorted(pair)} is not a finger coupling"
            )
        if matched in couplings:
            raise SceneBuildError(
                f"source URDF declares more than one coupling for the {matched} gripper"
            )
        data = [float(value) for value in equality.data]
        if abs(data[0]) > 1e-12 or abs(data[1] - 1.0) > 1e-12 or any(
            abs(value) > 1e-12 for value in data[2:5]
        ):
            raise SceneBuildError(
                f"source mimic for {matched!r} is not multiplier 1 / offset 0"
            )
        equality.name = f"Arm_{matched}_finger_coupling"
        equality.solref = list(COUPLING_SOLREF)
        equality.solimp = list(COUPLING_SOLIMP)
        couplings[matched] = equality
    if len(couplings) != len(SIDES):
        raise SceneBuildError(
            f"source URDF must declare one finger coupling per arm, found {len(couplings)}"
        )


def _add_actuators(
    spec: mujoco.MjSpec, config: MarvinRobotConfig, semantics: UrdfSemantics
) -> None:
    for public_side, prefix in SIDES:
        home = config.home_joint_positions.for_side(public_side)
        for index in range(1, ARM_JOINT_COUNT + 1):
            name = f"Arm_{prefix}{index}_Joint"
            joint = spec.joint(name)
            source = semantics.joints[name]
            if not source.lower <= home[index - 1] <= source.upper:
                raise SceneBuildError(f"home position for {name!r} is outside the source limits")
            joint.damping[:] = config.joint_damping
            joint.armature = config.joint_armature
            actuator = spec.add_actuator(
                name=f"Arm_{prefix}{index}_Actuator",
                trntype=mujoco.mjtTrn.mjTRN_JOINT,
                target=name,
            )
            actuator.set_to_position(kp=config.arm_kp[index - 1], kv=config.arm_kv[index - 1])
            actuator.ctrllimited = True
            actuator.ctrlrange = [source.lower, source.upper]
            actuator.forcelimited = True
            actuator.forcerange = [-source.effort, source.effort]
        for idx in (1, 2):
            joint = spec.joint(f"Arm_{prefix}_finger{idx}_Joint")
            joint.damping[:] = config.finger_damping
            joint.armature = FINGER_ARMATURE
        tendon = spec.add_tendon(name=f"Arm_{prefix}_gripper")
        tendon.wrap_joint(f"Arm_{prefix}_finger1_Joint", 1.0)
        tendon.wrap_joint(f"Arm_{prefix}_finger2_Joint", 1.0)
        actuator = spec.add_actuator(
            name=f"Arm_{prefix}_gripper_actuator",
            trntype=mujoco.mjtTrn.mjTRN_TENDON,
            target=tendon.name,
        )
        actuator.set_to_position(kp=config.gripper_kp, kv=config.gripper_kv)
        actuator.ctrllimited = True
        actuator.ctrlrange = [0.0, 2.0 * FINGER_TRAVEL]
        actuator.forcelimited = True
        actuator.forcerange = [-config.gripper_force, config.gripper_force]


def _add_sites(spec: mujoco.MjSpec, config: MarvinRobotConfig) -> None:
    inward = float(config.pad_inward_offset_m)
    for _, prefix in SIDES:
        spec.body(f"Arm_{prefix}_gripper_base").add_site(
            name=f"Arm_{prefix}_tcp",
            pos=[0.0, TCP_Y_OFFSET_M, 0.0],
            size=[0.006, 0.0, 0.0],
        )
        for idx in (1, 2):
            sign = 1.0 if idx == 1 else -1.0
            spec.body(f"Arm_{prefix}_finger{idx}").add_site(
                name=f"Arm_{prefix}_pad{idx}_inner",
                pos=[0.0, -0.223, sign * (PAD_INNER_Z_OPEN_M - inward)],
                size=[0.004, 0.0, 0.0],
            )


def _apply_marvin_appearance(spec: mujoco.MjSpec) -> None:
    """Apply matte white appearance to stand and 7 arm links."""
    white = [0.92, 0.92, 0.92, 1.0]
    white_bodies = {"ZJ_Robot_link"} | {
        f"Arm_{prefix}{i}_Link" for _, prefix in SIDES for i in range(1, 8)
    }
    for body_name in white_bodies:
        body = spec.body(body_name)
        for geom in body.geoms:
            geom.rgba = white


def _add_self_exclusions(spec: mujoco.MjSpec) -> None:
    """Exclude structural wrist hull overlap and opposing same-hand finger pads."""
    for _, prefix in SIDES:
        for first, second in SELF_EXCLUSION_PAIRS:
            spec.add_exclude(
                name=f"{prefix}_ex_{first}_{second}",
                bodyname1=f"Arm_{prefix}{first}_Link",
                bodyname2=f"Arm_{prefix}{second}_Link",
            )
        spec.add_exclude(
            name=f"{prefix}_ex_fingers",
            bodyname1=f"Arm_{prefix}_finger1",
            bodyname2=f"Arm_{prefix}_finger2",
        )


def add_marvin_robot(spec: mujoco.MjSpec, config: MarvinRobotConfig, urdf_path: Path) -> None:
    """Attach the source robot and add the calibrated CTAG2F90C gripper assembly."""
    child = mujoco.MjSpec.from_file(str(urdf_path))
    child.compiler.fusestatic = False
    child.compiler.discardvisual = False
    for key in list(child.keys):
        child.delete(key)
    mesh_dir = _prepare_meshes(child, urdf_path)
    semantics = read_urdf_semantics(urdf_path)
    _replace_oversized_stand(child, mesh_dir)
    _apply_marvin_appearance(child)
    _configure_finger_couplings(child)
    _add_gripper_assembly(child, config, mesh_dir)
    for body in child.bodies:
        if body.name.startswith("Arm_"):
            body.gravcomp = float(config.gravity_compensation)
    root = child.body("ZJ_Robot_link")
    root.pos = list(config.base_position)
    root.quat = list(_yaw_quaternion(config.base_yaw_deg))
    _add_actuators(child, config, semantics)
    _add_sites(child, config)
    _add_self_exclusions(child)
    frame = spec.worldbody.add_frame(name="marvin_root")
    previous_warning = mujoco.get_mju_user_warning()

    def _warning(message: str) -> None:
        if message.startswith("Attaching a spec with an empty prefix"):
            return
        if previous_warning is not None:
            previous_warning(message)
        else:
            print(message, file=sys.stderr)

    mujoco.set_mju_user_warning(_warning)
    try:
        spec.attach(child, prefix="", frame=frame)
    finally:
        mujoco.set_mju_user_warning(previous_warning)
    spec.meshdir = str(mesh_dir)
    spec.compiler.fusestatic = False
    spec.compiler.discardvisual = False


def _check_solver_params(model: mujoco.MjModel, eq_id: int, name: str) -> None:
    if any(
        abs(float(actual) - float(expected_value)) > 1e-12
        for actual, expected_value in zip(
            model.eq_solref[eq_id], COUPLING_SOLREF, strict=True
        )
    ) or any(
        abs(float(actual) - float(expected_value)) > 1e-12
        for actual, expected_value in zip(
            model.eq_solimp[eq_id], COUPLING_SOLIMP, strict=True
        )
    ):
        raise SceneBuildError(f"{name!r} solver parameters differ from the verified values")


def _verify_finger_couplings(model: mujoco.MjModel) -> None:
    """Verify the lateral mimic and quartic 4-bar axial equalities."""
    for _, prefix in SIDES:
        name = f"Arm_{prefix}_finger_coupling"
        try:
            equality = model.equality(name)
        except KeyError as error:
            raise SceneBuildError(f"compiled model lacks {name!r}") from error
        if int(model.eq_type[equality.id]) != int(mujoco.mjtEq.mjEQ_JOINT):
            raise SceneBuildError(f"{name!r} is not a joint equality")
        names = {
            model.joint(int(model.eq_obj1id[equality.id])).name,
            model.joint(int(model.eq_obj2id[equality.id])).name,
        }
        expected = {f"Arm_{prefix}_finger1_Joint", f"Arm_{prefix}_finger2_Joint"}
        if names != expected:
            raise SceneBuildError(f"{name!r} couples {sorted(names)}, expected {sorted(expected)}")
        data = model.eq_data[equality.id]
        if (
            abs(float(data[0])) > 1e-12
            or abs(float(data[1]) - 1.0) > 1e-12
            or any(abs(float(value)) > 1e-12 for value in data[2:5])
        ):
            raise SceneBuildError(f"{name!r} does not encode multiplier 1 / offset 0")
        _check_solver_params(model, equality.id, name)

        for idx in (1, 2):
            arc_name = f"Arm_{prefix}_axial{idx}_arc"
            try:
                arc = model.equality(arc_name)
            except KeyError as error:
                raise SceneBuildError(f"compiled model lacks {arc_name!r}") from error
            if int(model.eq_type[arc.id]) != int(mujoco.mjtEq.mjEQ_JOINT):
                raise SceneBuildError(f"{arc_name!r} is not a joint equality")
            obj1 = model.joint(int(model.eq_obj1id[arc.id])).name
            obj2 = model.joint(int(model.eq_obj2id[arc.id])).name
            if (obj1, obj2) != (
                f"Arm_{prefix}_axial{idx}_Joint",
                f"Arm_{prefix}_finger{idx}_Joint",
            ):
                raise SceneBuildError(f"{arc_name!r} couples {(obj1, obj2)}")
            _check_solver_params(model, arc.id, arc_name)

    expected_eq = len(SIDES) * 3
    if model.neq != expected_eq:
        raise SceneBuildError(
            f"expected exactly {expected_eq} finger/4-bar equalities, got {model.neq}"
        )


def bind_marvin_robot(
    model: mujoco.MjModel, config: MarvinRobotConfig, semantics: UrdfSemantics
) -> MarvinBinding:
    """Resolve and verify every name/address used by the Marvin contract."""
    if model.nu != 16:
        raise SceneBuildError(f"Marvin model has {model.nu} actuators, expected 16")
    if model.ntendon != 2:
        raise SceneBuildError(
            f"expected two gripper tendons, got ntendon={model.ntendon}"
        )
    _verify_finger_couplings(model)
    base_id = int(model.body("ZJ_Robot_link").id)
    if int(model.body_jntnum[base_id]) != 0:
        raise SceneBuildError("Marvin root body must be fixed, not jointed")
    arm_qpos: dict[str, tuple[int, ...]] = {}
    arm_dof: dict[str, tuple[int, ...]] = {}
    finger_qpos: dict[str, tuple[int, int]] = {}
    finger_dof: dict[str, tuple[int, int]] = {}
    axial_qpos: dict[str, tuple[int, int]] = {}
    axial_dof: dict[str, tuple[int, int]] = {}
    arm_actuators: dict[str, tuple[int, ...]] = {}
    gripper_actuators: dict[str, int] = {}
    tcp_sites: dict[str, int] = {}
    pad_sites: dict[str, dict[str, int]] = {}
    finger_pad_bodies: dict[str, tuple[int, int]] = {}
    arm_collision_bodies: dict[str, tuple[int, ...]] = {}
    hand_bodies: dict[str, int] = {}
    for public_side, prefix in SIDES:
        qpos: list[int] = []
        dof: list[int] = []
        actuators: list[int] = []
        for index in range(1, ARM_JOINT_COUNT + 1):
            name = f"Arm_{prefix}{index}_Joint"
            joint = model.joint(name)
            if int(model.jnt_type[joint.id]) != int(mujoco.mjtJoint.mjJNT_HINGE):
                raise SceneBuildError(f"joint {name!r} is not a hinge")
            source = semantics.joints[name]
            lower, upper = (float(value) for value in model.jnt_range[joint.id])
            if abs(lower - source.lower) > 1e-9 or abs(upper - source.upper) > 1e-9:
                raise SceneBuildError(
                    f"joint {name!r} range [{lower}, {upper}] differs from URDF "
                    f"[{source.lower}, {source.upper}]"
                )
            qpos.append(int(model.jnt_qposadr[joint.id]))
            dof.append(int(model.jnt_dofadr[joint.id]))
            actuator = model.actuator(f"Arm_{prefix}{index}_Actuator")
            if int(model.actuator_trntype[actuator.id]) != int(
                mujoco.mjtTrn.mjTRN_JOINT
            ) or int(model.actuator_trnid[actuator.id, 0]) != int(joint.id):
                raise SceneBuildError(f"actuator for {name!r} drives a different element")
            actuators.append(int(actuator.id))
        arm_qpos[public_side] = tuple(qpos)
        arm_dof[public_side] = tuple(dof)
        arm_actuators[public_side] = tuple(actuators)
        finger_addresses = []
        finger_dofs = []
        axial_addresses = []
        axial_dofs = []
        for idx in (1, 2):
            name = f"Arm_{prefix}_finger{idx}_Joint"
            joint = model.joint(name)
            if int(model.jnt_type[joint.id]) != int(mujoco.mjtJoint.mjJNT_SLIDE):
                raise SceneBuildError(f"joint {name!r} is not a prismatic finger joint")
            lower, upper = (float(value) for value in model.jnt_range[joint.id])
            if abs(lower) > 1e-9 or abs(upper - FINGER_TRAVEL) > 1e-9:
                raise SceneBuildError(
                    f"finger joint {name!r} range [{lower}, {upper}] is not [0, {FINGER_TRAVEL}]"
                )
            finger_addresses.append(int(model.jnt_qposadr[joint.id]))
            finger_dofs.append(int(model.jnt_dofadr[joint.id]))
            axial = model.joint(f"Arm_{prefix}_axial{idx}_Joint")
            if int(model.jnt_type[axial.id]) != int(mujoco.mjtJoint.mjJNT_SLIDE):
                raise SceneBuildError(f"joint {axial.name!r} is not a prismatic axial joint")
            axial_addresses.append(int(model.jnt_qposadr[axial.id]))
            axial_dofs.append(int(model.jnt_dofadr[axial.id]))
        finger_qpos[public_side] = (finger_addresses[0], finger_addresses[1])
        finger_dof[public_side] = (finger_dofs[0], finger_dofs[1])
        axial_qpos[public_side] = (axial_addresses[0], axial_addresses[1])
        axial_dof[public_side] = (axial_dofs[0], axial_dofs[1])
        gripper = model.actuator(f"Arm_{prefix}_gripper_actuator")
        if int(model.actuator_trntype[gripper.id]) != int(mujoco.mjtTrn.mjTRN_TENDON):
            raise SceneBuildError(f"gripper actuator {prefix!r} does not drive a tendon")
        gripper_actuators[public_side] = int(gripper.id)
        tcp_sites[public_side] = int(model.site(f"Arm_{prefix}_tcp").id)
        pad_sites[public_side] = {
            "finger1": int(model.site(f"Arm_{prefix}_pad1_inner").id),
            "finger2": int(model.site(f"Arm_{prefix}_pad2_inner").id),
        }
        finger_pad_bodies[public_side] = (
            int(model.body(f"Arm_{prefix}_finger1").id),
            int(model.body(f"Arm_{prefix}_finger2").id),
        )
        arm_collision_bodies[public_side] = tuple(
            int(model.body(f"Arm_{prefix}{i}_Link").id) for i in range(1, ARM_JOINT_COUNT + 1)
        ) + (
            int(model.body(f"Arm_{prefix}_gripper_base").id),
            int(model.body(f"Arm_{prefix}_finger1").id),
            int(model.body(f"Arm_{prefix}_finger2").id),
        )
        hand_bodies[public_side] = int(model.body(f"Arm_{prefix}_gripper_base").id)
    return MarvinBinding(
        arm_joint_qpos=MappingProxyType(arm_qpos),
        arm_joint_dof=MappingProxyType(arm_dof),
        finger_joint_qpos=MappingProxyType(finger_qpos),
        finger_joint_dof=MappingProxyType(finger_dof),
        axial_joint_qpos=MappingProxyType(axial_qpos),
        axial_joint_dof=MappingProxyType(axial_dof),
        arm_actuator_ids=MappingProxyType(arm_actuators),
        gripper_actuator_ids=MappingProxyType(gripper_actuators),
        tcp_site_ids=MappingProxyType(tcp_sites),
        pad_site_ids=MappingProxyType(pad_sites),
        finger_pad_body_ids=MappingProxyType(finger_pad_bodies),
        arm_collision_body_ids=MappingProxyType(arm_collision_bodies),
        hand_body_ids=MappingProxyType(hand_bodies),
        base_body_id=base_id,
        finger_travel=FINGER_TRAVEL,
        pad_inward_offset_m=float(config.pad_inward_offset_m),
    )


def pad_aperture_m(
    model: mujoco.MjModel, data: mujoco.MjData, robot: MarvinBinding, side: str
) -> float:
    """Physical inner-silicone-pad aperture (m) projected onto the closing axis."""
    del model
    pads = robot.pad_site_ids[side]
    first = data.site_xpos[pads["finger1"]]
    second = data.site_xpos[pads["finger2"]]
    hand = robot.hand_body_ids[side]
    axis = data.xmat[hand].reshape(3, 3)[:, 2]
    delta = (first[0] - second[0], first[1] - second[1], first[2] - second[2])
    inward = float(getattr(robot, "pad_inward_offset_m", 0.0))
    projected = sum(component * direction for component, direction in zip(delta, axis, strict=True))
    if inward <= 0.0:
        return abs(projected)
    return max(0.0, projected)


def gripper_opening(
    model: mujoco.MjModel, data: mujoco.MjData, robot: MarvinBinding, side: str
) -> float:
    """Public stroke opening (m) in [0.0, 0.090] matching CTAG2F90C encoder semantics."""
    del model
    pads = robot.pad_site_ids[side]
    first = data.site_xpos[pads["finger1"]]
    second = data.site_xpos[pads["finger2"]]
    hand = robot.hand_body_ids[side]
    axis = data.xmat[hand].reshape(3, 3)[:, 2]
    delta = (first[0] - second[0], first[1] - second[1], first[2] - second[2])
    inward = float(getattr(robot, "pad_inward_offset_m", 0.0))
    projected = sum(component * direction for component, direction in zip(delta, axis, strict=True))
    if inward <= 0.0:
        return abs(projected)
    return max(0.0, projected + 2.0 * inward)
