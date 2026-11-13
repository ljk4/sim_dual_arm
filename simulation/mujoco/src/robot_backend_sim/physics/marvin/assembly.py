"""Fixed-base Marvin dual-arm assembly.

The source URDF is imported unchanged.  Import adaptation is limited to mesh
path resolution, preservation of the fixed wrist frames, one in-memory split of
the oversized stand mesh, naming and tuning the two finger-mimic equalities
already imported from the source URDF, and the configured position servos,
self-weight compensation and finger slide damping. The only
contact-with-self exclusion is the structurally overlapping ``Arm_*5`` /
``Arm_*7`` pair; left/right arm collision and both finger bodies remain
collidable.
"""

from __future__ import annotations

import math
import struct
import warnings
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType

import mujoco

from robot_backend_sim.config.marvin_scene_config import MarvinRobotConfig
from robot_backend_sim.physics.scene_container import SceneBuildError

__all__ = [
    "ARM_JOINT_COUNT",
    "MarvinBinding",
    "UrdfSemantics",
    "add_marvin_robot",
    "bind_marvin_robot",
    "read_urdf_semantics",
]

SIDES = (("left", "L"), ("right", "R"))
ARM_JOINT_COUNT = 7
FINGER_TRAVEL = 0.04
FINGER_ARMATURE = 0.001
# The imported source-mimic equalities run at this verified stiffness.  The
# P01 duplicate manual equalities used the same values; keeping one copy avoids
# two constraints acting on the same finger pair.
COUPLING_SOLREF = (0.005, 1.0)
COUPLING_SOLIMP = (0.95, 0.99, 0.001, 0.5, 2.0)
# Source collision meshes are convex hulls.  The Arm_*5 and Arm_*7 hulls
# overlap by about 0.061 m at the reset pose and at every sampled reachable
# pose because the Arm_*6 wrist lies between them.  That pair is excluded so
# the overlap cannot inject permanent contact forces.  No other same-arm pair
# is excluded.
SELF_EXCLUSION_PAIRS = ((5, 7),)
_MUJOCO_STL_FACE_LIMIT = 200_000


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
    """Named MuJoCo addresses for the public 16-value Marvin contract."""

    arm_joint_qpos: Mapping[str, tuple[int, ...]]
    arm_joint_dof: Mapping[str, tuple[int, ...]]
    finger_joint_qpos: Mapping[str, tuple[int, int]]
    finger_joint_dof: Mapping[str, tuple[int, int]]
    arm_actuator_ids: Mapping[str, tuple[int, ...]]
    gripper_actuator_ids: Mapping[str, int]
    tcp_site_ids: Mapping[str, int]
    pad_site_ids: Mapping[str, Mapping[str, int]]
    hand_body_ids: Mapping[str, int]
    base_body_id: int
    finger_travel: float


def _side_prefix(public_side: str) -> str:
    for public, prefix in SIDES:
        if public == public_side:
            return prefix
    raise KeyError(public_side)


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
    # The stand is a visual reference; its 216k-face hull is not a useful
    # collision surface. Zero contype/conaffinity keeps it visible only.  The
    # P01A workspace is in front of the base (table x >= 0.40 m), so no proxy
    # is added; a path entering the base volume needs a verified primitive
    # proxy instead of this hull.
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


def _stl_bounds(path: Path) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    triangles = _read_binary_stl(path)
    xs = [vertex[0] for triangle in triangles for vertex in triangle]
    ys = [vertex[1] for triangle in triangles for vertex in triangle]
    zs = [vertex[2] for triangle in triangles for vertex in triangle]
    return (min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs))


def _pad_offsets(mesh_dir: Path) -> dict[str, tuple[float, float, float]]:
    """Minimum-aperture references projected to the hand's TCP station.

    The source fingers have tapered surfaces. Their innermost tips define the
    reported opening; these sites are measurement references, not a pair of
    flat collision pads spanning the finger length.
    """
    offsets: dict[str, tuple[float, float, float]] = {}
    for prefix in ("L", "R"):
        for finger in (10, 11):
            lower, upper = _stl_bounds(mesh_dir / f"Arm_{prefix}{finger}_Link.STL")
            # Use the common hand axis x=0 so the measured distance is the
            # perpendicular gap between the two inner finger planes.
            x = 0.0
            y = (lower[1] + upper[1]) / 2.0
            if finger == 10 or prefix == "R":
                inner = lower[2]
            else:
                inner = upper[2]
            offsets[f"Arm_{prefix}_pad{finger}_inner"] = (x, y, inner)
    return offsets


def _configure_finger_couplings(spec: mujoco.MjSpec) -> None:
    """Name and tune the two joint equalities imported from the URDF mimic.

    The source declares ``Arm_*11_Joint`` as a mimic of ``Arm_*10_Joint``.
    MuJoCo's URDF importer turns each declaration into one ``mjEQ_JOINT``
    constraint, so the assembly must not add a second equality for the same
    pair.
    """
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
                (f"Arm_{prefix}10_Joint", f"Arm_{prefix}11_Joint")
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
    for _public_side, prefix in SIDES:
        for index in range(1, ARM_JOINT_COUNT + 1):
            name = f"Arm_{prefix}{index}_Joint"
            joint = spec.joint(name)
            source = semantics.joints[name]
            if not source.lower <= config.home_joint_positions[index - 1] <= source.upper:
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
        for finger in (10, 11):
            joint = spec.joint(f"Arm_{prefix}{finger}_Joint")
            joint.damping[:] = config.finger_damping
            joint.armature = FINGER_ARMATURE
        tendon = spec.add_tendon(name=f"Arm_{prefix}_gripper")
        tendon.wrap_joint(f"Arm_{prefix}10_Joint", 1.0)
        tendon.wrap_joint(f"Arm_{prefix}11_Joint", 1.0)
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


def _add_sites(spec: mujoco.MjSpec, mesh_dir: Path) -> None:
    offsets = _pad_offsets(mesh_dir)
    for _, prefix in SIDES:
        # TCP at the chosen station midway between aperture references.
        tcp_y = offsets[f"Arm_{prefix}_pad10_inner"][1]
        spec.body(f"Arm_{prefix}9_Link").add_site(
            name=f"Arm_{prefix}_tcp", pos=[0.0, tcp_y, 0.0], size=[0.006, 0.0, 0.0]
        )
        for finger in (10, 11):
            spec.body(f"Arm_{prefix}{finger}_Link").add_site(
                name=f"Arm_{prefix}_pad{finger}_inner",
                pos=list(offsets[f"Arm_{prefix}_pad{finger}_inner"]),
                size=[0.004, 0.0, 0.0],
            )


def _add_self_exclusions(spec: mujoco.MjSpec) -> None:
    """Exclude only the pair whose source convex hulls overlap structurally."""
    for _, prefix in SIDES:
        for first, second in SELF_EXCLUSION_PAIRS:
            spec.add_exclude(
                name=f"{prefix}_ex_{first}_{second}",
                bodyname1=f"Arm_{prefix}{first}_Link",
                bodyname2=f"Arm_{prefix}{second}_Link",
            )


def add_marvin_robot(spec: mujoco.MjSpec, config: MarvinRobotConfig, urdf_path: Path) -> None:
    """Attach the source robot and add the executable assembly elements."""
    child = mujoco.MjSpec.from_file(str(urdf_path))
    child.compiler.fusestatic = False
    child.compiler.discardvisual = False
    for key in list(child.keys):
        child.delete(key)
    mesh_dir = _prepare_meshes(child, urdf_path)
    semantics = read_urdf_semantics(urdf_path)
    _replace_oversized_stand(child, mesh_dir)
    _configure_finger_couplings(child)
    for body in child.bodies:
        if body.name.startswith("Arm_"):
            body.gravcomp = float(config.gravity_compensation)
    root = child.body("ZJ_Robot_link")
    root.pos = list(config.base_position)
    root.quat = list(_yaw_quaternion(config.base_yaw_deg))
    _add_actuators(child, config, semantics)
    _add_sites(child, mesh_dir)
    _add_self_exclusions(child)
    frame = spec.worldbody.add_frame(name="marvin_root")
    # Source names must stay unprefixed; this spec is compiled in memory and
    # never saved, so the empty-prefix save warning does not apply.
    with warnings.catch_warnings():
        warnings.filterwarnings(
            "ignore",
            message="Attaching a spec with an empty prefix.*",
            category=UserWarning,
        )
        spec.attach(child, prefix="", frame=frame)
    spec.meshdir = str(mesh_dir)
    spec.compiler.fusestatic = False
    spec.compiler.discardvisual = False


def _verify_finger_couplings(model: mujoco.MjModel) -> None:
    """Verify the imported mimic constraints are the active gripper linkage."""
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
        expected = {f"Arm_{prefix}10_Joint", f"Arm_{prefix}11_Joint"}
        if names != expected:
            raise SceneBuildError(f"{name!r} couples {sorted(names)}, expected {sorted(expected)}")
        data = model.eq_data[equality.id]
        if (
            abs(float(data[0])) > 1e-12
            or abs(float(data[1]) - 1.0) > 1e-12
            or any(abs(float(value)) > 1e-12 for value in data[2:5])
        ):
            raise SceneBuildError(f"{name!r} does not encode multiplier 1 / offset 0")
        if any(
            abs(float(actual) - float(expected_value)) > 1e-12
            for actual, expected_value in zip(
                model.eq_solref[equality.id], COUPLING_SOLREF, strict=True
            )
        ) or any(
            abs(float(actual) - float(expected_value)) > 1e-12
            for actual, expected_value in zip(
                model.eq_solimp[equality.id], COUPLING_SOLIMP, strict=True
            )
        ):
            raise SceneBuildError(f"{name!r} solver parameters differ from the verified values")
    if model.neq != len(SIDES):
        raise SceneBuildError(
            f"expected exactly {len(SIDES)} finger couplings, got {model.neq} equalities"
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
    arm_actuators: dict[str, tuple[int, ...]] = {}
    gripper_actuators: dict[str, int] = {}
    tcp_sites: dict[str, int] = {}
    pad_sites: dict[str, dict[str, int]] = {}
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
        for finger in (10, 11):
            name = f"Arm_{prefix}{finger}_Joint"
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
        finger_qpos[public_side] = (finger_addresses[0], finger_addresses[1])
        finger_dof[public_side] = (finger_dofs[0], finger_dofs[1])
        gripper = model.actuator(f"Arm_{prefix}_gripper_actuator")
        if int(model.actuator_trntype[gripper.id]) != int(mujoco.mjtTrn.mjTRN_TENDON):
            raise SceneBuildError(f"gripper actuator {prefix!r} does not drive a tendon")
        gripper_actuators[public_side] = int(gripper.id)
        tcp_sites[public_side] = int(model.site(f"Arm_{prefix}_tcp").id)
        pad_sites[public_side] = {
            "finger10": int(model.site(f"Arm_{prefix}_pad10_inner").id),
            "finger11": int(model.site(f"Arm_{prefix}_pad11_inner").id),
        }
        hand_bodies[public_side] = int(model.body(f"Arm_{prefix}9_Link").id)
    return MarvinBinding(
        arm_joint_qpos=MappingProxyType(arm_qpos),
        arm_joint_dof=MappingProxyType(arm_dof),
        finger_joint_qpos=MappingProxyType(finger_qpos),
        finger_joint_dof=MappingProxyType(finger_dof),
        arm_actuator_ids=MappingProxyType(arm_actuators),
        gripper_actuator_ids=MappingProxyType(gripper_actuators),
        tcp_site_ids=MappingProxyType(tcp_sites),
        pad_site_ids=MappingProxyType(pad_sites),
        hand_body_ids=MappingProxyType(hand_bodies),
        base_body_id=base_id,
        finger_travel=FINGER_TRAVEL,
    )
