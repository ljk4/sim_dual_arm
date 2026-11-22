"""Robot-independent named I/O contract consumed by control and motion.

Every implementation maps the same public 16-value contract to its own compiled
model: left seven arm joints in radians, left total gripper opening in metres,
right seven arm joints, right total gripper opening.  Addresses are resolved by
full element name; no caller relies on adjacent qpos/dof/actuator ordering.

The implementations own no physics and never step.  A successful write means
only that the validated actuator commands were written; it says nothing about
tracking or arrival.
"""

from __future__ import annotations

import math
import numbers
import threading
from abc import ABC, abstractmethod
from collections.abc import Mapping, Sequence
from typing import NamedTuple

import mujoco
from robot_contracts.models import ContractError, JointSpec, RobotProfile
from robot_contracts.profiles import semantic_hash

from robot_backend_sim.common.config.camera_config import CameraConfigError, camera_declared_description
from robot_backend_sim.common.physics.scene_container import CompiledScene

__all__ = [
    "Measurement",
    "RobotIO",
    "check_owner_thread",
    "gripper_indices",
    "require_finite",
    "sim_time_ns",
    "validate_camera_contract",
    "validate_profile_contract",
    "validate_targets",
]

_MAX_TIME_NS = 2**63 - 1
_MAX_TIME_S = _MAX_TIME_NS / 1e9


class Measurement(NamedTuple):
    """Immutable SI measurement read directly from the current ``MjData``.

    ``sim_time_ns`` is the current MuJoCo simulation time rounded to integer
    nanoseconds.  ``positions`` are radians for the fourteen arm joints and
    metres of total finger opening for the two grippers.  ``velocities`` use
    rad/s and m/s respectively, with positive gripper velocity meaning opening.
    The tuples are fresh for every call.
    """

    sim_time_ns: int
    joint_names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]


class RobotIO(ABC):
    """Named public I/O for one compiled scene and one robot profile.

    Instances are bound to their creating thread because ``MjData`` is not
    thread-safe.  ``finger_bodies`` names the two contacts that form a grasp on
    each side as ``(body_name, joint_name)`` pairs; the generic contact layer
    measures them without knowing the robot family.
    """

    @property
    @abstractmethod
    def profile(self) -> RobotProfile: ...

    @property
    @abstractmethod
    def joint_names(self) -> tuple[str, ...]: ...

    @property
    @abstractmethod
    def finger_bodies(self) -> Mapping[str, tuple[tuple[str, str], ...]]: ...

    @abstractmethod
    def require_scene(self, scene: CompiledScene) -> None: ...

    @abstractmethod
    def measure(self) -> Measurement: ...

    @abstractmethod
    def apply_targets(self, positions: Sequence[float]) -> None: ...

    @abstractmethod
    def apply_reference(
        self, positions: Sequence[float], velocities: Sequence[float]
    ) -> None: ...


def check_owner_thread(owner: threading.Thread, name: str) -> None:
    if threading.current_thread() is not owner:
        raise ContractError("WRONG_THREAD", f"{name} must be used on its creating thread")


def sim_time_ns(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise ContractError("INVALID_MEASUREMENT", "simulation time must be a real number")
    time_s = float(value)
    if not math.isfinite(time_s) or time_s < 0.0 or time_s > _MAX_TIME_S:
        raise ContractError(
            "INVALID_MEASUREMENT",
            f"simulation time {time_s!r} is not a finite non-negative time",
        )
    return round(time_s * 1e9)


def require_finite(name: str, field: str, value: float) -> None:
    if not math.isfinite(value):
        raise ContractError("INVALID_MEASUREMENT", f"{name}.{field} is not finite")


def close(left: float, right: float) -> bool:
    return math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=1e-9)


def gripper_indices(joint_names: Sequence[str]) -> dict[str, int]:
    """Named gripper indices derived in joint order from ``*_gripper_width`` entries."""
    result: dict[str, int] = {}
    for index, name in enumerate(joint_names):
        if name.endswith("_gripper_width"):
            side = name[: -len("_gripper_width")]
            if not side or side in result:
                raise ContractError(
                    "PROFILE_MISMATCH", f"invalid duplicate or unnamed gripper joint {name!r}"
                )
            result[side] = index
    if not result:
        raise ContractError("PROFILE_MISMATCH", "joint order defines no *_gripper_width joint")
    return result


def validate_profile_contract(
    profile: RobotProfile, expected_names: Sequence[str]
) -> tuple[JointSpec, ...]:
    """Check public order, units, limits and velocity bounds once per binding."""
    if not isinstance(profile, RobotProfile):
        raise ContractError("PROFILE_MISMATCH", "profile must be a RobotProfile")
    joints = profile.joints
    expected = tuple(expected_names)
    if len(joints) != len(expected):
        raise ContractError(
            "PROFILE_MISMATCH",
            f"joints: expected {len(expected)} entries, got {len(joints)}",
        )
    for index, (joint, expected_name) in enumerate(zip(joints, expected, strict=True)):
        if joint.name != expected_name:
            raise ContractError(
                "PROFILE_MISMATCH",
                f"joints[{index}].name: expected {expected_name!r}, got {joint.name!r}",
            )
        expected_unit = "m" if expected_name.endswith("_gripper_width") else "rad"
        if joint.unit != expected_unit:
            raise ContractError(
                "PROFILE_MISMATCH",
                f"joints[{index}] ({joint.name}).unit: expected {expected_unit!r}, "
                f"got {joint.unit!r}",
            )
        if not math.isfinite(joint.max_velocity) or joint.max_velocity <= 0.0:
            raise ContractError(
                "PROFILE_MISMATCH",
                f"joints[{index}] ({joint.name}).max_velocity must be positive",
            )
    if not isinstance(profile.digest, str) or not profile.digest:
        raise ContractError("PROFILE_MISMATCH", "robot profile digest is missing")
    return joints


def validate_camera_contract(profile: RobotProfile, scene: CompiledScene) -> tuple[str, ...]:
    """Verify profile cameras against the scene declaration and compiled model."""
    cameras = profile.cameras
    scene_cameras = scene.config.cameras
    if len(cameras) != len(scene_cameras):
        raise ContractError(
            "CAMERA_MISMATCH",
            f"profile has {len(cameras)} cameras, scene has {len(scene_cameras)}",
        )
    for index, (camera, scene_camera) in enumerate(zip(cameras, scene_cameras, strict=True)):
        if camera.name != scene_camera.name:
            raise ContractError(
                "CAMERA_MISMATCH",
                f"cameras[{index}].name: expected {scene_camera.name!r}, got {camera.name!r}",
            )
        if camera.width != scene_camera.width or camera.height != scene_camera.height:
            raise ContractError(
                "CAMERA_MISMATCH",
                f"cameras[{index}] ({camera.name}): profile size "
                f"{camera.width}x{camera.height} does not match scene "
                f"{scene_camera.width}x{scene_camera.height}",
            )
        try:
            declaration = camera_declared_description(scene_camera)
        except CameraConfigError as error:
            raise ContractError(
                "CAMERA_MISMATCH",
                f"cameras[{index}] ({camera.name}): {error.field}: {error.message}",
            ) from error
        if camera.calibration_digest != semantic_hash(declaration):
            raise ContractError(
                "CAMERA_MISMATCH",
                f"cameras[{index}] ({camera.name}).calibration_digest: profile value "
                "does not match the declared scene camera description",
            )
    _validate_compiled_cameras(profile, scene)
    return tuple(camera.frame_id for camera in cameras)


def _validate_compiled_cameras(profile: RobotProfile, scene: CompiledScene) -> None:
    model = scene.model
    for camera in scene.config.cameras:
        try:
            camera_id = int(model.camera(camera.name).id)
            parent_id = int(model.body(camera.parent).id)
        except KeyError as error:
            raise ContractError("CAMERA_MISMATCH", f"missing camera/body: {camera.name}") from error
        if int(model.cam_bodyid[camera_id]) != parent_id or scene.indices.camera_ids.get(
            camera.name
        ) != camera_id:
            raise ContractError("CAMERA_MISMATCH", f"{camera.name}: parent or named index differs")
        if int(model.cam_mode[camera_id]) != int(mujoco.mjtCamLight.mjCAMLIGHT_FIXED):
            raise ContractError(
                "CAMERA_MISMATCH", f"{camera.name}: camera must have fixed local pose"
            )
        if tuple(int(value) for value in model.cam_resolution[camera_id]) != (
            camera.width,
            camera.height,
        ):
            raise ContractError("CAMERA_MISMATCH", f"{camera.name}: compiled resolution differs")
        if not close(model.cam_fovy[camera_id], camera.fovy):
            raise ContractError("CAMERA_MISMATCH", f"{camera.name}: compiled fovy differs")
        if any(
            not close(a, b) for a, b in zip(model.cam_pos[camera_id], camera.position, strict=True)
        ):
            raise ContractError("CAMERA_MISMATCH", f"{camera.name}: compiled position differs")
        actual = tuple(float(value) for value in model.cam_quat[camera_id])
        expected = camera.quaternion_wxyz
        same_rotation = all(close(a, b) for a, b in zip(actual, expected, strict=True)) or all(
            close(a, -b) for a, b in zip(actual, expected, strict=True)
        )
        if not same_rotation:
            raise ContractError("CAMERA_MISMATCH", f"{camera.name}: compiled orientation differs")


def validate_targets(
    positions: Sequence[float], joints: Sequence[JointSpec]
) -> tuple[float, ...]:
    """Validate one public target vector before any actuator write."""
    expected = len(joints)
    if isinstance(positions, str | bytes | bytearray):
        raise ContractError(
            "INVALID_TARGET", f"positions must be a sequence of {expected} numbers"
        )
    try:
        values = tuple(positions)
    except TypeError as error:
        raise ContractError(
            "INVALID_TARGET", f"positions must be a sequence of {expected} numbers"
        ) from error
    if len(values) != expected:
        raise ContractError(
            "INVALID_TARGET",
            f"positions: expected {expected} entries, got {len(values)}",
        )
    result = []
    for index, (value, joint) in enumerate(zip(values, joints, strict=True)):
        if isinstance(value, bool) or not isinstance(value, numbers.Real):
            raise ContractError(
                "INVALID_TARGET",
                f"positions[{index}] ({joint.name}): expected a real number, "
                f"got {type(value).__name__}",
            )
        try:
            number = float(value)
        except (OverflowError, TypeError, ValueError) as error:
            raise ContractError(
                "INVALID_TARGET",
                f"positions[{index}] ({joint.name}): not representable as a float",
            ) from error
        if not math.isfinite(number):
            raise ContractError(
                "INVALID_TARGET",
                f"positions[{index}] ({joint.name}): must be finite",
            )
        if not joint.minimum <= number <= joint.maximum:
            raise ContractError(
                "JOINT_LIMIT",
                f"positions[{index}] ({joint.name}): {number} outside "
                f"[{joint.minimum}, {joint.maximum}]",
            )
        result.append(number)
    return tuple(result)
