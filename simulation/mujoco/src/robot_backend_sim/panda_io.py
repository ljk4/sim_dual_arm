"""Bind the accepted 16-dimensional Panda profile to the real MuJoCo scene.

``PandaIO`` is deliberately a thin, named mapping.  It resolves every arm
joint, finger joint, actuator and qpos/dof address by full name, verifies the
profile against the compiled model and :class:`SceneConfig`, then exposes
measured SI state and validated raw actuator targets.  It does not schedule
targets, step the model or claim that a command has been reached; those
responsibilities belong to the later physics control core.
"""

from __future__ import annotations

import math
import numbers
import threading
from collections.abc import Sequence
from typing import NamedTuple

import mujoco
from robot_contracts.models import ContractError, JointSpec, RobotProfile
from robot_contracts.profiles import semantic_hash

from .scene import (
    FINGER_OPENING_RANGE,
    GRIPPER_CTRL_RANGE,
    Scene,
    gripper_ctrl,
)
from .scene_config import (
    FINGER_JOINT_LIMIT,
    PANDA_JOINT_RANGES,
    SceneConfig,
    SceneConfigError,
    camera_declared_description,
)

__all__ = ["JOINT_NAMES", "Measurement", "PandaIO"]

SIDES = ("left", "right")
JOINT_NAMES = tuple(
    f"{side}_{suffix}"
    for side in SIDES
    for suffix in (
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "joint7",
        "gripper_width",
    )
)
_ARM_COUNT = 7
_MAX_TIME_NS = 2**63 - 1
_MAX_TIME_S = _MAX_TIME_NS / 1e9


class Measurement(NamedTuple):
    """Immutable SI measurement read directly from the current ``MjData``.

    ``sim_time_ns`` is the current MuJoCo simulation time rounded to integer
    nanoseconds.  ``positions`` are radians for the fourteen arm joints and
    metres of total finger opening for the two grippers.  ``velocities`` use
    rad/s and m/s respectively.  The tuples are fresh for every call.
    """

    sim_time_ns: int
    joint_names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]


class PandaIO:
    """Validated named I/O for one dual-Panda scene and one robot profile.

    The object owns no physics state and performs no stepping.  A successful
    :meth:`apply_targets` means only that the sixteen ctrl entries were
    written; it says nothing about tracking or arrival.  Instances are bound
    to their creating thread because ``MjData`` is not thread-safe.
    """

    def __init__(
        self, scene: Scene, profile: RobotProfile, *, velocity_feedforward: bool = False
    ) -> None:
        self._owner_thread = threading.current_thread()
        self._scene = scene
        self._profile = profile
        self._joint_specs = _validate_profile(profile, scene.config)
        self._camera_frame_ids = _validate_cameras(profile, scene.config)
        _validate_compiled_cameras(scene)
        self._binding = _resolve_binding(scene, self._joint_specs)
        self.velocity_feedforward = velocity_feedforward
        self._velocity_gains = self._servo_velocity_gains() if velocity_feedforward else ()

    @property
    def joint_names(self) -> tuple[str, ...]:
        """The fixed SI joint order, independent of model name adjacency."""
        return JOINT_NAMES

    @property
    def camera_names(self) -> tuple[str, ...]:
        return tuple(camera.name for camera in self._profile.cameras)

    @property
    def camera_frame_ids(self) -> tuple[str, ...]:
        """Validated profile optical frame IDs for a later ROS adapter.

        The MuJoCo scene has no ROS optical frame objects; these IDs are
        published from the checked profile, not derived from the scene.
        """
        return self._camera_frame_ids

    @property
    def profile(self) -> RobotProfile:
        return self._profile

    def require_scene(self, scene: Scene) -> None:
        """Require the caller's thread and the exact Scene this I/O was bound to.

        A ``PandaIO`` owns no physics, but its qpos/dof/actuator addresses are
        valid only for the compiled ``Scene`` passed at construction.  This
        public check rejects both a wrong thread and a different Scene before a
        caller reads or writes any writable state.
        """
        self._check_thread()
        if scene is not self._scene:
            raise ContractError(
                "SCENE_MISMATCH",
                "PandaIO belongs to a different Scene than the caller supplied",
            )

    def measure(self) -> Measurement:
        """Read actual joint/finger state without stepping or caching targets."""
        self._check_thread()
        data = self._scene.data
        sim_time_ns = _sim_time_ns(data.time)
        positions: list[float] = []
        velocities: list[float] = []
        for side in SIDES:
            for name, qpos_address, dof_address in zip(
                _side_arm_names(side),
                self._binding.arm_qpos[side],
                self._binding.arm_dof[side],
                strict=True,
            ):
                position = float(data.qpos[qpos_address])
                velocity = float(data.qvel[dof_address])
                _require_finite(name, "position", position)
                _require_finite(name, "velocity", velocity)
                positions.append(position)
                velocities.append(velocity)
            finger_positions = self._binding.finger_qpos[side]
            finger_dofs = self._binding.finger_dof[side]
            total_position = float(data.qpos[finger_positions[0]] + data.qpos[finger_positions[1]])
            total_velocity = float(data.qvel[finger_dofs[0]] + data.qvel[finger_dofs[1]])
            gripper_name = f"{side}_gripper_width"
            _require_finite(gripper_name, "position", total_position)
            _require_finite(gripper_name, "velocity", total_velocity)
            positions.append(total_position)
            velocities.append(total_velocity)
        return Measurement(
            sim_time_ns=sim_time_ns,
            joint_names=tuple(JOINT_NAMES),
            positions=tuple(positions),
            velocities=tuple(velocities),
        )

    def apply_targets(self, positions: Sequence[float]) -> None:
        """Validate all sixteen SI targets, then write the matching ctrl entries.

        Arm targets are radians.  Gripper targets are total finger opening in
        metres and are converted continuously to the vendor's raw 0..255
        command (0.04 m becomes 127.5).  Nothing is clipped.  Validation
        happens before the first write, so a rejected target leaves ctrl and
        every physical field untouched.
        """
        self._check_thread()
        values = _validate_targets(positions, self._joint_specs)
        raw_targets: list[float] = []
        for side in SIDES:
            offset = 8 if side == "right" else 0
            raw_targets.extend(values[offset : offset + _ARM_COUNT])
            raw_targets.append(gripper_ctrl(values[offset + _ARM_COUNT]))
        ctrl = self._scene.data.ctrl
        for actuator_id, value in zip(self._binding.actuator_ids, raw_targets, strict=True):
            ctrl[actuator_id] = value

    def _servo_velocity_gains(self) -> tuple[float, ...]:
        """Convert SI reference velocity to the pinned affine PD ctrl units.

        force = gain*ctrl - kp*length - kd*length_velocity. Adding
        kd/gain*desired_length_velocity supplies the missing desired velocity.
        The gripper tendon length is half the total SI finger opening.
        """
        gains = []
        model = self._scene.model
        for axis, actuator in enumerate(self._binding.actuator_ids):
            gain = float(model.actuator_gainprm[actuator, 0])
            damping = -float(model.actuator_biasprm[actuator, 2])
            if not math.isfinite(gain) or gain <= 0 or not math.isfinite(damping) or damping < 0:
                raise ContractError("MODEL_MISMATCH", "invalid affine Panda servo gain")
            length_per_si = 0.5 if axis % 8 == 7 else 1.0
            gains.append(damping / gain * length_per_si)
        return tuple(gains)

    def apply_reference(self, positions: Sequence[float], velocities: Sequence[float]) -> None:
        """Track one unchanged position/velocity reference through affine PD.

        This is a low-level actuator command, not a change to policy targets or
        trajectory labels. Actuator ctrl/force ranges still apply.
        Disabled mode is exactly the original position-only I/O.
        """
        if not self.velocity_feedforward:
            self.apply_targets(positions)
            return
        self._check_thread()
        values = _validate_targets(positions, self._joint_specs)
        if len(velocities) != len(values):
            raise ContractError("INVALID_TARGET", "reference velocity dimension differs")
        raw = []
        for axis, (position, velocity, spec, gain) in enumerate(
            zip(values, velocities, self._joint_specs, self._velocity_gains, strict=True)
        ):
            _require_finite(spec.name, "reference_velocity", velocity)
            if abs(velocity) > spec.max_velocity + 1e-9:
                raise ContractError("INVALID_TARGET", "reference velocity exceeds hard limit")
            base = gripper_ctrl(position) if axis % 8 == 7 else position
            command = base + gain * velocity
            actuator = self._binding.actuator_ids[axis]
            if self._scene.model.actuator_ctrllimited[actuator]:
                lower, upper = self._scene.model.actuator_ctrlrange[actuator]
                command = max(float(lower), min(float(upper), command))
            raw.append(command)
        # Validate the whole reference before the first write.
        for actuator, command in zip(self._binding.actuator_ids, raw, strict=True):
            self._scene.data.ctrl[actuator] = command

    def _check_thread(self) -> None:
        if threading.current_thread() is not self._owner_thread:
            raise ContractError(
                "WRONG_THREAD",
                "PandaIO must be used on the thread that created it",
            )


class _Binding(NamedTuple):
    arm_qpos: dict[str, tuple[int, ...]]
    arm_dof: dict[str, tuple[int, ...]]
    finger_qpos: dict[str, tuple[int, int]]
    finger_dof: dict[str, tuple[int, int]]
    actuator_ids: tuple[int, ...]


def _side_arm_names(side: str) -> tuple[str, ...]:
    return tuple(f"{side}_joint{index}" for index in range(1, 8))


def _close(left: float, right: float) -> bool:
    return math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=1e-9)


def _validate_profile(profile: RobotProfile, config: SceneConfig) -> tuple[JointSpec, ...]:
    if not isinstance(profile, RobotProfile):
        raise ContractError("PROFILE_MISMATCH", "profile must be a RobotProfile")
    joints = profile.joints
    if len(joints) != len(JOINT_NAMES):
        raise ContractError(
            "PROFILE_MISMATCH",
            f"joints: expected 16 entries, got {len(joints)}",
        )
    for index, (joint, expected_name) in enumerate(zip(joints, JOINT_NAMES, strict=True)):
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

    home = config.reset
    home_positions = tuple(float(value) for value in home.home_joint_positions)
    gripper_opening = float(home.gripper_total_opening)
    if len(home_positions) != _ARM_COUNT:
        raise ContractError(
            "MODEL_MISMATCH",
            f"scene reset home must contain 7 arm positions, got {len(home_positions)}",
        )
    expected_home = (
        *home_positions[:7],
        gripper_opening,
        *home_positions[:7],
        gripper_opening,
    )
    expected_ranges = (
        *PANDA_JOINT_RANGES,
        (0.0, FINGER_OPENING_RANGE),
        *PANDA_JOINT_RANGES,
        (0.0, FINGER_OPENING_RANGE),
    )
    for index, (joint, initial, (lower, upper)) in enumerate(
        zip(joints, expected_home, expected_ranges, strict=True)
    ):
        if not _close(joint.minimum, lower) or not _close(joint.maximum, upper):
            raise ContractError(
                "PROFILE_MISMATCH",
                f"joints[{index}] ({joint.name}).range: expected [{lower}, {upper}], "
                f"got [{joint.minimum}, {joint.maximum}]",
            )
        if not _close(joint.initial, initial):
            raise ContractError(
                "PROFILE_MISMATCH",
                f"joints[{index}] ({joint.name}).initial: expected {initial}, got {joint.initial}",
            )
        if not math.isfinite(joint.max_velocity) or joint.max_velocity <= 0.0:
            raise ContractError(
                "PROFILE_MISMATCH",
                f"joints[{index}] ({joint.name}).max_velocity must be positive",
            )
    return joints


def _validate_cameras(profile: RobotProfile, config: SceneConfig) -> tuple[str, ...]:
    cameras = profile.cameras
    scene_cameras = config.cameras
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
        except SceneConfigError as error:
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
    return tuple(camera.frame_id for camera in cameras)


def _validate_compiled_cameras(scene: Scene) -> None:
    """Check the actual native cameras against the already validated declaration."""
    model = scene.model
    for camera in scene.config.cameras:
        try:
            camera_id = int(model.camera(camera.name).id)
            parent_id = int(model.body(camera.parent).id)
        except KeyError as error:
            raise ContractError("CAMERA_MISMATCH", f"missing camera/body: {camera.name}") from error
        if (
            int(model.cam_bodyid[camera_id]) != parent_id
            or scene.indices.camera_ids.get(camera.name) != camera_id
        ):
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
        if not _close(model.cam_fovy[camera_id], camera.fovy):
            raise ContractError("CAMERA_MISMATCH", f"{camera.name}: compiled fovy differs")
        if any(
            not _close(a, b) for a, b in zip(model.cam_pos[camera_id], camera.position, strict=True)
        ):
            raise ContractError("CAMERA_MISMATCH", f"{camera.name}: compiled position differs")
        actual = tuple(float(value) for value in model.cam_quat[camera_id])
        expected = camera.quaternion_wxyz
        same_rotation = all(_close(a, b) for a, b in zip(actual, expected, strict=True)) or all(
            _close(a, -b) for a, b in zip(actual, expected, strict=True)
        )
        if not same_rotation:
            raise ContractError("CAMERA_MISMATCH", f"{camera.name}: compiled orientation differs")


def _resolve_binding(scene: Scene, joints: tuple[JointSpec, ...]) -> _Binding:
    model = scene.model
    data = scene.data
    if model.nu != len(JOINT_NAMES):
        raise ContractError(
            "MODEL_MISMATCH",
            f"model has {model.nu} actuators, expected {len(JOINT_NAMES)}",
        )
    if len(data.qpos) != model.nq or len(data.qvel) != model.nv or len(data.ctrl) != model.nu:
        raise ContractError("MODEL_MISMATCH", "scene data dimensions do not match the model")

    arm_qpos: dict[str, tuple[int, ...]] = {}
    arm_dof: dict[str, tuple[int, ...]] = {}
    finger_qpos: dict[str, tuple[int, int]] = {}
    finger_dof: dict[str, tuple[int, int]] = {}
    actuator_ids: list[int] = []
    spec_by_name = {joint.name: joint for joint in joints}

    for side in SIDES:
        qpos_addresses = []
        dof_addresses = []
        for index in range(1, 8):
            name = f"{side}_joint{index}"
            joint_id = _joint_id(model, name)
            _require_joint(model, name, joint_id, mujoco.mjtJoint.mjJNT_HINGE)
            expected = spec_by_name[name]
            _require_joint_range(model, name, joint_id, expected)
            qpos_addresses.append(_joint_qpos(model, name, joint_id))
            dof_addresses.append(_joint_dof(model, name, joint_id))
            actuator_id = _actuator_id(model, f"{side}_actuator{index}")
            _require_actuator(
                model,
                f"{side}_actuator{index}",
                actuator_id,
                mujoco.mjtTrn.mjTRN_JOINT,
                joint_id,
                expected.minimum,
                expected.maximum,
            )
            actuator_ids.append(actuator_id)
        arm_qpos[side] = tuple(qpos_addresses)
        arm_dof[side] = tuple(dof_addresses)

        finger_qpos_addresses = []
        finger_dof_addresses = []
        for index in (1, 2):
            name = f"{side}_finger_joint{index}"
            joint_id = _joint_id(model, name)
            _require_joint(model, name, joint_id, mujoco.mjtJoint.mjJNT_SLIDE)
            _require_joint_range(
                model,
                name,
                joint_id,
                _Range(0.0, FINGER_JOINT_LIMIT),
            )
            finger_qpos_addresses.append(_joint_qpos(model, name, joint_id))
            finger_dof_addresses.append(_joint_dof(model, name, joint_id))
        finger_qpos[side] = (finger_qpos_addresses[0], finger_qpos_addresses[1])
        finger_dof[side] = (finger_dof_addresses[0], finger_dof_addresses[1])

        gripper_name = f"{side}_actuator8"
        gripper_id = _actuator_id(model, gripper_name)
        tendon_id = _tendon_id(model, f"{side}_split")
        _require_actuator(
            model,
            gripper_name,
            gripper_id,
            mujoco.mjtTrn.mjTRN_TENDON,
            tendon_id,
            0.0,
            GRIPPER_CTRL_RANGE,
        )
        actuator_ids.append(gripper_id)

    if len(set(actuator_ids)) != len(JOINT_NAMES):
        raise ContractError("MODEL_MISMATCH", "the sixteen actuator ids are not unique")
    _require_scene_indices(scene, arm_qpos, finger_qpos, actuator_ids)
    return _Binding(
        arm_qpos=arm_qpos,
        arm_dof=arm_dof,
        finger_qpos=finger_qpos,
        finger_dof=finger_dof,
        actuator_ids=tuple(actuator_ids),
    )


class _Range(NamedTuple):
    minimum: float
    maximum: float


def _joint_id(model: mujoco.MjModel, name: str) -> int:
    try:
        return int(model.joint(name).id)
    except KeyError as error:
        raise ContractError("MODEL_MISMATCH", f"joint {name!r} is missing") from error


def _joint_qpos(model: mujoco.MjModel, name: str, joint_id: int) -> int:
    address = int(model.jnt_qposadr[joint_id])
    if address < 0 or address >= model.nq:
        raise ContractError("MODEL_MISMATCH", f"joint {name!r} has an invalid qpos address")
    return address


def _joint_dof(model: mujoco.MjModel, name: str, joint_id: int) -> int:
    address = int(model.jnt_dofadr[joint_id])
    if address < 0 or address >= model.nv:
        raise ContractError("MODEL_MISMATCH", f"joint {name!r} has an invalid dof address")
    return address


def _require_joint(model: mujoco.MjModel, name: str, joint_id: int, expected_type: int) -> None:
    if int(model.jnt_type[joint_id]) != int(expected_type):
        raise ContractError(
            "MODEL_MISMATCH",
            f"joint {name!r} has type {int(model.jnt_type[joint_id])}, "
            f"expected {int(expected_type)}",
        )


def _require_joint_range(
    model: mujoco.MjModel, name: str, joint_id: int, expected: JointSpec | _Range
) -> None:
    if not int(model.jnt_limited[joint_id]):
        raise ContractError("MODEL_MISMATCH", f"joint {name!r} is not limited")
    lower, upper = (float(value) for value in model.jnt_range[joint_id])
    if not _close(lower, expected.minimum) or not _close(upper, expected.maximum):
        raise ContractError(
            "MODEL_MISMATCH",
            f"joint {name!r} range [{lower}, {upper}] does not match "
            f"profile/scene [{expected.minimum}, {expected.maximum}]",
        )


def _actuator_id(model: mujoco.MjModel, name: str) -> int:
    try:
        return int(model.actuator(name).id)
    except KeyError as error:
        raise ContractError("MODEL_MISMATCH", f"actuator {name!r} is missing") from error


def _tendon_id(model: mujoco.MjModel, name: str) -> int:
    try:
        return int(model.tendon(name).id)
    except KeyError as error:
        raise ContractError("MODEL_MISMATCH", f"tendon {name!r} is missing") from error


def _require_actuator(
    model: mujoco.MjModel,
    name: str,
    actuator_id: int,
    expected_type: int,
    expected_target_id: int,
    minimum: float,
    maximum: float,
) -> None:
    if not int(model.actuator_ctrllimited[actuator_id]):
        raise ContractError("MODEL_MISMATCH", f"actuator {name!r} is not ctrl-limited")
    if int(model.actuator_trntype[actuator_id]) != int(expected_type):
        raise ContractError(
            "MODEL_MISMATCH",
            f"actuator {name!r} transmission type does not match the expected element",
        )
    if int(model.actuator_trnid[actuator_id, 0]) != expected_target_id:
        raise ContractError(
            "MODEL_MISMATCH",
            f"actuator {name!r} does not drive the expected joint/tendon",
        )
    if any(
        not _close(actual, expected)
        for actual, expected in zip(
            model.actuator_gear[actuator_id], (1.0, 0.0, 0.0, 0.0, 0.0, 0.0), strict=True
        )
    ):
        raise ContractError("MODEL_MISMATCH", f"actuator {name!r}: unit transmission gear required")
    lower, upper = (float(value) for value in model.actuator_ctrlrange[actuator_id])
    if not _close(lower, minimum) or not _close(upper, maximum):
        raise ContractError(
            "MODEL_MISMATCH",
            f"actuator {name!r} ctrlrange [{lower}, {upper}] does not match [{minimum}, {maximum}]",
        )


def _require_scene_indices(
    scene: Scene,
    arm_qpos: dict[str, tuple[int, ...]],
    finger_qpos: dict[str, tuple[int, int]],
    actuator_ids: list[int],
) -> None:
    indices = scene.indices
    expected_actuators = {
        "left": tuple(actuator_ids[0:7]),
        "right": tuple(actuator_ids[8:15]),
    }
    expected_grippers = {"left": actuator_ids[7], "right": actuator_ids[15]}
    for side in SIDES:
        try:
            actual_arm_qpos = tuple(indices.arm_joint_qpos[side])
            actual_finger_qpos = tuple(indices.finger_joint_qpos[side])
            actual_arm_actuators = tuple(indices.arm_actuator_ids[side])
            actual_gripper = int(indices.gripper_actuator_ids[side])
        except (AttributeError, KeyError, TypeError) as error:
            raise ContractError(
                "MODEL_MISMATCH",
                f"scene indices for {side} are incomplete",
            ) from error
        if actual_arm_qpos != arm_qpos[side]:
            raise ContractError(
                "MODEL_MISMATCH",
                f"scene arm qpos indices for {side} do not match the model names",
            )
        if actual_finger_qpos != finger_qpos[side]:
            raise ContractError(
                "MODEL_MISMATCH",
                f"scene finger qpos indices for {side} do not match the model names",
            )
        if actual_arm_actuators != expected_actuators[side]:
            raise ContractError(
                "MODEL_MISMATCH",
                f"scene arm actuator indices for {side} do not match the model names",
            )
        if actual_gripper != expected_grippers[side]:
            raise ContractError(
                "MODEL_MISMATCH",
                f"scene gripper actuator index for {side} does not match the model name",
            )


def _validate_targets(
    positions: Sequence[float], joints: tuple[JointSpec, ...]
) -> tuple[float, ...]:
    if isinstance(positions, str | bytes | bytearray):
        raise ContractError("INVALID_TARGET", "positions must be a sequence of 16 numbers")
    try:
        values = tuple(positions)
    except TypeError as error:
        raise ContractError(
            "INVALID_TARGET", "positions must be a sequence of 16 numbers"
        ) from error
    if len(values) != len(joints):
        raise ContractError(
            "INVALID_TARGET",
            f"positions: expected 16 entries, got {len(values)}",
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


def _sim_time_ns(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, numbers.Real):
        raise ContractError("INVALID_MEASUREMENT", "simulation time must be a real number")
    time_s = float(value)
    if not math.isfinite(time_s) or time_s < 0.0 or time_s > _MAX_TIME_S:
        raise ContractError(
            "INVALID_MEASUREMENT",
            f"simulation time {time_s!r} is not a finite non-negative time",
        )
    return round(time_s * 1e9)


def _require_finite(name: str, field: str, value: float) -> None:
    if not math.isfinite(value):
        raise ContractError("INVALID_MEASUREMENT", f"{name}.{field} is not finite")
