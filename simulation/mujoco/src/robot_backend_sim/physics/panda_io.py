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
import threading
from collections.abc import Mapping, Sequence
from typing import NamedTuple

import mujoco
from robot_contracts.models import ContractError, JointSpec, RobotProfile

from robot_backend_sim.config.scene_config import (
    FINGER_JOINT_LIMIT,
    PANDA_JOINT_RANGES,
    SceneConfig,
)
from robot_backend_sim.physics.robot_io import (
    PUBLIC_JOINT_NAMES,
    Measurement,
    RobotIO,
    check_owner_thread,
    close,
    gripper_indices,
    require_finite,
    sim_time_ns,
    validate_camera_contract,
    validate_profile_contract,
    validate_targets,
)
from robot_backend_sim.physics.scene import (
    FINGER_OPENING_RANGE,
    GRIPPER_CTRL_RANGE,
    Scene,
    gripper_ctrl,
)

__all__ = ["JOINT_NAMES", "Measurement", "PandaIO"]

SIDES = ("left", "right")
JOINT_NAMES = PUBLIC_JOINT_NAMES
_ARM_COUNT = 7


class PandaIO(RobotIO):
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
        self._camera_frame_ids = validate_camera_contract(profile, scene)
        self._binding = _resolve_binding(scene, self._joint_specs)
        self.velocity_feedforward = velocity_feedforward
        self._gripper_axes = frozenset(gripper_indices(JOINT_NAMES).values())
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

    @property
    def finger_bodies(self) -> Mapping[str, tuple[tuple[str, str], ...]]:
        return self._scene.robot_geometry.finger_bodies

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
                require_finite(name, "position", position)
                require_finite(name, "velocity", velocity)
                positions.append(position)
                velocities.append(velocity)
            finger_positions = self._binding.finger_qpos[side]
            finger_dofs = self._binding.finger_dof[side]
            total_position = float(data.qpos[finger_positions[0]] + data.qpos[finger_positions[1]])
            total_velocity = float(data.qvel[finger_dofs[0]] + data.qvel[finger_dofs[1]])
            gripper_name = f"{side}_gripper_width"
            require_finite(gripper_name, "position", total_position)
            require_finite(gripper_name, "velocity", total_velocity)
            positions.append(total_position)
            velocities.append(total_velocity)
        return Measurement(
            sim_time_ns=sim_time_ns(data.time),
            joint_names=JOINT_NAMES,
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
        values = validate_targets(positions, self._joint_specs)
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
            length_per_si = 0.5 if axis in self._gripper_axes else 1.0
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
        values = validate_targets(positions, self._joint_specs)
        if len(velocities) != len(values):
            raise ContractError("INVALID_TARGET", "reference velocity dimension differs")
        raw = []
        for axis, (position, velocity, spec, gain) in enumerate(
            zip(values, velocities, self._joint_specs, self._velocity_gains, strict=True)
        ):
            require_finite(spec.name, "reference_velocity", velocity)
            if abs(velocity) > spec.max_velocity + 1e-9:
                raise ContractError("INVALID_TARGET", "reference velocity exceeds hard limit")
            base = gripper_ctrl(position) if axis in self._gripper_axes else position
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
        check_owner_thread(self._owner_thread, "PandaIO")


class _Binding(NamedTuple):
    arm_qpos: dict[str, tuple[int, ...]]
    arm_dof: dict[str, tuple[int, ...]]
    finger_qpos: dict[str, tuple[int, int]]
    finger_dof: dict[str, tuple[int, int]]
    actuator_ids: tuple[int, ...]


def _side_arm_names(side: str) -> tuple[str, ...]:
    return tuple(f"{side}_joint{index}" for index in range(1, 8))


def _validate_profile(profile: RobotProfile, config: SceneConfig) -> tuple[JointSpec, ...]:
    joints = validate_profile_contract(profile)
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
        if not close(joint.minimum, lower) or not close(joint.maximum, upper):
            raise ContractError(
                "PROFILE_MISMATCH",
                f"joints[{index}] ({joint.name}).range: expected [{lower}, {upper}], "
                f"got [{joint.minimum}, {joint.maximum}]",
            )
        if not close(joint.initial, initial):
            raise ContractError(
                "PROFILE_MISMATCH",
                f"joints[{index}] ({joint.name}).initial: expected {initial}, got {joint.initial}",
            )
    return joints


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
    if not close(lower, expected.minimum) or not close(upper, expected.maximum):
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
        not close(actual, expected)
        for actual, expected in zip(
            model.actuator_gear[actuator_id], (1.0, 0.0, 0.0, 0.0, 0.0, 0.0), strict=True
        )
    ):
        raise ContractError("MODEL_MISMATCH", f"actuator {name!r}: unit transmission gear required")
    lower, upper = (float(value) for value in model.actuator_ctrlrange[actuator_id])
    if not close(lower, minimum) or not close(upper, maximum):
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
