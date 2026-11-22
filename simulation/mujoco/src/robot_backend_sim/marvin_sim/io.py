"""Bind the Marvin handover profile to the compiled scene by element name.

The public order is left seven arm joints, left total opening, right seven arm
joints, right total opening.  Physical qpos starts with the right arm, so this
module reads and writes named qpos/dof/actuator addresses from
:class:`MarvinBinding` rather than any adjacent slice.

Total opening is measured as the perpendicular gap between the two inner finger
faces.  The actuator command is the tendon closure, which runs in the opposite
direction: ``closure = open_total - opening``.  Gripper velocity is therefore
the negative sum of the two finger-joint velocities so that positive still means
opening.  Velocity feedforward converts the desired opening rate to the servo's
closure command with the same negative sign.
"""

from __future__ import annotations

import math
import threading
from collections.abc import Mapping, Sequence

from robot_contracts.models import ContractError, JointSpec, RobotProfile

from robot_backend_sim.common.physics.robot_io import (
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
from robot_backend_sim.marvin_sim.assembly import MarvinBinding, gripper_opening
from robot_backend_sim.marvin_sim.robot_config import MARVIN_CONTROL_JOINT_NAMES
from robot_backend_sim.marvin_sim.scene_base import MarvinSceneBase

__all__ = ["MarvinIO", "JOINT_NAMES"]

SIDES = ("left", "right")
JOINT_NAMES = MARVIN_CONTROL_JOINT_NAMES
_ARM_COUNT = 7


class MarvinIO(RobotIO):
    """Validated named I/O for one compiled Marvin scene."""

    def __init__(
        self,
        scene: MarvinSceneBase,
        profile: RobotProfile,
        *,
        velocity_feedforward: bool = False,
    ) -> None:
        self._owner_thread = threading.current_thread()
        self._scene = scene
        self._profile = profile
        self._joint_specs = validate_profile_contract(profile, JOINT_NAMES)
        self._camera_frame_ids = validate_camera_contract(profile, scene)
        self._binding = scene.indices.robot
        _validate_scene_contract(scene, self._joint_specs)
        self._actuator_ids = _ordered_actuators(self._binding)
        self._gripper_axes = frozenset(gripper_indices(JOINT_NAMES).values())
        self.velocity_feedforward = velocity_feedforward
        self._velocity_gains = self._servo_velocity_gains() if velocity_feedforward else ()

    @property
    def joint_names(self) -> tuple[str, ...]:
        return JOINT_NAMES

    @property
    def camera_names(self) -> tuple[str, ...]:
        return tuple(camera.name for camera in self._profile.cameras)

    @property
    def camera_frame_ids(self) -> tuple[str, ...]:
        return self._camera_frame_ids

    @property
    def profile(self) -> RobotProfile:
        return self._profile

    @property
    def finger_bodies(self) -> Mapping[str, tuple[tuple[str, str], ...]]:
        return self._scene.robot_geometry.finger_bodies

    def require_scene(self, scene: MarvinSceneBase) -> None:
        self._check_thread()
        if scene is not self._scene:
            raise ContractError(
                "SCENE_MISMATCH",
                "MarvinIO belongs to a different Scene than the caller supplied",
            )

    def measure(self) -> Measurement:
        self._check_thread()
        model = self._scene.model
        data = self._scene.data
        positions: list[float] = []
        velocities: list[float] = []
        for side in SIDES:
            for index, (qpos_address, dof_address) in enumerate(
                zip(
                    self._binding.arm_joint_qpos[side],
                    self._binding.arm_joint_dof[side],
                    strict=True,
                ),
                start=1,
            ):
                name = f"{side}_joint{index}"
                position = float(data.qpos[qpos_address])
                velocity = float(data.qvel[dof_address])
                require_finite(name, "position", position)
                require_finite(name, "velocity", velocity)
                positions.append(position)
                velocities.append(velocity)
            opening = float(gripper_opening(model, data, self._binding, side))
            finger_dofs = self._binding.finger_joint_dof[side]
            opening_velocity = -float(data.qvel[finger_dofs[0]] + data.qvel[finger_dofs[1]])
            gripper_name = f"{side}_gripper_width"
            require_finite(gripper_name, "position", opening)
            require_finite(gripper_name, "velocity", opening_velocity)
            positions.append(opening)
            velocities.append(opening_velocity)
        return Measurement(
            sim_time_ns=sim_time_ns(data.time),
            joint_names=JOINT_NAMES,
            positions=tuple(positions),
            velocities=tuple(velocities),
        )

    def apply_targets(self, positions: Sequence[float]) -> None:
        """Validate all sixteen SI targets, then write the named ctrl entries.

        Arm targets are radians.  Gripper targets are total opening in metres
        and are converted to the tendon closure command continuously.  Nothing
        is clipped; a rejected target leaves every ctrl entry untouched.
        """
        self._check_thread()
        values = validate_targets(positions, self._joint_specs)
        raw = self._raw_commands(values)
        ctrl = self._scene.data.ctrl
        for actuator_id, value in zip(self._actuator_ids, raw, strict=True):
            ctrl[actuator_id] = value

    def _raw_commands(self, values: tuple[float, ...]) -> list[float]:
        raw: list[float] = []
        for side in SIDES:
            offset = 8 if side == "right" else 0
            raw.extend(values[offset : offset + _ARM_COUNT])
            raw.append(self._scene.gripper_ctrl(values[offset + _ARM_COUNT]))
        return raw

    def _servo_velocity_gains(self) -> tuple[float, ...]:
        """Convert SI reference velocity to the affine PD ctrl units.

        Arm ctrl is joint position, so the offset is ``kd/kp * qdot``.  Gripper
        ctrl is closure and the tendon length is the closure in metres, so the
        closure rate is the negative opening rate and the offset keeps that sign.
        Coupled-finger slide damping is projected into the tendon coordinate.
        """
        gains = []
        model = self._scene.model
        for axis, actuator in enumerate(self._actuator_ids):
            gain = float(model.actuator_gainprm[actuator, 0])
            damping = -float(model.actuator_biasprm[actuator, 2])
            if axis in self._gripper_axes:
                side = "left" if axis == 7 else "right"
                # The coupled fingers each travel half the tendon closure.
                damping += sum(
                    float(model.dof_damping[dof])
                    for dof in self._binding.finger_joint_dof[side]
                ) / 4.0
            if not math.isfinite(gain) or gain <= 0 or not math.isfinite(damping) or damping < 0:
                raise ContractError("MODEL_MISMATCH", "invalid affine Marvin servo gain")
            length_per_si = -1.0 if axis in self._gripper_axes else 1.0
            gains.append(damping / gain * length_per_si)
        return tuple(gains)

    def apply_reference(self, positions: Sequence[float], velocities: Sequence[float]) -> None:
        """Track one unchanged position/velocity reference through affine PD.

        Disabled mode is exactly the position-only I/O.  Enabled mode adds the
        converted reference velocity, with actuator ctrl ranges still applied.
        """
        if not self.velocity_feedforward:
            self.apply_targets(positions)
            return
        self._check_thread()
        values = validate_targets(positions, self._joint_specs)
        if len(velocities) != len(values):
            raise ContractError("INVALID_TARGET", "reference velocity dimension differs")
        raw = []
        for axis, (value, velocity, spec, gain) in enumerate(
            zip(values, velocities, self._joint_specs, self._velocity_gains, strict=True)
        ):
            require_finite(spec.name, "reference_velocity", velocity)
            if abs(velocity) > spec.max_velocity + 1e-9:
                raise ContractError("INVALID_TARGET", "reference velocity exceeds hard limit")
            base = (
                self._scene.gripper_ctrl(value)
                if axis in self._gripper_axes
                else value
            )
            command = base + gain * velocity
            actuator = self._actuator_ids[axis]
            if self._scene.model.actuator_ctrllimited[actuator]:
                lower, upper = self._scene.model.actuator_ctrlrange[actuator]
                command = max(float(lower), min(float(upper), command))
            raw.append(command)
        for actuator, command in zip(self._actuator_ids, raw, strict=True):
            self._scene.data.ctrl[actuator] = command

    def _check_thread(self) -> None:
        check_owner_thread(self._owner_thread, "MarvinIO")


def _ordered_actuators(binding: MarvinBinding) -> tuple[int, ...]:
    ids: list[int] = []
    for side in SIDES:
        ids.extend(binding.arm_actuator_ids[side])
        ids.append(binding.gripper_actuator_ids[side])
    if len(ids) != len(JOINT_NAMES) or len(set(ids)) != len(ids):
        raise ContractError("MODEL_MISMATCH", "the sixteen actuator ids are not unique")
    return tuple(ids)


def _validate_scene_contract(scene: MarvinSceneBase, joints: tuple[JointSpec, ...]) -> None:
    """Check profile limits, initial values and commands against the compiled model."""
    model = scene.model
    data = scene.data
    config = scene.config.robot
    binding = scene.indices.robot
    if model.nu != len(JOINT_NAMES):
        raise ContractError(
            "MODEL_MISMATCH", f"model has {model.nu} actuators, expected {len(JOINT_NAMES)}"
        )
    if len(data.qpos) != model.nq or len(data.qvel) != model.nv or len(data.ctrl) != model.nu:
        raise ContractError("MODEL_MISMATCH", "scene data dimensions do not match the model")
    if not close(config.gripper_open_total - config.gripper_closed_total, 0.08):
        raise ContractError(
            "MODEL_MISMATCH", "gripper opening range does not equal the tendon closure range"
        )
    spec_by_name = {joint.name: joint for joint in joints}
    for side in SIDES:
        for index in range(1, _ARM_COUNT + 1):
            public = f"{side}_joint{index}"
            expected = spec_by_name[public]
            source = config.home_joint_positions[index - 1]
            joint = model.joint(f"Arm_{'L' if side == 'left' else 'R'}{index}_Joint")
            lower, upper = (float(value) for value in model.jnt_range[joint.id])
            if not close(lower, expected.minimum) or not close(upper, expected.maximum):
                raise ContractError(
                    "PROFILE_MISMATCH",
                    f"{public} profile range [{expected.minimum}, {expected.maximum}] "
                    f"differs from the compiled range [{lower}, {upper}]",
                )
            if not close(expected.initial, source):
                raise ContractError(
                    "PROFILE_MISMATCH",
                    f"{public} initial {expected.initial} differs from the scene home {source}",
                )
            actuator = binding.arm_actuator_ids[side][index - 1]
            if int(model.actuator_trnid[actuator, 0]) != int(joint.id):
                raise ContractError(
                    "MODEL_MISMATCH", f"{public} actuator drives a different joint"
                )
            if not int(model.actuator_ctrllimited[actuator]):
                raise ContractError("MODEL_MISMATCH", f"{public} actuator is not ctrl-limited")
            ctrl_lower, ctrl_upper = (
                float(value) for value in model.actuator_ctrlrange[actuator]
            )
            if not close(ctrl_lower, expected.minimum) or not close(ctrl_upper, expected.maximum):
                raise ContractError(
                    "MODEL_MISMATCH",
                    f"{public} actuator ctrlrange [{ctrl_lower}, {ctrl_upper}] "
                    f"differs from the profile range",
                )
        gripper = spec_by_name[f"{side}_gripper_width"]
        if not close(gripper.minimum, config.gripper_closed_total) or not close(
            gripper.maximum, config.gripper_open_total
        ):
            raise ContractError(
                "PROFILE_MISMATCH",
                f"{side}_gripper_width profile range "
                f"[{gripper.minimum}, {gripper.maximum}] differs from the scene "
                f"[{config.gripper_closed_total}, {config.gripper_open_total}]",
            )
        if not close(gripper.initial, config.gripper_open_total):
            raise ContractError(
                "PROFILE_MISMATCH",
                f"{side}_gripper_width initial {gripper.initial} is not the open total",
            )
        actuator = binding.gripper_actuator_ids[side]
        ctrl_lower, ctrl_upper = (float(value) for value in model.actuator_ctrlrange[actuator])
        if not close(ctrl_lower, 0.0) or not close(
            ctrl_upper, config.gripper_open_total - config.gripper_closed_total
        ):
            raise ContractError(
                "MODEL_MISMATCH",
                f"{side} gripper actuator ctrlrange differs from the closure range",
            )
