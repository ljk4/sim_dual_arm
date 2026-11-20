from __future__ import annotations

from collections.abc import Sequence
import re
from typing import Annotated

from pydantic import BeforeValidator, Field, model_validator

from .validation import (
    ContractError,
    NonEmptyStr,
    NonNegativeInt,
    PositiveInt,
    StrictContractModel,
    StrictFloat,
    as_tuple,
    contract_error,
)

__all__ = [
    "CameraSpec",
    "ContractError",
    "JointSpec",
    "RobotProfile",
    "RunContext",
    "Target",
    "coordinated_axis_groups",
]
MAX_JOINTS, MAX_CAMERAS = 64, 8
_SAFE_CAMERA_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]{0,63}$")
FloatVector = Annotated[tuple[StrictFloat, ...], BeforeValidator(as_tuple),
                        Field(min_length=1, max_length=MAX_JOINTS)]
CameraDigest = Annotated[str, Field(strict=True, pattern=r"^[0-9a-f]{64}$")]


class RunContext(StrictContractModel):
    robot_id: NonEmptyStr
    robot_profile_hash: NonEmptyStr
    backend_instance_id: NonEmptyStr
    time_domain_id: NonEmptyStr
    clock_epoch: Annotated[NonNegativeInt, Field(le=2**63 - 1)]
    episode_id: str = ""
    control_epoch: Annotated[NonNegativeInt, Field(le=2**63 - 1)] = 0

    generation = property(lambda self: (self.backend_instance_id, self.clock_epoch))


class JointSpec(StrictContractModel):
    name: NonEmptyStr
    unit: NonEmptyStr
    minimum: StrictFloat = Field(validation_alias="min")
    maximum: StrictFloat = Field(validation_alias="max")
    max_velocity: StrictFloat
    initial: StrictFloat

    @model_validator(mode="after")
    def _limits(self) -> JointSpec:
        if self.minimum >= self.maximum:
            raise contract_error("minimum", "must be less than maximum")
        if self.max_velocity <= 0.0:
            raise contract_error("max_velocity", "must be positive")
        if not self.minimum <= self.initial <= self.maximum:
            raise contract_error("initial", "must lie inside the joint range")
        return self


class CameraSpec(StrictContractModel):
    name: NonEmptyStr
    width: Annotated[PositiveInt, Field(le=640)]
    height: Annotated[PositiveInt, Field(le=480)]
    frame_id: NonEmptyStr
    calibration_digest: CameraDigest


class RobotProfile(StrictContractModel):
    robot_id: NonEmptyStr
    profile_id: NonEmptyStr
    digest: str = ""
    test_only: bool
    joints: Annotated[tuple[JointSpec, ...], BeforeValidator(as_tuple),
                      Field(min_length=1, max_length=MAX_JOINTS)]
    cameras: Annotated[tuple[CameraSpec, ...], BeforeValidator(as_tuple),
                       Field(min_length=1, max_length=MAX_CAMERAS)]

    @model_validator(mode="after")
    def _profile_identity(self) -> RobotProfile:
        if len({joint.name for joint in self.joints}) != len(self.joints):
            raise contract_error("joints", "duplicate joint names")
        names = [camera.name for camera in self.cameras]
        if len(set(names)) != len(names):
            raise contract_error("cameras", "duplicate camera names")
        frames = [camera.frame_id for camera in self.cameras]
        if len(set(frames)) != len(frames):
            raise contract_error("cameras", "duplicate camera frame_id values")
        for camera in self.cameras:
            if not _SAFE_CAMERA_NAME.fullmatch(camera.name):
                raise contract_error(f"cameras.{camera.name}.name", "invalid ROS topic basename")
        return self

    joint_names = property(lambda self: tuple(joint.name for joint in self.joints))
    camera_names = property(lambda self: tuple(camera.name for camera in self.cameras))
    initial_positions = property(lambda self: tuple(joint.initial for joint in self.joints))

    @property
    def coordinated_axis_groups(self) -> tuple[tuple[int, ...], ...]:
        """Group revolute manipulator joints by side prefix; linear axes stay independent."""
        return coordinated_axis_groups(self.joints)


def coordinated_axis_groups(joints: Sequence[JointSpec]) -> tuple[tuple[int, ...], ...]:
    """Derive per-limb revolute joint index groups from canonical ``<prefix>_joint*`` names."""
    groups: dict[str, list[int]] = {}
    for index, joint in enumerate(joints):
        if joint.unit != "rad":
            continue
        prefix, sep, _ = joint.name.partition("_joint")
        if sep and prefix:
            groups.setdefault(prefix, []).append(index)
    return tuple(tuple(indices) for indices in groups.values() if len(indices) > 1)


class Target(StrictContractModel):
    time_ns: PositiveInt
    positions: FloatVector
