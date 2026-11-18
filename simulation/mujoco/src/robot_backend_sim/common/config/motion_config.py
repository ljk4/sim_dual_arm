"""Strict immutable configuration for the motion core."""

from __future__ import annotations

from pathlib import Path
from typing import Annotated

from pydantic import BeforeValidator, Field, model_validator
from robot_contracts.models import ContractError
from robot_contracts.validation import (
    NonEmptyStr,
    PositiveFloat,
    StrictMappingError,
    StrictMappingModel,
    UniqueKeyError,
    as_tuple,
    contract_error,
    read_unique_yaml,
    validate_config_model,
)

__all__ = [
    "CONTROL_JOINT_NAMES",
    "ControlConfig",
    "MotionConfigError",
    "load_control_config",
    "parse_control_config",
]

# Canonical public order shared by the accepted robot profiles: left arm,
# left gripper, right arm, right gripper.  A concrete robot binds its profile to
# this order at controller construction; the YAML joint_names field must match
# that profile rather than a robot-family module constant.
CONTROL_JOINT_NAMES = tuple(
    f"{side}_{name}" for side in ("left", "right")
    for name in (*[f"joint{index}" for index in range(1, 8)], "gripper_width"))
_JOINT_COUNT = len(CONTROL_JOINT_NAMES)
_MAX_NS = 2**63 - 1


MotionConfigError = StrictMappingError


class ControlConfig(StrictMappingModel):
    """Validated controller limits and deadlines in explicit SI units."""


    joint_names: Annotated[tuple[NonEmptyStr, ...], BeforeValidator(as_tuple),
                           Field(min_length=_JOINT_COUNT, max_length=_JOINT_COUNT)]
    max_accelerations: Annotated[tuple[PositiveFloat, ...], BeforeValidator(as_tuple),
                                 Field(min_length=_JOINT_COUNT, max_length=_JOINT_COUNT)]
    position_tolerances: Annotated[tuple[PositiveFloat, ...], BeforeValidator(as_tuple),
                                   Field(min_length=_JOINT_COUNT, max_length=_JOINT_COUNT)]
    velocity_tolerances: Annotated[tuple[PositiveFloat, ...], BeforeValidator(as_tuple),
                                   Field(min_length=_JOINT_COUNT, max_length=_JOINT_COUNT)]
    stable_duration_s: PositiveFloat
    settle_timeout_s: PositiveFloat
    motion_wall_timeout_s: PositiveFloat
    stop_wall_timeout_s: PositiveFloat
    contact_force_min_n: PositiveFloat
    finger_velocity_max_m_s: PositiveFloat

    @model_validator(mode="after")
    def _shape(self) -> ControlConfig:
        if len(set(self.joint_names)) != len(self.joint_names):
            raise contract_error("joint_names", "must be unique")
        for field in ("stable_duration_s", "settle_timeout_s",
                      "motion_wall_timeout_s", "stop_wall_timeout_s"):
            value = _nanoseconds(getattr(self, field))
            if value <= 0 or value > _MAX_NS:
                raise contract_error(field, "must convert to a positive int64 nanosecond duration")
        return self

    def require_joint_order(self, joint_names: tuple[str, ...]) -> None:
        """Bind this recipe to one robot profile; never assume a family order."""
        if tuple(self.joint_names) != tuple(joint_names):
            raise ContractError(
                "PROFILE_MISMATCH",
                "control config joint order differs from the robot profile",
            )

    stable_duration_ns = property(lambda self: _nanoseconds(self.stable_duration_s))
    settle_timeout_ns = property(lambda self: _nanoseconds(self.settle_timeout_s))
    motion_wall_timeout_ns = property(lambda self: _nanoseconds(self.motion_wall_timeout_s))
    stop_wall_timeout_ns = property(lambda self: _nanoseconds(self.stop_wall_timeout_s))

    def to_document(self) -> dict[str, object]:
        return {key: list(value) if isinstance(value, tuple) else value
                for key, value in self.model_dump().items()}


def _nanoseconds(seconds: float) -> int:
    try:
        return round(seconds * 1_000_000_000)
    except (OverflowError, ValueError):
        return _MAX_NS + 1


def parse_control_config(
    document: object, source: str | Path = "<control config>"
) -> ControlConfig:
    return validate_config_model(ControlConfig, document, source, MotionConfigError)


def load_control_config(path: Path | str) -> ControlConfig:
    source = Path(path)
    try:
        document = read_unique_yaml(source)
    except UniqueKeyError as error:
        raise MotionConfigError(error.source, error.field, error.message) from error
    return parse_control_config(document, source)
