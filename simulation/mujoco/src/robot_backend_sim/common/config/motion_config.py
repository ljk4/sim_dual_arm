"""Strict immutable configuration for the motion core."""

from __future__ import annotations

from pathlib import Path
from typing import Annotated

from pydantic import BeforeValidator, Field, model_validator
from robot_contracts.models import ContractError
from robot_contracts.trajectory.constants import MAX_JOINTS
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
    "ControlConfig",
    "MotionConfigError",
    "load_control_config",
    "parse_control_config",
]

_MAX_NS = 2**63 - 1


MotionConfigError = StrictMappingError


class ControlConfig(StrictMappingModel):
    """Validated controller limits and deadlines in explicit SI units."""

    joint_names: Annotated[
        tuple[NonEmptyStr, ...],
        BeforeValidator(as_tuple),
        Field(min_length=1, max_length=MAX_JOINTS),
    ]
    max_accelerations: Annotated[
        tuple[PositiveFloat, ...],
        BeforeValidator(as_tuple),
        Field(min_length=1, max_length=MAX_JOINTS),
    ]
    position_tolerances: Annotated[
        tuple[PositiveFloat, ...],
        BeforeValidator(as_tuple),
        Field(min_length=1, max_length=MAX_JOINTS),
    ]
    velocity_tolerances: Annotated[
        tuple[PositiveFloat, ...],
        BeforeValidator(as_tuple),
        Field(min_length=1, max_length=MAX_JOINTS),
    ]
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
        joint_count = len(self.joint_names)
        for field in ("max_accelerations", "position_tolerances", "velocity_tolerances"):
            if len(getattr(self, field)) != joint_count:
                raise contract_error(field, "must match joint_names length")
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
