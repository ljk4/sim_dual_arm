"""Strict immutable configuration for the native motion core."""

from __future__ import annotations

from pathlib import Path
from typing import Annotated

from pydantic import BeforeValidator, Field, model_validator
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
    "PandaControlConfig",
    "MotionConfigError",
    "load_control_config",
    "parse_control_config",
]

# Kept local so loading the pure configuration does not import MuJoCo.  The
# order is exactly PandaIO.JOINT_NAMES and the accepted panda_stack_v1 profile.
CONTROL_JOINT_NAMES = tuple(
    f"{side}_{name}" for side in ("left", "right")
    for name in (*[f"joint{index}" for index in range(1, 8)], "gripper_width"))
_JOINT_COUNT = len(CONTROL_JOINT_NAMES)
_MAX_NS = 2**63 - 1


MotionConfigError = StrictMappingError


class PandaControlConfig(StrictMappingModel):
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
    def _fixed_order(self) -> PandaControlConfig:
        if self.joint_names != CONTROL_JOINT_NAMES:
            raise contract_error("joint_names", "must match the fixed 16-joint order")
        for field in ("stable_duration_s", "settle_timeout_s",
                      "motion_wall_timeout_s", "stop_wall_timeout_s"):
            value = _nanoseconds(getattr(self, field))
            if value <= 0 or value > _MAX_NS:
                raise contract_error(field, "must convert to a positive int64 nanosecond duration")
        return self

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
) -> PandaControlConfig:
    return validate_config_model(PandaControlConfig, document, source, MotionConfigError)


def load_control_config(path: Path | str) -> PandaControlConfig:
    source = Path(path)
    try:
        document = read_unique_yaml(source)
    except UniqueKeyError as error:
        raise MotionConfigError(error.source, error.field, error.message) from error
    return parse_control_config(document, source)
