"""Strict immutable configuration for bounded target following."""

from __future__ import annotations

from pathlib import Path
from typing import Annotated

from pydantic import BeforeValidator, Field, model_validator
from robot_contracts.action_chunks import MAX_TARGET_COUNT
from robot_contracts.trajectory import MAX_HORIZON_NS
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

from .motion_config import CONTROL_JOINT_NAMES

__all__ = [
    "FollowerConfigError",
    "PandaFollowerConfig",
    "load_follower_config",
    "parse_follower_config",
]

_JOINT_COUNT = len(CONTROL_JOINT_NAMES)


FollowerConfigError = StrictMappingError


class PandaFollowerConfig(StrictMappingModel):
    """Validated bounded-follower selection, tail and deviation parameters."""


    joint_names: Annotated[tuple[NonEmptyStr, ...], BeforeValidator(as_tuple),
                           Field(min_length=_JOINT_COUNT, max_length=_JOINT_COUNT)]
    target_count: Annotated[int, Field(strict=True, ge=2, le=MAX_TARGET_COUNT)]
    max_target_deviations: Annotated[tuple[PositiveFloat, ...], BeforeValidator(as_tuple),
                                     Field(min_length=_JOINT_COUNT, max_length=_JOINT_COUNT)]
    max_terminal_tail_duration_s: PositiveFloat

    @model_validator(mode="after")
    def _fixed_order(self) -> PandaFollowerConfig:
        if self.joint_names != CONTROL_JOINT_NAMES:
            raise contract_error("joint_names", "must match the fixed 16-joint order")
        if not 1 <= round(self.max_terminal_tail_duration_s * 1_000_000_000) <= MAX_HORIZON_NS:
            raise contract_error("max_terminal_tail_duration_s",
                                 f"must convert to 1..{MAX_HORIZON_NS} ns")
        return self

    @property
    def max_terminal_tail_duration_ns(self) -> int:
        return round(self.max_terminal_tail_duration_s * 1_000_000_000)

    def to_document(self) -> dict[str, object]:
        return {
            "joint_names": list(self.joint_names),
            "target_count": self.target_count,
            "max_target_deviations": list(self.max_target_deviations),
            "max_terminal_tail_duration_s": self.max_terminal_tail_duration_s,
        }


def parse_follower_config(
    document: object, source: str | Path = "<follower config>"
) -> PandaFollowerConfig:
    """Validate one mapping and return an immutable follower configuration."""
    return validate_config_model(PandaFollowerConfig, document, source, FollowerConfigError)


def load_follower_config(path: Path | str) -> PandaFollowerConfig:
    """Load one YAML follower file, rejecting duplicate/unknown/bool/non-finite data."""
    source = Path(path)
    try:
        document = read_unique_yaml(source)
    except UniqueKeyError as error:
        raise FollowerConfigError(error.source, error.field, error.message) from error
    return parse_follower_config(document, source)
