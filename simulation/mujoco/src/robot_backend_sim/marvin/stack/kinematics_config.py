"""Strict planning/kinematics configuration for the Marvin stack task."""

from __future__ import annotations

from pathlib import Path
from typing import Annotated

from pydantic import Field, model_validator
from robot_contracts.validation import (
    PositiveFloat,
    StrictFloat,
    StrictMappingError,
    StrictMappingModel,
    UniqueKeyError,
    contract_error,
    read_unique_yaml,
    validate_config_model,
)

from robot_backend_sim.common.tasks.config import (
    ClearanceSettings as ClearanceConfig,
    IKSettingsBase,
)
from robot_backend_sim.marvin.robot_config import MARVIN_ARM_JOINT_RANGES, Seven

__all__ = [
    "ClearanceConfig",
    "IKConfig",
    "KinematicsCheckConfig",
    "KinematicsConfigError",
    "MARVIN_ARM_JOINT_RANGES",
    "STACK_KEY_POSES",
    "TargetConfig",
    "load_kinematics_config",
]

KinematicsConfigError = StrictMappingError

STACK_KEY_POSES = (
    "pregrasp",
    "grasp",
    "lift",
    "place_hover",
    "place",
    "retreat",
)


class IKConfig(IKSettingsBase):
    initial_right_joint_positions: Seven

    @model_validator(mode="after")
    def _limits(self) -> IKConfig:
        for index, (value, (lower, upper)) in enumerate(
            zip(self.initial_right_joint_positions, MARVIN_ARM_JOINT_RANGES, strict=True),
            start=1,
        ):
            margin = self.limit_margin_rad
            if not (lower + margin <= value <= upper - margin):
                raise contract_error(
                    f"initial_right_joint_positions[{index - 1}]",
                    f"must stay within [{lower + margin:.4f}, {upper - margin:.4f}]",
                )
        return self


class TargetConfig(StrictMappingModel):
    pregrasp_height_m: Annotated[PositiveFloat, Field(gt=0.01, le=0.5)]
    grasp_height_m: Annotated[PositiveFloat, Field(gt=0.001, le=0.1)]
    lift_height_m: Annotated[PositiveFloat, Field(gt=0.02, le=0.5)]
    place_hover_height_m: Annotated[PositiveFloat, Field(gt=0.02, le=0.5)]
    place_clearance_m: Annotated[PositiveFloat, Field(gt=0.001, le=0.1)]
    retreat_height_m: Annotated[PositiveFloat, Field(gt=0.02, le=0.5)]
    approach_tilt_deg: Annotated[StrictFloat, Field(ge=-60.0, le=60.0)] = 0.0
    approach_yaw_deg: Annotated[StrictFloat, Field(ge=-180.0, le=180.0)] = 0.0
    place_yaw_deg: Annotated[StrictFloat, Field(ge=-180.0, le=180.0)] = 0.0


class KinematicsCheckConfig(StrictMappingModel):
    source: str = ""
    ik: IKConfig
    clearance: ClearanceConfig
    targets: TargetConfig


def load_kinematics_config(
    path: Path | str, scene_config: object = None
) -> KinematicsCheckConfig:
    del scene_config
    source_path = Path(path).resolve()
    try:
        document = read_unique_yaml(source_path)
    except OSError as error:
        raise KinematicsConfigError(source_path, "<file>", str(error)) from error
    except UniqueKeyError as error:
        raise KinematicsConfigError(error.source, error.field, error.message) from error
    if not isinstance(document, dict):
        raise KinematicsConfigError(source_path, "<root>", "must be a mapping")
    return validate_config_model(
        KinematicsCheckConfig,
        {**document, "source": str(source_path)},
        source_path,
        KinematicsConfigError,
    )
