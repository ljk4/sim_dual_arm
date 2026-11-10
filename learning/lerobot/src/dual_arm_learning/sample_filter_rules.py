"""Strict Pydantic validation of the static-anchor filter rule document."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Annotated, Literal

from pydantic import BeforeValidator, Field, model_validator
from robot_contracts.validation import (
    NonEmptyStr,
    NonNegativeFloat,
    StrictMappingModel,
    as_tuple,
    contract_error,
)

from .config_yaml import ConfigError, load_yaml
from .dataset_common import file_sha256

LEAD_NS = 50_000_000
PERIOD_NS = 50_000_000
STATE_DIM = 16
REASON_REDUNDANT = "redundant_static_anchor"
RIGHT_ARM = (8, 9, 10, 11, 12, 13, 14)
GRIPPERS = (7, 15)
KNOWN_PHASES = (
    "pregrasp",
    "grasp",
    "close",
    "lift",
    "transport",
    "lower",
    "open",
    "retreat",
    "stable",
)

JointIndices = Annotated[tuple[Annotated[int, Field(strict=True, ge=0, le=15)], ...],
                         BeforeValidator(as_tuple)]
StepLimit = Annotated[NonNegativeFloat, Field(le=1.0)]
SpeedLimit = Annotated[NonNegativeFloat, Field(le=100.0)]
FrameCount = Annotated[int, Field(strict=True, ge=0, le=100_000)]


class FilterError(ValueError):
    """The filter rule or mask cannot be trusted."""


@dataclass(frozen=True, slots=True)
class FilterRule:
    schema_version: int
    fps: int
    chunk_size: int
    right_arm_indices: tuple[int, ...]
    gripper_indices: tuple[int, ...]
    max_right_step_rad: float
    max_right_velocity_rad_s: float
    max_gripper_step_m: float
    max_gripper_velocity_m_s: float
    min_static_run_frames: int
    keep_edge_frames: int
    interior_anchor_stride: int
    phase_boundary_frames: int
    episode_boundary_frames: int
    min_episode_kept_frames: int
    known_phases: tuple[str, ...]
    tracking_gate: str
    reason: str
    source: Path

    @property
    def sha256(self) -> str:
        return file_sha256(self.source)


class _ArmRule(StrictMappingModel):
    joint_indices: JointIndices
    max_step_rad: StepLimit
    max_velocity_rad_s: SpeedLimit

    @model_validator(mode="after")
    def _unique(self) -> _ArmRule:
        if len(self.joint_indices) != 7 or len(set(self.joint_indices)) != 7:
            raise contract_error("joint_indices", "must contain seven unique joint indices")
        if self.joint_indices != RIGHT_ARM:
            raise contract_error(
                "joint_indices", "must be the seven right arm rad axes"
            )
        return self


class _GripperRule(StrictMappingModel):
    joint_indices: JointIndices
    max_step_m: StepLimit
    max_velocity_m_s: SpeedLimit

    @model_validator(mode="after")
    def _unique(self) -> _GripperRule:
        if len(self.joint_indices) != 2 or len(set(self.joint_indices)) != 2:
            raise contract_error("joint_indices", "must contain two unique joint indices")
        if self.joint_indices != GRIPPERS:
            raise contract_error("joint_indices", "must be the left/right gripper metre axes")
        return self


class _StaticRuns(StrictMappingModel):
    min_frames: Annotated[int, Field(strict=True, ge=2, le=100_000)]
    edge_frames: FrameCount
    interior_stride: Annotated[int, Field(strict=True, ge=1, le=100_000)]
    phase_boundary_frames: FrameCount
    episode_boundary_frames: FrameCount
    min_episode_kept_frames: FrameCount
    reason: Literal["redundant_static_anchor"]


class _PhaseGate(StrictMappingModel):
    known_phases: Annotated[tuple[NonEmptyStr, ...], BeforeValidator(as_tuple)]
    tracking_gate: NonEmptyStr

    @model_validator(mode="after")
    def _phases(self) -> _PhaseGate:
        if len(set(self.known_phases)) != len(self.known_phases):
            raise contract_error("known_phases", "must be unique")
        if set(KNOWN_PHASES) - set(self.known_phases):
            raise contract_error("known_phases", "must contain every panda stack phase")
        return self


class _FilterDocument(StrictMappingModel):
    schema_version: Literal[1]
    fps: Literal[20]
    chunk_size: Literal[60]
    right_arm: _ArmRule
    grippers: _GripperRule
    static_runs: _StaticRuns
    phase_gate: _PhaseGate

    @model_validator(mode="after")
    def _run_shape(self) -> _FilterDocument:
        runs = self.static_runs
        if runs.edge_frames * 2 >= runs.min_frames:
            raise contract_error(
                "static_runs",
                "min_frames must exceed twice edge_frames so a run has an interior",
            )
        return self


def load_filter_rule(path: str | Path) -> FilterRule:
    source = Path(path).resolve()
    try:
        document = _FilterDocument(**load_yaml(source))
    except ConfigError as error:
        raise ConfigError(str(source), error.field, error.message) from error
    arm = document.right_arm
    grippers = document.grippers
    runs = document.static_runs
    return FilterRule(
        schema_version=document.schema_version,
        fps=document.fps,
        chunk_size=document.chunk_size,
        right_arm_indices=tuple(arm.joint_indices),
        gripper_indices=tuple(grippers.joint_indices),
        max_right_step_rad=float(arm.max_step_rad),
        max_right_velocity_rad_s=float(arm.max_velocity_rad_s),
        max_gripper_step_m=float(grippers.max_step_m),
        max_gripper_velocity_m_s=float(grippers.max_velocity_m_s),
        min_static_run_frames=runs.min_frames,
        keep_edge_frames=runs.edge_frames,
        interior_anchor_stride=runs.interior_stride,
        phase_boundary_frames=runs.phase_boundary_frames,
        episode_boundary_frames=runs.episode_boundary_frames,
        min_episode_kept_frames=runs.min_episode_kept_frames,
        known_phases=tuple(document.phase_gate.known_phases),
        tracking_gate=document.phase_gate.tracking_gate,
        reason=runs.reason,
        source=source,
    )


__all__ = [
    "FilterError",
    "FilterRule",
    "GRIPPERS",
    "KNOWN_PHASES",
    "REASON_REDUNDANT",
    "RIGHT_ARM",
    "load_filter_rule",
]
