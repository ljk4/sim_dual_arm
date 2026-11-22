"""Strict Pydantic validation of the static-anchor filter rule document."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Annotated, Literal

from pydantic import BeforeValidator, Field, model_validator
from robot_contracts.validation import (
    NonEmptyStr,
    NonNegativeFloat,
    PositiveInt,
    StrictMappingModel,
    as_tuple,
    contract_error,
)

from ..config import ConfigError, load_yaml_mapping
from .common import file_sha256

LEAD_NS = 50_000_000
PERIOD_NS = 50_000_000
STATE_DIM = 16
REASON_REDUNDANT = "redundant_static_anchor"
ARM_INDICES = (0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14)
GRIPPERS = (7, 15)

JointIndices = Annotated[
    tuple[Annotated[int, Field(strict=True, ge=0, le=255)], ...],
    BeforeValidator(as_tuple),
    Field(min_length=1),
]
StepLimit = Annotated[NonNegativeFloat, Field(le=1.0)]
SpeedLimit = Annotated[NonNegativeFloat, Field(le=100.0)]
FrameCount = Annotated[int, Field(strict=True, ge=0, le=100_000)]


def _optional_tuple(value: object) -> object:
    return None if value is None else as_tuple(value)


OptionalGateStates = Annotated[
    tuple[NonEmptyStr, ...] | None,
    BeforeValidator(_optional_tuple),
]


class FilterError(ValueError):
    """The filter rule or mask cannot be trusted."""


@dataclass(frozen=True, slots=True)
class FilterRule:
    fps: int
    chunk_size: int
    arm_indices: tuple[int, ...]
    gripper_indices: tuple[int, ...]
    max_arm_step_rad: float
    max_arm_velocity_rad_s: float
    max_gripper_step_m: float
    max_gripper_velocity_m_s: float
    min_static_run_frames: int
    keep_edge_frames: int
    interior_anchor_stride: int
    phase_boundary_frames: int
    episode_boundary_frames: int
    min_episode_kept_frames: int
    reason: str
    source: Path
    only_gate_states: tuple[str, ...] | None = None

    @property
    def sha256(self) -> str:
        return file_sha256(self.source)


class _ArmRule(StrictMappingModel):
    joint_indices: JointIndices = ARM_INDICES
    max_step_rad: StepLimit
    max_velocity_rad_s: SpeedLimit

    @model_validator(mode="after")
    def _unique(self) -> _ArmRule:
        if len(set(self.joint_indices)) != len(self.joint_indices):
            raise contract_error("joint_indices", "must contain unique joint indices")
        return self


class _GripperRule(StrictMappingModel):
    joint_indices: JointIndices = GRIPPERS
    max_step_m: StepLimit
    max_velocity_m_s: SpeedLimit

    @model_validator(mode="after")
    def _unique(self) -> _GripperRule:
        if len(set(self.joint_indices)) != len(self.joint_indices):
            raise contract_error("joint_indices", "must contain unique joint indices")
        return self


class _StaticRuns(StrictMappingModel):
    min_frames: Annotated[int, Field(strict=True, ge=2, le=100_000)]
    edge_frames: FrameCount
    interior_stride: Annotated[int, Field(strict=True, ge=1, le=100_000)]
    phase_boundary_frames: FrameCount
    episode_boundary_frames: FrameCount
    min_episode_kept_frames: FrameCount
    only_gate_states: OptionalGateStates = None
    reason: Literal["redundant_static_anchor"]

    @model_validator(mode="after")
    def _gates(self) -> _StaticRuns:
        if self.only_gate_states is not None:
            if not self.only_gate_states:
                raise contract_error("only_gate_states", "must not be empty")
            if len(set(self.only_gate_states)) != len(self.only_gate_states):
                raise contract_error("only_gate_states", "must contain unique gate states")
        return self


class _FilterDocument(StrictMappingModel):
    fps: PositiveInt
    chunk_size: PositiveInt
    arms: _ArmRule
    grippers: _GripperRule
    static_runs: _StaticRuns

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
        document = _FilterDocument(**load_yaml_mapping(source))
    except ConfigError as error:
        raise ConfigError(str(source), error.field, error.message) from error
    arms = document.arms
    grippers = document.grippers
    runs = document.static_runs
    return FilterRule(
        fps=document.fps,
        chunk_size=document.chunk_size,
        arm_indices=tuple(arms.joint_indices),
        gripper_indices=tuple(grippers.joint_indices),
        max_arm_step_rad=float(arms.max_step_rad),
        max_arm_velocity_rad_s=float(arms.max_velocity_rad_s),
        max_gripper_step_m=float(grippers.max_step_m),
        max_gripper_velocity_m_s=float(grippers.max_velocity_m_s),
        min_static_run_frames=runs.min_frames,
        keep_edge_frames=runs.edge_frames,
        interior_anchor_stride=runs.interior_stride,
        phase_boundary_frames=runs.phase_boundary_frames,
        episode_boundary_frames=runs.episode_boundary_frames,
        min_episode_kept_frames=runs.min_episode_kept_frames,
        reason=runs.reason,
        source=source,
        only_gate_states=(
            None if runs.only_gate_states is None else tuple(runs.only_gate_states)
        ),
    )


__all__ = [
    "ARM_INDICES",
    "FilterError",
    "FilterRule",
    "GRIPPERS",
    "LEAD_NS",
    "PERIOD_NS",
    "REASON_REDUNDANT",
    "STATE_DIM",
    "load_filter_rule",
]
