"""Shared strict configuration building blocks for simulation tasks."""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import BeforeValidator, Field, model_validator
from robot_contracts.validation import (
    PositiveFloat,
    PositiveInt,
    StrictFloat,
    StrictMappingModel,
    as_tuple,
    contract_error,
)

__all__ = [
    "BaseCapacityConfig",
    "ClearanceSettings",
    "HARD_MAX_CHUNKS",
    "IKSettingsBase",
    "SHA256_PATTERN",
    "Vec3",
    "validate_task_capacity",
]

HARD_MAX_CHUNKS = 4096
SHA256_PATTERN = r"^[0-9a-f]{64}$"

Vec3 = Annotated[
    tuple[StrictFloat, StrictFloat, StrictFloat],
    BeforeValidator(as_tuple),
]


class BaseCapacityConfig(StrictMappingModel):
    expected_chunks: Annotated[PositiveInt, Field(ge=1, le=HARD_MAX_CHUNKS)]
    expected_references: Annotated[PositiveInt, Field(ge=1, le=HARD_MAX_CHUNKS)]
    expected_events: Annotated[PositiveInt, Field(ge=1, le=4 * HARD_MAX_CHUNKS)]
    max_episode_chunks: Annotated[PositiveInt, Field(ge=1, le=HARD_MAX_CHUNKS)]
    max_teacher_requests: Annotated[PositiveInt, Field(ge=1, le=HARD_MAX_CHUNKS)]
    rpc_cache_entries: Annotated[PositiveInt, Field(ge=1, le=HARD_MAX_CHUNKS)]
    backend_chunk_history: Annotated[PositiveInt, Field(ge=1, le=HARD_MAX_CHUNKS)]


class IKSettingsBase(StrictMappingModel):
    dt_s: Annotated[PositiveFloat, Field(gt=1e-4, le=0.05)]
    max_iterations: Annotated[PositiveInt, Field(le=5000)]
    position_tolerance_m: Annotated[PositiveFloat, Field(gt=1e-6, le=0.05)]
    orientation_tolerance_rad: Annotated[PositiveFloat, Field(gt=1e-6, le=0.5)]
    posture_cost: Annotated[StrictFloat, Field(ge=0.0, le=1.0)]
    position_cost: Annotated[PositiveFloat, Field(gt=0.0, le=100.0)]
    orientation_cost: Annotated[PositiveFloat, Field(gt=0.0, le=100.0)]
    lm_damping: Annotated[StrictFloat, Field(ge=0.0, le=1.0)]
    limit_margin_rad: Annotated[StrictFloat, Field(ge=0.0, le=0.2)] = 0.05
    solver: Literal["daqp"]


class ClearanceSettings(StrictMappingModel):
    minimum_clearance_m: Annotated[PositiveFloat, Field(gt=1e-5, le=0.05)]
    collision_detection_distance_m: Annotated[PositiveFloat, Field(le=0.2)]
    sample_step_rad: Annotated[PositiveFloat, Field(gt=1e-5, le=0.05)]
    max_samples_per_segment: Annotated[PositiveInt, Field(ge=2, le=5000)]

    @model_validator(mode="after")
    def _clearance(self) -> ClearanceSettings:
        if self.collision_detection_distance_m < self.minimum_clearance_m:
            raise contract_error(
                "collision_detection_distance_m",
                "must be at least minimum_clearance_m",
            )
        return self


def validate_task_capacity(capacity: BaseCapacityConfig) -> None:
    """Validate internal ordering invariants across a task's capacity budget."""
    checks = (
        (
            capacity.max_episode_chunks >= capacity.expected_chunks,
            "capacity.max_episode_chunks",
            "must cover expected_chunks",
        ),
        (
            capacity.expected_references <= capacity.max_episode_chunks,
            "capacity.expected_references",
            "must not exceed max_episode_chunks",
        ),
        (
            capacity.expected_events >= 3 * capacity.expected_chunks,
            "capacity.expected_events",
            "must cover three events per chunk",
        ),
        (
            capacity.rpc_cache_entries >= capacity.max_teacher_requests,
            "capacity.rpc_cache_entries",
            "must cover max_teacher_requests",
        ),
        (
            capacity.backend_chunk_history >= capacity.max_episode_chunks,
            "capacity.backend_chunk_history",
            "must cover max_episode_chunks",
        ),
        (
            capacity.max_teacher_requests >= capacity.max_episode_chunks,
            "capacity.max_teacher_requests",
            "must cover max_episode_chunks",
        ),
    )
    for ok, field, message in checks:
        if not ok:
            raise contract_error(field, message)
