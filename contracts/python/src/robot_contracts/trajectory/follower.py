"""Follower normalization and limit flag data."""
from __future__ import annotations

from typing import Annotated, Self

from pydantic import BeforeValidator, Field, model_validator

from robot_contracts.models import FloatVector
from robot_contracts.validation import StrictContractModel, as_tuple, contract_error

from .constants import MAX_JOINTS


class FollowerLimitFlags(StrictContractModel):
    velocity_limited: Annotated[tuple[bool, ...], BeforeValidator(as_tuple),
                                Field(min_length=1, max_length=MAX_JOINTS)]
    acceleration_limited: Annotated[tuple[bool, ...], BeforeValidator(as_tuple),
                                    Field(min_length=1, max_length=MAX_JOINTS)]
    boundary_preserved: Annotated[tuple[bool, ...], BeforeValidator(as_tuple),
                                  Field(min_length=1, max_length=MAX_JOINTS)]

    @model_validator(mode="after")
    def _same_length(self) -> Self:
        if len({len(self.velocity_limited), len(self.acceleration_limited),
                len(self.boundary_preserved)}) != 1:
            raise contract_error("flags", "all flag arrays must have the same length")
        return self

    any_velocity_limited = property(lambda self: any(self.velocity_limited))
    any_acceleration_limited = property(lambda self: any(self.acceleration_limited))
    all_boundaries_preserved = property(lambda self: all(self.boundary_preserved))

class NormalizedFollowerPositions(StrictContractModel):
    sampled_positions: FloatVector
    applied_positions: FloatVector
    corrections: FloatVector

    @model_validator(mode="after")
    def _same_length(self) -> Self:
        if not (len(self.sampled_positions) == len(self.applied_positions)
                == len(self.corrections)):
            raise contract_error("corrections", "must match the sampled dimension")
        return self

    has_correction = property(lambda self: any(value != 0.0 for value in self.corrections))
