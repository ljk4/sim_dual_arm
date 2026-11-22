"""Immutable observation-anchored action windows and chunk hand-off anchors."""

from __future__ import annotations

from typing import Annotated, ClassVar, Self

from pydantic import BeforeValidator, Field, model_validator

from .models import Target
from .trajectory.spline import MAX_HORIZON_NS, MAX_JOINTS, MAX_SEGMENTS, ReferenceSample
from .validation import (
    NonEmptyStr,
    StrictContractModel,
    as_tuple,
    contract_error,
)

UINT64_MAX = 2**64 - 1
INT64_MAX = 2**63 - 1
MAX_TARGET_COUNT = 128
MAX_STREAM_ID_LENGTH = 128


class ActionWindow(StrictContractModel):
    """A pre-arranged window of original predictions relative to one hand-off."""

    _error_reason: ClassVar[str] = "INVALID_ACTION_WINDOW"
    observation_id: Annotated[int, Field(strict=True, ge=1, le=UINT64_MAX)]
    observation_time_ns: Annotated[int, Field(strict=True, ge=0, le=INT64_MAX)]
    handoff_time_ns: Annotated[int, Field(strict=True, ge=0, le=INT64_MAX)]
    source_targets: Annotated[tuple[Target, ...], BeforeValidator(as_tuple),
                              Field(min_length=1, max_length=MAX_SEGMENTS)]
    target_count: Annotated[int, Field(strict=True, ge=2, le=MAX_TARGET_COUNT)]

    @model_validator(mode="after")
    def _window(self) -> Self:
        if self.handoff_time_ns < self.observation_time_ns:
            raise contract_error("handoff_time_ns", "must not precede observation_time_ns",
                                 "INVALID_ACTION_WINDOW")
        previous = 0
        joints = len(self.source_targets[0].positions)
        for index, target in enumerate(self.source_targets):
            if target.time_ns <= previous or target.time_ns > MAX_HORIZON_NS:
                raise contract_error(
                    f"source_targets[{index}].time_ns",
                    "must be strictly increasing and inside the horizon",
                    "INVALID_ACTION_WINDOW",
                )
            if len(target.positions) != joints or joints > MAX_JOINTS:
                raise contract_error(
                    f"source_targets[{index}].positions",
                    "must match the first target dimension",
                    "INVALID_ACTION_WINDOW",
                )
            if self.observation_time_ns + target.time_ns > INT64_MAX:
                raise contract_error(
                    f"source_targets[{index}].time_ns",
                    "observation time plus relative time exceeds int64",
                    "INVALID_ACTION_WINDOW",
                )
            previous = target.time_ns
        selected = self._selection()
        if not selected:
            raise contract_error(
                "source_targets", "every prediction is at or before handoff_time_ns",
                "CHUNK_EXPIRED",
            )
        if len(selected) < self.target_count:
            raise contract_error(
                "source_targets",
                f"only {len(selected)} predictions are strictly after handoff_time_ns, "
                f"target_count is {self.target_count}",
                "INSUFFICIENT_HORIZON",
            )
        return self

    def _selection(self) -> tuple[int, ...]:
        return tuple(
            index
            for index, target in enumerate(self.source_targets)
            if self.observation_time_ns + target.time_ns > self.handoff_time_ns
        )[: self.target_count]

    @property
    def selected_indices(self) -> tuple[int, ...]:
        return self._selection()

    @property
    def skipped_overlap_count(self) -> int:
        return self.selected_indices[0]

    @property
    def skipped_overlap_indices(self) -> tuple[int, ...]:
        return tuple(range(self.skipped_overlap_count))

    @property
    def selected_source_targets(self) -> tuple[Target, ...]:
        return tuple(self.source_targets[index] for index in self.selected_indices)

    @property
    def targets(self) -> tuple[Target, ...]:
        return tuple(
            Target(
                self.observation_time_ns
                + self.source_targets[index].time_ns
                - self.handoff_time_ns,
                self.source_targets[index].positions,
            )
            for index in self.selected_indices
        )

    @property
    def end_time_ns(self) -> int:
        return self.targets[-1].time_ns

    @property
    def absolute_times_ns(self) -> tuple[int, ...]:
        return tuple(
            self.observation_time_ns + target.time_ns for target in self.source_targets
        )


class ChunkAnchor(StrictContractModel):
    """Reference state at a scheduled hand-off; no identity or scene is read."""

    _error_reason: ClassVar[str] = "INVALID_CHUNK_ANCHOR"
    stream_id: Annotated[NonEmptyStr, Field(max_length=MAX_STREAM_ID_LENGTH)]
    predecessor_seq: Annotated[int, Field(strict=True, ge=0, le=UINT64_MAX - 1)]
    start_backend_time_ns: Annotated[int, Field(strict=True, ge=0, le=INT64_MAX)]
    reference: ReferenceSample
    robot_profile_hash: NonEmptyStr

    @property
    def seq(self) -> int:
        """Sequence of the chunk this anchor starts, derived from predecessor."""
        return self.predecessor_seq + 1


__all__ = ["INT64_MAX", "MAX_TARGET_COUNT", "UINT64_MAX", "ActionWindow", "ChunkAnchor"]
