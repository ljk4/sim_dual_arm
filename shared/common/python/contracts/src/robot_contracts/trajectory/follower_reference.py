"""Accepted bounded-follower reference records and their canonical digest.

The curve representation, sampling and hard-boundary normalization are shared
conventions; a record is accepted from any producer that supplies a valid
:class:`ReferencePlan`.  ``algorithm_version`` keeps the producer's real
identity for readers and logs, but is not compared with any shared constant.
"""

from __future__ import annotations

import hashlib
import json
from typing import Annotated, ClassVar, Self

from pydantic import BeforeValidator, Field, model_validator

from robot_contracts.trajectory.constants import MAX_HORIZON_NS, MAX_JOINTS
from robot_contracts.trajectory.follower import FollowerLimitFlags
from robot_contracts.trajectory.spline import ReferencePlan

from ..action_chunks import MAX_TARGET_COUNT, ActionWindow, ChunkAnchor
from ..models import ContractError, FloatVector
from ..stream_request import (
    REQUEST_STREAM_FIRST,
    REQUEST_STREAM_NEXT,
    StreamRequest,
)
from ..validation import (
    NonEmptyStr,
    PositiveInt,
    StrictContractModel,
    as_tuple,
    contract_error,
)

FOLLOWER_SAMPLER_VERSION = "follower-sampler-v1"
FOLLOWER_NORMALIZATION_VERSION = "hard-boundary-normalization-v1"
FOLLOWER_REFERENCE_DIGEST_VERSION = "follower-reference-v2"
# The previous v1 digest covered the same content under the producer-specific
# field name ``ruckig_duration_ns``.  It is used only to verify recorded legacy
# records whose stored definition differs from the installed one by that name.
FOLLOWER_REFERENCE_LEGACY_DIGEST_VERSION = "follower-reference-v1"
_LEGACY_DURATION_FIELD = "ruckig_duration_ns"
FOLLOWER_REFERENCE_KINDS = (REQUEST_STREAM_FIRST, REQUEST_STREAM_NEXT)
DIGEST_HEX_LENGTH = 64
_UINT64_MAX = 2**64 - 1
_INT64_MAX = 2**63 - 1
_DEVIATION_ATOL = 1e-12


def _invalid(location: str, message: str) -> ContractError:
    return ContractError("INVALID_FOLLOWER_REFERENCE", f"{location}: {message}")


def follower_reference_digest_payload(record: FollowerReference) -> dict[str, object]:
    """Return the complete canonical document covered by the record digest."""
    document = record.model_dump(mode="json", exclude={"reference_digest"})
    document["window"]["selected_indices"] = list(record.window.selected_indices)
    return {"digest_version": FOLLOWER_REFERENCE_DIGEST_VERSION, **document}


def _digest_of(payload: dict[str, object]) -> str:
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":"),
                         ensure_ascii=True, allow_nan=False)
    return hashlib.sha256(encoded.encode("ascii")).hexdigest()


def compute_follower_reference_digest(record: FollowerReference) -> str:
    """SHA-256 over every protected identity/source/curve/limit/time field."""
    if not isinstance(record, FollowerReference):
        raise _invalid("record", "must be a FollowerReference")
    return _digest_of(follower_reference_digest_payload(record))


def follower_reference_legacy_digest_payload(record: FollowerReference) -> dict[str, object]:
    """Return the v1 canonical document, which named the curve duration after Ruckig."""
    document = follower_reference_digest_payload(record)
    document["digest_version"] = FOLLOWER_REFERENCE_LEGACY_DIGEST_VERSION
    document[_LEGACY_DURATION_FIELD] = document.pop("curve_duration_ns")
    return document


def compute_legacy_follower_reference_digest(record: FollowerReference) -> str:
    """SHA-256 over the same record under the previous v1 digest payload."""
    if not isinstance(record, FollowerReference):
        raise _invalid("record", "must be a FollowerReference")
    return _digest_of(follower_reference_legacy_digest_payload(record))


class FollowerAppliedSample(StrictContractModel):
    """One raw coefficient sample and its auditable hard-boundary correction."""

    _error_reason: ClassVar[str] = "INVALID_FOLLOWER_REFERENCE"

    sampled_positions: FloatVector
    applied_positions: FloatVector
    corrections: FloatVector
    sampled_velocities: FloatVector
    sampled_accelerations: FloatVector

    @property
    def has_correction(self) -> bool:
        return any(value != 0.0 for value in self.corrections)


class FollowerReference(StrictContractModel):
    """One accepted bounded-follower block with explicit, protected evidence."""

    _error_reason: ClassVar[str] = "INVALID_FOLLOWER_REFERENCE"

    request: StreamRequest
    motion_seq: Annotated[PositiveInt, Field(le=_UINT64_MAX)]
    accepted_sim_time_ns: Annotated[int, Field(strict=True, ge=0, le=_INT64_MAX)]
    accepted_wall_time_ns: Annotated[int, Field(strict=True, ge=0, le=_INT64_MAX)]
    joint_names: Annotated[tuple[NonEmptyStr, ...], BeforeValidator(as_tuple),
                           Field(min_length=1, max_length=MAX_JOINTS)]
    plan: ReferencePlan
    anchor: ChunkAnchor
    window: ActionWindow
    max_accelerations: FloatVector
    max_target_deviation_limits: FloatVector
    raw_terminal_velocities: FloatVector
    restricted_terminal_velocities: FloatVector
    actual_terminal_positions: FloatVector
    actual_terminal_velocities: FloatVector
    actual_terminal_accelerations: FloatVector
    max_target_deviations: FloatVector
    is_final: bool
    requested_window_ns: Annotated[int, Field(strict=True, ge=1, le=MAX_HORIZON_NS)]
    curve_duration_ns: Annotated[int, Field(strict=True, ge=1, le=MAX_HORIZON_NS)]
    duration_grid_ns: Annotated[int, Field(strict=True, ge=1, le=MAX_HORIZON_NS)] | None
    velocity_limited: Annotated[tuple[bool, ...], BeforeValidator(as_tuple),
                                Field(min_length=1, max_length=MAX_JOINTS)]
    acceleration_limited: Annotated[tuple[bool, ...], BeforeValidator(as_tuple),
                                    Field(min_length=1, max_length=MAX_JOINTS)]
    boundary_preserved: Annotated[tuple[bool, ...], BeforeValidator(as_tuple),
                                  Field(min_length=1, max_length=MAX_JOINTS)]
    algorithm_version: NonEmptyStr
    sampler_version: str = FOLLOWER_SAMPLER_VERSION
    normalization_version: str = FOLLOWER_NORMALIZATION_VERSION
    reference_digest: Annotated[str, Field(pattern=r"^(?:[0-9a-f]{64})?$")] | None = None

    @model_validator(mode="after")
    def _record(self) -> Self:
        request = self.request
        if request.kind not in FOLLOWER_REFERENCE_KINDS:
            raise contract_error("request.kind", "requires STREAM_FIRST or STREAM_NEXT")
        if request.anchor is None or request.window is None:
            raise contract_error("request", "requires an anchor and window")
        for name, expected in (
            ("sampler_version", FOLLOWER_SAMPLER_VERSION),
            ("normalization_version", FOLLOWER_NORMALIZATION_VERSION),
        ):
            if getattr(self, name) != expected:
                raise contract_error(name, f"unsupported version {getattr(self, name)!r}")
        joint_count = len(self.joint_names)
        if self.plan.joint_names != self.joint_names:
            raise contract_error("joint_names", "must match the explicit plan")
        if tuple(request.trajectory.joint_names) != self.joint_names:
            raise contract_error("joint_names", "must match the raw request")
        if request.anchor != self.anchor:
            raise contract_error("anchor", "must equal the raw request anchor")
        if request.window != self.window:
            raise contract_error("window", "must equal the recomputed request window")
        if self.window.observation_id != request.trajectory.observation_id:
            raise contract_error("window.observation_id", "must equal request observation_id")
        if self.window.target_count != request.target_count:
            raise contract_error("window.target_count", "must equal request target_count")
        if self.window.handoff_time_ns != self.anchor.start_sim_time_ns:
            raise contract_error("window.handoff_time_ns", "must equal anchor start time")
        if self.is_final != request.settle_at_end:
            raise contract_error("is_final", "must equal request settle_at_end")
        if not (request.observation_time_ns <= self.accepted_sim_time_ns
                <= self.anchor.start_sim_time_ns
                and self.accepted_sim_time_ns <= request.trajectory.accept_before_ns):
            raise contract_error("accepted_sim_time_ns",
                                 "must follow the observation and precede both deadlines")
        for name in ("max_accelerations", "max_target_deviation_limits",
                     "raw_terminal_velocities", "restricted_terminal_velocities",
                     "actual_terminal_positions", "actual_terminal_velocities",
                     "actual_terminal_accelerations", "max_target_deviations"):
            values = getattr(self, name)
            if len(values) != joint_count:
                raise contract_error(name, "must contain one value per joint")
            if name in ("max_accelerations", "max_target_deviation_limits") and any(
                    value <= 0.0 for value in values):
                raise contract_error(name, "must be positive")
        if any(value < 0.0 for value in self.max_target_deviations):
            raise contract_error("max_target_deviations", "must not be negative")
        for name in ("velocity_limited", "acceleration_limited", "boundary_preserved"):
            if len(getattr(self, name)) != joint_count:
                raise contract_error(name, "must contain one value per joint")
        if self.requested_window_ns != self.window.end_time_ns:
            raise contract_error("requested_window_ns", "must equal the window end time")
        if self.curve_duration_ns < self.plan.duration_ns:
            raise contract_error("curve_duration_ns", "must cover the plan duration")
        if self.duration_grid_ns is not None:
            if self.plan.duration_ns % self.duration_grid_ns != 0:
                raise contract_error("duration_grid_ns", "plan duration must lie on the grid")
        if self.is_final:
            if self.plan.duration_ns < self.requested_window_ns:
                raise contract_error("plan.duration_ns", "a final plan must cover the window")
        elif self.plan.duration_ns != self.requested_window_ns:
            raise contract_error("plan.duration_ns", "a non-final plan must end at the hand-off")
        expected = compute_follower_reference_digest(self)
        if self.reference_digest is None:
            object.__setattr__(self, "reference_digest", expected)
        elif self.reference_digest != expected:
            raise ContractError(
                "FOLLOWER_DIGEST_MISMATCH",
                "reference_digest does not cover the retained request/curve/times",
            )
        return self

    context = property(lambda self: self.request.trajectory.context)

    command_id = property(lambda self: self.request.trajectory.command_id)

    command_seq = property(lambda self: self.request.trajectory.command_seq)

    observation_id = property(lambda self: self.request.trajectory.observation_id)

    kind = property(lambda self: self.request.kind)

    mode = property(lambda self: self.request.mode)

    stream_id = property(lambda self: self.anchor.stream_id)

    chunk_seq = property(lambda self: self.anchor.seq)

    joint_count = property(lambda self: len(self.joint_names))

    duration_ns = property(lambda self: self.plan.duration_ns)

    start_sim_time_ns = property(lambda self: self.anchor.start_sim_time_ns)

    end_sim_time_ns = property(lambda self: self.start_sim_time_ns + self.plan.duration_ns)

    extension_ns = property(lambda self: max(0, self.plan.duration_ns - self.requested_window_ns))

    flags = property(lambda self: FollowerLimitFlags(
        self.velocity_limited, self.acceleration_limited, self.boundary_preserved))





    def to_document(self) -> dict[str, object]:
        document = self.model_dump(mode="json")
        document["window"]["selected_indices"] = list(self.window.selected_indices)
        return document






class FollowerValidationLimits(StrictContractModel):
    """Explicit pure-standard-library constraints for one follower record."""

    _error_reason: ClassVar[str] = "INVALID_FOLLOWER_REFERENCE"

    joint_names: Annotated[tuple[NonEmptyStr, ...], BeforeValidator(as_tuple),
                           Field(min_length=1, max_length=MAX_JOINTS)]
    max_accelerations: FloatVector
    max_target_deviations: FloatVector
    target_count: Annotated[int, Field(strict=True, ge=2, le=MAX_TARGET_COUNT)]
    max_terminal_tail_duration_ns: Annotated[int, Field(strict=True, ge=1, le=MAX_HORIZON_NS)]
    duration_grid_ns: Annotated[int, Field(strict=True, ge=1, le=MAX_HORIZON_NS)] | None = None

    @model_validator(mode="after")
    def _limits(self) -> Self:
        for name in ("max_accelerations", "max_target_deviations"):
            values = getattr(self, name)
            if len(values) != len(self.joint_names):
                raise contract_error(f"limits.{name}", "must match the joint dimension")
            if any(value <= 0.0 for value in values):
                raise contract_error(f"limits.{name}", "must be positive")
        return self
