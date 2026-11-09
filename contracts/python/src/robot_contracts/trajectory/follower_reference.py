"""Accepted bounded-follower reference records and their canonical digest."""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Sequence
from typing import Annotated, ClassVar, Self

from pydantic import BeforeValidator, Field, model_validator

from ..action_chunks import MAX_TARGET_COUNT, ActionWindow, ChunkAnchor
from ..models import ContractError, FloatVector, JointSpec, RobotProfile
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
from .follower import (
    FOLLOWER_ALGORITHM_VERSION,
    FollowerLimitFlags,
    PreparedFollowerPlan,
    normalize_follower_positions,
)
from .spline import (
    MAX_HORIZON_NS,
    MAX_JOINTS,
    ReferencePlan,
    ReferenceSample,
)

FOLLOWER_SAMPLER_VERSION = "follower-sampler-v1"
FOLLOWER_NORMALIZATION_VERSION = "hard-boundary-normalization-v1"
FOLLOWER_REFERENCE_DIGEST_VERSION = "follower-reference-v1"
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


def compute_follower_reference_digest(record: FollowerReference) -> str:
    """SHA-256 over every protected identity/source/curve/limit/time field."""
    if not isinstance(record, FollowerReference):
        raise _invalid("record", "must be a FollowerReference")
    encoded = json.dumps(follower_reference_digest_payload(record), sort_keys=True,
                         separators=(",", ":"), ensure_ascii=True, allow_nan=False)
    return hashlib.sha256(encoded.encode("ascii")).hexdigest()


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
    ruckig_duration_ns: Annotated[int, Field(strict=True, ge=1, le=MAX_HORIZON_NS)]
    duration_grid_ns: Annotated[int, Field(strict=True, ge=1, le=MAX_HORIZON_NS)] | None
    velocity_limited: Annotated[tuple[bool, ...], BeforeValidator(as_tuple),
                                Field(min_length=1, max_length=MAX_JOINTS)]
    acceleration_limited: Annotated[tuple[bool, ...], BeforeValidator(as_tuple),
                                    Field(min_length=1, max_length=MAX_JOINTS)]
    boundary_preserved: Annotated[tuple[bool, ...], BeforeValidator(as_tuple),
                                  Field(min_length=1, max_length=MAX_JOINTS)]
    algorithm_version: str = FOLLOWER_ALGORITHM_VERSION
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
            ("algorithm_version", FOLLOWER_ALGORITHM_VERSION),
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
        if self.ruckig_duration_ns < self.plan.duration_ns:
            raise contract_error("ruckig_duration_ns", "must cover the plan duration")
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

    def sample_at(self, sim_time_ns: int) -> ReferenceSample:
        """Return the untouched coefficient sample."""
        return self._plan_sample_at(sim_time_ns)

    def applied_sample_at(
        self, sim_time_ns: int, profile_or_joints: RobotProfile | Sequence[JointSpec],
    ) -> FollowerAppliedSample:
        """Reconstruct the physically applied reference and its corrections."""
        joints = _joints_from(profile_or_joints)
        sample = self.sample_at(sim_time_ns)
        normalized = normalize_follower_positions(sample.positions, joints)
        return FollowerAppliedSample(
            sampled_positions=normalized.sampled_positions,
            applied_positions=normalized.applied_positions,
            corrections=normalized.corrections,
            sampled_velocities=sample.velocities,
            sampled_accelerations=sample.accelerations,
        )

    def _plan_sample_at(self, sim_time_ns: int) -> ReferenceSample:
        if isinstance(sim_time_ns, bool) or not isinstance(sim_time_ns, int):
            raise _invalid("sim_time_ns", "must be an integer")
        if sim_time_ns < self.start_sim_time_ns or sim_time_ns > self.end_sim_time_ns:
            raise ContractError(
                "REFERENCE_EXPIRED",
                f"sim_time_ns {sim_time_ns} is outside "
                f"[{self.start_sim_time_ns}, {self.end_sim_time_ns}]")
        return self.plan.sample_interval(sim_time_ns - self.start_sim_time_ns)

    @classmethod
    def from_prepared(
        cls,
        prepared: PreparedFollowerPlan,
        request: StreamRequest,
        *,
        motion_seq: int,
        accepted_sim_time_ns: int,
        accepted_wall_time_ns: int,
        joint_names: Sequence[str] | None = None,
    ) -> FollowerReference:
        """Capture one prepared follower plan as an immutable accepted record."""
        if not isinstance(prepared, PreparedFollowerPlan):
            raise _invalid("prepared", "must be a PreparedFollowerPlan")
        if not isinstance(request, StreamRequest):
            raise _invalid("request", "must be a StreamRequest")
        names = prepared.joint_names if joint_names is None else tuple(joint_names)
        return cls(
            request=request,
            motion_seq=motion_seq,
            accepted_sim_time_ns=accepted_sim_time_ns,
            accepted_wall_time_ns=accepted_wall_time_ns,
            joint_names=names,
            plan=prepared.plan,
            anchor=prepared.anchor,
            window=prepared.window,
            max_accelerations=prepared.max_accelerations,
            max_target_deviation_limits=prepared.max_target_deviation_limits,
            raw_terminal_velocities=prepared.raw_terminal_velocities,
            restricted_terminal_velocities=prepared.restricted_terminal_velocities,
            actual_terminal_positions=prepared.actual_terminal_positions,
            actual_terminal_velocities=prepared.actual_terminal_velocities,
            actual_terminal_accelerations=prepared.actual_terminal_accelerations,
            max_target_deviations=prepared.max_target_deviations,
            is_final=prepared.is_final,
            requested_window_ns=prepared.requested_window_ns,
            ruckig_duration_ns=prepared.ruckig_duration_ns,
            duration_grid_ns=prepared.duration_grid_ns,
            velocity_limited=prepared.flags.velocity_limited,
            acceleration_limited=prepared.flags.acceleration_limited,
            boundary_preserved=prepared.flags.boundary_preserved,
        )

    def to_document(self) -> dict[str, object]:
        document = self.model_dump(mode="json")
        document["window"]["selected_indices"] = list(self.window.selected_indices)
        return document


def _joints_from(profile_or_joints: RobotProfile | Sequence[JointSpec]) -> tuple[JointSpec, ...]:
    if isinstance(profile_or_joints, RobotProfile):
        return tuple(profile_or_joints.joints)
    joints = tuple(profile_or_joints)
    if not joints or any(not isinstance(joint, JointSpec) for joint in joints):
        raise _invalid("joints", "must contain JointSpec values")
    return joints


def _same_vector(left: Sequence[float], right: Sequence[float]) -> bool:
    return len(left) == len(right) and all(
        math.isclose(a, b, rel_tol=0.0, abs_tol=_DEVIATION_ATOL)
        for a, b in zip(left, right, strict=True)
    )


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


def validate(record: FollowerReference, profile: RobotProfile,
             limits: FollowerValidationLimits) -> PreparedFollowerPlan:
    if not isinstance(record, FollowerReference):
        raise _invalid("record", "must be a FollowerReference")
    if not isinstance(profile, RobotProfile):
        raise ContractError("PROFILE_MISMATCH", "profile must be a RobotProfile")
    if not isinstance(limits, FollowerValidationLimits):
        raise _invalid("limits", "must be a FollowerValidationLimits")
    if limits.joint_names != tuple(profile.joint_names):
        raise ContractError("PROFILE_MISMATCH", "limits joint order differs from profile")
    if record.joint_names != limits.joint_names:
        raise ContractError("PROFILE_MISMATCH", "record joint order differs from limits")
    if record.anchor.robot_profile_hash != profile.digest:
        raise ContractError("PROFILE_MISMATCH", "record anchor profile digest differs")
    if not _same_vector(limits.max_accelerations, record.max_accelerations):
        raise ContractError("FOLLOWER_LIMIT_MISMATCH",
                            "record max_accelerations differ from the validation limits")
    if not _same_vector(limits.max_target_deviations, record.max_target_deviation_limits):
        raise ContractError("FOLLOWER_LIMIT_MISMATCH",
                            "record deviation limits differ from the validation limits")
    if record.window.target_count != limits.target_count:
        raise ContractError("FOLLOWER_LIMIT_MISMATCH",
                            "record window.target_count differs from the validation limits")
    if record.request.target_count != limits.target_count:
        raise ContractError("FOLLOWER_LIMIT_MISMATCH",
                            "raw request target_count differs from the validation limits")
    if record.duration_grid_ns != limits.duration_grid_ns:
        raise ContractError("FOLLOWER_GRID_MISMATCH",
                            "record duration grid differs from the validation limits")
    if (record.requested_window_ns > limits.max_terminal_tail_duration_ns
            or record.plan.duration_ns > limits.max_terminal_tail_duration_ns):
        raise ContractError("FOLLOWER_HORIZON_EXCEEDED",
                            "record horizon exceeds the configured terminal tail")
    if record.is_final != record.request.settle_at_end:
        raise _invalid("is_final", "differs from the raw request settle_at_end flag")
    if not (record.request.observation_time_ns <= record.accepted_sim_time_ns
            <= record.anchor.start_sim_time_ns
            and record.accepted_sim_time_ns <= record.request.trajectory.accept_before_ns):
        raise _invalid("accepted_sim_time_ns",
                       "must follow the observation and precede both deadlines")
    rebuilt = PreparedFollowerPlan(
        anchor=record.anchor,
        window=record.window,
        plan=record.plan,
        joints=tuple(profile.joints),
        max_accelerations=record.max_accelerations,
        max_target_deviation_limits=record.max_target_deviation_limits,
        raw_terminal_velocities=record.raw_terminal_velocities,
        restricted_terminal_velocities=record.restricted_terminal_velocities,
        is_final=record.is_final,
        requested_window_ns=record.requested_window_ns,
        ruckig_duration_ns=record.ruckig_duration_ns,
        duration_grid_ns=record.duration_grid_ns,
    )
    if rebuilt.plan != record.plan:
        raise _invalid("plan", "reconstructed plan differs")
    for name in (
        "actual_terminal_positions", "actual_terminal_velocities",
        "actual_terminal_accelerations", "max_target_deviations",
        "raw_terminal_velocities", "restricted_terminal_velocities",
    ):
        if not _same_vector(getattr(record, name), getattr(rebuilt, name)):
            raise _invalid(name, "differs from the independently rebuilt DSH-030 plan")
    if record.flags != rebuilt.flags:
        raise _invalid("flags", "limit flags differ from the independently rebuilt plan")
    if record.extension_ns != rebuilt.extension_ns:
        raise _invalid("extension_ns", "extended horizon differs from the rebuilt plan")
    return rebuilt


__all__ = [
    "DIGEST_HEX_LENGTH", "FOLLOWER_NORMALIZATION_VERSION", "FOLLOWER_REFERENCE_DIGEST_VERSION",
    "FOLLOWER_REFERENCE_KINDS", "FOLLOWER_SAMPLER_VERSION",
    "FollowerAppliedSample", "FollowerReference", "FollowerValidationLimits",
    "compute_follower_reference_digest", "validate",
]
