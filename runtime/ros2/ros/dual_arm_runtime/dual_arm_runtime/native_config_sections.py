"""Nested Pydantic sections for the diagnostic native runtime configuration."""

from __future__ import annotations

from typing import Annotated, Literal

from pydantic import BeforeValidator, Field, model_validator
from robot_contracts.validation import (
    NonEmptyStr,
    NonNegativeInt,
    PositiveFloat,
    PositiveInt,
    Sha256HexStr,
    StrictFloat,
    StrictMappingModel,
    as_tuple,
    contract_error,
)

MIN_LEASE_DURATION_NS = 100_000_000
MAX_LEASE_DURATION_NS = 10_000_000_000
MAX_CHUNKS = 512
MAX_TASK_BUDGET_NS = 30_000_000_000
STREAM_CHUNK_SIZE = 60
STREAM_PERIOD_NS = 50_000_000
INT64_MAX = 2**63 - 1
UINT64_MAX = 2**64 - 1
REQUIRED_CAPABILITIES = (
    "continuous_chunks", "exact_capture", "simulation_reset", "lease", "stop",
    "command_query", "native_events", "follower_reference",
)
_VECTOR = Annotated[tuple[StrictFloat, ...], BeforeValidator(as_tuple),
                    Field(min_length=1, max_length=64)]
_POSITIVE_VECTOR = Annotated[tuple[PositiveFloat, ...], BeforeValidator(as_tuple),
                             Field(min_length=1, max_length=64)]


class NativeMotion(StrictMappingModel):
    name: NonEmptyStr
    duration_ns: Annotated[PositiveInt, Field(le=3_000_000_000)]
    positions: _VECTOR


class NativeReset(StrictMappingModel):
    enabled: bool
    seed: Annotated[NonNegativeInt, Field(le=INT64_MAX)]


class NativeObservation(StrictMappingModel):
    max_skew_ns: Annotated[NonNegativeInt, Field(le=INT64_MAX)]
    max_age_ns: Annotated[PositiveInt, Field(le=INT64_MAX)]
    wall_arrival_timeout_s: PositiveFloat
    capacity: Annotated[PositiveInt, Field(le=4096)]
    reliable: bool = False


class NativeControl(StrictMappingModel):
    submit_query_retries: Annotated[int, Field(strict=True, ge=0, le=8)]
    poll_interval_s: PositiveFloat
    stop_timeout_s: PositiveFloat
    terminal_timeout_s: PositiveFloat


class NativeFollower(StrictMappingModel):
    target_count: Annotated[int, Field(strict=True, ge=2, le=128)]
    max_target_deviations: _POSITIVE_VECTOR
    max_terminal_tail_duration_ns: Annotated[PositiveInt, Field(le=3_000_000_000)]
    duration_grid_ns: Annotated[PositiveInt, Field(le=3_000_000_000)]


class FollowerConstraints(StrictMappingModel):
    joint_names: Annotated[tuple[NonEmptyStr, ...], BeforeValidator(as_tuple),
                           Field(min_length=1, max_length=64)]
    target_count: Annotated[int, Field(strict=True, ge=2, le=128)]
    max_target_deviations: _POSITIVE_VECTOR
    max_terminal_tail_duration_s: PositiveFloat

    @property
    def max_terminal_tail_duration_ns(self) -> int:
        return round(self.max_terminal_tail_duration_s * 1_000_000_000)


class NativePolicy(StrictMappingModel):
    """Policy RPC identity, bounds and the finite diagnostic stream budget."""

    endpoint: Annotated[NonEmptyStr, Field(pattern=r"^[^:\s]+:\d+$")]
    model_bundle_digest: Sha256HexStr
    processor_digest: Sha256HexStr
    observation_schema_hash: Sha256HexStr
    action_schema_hash: Sha256HexStr
    robot_profile_hash: Sha256HexStr
    allow_test_policy: bool
    expected_test_only: bool
    task_id: NonEmptyStr
    task_text: NonEmptyStr
    chunk_size: Annotated[int, Field(strict=True, ge=STREAM_CHUNK_SIZE,
                                     le=STREAM_CHUNK_SIZE)] = STREAM_CHUNK_SIZE
    action_period_ns: Annotated[int, Field(strict=True, ge=STREAM_PERIOD_NS,
                                           le=STREAM_PERIOD_NS)] = STREAM_PERIOD_NS
    target_count: Annotated[int, Field(strict=True, ge=2, le=128)] = 5
    initial_lead_ns: PositiveInt = 250_000_000
    max_chunks: Annotated[PositiveInt, Field(le=MAX_CHUNKS)] = 8
    total_sim_deadline_ns: PositiveInt = 8_000_000_000
    total_wall_deadline_s: PositiveFloat = 45.0
    connect_timeout_s: PositiveFloat = 2.0
    open_deadline_s: PositiveFloat = 2.0
    predict_deadline_s: PositiveFloat = 2.0
    close_deadline_s: PositiveFloat = 2.0
    max_message_bytes: PositiveInt = 16_777_216
    poll_interval_s: PositiveFloat = 0.02
    termination_mode: Literal["finite_chunks", "task"] = "finite_chunks"
    task_stop_margin_s: PositiveFloat = 1.0
    task_sim_budget_ns: Annotated[PositiveInt, Field(le=MAX_TASK_BUDGET_NS)] = 30_000_000_000
    startup_settle_sim_ns: Annotated[NonNegativeInt, Field(le=5_000_000_000)] = 0

    @model_validator(mode="after")
    def _policy(self) -> NativePolicy:
        if self.expected_test_only and not self.allow_test_policy:
            raise contract_error("allow_test_policy",
                                 "a test-only policy requires allow_test_policy: true")
        return self


class NativeTeacher(StrictMappingModel):
    """Program-teacher stream identity, independent of policy RPC settings."""

    chunk_size: Annotated[int, Field(strict=True, ge=STREAM_CHUNK_SIZE,
                                     le=STREAM_CHUNK_SIZE)] = STREAM_CHUNK_SIZE
    action_period_ns: Annotated[int, Field(strict=True, ge=STREAM_PERIOD_NS,
                                           le=STREAM_PERIOD_NS)] = STREAM_PERIOD_NS
    target_count: Annotated[int, Field(strict=True, ge=2, le=128)] = 5
    initial_lead_ns: PositiveInt = 250_000_000
    max_chunks: Annotated[PositiveInt, Field(le=MAX_CHUNKS)] = 128
    total_sim_deadline_ns: PositiveInt = 30_000_000_000
    total_wall_deadline_s: PositiveFloat = 60.0
    prepare_timeout_s: PositiveFloat = 12.0
    start_timeout_s: PositiveFloat = 5.0
    chunk_deadline_s: PositiveFloat = 2.0
    poll_interval_s: PositiveFloat = 0.02
    termination_mode: Literal["finite_chunks", "task"] = "task"
    task_stop_margin_s: PositiveFloat = 1.0
    task_sim_budget_ns: Annotated[PositiveInt, Field(le=MAX_TASK_BUDGET_NS)] = 30_000_000_000
