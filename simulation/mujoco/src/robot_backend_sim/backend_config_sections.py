"""Nested Pydantic sections for the backend deployment configuration."""

from __future__ import annotations

import re
from typing import Annotated, Literal

from pydantic import Field, model_validator
from robot_contracts.validation import (
    NonEmptyStr,
    NonNegativeInt,
    PositiveFloat,
    PositiveInt,
    StrictMappingModel,
    contract_error,
)

_NAMESPACE_SEGMENT = re.compile(r"^[A-Za-z0-9_]+$")
_INT64_MAX = 2**63 - 1
_MAX_HISTORY = 4096
_MAX_QUEUE = 4096


class RenderQualitySection(StrictMappingModel):
    enabled: bool = False


class RobotSection(StrictMappingModel):
    namespace: NonEmptyStr
    profile: NonEmptyStr
    schemas: NonEmptyStr
    scene: NonEmptyStr
    control: NonEmptyStr
    stream: NonEmptyStr
    follower: NonEmptyStr | None = None
    task: NonEmptyStr | None = None
    task_mode: Literal["teacher", "inference"] | None = None
    render_quality: RenderQualitySection = RenderQualitySection()
    velocity_feedforward: bool = False
    reliable_sensors: bool = False

    @model_validator(mode="after")
    def _names(self) -> RobotSection:
        text = self.namespace
        if len(text) > 128 or not text.startswith("/") or text == "/" or text.endswith("/"):
            raise contract_error("namespace", "must be an absolute ROS namespace")
        if any(not _NAMESPACE_SEGMENT.fullmatch(part) for part in text[1:].split("/")):
            raise contract_error("namespace", "segments must be ASCII letters, digits and '_'")
        if (self.task is None) != (self.task_mode is None):
            raise contract_error("task_mode", "must be set together with task")
        if self.render_quality.enabled and self.task is None:
            raise contract_error("render_quality.enabled", "requires task")
        return self


class RunSection(StrictMappingModel):
    seed: Annotated[NonNegativeInt, Field(le=_INT64_MAX)]
    capture_hz: Annotated[PositiveFloat, Field(gt=1.0, le=200.0)]
    realtime_speed: Annotated[PositiveFloat, Field(gt=0.1, le=100.0)]
    max_catchup_steps: Annotated[PositiveInt, Field(le=64)]


class QueueSection(StrictMappingModel):
    owner: Annotated[PositiveInt, Field(le=_MAX_QUEUE)]
    planner: Annotated[PositiveInt, Field(le=64)]
    render_snapshots: Annotated[PositiveInt, Field(le=64)]
    render_outputs: Annotated[PositiveInt, Field(le=64)]
    captures: Annotated[PositiveInt, Field(le=_MAX_QUEUE)]
    events: Annotated[PositiveInt, Field(le=_MAX_QUEUE)]
    feedback: Annotated[PositiveInt, Field(le=_MAX_QUEUE)]
    accepted_references: Annotated[PositiveInt, Field(le=_MAX_QUEUE)]
    follower_references: Annotated[PositiveInt, Field(le=_MAX_QUEUE)] | None = None


class LimitsSection(StrictMappingModel):
    planner_health_s: Annotated[PositiveFloat, Field(gt=0.1, le=120.0)]
    renderer_health_s: Annotated[PositiveFloat, Field(gt=0.1, le=120.0)]
    request_timeout_s: Annotated[PositiveFloat, Field(gt=0.1, le=120.0)]
    shutdown_s: Annotated[PositiveFloat, Field(gt=0.1, le=120.0)]
    command_history: Annotated[PositiveInt, Field(le=_MAX_HISTORY)] = 256
    chunk_history: Annotated[PositiveInt, Field(le=_MAX_HISTORY)] = 128


class LoggingSection(StrictMappingModel):
    queue_capacity: Annotated[PositiveInt, Field(le=_MAX_QUEUE)]
    max_event_bytes: Annotated[PositiveInt, Field(ge=1024, le=16 * 1024 * 1024)]
    max_file_bytes: Annotated[PositiveInt, Field(ge=1024, le=1024 * 1024 * 1024)]
    max_total_bytes: Annotated[PositiveInt, Field(ge=1024, le=16 * 1024 * 1024 * 1024)]
    close_timeout_s: Annotated[PositiveFloat, Field(gt=0.05, le=60.0)]
    flush_interval_s: Annotated[PositiveFloat, Field(gt=0.01, le=10.0)]
    console: bool

    @model_validator(mode="after")
    def _ordering(self) -> LoggingSection:
        if not self.max_event_bytes <= self.max_file_bytes <= self.max_total_bytes:
            raise contract_error("max_event_bytes",
                                 "must be <= max_file_bytes <= max_total_bytes")
        return self
