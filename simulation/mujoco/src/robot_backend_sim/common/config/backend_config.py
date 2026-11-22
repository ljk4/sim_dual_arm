"""Strict configuration for the runnable simulation backend."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Annotated, Literal

from pydantic import Field, model_validator
from robot_contracts.validation import (
    NonEmptyStr,
    NonNegativeInt,
    PositiveFloat,
    PositiveInt,
    StrictMappingError,
    StrictMappingModel,
    UniqueKeyError,
    contract_error,
    read_unique_yaml,
    validate_config_model,
)
from robot_run_logging import LogSettings

__all__ = [
    "BackendConfig",
    "BackendConfigError",
    "load_backend_config",
    "load_backend_document",
]

BackendConfigError = StrictMappingError


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
    task_type: NonEmptyStr | None = None
    task_mode: Literal["teacher", "inference"] | None = None
    render_quality: RenderQualitySection = RenderQualitySection()
    velocity_feedforward: bool = False
    reliable_sensors: bool = False
    visualization_enabled: bool = True

    @model_validator(mode="after")
    def _names(self) -> RobotSection:
        text = self.namespace
        if len(text) > 128 or not text.startswith("/") or text == "/" or text.endswith("/"):
            raise contract_error("namespace", "must be an absolute ROS namespace")
        if any(not _NAMESPACE_SEGMENT.fullmatch(part) for part in text[1:].split("/")):
            raise contract_error("namespace", "segments must be ASCII letters, digits and '_'")
        if (self.task is None) != (self.task_mode is None):
            raise contract_error("task_mode", "must be set together with task")
        if self.task is not None and self.task_type is None:
            raise contract_error("task_type", "is required when task is set")
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
    follower_references: Annotated[PositiveInt, Field(le=_MAX_QUEUE)]


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


class BackendConfig(StrictMappingModel):
    """One fully validated backend configuration."""

    source: Path | None = None
    robot: RobotSection
    run: RunSection
    queues: QueueSection
    limits: LimitsSection
    logging: LoggingSection
    profile_path: Path | None = Field(default=None, exclude=True)
    schemas_path: Path | None = Field(default=None, exclude=True)
    scene_path: Path | None = Field(default=None, exclude=True)
    control_path: Path | None = Field(default=None, exclude=True)
    stream_path: Path | None = Field(default=None, exclude=True)
    follower_path: Path | None = Field(default=None, exclude=True)
    task_path: Path | None = Field(default=None, exclude=True)

    namespace = property(lambda self: self.robot.namespace)
    task_mode = property(lambda self: self.robot.task_mode)
    task_type = property(lambda self: self.robot.task_type)
    reliable_sensors = property(lambda self: self.robot.reliable_sensors)
    velocity_feedforward = property(lambda self: self.robot.velocity_feedforward)
    render_quality_enabled = property(lambda self: self.robot.render_quality.enabled)
    visualization_enabled = property(lambda self: self.robot.visualization_enabled)
    seed = property(lambda self: self.run.seed)
    capture_hz = property(lambda self: self.run.capture_hz)
    realtime_speed = property(lambda self: self.run.realtime_speed)
    max_catchup_steps = property(lambda self: self.run.max_catchup_steps)
    owner_queue_capacity = property(lambda self: self.queues.owner)
    planner_queue_capacity = property(lambda self: self.queues.planner)
    render_snapshot_capacity = property(lambda self: self.queues.render_snapshots)
    render_output_capacity = property(lambda self: self.queues.render_outputs)
    capture_queue_capacity = property(lambda self: self.queues.captures)
    event_queue_capacity = property(lambda self: self.queues.events)
    feedback_queue_capacity = property(lambda self: self.queues.feedback)
    follower_reference_queue_capacity = property(lambda self: self.queues.follower_references)
    command_history_capacity = property(lambda self: self.limits.command_history)
    chunk_history_capacity = property(lambda self: self.limits.chunk_history)
    planner_health_timeout_s = property(lambda self: self.limits.planner_health_s)
    renderer_health_timeout_s = property(lambda self: self.limits.renderer_health_s)
    request_timeout_s = property(lambda self: self.limits.request_timeout_s)
    shutdown_timeout_s = property(lambda self: self.limits.shutdown_s)
    log = property(lambda self: LogSettings(**self.logging.model_dump()))


def load_backend_config(path: Path | str) -> BackendConfig:
    """Load and fully validate one backend YAML file."""
    source_path = Path(path).resolve()
    try:
        document = read_unique_yaml(source_path)
    except UniqueKeyError as error:
        raise BackendConfigError(error.source, error.field, error.message) from error
    return load_backend_document(document, source_path)


def load_backend_document(document: object, source: Path | str) -> BackendConfig:
    """Validate an in-memory backend document with the same strict model."""
    source_path = Path(source)
    config = validate_config_model(BackendConfig, document, source_path, BackendConfigError)
    updates: dict[str, object] = {"source": source_path}
    for name, directory in (("profile", False), ("schemas", True), ("scene", False),
                            ("control", False), ("stream", False),
                            ("follower", False), ("task", False)):
        raw = getattr(config.robot, name)
        if raw is None:
            continue
        candidate = Path(raw)
        resolved = (candidate if candidate.is_absolute()
                    else source_path.parent / candidate).resolve()
        if not (resolved.is_dir() if directory else resolved.is_file()):
            kind = "directory" if directory else "file"
            raise BackendConfigError(source_path, f"robot.{name}",
                                     f"must be an existing {kind}: {resolved}")
        updates[f"{name}_path"] = resolved
    return config.model_copy(update=updates)
