"""Strict configuration for the runnable simulation backend."""

from __future__ import annotations

from pathlib import Path

from pydantic import Field
from robot_contracts.validation import (
    StrictMappingError,
    StrictMappingModel,
    UniqueKeyError,
    read_unique_yaml,
    validate_config_model,
)
from robot_run_logging import LogSettings

from .backend_config_sections import (
    LimitsSection,
    LoggingSection,
    QueueSection,
    RobotSection,
    RunSection,
)

__all__ = ["BackendConfig", "BackendConfigError", "load_backend_config"]

BackendConfigError = StrictMappingError


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
    reliable_sensors = property(lambda self: self.robot.reliable_sensors)
    velocity_feedforward = property(lambda self: self.robot.velocity_feedforward)
    render_quality_enabled = property(lambda self: self.robot.render_quality.enabled)
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
    accepted_reference_queue_capacity = property(lambda self: self.queues.accepted_references)
    follower_reference_queue_capacity = property(
        lambda self: self.queues.follower_references
        if self.queues.follower_references is not None
        else self.queues.accepted_references)
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
