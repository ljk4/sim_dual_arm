"""Strict configuration for the runnable native simulation backend.

The YAML file owns deployment choices only: names, relative paths, timing,
bounded queue sizes and log limits.  Physics parameters remain in
``panda_stack.yaml``, controller parameters in ``panda_control.yaml`` and
stream timing in ``panda_stream.yaml``; this module never copies them.
Every path is resolved against the configuration file (or an explicit CLI
root), never against the process working directory.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from robot_run_logging import LogSettings

__all__ = ["BackendConfig", "BackendConfigError", "load_backend_config"]

_NAMESPACE_SEGMENT = re.compile(r"^[A-Za-z0-9_]+$")
_TOP_LEVEL = ("robot", "run", "queues", "limits", "logging")
_ROBOT_KEYS = ("namespace", "profile", "schemas", "scene", "control", "stream")
_ROBOT_OPTIONAL_KEYS = (
    "follower",
    "task",
    "task_mode",
    "render_quality",
    "velocity_feedforward",
    "reliable_sensors",
)
_RUN_KEYS = ("seed", "capture_hz", "realtime_speed", "max_catchup_steps")
_QUEUE_KEYS = (
    "owner",
    "planner",
    "render_snapshots",
    "render_outputs",
    "captures",
    "events",
    "feedback",
    "accepted_references",
)
_QUEUE_OPTIONAL_KEYS = ("follower_references",)
_LIMIT_KEYS = (
    "planner_health_s",
    "renderer_health_s",
    "request_timeout_s",
    "shutdown_s",
    "command_history",
    "chunk_history",
)
_LIMIT_OPTIONAL_KEYS = ("command_history", "chunk_history")
_LIMIT_REQUIRED_KEYS = tuple(key for key in _LIMIT_KEYS if key not in _LIMIT_OPTIONAL_KEYS)
_MAX_HISTORY = 4096
_TASK_MODES = ("teacher", "inference")
_LOG_KEYS = (
    "queue_capacity",
    "max_event_bytes",
    "max_file_bytes",
    "max_total_bytes",
    "close_timeout_s",
    "flush_interval_s",
    "console",
)
_MAX_QUEUE = 4096


class BackendConfigError(ValueError):
    """A configuration problem with the source file and field path attached."""

    def __init__(self, source: str | Path, field: str, message: str) -> None:
        self.source = str(source)
        self.field = field
        self.message = message
        super().__init__(f"{self.source}: {field}: {message}")


class _UniqueKeyLoader(yaml.SafeLoader):
    """Safe loader that rejects duplicate mapping keys instead of dropping one."""

    source_name = "<backend config>"


def _construct_unique_mapping(
    loader: _UniqueKeyLoader, node: yaml.MappingNode, deep: bool = False
) -> dict[Any, Any]:
    loader.flatten_mapping(node)
    mapping: dict[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        try:
            duplicate = key in mapping
        except TypeError as error:
            raise BackendConfigError(
                loader.source_name, "<mapping>", "keys must be hashable"
            ) from error
        if duplicate:
            raise BackendConfigError(loader.source_name, str(key), "duplicate key")
        mapping[key] = loader.construct_object(value_node, deep=deep)
    return mapping


_UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_unique_mapping
)


@dataclass(frozen=True, slots=True)
class BackendConfig:
    """One fully validated backend configuration."""

    source: Path
    namespace: str
    profile_path: Path
    schemas_path: Path
    scene_path: Path
    control_path: Path
    stream_path: Path
    follower_path: Path | None
    task_path: Path | None
    task_mode: str | None
    reliable_sensors: bool
    velocity_feedforward: bool
    render_quality_enabled: bool
    command_history_capacity: int
    chunk_history_capacity: int
    seed: int
    capture_hz: float
    realtime_speed: float
    max_catchup_steps: int
    owner_queue_capacity: int
    planner_queue_capacity: int
    render_snapshot_capacity: int
    render_output_capacity: int
    capture_queue_capacity: int
    event_queue_capacity: int
    feedback_queue_capacity: int
    accepted_reference_queue_capacity: int
    follower_reference_queue_capacity: int
    planner_health_timeout_s: float
    renderer_health_timeout_s: float
    request_timeout_s: float
    shutdown_timeout_s: float
    log: LogSettings


def _field(parent: str, child: str) -> str:
    return f"{parent}.{child}" if parent else child


def _mapping(value: object, source: str, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise BackendConfigError(source, field, "must be a mapping")
    if any(not isinstance(key, str) for key in value):
        raise BackendConfigError(source, field, "keys must be strings")
    return value


def _check_keys(
    document: dict[str, Any],
    source: str,
    field: str,
    required: tuple[str, ...],
    optional: tuple[str, ...] = (),
) -> None:
    allowed = set(required) | set(optional)
    for key in document:
        if key not in allowed:
            raise BackendConfigError(source, _field(field, key), "unknown key")
    for key in required:
        if key not in document:
            raise BackendConfigError(source, _field(field, key), "missing required key")


def _integer(
    value: object,
    source: str,
    field: str,
    *,
    minimum: int,
    maximum: int,
) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise BackendConfigError(source, field, "must be an integer; booleans are not accepted")
    result = int(value)
    if result < minimum or result > maximum:
        raise BackendConfigError(source, field, f"must be in {minimum}..{maximum}")
    return result


def _number(
    value: object,
    source: str,
    field: str,
    *,
    minimum: float,
    maximum: float,
    strict_minimum: bool = False,
    maximum_inclusive: bool = True,
) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise BackendConfigError(source, field, "must be a number; booleans are not accepted")
    try:
        result = float(value)
    except OverflowError as error:
        raise BackendConfigError(source, field, "must be finite") from error
    if not math.isfinite(result):
        raise BackendConfigError(source, field, "must be finite")
    if strict_minimum and result <= minimum:
        raise BackendConfigError(source, field, f"must be greater than {minimum}")
    if not strict_minimum and result < minimum:
        raise BackendConfigError(source, field, f"must be at least {minimum}")
    if maximum_inclusive and result > maximum:
        raise BackendConfigError(source, field, f"must be at most {maximum}")
    if not maximum_inclusive and result >= maximum:
        raise BackendConfigError(source, field, f"must be less than {maximum}")
    return result


def _boolean(value: object, source: str, field: str) -> bool:
    if not isinstance(value, bool):
        raise BackendConfigError(source, field, "must be a boolean")
    return value


def _text(value: object, source: str, field: str) -> str:
    if not isinstance(value, str) or not value:
        raise BackendConfigError(source, field, "must be a non-empty string")
    return value


def _namespace(value: object, source: str) -> str:
    text = _text(value, source, "robot.namespace")
    if len(text) > 128:
        raise BackendConfigError(source, "robot.namespace", "must be at most 128 characters")
    if not text.startswith("/") or text == "/" or text.endswith("/"):
        raise BackendConfigError(
            source, "robot.namespace", "must start with '/' and name at least one segment"
        )
    segments = text[1:].split("/")
    if any(not _NAMESPACE_SEGMENT.fullmatch(segment) for segment in segments):
        raise BackendConfigError(
            source,
            "robot.namespace",
            "segments must contain only ASCII letters, digits and '_'",
        )
    return text


def _path(
    value: object,
    source_path: Path,
    field: str,
    *,
    directory: bool,
) -> Path:
    text = _text(value, source_path, field)
    candidate = Path(text)
    resolved = (candidate if candidate.is_absolute() else source_path.parent / candidate).resolve()
    if directory and not resolved.is_dir():
        raise BackendConfigError(source_path, field, f"must be an existing directory: {resolved}")
    if not directory and not resolved.is_file():
        raise BackendConfigError(source_path, field, f"must be an existing file: {resolved}")
    return resolved


def _log_settings(value: object, source: str) -> LogSettings:
    document = _mapping(value, source, "logging")
    _check_keys(document, source, "logging", _LOG_KEYS)
    try:
        return LogSettings(
            queue_capacity=_integer(
                document["queue_capacity"],
                source,
                "logging.queue_capacity",
                minimum=1,
                maximum=_MAX_QUEUE,
            ),
            max_event_bytes=_integer(
                document["max_event_bytes"],
                source,
                "logging.max_event_bytes",
                minimum=1024,
                maximum=16 * 1024 * 1024,
            ),
            max_file_bytes=_integer(
                document["max_file_bytes"],
                source,
                "logging.max_file_bytes",
                minimum=1024,
                maximum=1024 * 1024 * 1024,
            ),
            max_total_bytes=_integer(
                document["max_total_bytes"],
                source,
                "logging.max_total_bytes",
                minimum=1024,
                maximum=16 * 1024 * 1024 * 1024,
            ),
            close_timeout_s=_number(
                document["close_timeout_s"],
                source,
                "logging.close_timeout_s",
                minimum=0.05,
                maximum=60.0,
                strict_minimum=True,
            ),
            flush_interval_s=_number(
                document["flush_interval_s"],
                source,
                "logging.flush_interval_s",
                minimum=0.01,
                maximum=10.0,
                strict_minimum=True,
            ),
            console=_boolean(document["console"], source, "logging.console"),
        )
    except ValueError as error:
        # LogSettings enforces cross-field bounds; keep the YAML field attached.
        raise BackendConfigError(source, "logging", str(error)) from error


def load_backend_config(path: Path | str) -> BackendConfig:
    """Load and fully validate one backend YAML file.

    Unknown/duplicate keys, booleans standing in for numbers, non-finite
    values, out-of-range queues/deadlines and missing referenced files are all
    rejected before MuJoCo is touched.
    """
    source_path = Path(path).resolve()
    source = str(source_path)
    try:
        text = source_path.read_text(encoding="utf-8")
    except (OSError, UnicodeError) as error:
        raise BackendConfigError(source, "<file>", f"cannot read configuration: {error}") from error
    loader = _UniqueKeyLoader(text)
    loader.source_name = source
    try:
        raw = loader.get_single_data()
    except BackendConfigError:
        raise
    except yaml.YAMLError as error:
        raise BackendConfigError(source, "<yaml>", f"invalid YAML: {error}") from error
    finally:
        loader.dispose()
    root = _mapping(raw, source, "<root>")
    _check_keys(root, source, "", _TOP_LEVEL)

    robot = _mapping(root["robot"], source, "robot")
    _check_keys(robot, source, "robot", _ROBOT_KEYS, _ROBOT_OPTIONAL_KEYS)
    run = _mapping(root["run"], source, "run")
    _check_keys(run, source, "run", _RUN_KEYS)
    queues = _mapping(root["queues"], source, "queues")
    _check_keys(queues, source, "queues", _QUEUE_KEYS, _QUEUE_OPTIONAL_KEYS)
    limits = _mapping(root["limits"], source, "limits")
    _check_keys(limits, source, "limits", _LIMIT_REQUIRED_KEYS, _LIMIT_OPTIONAL_KEYS)
    task_path = (
        None
        if "task" not in robot
        else _path(robot["task"], source_path, "robot.task", directory=False)
    )
    task_mode = (
        None if "task_mode" not in robot else _text(robot["task_mode"], source, "robot.task_mode")
    )
    if task_path is None and task_mode is not None:
        raise BackendConfigError(source, "robot.task_mode", "requires robot.task")
    if task_path is not None and task_mode is None:
        raise BackendConfigError(source, "robot.task_mode", "is required with robot.task")
    if task_mode is not None and task_mode not in _TASK_MODES:
        raise BackendConfigError(source, "robot.task_mode", f"must be one of {_TASK_MODES}")
    quality = _mapping(robot.get("render_quality", {}), source, "robot.render_quality")
    _check_keys(quality, source, "robot.render_quality", (), ("enabled",))
    render_quality_enabled = _boolean(
        quality.get("enabled", False), source, "robot.render_quality.enabled"
    )
    if render_quality_enabled and task_path is None:
        raise BackendConfigError(source, "robot.render_quality.enabled", "requires robot.task")

    return BackendConfig(
        source=source_path,
        namespace=_namespace(robot["namespace"], source),
        profile_path=_path(robot["profile"], source_path, "robot.profile", directory=False),
        schemas_path=_path(robot["schemas"], source_path, "robot.schemas", directory=True),
        scene_path=_path(robot["scene"], source_path, "robot.scene", directory=False),
        control_path=_path(robot["control"], source_path, "robot.control", directory=False),
        stream_path=_path(robot["stream"], source_path, "robot.stream", directory=False),
        follower_path=(
            None
            if "follower" not in robot
            else _path(robot["follower"], source_path, "robot.follower", directory=False)
        ),
        task_path=task_path,
        task_mode=task_mode,
        render_quality_enabled=render_quality_enabled,
        reliable_sensors=_boolean(
            robot.get("reliable_sensors", False), source, "robot.reliable_sensors"
        ),
        velocity_feedforward=_boolean(
            robot.get("velocity_feedforward", False), source, "robot.velocity_feedforward"
        ),
        command_history_capacity=_integer(
            limits.get("command_history", 256),
            source,
            "limits.command_history",
            minimum=1,
            maximum=_MAX_HISTORY,
        ),
        chunk_history_capacity=_integer(
            limits.get("chunk_history", 128),
            source,
            "limits.chunk_history",
            minimum=1,
            maximum=_MAX_HISTORY,
        ),
        seed=_integer(run["seed"], source, "run.seed", minimum=0, maximum=2**63 - 1),
        capture_hz=_number(
            run["capture_hz"],
            source,
            "run.capture_hz",
            minimum=1.0,
            maximum=200.0,
            strict_minimum=True,
        ),
        realtime_speed=_number(
            run["realtime_speed"],
            source,
            "run.realtime_speed",
            minimum=0.1,
            maximum=100.0,
            strict_minimum=True,
        ),
        max_catchup_steps=_integer(
            run["max_catchup_steps"],
            source,
            "run.max_catchup_steps",
            minimum=1,
            maximum=64,
        ),
        owner_queue_capacity=_integer(
            queues["owner"], source, "queues.owner", minimum=1, maximum=_MAX_QUEUE
        ),
        planner_queue_capacity=_integer(
            queues["planner"], source, "queues.planner", minimum=1, maximum=64
        ),
        render_snapshot_capacity=_integer(
            queues["render_snapshots"],
            source,
            "queues.render_snapshots",
            minimum=1,
            maximum=64,
        ),
        render_output_capacity=_integer(
            queues["render_outputs"],
            source,
            "queues.render_outputs",
            minimum=1,
            maximum=64,
        ),
        capture_queue_capacity=_integer(
            queues["captures"], source, "queues.captures", minimum=1, maximum=_MAX_QUEUE
        ),
        event_queue_capacity=_integer(
            queues["events"], source, "queues.events", minimum=1, maximum=_MAX_QUEUE
        ),
        feedback_queue_capacity=_integer(
            queues["feedback"], source, "queues.feedback", minimum=1, maximum=_MAX_QUEUE
        ),
        accepted_reference_queue_capacity=_integer(
            queues["accepted_references"],
            source,
            "queues.accepted_references",
            minimum=1,
            maximum=_MAX_QUEUE,
        ),
        follower_reference_queue_capacity=_integer(
            queues.get("follower_references", queues["accepted_references"]),
            source,
            "queues.follower_references",
            minimum=1,
            maximum=_MAX_QUEUE,
        ),
        planner_health_timeout_s=_number(
            limits["planner_health_s"],
            source,
            "limits.planner_health_s",
            minimum=0.1,
            maximum=120.0,
            strict_minimum=True,
        ),
        renderer_health_timeout_s=_number(
            limits["renderer_health_s"],
            source,
            "limits.renderer_health_s",
            minimum=0.1,
            maximum=120.0,
            strict_minimum=True,
        ),
        request_timeout_s=_number(
            limits["request_timeout_s"],
            source,
            "limits.request_timeout_s",
            minimum=0.1,
            maximum=120.0,
            strict_minimum=True,
        ),
        shutdown_timeout_s=_number(
            limits["shutdown_s"],
            source,
            "limits.shutdown_s",
            minimum=0.1,
            maximum=120.0,
            strict_minimum=True,
        ),
        log=_log_settings(root["logging"], source),
    )
