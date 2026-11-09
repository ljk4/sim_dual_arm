"""Process run identity, metadata, and explicit event-emission helpers.

This module deliberately owns only deployment-time concerns: the exclusive run
directory, ``resolved/runtime.json``, ``versions.json`` and the one
``EventLogger`` for this runtime process.  It does not call ROS, change the
robot configuration, or intercept unrelated log records.
"""

from __future__ import annotations

import hashlib
import importlib
import importlib.metadata
import json
import os
import platform
import re
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

from robot_run_logging import (
    EventContext,
    EventLogger,
    LogFailure,
    LogSettings,
    Scalar,
)

_RUN_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
_DEPENDENCY_DISTRIBUTIONS = (
    "dual-arm-robot-contracts",
    "dual-arm-run-logging",
    "dual-arm-ros-support",
    "PyYAML",
    "jsonschema",
)
_SOURCE_MODULES = (
    "dual_arm_runtime",
    "robot_contracts",
    "robot_run_logging",
    "robot_ros_support",
)


class EventSink(Protocol):
    """The narrow EventLogger surface the runtime calls explicitly."""

    def emit(
        self,
        event: str,
        *,
        message: str = "",
        level: str = "INFO",
        reason: str | None = None,
        context: EventContext | None = None,
        fields: dict[str, Scalar] | None = None,
        exception: BaseException | None = None,
        critical: bool = True,
    ) -> bool: ...

    def check_health(self) -> None: ...


@dataclass(frozen=True)
class RunSession:
    """The deployment identity and logger owned by one runtime process."""

    run_id: str
    instance_id: str
    output: Path
    run_directory: Path
    log_directory: Path
    logger: EventLogger


def new_run_id() -> str:
    return str(uuid.uuid4())


def validate_run_id(value: str) -> str:
    """Reject path components and identifiers unsafe for one run directory."""
    if not isinstance(value, str) or not _RUN_ID_PATTERN.fullmatch(value):
        raise ValueError(
            "run_id must contain 1..128 ASCII letters, digits, '.', '_' or '-', "
            "and must start with a letter or digit"
        )
    if value in {".", ".."}:
        raise ValueError("run_id must not be '.' or '..'")
    return value


def emit_event(
    sink: EventSink | None,
    event: str,
    *,
    message: str = "",
    level: str = "INFO",
    reason: str | None = None,
    context: EventContext | None = None,
    fields: dict[str, Scalar] | None = None,
    exception: BaseException | None = None,
    critical: bool = True,
) -> None:
    """Emit one event and turn any delivery/validation doubt into LogFailure."""
    if sink is None:
        return
    try:
        sink.emit(
            event,
            message=message,
            level=level,
            reason=reason,
            context=context,
            fields=fields,
            exception=exception,
            critical=critical,
        )
    except LogFailure:
        raise
    except Exception as error:
        raise LogFailure(f"{event}: {type(error).__name__}: {error}") from error


def emit_best_effort(
    sink: EventSink | None,
    event: str,
    *,
    message: str = "",
    level: str = "INFO",
    reason: str | None = None,
    context: EventContext | None = None,
    fields: dict[str, Scalar] | None = None,
    exception: BaseException | None = None,
    critical: bool = True,
) -> str | None:
    """Teardown-only helper: report a logging error without a recursive emit."""
    try:
        emit_event(
            sink,
            event,
            message=message,
            level=level,
            reason=reason,
            context=context,
            fields=fields,
            exception=exception,
            critical=critical,
        )
    except Exception as error:
        return f"{type(error).__name__}: {error}"
    return None


def check_health(sink: EventSink | None) -> None:
    """Raise LogFailure when the asynchronous writer has already failed."""
    if sink is None:
        return
    try:
        sink.check_health()
    except LogFailure:
        raise
    except Exception as error:
        raise LogFailure(f"logging health check: {type(error).__name__}: {error}") from error


def initialize_run(
    configuration: object,
    *,
    run_id: str,
    runs_output: Path,
    output: Path,
) -> RunSession:
    """Create the run directory exclusively and write metadata before any control."""
    validate_run_id(run_id)
    effective_settings = configuration.recipe.logging
    # Resolve only the container directory.  The final path component is a
    # validated run_id and is created exclusively in that directory, so an
    # existing file/symlink (including a dangling one) is never followed.
    run_parent = Path(runs_output).resolve()
    run_parent.mkdir(parents=True, exist_ok=True)
    run_directory = run_parent / run_id
    run_directory.mkdir(exist_ok=False)
    instance_id = str(uuid.uuid4())
    resolved = {
        "schema_version": 1,
        "run_id": run_id,
        "instance_id": instance_id,
        **configuration.snapshot,
    }
    versions = collect_versions(
        configuration,
        run_id=run_id,
        instance_id=instance_id,
        settings=effective_settings,
    )
    _write_json_atomic(run_directory / "resolved" / "runtime.json", resolved)
    _write_json_atomic(run_directory / "versions.json", versions)
    log_directory = run_directory / "logs" / "runtime" / instance_id
    logger = EventLogger(
        log_directory,
        run_id=run_id,
        component="runtime",
        settings=effective_settings,
        instance_id=instance_id,
    )
    return RunSession(
        run_id=run_id,
        instance_id=instance_id,
        output=Path(output),
        run_directory=run_directory,
        log_directory=log_directory,
        logger=logger,
    )


def collect_versions(
    configuration: object,
    *,
    run_id: str,
    instance_id: str,
    settings: LogSettings,
) -> dict[str, object]:
    """Collect a small, explicit provenance whitelist; never dump the environment."""
    snapshot = configuration.snapshot
    recipe = snapshot["recipe"]
    return {
        "schema_version": 1,
        "run_id": run_id,
        "instance_id": instance_id,
        "python": {
            "implementation": platform.python_implementation(),
            "version": platform.python_version(),
        },
        "dependencies": {name: _distribution_version(name) for name in _DEPENDENCY_DISTRIBUTIONS},
        "sources": {name: _source_path(name) for name in _SOURCE_MODULES},
        "configuration_sha256": _sha256(_canonical_json(snapshot)),
        "recipe_sha256": _sha256(_canonical_json(recipe)),
        "logging": {
            "queue_capacity": settings.queue_capacity,
            "max_event_bytes": settings.max_event_bytes,
            "max_file_bytes": settings.max_file_bytes,
            "max_total_bytes": settings.max_total_bytes,
            "close_timeout_s": settings.close_timeout_s,
            "flush_interval_s": settings.flush_interval_s,
            "console": settings.console,
        },
        # Filled by the launcher once an image/manifest exists. Unknown values
        # stay null rather than claiming a Git revision or source state.
        "image": {"digest": None},
        "git": {"revision": None, "dirty": None},
    }


def _canonical_json(document: object) -> str:
    return json.dumps(document, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _sha256(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _distribution_version(name: str) -> str | None:
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return None


def _source_path(name: str) -> str | None:
    try:
        module = importlib.import_module(name)
    except Exception:
        return None
    filename = getattr(module, "__file__", None)
    if not filename:
        return None
    return str(Path(filename).resolve())


def _write_json_atomic(path: Path, document: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    body = (json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()
    temporary = path.with_name(f".{path.name}.tmp")
    with temporary.open("wb") as stream:
        stream.write(body)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def runtime_summary(
    session: RunSession,
    *,
    record_directory: str = "",
    exit_code: int,
    process_outcome: str,
    logging_closed: bool,
) -> dict[str, object]:
    """Small machine-readable line printed by main for humans and old tooling."""
    return {
        "run_id": session.run_id,
        "run_directory": str(session.run_directory),
        "log_directory": str(session.log_directory),
        "record_directory": record_directory,
        "exit_code": exit_code,
        "process_outcome": process_outcome,
        "logging_closed": logging_closed,
    }
