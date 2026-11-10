"""ROS session, public-service client and manifest validation for control_cli."""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import stat
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from robot_api_msgs.msg import RuntimeStatus
from robot_api_msgs.srv import EndEpisode, StartEpisode
from robot_contracts.episodes import (
    EPISODE_SCHEMA_VERSION,
    RecordingResult,
    RuntimeOutcome,
    RuntimeResult,
    TaskOutcome,
    TaskResult,
)
from robot_ros_support.qos import INFO_QOS

STATUS_TOPIC = "/runtime/status"
START_SERVICE = "/runtime/episode/start"
END_SERVICE = "/runtime/episode/end"
RAW_ROOT = Path("/data/raw")
RUN_ID_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}")
EPISODE_ID_PATTERN = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}")
TERMINAL_PHASES = frozenset({"IDLE", "FAULT"})
ACTIVE_PHASES = frozenset({"PREPARING", "ACTIVE", "ENDING"})
MAX_MANIFEST_BYTES = 2 * 1024 * 1024


class CliError(Exception):
    """A stable, reportable failure for one CLI operation."""

    def __init__(self, reason: str, detail: str = "", **fields: Any) -> None:
        super().__init__(detail or reason)
        self.reason = reason
        self.detail = detail
        self.fields = fields


class UsageError(Exception):
    """Argument parsing failed; main still emits one JSON object."""


class _Parser(argparse.ArgumentParser):
    def error(self, message: str) -> None:
        raise UsageError(message)


def positive_timeout(value: object) -> float:
    """Reject bool, non-finite and non-positive deadlines at every entry point."""
    if isinstance(value, bool):
        raise ValueError("timeout must be a finite positive number of seconds")
    try:
        seconds = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError("timeout must be a finite positive number of seconds") from error
    if not math.isfinite(seconds) or seconds <= 0:
        raise ValueError("timeout must be a finite positive number of seconds")
    return seconds


def timeout_argument(value: str) -> float:
    try:
        return positive_timeout(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error


def validate_run_id(value: str) -> str:
    if not isinstance(value, str) or not RUN_ID_PATTERN.fullmatch(value):
        raise ValueError(
            "run id must contain 1..128 ASCII letters, digits, '.', '_' or '-', "
            "and must start with a letter or digit"
        )
    if value in {".", ".."} or ".." in value:
        raise ValueError("run id must not be '.' or '..' or contain '..'")
    return value


def validate_episode_id(value: str) -> str:
    if not isinstance(value, str) or not EPISODE_ID_PATTERN.fullmatch(value):
        raise ValueError("episode id must be a safe non-empty path segment")
    if value in {".", ".."} or ".." in value:
        raise ValueError("episode id must not contain '..'")
    return value


@dataclass(frozen=True)
class StatusSnapshot:
    phase: str
    episode_id: str
    reason: str
    record_directory: str

    def as_json(self) -> dict[str, str]:
        return {
            "phase": self.phase,
            "episode_id": self.episode_id,
            "reason": self.reason,
            "record_directory": self.record_directory,
        }


@dataclass(frozen=True)
class Ack:
    success: bool
    reason: str
    episode_id: str = ""

    def as_json(self) -> dict[str, Any]:
        value: dict[str, Any] = {"success": self.success, "reason": self.reason}
        if self.episode_id:
            value["episode_id"] = self.episode_id
        return value


class _RosSession:
    """Explicit context/executor/node lifetime for one command."""

    def __init__(self) -> None:
        self.context = Context()
        self.node: Node | None = None
        self.executor: SingleThreadedExecutor | None = None
        self.initialized = False

    def __enter__(self) -> _RosSession:
        rclpy.init(
            args=[],
            context=self.context,
            signal_handler_options=SignalHandlerOptions.NO,
        )
        self.initialized = True
        try:
            self.node = Node("dual_arm_runtime_cli", context=self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
        except BaseException:
            self._close()
            raise
        return self

    def __exit__(self, _type, _value, _traceback) -> None:
        self._close()

    def _close(self) -> None:
        if self.executor is not None:
            if self.node is not None:
                try:
                    self.executor.remove_node(self.node)
                except Exception as error:
                    print(f"cli cleanup: cannot remove ROS node: {error}", file=sys.stderr)
            try:
                self.executor.shutdown()
            except Exception as error:
                print(f"cli cleanup: cannot shut down executor: {error}", file=sys.stderr)
        if self.node is not None:
            try:
                self.node.destroy_node()
            except Exception as error:
                print(f"cli cleanup: cannot destroy ROS node: {error}", file=sys.stderr)
        if self.initialized:
            try:
                rclpy.shutdown(context=self.context)
            except Exception as error:
                print(f"cli cleanup: cannot shut down rclpy: {error}", file=sys.stderr)
        self.executor, self.node, self.initialized = None, None, False


class _RuntimeClient:
    """Minimal public-ROS surface used by the command implementations."""

    def __init__(self, session: _RosSession, raw_root: Path) -> None:
        if session.node is None or session.executor is None:
            raise RuntimeError("ROS session is not active")
        self._node = session.node
        self._executor = session.executor
        self._raw_root = raw_root
        self._status: StatusSnapshot | None = None
        self._start = self._node.create_client(StartEpisode, START_SERVICE)
        self._end = self._node.create_client(EndEpisode, END_SERVICE)
        self._subscription = self._node.create_subscription(
            RuntimeStatus,
            STATUS_TOPIC,
            self._on_status,
            INFO_QOS,
        )

    def _on_status(self, message: RuntimeStatus) -> None:
        self._status = StatusSnapshot(
            str(message.phase),
            str(message.episode_id),
            str(message.reason),
            str(message.record_directory),
        )

    @property
    def status(self) -> StatusSnapshot | None:
        return self._status

    def services(self) -> dict[str, bool]:
        return {
            "start": bool(self._start.service_is_ready()),
            "end": bool(self._end.service_is_ready()),
        }

    def _spin_until(self, predicate, deadline: float, reason: str, detail: str) -> None:
        while not predicate():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise CliError(reason, detail, status=self._status_json(), services=self.services())
            try:
                self._executor.spin_once(timeout_sec=min(0.1, remaining))
            except Exception as error:
                raise CliError(
                    "ROS_ERROR",
                    f"{type(error).__name__}: {error}",
                    status=self._status_json(),
                    services=self.services(),
                ) from error

    def wait_ready(
        self,
        deadline: float,
        *,
        require_status: bool,
        require_services: tuple[str, ...],
    ) -> None:
        def ready() -> bool:
            services = self.services()
            if require_status and self._status is None:
                return False
            return all(services[name] for name in require_services)

        try:
            self._spin_until(
                ready,
                deadline,
                "SERVICE_UNAVAILABLE",
                "runtime status or public services were not discovered",
            )
        except CliError as error:
            if self._status is None:
                error.reason = "STATUS_UNAVAILABLE"
            raise

    def wait_status(self, deadline: float) -> StatusSnapshot:
        self._spin_until(
            lambda: self._status is not None,
            deadline,
            "STATUS_UNAVAILABLE",
            "runtime has not published a status",
        )
        assert self._status is not None
        return self._status

    def wait_for_target(self, episode_id: str, deadline: float) -> StatusSnapshot:
        """Wait briefly for a just-started episode's status to replace a stale one."""
        status = self.wait_status(deadline)
        if status.episode_id == episode_id:
            return status

        def target() -> bool:
            current = self._status
            return current is not None and current.episode_id == episode_id

        try:
            self._spin_until(target, deadline, "EPISODE_MISMATCH", "status names another episode")
        except CliError as error:
            if error.reason == "EPISODE_MISMATCH":
                error.reason = "WAIT_TIMEOUT"
            raise
        assert self._status is not None
        return self._status

    def call_start(self, task_id: str, deadline: float) -> Ack:
        return self._call(
            self._start,
            StartEpisode.Request(task_id=task_id),
            deadline,
            "START_RPC_TIMEOUT",
        )

    def call_end(self, reason: str, deadline: float) -> Ack:
        return self._call(
            self._end,
            EndEpisode.Request(reason=reason),
            deadline,
            "END_RPC_TIMEOUT",
        )

    def _call(self, client, request, deadline: float, timeout_reason: str) -> Ack:
        try:
            future = client.call_async(request)
        except Exception as error:
            raise CliError(
                "RPC_ERROR",
                f"{type(error).__name__}: {error}",
                status=self._status_json(),
            ) from error
        try:
            self._spin_until(
                future.done,
                deadline,
                timeout_reason,
                "the request may have executed; its outcome is unknown",
            )
        except CliError as error:
            error.fields["may_have_executed"] = True
            raise
        try:
            response = future.result()
        except Exception as error:
            raise CliError(
                "RPC_ERROR",
                f"{type(error).__name__}: {error}",
                status=self._status_json(),
            ) from error
        if response is None:
            raise CliError("RPC_ERROR", "service returned no response")
        return Ack(
            bool(response.success),
            str(response.reason),
            str(getattr(response, "episode_id", "")),
        )

    def wait_terminal(self, episode_id: str, deadline: float) -> StatusSnapshot:
        seen_target = False
        while True:
            status = self._status
            if status is not None:
                if status.episode_id == episode_id:
                    seen_target = True
                    if status.phase in TERMINAL_PHASES:
                        return status
                elif seen_target and status.episode_id and status.phase in TERMINAL_PHASES:
                    raise CliError(
                        "EPISODE_MISMATCH",
                        "status switched to a different terminal episode",
                        status=status.as_json(),
                    )
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                current = self._status
                if current is not None and current.episode_id and current.episode_id != episode_id:
                    raise CliError(
                        "EPISODE_MISMATCH",
                        "terminal status names a different episode",
                        status=current.as_json(),
                    )
                raise CliError(
                    "WAIT_TIMEOUT",
                    "episode did not reach IDLE or FAULT before the deadline",
                    status=self._status_json(),
                )
            try:
                self._executor.spin_once(timeout_sec=min(0.1, remaining))
            except Exception as error:
                raise CliError(
                    "ROS_ERROR",
                    f"{type(error).__name__}: {error}",
                    status=self._status_json(),
                ) from error

    def _status_json(self) -> dict[str, str] | None:
        return None if self._status is None else self._status.as_json()

    def read_manifest(self, run_id: str, episode_id: str) -> dict[str, Any]:
        return read_manifest(self._raw_root, run_id, episode_id)


def _read_limited(path: Path) -> str:
    """Read at most 2 MiB of bytes from a regular file, then decode UTF-8."""
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_NONBLOCK", 0))
    try:
        if not stat.S_ISREG(os.fstat(descriptor).st_mode):
            raise CliError("MANIFEST_INVALID", "manifest must be a regular file")
        chunks: list[bytes] = []
        remaining = MAX_MANIFEST_BYTES + 1
        while remaining > 0:
            chunk = os.read(descriptor, min(65536, remaining))
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        data = b"".join(chunks)
    finally:
        os.close(descriptor)
    if len(data) > MAX_MANIFEST_BYTES:
        raise CliError("MANIFEST_TOO_LARGE", f"manifest exceeds {MAX_MANIFEST_BYTES} bytes")
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError as error:
        raise CliError("MANIFEST_INVALID", f"manifest is not UTF-8: {error}") from error


def _inside(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
    except ValueError:
        return False
    return True


def read_manifest(raw_root: Path, run_id: str, episode_id: str) -> dict[str, Any]:
    """Read only the final manifest inside <raw-root>/<run-id>/<episode-id>."""
    try:
        validate_run_id(run_id)
        validate_episode_id(episode_id)
    except ValueError as error:
        raise CliError("INVALID_ARGUMENT", str(error)) from error
    root = raw_root.resolve()
    run_directory = root / run_id
    try:
        resolved_run = run_directory.resolve(strict=True)
    except OSError as error:
        raise CliError("MANIFEST_MISSING", f"run directory is not readable: {error}") from error
    if not _inside(resolved_run, root) or resolved_run.parent != root:
        raise CliError("MANIFEST_OUTSIDE_ROOT", "run directory escapes the raw output root")
    episode_directory = resolved_run / episode_id
    try:
        resolved_episode = episode_directory.resolve(strict=True)
    except OSError as error:
        raise CliError("MANIFEST_MISSING", f"episode directory is not readable: {error}") from error
    if (
        not _inside(resolved_episode, root)
        or resolved_episode.parent != resolved_run
        or resolved_episode.name != episode_id
    ):
        raise CliError(
            "MANIFEST_OUTSIDE_ROOT",
            "episode directory is not the expected direct child of the run directory",
        )
    manifest_path = resolved_episode / "manifest.json"
    if manifest_path.is_symlink():
        raise CliError("MANIFEST_INVALID", "manifest must not be a symbolic link")
    try:
        resolved_manifest = manifest_path.resolve(strict=True)
    except OSError as error:
        raise CliError("MANIFEST_MISSING", f"manifest is not readable: {error}") from error
    if resolved_manifest.parent != resolved_episode or not _inside(resolved_manifest, root):
        raise CliError("MANIFEST_OUTSIDE_ROOT", "manifest escapes the episode directory")
    try:
        raw = _read_limited(resolved_manifest)
    except CliError:
        raise
    except OSError as error:
        raise CliError("MANIFEST_MISSING", f"manifest is not readable: {error}") from error
    except UnicodeError as error:
        raise CliError("MANIFEST_INVALID", f"manifest is not UTF-8: {error}") from error
    try:
        document = json.loads(raw)
    except (TypeError, ValueError) as error:
        raise CliError("MANIFEST_INVALID", f"manifest is not valid JSON: {error}") from error
    return _validate_manifest(document, run_id, episode_id, resolved_episode)


def _dimension(document: dict[str, Any], name: str) -> dict[str, Any]:
    value = document.get(name)
    if not isinstance(value, dict):
        raise CliError("MANIFEST_INVALID", f"manifest is missing {name} result")
    return value


def _validate_manifest(
    document: object,
    run_id: str,
    episode_id: str,
    directory: Path,
) -> dict[str, Any]:
    if not isinstance(document, dict):
        raise CliError("MANIFEST_INVALID", "manifest must be a JSON object")
    if document.get("schema_version") != EPISODE_SCHEMA_VERSION:
        raise CliError(
            "MANIFEST_INVALID",
            f"manifest schema_version must be {EPISODE_SCHEMA_VERSION}",
        )
    if document.get("run_id") != run_id:
        raise CliError("MANIFEST_INVALID", "manifest run_id does not match the request")
    if document.get("episode_id") != episode_id:
        raise CliError("MANIFEST_INVALID", "manifest episode_id does not match the request")
    recording_document = _dimension(document, "recording")
    runtime_document = _dimension(document, "runtime")
    task_document = _dimension(document, "task")
    try:
        recording = RecordingResult(
            recording_document["complete"],
            recording_document["failure_reason"],
        )
        runtime = RuntimeResult(
            RuntimeOutcome(runtime_document["outcome"]),
            runtime_document["reason"],
        )
        task = TaskResult(
            TaskOutcome(task_document["outcome"]),
            task_document["reason"],
        )
    except (KeyError, TypeError, ValueError) as error:
        raise CliError("MANIFEST_INVALID", f"manifest result is invalid: {error}") from error
    return {
        "schema_version": EPISODE_SCHEMA_VERSION,
        "run_id": run_id,
        "episode_id": episode_id,
        "directory": str(directory),
        "recording": {
            "complete": recording.complete,
            "failure_reason": recording.failure_reason,
        },
        "runtime": {"outcome": runtime.outcome.value, "reason": runtime.reason},
        "task": {"outcome": task.outcome.value, "reason": task.reason},
        "audit_verified": False,
    }


__all__ = [
    "ACTIVE_PHASES",
    "Ack",
    "CliError",
    "RAW_ROOT",
    "StatusSnapshot",
    "TERMINAL_PHASES",
    "UsageError",
    "read_manifest",
    "timeout_argument",
    "validate_episode_id",
    "validate_run_id",
]
