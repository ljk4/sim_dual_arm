"""Bounded write-behind native rosbag2/MCAP recorder with checked commit."""

from __future__ import annotations

import json
import os
import threading
import time
from collections.abc import Callable
from pathlib import Path

import rosbag2_py
from rclpy.serialization import serialize_message
from robot_contracts.episodes import (
    EPISODE_SCHEMA_VERSION,
    RecordingResult,
    RuntimeOutcome,
    RuntimeResult,
    TaskResult,
    manifest_schema_path,
)
from robot_contracts.models import ContractError
from robot_contracts.profiles import validate_document

from dual_arm_runtime.audit import audit_recording, sha256_file
from dual_arm_runtime.recording_queue import (
    BoundedRecordingQueue,
    RecordingQueueError,
    RecordingSettings,
    RecordingStats,
)


class _NativeWriter:
    """The only object allowed to touch a ``SequentialWriter``.

    It is constructed inside the recorder's writer thread, so ``open``,
    ``create_topic``, ``write`` and ``close`` all run on that one thread.
    """

    def __init__(self, directory: Path, topics: dict[str, str]) -> None:
        self._directory = directory
        self._topics = dict(topics)
        self._writer: rosbag2_py.SequentialWriter | None = None

    def open(self) -> None:
        writer = rosbag2_py.SequentialWriter()
        try:
            writer.open(
                rosbag2_py.StorageOptions(
                    uri=str(self._directory / "recording"), storage_id="mcap"
                ),
                rosbag2_py.ConverterOptions("", ""),
            )
            for index, (name, type_name) in enumerate(self._topics.items(), start=1):
                writer.create_topic(
                    rosbag2_py.TopicMetadata(
                        id=index,
                        name=name,
                        type=type_name,
                        serialization_format="cdr",
                    )
                )
        except BaseException:
            try:
                writer.close()
            except Exception:
                pass
            raise
        self._writer = writer

    def write(self, topic: str, payload: bytes, timestamp_ns: int) -> None:
        if self._writer is None:
            raise RuntimeError("native writer is not open")
        self._writer.write(topic, payload, timestamp_ns)

    def close(self) -> None:
        if self._writer is not None:
            self._writer.close()
            self._writer = None


class EpisodeRecorder:
    """Own one bounded queue and one native writer thread for an episode.

    ``append`` serializes on the calling thread, then hands immutable bytes to the
    queue.  It never waits for disk or free capacity.  ``finish`` stops producers,
    waits for the writer thread within ``close_timeout_s``, and only then runs the
    streaming audit, chunked hashing and manifest commit on its caller.

    Recording completeness is independent of the runtime result: a legitimate
    stop or signal does not corrupt an already accepted recording.  A close,
    count, audit or hash failure keeps the directory partial and marks
    ``recording.complete=false``.
    """

    def __init__(
        self,
        output_root: Path,
        episode_id: str,
        configuration: dict,
        schemas: Path,
        topics: dict[str, str],
        settings: RecordingSettings | None = None,
        *,
        run_id: str,
        source: str,
        test_only: bool,
        format_version: int = EPISODE_SCHEMA_VERSION,
    ) -> None:
        if not isinstance(run_id, str) or not run_id:
            raise ContractError("EPISODE_ERROR", "manifest run_id must be a non-empty string")
        if not isinstance(source, str) or not source:
            raise ContractError("EPISODE_ERROR", "manifest source must be a non-empty string")
        if not isinstance(test_only, bool):
            raise ContractError("EPISODE_ERROR", "manifest test_only must be a boolean")
        if isinstance(format_version, bool) or format_version not in (1, 2, 3, 4):
            raise ContractError(
                "EPISODE_ERROR", "manifest format_version must be one of 1, 2, 3 or 4"
            )
        self._format_version = format_version
        output_root.mkdir(parents=True, exist_ok=True)
        self._final = output_root / episode_id
        self.directory = output_root / f".partial-{episode_id}"
        self.directory.mkdir()
        self._episode_id, self._configuration, self._schemas = episode_id, configuration, schemas
        self._run_id, self._source, self._test_only = run_id, source, test_only
        self._topics = dict(topics)
        if settings is None:
            recipe = configuration.get("recipe")
            raw_settings = recipe.get("recording") if isinstance(recipe, dict) else None
            try:
                settings = (
                    RecordingSettings(**raw_settings)
                    if isinstance(raw_settings, dict)
                    else RecordingSettings()
                )
            except (TypeError, ValueError) as error:
                raise ContractError(
                    "RECORDING_ERROR", f"invalid recording settings: {error}"
                ) from error
        self._settings = settings
        self._started_at = time.time_ns()
        self._finish_lock = threading.Lock()
        self._finalized = False
        try:
            self._queue = BoundedRecordingQueue(
                self._settings,
                lambda: _NativeWriter(self.directory, self._topics),
                self._topics,
                thread_name=f"mcap-writer-{episode_id}",
            )
        except RecordingQueueError as error:
            raise ContractError("RECORDING_ERROR", f"writer startup failed: {error}") from error

    def append(self, topic: str, message, timestamp_ns: int) -> None:
        # Serialization is caller-side and outside the queue lock.  The resulting
        # bytes are immutable, so later mutation of ``message`` cannot change what
        # this call enqueued.
        if topic not in self._topics:
            reason = f"unregistered recording topic: {topic}"
            self._queue.fail(reason)
            raise ContractError("RECORDING_ERROR", reason)
        if isinstance(timestamp_ns, bool) or not isinstance(timestamp_ns, int):
            reason = "recording timestamp must be an integer"
            self._queue.fail(reason)
            raise ContractError("RECORDING_ERROR", reason)
        if timestamp_ns < 0:
            reason = "recording timestamp must not be negative"
            self._queue.fail(reason)
            raise ContractError("RECORDING_ERROR", reason)
        try:
            payload = serialize_message(message)
        except Exception as error:
            reason = f"serialize {topic}: {type(error).__name__}: {error}"
            self._queue.fail(reason)
            raise ContractError("RECORDING_ERROR", reason) from error
        try:
            self._queue.append(topic, payload, timestamp_ns)
        except RecordingQueueError as error:
            raise ContractError("RECORDING_ERROR", str(error)) from error

    def check_health(self) -> None:
        """Raise an asynchronous writer failure without doing any file I/O."""
        try:
            self._queue.check_health()
        except RecordingQueueError as error:
            raise ContractError("RECORDING_ERROR", str(error)) from error

    def stats(self) -> RecordingStats:
        return self._queue.stats()

    @staticmethod
    def _effective_runtime(
        runtime: RuntimeResult,
        recording_failure: str,
    ) -> RuntimeResult:
        """A failed recording can never leave the runtime marked completed."""
        if recording_failure and runtime.outcome is not RuntimeOutcome.FAILED:
            reason = recording_failure
            if runtime.reason and runtime.reason not in reason:
                reason = f"{reason}; {runtime.reason}"
            return RuntimeResult(RuntimeOutcome.FAILED, reason)
        return runtime

    def finish(
        self,
        runtime: RuntimeResult | None = None,
        *,
        current_runtime: Callable[[], RuntimeResult] | None = None,
        task_result: TaskResult | None = None,
    ) -> Path:
        """Drain/close within the budget, then audit and commit on this thread.

        ``current_runtime`` is re-read after draining, auditing and hashing, and
        once more after publication.  An end request or signal that arrives while
        the finalizer is busy therefore updates ``runtime.outcome`` instead of
        being silently recorded as ``completed``.

        A writer close failure or timeout keeps the directory recognizable as a
        partial episode and raises; this method never force-closes the native
        handle and never publishes a success manifest for a timed-out writer.
        """
        with self._finish_lock:
            if self._finalized:
                raise RuntimeError("Episode has already been finalized")
            self._finalized = True

        initial_runtime = RuntimeResult.completed() if runtime is None else runtime
        if not isinstance(initial_runtime, RuntimeResult):
            raise TypeError("runtime must be a RuntimeResult")
        task_result = TaskResult() if task_result is None else task_result
        if not isinstance(task_result, TaskResult):
            raise TypeError("task_result must be a TaskResult")

        def latest_runtime() -> RuntimeResult:
            value = current_runtime() if current_runtime is not None else initial_runtime
            if not isinstance(value, RuntimeResult):
                raise TypeError("current_runtime must return a RuntimeResult")
            return value

        try:
            stats = self._queue.close(self._settings.close_timeout_s)
        except RecordingQueueError as error:
            raise ContractError("RECORDING_ERROR", f"writer close failed: {error}") from error
        recording_failure = ""
        if stats.accepted != stats.written or stats.pending_items or stats.pending_bytes:
            recording_failure = "RECORDING_ERROR: writer closed with accepted data not written"
        if not recording_failure:
            try:
                report = audit_recording(
                    self.directory / "recording",
                    self._episode_id,
                    self._configuration,
                    version=self._format_version,
                )
                if report["message_counts"] != dict(stats.written_counts):
                    recording_failure = "RECORDING_ERROR: message counts differ"
            except ContractError as error:
                recording_failure = str(error) if error.reason else f"RECORDING_ERROR: {error}"
            except Exception as error:
                recording_failure = f"RECORDING_ERROR: {error}"
        files = {
            path.relative_to(self.directory).as_posix(): sha256_file(path)
            for path in sorted((self.directory / "recording").rglob("*"))
            if path.is_file()
        }
        recording_complete = not recording_failure
        recording_result = RecordingResult(recording_complete, recording_failure)
        runtime_result = self._effective_runtime(latest_runtime(), recording_failure)
        manifest = {
            "schema_version": self._format_version,
            "episode_id": self._episode_id,
            "run_id": self._run_id,
            "source": self._source,
            "test_only": self._test_only,
            "robot_profile_hash": self._configuration["profile_hash"],
            "observation_schema_hash": self._configuration["observation_hash"],
            "action_schema_hash": self._configuration["action_hash"],
            "started_at_ns": self._started_at,
            "ended_at_ns": time.time_ns(),
            "message_counts": dict(stats.written_counts),
            "files": files,
            "recording": {
                "complete": recording_result.complete,
                "failure_reason": recording_result.failure_reason,
            },
            "runtime": {
                "outcome": runtime_result.outcome.value,
                "reason": runtime_result.reason,
            },
            "task": {
                "outcome": task_result.outcome.value,
                "reason": task_result.reason,
            },
            "configuration": self._configuration,
        }

        def write_manifest() -> None:
            manifest["runtime"] = {
                "outcome": runtime_result.outcome.value,
                "reason": runtime_result.reason,
            }
            validate_document(
                manifest,
                manifest_schema_path(self._schemas, self._format_version),
            )
            temporary = self.directory / "manifest.json.tmp"
            with temporary.open("w") as stream:
                json.dump(manifest, stream, indent=2, allow_nan=False)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())

        def commit_manifest() -> None:
            write_manifest()
            (self.directory / "manifest.json.tmp").replace(self.directory / "manifest.json")

        write_manifest()
        latest = self._effective_runtime(latest_runtime(), recording_failure)
        if latest != runtime_result:
            runtime_result = latest
            write_manifest()
        (self.directory / "manifest.json.tmp").replace(self.directory / "manifest.json")
        if recording_complete:
            self.directory.rename(self._final)
            self.directory = self._final
        # Re-check once after publication as well: a signal may have arrived while
        # the manifest was written or the directory renamed.
        latest = self._effective_runtime(latest_runtime(), recording_failure)
        if latest != runtime_result:
            runtime_result = latest
            commit_manifest()
        return self.directory
