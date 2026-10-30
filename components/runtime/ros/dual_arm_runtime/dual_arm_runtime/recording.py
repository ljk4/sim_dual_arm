"""Thread-safe native rosbag2/MCAP writer with checked, atomic episode commit."""

import hashlib
import json
import os
import threading
import time
from collections import Counter
from pathlib import Path

import rosbag2_py
from rclpy.serialization import serialize_message
from robot_contracts.profiles import validate_document

from dual_arm_runtime.audit import audit_recording


class EpisodeRecorder:
    """Own the writer; callers only append measurements, commands and references."""

    def __init__(
        self,
        output_root: Path,
        episode_id: str,
        configuration: dict,
        schemas: Path,
        topics: dict[str, str],
    ) -> None:
        output_root.mkdir(parents=True, exist_ok=True)
        self._final = output_root / episode_id
        self.directory = output_root / f".partial-{episode_id}"
        self.directory.mkdir()
        self._episode_id, self._configuration, self._schemas = episode_id, configuration, schemas
        self._lock = threading.Lock()
        self._counts: Counter = Counter()
        self._started_at = time.time_ns()
        self._writer = rosbag2_py.SequentialWriter()
        self._writer.open(
            rosbag2_py.StorageOptions(uri=str(self.directory / "recording"), storage_id="mcap"),
            rosbag2_py.ConverterOptions("", ""),
        )
        for index, (name, type_name) in enumerate(topics.items(), start=1):
            self._writer.create_topic(
                rosbag2_py.TopicMetadata(
                    id=index,
                    name=name,
                    type=type_name,
                    serialization_format="cdr",
                )
            )

    def append(self, topic: str, message, timestamp_ns: int) -> None:
        with self._lock:
            if self._writer is not None:
                self._writer.write(topic, serialize_message(message), timestamp_ns)
                self._counts[topic] += 1

    def finish(self, failure_reason: str = "") -> Path:
        with self._lock:
            if self._writer is None:
                raise RuntimeError("Episode has already been finalized")
            self._writer.close()
            self._writer = None
        if not failure_reason:
            try:
                report = audit_recording(
                    self.directory / "recording",
                    self._episode_id,
                    self._configuration,
                )
                if report["message_counts"] != dict(self._counts):
                    failure_reason = "RECORDING_ERROR: message counts differ"
            except Exception as error:
                failure_reason = f"RECORDING_ERROR: {error}"
        files = {
            path.relative_to(self.directory).as_posix(): hashlib.sha256(
                path.read_bytes()
            ).hexdigest()
            for path in sorted((self.directory / "recording").rglob("*"))
            if path.is_file()
        }
        manifest = {
            "schema_version": 1,
            "complete": not failure_reason,
            "test_only": True,
            "source": "mock",
            "episode_id": self._episode_id,
            "robot_profile_hash": self._configuration["profile_hash"],
            "observation_schema_hash": self._configuration["observation_hash"],
            "action_schema_hash": self._configuration["action_hash"],
            "started_at_ns": self._started_at,
            "ended_at_ns": time.time_ns(),
            "message_counts": dict(self._counts),
            "files": files,
            "failure_reason": failure_reason,
            "configuration": self._configuration,
        }
        validate_document(manifest, self._schemas / "episode_manifest.schema.json")
        temporary = self.directory / "manifest.json.tmp"
        with temporary.open("w") as stream:
            json.dump(manifest, stream, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        temporary.replace(self.directory / "manifest.json")
        if not failure_reason:
            self.directory.rename(self._final)
            self.directory = self._final
        return self.directory
