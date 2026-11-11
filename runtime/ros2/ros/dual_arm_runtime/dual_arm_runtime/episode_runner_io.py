"""Protocols and the thread-safe failure latch used by one episode runner."""

from __future__ import annotations

import threading
from collections.abc import Callable, Sequence
from pathlib import Path
from typing import TYPE_CHECKING, Protocol

from robot_contracts.action_chunks import ChunkAnchor
from robot_contracts.episodes import RuntimeResult, TaskResult
from robot_contracts.execution_feedback import ExecutionFeedback
from robot_contracts.models import RunContext
from robot_contracts.stream_request import StreamRequest
from robot_contracts.trajectory.follower_reference import FollowerReference

from dual_arm_runtime.observation import ExactObservation
from dual_arm_runtime.recording_queue import RecordingStats

if TYPE_CHECKING:
    from robot_api_msgs.msg import BackendInfo


class FailureLog:
    """Aggregate the primary fault and later cleanup/recording reasons.

    The lock protects only the string.  Setting the stop event and waking the ROS
    condition happen outside it; no RPC, join or disk operation runs here.
    """

    def __init__(self, stop_requested: threading.Event, notify: Callable[[], None]) -> None:
        self._lock = threading.Lock()
        self._stop_requested = stop_requested
        self._notify = notify
        self._reason = ""

    @property
    def reason(self) -> str:
        with self._lock:
            return self._reason

    def reset(self, reason: str = "") -> None:
        with self._lock:
            self._reason = reason

    def fail(self, reason: str) -> None:
        """Keep the first primary fault and request a stop."""
        with self._lock:
            if not self._reason:
                self._reason = reason
        self._stop_requested.set()
        self._notify()

    def request_stop(self) -> None:
        """Ask the worker to stop without recording a fault."""
        self._stop_requested.set()
        self._notify()

    def append(self, reason: str) -> None:
        """Append a later cleanup reason without losing the primary fault."""
        with self._lock:
            if not reason or reason in self._reason:
                return
            if self._reason:
                self._reason = f"{self._reason}; {reason}"
            else:
                self._reason = reason
        self._stop_requested.set()
        self._notify()

    def merge(self, reason: str) -> None:
        """Take a recorder's combined reason without duplicating the primary fault."""
        with self._lock:
            if not reason or reason in self._reason:
                return
            if self._reason and self._reason not in reason:
                self._reason = f"{self._reason}; {reason}"
            else:
                self._reason = reason
        self._stop_requested.set()
        self._notify()


class Recorder(Protocol):
    """The small recorder surface owned by one episode."""

    directory: Path

    def append(self, topic: str, message: object, timestamp_ns: int) -> None: ...

    def check_health(self) -> None: ...

    def stats(self) -> RecordingStats: ...

    def finish(
        self,
        runtime: RuntimeResult | None = None,
        *,
        current_runtime: Callable[[], RuntimeResult] | None = None,
        task_result: TaskResult | None = None,
    ) -> Path: ...


class ControlPort(Protocol):
    @property
    def lease_id(self) -> str: ...

    def wait_ready(self) -> None: ...

    def acquire(self, context: RunContext) -> str: ...

    def reset(self, context: RunContext, lease_id: str, seed: int) -> RunContext: ...

    def query(
        self, context: RunContext, command_id: str, *, request: StreamRequest
    ) -> ExecutionFeedback: ...

    def get_anchor(
        self, context: RunContext, start_sim_time_ns: int | None = None
    ) -> ChunkAnchor: ...

    def submit_follower(self, request: StreamRequest) -> object: ...

    def stop_episode(self, reason: str) -> None: ...

    def cleanup(self, reason: str = "EPISODE_END") -> None: ...


class EpisodeIO(Protocol):
    """ROS-side operations with bounded waits; every method is worker-thread safe."""

    @property
    def info_topic(self) -> str: ...

    def wait_backend(
        self, episode_id: str, timeout_s: float, stop: threading.Event
    ) -> tuple[RunContext, BackendInfo]: ...

    def bind_context(self, context: RunContext) -> None: ...

    def record_backend_info(self, context: RunContext) -> None: ...

    def wait_observation(
        self, after_id: int, timeout_s: float, stop: threading.Event
    ) -> ExactObservation: ...

    def publish_command(self, request: StreamRequest, timestamp_ns: int) -> None: ...

    def record_follower_reference(self, reference: FollowerReference) -> None: ...

    def record_task_state(self, fact: object) -> None: ...

    def task_client(self) -> object: ...

    def record_execution(self, execution: ExecutionFeedback) -> None: ...

    def wait_terminal(
        self, command_id: str, timeout_s: float, stop: threading.Event
    ) -> ExecutionFeedback: ...

    def wait_stop_settled(self, command_ids: Sequence[str], timeout_s: float) -> None: ...

    def robot_time_ns(self) -> int: ...

    def publish_status(
        self, phase: str, episode_id: str, reason: str, record_directory: str
    ) -> None: ...

    def record_topics(self) -> dict[str, str]: ...

    def log_error(self, message: str) -> None: ...

    def notify(self) -> None: ...


__all__ = ["ControlPort", "EpisodeIO", "FailureLog", "Recorder"]
