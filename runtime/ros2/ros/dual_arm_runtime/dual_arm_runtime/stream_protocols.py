"""Protocols shared by continuous stream runners and their adapters."""

from __future__ import annotations

import threading
from typing import Protocol

from robot_contracts.execution_feedback import ExecutionFeedback
from robot_contracts.models import RunContext
from robot_contracts.stream_request import StreamRequest

from dual_arm_runtime.observation import ExactObservation


class StreamControl(Protocol):
    @property
    def lease_id(self) -> str: ...

    def get_anchor(self, context: RunContext, start_backend_time_ns: int | None = None): ...

    def submit_follower(self, request: StreamRequest): ...

    def query(
        self, context: RunContext, command_id: str, *, ignore_stop: bool = False
    ) -> ExecutionFeedback: ...


class StreamIO(Protocol):
    def wait_observation(
        self, after_id: int, timeout_s: float, stop: threading.Event
    ) -> ExactObservation: ...

    def publish_command(self, request: StreamRequest, timestamp_ns: int) -> None: ...

    def record_follower_reference(self, reference: object) -> None: ...

    def record_execution(self, execution: ExecutionFeedback) -> None: ...

    def wait_terminal(
        self, command_id: str, timeout_s: float, stop: threading.Event
    ) -> ExecutionFeedback: ...

    def robot_time_ns(self) -> int: ...


class TaskMonitor(Protocol):
    """Bounded evaluation-only task lifecycle used by task-terminated streams."""

    def prepare(
        self,
        context: RunContext,
        episode_id: str,
        request_id: str,
        lease_id: str,
        seed: int,
    ) -> object: ...

    def start(
        self,
        context: RunContext,
        episode_id: str,
        request_id: str,
        lease_id: str,
        task_epoch: int,
        plan_digest: str,
    ) -> int: ...

    def state(self, context: RunContext, task_epoch: int = 0) -> object: ...


__all__ = ["StreamControl", "StreamIO", "TaskMonitor"]
