"""Lease ownership, validated trajectories and deterministic mock interpolation."""

import hashlib
import json
import math
import threading
import uuid
from dataclasses import dataclass

from robot_contracts.models import ContractError, RobotProfile, RunContext, Trajectory

TERMINAL_PHASES = frozenset({"COMPLETED", "CANCELED", "FAILED"})


@dataclass(frozen=True)
class Event:
    sequence: int
    context: RunContext
    command: Trajectory
    phase: str
    reason: str


@dataclass
class Execution:
    command: Trajectory
    fingerprint: str
    started_at_ns: int
    initial_positions: tuple[float, ...]
    phase: str = "STARTED"
    reason: str = ""


class BackendEngine:
    """Thread-safe backend state; callers supply robot and monotonic clocks."""

    def __init__(self, profile: RobotProfile, max_history: int = 256) -> None:
        if max_history < 1:
            raise ValueError("Command history must retain at least one execution")
        self.profile = profile
        self.context = RunContext(
            profile.robot_id,
            profile.digest,
            str(uuid.uuid4()),
            "mock-system",
            0,
        )
        self.positions = profile.initial_positions
        self._lock = threading.RLock()
        self._owner = ""
        self._lease_id = ""
        self._lease_until_ns = 0
        self._highest_sequence = 0
        self._history: dict[str, Execution] = {}
        self._active: Execution | None = None
        self._events: list[Event] = []
        self._event_sequence = 0
        self._max_history = max_history

    def snapshot(self) -> tuple[RunContext, tuple[float, ...]]:
        with self._lock:
            return self.context, self.positions

    def _check_generation(self, context: RunContext) -> None:
        expected = self.context
        for name in (
            "robot_id",
            "robot_profile_hash",
            "backend_instance_id",
            "time_domain_id",
            "clock_epoch",
        ):
            if getattr(context, name) != getattr(expected, name):
                raise ContractError("STALE_CONTEXT", name)

    def acquire(self, context: RunContext, owner: str, duration_ns: int, now_ns: int) -> str:
        with self._lock:
            self.expire(now_ns)
            self._check_generation(context)
            if self._lease_id:
                raise ContractError("CONTROL_BUSY")
            if not owner or not context.episode_id or context.control_epoch < 1:
                raise ContractError("INVALID_CONTEXT")
            if not 100_000_000 <= duration_ns <= 10_000_000_000:
                raise ContractError("INVALID_LEASE_DURATION")
            self.context = context
            self._owner = owner
            self._lease_id = str(uuid.uuid4())
            self._lease_until_ns = now_ns + duration_ns
            self._highest_sequence = 0
            return self._lease_id

    def _check_lease(self, context: RunContext, lease_id: str, now_ns: int) -> None:
        self.expire(now_ns)
        self._check_generation(context)
        if context != self.context:
            raise ContractError("STALE_CONTEXT", "Episode or control epoch")
        if not self._lease_id or lease_id != self._lease_id:
            raise ContractError("LEASE_EXPIRED")

    def renew(self, context: RunContext, lease_id: str, duration_ns: int, now_ns: int) -> None:
        with self._lock:
            self._check_lease(context, lease_id, now_ns)
            if not 100_000_000 <= duration_ns <= 10_000_000_000:
                raise ContractError("INVALID_LEASE_DURATION")
            self._lease_until_ns = now_ns + duration_ns

    def release(self, context: RunContext, lease_id: str, now_ns: int) -> None:
        with self._lock:
            self._check_lease(context, lease_id, now_ns)
            self._stop("CONTROL_RELEASED")
            self._lease_id = ""
            self._owner = ""

    def expire(self, now_ns: int) -> None:
        with self._lock:
            if self._lease_id and now_ns >= self._lease_until_ns:
                self._stop("LEASE_EXPIRED")
                self._lease_id = ""
                self._owner = ""

    def stop(self, context: RunContext, lease_id: str, now_ns: int, reason: str) -> None:
        with self._lock:
            self._check_lease(context, lease_id, now_ns)
            self._stop(reason or "STOP_REQUESTED")

    def _stop(self, reason: str) -> None:
        if self._active is not None:
            self._finish("CANCELED", reason)

    def cancel(
        self,
        context: RunContext,
        lease_id: str,
        command_id: str,
        now_ns: int,
    ) -> None:
        with self._lock:
            self._check_lease(context, lease_id, now_ns)
            if self._active is not None and self._active.command.command_id == command_id:
                self._finish("CANCELED", "CANCEL_REQUESTED")

    @staticmethod
    def _fingerprint(command: Trajectory) -> str:
        payload = (
            command.joint_names,
            command.observation_id,
            command.accept_before_ns,
            [(point.time_ns, point.positions) for point in command.targets],
        )
        return hashlib.sha256(json.dumps(payload, allow_nan=False).encode()).hexdigest()

    def _validate(self, command: Trajectory, robot_now_ns: int) -> None:
        if robot_now_ns > command.accept_before_ns:
            raise ContractError("COMMAND_EXPIRED")
        if not command.command_id or command.command_seq <= self._highest_sequence:
            raise ContractError("COMMAND_OUT_OF_ORDER")
        if command.observation_id < 1:
            raise ContractError("INVALID_TRAJECTORY", "Missing observation reference")
        if command.joint_names != self.profile.joint_names:
            raise ContractError("SCHEMA_MISMATCH", "Joint names/order")
        if not command.targets or command.targets[-1].time_ns > 3_000_000_000:
            raise ContractError("INVALID_TRAJECTORY", "Empty or excessive horizon")
        previous_time = 0
        previous = self.positions
        for point in command.targets:
            if point.time_ns <= previous_time or len(point.positions) != len(previous):
                raise ContractError("INVALID_TRAJECTORY", "Dimensions or time ordering")
            seconds = (point.time_ns - previous_time) / 1e9
            for joint, target, start in zip(
                self.profile.joints,
                point.positions,
                previous,
                strict=True,
            ):
                if not math.isfinite(target):
                    raise ContractError("INVALID_TRAJECTORY", "Non-finite position")
                if not joint.minimum <= target <= joint.maximum:
                    raise ContractError("JOINT_LIMIT", joint.name)
                if abs(target - start) / seconds > joint.max_velocity + 1e-9:
                    raise ContractError("VELOCITY_LIMIT", joint.name)
            previous_time, previous = point.time_ns, point.positions

    def submit(self, command: Trajectory, robot_now_ns: int, steady_now_ns: int) -> Execution:
        with self._lock:
            self._check_lease(command.context, command.lease_id, steady_now_ns)
            try:
                fingerprint = self._fingerprint(command)
            except ValueError as error:
                raise ContractError("INVALID_TRAJECTORY", "Non-finite data") from error
            previous = self._history.get(command.command_id)
            if previous is not None:
                if previous.command.lease_id != command.lease_id:
                    raise ContractError("COMMAND_ID_CONFLICT")
                if previous.command.command_seq != command.command_seq:
                    raise ContractError("COMMAND_ID_CONFLICT")
                if previous.fingerprint != fingerprint:
                    raise ContractError("COMMAND_ID_CONFLICT")
                return previous
            self._validate(command, robot_now_ns)
            if self._active is not None:
                raise ContractError("CONTROL_BUSY")
            if len(self._events) > 1021:
                raise ContractError("EVENT_QUEUE_FULL", "Consumer must drain events")
            self._highest_sequence = command.command_seq
            execution = Execution(command, fingerprint, steady_now_ns, self.positions)
            self._history[command.command_id] = execution
            self._active = execution
            self._emit(execution, "ACCEPTED", "")
            self._emit(execution, "STARTED", "")
            while len(self._history) > self._max_history:
                del self._history[next(iter(self._history))]
            return execution

    def tick(self, steady_now_ns: int) -> None:
        with self._lock:
            self.expire(steady_now_ns)
            execution = self._active
            if execution is None:
                return
            elapsed = max(0, steady_now_ns - execution.started_at_ns)
            start_time, start = 0, execution.initial_positions
            for target in execution.command.targets:
                if elapsed < target.time_ns:
                    fraction = (elapsed - start_time) / (target.time_ns - start_time)
                    self.positions = tuple(
                        left + fraction * (right - left)
                        for left, right in zip(start, target.positions, strict=True)
                    )
                    return
                start_time, start = target.time_ns, target.positions
            self.positions = execution.command.targets[-1].positions
            self._finish("COMPLETED", "")

    def _emit(self, execution: Execution, phase: str, reason: str) -> None:
        self._event_sequence += 1
        self._events.append(
            Event(
                self._event_sequence,
                execution.command.context,
                execution.command,
                phase,
                reason,
            )
        )

    def _finish(self, phase: str, reason: str) -> None:
        execution = self._active
        if execution is not None:
            execution.phase, execution.reason = phase, reason
            self._emit(execution, phase, reason)
            self._active = None

    def drain_events(self) -> list[Event]:
        with self._lock:
            events, self._events = self._events, []
            return events

    def execution_status(self, command_id: str) -> tuple[str, str]:
        with self._lock:
            execution = self._history.get(command_id)
            if execution is None:
                raise ContractError("RESULT_EXPIRED", command_id)
            return execution.phase, execution.reason
