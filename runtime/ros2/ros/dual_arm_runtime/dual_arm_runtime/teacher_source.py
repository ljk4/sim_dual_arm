"""Program-teacher target source over the 033 ROS services.

This module is deliberately free of MuJoCo and mink: it only reads immutable
teacher chunks and frozen task facts through :class:`TeacherTaskClient`.  The
continuous stream core still owns the fixed anchor, the configured chunk
window, SubmitFollower identity and the actual stop.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable

from robot_contracts.models import ContractError, RobotProfile, RunContext
from robot_contracts.trajectory.follower_reference import FollowerValidationLimits

from dual_arm_runtime.chunk_stream import (
    ContinuousStreamRunner,
    StreamControl,
    StreamIO,
    _TaskComplete,
)
from dual_arm_runtime.observation import ExactObservation
from dual_arm_runtime.run_logging import EventSink
from dual_arm_runtime.runtime_config import RuntimeConfiguration, TeacherConfig
from dual_arm_runtime.target_source import (
    Prediction,
    TaskTerminal,
    task_terminal_fact,
)
from dual_arm_runtime.teacher_client import TeacherTaskClient

__all__ = ["TeacherStreamRunner", "TeacherTargetSource"]


class TeacherTargetSource:
    """Read one complete teacher chunk for every exact capture."""

    def __init__(
        self,
        client: TeacherTaskClient,
        teacher: TeacherConfig,
        profile: RobotProfile,
        *,
        target_count: int,
        task_seed: int,
        log: EventSink | None = None,
    ) -> None:
        self.client = client
        self.teacher = teacher
        self.profile = profile
        self.task_seed = task_seed
        self.log = log
        self.chunk_size = teacher.chunk_size
        self.target_count = target_count
        self.max_chunks = teacher.max_chunks
        self.initial_lead_ns = teacher.initial_lead_ns
        self.predict_deadline_s = teacher.chunk_deadline_s
        self.poll_interval_s = teacher.poll_interval_s
        self.termination_mode = teacher.termination_mode
        self.total_sim_deadline_ns = teacher.total_sim_deadline_ns
        self.total_wall_deadline_s = teacher.total_wall_deadline_s
        self.task_stop_margin_s = teacher.task_stop_margin_s
        self.task_sim_budget_ns = teacher.task_sim_budget_ns
        self._context: RunContext | None = None
        self._episode_id = ""
        self._lease_id = ""
        self._task_epoch = 0
        self._task_end: tuple[str, str, object | None] = ("", "", None)

    def bind_lease(self, lease_id: str) -> None:
        self._lease_id = lease_id

    def begin(self, context: RunContext, episode_id: str, task_id: str) -> int | None:
        del task_id
        self._context = context
        self._episode_id = episode_id
        prepared = self.client.prepare(
            context,
            episode_id,
            f"{episode_id}-prepare",
            self._lease_id,
            self.task_seed,
        )
        if prepared.prepare_kind != "teacher":
            raise ContractError(
                "TEACHER_DISABLED",
                f"backend prepared {prepared.prepare_kind!r}, not a teacher plan",
            )
        self._task_epoch = prepared.task_epoch
        return self.client.start(
            context,
            episode_id,
            f"{episode_id}-start",
            self._lease_id,
            prepared.task_epoch,
            prepared.plan_digest,
        )

    def predict(
        self, observation: ExactObservation, chunk_index: int, request_id: str,
        *, handoff_time_ns: int = 0,
    ) -> Prediction:
        if self._context is None:
            raise ContractError("STREAM_NOT_STARTED", "teacher source has no context")
        try:
            chunk = self.client.chunk(
                self._context,
                self._episode_id,
                self._task_epoch,
                request_id,
                observation.state.sequence,
                observation.stamp_ns,
                robot_profile_hash=self.profile.digest,
                chunk_size=self.chunk_size,
                joint_count=len(self.profile.joints),
                period_ns=self.teacher.action_period_ns,
                handoff_time_ns=handoff_time_ns,
            )
        except ContractError as error:
            if error.reason in {"TASK_NOT_RUNNING", "TEACHER_PLAN_COMPLETE"}:
                state = self.client.state(self._context, self._task_epoch)
                runtime, task = task_terminal_fact(state)
                if runtime:
                    self._task_end = (runtime, task, state)
                    raise TaskTerminal(runtime, task, state) from error
                raise ContractError(
                    "TEACHER_STREAM_RACE",
                    f"teacher chunk unavailable while task is {getattr(state, 'phase', '?')}",
                ) from error
            if error.reason == "TEACHER_GATE_FAILED":
                self._task_end = (
                    "TASK_FAILED",
                    f"GATE_FAILED: {error}",
                    self.client.frozen_state,
                )
            raise
        state = self.client.state(self._context, self._task_epoch)
        runtime, task = task_terminal_fact(state)
        if runtime:
            self._task_end = (runtime, task, state)
        return Prediction(
            context=self._context,
            observation_id=observation.observation_id,
            joint_names=self.profile.joint_names,
            times_from_observation_ns=chunk.times_from_observation_ns,
            positions=chunk.positions,
            target_count=self.target_count,
            phase=chunk.phase,
            gate_state=chunk.gate_state,
            gate_failure=chunk.gate_failure,
            chunk_digest=chunk.chunk_digest,
            task_end=runtime,
            task_reason=task,
            task_state=state if runtime else None,
        )

    def task_terminal(self) -> tuple[str, str, object | None]:
        return self._task_end

    def finish(self) -> None:
        return None


class TeacherStreamRunner(ContinuousStreamRunner):
    """Continuous teacher stream whose normal end is a frozen task fact."""

    def __init__(
        self,
        configuration: RuntimeConfiguration,
        context: RunContext,
        episode_id: str,
        task_id: str,
        io: StreamIO,
        control: StreamControl,
        source: TeacherTargetSource,
        limits: object,
        *,
        stop_requested: threading.Event,
        log: EventSink | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not isinstance(limits, FollowerValidationLimits):
            raise ContractError("STREAM_CONFIGURATION", "follower limits must be concrete")
        super().__init__(
            configuration,
            context,
            episode_id,
            task_id,
            io,
            control,
            source,
            limits,
            stop_requested=stop_requested,
            log=log,
            clock=clock,
        )

    def _kind(self) -> str:
        return "teacher"

    def run(self) -> None:
        try:
            super().run()
        except _TaskComplete:
            return
        except ContractError as error:
            if error.reason == "TEACHER_GATE_FAILED":
                self.task_end = self.source.task_terminal()
                self._emit(
                    "teacher.gate_failed",
                    level="ERROR",
                    reason="TEACHER_GATE_FAILED",
                    fields={
                        "gate_failure": str(error),
                        "task_end": self.task_end[0] or None,
                    },
                )
                return
            raise
