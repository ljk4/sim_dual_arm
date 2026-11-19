"""Continuous bounded-follower stream shared by policy and teacher sources.

The module owns only stream sequencing: one fresh exact capture per block, one
complete 60x16 prediction per capture, the backend's fixed absolute anchor, and
a single SubmitFollower command identity whose chunk sequence follows that
anchor.  A typed :class:`TargetSource` supplies predictions; the policy adapter
keeps using real gRPC, the teacher adapter uses the 033 ROS services.  ROS
callbacks, leases, recording and physical execution stay in their existing
owners.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading
import time
from collections.abc import Callable

from robot_contracts.action_chunks import ActionWindow
from robot_contracts.models import ContractError, RobotProfile, RunContext, Target
from robot_contracts.stream_request import (
    REQUEST_STREAM_FIRST,
    REQUEST_STREAM_NEXT,
    StreamRequest,
    Trajectory,
)
from robot_contracts.trajectory.follower_reference import FollowerValidationLimits
from robot_run_logging import EventContext, Scalar
from robot_trajectory.transition_retiming import (
    RetimedWindow,
    gripper_axes,
    retime_action_window,
)

from dual_arm_runtime.follower_limits import follower_limits  # noqa: F401 - re-export
from dual_arm_runtime.observation import ExactObservation
from dual_arm_runtime.policy_client import (
    PolicyClient,
    PolicyPrediction,
    observation_from_exact,
)
from dual_arm_runtime.run_logging import EventSink, emit_best_effort
from dual_arm_runtime.runtime_config import PolicyConfig, RuntimeConfiguration
from dual_arm_runtime.stream_protocols import StreamControl, StreamIO, TaskMonitor  # noqa: F401
from dual_arm_runtime.target_source import (
    Prediction,
    TargetSource,
    TaskTerminal,
    task_terminal_fact,
)


class PolicyTargetSource:
    """Adapt the real gRPC policy client to the typed target-source boundary."""

    def __init__(
        self,
        client: PolicyClient,
        policy: PolicyConfig,
        target_count: int,
        task: TaskMonitor | None = None,
        task_seed: int = 0,
    ) -> None:
        self.client = client
        self.policy = policy
        self.task = task
        self.task_seed = task_seed
        self.chunk_size = policy.chunk_size
        self.target_count = target_count
        self.max_chunks = policy.max_chunks
        self.initial_lead_ns = policy.initial_lead_ns
        self.predict_deadline_s = policy.predict_deadline_s
        self.poll_interval_s = policy.poll_interval_s
        self.termination_mode = policy.termination_mode
        self.total_sim_deadline_ns = policy.total_sim_deadline_ns
        self.total_wall_deadline_s = policy.total_wall_deadline_s
        self.task_stop_margin_s = policy.task_stop_margin_s
        self.task_sim_budget_ns = policy.task_sim_budget_ns
        self.warmup_on_settle = policy.warmup_on_settle
        self.async_inference = policy.async_inference
        self.prefetch_lead_ns = policy.prefetch_lead_ns
        self.action_execution_latency_ns = policy.action_execution_latency_ns
        self._context: RunContext | None = None
        self._episode_id = ""
        self._task_epoch = 0
        self._last_task: object | None = None
        self._task_end: tuple[str, str, object | None] = ("", "", None)

    def warm_up(
        self,
        observation: ExactObservation,
        request_id: str,
        *,
        context: RunContext | None = None,
    ) -> None:
        """Warm up the full RPC + GPU inference pipeline before StartTask starts the task clock."""
        effective_context = context or self._context or observation.context
        if effective_context is None:
            raise ContractError("STREAM_NOT_STARTED", "policy warm-up has no context")
        policy_observation = observation_from_exact(
            observation, self.client.profile, self.policy
        )
        self.client.predict(
            effective_context,
            self.client.session_id,
            request_id,
            policy_observation,
            deadline_s=self.predict_deadline_s,
        )
        reset_session = getattr(self.client, "reset", None)
        if callable(reset_session):
            reset_session(effective_context, f"{request_id}-reset")

    def begin(self, context: RunContext, episode_id: str, task_id: str) -> int | None:
        self._context = context
        self._episode_id = episode_id
        if self.termination_mode != "task":
            return None
        if self.task is None:
            raise ContractError("TASK_MONITOR_MISSING", "task mode has no task monitor")
        prepared = self.task.prepare(
            context,
            episode_id,
            f"{episode_id}-prepare",
            self._lease_id,
            self.task_seed,
        )
        self._task_epoch = int(prepared.task_epoch)
        plan_digest = str(prepared.plan_digest)
        return self.task.start(
            context,
            episode_id,
            f"{episode_id}-start",
            self._lease_id,
            self._task_epoch,
            plan_digest,
        )

    @property
    def _lease_id(self) -> str:
        return str(getattr(self, "lease_id", ""))

    def bind_lease(self, lease_id: str) -> None:
        self.lease_id = lease_id

    def predict(
        self, observation: ExactObservation, chunk_index: int, request_id: str,
        *, handoff_time_ns: int = 0,
    ) -> Prediction:
        del handoff_time_ns
        if self._context is None:
            raise ContractError("STREAM_NOT_STARTED", "policy source has no context")
        policy_observation = observation_from_exact(
            observation, self.client.profile, self.policy
        )
        reply: PolicyPrediction = self.client.predict(
            self._context,
            self.client.session_id,
            request_id,
            policy_observation,
            deadline_s=self.predict_deadline_s,
        )
        self._refresh_task()
        end, task_reason, task_state = self._task_end
        return Prediction(
            context=reply.context,
            observation_id=reply.observation_id,
            joint_names=reply.joint_names,
            times_from_observation_ns=reply.times_from_observation_ns,
            positions=reply.positions,
            target_count=reply.target_count,
            chunk_digest="",
            task_end=end,
            task_reason=task_reason,
            task_state=task_state,
        )

    def _refresh_task(self) -> None:
        if self.termination_mode != "task" or self.task is None or self._context is None:
            return
        state = self.task.state(self._context, self._task_epoch)
        self._last_task = state
        runtime, task = task_terminal_fact(state)
        if runtime:
            self._task_end = (runtime, task, state)

    def task_terminal(self) -> tuple[str, str, object | None]:
        return self._task_end

    def finish(self) -> None:
        self.client.close()


class ContinuousStreamRunner:
    """Run a continuous 60x16/50 ms stream with the configured selection window."""

    def __init__(
        self,
        configuration: RuntimeConfiguration,
        context: RunContext,
        episode_id: str,
        task_id: str,
        io: StreamIO,
        control: StreamControl,
        source: TargetSource,
        limits: FollowerValidationLimits,
        *,
        stop_requested: threading.Event,
        log: EventSink | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if not isinstance(limits, FollowerValidationLimits):
            raise ContractError("POLICY_CONFIGURATION", "follower limits must be concrete")
        self.configuration = configuration
        self.profile: RobotProfile = configuration.profile
        self.context = context
        self.episode_id = episode_id
        self.task_id = task_id
        self.io = io
        self.control = control
        self.source = source
        self.limits = limits
        self.stop_requested = stop_requested
        self.log = log
        self.clock = clock
        self.command_id = f"{episode_id}-{self._kind()}"
        self.command_seq = 1
        self.chunks = 0
        self.selected_points = 0
        self.last_observation_id = 0
        self.task_start_sim_time_ns: int | None = None
        self.task_end: tuple[str, str, object | None] = ("", "", None)
        self._wall_deadline = clock() + source.total_wall_deadline_s
        # Policy source needs the live lease for task prepare/start.
        bind = getattr(source, "bind_lease", None)
        if callable(bind):
            bind(control.lease_id)

    def _kind(self) -> str:
        return "stream"

    def _close_runner(self) -> None:
        return None

    def run(self) -> None:
        try:
            self.task_start_sim_time_ns = self.source.begin(
                self.context, self.episode_id, self.task_id
            )
            first_observation = self._first_observation()
            anchor = self.control.get_anchor(self.context)
            self._first_anchor_checks(anchor, first_observation)
            simulation_deadline = self._simulation_deadline(anchor)
            chunk_index = 0
            while True:
                self._check_stop()
                self._check_task_end()
                if chunk_index >= self.source.max_chunks:
                    if self.source.termination_mode == "task":
                        self._finish_task("TASK_TIMEOUT", "TASK_TIMEOUT", None)
                        break
                    self._validate_last_command()
                    break
                chunk_index += 1
                if chunk_index > 1:
                    anchor = self._wait_successor_anchor(chunk_index)
                    self._check_stop()
                    self._check_task_end()
                observation, prediction = self._acquire_chunk_prediction(
                    chunk_index,
                    anchor,
                    first_observation if chunk_index == 1 else None,
                    simulation_deadline,
                )
                self._run_chunk(chunk_index, observation, anchor, prediction=prediction)
                self._check_task_end()
            self._emit_completed()
        finally:
            self._close_runner()

    def _acquire_chunk_prediction(
        self,
        chunk_index: int,
        anchor: object,
        first_observation: ExactObservation | None,
        simulation_deadline: int,
    ) -> tuple[ExactObservation, Prediction]:
        if chunk_index == 1:
            assert first_observation is not None
            observation = first_observation
        else:
            observation = self._wait_successor_observation(anchor)
            self._check_stop()
            if observation.stamp_ns >= anchor.start_sim_time_ns:  # type: ignore[attr-defined]
                raise ContractError(
                    "STREAM_LATE_PREDICTION",
                    "successor observation is not before its fixed hand-off",
                )
        self._check_deadline(observation, simulation_deadline)
        self.last_observation_id = observation.observation_id
        request_id = self._request_id(chunk_index, observation.observation_id)
        try:
            prediction = self.source.predict(
                observation,
                chunk_index,
                request_id,
                handoff_time_ns=int(anchor.start_sim_time_ns),  # type: ignore[attr-defined]
            )
        except TaskTerminal as terminal:
            self.task_end = (terminal.runtime_reason, terminal.task_reason, terminal.state)
            raise _TaskComplete from terminal
        return observation, prediction

    def _request_id(self, chunk_index: int, observation_id: int) -> str:
        return f"{self.episode_id}-{self._kind()}-{chunk_index}-obs-{observation_id}"

    def _wait_successor_observation(self, anchor: object) -> ExactObservation:
        prefetch_lead_ns = int(getattr(self.source, "prefetch_lead_ns", 0))
        period_ns = int(getattr(self.source, "action_period_ns", 50_000_000))
        target_obs_ns = (
            int(anchor.start_sim_time_ns) - prefetch_lead_ns  # type: ignore[attr-defined]
            if prefetch_lead_ns > 0
            else 0
        )
        while True:
            observation = self.io.wait_observation(
                self.last_observation_id,
                self.configuration.recipe.call_timeout_s,
                self.stop_requested,
            )
            self.last_observation_id = observation.observation_id
            self._check_stop()
            self._check_task_end()
            if prefetch_lead_ns <= 0 or observation.stamp_ns + period_ns > target_obs_ns:
                return observation

    def _first_observation(self) -> ExactObservation:
        while True:
            observation = self.io.wait_observation(
                self.last_observation_id,
                self.configuration.recipe.call_timeout_s,
                self.stop_requested,
            )
            self.last_observation_id = observation.observation_id
            self._check_stop()
            if (
                self.task_start_sim_time_ns is not None
                and observation.stamp_ns <= self.task_start_sim_time_ns
            ):
                # A capture at the start timestamp may have been taken before
                # StartTask initialized teacher feedback. Require a later step.
                continue
            return observation

    # ------------------------------------------------------------- sequencing
    def _run_chunk(
        self,
        chunk_index: int,
        observation: ExactObservation,
        anchor: object,
        *,
        prediction: Prediction | None = None,
    ) -> None:
        self.last_observation_id = max(self.last_observation_id, observation.observation_id)
        request_id = self._request_id(chunk_index, observation.observation_id)
        if prediction is None:
            try:
                prediction = self.source.predict(
                    observation,
                    chunk_index,
                    request_id,
                    handoff_time_ns=int(anchor.start_sim_time_ns),  # type: ignore[attr-defined]
                )
            except TaskTerminal as terminal:
                self.task_end = (terminal.runtime_reason, terminal.task_reason, terminal.state)
                raise _TaskComplete from terminal
        self._check_stop()
        self._validate_prediction(prediction, observation)
        pending = prediction.task_end
        if pending:
            # The judge became terminal while the prediction was in flight.  The
            # prediction was never submitted, so it cannot become a label.
            self.task_end = (pending, prediction.task_reason, prediction.task_state)
            return
        final = (
            self.source.termination_mode == "finite_chunks"
            and chunk_index == self.source.max_chunks
        )
        request, retiming = self._request(
            observation, anchor, prediction, chunk_index, request_id, final=final
        )
        if request.window is None or len(request.window.selected_indices) != (
            self.source.target_count
        ):
            raise ContractError(
                "INSUFFICIENT_HORIZON",
                "prediction has fewer than the configured selected points after hand-off",
            )
        self.io.publish_command(request, self.io.robot_time_ns())
        if retiming is not None and retiming.retimed:
            self._emit_retiming(retiming, observation, chunk_index, request_id)
        result = self.control.submit_follower(request)
        self.io.record_follower_reference(result.reference)
        self.io.record_execution(result.execution)
        self.chunks += 1
        self.selected_points += request.target_count
        self._emit(
            "follower.chunk_accepted",
            reason="FOLLOWER_CHUNK_ACCEPTED",
            context=EventContext(
                episode_id=self.episode_id,
                request_id=request_id,
                command_id=self.command_id,
                clock_epoch=self.context.clock_epoch,
                sim_time_ns=observation.stamp_ns,
            ),
            fields={
                "chunk_index": chunk_index,
                "chunk_seq": result.reference.chunk_seq,
                "observation_id": observation.observation_id,
                "state_seq": observation.state_seq,
                "target_count": request.target_count,
                "final": final,
                "source": self._kind(),
                "phase": prediction.phase or None,
                "gate_state": prediction.gate_state or None,
                "accepted_sim_time_ns": result.reference.accepted_sim_time_ns,
                "accepted_wall_time_ns": result.reference.accepted_wall_time_ns,
                "reference_digest": result.reference.reference_digest,
                "chunk_digest": prediction.chunk_digest or None,
                "queried": result.queried,
                "submit_attempts": result.submit_attempts,
            },
        )

    def _request(
        self,
        observation: ExactObservation,
        anchor: object,
        prediction: Prediction,
        chunk_index: int,
        request_id: str,
        *,
        final: bool,
    ) -> tuple[StreamRequest, RetimedWindow | None]:
        targets = tuple(
            Target(time_ns, positions)
            for time_ns, positions in zip(
                prediction.times_from_observation_ns,
                prediction.positions,
                strict=True,
            )
        )
        window = ActionWindow(
            observation.observation_id,
            observation.stamp_ns,
            int(anchor.start_sim_time_ns),  # type: ignore[attr-defined]
            targets,
            self.source.target_count,
        )
        window = self._prepare_window(
            window, observation, anchor, prediction, chunk_index, request_id
        )
        retiming = self._retime(window, anchor)
        response_targets = window.source_targets if retiming is None else retiming.window.source_targets
        accept_before = self.io.robot_time_ns() + self.configuration.recipe.command_acceptance_ns
        request = StreamRequest(
            trajectory=Trajectory(
                self.context,
                self.control.lease_id,
                self.command_id,
                self.command_seq,
                observation.observation_id,
                accept_before,
                self.profile.joint_names,
                response_targets,
            ),
            kind=REQUEST_STREAM_FIRST if chunk_index == 1 else REQUEST_STREAM_NEXT,
            anchor=anchor,  # type: ignore[arg-type]
            observation_time_ns=observation.stamp_ns,
            target_count=self.source.target_count,
            settle_at_end=final,
        )
        return request, retiming

    def _prepare_window(
        self,
        window: ActionWindow,
        observation: ExactObservation,
        anchor: object,
        prediction: Prediction,
        chunk_index: int,
        request_id: str,
    ) -> ActionWindow:
        """Source-specific window preparation hook; shared stream leaves window unchanged."""
        del observation, anchor, prediction, chunk_index, request_id
        return window

    def _retime(self, window: ActionWindow, anchor: object) -> RetimedWindow | None:
        """Source-specific target shaping hook; the shared stream does none."""
        return None

    def _emit_retiming(
        self,
        retiming: RetimedWindow,
        observation: ExactObservation,
        chunk_index: int,
        request_id: str,
    ) -> None:
        """Record each explicit original-to-execution gripper target change."""
        for axis in retiming.axes:
            self._emit(
                "follower.chunk_retimed",
                reason="FOLLOWER_CHUNK_RETIMED",
                context=EventContext(
                    episode_id=self.episode_id,
                    request_id=request_id,
                    command_id=self.command_id,
                    clock_epoch=self.context.clock_epoch,
                    sim_time_ns=observation.stamp_ns,
                ),
                fields={
                    "chunk_index": chunk_index,
                    "observation_id": observation.observation_id,
                    "axis": axis.name,
                    "original_first": axis.original_positions[0],
                    "original_last": axis.original_positions[-1],
                    "adapted_first": axis.adapted_positions[0],
                    "adapted_last": axis.adapted_positions[-1],
                    "original_excess": axis.original_excess,
                    "deviation_budget": axis.budget,
                    "selected_points": len(axis.original_positions),
                },
            )

    def _simulation_deadline(self, anchor: object) -> int:
        source = self.source
        if source.termination_mode == "task" and self.task_start_sim_time_ns is not None:
            return (
                self.task_start_sim_time_ns
                + source.task_sim_budget_ns
                - round(source.task_stop_margin_s * 1e9)
            )
        return int(anchor.start_sim_time_ns) + source.total_sim_deadline_ns  # type: ignore[attr-defined]

    def _check_task_end(self) -> None:
        runtime, task, state = self.source.task_terminal()
        if runtime:
            self.task_end = (runtime, task, state)
            raise _TaskComplete

    def _finish_task(self, runtime: str, task: str, state: object | None) -> None:
        self.task_end = (runtime, task, state)

    def _validate_last_command(self) -> None:
        terminal = self.io.wait_terminal(
            self.command_id,
            self.configuration.recipe.control.terminal_timeout_s,
            self.stop_requested,
        )
        if terminal.phase != "COMPLETED":
            raise ContractError(
                "STREAM_NOT_COMPLETED",
                f"{self.command_id}: {terminal.phase} {terminal.reason}".strip(),
            )

    def _emit_completed(self) -> None:
        self._emit(
            "policy.completed",
            reason="STREAM_COMPLETED",
            fields={
                "chunks": self.chunks,
                "selected_points": self.selected_points,
                "command_id": self.command_id,
                "source": self._kind(),
            },
        )

    def _first_anchor_checks(self, anchor: object, observation: ExactObservation) -> None:
        if int(anchor.predecessor_seq) != 0:  # type: ignore[attr-defined]
            raise ContractError("STREAM_ANCHOR_MISMATCH", "first anchor predecessor is not zero")
        lead = int(anchor.start_sim_time_ns) - observation.stamp_ns  # type: ignore[attr-defined]
        if lead <= 0:
            raise ContractError("STREAM_ANCHOR_MISMATCH", "first hand-off is not in the future")
        if lead > self.source.initial_lead_ns + 500_000_000:
            raise ContractError(
                "STREAM_ANCHOR_MISMATCH", "first hand-off exceeds the expected initial lead"
            )
        if anchor.robot_profile_hash != self.profile.digest:  # type: ignore[attr-defined]
            raise ContractError("PROFILE_MISMATCH", "first anchor profile differs")

    def _wait_successor_anchor(self, chunk_index: int) -> object:
        deadline = min(
            self._wall_deadline,
            self.clock() + self.configuration.recipe.command_timeout_s,
        )
        last_error: Exception | None = None
        while self.clock() < deadline:
            self._check_stop()
            try:
                anchor = self.control.get_anchor(self.context)
            except ContractError as error:
                if error.reason == "STREAM_ANCHOR_UNAVAILABLE":
                    # A stopped stream cannot produce a successor. Recover its
                    # authoritative failure instead of reporting the symptom.
                    try:
                        execution = self.control.query(
                            self.context, self.command_id, ignore_stop=True
                        )
                    except ContractError:
                        raise error from None
                    self.io.record_execution(execution)
                    if execution.phase in ("STOPPING", "FAILED", "CANCELED"):
                        reason = execution.stop_reason or execution.reason
                        if reason:
                            raise ContractError(
                                reason, "stream stopped before successor"
                            ) from error
                    raise
                if error.reason not in {
                    "STREAM_PENDING_EXISTS",
                    "STREAM_NOT_STARTED",
                    "CHUNK_RESULT_EXPIRED",
                    "RESULT_EXPIRED",
                    "STREAM_NOT_CONFIGURED",
                }:
                    raise
                last_error = error
                time.sleep(self.source.poll_interval_s)
                continue
            if int(anchor.predecessor_seq) != chunk_index - 1:
                raise ContractError(
                    "STREAM_ANCHOR_MISMATCH",
                    f"chunk {chunk_index} predecessor is {anchor.predecessor_seq}",
                )
            if anchor.robot_profile_hash != self.profile.digest:
                raise ContractError("PROFILE_MISMATCH", "successor anchor profile differs")
            return anchor
        raise ContractError(
            "STREAM_ANCHOR_TIMEOUT",
            f"no fixed successor anchor for chunk {chunk_index}: {last_error or 'deadline'}",
        )

    # ---------------------------------------------------------------- checks
    def _check_stop(self) -> None:
        if self.stop_requested.is_set():
            raise ContractError("EPISODE_STOPPED", "stream stopped")
        if self.clock() >= self._wall_deadline:
            raise ContractError("STREAM_WALL_DEADLINE", "stream wall deadline exceeded")

    def _check_deadline(self, observation: ExactObservation, simulation_deadline: int) -> None:
        if observation.stamp_ns > simulation_deadline:
            if self.source.termination_mode == "task":
                self._finish_task("TASK_TIMEOUT", "TASK_TIMEOUT", None)
                raise _TaskComplete
            raise ContractError("STREAM_SIM_DEADLINE", "stream simulation deadline exceeded")
        self._check_stop()

    def _validate_prediction(
        self, prediction: Prediction, observation: ExactObservation
    ) -> None:
        from robot_contracts.models import Target
        from robot_trajectory.spline_math import validate_reference_inputs

        if prediction.context != self.context:
            raise ContractError("STREAM_CONTEXT_MISMATCH", "prediction context differs")
        if prediction.observation_id != observation.observation_id:
            raise ContractError("STREAM_OBSERVATION_MISMATCH", "prediction observation differs")
        if len(prediction.times_from_observation_ns) != self.source.chunk_size:
            raise ContractError("STREAM_ACTION_MISMATCH", "prediction is not a complete chunk")
        if len(prediction.positions) != self.source.chunk_size:
            raise ContractError("STREAM_ACTION_MISMATCH", "prediction positions are incomplete")
        if tuple(prediction.joint_names) != self.profile.joint_names:
            raise ContractError("STREAM_ACTION_MISMATCH", "prediction joint order differs")
        if prediction.target_count < self.source.target_count:
            raise ContractError("STREAM_ACTION_MISMATCH", "prediction is too short")
        targets = tuple(
            Target(time_ns, positions)
            for time_ns, positions in zip(
                prediction.times_from_observation_ns,
                prediction.positions,
                strict=True,
            )
        )
        try:
            validate_reference_inputs(
                targets[0].positions,
                targets,
                self.profile.joints,
                tuple(
                    float(value)
                    for value in self.configuration.control_document["max_accelerations"]
                ),
            )
        except (ContractError, KeyError, TypeError, ValueError) as error:
            raise ContractError("STREAM_ACTION_MISMATCH", str(error)) from error

    def _emit(
        self,
        event: str,
        *,
        level: str = "INFO",
        reason: str | None = None,
        context: EventContext | None = None,
        fields: dict[str, Scalar] | None = None,
        exception: BaseException | None = None,
    ) -> None:
        error = emit_best_effort(
            self.log,
            event,
            level=level,
            reason=reason,
            context=context,
            fields=fields,
            exception=exception,
            critical=True,
        )
        if error is not None:
            raise ContractError("LOGGING_ERROR", error)


class _TaskComplete(Exception):
    """Internal control flow: the task judge froze a terminal fact."""


@dataclass(frozen=True, slots=True)
class _CachedPrediction:
    observation_id: int
    stamp_ns: int
    by_abs_time_ns: dict[int, tuple[float, ...]]


class PolicyStreamRunner(ContinuousStreamRunner):
    """Continuous policy runner with true async inference and timestamp-invariant wait advance."""

    def __init__(
        self,
        configuration: RuntimeConfiguration,
        context: RunContext,
        episode_id: str,
        task_id: str,
        io: StreamIO,
        control: StreamControl,
        client: PolicyClient,
        limits: FollowerValidationLimits,
        *,
        stop_requested: threading.Event,
        log: EventSink | None = None,
        clock: Callable[[], float] = time.monotonic,
        task_monitor: TaskMonitor | None = None,
    ) -> None:
        policy = configuration.recipe.policy
        follower = configuration.recipe.follower
        if policy is None:
            raise ContractError("POLICY_CONFIGURATION", "policy settings are absent")
        if follower is None:
            raise ContractError("FOLLOWER_DISABLED", "follower settings are absent")
        self.policy = policy
        self.client = client
        source = PolicyTargetSource(
            client,
            policy,
            follower.target_count,
            task_monitor,
            task_seed=configuration.recipe.reset.seed,
        )
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
        self.retiming_axes = tuple(range(len(self.profile.joint_names)))
        self._prediction_history: deque[_CachedPrediction] = deque(maxlen=12)
        self._async_cond = threading.Condition()
        self._async_thread: threading.Thread | None = None
        self._async_closed = False
        self._async_job: tuple[int, int, int, ExactObservation | None] | None = None
        self._async_completed_chunk = 0
        self._async_latest: tuple[int, ExactObservation, Prediction] | None = None
        self._async_error: BaseException | None = None

    def _kind(self) -> str:
        return "policy"

    def _close_runner(self) -> None:
        thread = self._async_thread
        if thread is None:
            return
        with self._async_cond:
            self._async_closed = True
            self._async_cond.notify_all()
        thread.join(timeout=1.0)
        self._async_thread = None

    def _ensure_async_producer(self) -> None:
        if self._async_thread is not None:
            return
        self._async_closed = False
        thread = threading.Thread(
            target=self._async_producer_loop,
            name="policy-async-producer",
            daemon=True,
        )
        self._async_thread = thread
        thread.start()

    def _record_cached_prediction(
        self, observation: ExactObservation, prediction: Prediction
    ) -> None:
        if prediction.task_end:
            return
        by_abs_time_ns = {
            observation.stamp_ns + time_ns: positions
            for time_ns, positions in zip(
                prediction.times_from_observation_ns,
                prediction.positions,
                strict=True,
            )
        }
        entry = _CachedPrediction(
            observation_id=observation.observation_id,
            stamp_ns=observation.stamp_ns,
            by_abs_time_ns=by_abs_time_ns,
        )
        with self._async_cond:
            if (
                self._prediction_history
                and self._prediction_history[-1].observation_id == entry.observation_id
            ):
                return
            self._prediction_history.append(entry)

    def _async_producer_loop(self) -> None:
        period_ns = int(self.policy.action_period_ns)
        prefetch_lead_ns = int(self.policy.prefetch_lead_ns)
        while True:
            with self._async_cond:
                while (
                    not self._async_closed
                    and (
                        self._async_job is None
                        or self._async_completed_chunk >= self._async_job[0]
                    )
                ):
                    self._async_cond.wait(timeout=0.05)
                if self._async_closed:
                    return
                assert self._async_job is not None
                chunk_index, handoff_ns, target_obs_ns, fixed_obs = self._async_job
            try:
                if fixed_obs is not None:
                    request_id = self._request_id(chunk_index, fixed_obs.observation_id)
                    prediction = self.source.predict(
                        fixed_obs,
                        chunk_index,
                        request_id,
                        handoff_time_ns=handoff_ns,
                    )
                    self._validate_prediction(prediction, fixed_obs)
                    self._record_cached_prediction(fixed_obs, prediction)
                    with self._async_cond:
                        self._async_latest = (chunk_index, fixed_obs, prediction)
                        self._async_completed_chunk = chunk_index
                        self._async_cond.notify_all()
                    continue

                speculative_done = False
                speculative_ns = (
                    handoff_ns - min(2 * prefetch_lead_ns, 450_000_000)
                    if prefetch_lead_ns > 0
                    else 0
                )
                while True:
                    with self._async_cond:
                        if self._async_closed:
                            return
                        if self._async_job is None or self._async_job[0] != chunk_index:
                            break
                    observation = self.io.wait_observation(
                        self.last_observation_id,
                        self.configuration.recipe.call_timeout_s,
                        self.stop_requested,
                    )
                    self.last_observation_id = observation.observation_id
                    if self.stop_requested.is_set():
                        return
                    if observation.stamp_ns >= handoff_ns:
                        with self._async_cond:
                            has_backup = (
                                self._async_latest is not None
                                and self._async_latest[0] == chunk_index
                            )
                            if has_backup:
                                self._async_completed_chunk = chunk_index
                                self._async_cond.notify_all()
                                break
                        raise ContractError(
                            "STREAM_LATE_PREDICTION",
                            "successor observation is not before its fixed hand-off",
                        )
                    reached_target = (
                        prefetch_lead_ns <= 0 or observation.stamp_ns + period_ns > target_obs_ns
                    )
                    run_speculative = (
                        not reached_target
                        and not speculative_done
                        and speculative_ns > 0
                        and observation.stamp_ns + period_ns > speculative_ns
                    )
                    if not reached_target and not run_speculative:
                        continue
                    request_id = self._request_id(chunk_index, observation.observation_id)
                    prediction = self.source.predict(
                        observation,
                        chunk_index,
                        request_id,
                        handoff_time_ns=handoff_ns,
                    )
                    self._validate_prediction(prediction, observation)
                    self._record_cached_prediction(observation, prediction)
                    with self._async_cond:
                        self._async_latest = (chunk_index, observation, prediction)
                        if reached_target:
                            self._async_completed_chunk = chunk_index
                        self._async_cond.notify_all()
                    if reached_target:
                        break
                    speculative_done = True
            except BaseException as error:
                with self._async_cond:
                    self._async_error = error
                    self._async_cond.notify_all()
                return

    def _acquire_chunk_prediction(
        self,
        chunk_index: int,
        anchor: object,
        first_observation: ExactObservation | None,
        simulation_deadline: int,
    ) -> tuple[ExactObservation, Prediction]:
        if not self.policy.async_inference:
            observation, prediction = super()._acquire_chunk_prediction(
                chunk_index, anchor, first_observation, simulation_deadline
            )
            self._validate_prediction(prediction, observation)
            self._record_cached_prediction(observation, prediction)
            return observation, prediction

        self._ensure_async_producer()
        handoff_ns = int(anchor.start_sim_time_ns)  # type: ignore[attr-defined]
        prefetch_lead_ns = int(self.policy.prefetch_lead_ns)
        target_obs_ns = handoff_ns - prefetch_lead_ns if prefetch_lead_ns > 0 else 0
        fallback_margin_ns = (
            min(100_000_000, max(50_000_000, prefetch_lead_ns // 2))
            if prefetch_lead_ns > 0
            else 0
        )
        with self._async_cond:
            self._async_job = (chunk_index, handoff_ns, target_obs_ns, first_observation)
            self._async_cond.notify_all()

        interval = min(float(self.source.poll_interval_s), 0.005)
        while True:
            self._check_stop()
            self._check_task_end()
            with self._async_cond:
                if self._async_error is not None:
                    error = self._async_error
                    self._async_error = None
                    if isinstance(error, TaskTerminal):
                        self.task_end = (error.runtime_reason, error.task_reason, error.state)
                        raise _TaskComplete from error
                    raise error
                latest = self._async_latest
                if (
                    self._async_completed_chunk >= chunk_index
                    and latest is not None
                    and latest[0] == chunk_index
                ):
                    _, observation, prediction = latest
                    break
                if (
                    fallback_margin_ns > 0
                    and latest is not None
                    and latest[0] == chunk_index
                    and self.io.robot_time_ns() >= handoff_ns - fallback_margin_ns
                ):
                    _, observation, prediction = latest
                    break
                self._async_cond.wait(timeout=interval)

        self._check_deadline(observation, simulation_deadline)
        return observation, prediction

    def _axis_motion_threshold(self, axis: int) -> float:
        return 0.003 if self.profile.joints[axis].unit == "m" else 0.030

    def _has_window_progress(
        self,
        positions_seq: tuple[tuple[float, ...], ...],
        baseline: tuple[float, ...],
    ) -> bool:
        """Return True when the window makes net progress from anchor baseline to window end."""
        end_positions = positions_seq[-1]
        return any(
            abs(end - base) >= self._axis_motion_threshold(axis)
            for axis, (end, base) in enumerate(zip(end_positions, baseline, strict=True))
        )

    def _execution_latency_steps(self) -> int:
        latency_ns = int(self.policy.action_execution_latency_ns)
        if latency_ns <= 0:
            return 0
        period_ns = int(self.policy.action_period_ns)
        return round(latency_ns / period_ns)

    def _apply_execution_latency(
        self,
        window: ActionWindow,
        exec_steps: int,
        chunk_index: int,
        request_id: str,
    ) -> ActionWindow:
        """Discard the first ``exec_steps`` predicted steps for UMI-style execution latency matching."""
        if exec_steps <= 0:
            return window
        all_targets = window.source_targets
        last_selected = window.selected_indices[-1]
        if last_selected + exec_steps >= len(all_targets):
            raise ContractError(
                "INSUFFICIENT_HORIZON",
                "prediction horizon cannot cover hand-off plus action_execution_latency_ns",
            )
        last_index = len(all_targets) - 1
        matched = tuple(
            Target(
                all_targets[index].time_ns,
                all_targets[min(index + exec_steps, last_index)].positions,
            )
            for index in range(len(all_targets))
        )
        self._emit(
            "policy.latency_matched",
            reason="LATENCY_MATCHED",
            context=EventContext(
                episode_id=self.episode_id,
                request_id=request_id,
                command_id=self.command_id,
                clock_epoch=self.context.clock_epoch,
                sim_time_ns=window.observation_time_ns,
            ),
            fields={
                "chunk_index": chunk_index,
                "observation_id": window.observation_id,
                "inference_latency_ns": window.handoff_time_ns - window.observation_time_ns,
                "execution_latency_ns": int(self.policy.action_execution_latency_ns),
                "inference_skipped_steps": window.skipped_overlap_count,
                "execution_skipped_steps": exec_steps,
                "total_skipped_steps": window.skipped_overlap_count + exec_steps,
            },
        )
        return ActionWindow(
            window.observation_id,
            window.observation_time_ns,
            window.handoff_time_ns,
            matched,
            window.target_count,
        )

    def _prepare_window(
        self,
        window: ActionWindow,
        observation: ExactObservation,
        anchor: object,
        prediction: Prediction,
        chunk_index: int,
        request_id: str,
    ) -> ActionWindow:
        """Apply latency matching, then newest-wins timestamp-invariant wait advancement."""
        del prediction
        exec_steps = self._execution_latency_steps()
        matched_window = self._apply_execution_latency(window, exec_steps, chunk_index, request_id)
        selected = matched_window.selected_source_targets
        baseline = anchor.reference.positions  # type: ignore[attr-defined]
        selected_positions = tuple(target.positions for target in selected)
        if self._has_window_progress(selected_positions, baseline):
            return matched_window

        exec_offset_ns = exec_steps * int(self.policy.action_period_ns)
        required_abs_times = tuple(
            observation.stamp_ns + target.time_ns + exec_offset_ns for target in selected
        )
        with self._async_cond:
            history = list(reversed(self._prediction_history))
        for entry in history:
            if entry.observation_id == observation.observation_id:
                continue
            if not all(t_abs in entry.by_abs_time_ns for t_abs in required_abs_times):
                continue
            hist_positions = tuple(
                entry.by_abs_time_ns[t_abs] for t_abs in required_abs_times
            )
            if not self._has_window_progress(hist_positions, baseline):
                continue
            merged = list(matched_window.source_targets)
            for idx, positions in zip(matched_window.selected_indices, hist_positions, strict=True):
                merged[idx] = Target(matched_window.source_targets[idx].time_ns, positions)
            self._emit(
                "policy.wait_advance_applied",
                reason="WAIT_ADVANCE_APPLIED",
                context=EventContext(
                    episode_id=self.episode_id,
                    request_id=request_id,
                    command_id=self.command_id,
                    clock_epoch=self.context.clock_epoch,
                    sim_time_ns=observation.stamp_ns,
                ),
                fields={
                    "chunk_index": chunk_index,
                    "observation_id": observation.observation_id,
                    "source_observation_id": entry.observation_id,
                    "source_stamp_ns": entry.stamp_ns,
                    "handoff_sim_time_ns": int(anchor.start_sim_time_ns),  # type: ignore[attr-defined]
                },
            )
            return ActionWindow(
                matched_window.observation_id,
                matched_window.observation_time_ns,
                matched_window.handoff_time_ns,
                tuple(merged),
                matched_window.target_count,
            )
        return matched_window

    def _retime(self, window: ActionWindow, anchor: object) -> RetimedWindow | None:
        return retime_action_window(
            window,
            anchor,  # type: ignore[arg-type]
            self.profile.joints,
            self.limits.max_accelerations,
            self.limits.max_target_deviations,
            self.retiming_axes,
            axis_groups=self.profile.coordinated_axis_groups,
        )

    def run(self) -> None:
        try:
            self._startup_settle()
            super().run()
        except _TaskComplete:
            pass

    def _startup_settle(self) -> None:
        """Discard captures until the configured simulation interval has elapsed.

        The wait is deliberately before ``source.begin`` so it cannot consume the
        task's 30 s budget.  It still uses the stream's bounded observation wait,
        stop flag and wall deadlines, and leaves ``last_observation_id`` at the
        final settled capture so the first ACT prediction can only use a newer one.
        """
        settle_ns = self.policy.startup_settle_sim_ns
        if settle_ns == 0:
            return
        deadline = min(
            self._wall_deadline,
            self.clock() + self.configuration.recipe.startup_timeout_s,
        )
        observation = self._wait_startup_observation(deadline)
        first_observation_id = observation.observation_id
        first_stamp_ns = observation.stamp_ns
        last_stamp_ns = first_stamp_ns
        if self.policy.warmup_on_settle:
            warm_up = getattr(self.source, "warm_up", None)
            if callable(warm_up):
                warm_up(
                    observation,
                    f"{self.episode_id}-warmup-obs-{observation.observation_id}",
                    context=self.context,
                )
        self._emit(
            "policy.startup_settle.started",
            reason="STARTUP_SETTLE_STARTED",
            context=EventContext(
                episode_id=self.episode_id,
                clock_epoch=self.context.clock_epoch,
                sim_time_ns=first_stamp_ns,
            ),
            fields={
                "configured_sim_ns": settle_ns,
                "observation_id": first_observation_id,
                "stamp_ns": first_stamp_ns,
            },
        )
        observations = 1
        while last_stamp_ns - first_stamp_ns < settle_ns:
            previous_stamp_ns = last_stamp_ns
            observation = self._wait_startup_observation(deadline)
            if observation.stamp_ns < previous_stamp_ns:
                raise ContractError(
                    "STREAM_OBSERVATION_REGRESSION",
                    "startup settle observed simulation time going backwards",
                )
            last_stamp_ns = observation.stamp_ns
            observations += 1
        self._emit(
            "policy.startup_settle.completed",
            reason="STARTUP_SETTLE_COMPLETED",
            context=EventContext(
                episode_id=self.episode_id,
                clock_epoch=self.context.clock_epoch,
                sim_time_ns=last_stamp_ns,
            ),
            fields={
                "configured_sim_ns": settle_ns,
                "actual_sim_ns": last_stamp_ns - first_stamp_ns,
                "first_observation_id": first_observation_id,
                "first_stamp_ns": first_stamp_ns,
                "last_observation_id": observation.observation_id,
                "last_stamp_ns": last_stamp_ns,
                "observations": observations,
            },
        )

    def _wait_startup_observation(self, deadline: float) -> ExactObservation:
        self._check_stop()
        remaining = deadline - self.clock()
        if remaining <= 0:
            raise ContractError(
                "STREAM_STARTUP_TIMEOUT",
                "startup settle exceeded startup_timeout_s",
            )
        observation = self.io.wait_observation(
            self.last_observation_id,
            min(self.configuration.recipe.call_timeout_s, remaining),
            self.stop_requested,
        )
        self._check_stop()
        if observation.context != self.context:
            raise ContractError(
                "STREAM_CONTEXT_MISMATCH",
                "startup observation context differs",
            )
        if observation.observation_id <= self.last_observation_id:
            raise ContractError(
                "STREAM_OBSERVATION_MISMATCH",
                "startup observation is not newer than the settled capture",
            )
        self.last_observation_id = observation.observation_id
        return observation


__all__ = [
    "ContinuousStreamRunner",
    "PolicyStreamRunner",
    "PolicyTargetSource",
    "StreamControl",
    "StreamIO",
    "TargetSource",
    "TaskMonitor",
    "follower_limits",
]
