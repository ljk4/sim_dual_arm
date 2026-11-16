"""Independent handover task state, measured gates and final physical proof.

The runtime advances behaviour phases only from owner-thread observations.  It
never treats a teacher phase name, a completed plan segment or an accepted
command as evidence: holds come from two-finger contact, lift from the block
leaving the table, transfer from the right-hand relative pose, and success from
the eight placed vertices, box-bottom support, both hands released and a
continuous stable window.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from robot_contracts.models import ContractError, RunContext
from robot_contracts.task_state import TaskState

from robot_backend_sim.tasks.handover.gate import HandoverGateFact
from robot_backend_sim.tasks.handover.observation import (
    HandoverBinding,
    HandoverObservation,
    relative_block_pose,
    rotation_angle,
)
from robot_backend_sim.tasks.handover.plan import _placement_containment
from robot_backend_sim.tasks.handover.task_config import HandoverTaskConfig

if TYPE_CHECKING:
    from robot_backend_sim.tasks.handover.teacher import HandoverPlan, HandoverStep

__all__ = [
    "HandoverTaskError",
    "HandoverTaskRuntime",
    "TaskCondition",
    "evaluation_conditions",
]

_TERMINAL_STATES = ("SUCCEEDED", "FAILED")
_LEFT, _RIGHT = "left", "right"


class HandoverTaskError(ContractError):
    """A bounded task-interface error using the public ContractError shape."""


def _error(reason: str, message: str) -> HandoverTaskError:
    return HandoverTaskError(reason, message)


@dataclass(frozen=True, slots=True)
class TaskCondition:
    name: str
    passed: bool
    actual: float
    threshold: float
    unit: str
    detail: str


@dataclass(frozen=True, slots=True)
class PhaseEvent:
    phase: str
    step: str
    sim_time_ns: int
    wait_s: float


@dataclass(frozen=True, slots=True)
class _StabilityReference:
    position: tuple[float, float, float]
    rotation: tuple[tuple[float, ...], ...]


@dataclass(frozen=True, slots=True)
class _SequenceFact:
    """Owner-thread proof of the real airborne handover order."""

    phase: str
    left_airborne_stable: bool
    dual_airborne_stable: bool
    left_release_observed: bool
    right_airborne_stable: bool
    complete: bool
    left_stable_sim_time_ns: int | None
    dual_stable_sim_time_ns: int | None
    left_release_sim_time_ns: int | None
    right_stable_sim_time_ns: int | None
    left_hold_elapsed_s: float
    dual_hold_elapsed_s: float
    right_hold_elapsed_s: float
    left_min_force_n: float
    right_min_force_n: float
    handover_window: bool


@dataclass(frozen=True, slots=True)
class _PrepareFact:
    request_id: str
    content_digest: str
    plan_digest: str
    state: TaskState
    step_count: int = 0
    ik_max_position_error_m: float = 0.0
    ik_max_orientation_error_rad: float = 0.0
    plan_duration_s: float = 0.0
    prepare_kind: str = "teacher"


@dataclass(frozen=True, slots=True)
class _StartFact:
    request_id: str
    content_digest: str
    start_sim_time_ns: int
    state: TaskState


@dataclass(frozen=True, slots=True)
class _PrepareOutcome:
    fact: _PrepareFact
    idempotent: bool


def _norm(values: tuple[float, ...]) -> float:
    return math.sqrt(sum(value * value for value in values))


def _vertices(layout: object, observation: HandoverObservation) -> list[tuple[float, float, float]]:
    import numpy as np

    from robot_backend_sim.tasks.handover.observation import quaternion_matrix

    half = np.asarray(layout.block_half_extents, dtype=np.float64)  # type: ignore[attr-defined]
    rotation = np.asarray(quaternion_matrix(observation.block_quaternion_wxyz))
    center = np.asarray(observation.block_center, dtype=np.float64)
    corners = np.array(
        [
            [sx * half[0], sy * half[1], sz * half[2]]
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
            for sz in (-1.0, 1.0)
        ],
        dtype=np.float64,
    )
    return [tuple(float(value) for value in row) for row in corners @ rotation.T + center]


def _containment(
    layout: object, observation: HandoverObservation
) -> tuple[bool, str]:
    import numpy as np

    from robot_backend_sim.tasks.handover.observation import quaternion_matrix

    rotation = np.asarray(quaternion_matrix(observation.block_quaternion_wxyz))
    return _placement_containment(
        layout,  # type: ignore[arg-type]
        np.asarray(observation.block_center, dtype=np.float64),
        rotation,
        layout.block_half_extents,  # type: ignore[attr-defined]
    )


def evaluation_conditions(
    observation: HandoverObservation,
    binding: HandoverBinding,
    config: HandoverTaskConfig,
    layout: object,
    *,
    position_reference: tuple[float, float, float] | None,
    rotation_reference: tuple[tuple[float, ...], ...] | None,
    sequence_complete: bool,
) -> tuple[TaskCondition, ...]:
    """Every independent final-state condition; no single fact is a shortcut."""
    success = config.success
    contained, message = _containment(layout, observation)
    box_support = observation.support_upward_force(binding.box_bottom_geom_names)
    table_support = observation.support_force(binding.support_body_names[0])
    left_released = not observation.left_contacts
    right_released = not observation.right_contacts
    linear_speed = _norm(observation.block_linear_velocity)
    angular_speed = _norm(observation.block_angular_velocity)
    drift = 0.0
    rotation_drift = 0.0
    if position_reference is not None:
        drift = math.sqrt(
            sum(
                (observation.block_center[index] - position_reference[index]) ** 2
                for index in range(3)
            )
        )
    if rotation_reference is not None:
        from robot_backend_sim.tasks.handover.observation import quaternion_matrix

        rotation_drift = rotation_angle(
            rotation_reference, quaternion_matrix(observation.block_quaternion_wxyz)
        )
    specs = (
        ("block_contained", contained, 1.0 if contained else 0.0, 1.0, "bool", message),
        (
            "box_bottom_support",
            box_support >= success.min_box_support_normal_force_n,
            box_support,
            success.min_box_support_normal_force_n,
            "N",
            "the block must rest on the real box bottom",
        ),
        (
            "not_on_table",
            table_support < success.min_box_support_normal_force_n,
            table_support,
            success.min_box_support_normal_force_n,
            "N",
            "the block must not be supported by the table",
        ),
        (
            "left_released",
            left_released,
            0.0 if left_released else 1.0,
            0.0,
            "contacts",
            "the left fingers must not touch the block",
        ),
        (
            "right_released",
            right_released,
            0.0 if right_released else 1.0,
            0.0,
            "contacts",
            "the right fingers must not touch the block",
        ),
        (
            "block_linear_speed",
            linear_speed <= success.max_linear_speed_m_s,
            linear_speed,
            success.max_linear_speed_m_s,
            "m/s",
            "the block must be at rest",
        ),
        (
            "block_angular_speed",
            angular_speed <= success.max_angular_speed_rad_s,
            angular_speed,
            success.max_angular_speed_rad_s,
            "rad/s",
            "the block must not rotate",
        ),
        (
            "block_position_drift",
            drift <= success.max_position_drift_m,
            drift,
            success.max_position_drift_m,
            "m",
            "the block must hold its placed pose",
        ),
        (
            "block_orientation_drift",
            rotation_drift <= success.max_orientation_drift_rad,
            rotation_drift,
            success.max_orientation_drift_rad,
            "rad",
            "the block must hold its placed orientation",
        ),
        (
            "airborne_handover_sequence",
            sequence_complete,
            1.0 if sequence_complete else 0.0,
            1.0,
            "bool",
            "left sole airborne hold, airborne dual hold, then left release "
            "with a right sole airborne hold, strictly in order",
        ),
    )
    return tuple(TaskCondition(*spec) for spec in specs)


class HandoverTaskRuntime:
    """One task's prepare/start/tick/reset state machine and teacher gate."""

    def __init__(
        self,
        config: HandoverTaskConfig,
        binding: HandoverBinding,
        layout: object,
    ) -> None:
        self.config = config
        self.binding = binding
        self.layout = layout
        self._seq = self._task_epoch = 0
        self._state, self._reason = "IDLE", ""
        self._context: RunContext | None = None
        self._plan: HandoverPlan | None = None
        self._plan_digest, self._plan_duration_ns = "", 0
        self._start_sim_ns = self._start_wall_ns = 0
        self._observation: HandoverObservation | None = None
        self._conditions: tuple[TaskCondition, ...] = ()
        self._success_observation: HandoverObservation | None = None
        self._success_conditions: tuple[TaskCondition, ...] = ()
        self._prepare_cache: dict[str, _PrepareFact] = {}
        self._start_fact: _StartFact | None = None
        self._step_index = 0
        self._step_active_ns = -1
        self._step_stable_since_ns: int | None = None
        self._step_wait_s = 0.0
        self._step_status = "TRACKING"
        self._failure = ""
        self._events: list[PhaseEvent] = []
        self._established: dict[str, bool] = {_LEFT: False, _RIGHT: False}
        self._loss_since_ns: dict[str, int | None] = {_LEFT: None, _RIGHT: None}
        self._references: dict[str, _StabilityReference] = {}
        self._handover_window = False
        self._window_violation = ""
        self._right_catch_ns = self._left_release_ns = 0
        self._left_retreat_done = False
        self._success_since_ns: int | None = None
        self._first_satisfied_ns = self._completed_ns = 0
        self._success_position_reference: tuple[float, float, float] | None = None
        self._success_rotation_reference: tuple[tuple[float, ...], ...] | None = None
        self._behavior_phase = "idle"
        self._sequence_phase = "left"
        self._sequence_since_ns: int | None = None
        self._sequence_started_ns: dict[str, int | None] = {
            "left": None,
            "dual": None,
            "right": None,
        }
        self._sequence_times: dict[str, int | None] = {
            "left": None,
            "dual": None,
            "release": None,
            "right": None,
        }
        self._sequence_references: dict[str, _StabilityReference] = {}

    # -------------------------------------------------------------- properties
    @property
    def state(self) -> TaskState:
        return self._snapshot()

    @property
    def plan(self) -> HandoverPlan | None:
        return self._plan

    @property
    def plan_digest(self) -> str:
        return self._plan_digest

    @property
    def plan_duration_ns(self) -> int:
        return self._plan_duration_ns

    @property
    def task_epoch(self) -> int:
        return self._task_epoch

    @property
    def context(self) -> RunContext | None:
        return self._context

    @property
    def behavior_phase(self) -> str:
        return self._behavior_phase

    @property
    def step_name(self) -> str:
        if self._plan is None or not self._plan.steps:
            return ""
        return self._plan.steps[min(self._step_index, len(self._plan.steps) - 1)].name

    @property
    def step_index(self) -> int:
        return self._step_index

    @property
    def failure(self) -> str:
        return self._failure

    def gate_fact(self) -> HandoverGateFact:
        """Freeze the current measured step fact for one retained capture."""
        stable = 0.0
        if self._step_stable_since_ns is not None and self._observation is not None:
            stable = max(
                0.0,
                (self._observation.sim_time_ns - self._step_stable_since_ns) / 1e9,
            )
        return HandoverGateFact(
            step_index=self._step_index,
            step=self.step_name,
            phase=self._behavior_phase,
            status=self._step_status,
            active_start_sim_time_ns=self._step_active_ns,
            phase_wait_s=self._step_wait_s,
            stable_elapsed_s=stable,
            failure=self._failure,
        )

    # ---------------------------------------------------------------- prepare
    def require_prepare_slot(self) -> None:
        if self._state == "RUNNING":
            raise _error("TASK_BUSY", "a handover task is already running")
        if self._state == "PREPARED":
            raise _error("TASK_ALREADY_PREPARED", "reset before preparing a different plan")
        if self._state in _TERMINAL_STATES:
            raise _error("TASK_TERMINAL", "reset the terminal task before preparing again")
        if len(self._prepare_cache) >= self.config.capacity.rpc_cache_entries:
            raise _error("TASK_CACHE_FULL", "prepare idempotency cache is full")

    def lookup_prepare(self, request_id: str, content_digest: str) -> _PrepareOutcome | None:
        fact = self._prepare_cache.get(request_id)
        if fact is None:
            return None
        if fact.content_digest != content_digest:
            raise _error("REQUEST_ID_CONFLICT", "same prepare request_id has different content")
        return _PrepareOutcome(fact, idempotent=True)

    def commit_prepare(
        self,
        *,
        request_id: str,
        content_digest: str,
        context: RunContext,
        plan: HandoverPlan,
        plan_digest: str,
        plan_duration_ns: int,
        step_count: int = 0,
        ik_max_position_error_m: float = 0.0,
        ik_max_orientation_error_rad: float = 0.0,
        prepare_kind: str = "teacher",
    ) -> _PrepareOutcome:
        existing = self.lookup_prepare(request_id, content_digest)
        if existing is not None:
            return existing
        self.require_prepare_slot()
        if not isinstance(plan_digest, str) or not plan_digest:
            raise _error("INVALID_TASK_PLAN", "plan digest must be non-empty")
        if plan_duration_ns <= 0:
            raise _error("INVALID_TASK_PLAN", "plan duration must be positive")
        self._task_epoch += 1
        self._seq += 1
        self._context, self._plan = context, plan
        self._plan_digest, self._plan_duration_ns = plan_digest, plan_duration_ns
        self._state, self._reason = "PREPARED", ""
        self._start_sim_ns = self._start_wall_ns = 0
        self._observation = None
        self._conditions = ()
        self._success_observation = None
        self._success_conditions = ()
        self._start_fact = None
        self._reset_phase_state()
        fact = _PrepareFact(
            request_id,
            content_digest,
            plan_digest,
            self._snapshot(),
            step_count,
            ik_max_position_error_m,
            ik_max_orientation_error_rad,
            plan_duration_ns / 1e9,
            prepare_kind,
        )
        self._prepare_cache[request_id] = fact
        return _PrepareOutcome(fact, idempotent=False)

    def _reset_phase_state(self) -> None:
        self._step_index = 0
        self._step_active_ns = -1
        self._step_stable_since_ns = None
        self._step_wait_s = 0.0
        self._step_status = "TRACKING"
        self._failure = ""
        self._events = []
        self._established = {_LEFT: False, _RIGHT: False}
        self._loss_since_ns = {_LEFT: None, _RIGHT: None}
        self._references = {}
        self._handover_window = False
        self._window_violation = ""
        self._right_catch_ns = self._left_release_ns = 0
        self._left_retreat_done = False
        self._success_since_ns = None
        self._first_satisfied_ns = self._completed_ns = 0
        self._success_position_reference = None
        self._success_rotation_reference = None
        self._behavior_phase = "idle"
        self._sequence_phase = "left"
        self._sequence_since_ns = None
        self._sequence_started_ns = {"left": None, "dual": None, "right": None}
        self._sequence_times = {
            "left": None,
            "dual": None,
            "release": None,
            "right": None,
        }
        self._sequence_references = {}

    def start(
        self,
        *,
        request_id: str,
        content_digest: str,
        context: RunContext,
        plan_digest: str,
        task_epoch: int,
        sim_time_ns: int,
        wall_ns: int,
    ) -> tuple[TaskState, bool]:
        fact = self._start_fact
        if fact is not None and self._state in ("RUNNING", *_TERMINAL_STATES):
            if fact.request_id != request_id or fact.content_digest != content_digest:
                raise _error("START_CONFLICT", "task has already started with a different request")
            return self._snapshot(), True
        if self._state != "PREPARED" or self._plan is None:
            raise _error("TASK_NOT_PREPARED", "no prepared task plan exists")
        if (
            context != self._context
            or plan_digest != self._plan_digest
            or task_epoch != self._task_epoch
        ):
            raise _error("STALE_TASK_CONTEXT", "start does not match the prepared task")
        self._seq += 1
        self._start_sim_ns, self._start_wall_ns = int(sim_time_ns), int(wall_ns)
        self._reset_phase_state()
        self._state, self._reason = "RUNNING", ""
        if self._plan.steps:
            self._step_active_ns = self._start_sim_ns
            self._behavior_phase = self._plan.steps[0].phase
        state = self._snapshot()
        self._start_fact = _StartFact(request_id, content_digest, self._start_sim_ns, state)
        return state, False

    # ------------------------------------------------------------------- tick
    def tick(self, observation: HandoverObservation, wall_ns: int) -> TaskState:
        self._seq += 1
        self._observation = observation
        if self._state != "RUNNING":
            return self._snapshot()
        elapsed_ns = observation.sim_time_ns - self._start_sim_ns
        if elapsed_ns < 0:
            raise _error("INVALID_TASK_CLOCK", "task sim clock moved backwards")
        if observation.sim_time_ns > self._start_sim_ns + round(
            self.config.success.max_task_sim_duration_s * 1e9
        ):
            self._fail("TIMEOUT", "task exceeded the configured simulation budget")
            return self._snapshot()
        if int(wall_ns) - self._start_wall_ns > round(
            self.config.success.max_task_wall_duration_s * 1e9
        ):
            self._fail("WALL_TIMEOUT", "task exceeded the configured wall watchdog")
            return self._snapshot()
        if self._failure:
            return self._snapshot()

        self._update_sequence(observation)
        sequence = self._sequence_fact()
        conditions = evaluation_conditions(
            observation,
            self.binding,
            self.config,
            self.layout,
            position_reference=self._success_position_reference,
            rotation_reference=self._success_rotation_reference,
            sequence_complete=sequence.complete,
        )
        self._conditions = conditions
        self._update_physical_guards(observation)
        if self._failure:
            return self._snapshot()

        if self._plan is not None and self._plan.steps:
            self._update_steps(observation)
        else:
            self._behavior_phase = self._derived_phase(observation)
        self._update_success(observation, conditions)
        return self._snapshot()

    # -------------------------------------------------------------- step gates
    def _update_steps(self, observation: HandoverObservation) -> None:
        plan = self._plan
        assert plan is not None
        if self._step_active_ns < 0:
            self._step_active_ns = observation.sim_time_ns
        step = plan.steps[min(self._step_index, len(plan.steps) - 1)]
        self._behavior_phase = step.phase
        minimum = self.config.planning.diagnostics.contact_force_min_n
        for side in step.holders:
            if observation.holding(side, minimum):
                self._remember_reference(observation, side)
            elif not self._established[side]:
                self._references.pop(side, None)
        elapsed_ns = observation.sim_time_ns - self._step_active_ns
        duration_ns = round(step.duration_s * 1e9)
        timeout_ns = duration_ns + round(self.config.gates.phase_extra_timeout_s * 1e9)
        self._step_wait_s = max(0.0, (elapsed_ns - duration_ns) / 1e9)
        passed = self._condition(observation, step)
        if passed:
            if self._step_stable_since_ns is None:
                self._step_stable_since_ns = observation.sim_time_ns
        else:
            self._step_stable_since_ns = None
        stable_ns = (
            0
            if self._step_stable_since_ns is None
            else observation.sim_time_ns - self._step_stable_since_ns
        )
        if step.final:
            self._step_status = "HOLDING" if passed else "WAITING_FOR_FEEDBACK"
        elif elapsed_ns < duration_ns:
            self._step_status = "TRACKING"
        else:
            self._step_status = "WAITING_FOR_FEEDBACK" if not passed else "HOLDING"
        if self._failure:
            self._step_status = "FAILED"
            return
        if (
            not step.final
            and passed
            and elapsed_ns >= duration_ns
            and stable_ns >= round(self.config.gates.stable_duration_s * 1e9)
        ):
            self._advance_step(observation)
            return
        if elapsed_ns > timeout_ns:
            self._fail(
                "GATE_TIMEOUT",
                f"{step.phase}/{step.name}: gate did not pass within "
                f"{step.duration_s + self.config.gates.phase_extra_timeout_s:.2f} s",
            )

    def _advance_step(self, observation: HandoverObservation) -> None:
        plan = self._plan
        assert plan is not None
        step = plan.steps[self._step_index]
        self._events.append(
            PhaseEvent(step.phase, step.name, observation.sim_time_ns, self._step_wait_s)
        )
        if step.gate == "left_grasp":
            self._established[_LEFT] = True
            self._remember_reference(observation, _LEFT)
        elif step.gate == "dual_hold":
            self._established[_LEFT] = True
            self._established[_RIGHT] = True
            self._remember_reference(observation, _RIGHT)
            self._right_catch_ns = observation.sim_time_ns
        elif step.gate == "left_release":
            self._established[_LEFT] = False
            self._references.pop(_LEFT, None)
            self._left_release_ns = observation.sim_time_ns
        elif step.gate == "left_retreat":
            self._left_retreat_done = True
        if step.gate == "released":
            self._established[_RIGHT] = False
            self._references.pop(_RIGHT, None)
        self._step_index += 1
        if self._step_index >= len(plan.steps):
            self._step_index = len(plan.steps) - 1
        self._step_active_ns = observation.sim_time_ns
        self._step_stable_since_ns = None
        self._step_wait_s = 0.0

    # --------------------------------------------------------- handover proof
    def _update_sequence(self, observation: HandoverObservation) -> None:
        """Measure the handover order from physics, independent of plan steps."""
        minimum = self.config.planning.diagnostics.contact_force_min_n
        left_holding = observation.holding(_LEFT, minimum)
        right_holding = observation.holding(_RIGHT, minimum)
        externally_supported = observation.external_support_force(()) > 0.0
        airborne_left_candidate = (
            left_holding
            and not right_holding
            and not externally_supported
            and observation.block_height_above_initial_m
            >= self.config.gates.lift_height_m
        )
        if airborne_left_candidate or (
            left_holding and self._sequence_times["left"] is not None
        ):
            self._remember_sequence_reference(observation, _LEFT)
        elif self._sequence_times["left"] is None:
            self._sequence_references.pop(_LEFT, None)
        airborne_dual_candidate = (
            left_holding
            and right_holding
            and not externally_supported
            and self._sequence_times["left"] is not None
        )
        if airborne_dual_candidate or (
            right_holding and self._sequence_times["dual"] is not None
        ):
            self._remember_sequence_reference(observation, _RIGHT)
        elif self._sequence_times["dual"] is None:
            self._sequence_references.pop(_RIGHT, None)

        phase = self._sequence_phase
        if phase == "left":
            condition = (
                left_holding
                and not right_holding
                and not externally_supported
                and observation.block_height_above_initial_m
                >= self.config.gates.lift_height_m
                and self._sequence_side_stable(observation, _LEFT)
            )
        elif phase == "dual":
            condition = (
                left_holding
                and right_holding
                and not externally_supported
                and self._sequence_side_stable(observation, _LEFT)
                and self._sequence_side_stable(observation, _RIGHT)
            )
        elif phase == "right":
            condition = (
                right_holding
                and not externally_supported
                and self._sequence_side_stable(observation, _RIGHT)
                and not observation.left_contacts
                and observation.left_opening_m
                >= self.config.gates.release_opening_m
                - self.config.gates.gripper_position_tolerance_m
                and observation.left_finger_speed_m_s
                <= self.config.gates.release_velocity_max_m_s
            )
        else:
            condition = False

        now = observation.sim_time_ns
        if not condition:
            self._sequence_since_ns = None
            if phase == "right":
                self._sequence_started_ns["right"] = None
                self._sequence_times["release"] = None
            elif phase == "left":
                self._sequence_started_ns["left"] = None
            elif phase == "dual":
                self._sequence_started_ns["dual"] = None
            self._handover_window = self._proof_window()
            return
        if self._sequence_since_ns is None:
            self._sequence_since_ns = now
            self._sequence_started_ns[phase] = now
            if phase == "right":
                self._sequence_times["release"] = now
        if now - self._sequence_since_ns >= round(
            self.config.gates.stable_duration_s * 1e9
        ):
            self._sequence_times[phase] = now
            self._sequence_since_ns = None
            if phase == "left":
                self._sequence_phase = "dual"
            elif phase == "dual":
                self._sequence_phase = "right"
            else:
                self._sequence_phase = "complete"
        self._handover_window = self._proof_window()

    def _proof_window(self) -> bool:
        return (
            self._sequence_times["left"] is not None
            and self._sequence_phase != "complete"
        )

    def _remember_sequence_reference(
        self, observation: HandoverObservation, side: str
    ) -> None:
        if side in self._sequence_references:
            return
        position, rotation = relative_block_pose(observation, side)
        self._sequence_references[side] = _StabilityReference(position, rotation)

    def _sequence_side_stable(self, observation: HandoverObservation, side: str) -> bool:
        return self._side_stable(
            observation,
            side,
            require_hold=True,
            require_airborne=False,
            references=self._sequence_references,
        )

    def _sequence_fact(self) -> _SequenceFact:
        now = 0 if self._observation is None else self._observation.sim_time_ns

        def elapsed(key: str) -> float:
            started = self._sequence_started_ns[key]
            if started is None:
                return 0.0
            completed = self._sequence_times[key]
            end = now if completed is None else completed
            return max(0.0, (end - started) / 1e9)

        times = self._sequence_times
        observation = self._observation
        return _SequenceFact(
            phase=self._sequence_phase,
            left_airborne_stable=times["left"] is not None,
            dual_airborne_stable=times["dual"] is not None,
            left_release_observed=times["release"] is not None,
            right_airborne_stable=times["right"] is not None,
            complete=self._sequence_phase == "complete",
            left_stable_sim_time_ns=times["left"],
            dual_stable_sim_time_ns=times["dual"],
            left_release_sim_time_ns=times["release"],
            right_stable_sim_time_ns=times["right"],
            left_hold_elapsed_s=elapsed("left"),
            dual_hold_elapsed_s=elapsed("dual"),
            right_hold_elapsed_s=elapsed("right"),
            left_min_force_n=(
                0.0 if observation is None else _min_force(observation, _LEFT)
            ),
            right_min_force_n=(
                0.0 if observation is None else _min_force(observation, _RIGHT)
            ),
            handover_window=self._handover_window,
        )

    def _condition(self, observation: HandoverObservation, step: HandoverStep) -> bool:
        gates = self.config.gates
        if step.gate == "position":
            return self._arm_at_target(observation, step)
        if step.gate == "left_grasp":
            return (
                self._arm_at_target(observation, step)
                and observation.holding(_LEFT, self.config.planning.diagnostics.contact_force_min_n)
                and observation.left_finger_speed_m_s
                <= gates.release_velocity_max_m_s
            )
        if step.gate in ("left_lift", "handover"):
            held = self._side_stable(
                observation, _LEFT, require_hold=True, require_airborne=True
            )
            if step.gate == "left_lift":
                return (
                    held
                    and self._arm_at_target(observation, step)
                    and observation.block_height_above_initial_m >= gates.lift_height_m
                )
            return held and self._arm_at_target(observation, step)
        if step.gate == "dual_hold":
            return (
                self._arm_at_target(observation, step)
                and observation.holding(_LEFT, self.config.planning.diagnostics.contact_force_min_n)
                and self._side_stable(
                    observation, _RIGHT, require_hold=True, require_airborne=True
                )
            )
        if step.gate == "left_release":
            return (
                self._side_stable(
                    observation, _RIGHT, require_hold=True, require_airborne=True
                )
                and not observation.left_contacts
                and observation.left_opening_m
                >= gates.release_opening_m - gates.gripper_position_tolerance_m
                and observation.left_finger_speed_m_s <= gates.release_velocity_max_m_s
            )
        if step.gate in ("right_hold", "right_carry"):
            held = self._side_stable(
                observation, _RIGHT, require_hold=True, require_airborne=True
            )
            return held and self._arm_at_target(observation, step)
        if step.gate == "left_retreat":
            return (
                self._side_stable(
                    observation, _RIGHT, require_hold=True, require_airborne=True
                )
                and not observation.left_contacts
                and self._arm_at_target(observation, step)
            )
        if step.gate == "right_place":
            contained, _message = _containment(self.layout, observation)
            return (
                contained
                and self._side_stable(
                    observation, _RIGHT, require_hold=True, require_airborne=False
                )
                and self._arm_at_target(observation, step)
            )
        if step.gate == "released":
            success = self.config.success
            contained, _message = _containment(self.layout, observation)
            box_support = observation.support_upward_force(
                self.binding.box_bottom_geom_names
            )
            return (
                contained
                and box_support >= success.min_box_support_normal_force_n
                and not observation.left_contacts
                and not observation.right_contacts
            )
        if step.gate == "stable":
            return all(condition.passed for condition in self._conditions)
        return False

    def _arm_at_target(self, observation: HandoverObservation, step: HandoverStep) -> bool:
        gates = self.config.gates
        plan = self._plan
        assert plan is not None
        left_target = step.end_positions[0:7]
        right_target = step.end_positions[8:15]
        left_ok = all(
            abs(measured - wanted) <= gates.arm_position_tolerance_rad
            for measured, wanted in zip(observation.left_positions, left_target, strict=True)
        ) and all(
            abs(value) <= gates.arm_velocity_tolerance_rad_s
            for value in observation.left_velocities
        )
        right_ok = all(
            abs(measured - wanted) <= gates.arm_position_tolerance_rad
            for measured, wanted in zip(observation.right_positions, right_target, strict=True)
        ) and all(
            abs(value) <= gates.arm_velocity_tolerance_rad_s
            for value in observation.right_velocities
        )
        if not (left_ok and right_ok):
            return False
        for _side, target, measured in (
            (_LEFT, step.end_positions[7], observation.left_opening_m),
            (_RIGHT, step.end_positions[15], observation.right_opening_m),
        ):
            if abs(target - plan.open_total_m) <= 1e-3 and (
                abs(measured - target) > gates.gripper_position_tolerance_m
            ):
                return False
        return True

    def _side_stable(
        self,
        observation: HandoverObservation,
        side: str,
        *,
        require_hold: bool,
        require_airborne: bool,
        references: dict[str, _StabilityReference] | None = None,
    ) -> bool:
        diagnostics = self.config.planning.diagnostics
        if require_hold and not observation.holding(
            side, diagnostics.contact_force_min_n
        ):
            return False
        if require_airborne and self._external_support(observation) > 0.0:
            return False
        source = self._references if references is None else references
        reference = source.get(side)
        if reference is None:
            return False
        position, rotation = relative_block_pose(observation, side)
        if math.sqrt(
            sum((position[index] - reference.position[index]) ** 2 for index in range(3))
        ) > diagnostics.relative_position_tolerance_m:
            return False
        if rotation_angle(reference.rotation, rotation) > (
            diagnostics.relative_orientation_tolerance_rad
        ):
            return False
        return True

    def _remember_reference(self, observation: HandoverObservation, side: str) -> None:
        if side in self._references:
            return
        position, rotation = relative_block_pose(observation, side)
        self._references[side] = _StabilityReference(position, rotation)

    def _external_support(self, observation: HandoverObservation) -> float:
        # Fingers are not support contacts, so every reported support body is an
        # external support.  Box support is allowed only after placement starts.
        return observation.external_support_force(self._allowed_support())

    def _allowed_support(self) -> tuple[str, ...]:
        if self._plan is None or not self._plan.steps:
            return ()
        step = self._plan.steps[min(self._step_index, len(self._plan.steps) - 1)]
        return self.binding.support_body_names[1:] if step.allow_box_support else ()

    # ---------------------------------------------------------- physical guards
    def _update_physical_guards(self, observation: HandoverObservation) -> None:
        gates = self.config.gates
        if observation.robot_robot_force_n > gates.forbidden_contact_force_n:
            self._fail(
                "FORBIDDEN_CONTACT",
                f"robot-robot contact {observation.robot_robot_force_n:.3f} N",
            )
            return
        if observation.robot_box_force_n > gates.forbidden_contact_force_n:
            self._fail(
                "FORBIDDEN_CONTACT",
                f"robot-box contact {observation.robot_box_force_n:.3f} N",
            )
            return
        if self._handover_window:
            external = self._external_support(observation)
            if external > 0.0:
                self._window_violation = "external support during handover"
                self._fail(
                    "EXTERNAL_SUPPORT",
                    "the block was supported by an external body inside the handover window",
                )
                return
        floor = self.binding.box_inner_bottom_z_m - 0.02
        if observation.block_center[2] < min(self.binding.table_top_z_m - 0.02, floor):
            self._fail("DROPPED", "the block fell below the workspace")
            return
        self._update_hold_loss(observation)

    def _update_hold_loss(self, observation: HandoverObservation) -> None:
        minimum = self.config.planning.diagnostics.contact_force_min_n
        tolerance_ns = round(self.config.gates.hold_loss_tolerance_s * 1e9)
        for side in (_LEFT, _RIGHT):
            if not self._holder_required(observation, side):
                self._loss_since_ns[side] = None
                continue
            if observation.holding(side, minimum):
                self._loss_since_ns[side] = None
                continue
            since = self._loss_since_ns[side]
            if since is None:
                self._loss_since_ns[side] = observation.sim_time_ns
                since = observation.sim_time_ns
            if observation.sim_time_ns - since > tolerance_ns:
                step = (
                    None
                    if self._plan is None or not self._plan.steps
                    else self._plan.steps[
                        min(self._step_index, len(self._plan.steps) - 1)
                    ]
                )
                self._fail(
                    "HOLD_LOST",
                    f"the {side} hand lost the block during "
                    f"{'the handover' if step is None else step.phase}",
                )
                return

    def _holder_required(self, observation: HandoverObservation, side: str) -> bool:
        if self._plan is not None and self._plan.steps:
            step = self._plan.steps[min(self._step_index, len(self._plan.steps) - 1)]
            return self._established[side] and side in step.holders
        times = self._sequence_times
        if side == _LEFT:
            # The stable airborne dual hold establishes the right hand as a
            # holder. Left contact may then end during the opening ramp, before
            # full opening and low speed establish the separate release proof.
            # Right hold loss remains guarded throughout that transition.
            return times["left"] is not None and times["dual"] is None
        if times["dual"] is None:
            return False
        contained, _message = _containment(self.layout, observation)
        bottom = observation.support_upward_force(self.binding.box_bottom_geom_names)
        placed = (
            contained
            and bottom >= self.config.success.min_box_support_normal_force_n
        )
        return not placed

    # ---------------------------------------------------------------- success
    def _update_success(
        self, observation: HandoverObservation, conditions: tuple[TaskCondition, ...]
    ) -> None:
        if self._failure or self._state != "RUNNING":
            return
        at_final_step = (
            self._plan is None
            or not self._plan.steps
            or self._step_index >= len(self._plan.steps) - 1
        )
        all_passed = (
            at_final_step
            and self._sequence_phase == "complete"
            and all(condition.passed for condition in conditions)
        )
        if not all_passed:
            self._success_since_ns = None
            self._success_position_reference = None
            self._success_rotation_reference = None
            return
        if self._success_since_ns is None:
            self._success_since_ns = observation.sim_time_ns
            self._first_satisfied_ns = observation.sim_time_ns
            self._success_position_reference = observation.block_center
            from robot_backend_sim.tasks.handover.observation import quaternion_matrix

            self._success_rotation_reference = quaternion_matrix(
                observation.block_quaternion_wxyz
            )
        if observation.sim_time_ns - self._success_since_ns >= round(
            self.config.success.stable_duration_s * 1e9
        ):
            self._completed_ns = observation.sim_time_ns
            self._success_observation = observation
            self._success_conditions = conditions
            self._state, self._reason = "SUCCEEDED", "SUCCESS"
            self._behavior_phase = "stable"

    def _derived_phase(self, observation: HandoverObservation) -> str:
        """Physical phase used when no teacher sequence is running."""
        minimum = self.config.planning.diagnostics.contact_force_min_n
        left = observation.holding(_LEFT, minimum)
        right = observation.holding(_RIGHT, minimum)
        if not observation.left_contacts and not observation.right_contacts:
            contained, _message = _containment(self.layout, observation)
            if contained:
                return "stable"
            if not left and not right:
                return "released"
        if right and not left:
            return "right_hold"
        if right and left:
            return "dual_hold"
        if left:
            return "left_hold"
        return "approach"

    # -------------------------------------------------------------- terminals
    def reset(self, reason: str = "RESET") -> TaskState:
        self._task_epoch += 1
        self._state, self._reason = "IDLE", reason
        self._context = None
        self._plan = None
        self._plan_digest, self._plan_duration_ns = "", 0
        self._start_sim_ns = self._start_wall_ns = 0
        self._observation = None
        self._conditions = ()
        self._success_observation = None
        self._success_conditions = ()
        self._prepare_cache.clear()
        self._start_fact = None
        self._reset_phase_state()
        self._seq += 1
        return self._snapshot()

    def fail_control(self, reason: str) -> TaskState:
        if self._state == "SUCCEEDED":
            return self._snapshot()
        if self._state == "RUNNING":
            self._fail("CONTROL_FAILED", reason)
        return self._snapshot()

    def _fail(self, reason: str, detail: str) -> None:
        self._state = "FAILED"
        self._failure = f"{reason}: {detail}" if detail else reason
        self._reason = self._failure
        self._success_since_ns = None
        self._step_status = "FAILED"

    # --------------------------------------------------------------- snapshot
    def _snapshot(self) -> TaskState:
        observation = self._observation
        conditions = self._conditions
        if self._state == "SUCCEEDED" and self._success_observation is not None:
            observation = self._success_observation
            conditions = self._success_conditions
        stable_elapsed = 0.0
        if self._state == "RUNNING" and self._success_since_ns is not None and observation:
            stable_elapsed = max(
                0.0, (observation.sim_time_ns - self._success_since_ns) / 1e9
            )
        elif self._state == "SUCCEEDED":
            stable_elapsed = self.config.success.stable_duration_s
        step = (
            None
            if self._plan is None or not self._plan.steps
            else self._plan.steps[min(self._step_index, len(self._plan.steps) - 1)]
        )
        contained, containment_message = (
            (False, "no observation")
            if observation is None
            else _containment(self.layout, observation)
        )
        vertices = [] if observation is None else _vertices(self.layout, observation)
        sequence = self._sequence_fact()
        metrics: dict[str, Any] = {
            "reason": self._reason,
            "failure": self._failure,
            "behavior_phase": self._behavior_phase,
            "step": "" if step is None else step.name,
            "step_index": self._step_index,
            "step_count": 0 if self._plan is None else len(self._plan.steps),
            "gate_state": self._step_status,
            "phase_enter_sim_time_ns": self._step_active_ns,
            "phase_wait_s": self._step_wait_s,
            "gate_stable_elapsed_s": (
                0.0
                if self._step_stable_since_ns is None or observation is None
                else max(
                    0.0,
                    (observation.sim_time_ns - self._step_stable_since_ns) / 1e9,
                )
            ),
            "task_start_sim_time_ns": self._start_sim_ns,
            "first_satisfied_sim_time_ns": self._first_satisfied_ns,
            "completed_sim_time_ns": self._completed_ns,
            "stable_elapsed_s": stable_elapsed,
            "block_name": self.binding.block_name,
            "block": None
            if observation is None
            else {
                "center": list(observation.block_center),
                "quaternion_wxyz": list(observation.block_quaternion_wxyz),
                "linear_velocity": list(observation.block_linear_velocity),
                "angular_velocity": list(observation.block_angular_velocity),
            },
            "block_vertices_world": [list(vertex) for vertex in vertices],
            "block_height_above_initial_m": (
                0.0 if observation is None else observation.block_height_above_initial_m
            ),
            "left_finger_contacts": 0 if observation is None else len(observation.left_contacts),
            "right_finger_contacts": 0 if observation is None else len(observation.right_contacts),
            "left_min_force_n": _min_force(observation, _LEFT),
            "right_min_force_n": _min_force(observation, _RIGHT),
            "left_finger_speed_m_s": (
                0.0 if observation is None else observation.left_finger_speed_m_s
            ),
            "right_finger_speed_m_s": (
                0.0 if observation is None else observation.right_finger_speed_m_s
            ),
            "left_opening_m": 0.0 if observation is None else observation.left_opening_m,
            "right_opening_m": 0.0 if observation is None else observation.right_opening_m,
            "left_holding": False
            if observation is None
            else observation.holding(_LEFT, self.config.planning.diagnostics.contact_force_min_n),
            "right_holding": False
            if observation is None
            else observation.holding(_RIGHT, self.config.planning.diagnostics.contact_force_min_n),
            "right_catch_sim_time_ns": self._right_catch_ns,
            "left_release_sim_time_ns": self._left_release_ns,
            "left_retreat_done": self._left_retreat_done,
            "table_support_force_n": 0.0
            if observation is None
            else observation.support_force(self.binding.support_body_names[0]),
            "box_bottom_support_force_n": 0.0
            if observation is None
            else observation.support_upward_force(
                self.binding.box_bottom_geom_names
            ),
            "external_support_force_n": 0.0
            if observation is None
            else observation.external_support_force(self._allowed_support()),
            "handover_window": self._handover_window,
            "handover_window_violation": self._window_violation,
            "handover_sequence": {
                "phase": sequence.phase,
                "left_airborne_stable": sequence.left_airborne_stable,
                "dual_airborne_stable": sequence.dual_airborne_stable,
                "left_release_observed": sequence.left_release_observed,
                "right_airborne_stable": sequence.right_airborne_stable,
                "complete": sequence.complete,
                "left_stable_sim_time_ns": sequence.left_stable_sim_time_ns,
                "dual_stable_sim_time_ns": sequence.dual_stable_sim_time_ns,
                "left_release_sim_time_ns": sequence.left_release_sim_time_ns,
                "right_stable_sim_time_ns": sequence.right_stable_sim_time_ns,
                "left_hold_elapsed_s": sequence.left_hold_elapsed_s,
                "dual_hold_elapsed_s": sequence.dual_hold_elapsed_s,
                "right_hold_elapsed_s": sequence.right_hold_elapsed_s,
                "left_min_force_n": sequence.left_min_force_n,
                "right_min_force_n": sequence.right_min_force_n,
            },
            "robot_robot_force_n": 0.0 if observation is None else observation.robot_robot_force_n,
            "robot_box_force_n": 0.0 if observation is None else observation.robot_box_force_n,
            "block_contained": contained,
            "containment_message": containment_message,
            "left_released": False if observation is None else not observation.left_contacts,
            "right_released": False if observation is None else not observation.right_contacts,
            "linear_speed_m_s": 0.0
            if observation is None
            else _norm(observation.block_linear_velocity),
            "angular_speed_rad_s": 0.0
            if observation is None
            else _norm(observation.block_angular_velocity),
            "conditions_passed": bool(conditions) and all(c.passed for c in conditions),
            "conditions": [
                {
                    "name": item.name,
                    "passed": item.passed,
                    "actual": item.actual,
                    "threshold": item.threshold,
                    "unit": item.unit,
                    "detail": item.detail,
                }
                for item in conditions
            ],
            "phase_events": [
                {
                    "phase": event.phase,
                    "step": event.step,
                    "sim_time_ns": event.sim_time_ns,
                    "wait_s": event.wait_s,
                }
                for event in self._events
            ],
        }
        return TaskState(
            context=self._context,
            state_seq=self._seq,
            sample_time_ns=0 if observation is None else observation.sim_time_ns,
            task_epoch=self._task_epoch,
            task_type="handover",
            phase=self._state,
            succeeded=self._state == "SUCCEEDED",
            terminal=self._state in _TERMINAL_STATES,
            failure_reason=self._failure if self._state == "FAILED" else "",
            plan_digest=self._plan_digest,
            task_config_digest=self.config.config_digest,
            metrics=metrics,
        )


def _min_force(observation: HandoverObservation | None, side: str) -> float:
    if observation is None:
        return 0.0
    contacts = observation.finger_contacts(side)
    return min((contact.normal_force_n for contact in contacts), default=0.0)
