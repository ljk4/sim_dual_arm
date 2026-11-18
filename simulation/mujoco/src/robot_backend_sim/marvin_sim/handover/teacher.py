"""Handover program-teacher plan and 60-point raw target sampler.

The planner reuses the P02B target chain and measured segment durations, adds
the discrete close/open steps, and exposes one step at a time.  A chunk can
only describe the step that the task gate had already reached for the requested
capture; it never samples past an unpassed gate.  Planning reads a frozen
snapshot, never the live ``MjData``, and writes no scene state.
"""

from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass, field
from typing import Any

from robot_contracts.models import ContractError, RobotProfile, RunContext, Target
from robot_trajectory.spline_math import validate_reference_inputs

from robot_backend_sim.marvin_sim.handover.scene_config import MarvinSceneConfig
from robot_backend_sim.common.config.motion_config import ControlConfig
from robot_backend_sim.common.tasks.base import TaskChunk
from robot_backend_sim.marvin_sim.handover.gate import HandoverFeedback, HandoverGateFact
from robot_backend_sim.marvin_sim.handover.plan import (
    HandoverLayout,
    layout_from_config,
    plan_handover,
)
from robot_backend_sim.marvin_sim.handover.task_config import BEHAVIOUR_PHASES, HandoverTaskConfig

__all__ = [
    "FrozenHandoverScene",
    "HandoverPlan",
    "HandoverSnapshot",
    "HandoverStep",
    "TeacherError",
    "build_handover_plan",
    "sample_handover_chunk",
]


class TeacherError(ContractError):
    """A bounded teacher failure suitable for a service response reason."""


def _error(reason: str, message: str) -> TeacherError:
    return TeacherError(reason, message)


@dataclass(frozen=True, slots=True)
class HandoverSnapshot:
    """Immutable truth captured at prepare; never a live MjData reference."""

    context: RunContext
    task_epoch: int
    sim_time_ns: int
    capture_seq: int
    qpos: tuple[float, ...]
    seed: int
    controller_command_id: str


@dataclass(frozen=True, slots=True)
class _FrozenData:
    qpos: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class FrozenHandoverScene:
    """Read-only scene view used by the planner thread."""

    model: Any
    config: MarvinSceneConfig
    robot_geometry: Any
    object_free_joint_qpos: dict[str, int]
    support_body_names: tuple[str, ...]
    data: _FrozenData


@dataclass(frozen=True, slots=True)
class HandoverStep:
    index: int
    name: str
    phase: str
    start_positions: tuple[float, ...]
    end_positions: tuple[float, ...]
    duration_s: float
    gate: str
    holders: tuple[str, ...]
    allow_box_support: bool = False
    final: bool = False

    @property
    def kind(self) -> str:
        if self.start_positions[:7] != self.end_positions[:7]:
            return "left"
        if self.start_positions[8:15] != self.end_positions[8:15]:
            return "right"
        if self.start_positions != self.end_positions:
            return "gripper"
        return "hold"


@dataclass(frozen=True, slots=True)
class HandoverPlan:
    context: RunContext
    task_epoch: int
    config_digest: str
    plan_digest: str
    steps: tuple[HandoverStep, ...]
    duration_ns: int
    open_total_m: float
    velocities: tuple[float, ...]
    accelerations: tuple[float, ...]
    ik_max_position_error_m: float
    ik_max_orientation_error_rad: float
    layout: HandoverLayout
    source_plan: Any = field(default=None, repr=False)

    @property
    def duration_s(self) -> float:
        return self.duration_ns / 1e9

    def step_at(self, index: int) -> HandoverStep:
        return self.steps[min(max(index, 0), len(self.steps) - 1)]


def _duration(
    start: tuple[float, ...],
    end: tuple[float, ...],
    velocities: tuple[float, ...],
    accelerations: tuple[float, ...],
    *,
    minimum_s: float,
) -> float:
    duration = float(minimum_s)
    for index, (first, second) in enumerate(zip(start, end, strict=True)):
        delta = abs(second - first)
        if delta <= 0.0:
            continue
        by_velocity = 1.5 * delta / velocities[index]
        by_acceleration = math.sqrt(6.0 * delta / accelerations[index])
        duration = max(duration, by_velocity, by_acceleration)
    return duration


def _joint_limits(
    profile: RobotProfile, control: ControlConfig, teacher: Any
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    velocities = [
        float(joint.max_velocity) * teacher.max_joint_velocity_fraction
        for joint in profile.joints
    ]
    accelerations = [
        float(value) * teacher.max_joint_acceleration_fraction
        for value in control.max_accelerations
    ]
    for index in (7, 15):
        velocities[index] = teacher.gripper_velocity_m_s
        accelerations[index] = teacher.gripper_acceleration_m_s2
    return tuple(velocities), tuple(accelerations)


def _public_positions(
    qpos: tuple[float, ...],
    robot: Any,
    left_opening: float,
    right_opening: float,
) -> tuple[float, ...]:
    left = tuple(float(qpos[address]) for address in robot.arm_joint_qpos["left"])
    right = tuple(float(qpos[address]) for address in robot.arm_joint_qpos["right"])
    return (*left, float(left_opening), *right, float(right_opening))


def _plan_digest(
    config: HandoverTaskConfig,
    snapshot: HandoverSnapshot,
    steps: tuple[HandoverStep, ...],
    layout: HandoverLayout,
) -> str:
    payload = {
        "version": 1,
        "task_config_digest": config.config_digest,
        "context": {
            "robot_id": snapshot.context.robot_id,
            "backend_instance_id": snapshot.context.backend_instance_id,
            "clock_epoch": snapshot.context.clock_epoch,
            "episode_id": snapshot.context.episode_id,
        },
        "layout": {
            "block_center": layout.block_initial_center,
            "block_yaw_rad": layout.block_initial_yaw_rad,
            "box_center": layout.box_center,
            "box_yaw_rad": layout.box_yaw_rad,
        },
        "steps": [
            {
                "name": step.name,
                "phase": step.phase,
                "start_s": step.start_positions,
                "end_s": step.end_positions,
                "duration_s": step.duration_s,
                "gate": step.gate,
                "holders": list(step.holders),
                "allow_box_support": step.allow_box_support,
                "final": step.final,
            }
            for step in steps
        ],
    }
    text = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(text.encode()).hexdigest()


def _step(
    index: int,
    name: str,
    phase: str,
    start: tuple[float, ...],
    end: tuple[float, ...],
    duration: float,
    gate: str,
    holders: tuple[str, ...],
    *,
    allow_box_support: bool = False,
    final: bool = False,
) -> HandoverStep:
    return HandoverStep(
        index=index,
        name=name,
        phase=phase,
        start_positions=tuple(float(value) for value in start),
        end_positions=tuple(float(value) for value in end),
        duration_s=float(duration),
        gate=gate,
        holders=holders,
        allow_box_support=allow_box_support,
        final=final,
    )


def build_handover_plan(
    model: Any,
    snapshot: HandoverSnapshot,
    profile: RobotProfile,
    control_config: ControlConfig,
    task_config: HandoverTaskConfig,
    *,
    scene_config: MarvinSceneConfig,
    robot: Any,
    robot_geometry: Any,
    object_free_joint_qpos: dict[str, int],
    support_body_names: tuple[str, ...],
    deadline: float | None = None,
) -> HandoverPlan:
    """Build the gated close/open sequence from the P02B candidate chain."""
    started = time.monotonic()
    teacher = task_config.teacher
    budget_deadline = (
        started + teacher.plan_wall_timeout_s if deadline is None else deadline
    )
    planning = task_config.planning
    frozen = FrozenHandoverScene(
        model=model,
        config=scene_config,
        robot_geometry=robot_geometry,
        object_free_joint_qpos=object_free_joint_qpos,
        support_body_names=support_body_names,
        data=_FrozenData(tuple(float(value) for value in snapshot.qpos)),
    )
    layout = layout_from_config(
        scene_config,
        snapshot.qpos,
        int(object_free_joint_qpos[scene_config.task.block.name]),
    )
    by_name = {joint.name: joint for joint in profile.joints}
    velocities = {
        side: tuple(
            float(by_name[f"{side}_joint{index}"].max_velocity) for index in range(1, 8)
        )
        for side in ("left", "right")
    }
    source_plan = plan_handover(
        frozen,  # type: ignore[arg-type]
        planning,
        max_velocities=velocities,
        deadline=budget_deadline,
    )
    if not source_plan.passed:
        raise _error(
            "TEACHER_PLAN_INVALID",
            "; ".join(source_plan.failures) or "the P02B candidate chain no longer passes",
        )
    if time.monotonic() > budget_deadline:
        raise _error("TEACHER_PLAN_TIMEOUT", "planning exceeded its wall budget")
    targets = {target.name: target for target in source_plan.targets}
    if any(not target.converged for target in targets.values()):
        raise _error("TEACHER_IK_FAILED", "the source plan contains an unconverged target")

    open_total = float(scene_config.robot.gripper_open_total)
    left_close = float(teacher.left_close_opening_m)
    right_close = float(teacher.right_close_opening_m)
    velocity_limits, acceleration_limits = _joint_limits(profile, control_config, teacher)
    minimum_s = float(teacher.phase_min_duration_s)

    start = _public_positions(snapshot.qpos, robot, open_total, open_total)
    steps: list[HandoverStep] = []

    def public(name: str, left_open: float, right_open: float) -> tuple[float, ...]:
        return _public_positions(targets[name].qpos, robot, left_open, right_open)

    def add(
        name: str,
        phase: str,
        end: tuple[float, ...],
        duration: float,
        gate: str,
        holders: tuple[str, ...],
        *,
        allow_box_support: bool = False,
        final: bool = False,
    ) -> None:
        nonlocal start
        if duration <= 0.0:
            duration = _duration(
                start,
                end,
                velocity_limits,
                acceleration_limits,
                minimum_s=minimum_s,
            )
        steps.append(
            _step(
                len(steps),
                name,
                phase,
                start,
                end,
                duration,
                gate,
                holders,
                allow_box_support=allow_box_support,
                final=final,
            )
        )
        start = end

    add("left_ready", "left_approach", public("left_ready", open_total, open_total),
        0.0, "position", ())
    add("left_pregrasp", "left_approach",
        public("left_pregrasp", open_total, open_total),
        0.0, "position", ())
    add("left_grasp_pose", "left_approach",
        public("left_grasp", open_total, open_total),
        0.0, "position", ())
    add("left_close", "left_grasp", public("left_grasp", left_close, open_total),
        0.0, "left_grasp", ("left",))
    add("left_lift", "left_lift", public("left_lift", left_close, open_total),
        0.0, "left_lift", ("left",))
    add("left_handover", "handover_approach",
        public("left_handover", left_close, open_total),
        0.0, "handover", ("left",))
    add("right_handover_wait", "handover_approach",
        public("right_handover_wait", left_close, open_total),
        0.0, "handover", ("left",))
    add("right_handover_pregrasp", "handover_approach",
        public("right_handover_pregrasp", left_close, open_total),
        0.0, "handover", ("left",))
    add("right_handover_grasp", "handover_approach",
        public("right_handover_grasp", left_close, open_total),
        0.0, "handover", ("left",))
    add("right_close", "dual_hold",
        public("right_handover_grasp", left_close, right_close),
        0.0, "dual_hold", ("left", "right"))
    add("left_open", "left_release",
        public("right_handover_grasp", open_total, right_close),
        0.0, "left_release", ("right",))
    add("left_retreat", "left_retreat",
        public("left_retreat", open_total, right_close),
        0.0, "left_retreat", ("right",))
    add("right_preplace", "right_transport",
        public("right_preplace", open_total, right_close),
        0.0, "right_carry", ("right",))
    add("right_place", "right_place",
        public("right_place", open_total, right_close),
        0.0, "right_place", ("right",),
        allow_box_support=True)
    add("right_open", "right_release",
        public("right_place", open_total, open_total),
        0.0, "released", (), allow_box_support=True)
    add("stable_hold", "stable",
        public("right_place", open_total, open_total),
        float(task_config.success.stable_duration_s), "stable", (),
        allow_box_support=True, final=True)

    total_s = sum(step.duration_s for step in steps)
    if total_s > teacher.max_plan_duration_s + 1e-9:
        raise _error(
            "TEACHER_PLAN_TOO_LONG",
            f"plan {total_s:.3f} s exceeds {teacher.max_plan_duration_s:g} s",
        )
    if total_s > task_config.success.max_task_sim_duration_s - 1e-9:
        raise _error("TEACHER_PLAN_TOO_LONG", "plan leaves no task-budget margin")
    ik_position = max(float(target.position_error_m) for target in targets.values())
    ik_orientation = max(
        float(target.orientation_error_rad) for target in targets.values()
    )
    if not all(math.isfinite(value) for step in steps for value in step.end_positions):
        raise _error("INVALID_TEACHER_TARGET", "teacher produced a non-finite target")
    digest = _plan_digest(task_config, snapshot, tuple(steps), layout)
    return HandoverPlan(
        context=snapshot.context,
        task_epoch=snapshot.task_epoch,
        config_digest=task_config.config_digest,
        plan_digest=digest,
        steps=tuple(steps),
        duration_ns=round(total_s * 1e9),
        open_total_m=open_total,
        velocities=velocity_limits,
        accelerations=acceleration_limits,
        ik_max_position_error_m=ik_position,
        ik_max_orientation_error_rad=ik_orientation,
        layout=layout,
        source_plan=source_plan,
    )


def _teacher_gate(
    plan: HandoverPlan,
    gate: HandoverGateFact,
    feedback: HandoverFeedback,
    capture_seq: int,
    observation_time_ns: int,
    chunk_size: int,
    period_ns: int,
) -> HandoverStep:
    if chunk_size != 60 or period_ns != 50_000_000:
        raise _error("INVALID_TEACHER_CLOCK", "teacher chunks are exactly 60 points at 50 ms")
    if feedback.capture_seq != capture_seq or feedback.sim_time_ns != observation_time_ns:
        raise _error(
            "TEACHER_FEEDBACK_MISMATCH",
            "feedback does not belong to the retained capture/observation",
        )
    if gate.failure or gate.status == "FAILED":
        raise _error("TEACHER_GATE_FAILED", gate.failure or "task gate failed")
    if gate.step_index >= len(plan.steps):
        raise _error("TEACHER_PLAN_COMPLETE", "all gated steps have passed")
    if gate.active_start_sim_time_ns < 0:
        raise _error("TEACHER_NOT_STARTED", "no measured step start exists")
    return plan.step_at(gate.step_index)


def _trapezoid_progress(
    elapsed_s: float, duration_s: float, acceleration: float
) -> float:
    """Acceleration-limited scalar progress shared by all joints.

    The bounded follower plans an acceleration-limited trajectory (infinite
    jerk).  Sampling the same shape keeps the selected window inside its
    reference-deviation budget; a smoothstep is slower than the follower at the
    start of a long move and was measured to exceed that budget.
    """
    if duration_s <= 0.0 or elapsed_s <= 0.0:
        return 0.0
    if elapsed_s >= duration_s:
        return 1.0
    if acceleration <= 0.0:
        # A hold step has identical start and end positions.
        return 1.0
    product = acceleration * duration_s * duration_s
    if product < 4.0:
        raise _error(
            "TEACHER_PLAN_INVALID",
            f"step duration {duration_s:.3f} s is below the acceleration-limited minimum",
        )
    discriminant = acceleration * acceleration * duration_s * duration_s - 4.0 * acceleration
    cruise = (
        acceleration * duration_s - math.sqrt(max(0.0, discriminant))
    ) / 2.0
    if cruise <= 0.0:
        return min(1.0, elapsed_s / duration_s)
    accelerate_s = cruise / acceleration
    accelerate_distance = 0.5 * acceleration * accelerate_s * accelerate_s
    cruise_s = max(0.0, (1.0 - 2.0 * accelerate_distance) / cruise)
    if elapsed_s < accelerate_s:
        return 0.5 * acceleration * elapsed_s * elapsed_s
    if elapsed_s < accelerate_s + cruise_s:
        return accelerate_distance + cruise * (elapsed_s - accelerate_s)
    return 1.0 - 0.5 * acceleration * (duration_s - elapsed_s) ** 2


def _scalar_acceleration(
    step: HandoverStep, accelerations: tuple[float, ...]
) -> float:
    """Scalar acceleration for this step's joint-linear path."""
    acceleration = math.inf
    for first, last, limit in zip(
        step.start_positions, step.end_positions, accelerations, strict=True
    ):
        delta = abs(last - first)
        if delta <= 0.0:
            continue
        acceleration = min(acceleration, limit / delta)
    if not math.isfinite(acceleration) or acceleration <= 0.0:
        return 0.0
    return acceleration


def _teacher_positions(
    step: HandoverStep,
    start_ns: int,
    observation_time_ns: int,
    chunk_size: int,
    period_ns: int,
    accelerations: tuple[float, ...],
) -> tuple[tuple[tuple[float, ...], ...], tuple[str, ...], tuple[int, ...]]:
    positions: list[tuple[float, ...]] = []
    phases: list[str] = []
    times: list[int] = []
    duration_ns = round(step.duration_s * 1e9)
    acceleration = _scalar_acceleration(step, accelerations)
    for index in range(chunk_size):
        target_time_ns = observation_time_ns + (index + 1) * period_ns
        elapsed = target_time_ns - start_ns
        if step.final or duration_ns <= 0 or elapsed >= duration_ns:
            position = step.end_positions
        else:
            progress = _trapezoid_progress(
                max(0.0, elapsed / 1e9), step.duration_s, acceleration
            )
            position = tuple(
                first + (last - first) * progress
                for first, last in zip(
                    step.start_positions, step.end_positions, strict=True
                )
            )
        positions.append(position)
        phases.append(step.phase)
        times.append((index + 1) * period_ns)
    return tuple(positions), tuple(phases), tuple(times)


def _chunk_digest(
    plan: HandoverPlan,
    step: HandoverStep,
    capture_seq: int,
    observation_time_ns: int,
    positions: tuple[tuple[float, ...], ...],
    gate: HandoverGateFact,
) -> str:
    payload = {
        "plan_digest": plan.plan_digest,
        "step": step.name,
        "capture_seq": capture_seq,
        "observation_time_ns": observation_time_ns,
        "positions": positions,
        "phase": step.phase,
        "gate": gate.status,
    }
    text = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(text.encode()).hexdigest()


def sample_handover_chunk(
    plan: HandoverPlan,
    *,
    gate: HandoverGateFact,
    feedback: HandoverFeedback,
    request_id: str,
    task_epoch: int,
    capture_seq: int,
    observation_time_ns: int,
    chunk_size: int,
    period_ns: int,
    profile: RobotProfile,
    control_config: ControlConfig,
    start_sim_time_ns: int,
) -> TaskChunk:
    """Sample only the currently gated step and hold its end pose.

    ``start_sim_time_ns`` is the scheduled hand-off at which the first chunk of
    this step begins moving.  Aligning the target clock with the hand-off keeps
    the reference continuous with the state the follower inherits.
    """
    step = _teacher_gate(
        plan, gate, feedback, capture_seq, observation_time_ns, chunk_size, period_ns
    )
    positions, phases, times = _teacher_positions(
        step,
        start_sim_time_ns,
        observation_time_ns,
        chunk_size,
        period_ns,
        plan.accelerations,
    )
    if not all(math.isfinite(value) for position in positions for value in position):
        raise _error("INVALID_TEACHER_TARGET", "teacher produced a non-finite target")
    targets = tuple(
        Target(time_ns, position) for time_ns, position in zip(times, positions, strict=True)
    )
    validate_reference_inputs(
        positions[0],
        targets,
        profile.joints,
        control_config.max_accelerations,
    )
    digest = _chunk_digest(plan, step, capture_seq, observation_time_ns, positions, gate)
    duration_ns = round(step.duration_s * 1e9)
    wait_s = max(0.0, (observation_time_ns - start_sim_time_ns - duration_ns) / 1e9)
    return TaskChunk(
        request_id=request_id,
        task_epoch=task_epoch,
        capture_seq=capture_seq,
        observation_time_ns=observation_time_ns,
        source_time_ns=feedback.sim_time_ns,
        robot_profile_hash=plan.context.robot_profile_hash,
        plan_digest=plan.plan_digest,
        chunk_digest=digest,
        phase=step.phase,
        phases=phases,
        phase_codes=tuple(BEHAVIOUR_PHASES.index(phase) for phase in phases),
        time_from_observation_ns=times,
        positions=positions,
        ik_max_position_error_m=plan.ik_max_position_error_m,
        ik_max_orientation_error_rad=plan.ik_max_orientation_error_rad,
        gate_state=gate.status,
        gate_failure=gate.failure,
        phase_start_sim_time_ns=start_sim_time_ns,
        phase_wait_s=wait_s,
    )
