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
from dataclasses import dataclass
from typing import Any

from robot_contracts.models import ContractError, RobotProfile

from robot_backend_sim.marvin_sim.handover.scene_config import MarvinSceneConfig
from robot_backend_sim.common.config.motion_config import ControlConfig
from robot_backend_sim.common.tasks.base import TaskChunk, TeacherPrepareSnapshot
from robot_backend_sim.common.tasks.math import duration_from_limits
from robot_backend_sim.common.tasks.plugin_base import sample_step_chunk, teacher_joint_limits
from robot_backend_sim.marvin_sim.handover.gate import HandoverFeedback, HandoverGateFact
from robot_backend_sim.marvin_sim.handover.plan import (
    HandoverLayout,
    layout_from_config,
    perturb_handover_plan_config,
    plan_handover,
)
from robot_backend_sim.marvin_sim.handover.task_config import BEHAVIOUR_PHASES, HandoverTaskConfig
from robot_backend_sim.marvin_sim.robot_config import public_dual_arm_positions

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


HandoverSnapshot = TeacherPrepareSnapshot


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
    final: bool = False
    waypoints: tuple[tuple[float, ...], ...] = ()
    segment_durations_s: tuple[float, ...] = ()
    segment_phases: tuple[str, ...] = ()


@dataclass(frozen=True, slots=True)
class HandoverPlan:
    context: Any
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

    @property
    def duration_s(self) -> float:
        return self.duration_ns / 1e9

    def step_at(self, index: int) -> HandoverStep:
        return self.steps[min(max(index, 0), len(self.steps) - 1)]


def _plan_digest(
    config: HandoverTaskConfig,
    snapshot: HandoverSnapshot,
    steps: tuple[HandoverStep, ...],
    layout: HandoverLayout,
) -> str:
    payload = {
        "task_config_digest": config.config_digest,
        "seed": snapshot.seed,
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
                "final": step.final,
                "waypoints": [list(point) for point in step.waypoints],
                "segment_durations_s": list(step.segment_durations_s),
                "segment_phases": list(step.segment_phases),
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
    final: bool = False,
    waypoints: tuple[tuple[float, ...], ...] = (),
    segment_durations_s: tuple[float, ...] = (),
    segment_phases: tuple[str, ...] = (),
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
        final=final,
        waypoints=tuple(tuple(float(value) for value in point) for point in waypoints),
        segment_durations_s=tuple(float(value) for value in segment_durations_s),
        segment_phases=segment_phases,
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
    attempts = planning.diversity.max_attempts if planning.diversity.enabled else 1
    source_plan = None
    speed_scale = 1.0
    for attempt in range(attempts):
        if time.monotonic() > budget_deadline:
            break
        candidate_planning, speed_scale = perturb_handover_plan_config(
            planning, int(snapshot.seed), attempt
        )
        source_plan = plan_handover(
            frozen,  # type: ignore[arg-type]
            candidate_planning,
            max_velocities=velocities,
            deadline=budget_deadline,
            teacher_only=True,
        )
        if source_plan.passed:
            break
    if source_plan is None:
        raise _error("TEACHER_PLAN_TIMEOUT", "planning exceeded its wall budget")
    if not source_plan.passed:
        raise _error(
            "TEACHER_PLAN_INVALID",
            "; ".join(source_plan.failures)
            or "the P02B candidate chain no longer passes",
        )
    if time.monotonic() > budget_deadline:
        raise _error("TEACHER_PLAN_TIMEOUT", "planning exceeded its wall budget")
    targets = {target.name: target for target in source_plan.targets}
    if any(not target.converged for target in targets.values()):
        raise _error("TEACHER_IK_FAILED", "the source plan contains an unconverged target")

    open_total = float(scene_config.robot.gripper_open_total)
    left_close = float(teacher.left_close_opening_m)
    right_close = float(teacher.right_close_opening_m)
    velocity_limits, acceleration_limits = teacher_joint_limits(profile, control_config, teacher)
    scaled_velocities = list(velocity_limits)
    for index in (*range(7), *range(8, 15)):
        scaled_velocities[index] = min(
            float(velocity_limits[index]) * speed_scale,
            float(profile.joints[index].max_velocity) * 0.95,
        )
    velocity_limits = tuple(scaled_velocities)
    minimum_s = float(teacher.phase_min_duration_s)

    start = public_dual_arm_positions(
        snapshot.qpos, robot.arm_joint_qpos["left"], robot.arm_joint_qpos["right"], open_total, open_total
    )
    steps: list[HandoverStep] = []
    waypoint_source = source_plan.teacher_waypoints

    def public_qpos(
        qpos: tuple[float, ...], left_open: float, right_open: float
    ) -> tuple[float, ...]:
        return public_dual_arm_positions(
            qpos,
            robot.arm_joint_qpos["left"],
            robot.arm_joint_qpos["right"],
            left_open,
            right_open,
        )

    def add(
        name: str,
        phase: str,
        end: tuple[float, ...],
        duration: float,
        gate: str,
        holders: tuple[str, ...],
        *,
        final: bool = False,
    ) -> None:
        nonlocal start
        if duration <= 0.0:
            duration = duration_from_limits(
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
                final=final,
            )
        )
        start = end

    def add_waypointed(
        name: str,
        phase: str,
        points: tuple[tuple[float, ...], ...],
        phases: tuple[str, ...],
        gate: str,
        holders: tuple[str, ...],
    ) -> None:
        nonlocal start
        durations = tuple(
            duration_from_limits(
                first,
                last,
                velocity_limits,
                acceleration_limits,
                minimum_s=minimum_s,
            )
            for first, last in zip(points, points[1:], strict=False)
        )
        steps.append(
            _step(
                len(steps),
                name,
                phase,
                points[0],
                points[-1],
                sum(durations),
                gate,
                holders,
                waypoints=points,
                segment_durations_s=durations,
                segment_phases=phases,
            )
        )
        start = points[-1]

    add_waypointed(
        "left_approach",
        "left_approach",
        (
            start,
            public_qpos(waypoint_source["ready_wait70"], open_total, open_total),
            public_qpos(waypoint_source["pregrasp_wait100"], open_total, open_total),
            public_qpos(waypoint_source["grasp_wait100"], open_total, open_total),
        ),
        ("left_approach", "left_approach", "left_approach"),
        "position",
        (),
    )
    add(
        "left_close",
        "left_grasp",
        public_qpos(waypoint_source["grasp_wait100"], left_close, open_total),
        0.0,
        "left_grasp",
        ("left",),
    )
    add(
        "left_lift",
        "left_lift",
        public_qpos(waypoint_source["handover_wait100"], left_close, open_total),
        0.0,
        "left_lift",
        ("left",),
    )
    add_waypointed(
        "right_handover_grasp",
        "handover_approach",
        (
            start,
            public_qpos(waypoint_source["right_pregrasp"], left_close, open_total),
            public_qpos(waypoint_source["right_grasp"], left_close, open_total),
        ),
        ("handover_approach", "handover_approach"),
        "handover",
        ("left",),
    )
    add(
        "right_close",
        "dual_hold",
        public_qpos(waypoint_source["right_grasp"], left_close, right_close),
        0.0,
        "dual_hold",
        ("left", "right"),
    )
    add(
        "left_open",
        "left_release",
        public_qpos(waypoint_source["right_grasp"], open_total, right_close),
        0.0,
        "left_release",
        ("right",),
    )
    add_waypointed(
        "right_place",
        "right_place",
        (
            start,
            public_qpos(waypoint_source["retreat35"], open_total, right_close),
            public_qpos(waypoint_source["retreat100_preplace"], open_total, right_close),
            public_qpos(waypoint_source["retreat100_place"], open_total, right_close),
        ),
        ("left_retreat", "right_transport", "right_place"),
        "right_place",
        ("right",),
    )
    add(
        "right_open",
        "right_release",
        public_qpos(waypoint_source["retreat100_place"], open_total, open_total),
        0.0,
        "released",
        (),
    )
    add(
        "stable_hold",
        "stable",
        public_qpos(waypoint_source["retreat100_place"], open_total, open_total),
        float(task_config.success.stable_duration_s),
        "stable",
        (),
        final=True,
    )

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
    )


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
    """Sample the currently gated step, including its multi-segment waypoints."""
    step = plan.step_at(gate.step_index)
    if not step.segment_durations_s:
        return sample_step_chunk(
            plan,
            steps=plan.steps,
            behaviour_phases=BEHAVIOUR_PHASES,
            gate=gate,
            feedback=feedback,
            request_id=request_id,
            task_epoch=task_epoch,
            capture_seq=capture_seq,
            observation_time_ns=observation_time_ns,
            chunk_size=chunk_size,
            period_ns=period_ns,
            profile=profile,
            control_config=control_config,
            start_sim_time_ns=int(start_sim_time_ns),
            error_cls=TeacherError,
        )

    from robot_contracts.models import Target
    from robot_trajectory.spline_math import validate_reference_inputs

    from robot_backend_sim.common.tasks.math import scalar_acceleration, trapezoid_progress

    if chunk_size != 60 or period_ns != 50_000_000:
        raise _error(
            "INVALID_TEACHER_CLOCK", "teacher chunks are exactly 60 points at 50 ms"
        )
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

    duration_ns = round(step.duration_s * 1e9)
    points = step.waypoints
    durations = step.segment_durations_s
    segment_phases = step.segment_phases
    cumulative: list[float] = []
    total_duration_s = 0.0
    for duration in durations:
        cumulative.append(total_duration_s)
        total_duration_s += duration
    accelerations = tuple(
        scalar_acceleration(points[index], points[index + 1], plan.accelerations)
        for index in range(len(points) - 1)
    )

    positions: list[tuple[float, ...]] = []
    point_phases: list[str] = []
    times: list[int] = []
    for index in range(chunk_size):
        target_time_ns = observation_time_ns + (index + 1) * period_ns
        elapsed_s = (target_time_ns - start_sim_time_ns) / 1e9
        if elapsed_s >= step.duration_s:
            position = step.end_positions
            point_phase = segment_phases[-1]
        elif elapsed_s <= 0.0:
            position = step.start_positions
            point_phase = segment_phases[0]
        else:
            segment_index = len(durations) - 1
            for candidate, segment_start in enumerate(cumulative):
                if segment_start <= elapsed_s < segment_start + durations[candidate]:
                    segment_index = candidate
                    break
            local_s = elapsed_s - cumulative[segment_index]
            progress = trapezoid_progress(
                local_s, durations[segment_index], accelerations[segment_index]
            )
            first = points[segment_index]
            last = points[segment_index + 1]
            position = tuple(
                start + (end - start) * progress
                for start, end in zip(first, last, strict=True)
            )
            point_phase = segment_phases[segment_index]
        positions.append(position)
        point_phases.append(point_phase)
        times.append((index + 1) * period_ns)

    if not all(math.isfinite(value) for pos in positions for value in pos):
        raise _error("INVALID_TEACHER_TARGET", "teacher produced a non-finite target")

    targets = tuple(
        Target(time_ns, pos) for time_ns, pos in zip(times, positions, strict=True)
    )
    validate_reference_inputs(
        positions[0], targets, profile.joints, control_config.max_accelerations
    )

    digest_payload = {
        "plan_digest": plan.plan_digest,
        "step": step.name,
        "capture_seq": capture_seq,
        "observation_time_ns": observation_time_ns,
        "positions": positions,
        "phase": point_phases[0],
        "gate": gate.status,
    }
    chunk_digest = hashlib.sha256(
        json.dumps(
            digest_payload, sort_keys=True, separators=(",", ":"), allow_nan=False
        ).encode()
    ).hexdigest()
    wait_s = max(0.0, (observation_time_ns - start_sim_time_ns - duration_ns) / 1e9)
    return TaskChunk(
        request_id=request_id,
        task_epoch=task_epoch,
        capture_seq=capture_seq,
        observation_time_ns=observation_time_ns,
        source_time_ns=feedback.sim_time_ns,
        robot_profile_hash=plan.context.robot_profile_hash,
        plan_digest=plan.plan_digest,
        chunk_digest=chunk_digest,
        phase=point_phases[0],
        phases=tuple(point_phases),
        phase_codes=tuple(BEHAVIOUR_PHASES.index(phase) for phase in point_phases),
        time_from_observation_ns=tuple(times),
        positions=tuple(positions),
        ik_max_position_error_m=plan.ik_max_position_error_m,
        ik_max_orientation_error_rad=plan.ik_max_orientation_error_rad,
        gate_state=gate.status,
        gate_failure=gate.failure,
        phase_start_sim_time_ns=start_sim_time_ns,
        phase_wait_s=wait_s,
    )
