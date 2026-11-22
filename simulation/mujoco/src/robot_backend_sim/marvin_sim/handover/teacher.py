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
    source_plan: Any = field(default=None, repr=False)

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
    velocity_limits, acceleration_limits = teacher_joint_limits(profile, control_config, teacher)
    minimum_s = float(teacher.phase_min_duration_s)

    start = public_dual_arm_positions(
        snapshot.qpos, robot.arm_joint_qpos["left"], robot.arm_joint_qpos["right"], open_total, open_total
    )
    steps: list[HandoverStep] = []

    def public(name: str, left_open: float, right_open: float) -> tuple[float, ...]:
        return public_dual_arm_positions(
            targets[name].qpos,
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
        allow_box_support: bool = False,
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
    """Sample only the currently gated step and hold its end pose."""
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
