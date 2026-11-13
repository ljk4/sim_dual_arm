"""Program-teacher planner and 60-point raw target sampler.

The teacher reads the object truth frozen in a prepared task snapshot, solves
private mink key poses, checks collisions and samples a continuous smoothstep
trajectory.  It never calls ``apply_targets``/``mj_step``, never touches live
``Scene.data`` and never submits a command; the caller must still pass every
target through the accepted ``SubmitFollower``/five-point window path.
"""

from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass
from typing import Any

from robot_contracts.models import ContractError, RobotProfile, RunContext, Target
from robot_trajectory.spline_math import validate_reference_inputs

from robot_backend_sim.config.kinematics_config import KinematicsCheckConfig
from robot_backend_sim.config.motion_config import ControlConfig
from robot_backend_sim.kinematics.kinematics import (
    CollisionPairSet,
    KeyPoseTarget,
    ObjectPose,
    PathResult,
    build_collision_pairs,
    build_targets,
    check_path,
    measure_clearance,
    solve_key_poses,
)
from robot_backend_sim.tasks.base import TaskChunk
from robot_backend_sim.tasks.stack.config import PHASES, StackTaskConfig
from robot_backend_sim.tasks.stack.gate import TeacherFeedback, TeacherGateFact, TeacherProgress

__all__ = [
    "TeacherChunk", "TeacherError", "TeacherFeedback", "TeacherGateFact",
    "TeacherPlan", "TeacherProgress", "TeacherSegment", "TeacherSnapshot",
    "build_teacher_plan", "sample_teacher_chunk",
]


class TeacherError(ContractError):
    """A bounded teacher failure suitable for a service response reason."""


def _error(reason: str, message: str) -> TeacherError:
    return TeacherError(reason, message)


@dataclass(frozen=True, slots=True)
class TeacherSnapshot:
    """Immutable truth captured at prepare; never a live MjData reference."""

    context: RunContext
    task_epoch: int
    sim_time_ns: int
    qpos: tuple[float, ...]
    source: ObjectPose
    target: ObjectPose
    source_half_extents_m: tuple[float, float, float]
    target_half_extents_m: tuple[float, float, float]
    left_positions: tuple[float, ...]
    right_positions: tuple[float, ...]
    left_gripper_opening_m: float
    right_gripper_opening_m: float
    table_top_z_m: float


@dataclass(frozen=True, slots=True)
class TeacherSegment:
    phase: str
    start_s: float
    end_s: float
    start_positions: tuple[float, ...]
    end_positions: tuple[float, ...]

    @property
    def duration_s(self) -> float:
        return self.end_s - self.start_s

    @property
    def kind(self) -> str:
        if self.start_positions[:8] == self.end_positions[:8]:
            if self.start_positions[8:15] == self.end_positions[8:15]:
                if self.start_positions[15] == self.end_positions[15]:
                    return "hold"
                return "gripper"
        return "joint"


@dataclass(frozen=True, slots=True)
class TeacherPlan:
    context: RunContext
    task_epoch: int
    config_digest: str
    plan_digest: str
    segments: tuple[TeacherSegment, ...]
    duration_ns: int
    ik_results: tuple[Any, ...]
    collision_results: tuple[PathResult, ...]
    key_poses: tuple[KeyPoseTarget, ...]
    allowed_contact_pairs: tuple[str, ...]
    source_center: tuple[float, float, float]

    @property
    def duration_s(self) -> float:
        return self.duration_ns / 1e9

    @property
    def start_positions(self) -> tuple[float, ...]:
        return self.segments[0].start_positions

    def segment_at(self, time_s: float) -> TeacherSegment:
        for segment in self.segments:
            if segment.start_s <= time_s < segment.end_s:
                return segment
        return self.segments[-1]

    def sample_positions(self, time_s: float) -> tuple[float, ...]:
        clamped = max(0.0, min(time_s, self.duration_s))
        segment = self.segment_at(clamped)
        duration = segment.duration_s
        if duration <= 0.0:
            return segment.end_positions
        u = max(0.0, min(1.0, (clamped - segment.start_s) / duration))
        smooth = u * u * (3.0 - 2.0 * u)
        return tuple(
            start + (end - start) * smooth
            for start, end in zip(segment.start_positions, segment.end_positions, strict=True)
        )

    def sample_velocities(self, time_s: float) -> tuple[float, ...]:
        clamped = max(0.0, min(time_s, self.duration_s))
        segment = self.segment_at(clamped)
        duration = segment.duration_s
        if duration <= 0.0:
            return (0.0,) * len(segment.end_positions)
        u = max(0.0, min(1.0, (clamped - segment.start_s) / duration))
        derivative = 6.0 * u * (1.0 - u) / duration
        return tuple(
            (end - start) * derivative
            for start, end in zip(segment.start_positions, segment.end_positions, strict=True)
        )


TeacherChunk = TaskChunk




def _filtered_pairs(model: Any) -> tuple[CollisionPairSet, tuple[str, ...]]:
    """Allow finger-source contact; keep arm/table/column/target collisions."""
    base = build_collision_pairs(model)
    kept = []
    allowed = []
    for pair in base.pairs:
        names = {pair.body1 or "", pair.body2 or ""}
        finger = any("finger" in name for name in names)
        if finger and "source" in names:
            allowed.append(f"{pair.body1}/{pair.body2} ({pair.geom1_name}/{pair.geom2_name})")
            continue
        kept.append(pair)
    if not kept:
        raise _error("COLLISION_PAIRS_EMPTY", "no teacher collision pairs remain")
    return (
        CollisionPairSet(
            pairs=tuple(kept),
            categories=dict(base.categories),
            exclusions=dict(base.exclusions),
            total_physical_pairs=base.total_physical_pairs,
            explicit_pair_count=base.explicit_pair_count,
        ),
        tuple(sorted(allowed)),
    )


def _joint_limits(
    profile: RobotProfile, control: ControlConfig, teacher: Any
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    velocities = [
        float(joint.max_velocity) * teacher.max_joint_velocity_fraction for joint in profile.joints
    ]
    accelerations = [
        float(value) * teacher.max_joint_acceleration_fraction
        for value in control.max_accelerations
    ]
    # The gripper's configured full-opening speed is a hard 0.04 m/s and its
    # ramp uses the separate 0.5 m/s^2 budget, not 85% of a generic joint.
    for index in (7, 15):
        velocities[index] = teacher.gripper_velocity_m_s
        accelerations[index] = teacher.gripper_acceleration_m_s2
    return tuple(velocities), tuple(accelerations)


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
        # Cubic smoothstep: peak speed 1.5*delta/T, peak acceleration 6*delta/T^2.
        by_velocity = 1.5 * delta / velocities[index]
        by_acceleration = math.sqrt(6.0 * delta / accelerations[index])
        duration = max(duration, by_velocity, by_acceleration)
    return duration


def _smooth_segment(
    phase: str,
    start_s: float,
    start: tuple[float, ...],
    end: tuple[float, ...],
    velocities: tuple[float, ...],
    accelerations: tuple[float, ...],
    *,
    minimum_s: float,
) -> TeacherSegment:
    if len(start) != 16 or len(end) != 16:
        raise _error("INVALID_TEACHER_PLAN", "teacher targets must have 16 values")
    duration = _duration(start, end, velocities, accelerations, minimum_s=minimum_s)
    return TeacherSegment(phase, start_s, start_s + duration, start, end)


def _plan_digest(
    config: StackTaskConfig,
    snapshot: TeacherSnapshot,
    segments: tuple[TeacherSegment, ...],
    snapshot_allowed_contacts: tuple[str, ...],
) -> str:
    payload = {
        "version": 1,
        "config_digest": config.config_digest,
        "context": {
            "robot_id": snapshot.context.robot_id,
            "backend_instance_id": snapshot.context.backend_instance_id,
            "clock_epoch": snapshot.context.clock_epoch,
            "episode_id": snapshot.context.episode_id,
        },
        "source": [snapshot.source.center, snapshot.source.yaw_rad],
        "target": [snapshot.target.center, snapshot.target.yaw_rad],
        "allowed_contact_pairs": list(snapshot_allowed_contacts),
        "segments": [
            {
                "phase": segment.phase,
                "start_s": segment.start_s,
                "end_s": segment.end_s,
                "start": segment.start_positions,
                "end": segment.end_positions,
            }
            for segment in segments
        ],
    }
    text = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(text.encode()).hexdigest()


def _gripper_pose_clearance(
    model: Any,
    initial_qpos: tuple[float, ...],
    right_positions: tuple[float, ...],
    right_opening: float,
    pair_set: CollisionPairSet,
    clearance: Any,
    *,
    segment: str,
) -> PathResult:
    """Check a closed/open finger pose without writing the active scene."""
    import mink
    import numpy as np

    configuration = mink.Configuration(model)
    qpos = np.asarray(initial_qpos, dtype=np.float64).copy()
    right_addresses = [int(model.joint(f"right_joint{index}").qposadr[0]) for index in range(1, 8)]
    for address, value in zip(right_addresses, right_positions, strict=True):
        qpos[address] = value
    for index in (1, 2):
        address = int(model.joint(f"right_finger_joint{index}").qposadr[0])
        qpos[address] = right_opening / 2.0
    # The left fingers keep their captured initial opening; only the right
    # gripper command is changed by the close/open phases.
    for index in (1, 2):
        address = int(model.joint(f"left_finger_joint{index}").qposadr[0])
        qpos[address] = initial_qpos[address]
    configuration.update(qpos)
    started = time.monotonic()
    point = measure_clearance(
        model,
        configuration.data,
        pair_set,
        detection_distance_m=clearance.collision_detection_distance_m,
        sample_index=0,
        fraction=0.0,
    )
    passed = point.distance_m >= clearance.minimum_clearance_m
    return PathResult(
        segment=segment,
        sample_count=1,
        requested_max_step_rad=0.0,
        actual_max_step_rad=0.0,
        minimum=point,
        passed=passed,
        elapsed_s=time.monotonic() - started,
        failure_reason=None if passed else "clearance_below_minimum",
        failure_message=(
            None
            if passed
            else (
                f"{segment}: clearance {point.distance_m:.6f} m < "
                f"{clearance.minimum_clearance_m:.6f} m at "
                f"{point.geom1_name}/{point.geom2_name}"
            )
        ),
    )


def _teacher_ik(
    model,
    snapshot: TeacherSnapshot,
    control_config: ControlConfig,
    kinematics_config: KinematicsCheckConfig,
    teacher,
    *,
    started: float,
    ik_deadline: float,
    budget_deadline: float,
):
    try:
        targets = build_targets(
            snapshot.source,
            snapshot.target,
            table_top_z=snapshot.table_top_z_m,
            source_half_z=snapshot.source_half_extents_m[2],
            target_half_z=snapshot.target_half_extents_m[2],
            config=kinematics_config.targets,
        )
    except Exception as error:  # kinematics errors carry bounded reasons
        raise _error("TEACHER_TARGETS_INVALID", f"{type(error).__name__}: {error}") from error
    pair_set, allowed_contact_pairs = _filtered_pairs(model)
    try:
        results = solve_key_poses(
            model,
            snapshot.qpos,
            targets,
            kinematics_config.ik,
            kinematics_config.clearance,
            pair_set,
        )
    except Exception as error:
        raise _error("TEACHER_IK_FAILED", f"{type(error).__name__}: {error}") from error
    if len(results) != len(targets):
        raise _error("TEACHER_IK_FAILED", "IK did not return all six key poses")
    failed = [result for result in results if not result.converged]
    if failed:
        first = failed[0]
        raise _error(
            "TEACHER_IK_FAILED",
            f"{first.phase}: {first.reason}: {first.message}",
        )
    if time.monotonic() > min(budget_deadline, ik_deadline):
        raise _error("TEACHER_IK_TIMEOUT", "IK preparation exceeded its wall budget")
    del control_config, teacher, started
    return targets, results, pair_set, allowed_contact_pairs


def _teacher_segments(
    snapshot: TeacherSnapshot,
    teacher,
    results,
    profile: RobotProfile,
    control_config: ControlConfig,
) -> tuple[list[TeacherSegment], float]:
    right_home = snapshot.right_positions
    left = snapshot.left_positions
    left_gripper = snapshot.left_gripper_opening_m
    right_gripper = snapshot.right_gripper_opening_m
    release = teacher.release_gripper_opening_m
    close_target = 0.0

    def full(right: tuple[float, ...], opening: float) -> tuple[float, ...]:
        return (*left, left_gripper, *right, opening)

    velocities, accelerations = _joint_limits(profile, control_config, teacher)
    segments: list[TeacherSegment] = []
    start_s = 0.0
    current = full(right_home, right_gripper)

    def add(
        phase: str,
        next_right: tuple[float, ...],
        next_opening: float,
        minimum_s: float,
    ) -> None:
        nonlocal start_s, current
        end = full(next_right, next_opening)
        segment = _smooth_segment(
            phase,
            start_s,
            current,
            end,
            velocities,
            accelerations,
            minimum_s=minimum_s,
        )
        if segment.end_s > teacher.max_plan_duration_s + 1e-12:
            raise _error(
                "TEACHER_PLAN_TOO_LONG",
                f"phase {phase!r} ends at {segment.end_s:.3f} s, over "
                f"{teacher.max_plan_duration_s:g} s",
            )
        segments.append(segment)
        start_s = segment.end_s
        current = end

    phase_min = teacher.phase_min_duration_s
    add("pregrasp", tuple(results[0].qpos_right), right_gripper, phase_min["pregrasp"])
    add("grasp", tuple(results[1].qpos_right), right_gripper, phase_min["grasp"])
    add("close", tuple(results[1].qpos_right), close_target, phase_min["close"])
    add("lift", tuple(results[2].qpos_right), close_target, phase_min["lift"])
    add("transport", tuple(results[3].qpos_right), close_target, phase_min["transport"])
    add("lower", tuple(results[4].qpos_right), close_target, phase_min["lower"])
    add("open", tuple(results[4].qpos_right), release, phase_min["open"])
    add("retreat", tuple(results[5].qpos_right), release, phase_min["retreat"])
    add("stable", tuple(results[5].qpos_right), release, teacher.stable_hold_s)
    return segments, segments[-1].end_s


def _validate_plan_length(segments, task_config, teacher) -> None:
    total = segments[-1].end_s
    if total > teacher.max_plan_duration_s + 1e-12:
        raise _error(
            "TEACHER_PLAN_TOO_LONG",
            f"plan {total:.3f} s exceeds {teacher.max_plan_duration_s:g} s; "
            f"last phase {segments[-1].phase}",
        )
    if total > task_config.success.max_task_sim_duration_s - 1e-9:
        raise _error("TEACHER_PLAN_TOO_LONG", "plan leaves no task-budget margin")


def _teacher_collisions(
    model,
    snapshot: TeacherSnapshot,
    segments,
    results,
    pair_set,
    clearance,
    *,
    budget_deadline: float,
    close_target: float,
    release: float,
) -> list[PathResult]:
    collision_results: list[PathResult] = []
    for segment in segments:
        if segment.kind != "joint":
            continue
        if time.monotonic() > budget_deadline:
            raise _error("TEACHER_PLAN_TIMEOUT", "collision planning exceeded its wall budget")
        path = check_path(
            model,
            snapshot.qpos,
            segment.start_positions[8:15],
            segment.end_positions[8:15],
            pair_set,
            clearance,
            segment=f"teacher:{segment.phase}",
            deadline=budget_deadline,
        )
        collision_results.append(path)
        if not path.passed:
            raise _error(
                "TEACHER_COLLISION",
                f"{segment.phase}: {path.failure_reason}: {path.failure_message}",
            )
    # The close command may be 0, but the physical fingers stop on the 4 cm
    # source.  Collision-check that expected opening, not interpenetrating
    # zero-width finger geometry; the raw target stays 0 as required.
    closed_check_opening = max(close_target, 2.0 * snapshot.source_half_extents_m[0])
    for segment, right_positions, opening in (
        ("teacher:close_pose", tuple(results[1].qpos_right), closed_check_opening),
        ("teacher:open_pose", tuple(results[4].qpos_right), release),
    ):
        path = _gripper_pose_clearance(
            model,
            snapshot.qpos,
            right_positions,
            opening,
            pair_set,
            clearance,
            segment=segment,
        )
        collision_results.append(path)
        if not path.passed:
            raise _error("TEACHER_COLLISION", f"{segment}: {path.failure_message}")
    return collision_results


def build_teacher_plan(
    model: Any,
    snapshot: TeacherSnapshot,
    profile: RobotProfile,
    control_config: ControlConfig,
    kinematics_config: KinematicsCheckConfig,
    task_config: StackTaskConfig,
    *,
    deadline: float | None = None,
) -> TeacherPlan:
    """Solve six private key poses and build the bounded timed plan.

    The caller supplies an immutable snapshot and a read-only model.  The
    private mink ``Configuration`` owns its own ``MjData``; the live scene is
    never updated or stepped.
    """
    started = time.monotonic()
    teacher = task_config.teacher
    budget_deadline = started + teacher.plan_wall_timeout_s if deadline is None else deadline
    ik_deadline = started + teacher.ik_wall_timeout_s
    if not teacher.enabled:
        raise _error("TEACHER_DISABLED", "teacher planning is disabled by task configuration")
    targets, results, pair_set, allowed_contact_pairs = _teacher_ik(
        model,
        snapshot,
        control_config,
        kinematics_config,
        teacher,
        started=started,
        ik_deadline=ik_deadline,
        budget_deadline=budget_deadline,
    )
    segments, total = _teacher_segments(snapshot, teacher, results, profile, control_config)
    _validate_plan_length(segments, task_config, teacher)
    collision_results = _teacher_collisions(
        model,
        snapshot,
        segments,
        results,
        pair_set,
        kinematics_config.clearance,
        budget_deadline=budget_deadline,
        close_target=0.0,
        release=teacher.release_gripper_opening_m,
    )
    plan = _plan_digest(task_config, snapshot, tuple(segments), allowed_contact_pairs)
    return TeacherPlan(
        context=snapshot.context,
        task_epoch=snapshot.task_epoch,
        config_digest=task_config.config_digest,
        plan_digest=plan,
        segments=tuple(segments),
        duration_ns=round(total * 1e9),
        ik_results=tuple(results),
        collision_results=tuple(collision_results),
        key_poses=targets,
        allowed_contact_pairs=allowed_contact_pairs,
        source_center=snapshot.source.center,
    )


def _teacher_gate(plan, gate, feedback, capture_seq, observation_time_ns, chunk_size, period_ns):
    if chunk_size != 60 or period_ns != 50_000_000:
        raise _error("INVALID_TEACHER_CLOCK", "teacher chunks are exactly 60 points at 50 ms")
    if feedback.capture_seq != capture_seq or feedback.sim_time_ns != observation_time_ns:
        raise _error(
            "TEACHER_FEEDBACK_MISMATCH",
            "feedback does not belong to the retained capture/observation",
        )
    if gate.failure:
        raise _error("TEACHER_GATE_FAILED", gate.failure)
    if gate.status == "FAILED":
        raise _error("TEACHER_GATE_FAILED", gate.failure)
    if gate.phase_index >= len(plan.segments):
        raise _error("TEACHER_PLAN_COMPLETE", "all gated phases have passed")
    segment = plan.segments[gate.phase_index]
    actual_start = gate.actual_start_sim_time_ns
    if actual_start < 0:
        raise _error("TEACHER_NOT_STARTED", "no measured phase start exists")
    return segment, actual_start


def _teacher_positions(segment, actual_start, observation_time_ns, chunk_size, period_ns):
    positions = []
    phases = []
    times = []
    for index in range(chunk_size):
        target_time_ns = observation_time_ns + (index + 1) * period_ns
        elapsed = target_time_ns - actual_start
        if (
            segment.phase == "stable"
            or segment.duration_s <= 0
            or elapsed >= segment.duration_s * 1e9
        ):
            position = segment.end_positions
        else:
            u = max(0.0, min(1.0, elapsed / (segment.duration_s * 1e9)))
            smooth = u * u * (3.0 - 2.0 * u)
            position = tuple(
                left + (right - left) * smooth
                for left, right in zip(
                    segment.start_positions, segment.end_positions, strict=True
                )
            )
        positions.append(position)
        phases.append(segment.phase)
        times.append((index + 1) * period_ns)
    return positions, phases, times


def _teacher_chunk_document(
    plan, gate, capture_seq, observation_time_ns, positions, segment
) -> tuple[dict, str]:
    payload = {
        "plan_digest": plan.plan_digest,
        "capture_seq": capture_seq,
        "observation_time_ns": observation_time_ns,
        "positions": positions,
        "phase": segment.phase,
        "gate": gate.status,
    }
    text = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return payload, hashlib.sha256(text.encode()).hexdigest()


def sample_teacher_chunk(
    plan: TeacherPlan,
    *,
    gate: TeacherGateFact,
    feedback: TeacherFeedback,
    request_id: str,
    task_epoch: int,
    capture_seq: int,
    observation_time_ns: int,
    chunk_size: int,
    period_ns: int,
    profile: RobotProfile,
    control_config: ControlConfig,
) -> TeacherChunk:
    """Sample only the currently gated phase and hold its end pose.

    Points never cross an unpassed gate.  Gate evidence is the immutable
    per-capture feedback, not a wall clock or the nominal plan time.
    """
    segment, actual_start = _teacher_gate(
        plan, gate, feedback, capture_seq, observation_time_ns, chunk_size, period_ns
    )
    positions, phases, times = _teacher_positions(
        segment, actual_start, observation_time_ns, chunk_size, period_ns
    )
    first = positions[0]
    if not all(math.isfinite(value) for position in positions for value in position):
        raise _error("INVALID_TEACHER_TARGET", "teacher produced a non-finite target")
    targets = tuple(
        Target(time_ns, position) for time_ns, position in zip(times, positions, strict=True)
    )
    validate_reference_inputs(
        first,
        targets,
        profile.joints,
        control_config.max_accelerations,
    )
    for position in positions:
        if position[:8] != plan.start_positions[:8]:
            raise _error("LEFT_ARM_MOVED", "teacher may not move the left arm")
    _payload, digest = _teacher_chunk_document(
        plan, gate, capture_seq, observation_time_ns, positions, segment
    )
    return TaskChunk(
        request_id=request_id,
        task_epoch=task_epoch,
        capture_seq=capture_seq,
        observation_time_ns=observation_time_ns,
        source_time_ns=feedback.sim_time_ns,
        robot_profile_hash=plan.context.robot_profile_hash,
        plan_digest=plan.plan_digest,
        chunk_digest=digest,
        phase=segment.phase,
        phases=tuple(phases),
        phase_codes=tuple(PHASES.index(phase) for phase in phases),
        time_from_observation_ns=tuple(times),
        positions=tuple(positions),
        ik_max_position_error_m=max(result.position_error_m for result in plan.ik_results),
        ik_max_orientation_error_rad=max(
            result.orientation_error_rad for result in plan.ik_results
        ),
        gate_state=gate.status,
        gate_failure=gate.failure,
        phase_start_sim_time_ns=gate.actual_start_sim_time_ns,
        phase_wait_s=gate.phase_wait_s,
    )
