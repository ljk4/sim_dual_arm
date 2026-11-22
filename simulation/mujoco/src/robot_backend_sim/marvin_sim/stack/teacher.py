"""Stack program-teacher plan and 60-point raw target sampler for Marvin."""

from __future__ import annotations

import hashlib
import json
import time
from dataclasses import dataclass
from typing import Any

from robot_contracts.models import ContractError, RobotProfile

from robot_backend_sim.common.config.motion_config import ControlConfig
from robot_backend_sim.common.kinematics.common import extract_object_pose
from robot_backend_sim.common.kinematics.kinematics_clearance import CollisionPairSet
from robot_backend_sim.common.tasks.base import TaskChunk, TeacherPrepareSnapshot
from robot_backend_sim.common.tasks.math import (
    duration_from_limits,
    quaternion_wxyz_to_matrix,
)
from robot_backend_sim.common.tasks.plugin_base import sample_step_chunk, teacher_joint_limits
from robot_backend_sim.marvin_sim.kinematics import (
    BlockEnvelope,
    CarriedBlock,
    relative_block_pose,
    tcp_matrix,
)
from robot_backend_sim.marvin_sim.robot_config import public_dual_arm_positions
from robot_backend_sim.marvin_sim.scene_base import marvin_robot_geometry
from robot_backend_sim.marvin_sim.stack.config import BEHAVIOUR_PHASES, StackTaskConfig
from robot_backend_sim.marvin_sim.stack.gate import StackGateFact, TeacherFeedback
from robot_backend_sim.marvin_sim.stack.kinematics_config import KinematicsCheckConfig
from robot_backend_sim.marvin_sim.stack.plan import (
    IKResult,
    PathResult,
    build_targets,
    check_gripper_pose_clearance,
    check_path,
    solve_key_poses,
    stack_object_pairs,
    stack_pair_set,
)
from robot_backend_sim.marvin_sim.stack.scene_config import MarvinStackSceneConfig

__all__ = [
    "StackPlan",
    "StackPlanSegment",
    "TeacherError",
    "TeacherSnapshot",
    "build_teacher_plan",
    "sample_teacher_chunk",
]


class TeacherError(ContractError):
    """Bounded stack teacher failure."""


def _error(reason: str, message: str) -> TeacherError:
    return TeacherError(reason, message)


TeacherSnapshot = TeacherPrepareSnapshot


@dataclass(frozen=True, slots=True)
class StackPlanSegment:
    index: int
    name: str
    phase: str
    start_positions: tuple[float, ...]
    end_positions: tuple[float, ...]
    duration_s: float
    gate: str
    final: bool = False


@dataclass(frozen=True, slots=True)
class StackPlan:
    context: Any
    task_epoch: int
    config_digest: str
    plan_digest: str
    segments: tuple[StackPlanSegment, ...]
    duration_ns: int
    velocities: tuple[float, ...]
    accelerations: tuple[float, ...]
    ik_results: tuple[IKResult, ...]
    collision_results: tuple[PathResult, ...]
    allowed_contact_pairs: tuple[str, ...]
    ik_max_position_error_m: float
    ik_max_orientation_error_rad: float
    source_initial_center: tuple[float, float, float]
    target_center: tuple[float, float, float]
    expected_placed_source_z_m: float
    start_positions: tuple[float, ...]

    @property
    def duration_s(self) -> float:
        return self.duration_ns / 1e9

    def step_at(self, index: int) -> StackPlanSegment:
        return self.segments[min(max(index, 0), len(self.segments) - 1)]


def _drop_finger_source_pairs(
    base_pairs: CollisionPairSet, finger_bodies: frozenset[str]
) -> CollisionPairSet:
    pairs = tuple(
        pair
        for pair in base_pairs.pairs
        if not (
            pair.category == "robot_object"
            and (
                (pair.body1 in finger_bodies and pair.body2 == "source")
                or (pair.body2 in finger_bodies and pair.body1 == "source")
            )
        )
    )
    categories: dict[str, int] = {}
    for pair in pairs:
        categories[pair.category] = categories.get(pair.category, 0) + 1
    return CollisionPairSet(
        pairs=pairs,
        categories=categories,
        exclusions=dict(base_pairs.exclusions),
        total_physical_pairs=base_pairs.total_physical_pairs,
        explicit_pair_count=base_pairs.explicit_pair_count,
    )


def _plan_digest(
    config: StackTaskConfig,
    snapshot: TeacherSnapshot,
    segments: tuple[StackPlanSegment, ...],
    source_center: tuple[float, float, float],
    target_center: tuple[float, float, float],
) -> str:
    payload = {
        "task_config_digest": config.config_digest,
        "context": {
            "robot_id": snapshot.context.robot_id,
            "backend_instance_id": snapshot.context.backend_instance_id,
            "clock_epoch": snapshot.context.clock_epoch,
            "episode_id": snapshot.context.episode_id,
        },
        "source_center": source_center,
        "target_center": target_center,
        "segments": [
            {
                "name": s.name,
                "phase": s.phase,
                "start_s": s.start_positions,
                "end_s": s.end_positions,
                "duration_s": s.duration_s,
                "gate": s.gate,
                "final": s.final,
            }
            for s in segments
        ],
    }
    text = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(text.encode()).hexdigest()


def build_teacher_plan(
    model: Any,
    snapshot: TeacherSnapshot,
    profile: RobotProfile,
    control_config: ControlConfig,
    kinematics_config: KinematicsCheckConfig,
    task_config: StackTaskConfig,
    *,
    scene_config: MarvinStackSceneConfig,
    deadline: float | None = None,
) -> StackPlan:
    started = time.monotonic()
    teacher = task_config.teacher
    budget_deadline = (
        started + teacher.plan_wall_timeout_s if deadline is None else deadline
    )

    geometry = marvin_robot_geometry()
    left_chain = next(c for c in geometry.chains if c.side == "left")
    right_chain = next(c for c in geometry.chains if c.side == "right")
    left_arm_qpos = tuple(
        int(model.jnt_qposadr[int(model.joint(name).id)]) for name in left_chain.joint_names
    )
    right_arm_qpos = tuple(
        int(model.jnt_qposadr[int(model.joint(name).id)]) for name in right_chain.joint_names
    )

    source_addr = int(model.jnt_qposadr[int(model.joint("source_free").id)])
    target_addr = int(model.jnt_qposadr[int(model.joint("target_free").id)])
    source_pose = extract_object_pose(snapshot.qpos, source_addr, "source")
    target_pose = extract_object_pose(snapshot.qpos, target_addr, "target")

    table_top_z = float(scene_config.robot.base_position[2] + scene_config.task.table.top_z)
    source_half_z = float(task_config.source_half_extents_m[2])
    target_half_z = float(task_config.target_half_extents_m[2])

    targets = build_targets(
        source_pose,
        target_pose,
        table_top_z=table_top_z,
        source_half_z=source_half_z,
        target_half_z=target_half_z,
        config=kinematics_config.targets,
    )
    all_pairs = stack_pair_set(model)
    ik_results = solve_key_poses(
        model,
        snapshot.qpos,
        targets,
        kinematics_config.ik,
        kinematics_config.clearance,
        None,
        deadline=budget_deadline,
    )
    if len(ik_results) != len(targets) or not all(r.converged for r in ik_results):
        bad = next((r for r in ik_results if not r.converged), None)
        msg = bad.message if bad is not None and bad.message else "IK failed"
        raise _error("TEACHER_IK_FAILED", f"stack IK failed: {msg}")
    if time.monotonic() > budget_deadline:
        raise _error("TEACHER_PLAN_TIMEOUT", "planning exceeded wall timeout")

    solved = {r.phase: r for r in ik_results}
    right_finger_names = tuple(body for body, _ in geometry.finger_bodies["right"])
    holding_pairs = _drop_finger_source_pairs(all_pairs, frozenset(right_finger_names))
    object_pairs = stack_object_pairs(model, excluded_body_names=right_finger_names)
    allowed_contact_pairs = tuple(f"{body}:source" for body in right_finger_names)

    # Carry envelope fixed in Arm_R_tcp at solved grasp configuration.
    grasp_mat = tcp_matrix(model, solved["grasp"].qpos, right_chain.tcp_site)
    grasp_pos = (float(grasp_mat[0, 3]), float(grasp_mat[1, 3]), float(grasp_mat[2, 3]))
    grasp_rot = tuple(
        tuple(float(v) for v in row) for row in grasp_mat[:3, :3]
    )
    source_quat = tuple(
        float(value) for value in snapshot.qpos[source_addr + 3 : source_addr + 7]
    )
    source_rot = quaternion_wxyz_to_matrix(source_quat)
    rel_pose = relative_block_pose(
        grasp_pos, grasp_rot, source_pose.center, source_rot
    )
    envelope = BlockEnvelope(
        object_name="source",
        body_id=int(model.body("source").id),
        geom_id=int(model.geom("source_geom").id),
        qpos_address=source_addr,
        holder_tcp_site=right_chain.tcp_site,
        relative_pose=rel_pose,
    )
    open_total = float(scene_config.robot.gripper_open_total)
    cube_width_m = 2.0 * float(task_config.source_half_extents_m[0])
    per_finger_closed = max(0.0, (open_total - cube_width_m) / 2.0)
    closed_qpos = list(snapshot.qpos)
    for _body_name, joint_name in geometry.finger_bodies["right"]:
        addr = int(model.jnt_qposadr[int(model.joint(joint_name).id)])
        closed_qpos[addr] = per_finger_closed
    carried = CarriedBlock(envelope, solved["grasp"].qpos, True)

    initial_right = tuple(float(snapshot.qpos[a]) for a in right_arm_qpos)
    by_name = {joint.name: joint for joint in profile.joints}
    right_vel_limits = tuple(
        float(by_name[f"right_joint{i}"].max_velocity) for i in range(1, 8)
    )

    clearance_checks: tuple[PathResult, ...] = (
        check_path(
            model,
            snapshot.qpos,
            initial_right,
            solved["pregrasp"].qpos_right,
            all_pairs,
            kinematics_config.clearance,
            segment="pregrasp",
            max_velocities=right_vel_limits,
            deadline=budget_deadline,
        ),
        check_path(
            model,
            snapshot.qpos,
            solved["pregrasp"].qpos_right,
            solved["grasp"].qpos_right,
            all_pairs,
            kinematics_config.clearance,
            segment="grasp",
            max_velocities=right_vel_limits,
            deadline=budget_deadline,
        ),
        check_gripper_pose_clearance(
            model,
            solved["grasp"].qpos,
            opening_m=teacher.close_gripper_opening_m,
            gripper_open_total=open_total,
            pair_set=holding_pairs,
            clearance=kinematics_config.clearance,
            segment="close",
        ),
        check_path(
            model,
            closed_qpos,
            solved["grasp"].qpos_right,
            solved["lift"].qpos_right,
            holding_pairs,
            kinematics_config.clearance,
            segment="lift",
            object_pairs=object_pairs,
            max_velocities=right_vel_limits,
            carried=carried,
            allowed_contacts_at_start=("table",),
            deadline=budget_deadline,
        ),
        check_path(
            model,
            closed_qpos,
            solved["lift"].qpos_right,
            solved["place_hover"].qpos_right,
            holding_pairs,
            kinematics_config.clearance,
            segment="transport",
            object_pairs=object_pairs,
            max_velocities=right_vel_limits,
            carried=carried,
            deadline=budget_deadline,
        ),
        check_path(
            model,
            closed_qpos,
            solved["place_hover"].qpos_right,
            solved["place"].qpos_right,
            holding_pairs,
            kinematics_config.clearance,
            segment="lower",
            object_pairs=object_pairs,
            max_velocities=right_vel_limits,
            carried=carried,
            allowed_contacts_at_end=("target",),
            deadline=budget_deadline,
        ),
        check_gripper_pose_clearance(
            model,
            solved["place"].qpos,
            opening_m=open_total,
            gripper_open_total=open_total,
            pair_set=all_pairs,
            clearance=kinematics_config.clearance,
            segment="open",
        ),
        check_path(
            model,
            snapshot.qpos,
            solved["place"].qpos_right,
            solved["retreat"].qpos_right,
            all_pairs,
            kinematics_config.clearance,
            segment="retreat",
            max_velocities=right_vel_limits,
            deadline=budget_deadline,
        ),
    )
    for result in clearance_checks:
        if not result.passed:
            raise _error(
                "TEACHER_PLAN_INVALID",
                result.failure_message or f"clearance failed at {result.segment}",
            )

    velocity_limits, acceleration_limits = teacher_joint_limits(profile, control_config, teacher)
    close_open = float(teacher.close_gripper_opening_m)

    start_positions = public_dual_arm_positions(
        snapshot.qpos, left_arm_qpos, right_arm_qpos, open_total, open_total
    )

    def public(phase: str, right_opening: float) -> tuple[float, ...]:
        return public_dual_arm_positions(
            solved[phase].qpos, left_arm_qpos, right_arm_qpos, open_total, right_opening
        )

    specs = (
        ("pregrasp", "pregrasp", public("pregrasp", open_total), "arm", False),
        ("grasp", "grasp", public("grasp", open_total), "arm", False),
        ("close", "close", public("grasp", close_open), "close", False),
        ("lift", "lift", public("lift", close_open), "lift", False),
        ("transport", "transport", public("place_hover", close_open), "carry", False),
        ("lower", "lower", public("place", close_open), "place", False),
        ("open", "open", public("place", open_total), "release", False),
        ("retreat", "retreat", public("retreat", open_total), "retreat", False),
        ("stable", "stable", public("retreat", open_total), "stable", True),
    )

    segments: list[StackPlanSegment] = []
    cur = start_positions
    for idx, (name, phase, end, gate, is_final) in enumerate(specs):
        min_s = float(teacher.phase_min_duration_s[phase])
        if is_final:
            duration_s = float(max(min_s, task_config.success.stable_duration_s))
        else:
            duration_s = duration_from_limits(
                cur, end, velocity_limits, acceleration_limits, minimum_s=min_s
            )
        segments.append(
            StackPlanSegment(
                index=idx,
                name=name,
                phase=phase,
                start_positions=cur,
                end_positions=end,
                duration_s=duration_s,
                gate=gate,
                final=is_final,
            )
        )
        cur = end

    total_s = sum(s.duration_s for s in segments)
    if total_s > teacher.max_plan_duration_s + 1e-9:
        raise _error(
            "TEACHER_PLAN_TOO_LONG",
            f"plan {total_s:.3f} s exceeds {teacher.max_plan_duration_s:g} s",
        )
    if total_s >= task_config.success.max_task_sim_duration_s - 1e-9:
        raise _error("TEACHER_PLAN_TOO_LONG", "plan leaves no task-budget margin")

    ik_pos_err = max(float(r.position_error_m) for r in ik_results)
    ik_ori_err = max(float(r.orientation_error_rad) for r in ik_results)
    expected_placed_z = (
        table_top_z
        + 2.0 * target_half_z
        + source_half_z
        + float(kinematics_config.targets.place_clearance_m)
    )
    digest = _plan_digest(
        task_config, snapshot, tuple(segments), source_pose.center, target_pose.center
    )
    return StackPlan(
        context=snapshot.context,
        task_epoch=snapshot.task_epoch,
        config_digest=task_config.config_digest,
        plan_digest=digest,
        segments=tuple(segments),
        duration_ns=round(total_s * 1e9),
        velocities=velocity_limits,
        accelerations=acceleration_limits,
        ik_results=ik_results,
        collision_results=clearance_checks,
        allowed_contact_pairs=allowed_contact_pairs,
        ik_max_position_error_m=ik_pos_err,
        ik_max_orientation_error_rad=ik_ori_err,
        source_initial_center=source_pose.center,
        target_center=target_pose.center,
        expected_placed_source_z_m=expected_placed_z,
        start_positions=start_positions,
    )


def sample_teacher_chunk(
    plan: StackPlan,
    *,
    gate: StackGateFact,
    feedback: TeacherFeedback,
    request_id: str,
    task_epoch: int,
    capture_seq: int,
    observation_time_ns: int,
    chunk_size: int,
    period_ns: int,
    profile: RobotProfile,
    control_config: ControlConfig,
    start_sim_time_ns: int | None = None,
) -> TaskChunk:
    return sample_step_chunk(
        plan,
        steps=plan.segments,
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
        start_sim_time_ns=(
            gate.active_start_sim_time_ns
            if start_sim_time_ns is None
            else int(start_sim_time_ns)
        ),
        error_cls=TeacherError,
    )
