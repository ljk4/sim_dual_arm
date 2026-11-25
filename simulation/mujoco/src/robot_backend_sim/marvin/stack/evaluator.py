"""Independent stack task state, measured gates and 14-condition physical proof."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import mujoco
import numpy as np
from robot_contracts.models import ContractError, RunContext
from robot_contracts.task_state import TaskState

from robot_backend_sim.common.physics.robot_io import sim_time_ns
from robot_backend_sim.common.tasks.math import (
    contact_normal_force,
    quaternion_wxyz_to_matrix,
)
from robot_backend_sim.marvin.assembly import gripper_opening
from robot_backend_sim.marvin.stack.config import StackTaskConfig
from robot_backend_sim.marvin.stack.gate import StackGateFact, TeacherProgress

if TYPE_CHECKING:
    from robot_backend_sim.marvin.stack.teacher import StackPlan

__all__ = [
    "ObjectState",
    "StackTaskError",
    "StackTaskRuntime",
    "TaskBinding",
    "TaskCondition",
    "TaskEvaluation",
    "TaskObservation",
    "capture_task_observation",
    "evaluate_task",
    "make_task_binding",
]

_TERMINAL_STATES = ("SUCCEEDED", "FAILED")


class StackTaskError(ContractError):
    """Bounded stack task interface error."""


def _error(reason: str, message: str) -> StackTaskError:
    return StackTaskError(reason, message)


@dataclass(frozen=True, slots=True)
class ObjectState:
    center: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]
    linear_velocity: tuple[float, float, float]
    angular_velocity: tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class TaskBinding:
    source_body_id: int
    target_body_id: int
    table_body_id: int
    finger_body_ids: tuple[int, ...]
    right_finger_body_ids: tuple[int, int]
    left_finger_body_ids: tuple[int, int]
    source_qpos_address: int
    target_qpos_address: int
    source_dof_address: int
    target_dof_address: int
    right_tcp_site_id: int
    source_half_extents_m: tuple[float, float, float]
    target_half_extents_m: tuple[float, float, float]
    table_top_z_m: float
    right_arm_qpos_addresses: tuple[int, ...] = ()
    right_arm_dof_addresses: tuple[int, ...] = ()
    right_finger_dof_addresses: tuple[int, ...] = ()


@dataclass(frozen=True, slots=True)
class TaskObservation:
    sim_time_ns: int
    source: ObjectState
    target: ObjectState
    source_target_normal_force_n: float
    target_table_normal_force_n: float
    source_table_normal_force_n: float
    finger_contact_count: int
    right_gripper_opening_m: float
    right_tcp_position: tuple[float, float, float]
    right_tcp_to_source_m: float
    table_top_z_m: float
    source_target_vertical_component: float = 1.0
    right_finger_source_contacts: int = 0
    right_finger_source_min_force_n: float = 0.0
    right_finger_source_max_speed_m_s: float = 0.0
    right_positions: tuple[float, ...] = ()
    right_velocities: tuple[float, ...] = ()
    right_gripper_velocity_m_s: float = 0.0


@dataclass(frozen=True, slots=True)
class TaskCondition:
    name: str
    passed: bool
    actual: float
    threshold: float
    unit: str
    detail: str


@dataclass(frozen=True, slots=True)
class TaskEvaluation:
    conditions: tuple[TaskCondition, ...]

    @property
    def all_passed(self) -> bool:
        return all(condition.passed for condition in self.conditions)

    def condition(self, name: str) -> TaskCondition:
        for item in self.conditions:
            if item.name == name:
                return item
        raise KeyError(name)


def make_task_binding(
    model: mujoco.MjModel,
    config: StackTaskConfig,
    *,
    table_top_z_m: float,
) -> TaskBinding:
    del config
    try:
        source_body_id = int(model.body("source").id)
        target_body_id = int(model.body("target").id)
        table_body_id = int(model.body("table").id)
        source_joint = model.joint("source_free")
        target_joint = model.joint("target_free")
        right_tcp_site_id = int(model.site("Arm_R_tcp").id)
        left_fingers = (
            int(model.body("Arm_L10_Link").id),
            int(model.body("Arm_L11_Link").id),
        )
        right_fingers = (
            int(model.body("Arm_R10_Link").id),
            int(model.body("Arm_R11_Link").id),
        )
        right_arm_qpos = tuple(
            int(model.jnt_qposadr[int(model.joint(f"Arm_R{index}_Joint").id)])
            for index in range(1, 8)
        )
        right_arm_dof = tuple(
            int(model.jnt_dofadr[int(model.joint(f"Arm_R{index}_Joint").id)])
            for index in range(1, 8)
        )
        right_finger_dof = (
            int(model.jnt_dofadr[int(model.joint("Arm_R10_Joint").id)]),
            int(model.jnt_dofadr[int(model.joint("Arm_R11_Joint").id)]),
        )
    except KeyError as error:
        raise ContractError("MODEL_MISMATCH", f"stack binding is missing {error}") from error
    source_geom = model.geom("source_geom")
    target_geom = model.geom("target_geom")
    return TaskBinding(
        source_body_id=source_body_id,
        target_body_id=target_body_id,
        table_body_id=table_body_id,
        finger_body_ids=(*left_fingers, *right_fingers),
        right_finger_body_ids=right_fingers,
        left_finger_body_ids=left_fingers,
        source_qpos_address=int(model.jnt_qposadr[int(source_joint.id)]),
        target_qpos_address=int(model.jnt_qposadr[int(target_joint.id)]),
        source_dof_address=int(model.jnt_dofadr[int(source_joint.id)]),
        target_dof_address=int(model.jnt_dofadr[int(target_joint.id)]),
        right_tcp_site_id=right_tcp_site_id,
        source_half_extents_m=tuple(float(v) for v in model.geom_size[int(source_geom.id)]),
        target_half_extents_m=tuple(float(v) for v in model.geom_size[int(target_geom.id)]),
        table_top_z_m=float(table_top_z_m),
        right_arm_qpos_addresses=right_arm_qpos,
        right_arm_dof_addresses=right_arm_dof,
        right_finger_dof_addresses=right_finger_dof,
    )


def _contacts_between(
    model: Any, data: Any, body_a: int, body_b: int
) -> list[tuple[int, Any, float]]:
    body_of_geom = model.geom_bodyid
    hits: list[tuple[int, Any, float]] = []
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        if int(contact.efc_address) < 0:
            continue
        b1 = int(body_of_geom[int(contact.geom1)])
        b2 = int(body_of_geom[int(contact.geom2)])
        if (b1 == body_a and b2 == body_b) or (b1 == body_b and b2 == body_a):
            hits.append((index, contact, contact_normal_force(model, data, index)))
    return hits


def _finger_contacts(
    model: Any,
    data: Any,
    finger_body_ids: tuple[int, ...],
    object_body_ids: tuple[int, ...],
) -> tuple[int, float]:
    """Count distinct finger bodies actively touching any named object body."""
    body_of_geom = model.geom_bodyid
    finger_set = set(finger_body_ids)
    object_set = set(object_body_ids)
    active_fingers: set[int] = set()
    max_force = 0.0
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        if int(contact.efc_address) < 0:
            continue
        b1 = int(body_of_geom[int(contact.geom1)])
        b2 = int(body_of_geom[int(contact.geom2)])
        if (b1 in finger_set and b2 in object_set) or (b2 in finger_set and b1 in object_set):
            finger_id = b1 if b1 in finger_set else b2
            active_fingers.add(finger_id)
            max_force = max(max_force, contact_normal_force(model, data, index))
    return len(active_fingers), max_force


def _right_finger_source_evidence(
    scene: Any, data: Any, binding: TaskBinding
) -> tuple[int, float, float]:
    """Count active right fingers holding source, min per-finger force, max finger speed."""
    model = scene.model
    body_of_geom = model.geom_bodyid
    right_fingers = set(binding.right_finger_body_ids)
    per_finger_force: dict[int, float] = {}
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        if int(contact.efc_address) < 0:
            continue
        b1 = int(body_of_geom[int(contact.geom1)])
        b2 = int(body_of_geom[int(contact.geom2)])
        if (b1 in right_fingers and b2 == binding.source_body_id) or (
            b2 in right_fingers and b1 == binding.source_body_id
        ):
            fid = b1 if b1 in right_fingers else b2
            per_finger_force[fid] = per_finger_force.get(fid, 0.0) + contact_normal_force(
                model, data, index
            )
    max_speed = max(
        (abs(float(data.qvel[a])) for a in binding.right_finger_dof_addresses),
        default=0.0,
    )
    return len(per_finger_force), min(per_finger_force.values(), default=0.0), max_speed


def _object_state(data: Any, qpos_addr: int, dof_addr: int) -> ObjectState:
    q = data.qpos[qpos_addr : qpos_addr + 7]
    v = data.qvel[dof_addr : dof_addr + 6]
    return ObjectState(
        center=(float(q[0]), float(q[1]), float(q[2])),
        quaternion_wxyz=(float(q[3]), float(q[4]), float(q[5]), float(q[6])),
        linear_velocity=(float(v[0]), float(v[1]), float(v[2])),
        angular_velocity=(float(v[3]), float(v[4]), float(v[5])),
    )


def _support_upward_component(
    model: Any, contact: Any, upper_body_id: int, lower_body_id: int
) -> float:
    b1 = int(model.geom_bodyid[int(contact.geom1)])
    b2 = int(model.geom_bodyid[int(contact.geom2)])
    nz = float(contact.frame[2])
    if b1 == lower_body_id and b2 == upper_body_id:
        return nz
    if b1 == upper_body_id and b2 == lower_body_id:
        return -nz
    return 0.0


def capture_task_observation(
    scene: Any, binding: TaskBinding, config: StackTaskConfig
) -> TaskObservation:
    del config
    model = scene.model
    data = scene.data
    source = _object_state(data, binding.source_qpos_address, binding.source_dof_address)
    target = _object_state(data, binding.target_qpos_address, binding.target_dof_address)

    st_hits = _contacts_between(
        model, data, binding.source_body_id, binding.target_body_id
    )
    st_force = sum(force for _, _, force in st_hits)
    st_vertical = (
        max(
            _support_upward_component(
                model, contact, binding.source_body_id, binding.target_body_id
            )
            for _, contact, _ in st_hits
        )
        if st_hits
        else 0.0
    )

    tt_hits = _contacts_between(
        model, data, binding.target_body_id, binding.table_body_id
    )
    tt_force = sum(force for _, _, force in tt_hits)

    stab_hits = _contacts_between(
        model, data, binding.source_body_id, binding.table_body_id
    )
    stab_force = sum(force for _, _, force in stab_hits)

    finger_count, _ = _finger_contacts(
        model,
        data,
        binding.finger_body_ids,
        (binding.source_body_id, binding.target_body_id),
    )
    rf_contacts, rf_min_force, rf_max_speed = _right_finger_source_evidence(
        scene, data, binding
    )

    tcp_pos = tuple(float(v) for v in data.site_xpos[binding.right_tcp_site_id])
    tcp_dist = math.dist(tcp_pos, source.center)
    opening_m = float(gripper_opening(model, data, scene.indices.robot, "right"))
    gripper_vel = (
        -sum(float(data.qvel[a]) for a in binding.right_finger_dof_addresses)
        if binding.right_finger_dof_addresses
        else 0.0
    )
    right_pos = (
        tuple(float(data.qpos[a]) for a in binding.right_arm_qpos_addresses)
        if binding.right_arm_qpos_addresses
        else ()
    )
    right_vel = (
        tuple(float(data.qvel[a]) for a in binding.right_arm_dof_addresses)
        if binding.right_arm_dof_addresses
        else ()
    )

    return TaskObservation(
        sim_time_ns=sim_time_ns(data.time),
        source=source,
        target=target,
        source_target_normal_force_n=st_force,
        target_table_normal_force_n=tt_force,
        source_table_normal_force_n=stab_force,
        finger_contact_count=finger_count,
        right_gripper_opening_m=opening_m,
        right_tcp_position=tcp_pos,  # type: ignore[arg-type]
        right_tcp_to_source_m=tcp_dist,
        table_top_z_m=binding.table_top_z_m,
        source_target_vertical_component=st_vertical,
        right_finger_source_contacts=rf_contacts,
        right_finger_source_min_force_n=rf_min_force,
        right_finger_source_max_speed_m_s=rf_max_speed,
        right_positions=right_pos,
        right_velocities=right_vel,
        right_gripper_velocity_m_s=gripper_vel,
    )


def _quat_to_rot(q: tuple[float, float, float, float]) -> np.ndarray:
    return np.asarray(quaternion_wxyz_to_matrix(q), dtype=np.float64)


def _upright_tilt_rad(q: tuple[float, float, float, float]) -> float:
    rot = _quat_to_rot(q)
    # Any 90-degree cube rotation flips a local axis away from world +Z; the upright
    # axis for our cubes is local Z, so dot with world +Z checks face-up alignment.
    cos_theta = max(-1.0, min(1.0, float(rot[2, 2])))
    return math.acos(cos_theta)


def _norm3(v: tuple[float, float, float]) -> float:
    return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def evaluate_task(
    observation: TaskObservation, binding: TaskBinding, config: StackTaskConfig
) -> TaskEvaluation:
    success = config.success
    s_half = binding.source_half_extents_m
    t_half = binding.target_half_extents_m

    expected_z = (
        observation.target.center[2] + t_half[2] + s_half[2]
    )
    z_err = abs(observation.source.center[2] - expected_z)
    h_ok = z_err <= success.support_height_tolerance_m

    # Project source center into target body's local XY frame so tolerance stays invariant to yaw.
    t_rot = _quat_to_rot(observation.target.quaternion_wxyz)
    delta = np.asarray(observation.source.center, dtype=np.float64) - np.asarray(
        observation.target.center, dtype=np.float64
    )
    local_delta = t_rot.T @ delta
    margin_x = abs(float(local_delta[0])) - (t_half[0] - success.support_xy_margin_m)
    margin_y = abs(float(local_delta[1])) - (t_half[1] - success.support_xy_margin_m)
    xy_excess = max(margin_x, margin_y)
    xy_ok = xy_excess <= 0.0

    st_ok = (
        observation.source_target_normal_force_n >= success.min_support_normal_force_n
        and observation.source_target_vertical_component
        >= success.min_vertical_normal_component
    )

    target_z_expected = observation.table_top_z_m + t_half[2]
    target_z_ok = (
        abs(observation.target.center[2] - target_z_expected)
        <= success.support_height_tolerance_m
    )
    tt_ok = (
        observation.target_table_normal_force_n >= success.min_target_table_normal_force_n
        and target_z_ok
    )

    s_not_table_ok = (
        observation.source_table_normal_force_n <= success.max_source_table_normal_force_n
    )

    s_tilt = _upright_tilt_rad(observation.source.quaternion_wxyz)
    t_tilt = _upright_tilt_rad(observation.target.quaternion_wxyz)
    s_tilt_ok = s_tilt <= success.max_tilt_rad
    t_tilt_ok = t_tilt <= success.max_tilt_rad

    s_lin = _norm3(observation.source.linear_velocity)
    t_lin = _norm3(observation.target.linear_velocity)
    s_ang = _norm3(observation.source.angular_velocity)
    t_ang = _norm3(observation.target.angular_velocity)

    conditions = (
        TaskCondition(
            "source_on_target_height",
            h_ok,
            z_err,
            success.support_height_tolerance_m,
            "m",
            f"|source_z - expected_z| = {z_err:.5f} m",
        ),
        TaskCondition(
            "source_xy_inside_target",
            xy_ok,
            xy_excess,
            0.0,
            "m",
            f"signed xy overhang inside margin = {xy_excess:.6f} m",
        ),
        TaskCondition(
            "source_target_vertical_contact",
            st_ok,
            observation.source_target_normal_force_n,
            success.min_support_normal_force_n,
            "N",
            f"force={observation.source_target_normal_force_n:.3f}N "
            f"nz={observation.source_target_vertical_component:.3f}",
        ),
        TaskCondition(
            "target_on_table",
            tt_ok,
            observation.target_table_normal_force_n,
            success.min_target_table_normal_force_n,
            "N",
            f"force={observation.target_table_normal_force_n:.3f}N target_z_ok={target_z_ok}",
        ),
        TaskCondition(
            "source_not_on_table",
            s_not_table_ok,
            observation.source_table_normal_force_n,
            success.max_source_table_normal_force_n,
            "N",
            f"source_table_force={observation.source_table_normal_force_n:.4f}N",
        ),
        TaskCondition(
            "source_tilt",
            s_tilt_ok,
            s_tilt,
            success.max_tilt_rad,
            "rad",
            f"tilt={s_tilt:.4f} rad",
        ),
        TaskCondition(
            "target_tilt",
            t_tilt_ok,
            t_tilt,
            success.max_tilt_rad,
            "rad",
            f"tilt={t_tilt:.4f} rad",
        ),
        TaskCondition(
            "source_linear_speed",
            s_lin <= success.max_linear_speed_m_s,
            s_lin,
            success.max_linear_speed_m_s,
            "m/s",
            f"speed={s_lin:.4f} m/s",
        ),
        TaskCondition(
            "target_linear_speed",
            t_lin <= success.max_linear_speed_m_s,
            t_lin,
            success.max_linear_speed_m_s,
            "m/s",
            f"speed={t_lin:.4f} m/s",
        ),
        TaskCondition(
            "source_angular_speed",
            s_ang <= success.max_angular_speed_rad_s,
            s_ang,
            success.max_angular_speed_rad_s,
            "rad/s",
            f"ang={s_ang:.4f} rad/s",
        ),
        TaskCondition(
            "target_angular_speed",
            t_ang <= success.max_angular_speed_rad_s,
            t_ang,
            success.max_angular_speed_rad_s,
            "rad/s",
            f"ang={t_ang:.4f} rad/s",
        ),
        TaskCondition(
            "no_finger_contact",
            observation.finger_contact_count == 0,
            float(observation.finger_contact_count),
            0.0,
            "count",
            f"active_finger_contacts={observation.finger_contact_count}",
        ),
        TaskCondition(
            "right_gripper_opening",
            observation.right_gripper_opening_m >= success.min_gripper_opening_m,
            observation.right_gripper_opening_m,
            success.min_gripper_opening_m,
            "m",
            f"opening={observation.right_gripper_opening_m:.5f} m",
        ),
        TaskCondition(
            "right_tcp_clearance",
            observation.right_tcp_to_source_m >= success.min_right_tcp_to_source_m,
            observation.right_tcp_to_source_m,
            success.min_right_tcp_to_source_m,
            "m",
            f"distance={observation.right_tcp_to_source_m:.4f} m",
        ),
    )
    return TaskEvaluation(conditions=conditions)


@dataclass(frozen=True, slots=True)
class _PrepareFact:
    request_id: str
    content_digest: str
    plan_digest: str
    plan_duration_ns: int
    step_count: int
    ik_max_position_error_m: float
    ik_max_orientation_error_rad: float
    prepare_kind: str
    state: TaskState


@dataclass(frozen=True, slots=True)
class PrepareOutcome:
    fact: _PrepareFact
    idempotent: bool


@dataclass(frozen=True, slots=True)
class _StartFact:
    request_id: str
    content_digest: str
    state: TaskState


class StackTaskRuntime:
    """Owner-thread state machine for prepare, running evaluation and teacher gates."""

    def __init__(self, config: StackTaskConfig, binding: TaskBinding) -> None:
        self.config = config
        self.binding = binding
        self._task_epoch = 0
        self._seq = 0
        self._state = "IDLE"
        self._reason = ""
        self._context: RunContext | None = None
        self._plan: StackPlan | None = None
        self._plan_digest = ""
        self._plan_duration_ns = 0
        self._prepare_kind = "teacher"
        self._start_sim_ns = 0
        self._start_wall_ns = 0
        self._first_satisfied_ns: int | None = None
        self._completed_ns: int | None = None
        self._conditions: tuple[TaskCondition, ...] = ()
        self._observation: TaskObservation | None = None
        self._success_conditions: tuple[TaskCondition, ...] = ()
        self._success_observation: TaskObservation | None = None
        self._prepare_cache: dict[str, _PrepareFact] = {}
        self._start_cache: dict[str, _StartFact] = {}
        self._progress: TeacherProgress | None = None

    @property
    def task_epoch(self) -> int:
        return self._task_epoch

    @property
    def plan(self) -> StackPlan | None:
        return self._plan

    @property
    def plan_digest(self) -> str:
        return self._plan_digest

    @property
    def plan_duration_ns(self) -> int:
        return self._plan_duration_ns

    @property
    def progress(self) -> TeacherProgress | None:
        return self._progress

    @property
    def state(self) -> TaskState:
        obs = self._success_observation if self._state == "SUCCEEDED" else self._observation
        conditions = self._success_conditions if self._state == "SUCCEEDED" else self._conditions
        obs_ns = 0 if obs is None else int(obs.sim_time_ns)
        stable_elapsed_s = (
            0.0
            if self._first_satisfied_ns is None
            else max(
                0.0,
                (
                    (self._completed_ns if self._completed_ns is not None else obs_ns)
                    - self._first_satisfied_ns
                )
                / 1e9,
            )
        )
        metrics: dict[str, Any] = {
            "task_start_sim_time_ns": int(self._start_sim_ns),
            "first_satisfied_sim_time_ns": 0
            if self._first_satisfied_ns is None
            else int(self._first_satisfied_ns),
            "completed_sim_time_ns": 0
            if self._completed_ns is None
            else int(self._completed_ns),
            "stable_elapsed_s": stable_elapsed_s,
            "plan_duration_ns": int(self._plan_duration_ns),
            "reason": self._reason,
            "finger_contact_count": 0 if obs is None else int(obs.finger_contact_count),
            "conditions_passed": bool(conditions) and all(c.passed for c in conditions),
            "conditions_bitmask": sum(
                (1 << idx) for idx, c in enumerate(conditions) if c.passed
            ),
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
        }
        if self._progress is not None:
            fact = self._progress.fact()
            metrics["teacher_phase"] = fact.phase
            metrics["teacher_step_index"] = fact.step_index
        return TaskState(
            context=self._context,
            state_seq=self._seq,
            sample_time_ns=obs_ns,
            task_epoch=self._task_epoch,
            task_type="stack",
            phase=self._state,
            succeeded=self._state == "SUCCEEDED",
            terminal=self._state in _TERMINAL_STATES,
            failure_reason=self._reason if self._state == "FAILED" else "",
            plan_digest=self._plan_digest,
            task_config_digest=self.config.config_digest,
            metrics=metrics,
        )

    def gate_fact(self) -> StackGateFact:
        if self._progress is None:
            return StackGateFact(
                step_index=0,
                step="",
                phase="",
                status="IDLE",
                active_start_sim_time_ns=-1,
                phase_wait_s=0.0,
                stable_elapsed_s=0.0,
                failure="",
            )
        return self._progress.fact()

    def require_prepare_slot(self) -> None:
        if self._state == "RUNNING":
            raise _error("TASK_BUSY", "stack task is already running")
        if self._state == "PREPARED":
            raise _error("TASK_ALREADY_PREPARED", "reset before preparing again")
        if self._state in _TERMINAL_STATES:
            raise _error("TASK_TERMINAL", "reset terminal task before preparing again")
        if len(self._prepare_cache) >= self.config.capacity.rpc_cache_entries:
            raise _error("TASK_CACHE_FULL", "prepare idempotency cache is full")

    def lookup_prepare(self, request_id: str, content_digest: str) -> PrepareOutcome | None:
        fact = self._prepare_cache.get(request_id)
        if fact is None:
            return None
        if fact.content_digest != content_digest:
            raise _error("REQUEST_ID_CONFLICT", "same prepare request_id has different content")
        return PrepareOutcome(fact, idempotent=True)

    def commit_prepare(
        self,
        *,
        request_id: str,
        content_digest: str,
        context: RunContext,
        plan: StackPlan | object,
        plan_digest: str,
        plan_duration_ns: int,
        step_count: int = 0,
        ik_max_position_error_m: float = 0.0,
        ik_max_orientation_error_rad: float = 0.0,
        prepare_kind: str = "teacher",
    ) -> PrepareOutcome:
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
        self._context = context
        self._plan = plan  # type: ignore[assignment]
        self._plan_digest = plan_digest
        self._plan_duration_ns = int(plan_duration_ns)
        self._prepare_kind = prepare_kind
        self._state = "PREPARED"
        self._reason = ""
        self._start_sim_ns = 0
        self._start_wall_ns = 0
        self._first_satisfied_ns = None
        self._completed_ns = None
        self._conditions = ()
        self._observation = None
        self._success_conditions = ()
        self._success_observation = None
        self._start_cache.clear()
        self._progress = (
            TeacherProgress(plan, self.config)
            if hasattr(plan, "segments")
            else None
        )
        state = self.state
        fact = _PrepareFact(
            request_id=request_id,
            content_digest=content_digest,
            plan_digest=plan_digest,
            plan_duration_ns=self._plan_duration_ns,
            step_count=step_count,
            ik_max_position_error_m=ik_max_position_error_m,
            ik_max_orientation_error_rad=ik_max_orientation_error_rad,
            prepare_kind=prepare_kind,
            state=state,
        )
        self._prepare_cache[request_id] = fact
        return PrepareOutcome(fact, idempotent=False)

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
        cached = self._start_cache.get(request_id)
        if cached is not None:
            if cached.content_digest != content_digest:
                raise _error("REQUEST_ID_CONFLICT", "same start request_id has different content")
            return cached.state, True
        if self._plan is None or self._state == "IDLE":
            raise _error("TASK_NOT_PREPARED", "prepare task before starting")
        if self._state != "PREPARED":
            raise _error("START_CONFLICT", f"cannot start task from state {self._state}")
        if (
            self._context != context
            or task_epoch != self._task_epoch
            or plan_digest != self._plan_digest
        ):
            raise _error("START_CONFLICT", "start context, epoch or plan_digest differs")
        self._state = "RUNNING"
        self._seq += 1
        self._reason = ""
        self._start_sim_ns = int(sim_time_ns)
        self._start_wall_ns = int(wall_ns)
        self._first_satisfied_ns = None
        self._completed_ns = None
        if self._progress is not None:
            self._progress.actual_start_ns = int(sim_time_ns)
        state = self.state
        self._start_cache[request_id] = _StartFact(request_id, content_digest, state)
        return state, False

    def tick(self, observation: TaskObservation, wall_ns: int) -> TaskState:
        self._observation = observation
        if self._state in _TERMINAL_STATES or self._state != "RUNNING":
            return self.state
        sim_elapsed_s = (observation.sim_time_ns - self._start_sim_ns) / 1e9
        wall_elapsed_s = (int(wall_ns) - self._start_wall_ns) / 1e9
        if sim_elapsed_s > self.config.success.max_task_sim_duration_s:
            self._state = "FAILED"
            self._seq += 1
            self._reason = (
                f"TIMEOUT:sim_elapsed={sim_elapsed_s:.3f}s>{self.config.success.max_task_sim_duration_s:g}s"
            )
            self._completed_ns = observation.sim_time_ns
            return self.state
        if wall_elapsed_s > self.config.success.max_task_wall_duration_s:
            self._state = "FAILED"
            self._seq += 1
            self._reason = (
                f"WALL_TIMEOUT:wall_elapsed={wall_elapsed_s:.3f}s>{self.config.success.max_task_wall_duration_s:g}s"
            )
            self._completed_ns = observation.sim_time_ns
            return self.state

        if self._progress is not None and self._plan is not None:
            from robot_backend_sim.marvin.stack.gate import TeacherFeedback

            feedback = TeacherFeedback(
                capture_seq=0,
                sim_time_ns=observation.sim_time_ns,
                right_positions=observation.right_positions,
                right_velocities=observation.right_velocities,
                right_gripper_opening_m=observation.right_gripper_opening_m,
                right_gripper_velocity_m_s=observation.right_gripper_velocity_m_s,
                right_finger_source_contacts=observation.right_finger_source_contacts,
                right_finger_source_min_force_n=observation.right_finger_source_min_force_n,
                right_finger_source_max_speed_m_s=observation.right_finger_source_max_speed_m_s,
                source_center=observation.source.center,
                right_tcp_distance_m=observation.right_tcp_to_source_m,
            )
            gate = self._progress.update(feedback, self._state)
            if gate.status == "FAILED":
                self._state = "FAILED"
                self._seq += 1
                self._reason = gate.failure
                self._completed_ns = observation.sim_time_ns
                return self.state

        evaluation = evaluate_task(observation, self.binding, self.config)
        self._conditions = evaluation.conditions
        if evaluation.all_passed:
            if self._first_satisfied_ns is None:
                self._first_satisfied_ns = observation.sim_time_ns
            held_s = (observation.sim_time_ns - self._first_satisfied_ns) / 1e9
            if held_s >= self.config.success.stable_duration_s - 1e-9:
                self._state = "SUCCEEDED"
                self._seq += 1
                self._reason = "SUCCESS"
                self._completed_ns = observation.sim_time_ns
                self._success_observation = observation
                self._success_conditions = evaluation.conditions
        else:
            self._first_satisfied_ns = None
        return self.state

    def reset(self, reason: str = "reset") -> TaskState:
        del reason
        self._task_epoch += 1
        self._seq += 1
        self._state = "IDLE"
        self._reason = ""
        self._plan = None
        self._plan_digest = ""
        self._plan_duration_ns = 0
        self._start_sim_ns = 0
        self._start_wall_ns = 0
        self._first_satisfied_ns = None
        self._completed_ns = None
        self._conditions = ()
        self._observation = None
        self._success_conditions = ()
        self._success_observation = None
        self._start_cache.clear()
        self._progress = None
        return self.state

    def fail_control(self, reason: str) -> TaskState:
        if self._state in _TERMINAL_STATES:
            return self.state
        self._state = "FAILED"
        self._seq += 1
        self._reason = reason or "CONTROL_FAILED"
        return self.state
