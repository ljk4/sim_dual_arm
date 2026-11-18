"""Independent stack-task state, physical evidence and success evaluation."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from robot_contracts.models import ContractError, RunContext
from robot_contracts.task_state import TaskState

from robot_backend_sim.panda_sim.stack.config import StackTaskConfig, SuccessThresholds

__all__ = ["ObjectState", "StackTaskError", "StackTaskRuntime", "TaskBinding",
           "TaskCondition", "TaskEvaluation", "TaskObservation",
           "capture_task_observation", "evaluate_task", "make_task_binding"]

_FINGER_BODIES = (
    "left_left_finger", "left_right_finger", "right_left_finger", "right_right_finger",
)
_TERMINAL_STATES = ("SUCCEEDED", "FAILED")


class StackTaskError(ContractError):
    """A bounded task-interface error using the public ContractError shape."""


_error = StackTaskError


@dataclass(frozen=True, slots=True)
class ObjectState:


    center: tuple[float, float, float]
    quaternion_wxyz: tuple[float, float, float, float]
    linear_velocity: tuple[float, float, float]
    angular_velocity: tuple[float, float, float]

    @property
    def linear_speed_m_s(self) -> float:
        return math.sqrt(sum(value * value for value in self.linear_velocity))

    @property
    def angular_speed_rad_s(self) -> float:
        return math.sqrt(sum(value * value for value in self.angular_velocity))

    @property
    def tilt_rad(self) -> float:
        _, x, y, _ = self.quaternion_wxyz
        z_world = 1.0 - 2.0 * (x * x + y * y)
        return math.acos(max(-1.0, min(1.0, z_world)))


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
    right_finger_source_contacts: int = 0
    right_finger_source_min_force_n: float = 0.0
    right_finger_source_max_speed_m_s: float = 0.0


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
        for condition in self.conditions:
            if condition.name == name:
                return condition
        raise KeyError(name)


@dataclass(frozen=True, slots=True)
class TaskBinding:


    source_body_id: int
    target_body_id: int
    table_body_id: int
    finger_body_ids: tuple[int, ...]
    source_qpos_address: int
    target_qpos_address: int
    source_dof_address: int
    target_dof_address: int
    right_tcp_site_id: int
    source_half_extents_m: tuple[float, float, float]
    target_half_extents_m: tuple[float, float, float]
    table_top_z_m: float


def make_task_binding(model: Any, config: StackTaskConfig, *, table_top_z_m: float) -> TaskBinding:
    """Resolve fixed object/finger/table/TCP names and validate cube geometry."""
    try:
        source_body = int(model.body(config.source_object).id)
        target_body = int(model.body(config.target_object).id)
        table_body = int(model.body("table").id)
    except KeyError as error:
        raise _error("MODEL_MISMATCH", f"task body is missing: {error}") from error
    finger_ids: list[int] = []
    for name in _FINGER_BODIES:
        try:
            finger_ids.append(int(model.body(name).id))
        except KeyError as error:
            raise _error("MODEL_MISMATCH", f"task finger body is missing: {error}") from error

    def free_addresses(body_id: int, name: str) -> tuple[int, int]:
        joint_id = int(model.body_jntadr[body_id])
        if int(model.body_jntnum[body_id]) != 1 or int(model.jnt_type[joint_id]) != 0:
            raise _error("MODEL_MISMATCH", f"{name!r} must own exactly one free joint")
        return int(model.jnt_qposadr[joint_id]), int(model.jnt_dofadr[joint_id])

    source_qpos, source_dof = free_addresses(source_body, config.source_object)
    target_qpos, target_dof = free_addresses(target_body, config.target_object)
    try:
        tcp_site = int(model.site("right_tcp").id)
    except KeyError as error:
        raise _error("MODEL_MISMATCH", "right_tcp site is missing") from error
    return TaskBinding(
        source_body_id=source_body, target_body_id=target_body, table_body_id=table_body,
        finger_body_ids=tuple(finger_ids), source_qpos_address=source_qpos,
        target_qpos_address=target_qpos, source_dof_address=source_dof,
        target_dof_address=target_dof, right_tcp_site_id=tcp_site,
        source_half_extents_m=config.source_half_extents_m,
        target_half_extents_m=config.target_half_extents_m,
        table_top_z_m=float(table_top_z_m),
    )


def _object_state(data: Any, qpos_address: int, dof_address: int) -> ObjectState:
    try:
        qpos = tuple(float(value) for value in data.qpos[qpos_address : qpos_address + 7])
        qvel = tuple(float(value) for value in data.qvel[dof_address : dof_address + 6])
    except (IndexError, TypeError, ValueError) as error:
        raise _error("INVALID_TASK_STATE", "object free-joint state is not readable") from error
    if len(qpos) != 7 or len(qvel) != 6 or not all(math.isfinite(v) for v in qpos + qvel):
        raise _error("INVALID_TASK_STATE", "object free-joint state is not finite")
    return ObjectState(
        center=(qpos[0], qpos[1], qpos[2]),
        quaternion_wxyz=(qpos[3], qpos[4], qpos[5], qpos[6]),
        linear_velocity=(qvel[0], qvel[1], qvel[2]),
        angular_velocity=(qvel[3], qvel[4], qvel[5]),
    )


def _contacts_between(
    model: Any, data: Any, first: int, second: int, *, vertical_only: float | None
) -> tuple[float, int]:
    import mujoco
    import numpy as np

    body_of_geom = [int(value) for value in model.geom_bodyid]
    total, count = 0.0, 0
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        body1 = body_of_geom[int(contact.geom1)]
        body2 = body_of_geom[int(contact.geom2)]
        if not (
            (body1 == first and body2 == second) or (body1 == second and body2 == first)
        ):
            continue
        if vertical_only is not None and abs(float(contact.frame[2])) < vertical_only:
            continue
        force = np.zeros(6, dtype=float)
        mujoco.mj_contactForce(model, data, index, force)
        total += abs(float(force[0]))
        count += 1
    return total, count


def _finger_contacts(
    model: Any, data: Any, finger_ids: tuple[int, ...], object_ids: tuple[int, int]
) -> tuple[int, float]:
    import mujoco
    import numpy as np

    finger_set, object_set = set(finger_ids), set(object_ids)
    body_of_geom = [int(value) for value in model.geom_bodyid]
    count, total = 0, 0.0
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        if int(contact.efc_address) < 0:
            continue
        body1 = body_of_geom[int(contact.geom1)]
        body2 = body_of_geom[int(contact.geom2)]
        if (body1 in finger_set and body2 in object_set) or (
            body2 in finger_set and body1 in object_set
        ):
            force = np.zeros(6, dtype=float)
            mujoco.mj_contactForce(model, data, index, force)
            count += 1
            total += abs(float(force[0]))
    return count, total


def _right_finger_source_evidence(
    scene: Any, data: Any, binding: TaskBinding
) -> tuple[int, float, float]:
    """Return contacts/min force for both right fingers on the source only."""
    import mujoco
    import numpy as np

    model = scene.model
    body_of_geom = [int(value) for value in model.geom_bodyid]
    source = binding.source_body_id
    right_fingers = {
        int(model.body("right_left_finger").id), int(model.body("right_right_finger").id)
    }
    per_finger: dict[int, list[float]] = {body: [] for body in right_fingers}
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        if int(contact.efc_address) < 0:
            continue
        body1 = body_of_geom[int(contact.geom1)]
        body2 = body_of_geom[int(contact.geom2)]
        if body1 in right_fingers and body2 == source:
            other = body1
        elif body2 in right_fingers and body1 == source:
            other = body2
        else:
            continue
        force = np.zeros(6, dtype=float)
        mujoco.mj_contactForce(model, data, index, force)
        per_finger[other].append(abs(float(force[0])))
    contacts = sum(1 for forces in per_finger.values() if forces)
    if contacts == 0:
        return 0, 0.0, 0.0
    min_force = min(sum(forces) for forces in per_finger.values() if forces)
    speeds = [
        abs(float(data.qvel[int(model.joint(f"right_finger_joint{index}").dofadr[0])]))
        for index in (1, 2)
    ]
    return contacts, min_force, max(speeds)


def capture_task_observation(
    scene: Any, binding: TaskBinding, config: StackTaskConfig
) -> TaskObservation:
    """Copy all task evidence from live ``MjData`` on the owner thread only."""
    model, data = scene.model, scene.data
    source = _object_state(data, binding.source_qpos_address, binding.source_dof_address)
    target = _object_state(data, binding.target_qpos_address, binding.target_dof_address)
    threshold = config.success.min_vertical_normal_component
    source_target, _ = _contacts_between(
        model, data, binding.source_body_id, binding.target_body_id, vertical_only=threshold
    )
    target_table, _ = _contacts_between(
        model, data, binding.target_body_id, binding.table_body_id, vertical_only=threshold
    )
    source_table, _ = _contacts_between(
        model, data, binding.source_body_id, binding.table_body_id, vertical_only=threshold
    )
    finger_count, _ = _finger_contacts(
        model, data, binding.finger_body_ids, (binding.source_body_id, binding.target_body_id)
    )
    right_contacts, right_min_force, right_max_speed = _right_finger_source_evidence(
        scene, data, binding
    )
    finger_addresses = [
        *scene.indices.robot.finger_joint_qpos["right"],
    ]
    opening = float(data.qpos[finger_addresses[0]] + data.qpos[finger_addresses[1]])
    tcp = tuple(float(value) for value in data.site_xpos[binding.right_tcp_site_id])
    distance = math.sqrt(sum((tcp[index] - source.center[index]) ** 2 for index in range(3)))
    return TaskObservation(
        sim_time_ns=round(float(data.time) * 1_000_000_000), source=source, target=target,
        source_target_normal_force_n=source_target,
        target_table_normal_force_n=target_table,
        source_table_normal_force_n=source_table, finger_contact_count=finger_count,
        right_gripper_opening_m=opening, right_tcp_position=tcp,
        right_tcp_to_source_m=distance, table_top_z_m=float(scene.config.table.top_z),
        right_finger_source_contacts=right_contacts,
        right_finger_source_min_force_n=right_min_force,
        right_finger_source_max_speed_m_s=right_max_speed,
    )


def _pose(state: ObjectState | None) -> dict[str, object] | None:
    if state is None:
        return None
    return {"center": list(state.center), "quaternion_wxyz": list(state.quaternion_wxyz),
            "linear_velocity": list(state.linear_velocity),
            "angular_velocity": list(state.angular_velocity)}


def _metric(observation: TaskObservation | None, name: str, default: object = 0.0) -> object:
    return default if observation is None else getattr(observation, name)


def _height_error(observation: TaskObservation, binding: TaskBinding) -> float:
    expected = (
        observation.target.center[2]
        + binding.target_half_extents_m[2]
        + binding.source_half_extents_m[2]
    )
    return observation.source.center[2] - expected


def _support_xy_error(
    observation: TaskObservation, binding: TaskBinding, margin_m: float
) -> tuple[float, float]:
    target = observation.target
    w, x, y, z = target.quaternion_wxyz
    rotation = (
        (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)),
        (2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)),
        (2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)),
    )
    delta = tuple(observation.source.center[i] - target.center[i] for i in range(3))
    # The rotation maps target-local vectors to world, so world deltas use R.T.
    local = tuple(
        sum(rotation[column][row] * delta[column] for column in range(3)) for row in range(3)
    )
    return (
        abs(local[0]) - (binding.target_half_extents_m[0] - margin_m),
        abs(local[1]) - (binding.target_half_extents_m[1] - margin_m),
    )


def evaluate_task(
    observation: TaskObservation, binding: TaskBinding, config: StackTaskConfig
) -> TaskEvaluation:
    """Evaluate every independent condition; no single height/contact shortcut."""
    thresholds: SuccessThresholds = config.success
    height_error = _height_error(observation, binding)
    xy_x_error, xy_y_error = _support_xy_error(
        observation, binding, thresholds.support_xy_margin_m
    )
    source, target = observation.source, observation.target
    specs = (
        ("source_on_target_height", abs(height_error) <= thresholds.support_height_tolerance_m,
         height_error, thresholds.support_height_tolerance_m, "m (source_z - target_top)",
         "height difference must equal the two half heights"),
        ("source_xy_inside_target", xy_x_error <= 0.0 and xy_y_error <= 0.0,
         max(xy_x_error, xy_y_error), 0.0, "m (over edge)",
         "source centre must stay at least the configured margin inside the target"),
        ("source_target_vertical_contact",
         observation.source_target_normal_force_n >= thresholds.min_support_normal_force_n,
         observation.source_target_normal_force_n, thresholds.min_support_normal_force_n, "N",
         "only contact normals with a sufficient world-Z component count as support"),
        ("target_on_table",
         observation.target_table_normal_force_n >= thresholds.min_table_normal_force_n
         and abs(target.center[2] - observation.table_top_z_m
                 - binding.target_half_extents_m[2]) <= thresholds.support_height_tolerance_m,
         observation.target_table_normal_force_n, thresholds.min_table_normal_force_n, "N",
         "target must be supported by the real table contact at the expected height"),
        ("source_not_on_table",
         observation.source_table_normal_force_n < thresholds.min_table_normal_force_n,
         observation.source_table_normal_force_n, thresholds.min_table_normal_force_n, "N",
         "source must be lifted off the table, not resting beside the target"),
        ("source_tilt", source.tilt_rad <= thresholds.max_tilt_rad, source.tilt_rad,
         thresholds.max_tilt_rad, "rad", "source must stay upright"),
        ("target_tilt", target.tilt_rad <= thresholds.max_tilt_rad, target.tilt_rad,
         thresholds.max_tilt_rad, "rad", "target must stay upright"),
        ("source_linear_speed", source.linear_speed_m_s <= thresholds.max_linear_speed_m_s,
         source.linear_speed_m_s, thresholds.max_linear_speed_m_s, "m/s",
         "source must be at rest"),
        ("target_linear_speed", target.linear_speed_m_s <= thresholds.max_linear_speed_m_s,
         target.linear_speed_m_s, thresholds.max_linear_speed_m_s, "m/s",
         "target must be at rest"),
        ("source_angular_speed", source.angular_speed_rad_s <= thresholds.max_angular_speed_rad_s,
         source.angular_speed_rad_s, thresholds.max_angular_speed_rad_s, "rad/s",
         "source must not rotate"),
        ("target_angular_speed", target.angular_speed_rad_s <= thresholds.max_angular_speed_rad_s,
         target.angular_speed_rad_s, thresholds.max_angular_speed_rad_s, "rad/s",
         "target must not rotate"),
        ("no_finger_contact", observation.finger_contact_count == 0,
         float(observation.finger_contact_count), 0.0, "contacts",
         "neither finger may touch or pinch either block"),
        ("right_gripper_opening",
         observation.right_gripper_opening_m >= thresholds.min_gripper_opening_m,
         observation.right_gripper_opening_m, thresholds.min_gripper_opening_m, "m",
         "the right gripper must be measurably released"),
        ("right_tcp_clearance",
         observation.right_tcp_to_source_m >= thresholds.min_tcp_clearance_m,
         observation.right_tcp_to_source_m, thresholds.min_tcp_clearance_m, "m",
         "the TCP must have retreated from the source centre"),
    )
    return TaskEvaluation(tuple(TaskCondition(*spec) for spec in specs))


@dataclass(frozen=True, slots=True)
class _PrepareFact:
    request_id: str
    content_digest: str
    plan_digest: str
    state: TaskState
    key_pose_count: int = 0
    collision_segment_count: int = 0
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


class StackTaskRuntime:
    """One task's explicit prepare/start/tick/reset state machine."""

    def __init__(self, config: StackTaskConfig, binding: TaskBinding) -> None:
        self.config = config
        self.binding = binding
        self._seq = self._task_epoch = 0
        self._state, self._reason = "IDLE", ""
        self._context: RunContext | None = None
        self._plan: object | None = None
        self._plan_digest, self._plan_duration_ns = "", 0
        self._start_sim_ns = self._start_wall_ns = 0
        self._stable_start_ns: int | None = None
        self._first_satisfied_ns = self._completed_ns = 0
        self._conditions: tuple[TaskCondition, ...] = ()
        self._observation = self._success_observation = None
        self._success_conditions: tuple[TaskCondition, ...] = ()
        self._prepare_cache: dict[str, _PrepareFact] = {}
        self._start_fact: _StartFact | None = None
        self._lifted = False
    @property
    def state(self) -> TaskState:
        return self._snapshot()

    @property
    def plan(self) -> object | None:
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

    def require_prepare_slot(self) -> None:
        if self._state == "RUNNING":
            raise _error("TASK_BUSY", "a stack task is already running")
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
        self, *, request_id: str, content_digest: str, context: RunContext, plan: object,
        plan_digest: str, plan_duration_ns: int, key_pose_count: int = 0,
        collision_segment_count: int = 0, ik_max_position_error_m: float = 0.0,
        ik_max_orientation_error_rad: float = 0.0, prepare_kind: str = "teacher",
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
        self._stable_start_ns = None
        self._first_satisfied_ns = self._completed_ns = 0
        self._conditions = ()
        self._success_observation = None
        self._success_conditions = ()
        self._lifted = False
        self._start_fact = None
        fact = _PrepareFact(
            request_id, content_digest, plan_digest, self._snapshot(), key_pose_count,
            collision_segment_count, ik_max_position_error_m, ik_max_orientation_error_rad,
            plan_duration_ns / 1e9, prepare_kind,
        )
        self._prepare_cache[request_id] = fact
        return _PrepareOutcome(fact, idempotent=False)

    def start(
        self, *, request_id: str, content_digest: str, context: RunContext,
        plan_digest: str, task_epoch: int, sim_time_ns: int, wall_ns: int,
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
        self._stable_start_ns = None
        self._first_satisfied_ns = self._completed_ns = 0
        self._lifted = False
        self._conditions = ()
        self._state, self._reason = "RUNNING", ""
        state = self._snapshot()
        self._start_fact = _StartFact(request_id, content_digest, self._start_sim_ns, state)
        return state, False

    def tick(self, observation: TaskObservation, wall_ns: int) -> TaskState:
        self._seq += 1
        self._observation = observation
        if self._state != "RUNNING":
            return self._snapshot()
        elapsed_ns = observation.sim_time_ns - self._start_sim_ns
        if elapsed_ns < 0:
            raise _error("INVALID_TASK_CLOCK", "task sim clock moved backwards")
        max_sim_ns = round(self.config.success.max_task_sim_duration_s * 1e9)
        max_wall_ns = round(self.config.success.max_task_wall_duration_s * 1e9)
        if elapsed_ns > max_sim_ns:
            self._fail("TIMEOUT", "task exceeded the configured simulation budget")
            return self._snapshot()
        if int(wall_ns) - self._start_wall_ns > max_wall_ns:
            self._fail("WALL_TIMEOUT", "task exceeded the configured wall watchdog")
            return self._snapshot()
        evaluation = evaluate_task(observation, self.binding, self.config)
        self._conditions = evaluation.conditions
        if observation.source.center[2] > observation.table_top_z_m + (
            self.binding.source_half_extents_m[2] + self.config.teacher.lift_height_m
        ):
            self._lifted = True
        if self._dropped(observation):
            self._fail("DROPPED", "a block was dropped or left the table")
            return self._snapshot()
        if evaluation.all_passed:
            if self._stable_start_ns is None:
                self._stable_start_ns = observation.sim_time_ns
                self._first_satisfied_ns = observation.sim_time_ns
            stable_ns = observation.sim_time_ns - self._stable_start_ns
            if stable_ns >= round(self.config.success.stable_duration_s * 1e9):
                self._completed_ns = observation.sim_time_ns
                self._success_observation = observation
                self._success_conditions = evaluation.conditions
                self._state, self._reason = "SUCCEEDED", "SUCCESS"
        else:
            self._stable_start_ns = None
            self._first_satisfied_ns = 0
        return self._snapshot()

    def reset(self, reason: str = "RESET") -> TaskState:
        self._task_epoch += 1
        self._state, self._reason = "IDLE", reason
        self._context = None
        self._plan = None
        self._plan_digest, self._plan_duration_ns = "", 0
        self._start_sim_ns = self._start_wall_ns = 0
        self._stable_start_ns = None
        self._first_satisfied_ns = self._completed_ns = 0
        self._conditions = ()
        self._observation = None
        self._success_observation = None
        self._success_conditions = ()
        self._prepare_cache.clear()
        self._start_fact = None
        self._lifted = False
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
        self._reason = f"{reason}: {detail}" if detail else reason
        self._stable_start_ns = None

    def _dropped(self, observation: TaskObservation) -> bool:
        floor = observation.table_top_z_m - 0.02
        if observation.source.center[2] < floor or observation.target.center[2] < floor:
            return True
        return self._lifted and (
            observation.source_table_normal_force_n
            >= self.config.success.min_table_normal_force_n
        )

    def _snapshot(self) -> TaskState:
        observation = self._observation
        conditions = self._conditions
        if self._state == "SUCCEEDED" and self._success_observation is not None:
            observation = self._success_observation
            conditions = self._success_conditions
        stable_elapsed = 0.0
        if self._state == "RUNNING" and self._stable_start_ns is not None and observation:
            stable_elapsed = max(0.0, (observation.sim_time_ns - self._stable_start_ns) / 1e9)
        elif self._state == "SUCCEEDED":
            stable_elapsed = self.config.success.stable_duration_s
        bitmask = sum(1 << index for index, item in enumerate(conditions) if item.passed)
        metrics: dict[str, Any] = {
            "reason": self._reason,
            "task_start_sim_time_ns": self._start_sim_ns,
            "first_satisfied_sim_time_ns": self._first_satisfied_ns,
            "completed_sim_time_ns": self._completed_ns,
            "stable_elapsed_s": stable_elapsed,
            "source": _pose(None if observation is None else observation.source),
            "target": _pose(None if observation is None else observation.target),
            "source_target_normal_force_n": _metric(
                observation, "source_target_normal_force_n"
            ),
            "target_table_normal_force_n": _metric(observation, "target_table_normal_force_n"),
            "source_table_normal_force_n": _metric(observation, "source_table_normal_force_n"),
            "finger_contact_count": _metric(observation, "finger_contact_count", 0),
            "right_gripper_opening_m": _metric(observation, "right_gripper_opening_m"),
            "right_tcp_distance_m": _metric(observation, "right_tcp_to_source_m"),
            "conditions_passed": bool(conditions) and all(c.passed for c in conditions),
            "conditions_bitmask": bitmask,
            "conditions": [
                {"name": item.name, "passed": item.passed, "actual": item.actual,
                 "threshold": item.threshold, "unit": item.unit, "detail": item.detail}
                for item in conditions
            ],
        }
        return TaskState(
            context=self._context, state_seq=self._seq,
            sample_time_ns=0 if observation is None else observation.sim_time_ns,
            task_epoch=self._task_epoch, task_type="stack", phase=self._state,
            succeeded=self._state == "SUCCEEDED", terminal=self._state in _TERMINAL_STATES,
            failure_reason=self._reason if self._state == "FAILED" else "",
            plan_digest=self._plan_digest, task_config_digest=self.config.config_digest,
            metrics=metrics,
        )
