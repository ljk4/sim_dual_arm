"""Generic simulation-task plugin protocol and immutable host-facing facts.

Nothing here imports ROS, MuJoCo or a concrete task implementation.  Host
layers (worker, owner, node, runtime client) depend only on this protocol and
on ``robot_contracts.task_state.TaskState``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Protocol, runtime_checkable

from robot_contracts.models import RobotProfile, RunContext
from robot_contracts.task_state import TaskState

__all__ = [
    "SimTaskPlugin",
    "TaskChunk",
    "TaskChunkView",
    "TaskPlan",
    "TaskPrepareResult",
    "TaskPrepareSnapshot",
]


@runtime_checkable
class TaskPrepareSnapshot(Protocol):
    """Opaque immutable truth captured on the owner thread before planning."""

    context: RunContext
    task_epoch: int
    sim_time_ns: int
    capture_seq: int
    qpos: tuple[float, ...]
    seed: int
    controller_command_id: str


@dataclass(frozen=True, slots=True)
class TaskPlan:
    """One planner result ready for the owner to commit immutably."""

    plan: object
    plan_digest: str
    duration_ns: int
    prepare_kind: str = "teacher"


@dataclass(frozen=True, slots=True)
class TaskPrepareResult:
    """Bounded prepare fact returned to the generic PrepareTask service."""

    task_type: str
    task_epoch: int
    task_config_digest: str
    plan_digest: str
    idempotent: bool
    state: TaskState
    prepare_kind: str = "teacher"
    metrics: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class TaskChunkView:
    """Immutable view of one capture plus the plugin's current gate evidence."""

    plan: Any
    task_epoch: int
    task_start_sim_time_ns: int
    current_sim_time_ns: int
    latest_capture_seq: int
    state: TaskState
    feedback: Any
    progress: Any


@dataclass(frozen=True, slots=True)
class TaskChunk:
    """One plugin-produced 60x16 target chunk with gate evidence."""

    request_id: str
    task_epoch: int
    capture_seq: int
    observation_time_ns: int
    source_time_ns: int
    robot_profile_hash: str
    plan_digest: str
    chunk_digest: str
    phase: str
    phases: tuple[str, ...]
    phase_codes: tuple[int, ...]
    time_from_observation_ns: tuple[int, ...]
    positions: tuple[tuple[float, ...], ...]
    ik_max_position_error_m: float
    ik_max_orientation_error_rad: float
    gate_state: str
    gate_failure: str
    phase_start_sim_time_ns: int
    phase_wait_s: float


@runtime_checkable
class SimTaskPlugin(Protocol):
    """One task plugin used by the owner, planner and ROS adapter."""

    task_type: str

    def bind_scene(self, scene: object) -> None: ...

    def capacity_requirements(self) -> dict[str, int]: ...

    def load_kinematics(self, scene_config: object) -> object | None: ...

    @property
    def plan_timeout_s(self) -> float: ...

    def segmentation_geoms(self, model: object) -> dict[str, str]: ...

    @property
    def teacher_enabled(self) -> bool: ...

    @property
    def state(self) -> TaskState: ...

    def prepare_snapshot(
        self,
        *,
        scene: object,
        controller: object,
        context: RunContext,
        lease_id: str,
        request_id: str,
        content_digest: str,
        episode_id: str,
        seed: int,
        capture_seq: int,
        sim_time_ns: int,
        now: int,
        reset_seed: int,
    ) -> TaskPrepareSnapshot: ...

    def build_plan(
        self,
        snapshot: TaskPrepareSnapshot,
        *,
        model: object,
        profile: RobotProfile,
        control_config: object,
        kinematics_config: object | None,
        deadline: float,
    ) -> TaskPlan: ...

    def commit_prepare(
        self,
        *,
        scene: object,
        controller: object,
        snapshot: TaskPrepareSnapshot,
        plan: TaskPlan,
        request_id: str,
        content_digest: str,
        context: RunContext,
        lease_id: str,
        now: int,
    ) -> TaskPrepareResult: ...

    def start(
        self,
        *,
        controller: object,
        context: RunContext,
        lease_id: str,
        request_id: str,
        content_digest: str,
        plan_digest: str,
        task_epoch: int,
        episode_id: str,
        sim_time_ns: int,
        wall_ns: int,
        command_id: str,
    ) -> tuple[TaskState, bool]: ...

    def capture_observation(self, scene: object) -> object: ...

    def tick(self, observation: object, wall_ns: int) -> TaskState: ...

    def reset(self, reason: str) -> TaskState: ...

    def fail_control(self, reason: str) -> TaskState: ...

    def on_capture(self, capture: object, observation: object) -> None: ...

    def chunk_view(
        self,
        *,
        context: RunContext,
        task_epoch: int,
        capture_seq: int,
        observation_time_ns: int,
        latest_capture_seq: int,
        capture_times: object,
        current_sim_time_ns: int,
    ) -> TaskChunkView: ...

    def sample_chunk(
        self,
        view: TaskChunkView,
        *,
        request_id: str,
        capture_seq: int,
        observation_time_ns: int,
        profile: RobotProfile,
        control_config: object,
    ) -> TaskChunk: ...

    def fill_chunk_response(self, response: object, chunk: TaskChunk) -> object: ...

    def clear_chunk_cache(self) -> None: ...
