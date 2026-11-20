"""Marvin stack task plugin, configuration entry points and geometry validation."""

from __future__ import annotations

import hashlib
import json
from collections import OrderedDict
from typing import Any

from robot_contracts.models import ContractError, RobotProfile, RunContext
from robot_contracts.task_state import TaskState

from robot_backend_sim.common.tasks.base import (
    CAPTURE_HISTORY_LIMIT,
    TaskChunk,
    TaskChunkView,
    TaskPlan,
    TaskPrepareResult,
    TaskPrepareSnapshot,
)
from robot_backend_sim.marvin_sim.stack.config import (
    StackTaskConfig,
    load_task_config,
    validate_scene_geometry,
)
from robot_backend_sim.marvin_sim.stack.evaluator import (
    StackTaskRuntime,
    TaskBinding,
    capture_task_observation,
    make_task_binding,
)
from robot_backend_sim.marvin_sim.stack.gate import StackGateFact, TeacherFeedback
from robot_backend_sim.marvin_sim.stack.kinematics_config import (
    KinematicsCheckConfig,
    load_kinematics_config,
)
from robot_backend_sim.marvin_sim.stack.teacher import (
    StackPlan,
    TeacherSnapshot,
    build_teacher_plan,
    sample_teacher_chunk,
)

__all__ = ["StackTaskPlugin", "TEACHER_CAPABILITY"]

TEACHER_CAPABILITY = "marvin_stack_api_msgs.teacher_chunk"
_COMMAND_HISTORY = 64


def _digest(payload: dict[str, object]) -> str:
    text = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(text.encode()).hexdigest()


def _error(reason: str, message: str) -> ContractError:
    return ContractError(reason, message)


class StackTaskPlugin:
    """Marvin single-arm (right) cube stack plugin implementing SimTaskPlugin."""

    task_type = "stack"
    teacher_capability = TEACHER_CAPABILITY

    def __init__(self, config: StackTaskConfig, *, task_mode: str | None) -> None:
        self.config = config
        self._task_mode = task_mode
        self._scene_config: Any = None
        self._binding: TaskBinding | None = None
        self._runtime: StackTaskRuntime | None = None
        self._teacher_feedback: OrderedDict[int, tuple[TeacherFeedback, StackGateFact]] = (
            OrderedDict()
        )
        self._step_motion_start: dict[int, int] = {}
        self._prepared_command_id = ""
        self._chunk_cache: dict[str, tuple[str, TaskChunk]] = {}

    @classmethod
    def from_path(
        cls, path: object, *, task_mode: str | None, scene_config: object
    ) -> StackTaskPlugin:
        config = load_task_config(path)  # type: ignore[arg-type]
        if getattr(scene_config, "task", None) is None or scene_config.task.environment != "stack":  # type: ignore[attr-defined]
            raise _error("SCENE_MISMATCH", "stack plugin requires a Marvin stack scene")
        validate_scene_geometry(config, scene_config)  # type: ignore[arg-type]
        plugin = cls(config, task_mode=task_mode)
        plugin._scene_config = scene_config
        return plugin

    @property
    def config_digest(self) -> str:
        return self.config.config_digest

    @property
    def teacher_enabled(self) -> bool:
        return self.config.teacher.enabled and self._task_mode == "teacher"

    @property
    def plan_timeout_s(self) -> float:
        return self.config.teacher.plan_wall_timeout_s

    @property
    def state(self) -> TaskState:
        return self._require_runtime().state

    @property
    def binding(self) -> TaskBinding | None:
        return self._binding

    def capacity_requirements(self) -> dict[str, int]:
        capacity = self.config.capacity
        return {
            "chunk_history": capacity.max_episode_chunks,
            "command_history": _COMMAND_HISTORY,
            "event_queue": capacity.expected_events,
            "accepted_reference_queue": capacity.expected_references,
        }

    def segmentation_geoms(self, model: object) -> dict[str, str]:
        del model
        return {
            "source": "source_geom",
            "target": "target_geom",
            "table": "table_top",
        }

    def load_kinematics(self, scene_config: object) -> KinematicsCheckConfig:
        return load_kinematics_config(self.config.planning_config, scene_config)  # type: ignore[arg-type]

    def bind_scene(self, scene: object) -> None:
        config = self._scene_config
        if config is None:
            raise _error("SCENE_NOT_READY", "stack plugin has no scene config")
        table_top_z_m = float(config.robot.base_position[2] + config.task.table.top_z)
        binding = make_task_binding(
            scene.model, self.config, table_top_z_m=table_top_z_m  # type: ignore[attr-defined]
        )
        self._binding = binding
        self._runtime = StackTaskRuntime(self.config, binding)

    def _require_runtime(self) -> StackTaskRuntime:
        if self._runtime is None:
            raise _error("TASK_NOT_READY", "task plugin has no bound scene")
        return self._runtime

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
    ) -> TaskPrepareSnapshot:
        runtime = self._require_runtime()
        if not isinstance(request_id, str) or not request_id:
            raise _error("INVALID_REQUEST_ID", "request_id must be non-empty")
        if not isinstance(content_digest, str) or not content_digest:
            raise _error("INVALID_REQUEST_ID", "content digest must be non-empty")
        if not isinstance(seed, int) or isinstance(seed, bool) or seed < 0:
            raise _error("INVALID_SEED", "seed must be a non-negative integer")
        if not controller.lease_held(context, lease_id, now):  # type: ignore[attr-defined]
            raise _error("LEASE_EXPIRED", "prepare does not own the current lease")
        if context != controller.context or episode_id != context.episode_id:  # type: ignore[attr-defined]
            raise _error("STALE_CONTEXT", "prepare context/episode differs")
        if runtime.state.phase == "RUNNING":
            raise _error("TASK_BUSY", "a stack task is already running")
        if not controller.is_quiescent():  # type: ignore[attr-defined]
            raise _error("TASK_NOT_QUIESCENT", "prepare requires a still robot")
        if seed != reset_seed:
            raise _error(
                "SEED_MISMATCH",
                f"prepare seed {seed} differs from the last Scene.reset seed {reset_seed}",
            )
        cached = runtime.lookup_prepare(request_id, content_digest)
        if cached is not None:
            fact = cached.fact
            return TaskPrepareResult(  # type: ignore[return-value]
                task_type=self.task_type,
                task_epoch=fact.state.task_epoch,
                task_config_digest=self.config.config_digest,
                plan_digest=fact.plan_digest,
                idempotent=True,
                state=fact.state,
                prepare_kind=fact.prepare_kind,
                metrics={
                    "step_count": fact.step_count,
                    "ik_max_position_error_m": fact.ik_max_position_error_m,
                    "ik_max_orientation_error_rad": fact.ik_max_orientation_error_rad,
                    "planned_duration_s": fact.plan_duration_ns / 1e9,
                },
            )
        runtime.require_prepare_slot()
        status = controller.current_status()  # type: ignore[attr-defined]
        return TeacherSnapshot(
            context=context,
            task_epoch=runtime.task_epoch + 1,
            sim_time_ns=sim_time_ns,
            capture_seq=capture_seq,
            qpos=tuple(float(value) for value in scene.data.qpos),  # type: ignore[attr-defined]
            seed=seed,
            controller_command_id="" if status is None else status.command_id,
        )

    def build_plan(
        self,
        snapshot: TaskPrepareSnapshot,
        *,
        model: object,
        profile: RobotProfile,
        control_config: object,
        kinematics_config: object | None,
        deadline: float,
    ) -> TaskPlan:
        snap = snapshot
        assert isinstance(snap, TeacherSnapshot)
        if not self.teacher_enabled:
            duration_ns = round(self.config.success.stable_duration_s * 1e9)
            plan_digest = _digest(
                {
                    "kind": "evaluation_only",
                    "task_config_digest": self.config.config_digest,
                    "context": snap.context.__dict__,
                    "task_epoch": snap.task_epoch,
                }
            )
            return TaskPlan(object(), plan_digest, duration_ns, "evaluation_only")
        if kinematics_config is None:
            raise _error("TASK_NOT_READY", "stack teacher requires kinematics_config")
        plan = build_teacher_plan(
            model,
            snap,
            profile,
            control_config,  # type: ignore[arg-type]
            kinematics_config,  # type: ignore[arg-type]
            self.config,
            scene_config=self._scene_config,
            deadline=deadline,
        )
        return TaskPlan(plan, plan.plan_digest, plan.duration_ns, "teacher")

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
    ) -> TaskPrepareResult:
        runtime = self._require_runtime()
        snap = snapshot
        assert isinstance(snap, TeacherSnapshot)
        if not controller.lease_held(context, lease_id, now):  # type: ignore[attr-defined]
            raise _error("LEASE_EXPIRED", "prepare lease expired before commit")
        if context != controller.context:  # type: ignore[attr-defined]
            raise _error("STALE_CONTEXT", "prepare context changed before commit")
        if runtime.state.phase != "IDLE" or runtime.task_epoch != snap.task_epoch - 1:
            raise _error("STALE_TASK_PREPARE", "task generation changed before commit")
        status = controller.current_status()  # type: ignore[attr-defined]
        if ("" if status is None else status.command_id) != snap.controller_command_id:
            raise _error(
                "STALE_TASK_PREPARE", "another motion ran while task planning was pending"
            )
        if not controller.is_quiescent():  # type: ignore[attr-defined]
            raise _error("TASK_NOT_QUIESCENT", "robot moved before prepare commit")
        binding = self._binding
        assert binding is not None
        for addr in (binding.source_qpos_address, binding.target_qpos_address):
            current_obj = tuple(float(v) for v in scene.data.qpos[addr : addr + 7])  # type: ignore[attr-defined]
            snap_obj = tuple(snap.qpos[addr : addr + 7])
            if _moved(current_obj, snap_obj):
                raise _error(
                    "STALE_TASK_PREPARE", "scene objects moved between snapshot and commit"
                )
        self._teacher_feedback.clear()
        self._step_motion_start.clear()
        self.clear_chunk_cache()
        stack_plan = plan.plan if isinstance(plan.plan, StackPlan) else None
        outcome = runtime.commit_prepare(
            request_id=request_id,
            content_digest=content_digest,
            context=context,
            plan=plan.plan,
            plan_digest=plan.plan_digest,
            plan_duration_ns=plan.duration_ns,
            step_count=len(stack_plan.segments) if stack_plan is not None else 0,
            ik_max_position_error_m=(
                stack_plan.ik_max_position_error_m if stack_plan is not None else 0.0
            ),
            ik_max_orientation_error_rad=(
                stack_plan.ik_max_orientation_error_rad if stack_plan is not None else 0.0
            ),
            prepare_kind=plan.prepare_kind,
        )
        self._prepared_command_id = snap.controller_command_id
        return TaskPrepareResult(
            task_type=self.task_type,
            task_epoch=outcome.fact.state.task_epoch,
            task_config_digest=self.config.config_digest,
            plan_digest=plan.plan_digest,
            idempotent=outcome.idempotent,
            state=outcome.fact.state,
            prepare_kind=plan.prepare_kind,
            metrics={
                "step_count": outcome.fact.step_count,
                "ik_max_position_error_m": outcome.fact.ik_max_position_error_m,
                "ik_max_orientation_error_rad": outcome.fact.ik_max_orientation_error_rad,
                "planned_duration_s": plan.duration_ns / 1e9,
            },
        )

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
    ) -> tuple[TaskState, bool]:
        runtime = self._require_runtime()
        if not controller.lease_held(context, lease_id, wall_ns):  # type: ignore[attr-defined]
            raise _error("LEASE_EXPIRED", "start does not own the current lease")
        if context != controller.context or episode_id != context.episode_id:  # type: ignore[attr-defined]
            raise _error("STALE_CONTEXT", "start context/episode differs")
        if runtime.state.phase == "PREPARED" and command_id != self._prepared_command_id:
            raise _error("STALE_TASK_PREPARE", "robot command changed after prepare")
        state, idempotent = runtime.start(
            request_id=request_id,
            content_digest=content_digest,
            context=context,
            plan_digest=plan_digest,
            task_epoch=task_epoch,
            sim_time_ns=sim_time_ns,
            wall_ns=wall_ns,
        )
        if not idempotent and self.teacher_enabled:
            self._teacher_feedback.clear()
            self._step_motion_start.clear()
            self.clear_chunk_cache()
        return state, idempotent

    def capture_observation(self, scene: object) -> object:
        binding = self._binding
        if binding is None:
            raise _error("TASK_NOT_READY", "task plugin has no bound scene")
        return capture_task_observation(scene, binding, self.config)

    def tick(self, observation: object, wall_ns: int) -> TaskState:
        return self._require_runtime().tick(observation, wall_ns)  # type: ignore[arg-type]

    def reset(self, reason: str) -> TaskState:
        runtime = self._require_runtime()
        self._teacher_feedback.clear()
        self._step_motion_start.clear()
        self.clear_chunk_cache()
        return runtime.reset(reason)

    def fail_control(self, reason: str) -> TaskState:
        return self._require_runtime().fail_control(reason)

    def on_capture(self, capture: object, observation: object) -> None:
        if not self.teacher_enabled or self._runtime is None or observation is None:
            return
        gate = self._runtime.gate_fact()
        feedback = TeacherFeedback(
            capture_seq=int(capture.capture_seq),  # type: ignore[attr-defined]
            sim_time_ns=int(capture.sim_time_ns),  # type: ignore[attr-defined]
            right_positions=observation.right_positions,  # type: ignore[attr-defined]
            right_velocities=observation.right_velocities,  # type: ignore[attr-defined]
            right_gripper_opening_m=observation.right_gripper_opening_m,  # type: ignore[attr-defined]
            right_gripper_velocity_m_s=observation.right_gripper_velocity_m_s,  # type: ignore[attr-defined]
            right_finger_source_contacts=observation.right_finger_source_contacts,  # type: ignore[attr-defined]
            right_finger_source_min_force_n=observation.right_finger_source_min_force_n,  # type: ignore[attr-defined]
            right_finger_source_max_speed_m_s=observation.right_finger_source_max_speed_m_s,  # type: ignore[attr-defined]
            source_center=observation.source.center,  # type: ignore[attr-defined]
            right_tcp_distance_m=observation.right_tcp_to_source_m,  # type: ignore[attr-defined]
        )
        self._teacher_feedback[feedback.capture_seq] = (feedback, gate)
        if len(self._teacher_feedback) > CAPTURE_HISTORY_LIMIT:
            self._teacher_feedback.popitem(last=False)

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
        handoff_time_ns: int = 0,
    ) -> TaskChunkView:
        runtime = self._require_runtime()
        state = runtime.state
        if state.phase != "RUNNING":
            raise _error("TASK_NOT_RUNNING", "teacher chunks require a running task")
        if state.context != context or task_epoch != state.task_epoch:
            raise _error("STALE_TASK_CONTEXT", "teacher chunk task epoch differs")
        if isinstance(capture_seq, bool) or not isinstance(capture_seq, int) or capture_seq < 1:
            raise _error("INVALID_CAPTURE", "capture_seq must be positive")
        if capture_seq > latest_capture_seq:
            raise _error("CAPTURE_NOT_RETAINED", "capture_seq is not retained yet")
        if observation_time_ns > current_sim_time_ns:
            raise _error(
                "TEACHER_FUTURE_OBSERVATION",
                "observation time is later than the current simulation time",
            )
        retained_time = next(
            (sim_ns for seq, sim_ns in reversed(capture_times) if seq == capture_seq),  # type: ignore[attr-defined]
            None,
        )
        if retained_time is None:
            raise _error("CAPTURE_NOT_RETAINED", "capture_seq is no longer retained")
        if observation_time_ns != retained_time:
            raise _error(
                "TEACHER_OBSERVATION_MISMATCH",
                "observation time does not belong to the retained capture",
            )
        start_ns = int(state.metric("task_start_sim_time_ns", 0))
        if observation_time_ns < start_ns:
            raise _error("TEACHER_BEFORE_START", "observation precedes task start")
        plan = runtime.plan
        if plan is None or not isinstance(plan, StackPlan):
            raise _error("TASK_NOT_PREPARED", "task has no immutable stack plan")
        retained_feedback = self._teacher_feedback.get(capture_seq)
        if retained_feedback is None:
            raise _error(
                "CAPTURE_FEEDBACK_MISSING",
                "no immutable teacher feedback exists for this capture",
            )
        feedback, gate = retained_feedback
        return TaskChunkView(
            plan=plan,
            task_epoch=state.task_epoch,
            task_start_sim_time_ns=start_ns,
            current_sim_time_ns=current_sim_time_ns,
            latest_capture_seq=latest_capture_seq,
            state=state,
            feedback=feedback,
            progress=gate,
            handoff_time_ns=handoff_time_ns,
        )

    def sample_chunk(
        self,
        view: TaskChunkView,
        *,
        request_id: str,
        capture_seq: int,
        observation_time_ns: int,
        profile: RobotProfile,
        control_config: object,
    ) -> TaskChunk:
        content_digest = _digest(
            {
                "kind": "chunk",
                "context": view.state.context.__dict__
                if view.state.context is not None
                else None,
                "task_epoch": view.task_epoch,
                "request_id": request_id,
                "capture_seq": capture_seq,
                "observation_time_ns": observation_time_ns,
                "handoff_time_ns": view.handoff_time_ns,
                "plan_digest": view.plan.plan_digest,
            }
        )
        cached = self._chunk_cache.get(request_id)
        if cached is not None:
            if cached[0] != content_digest:
                raise _error(
                    "REQUEST_ID_CONFLICT", "same teacher request_id has different content"
                )
            return cached[1]
        if len(self._chunk_cache) >= self.config.teacher.chunk_cache_entries:
            raise _error(
                "TEACHER_CACHE_FULL",
                "teacher idempotency cache is full; refusing to drop an audited chunk",
            )
        step_index = view.progress.step_index
        motion_start = self._step_motion_start.get(step_index)
        if motion_start is None:
            motion_start = (
                view.handoff_time_ns or view.progress.active_start_sim_time_ns
            )
            self._step_motion_start[step_index] = motion_start
            if self._runtime is not None and self._runtime.progress is not None:
                self._runtime.progress.bind_motion_start(step_index, motion_start)
        chunk = sample_teacher_chunk(
            view.plan,
            gate=view.progress,
            feedback=view.feedback,
            request_id=request_id,
            task_epoch=view.task_epoch,
            capture_seq=capture_seq,
            observation_time_ns=observation_time_ns,
            chunk_size=self.config.teacher.chunk_size,
            period_ns=self.config.teacher.action_period_ns,
            profile=profile,
            control_config=control_config,  # type: ignore[arg-type]
            start_sim_time_ns=motion_start,
        )
        self._chunk_cache[request_id] = (content_digest, chunk)
        return chunk

    def fill_chunk_response(self, response: object, chunk: TaskChunk) -> object:
        from robot_backend_sim.marvin_sim.stack.ros_conv import fill_teacher_chunk_response

        return fill_teacher_chunk_response(response, chunk)

    def clear_chunk_cache(self) -> None:
        self._chunk_cache.clear()


def _moved(current: tuple[float, ...], original: tuple[float, ...]) -> bool:
    if any(abs(a - b) > 1e-4 for a, b in zip(current[:3], original[:3], strict=True)):
        return True
    dot = abs(sum(a * b for a, b in zip(current[3:7], original[3:7], strict=True)))
    return abs(dot - 1.0) > 1e-6
