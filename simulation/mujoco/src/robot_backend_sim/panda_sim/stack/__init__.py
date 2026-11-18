"""Stack task plugin: prepare/start/tick/chunk over the generic plugin protocol."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from typing import Any

from robot_contracts.models import ContractError, RobotProfile, RunContext
from robot_contracts.task_state import TaskState

from robot_backend_sim.panda_sim.stack.scene_config import SceneConfig
from robot_backend_sim.common.kinematics.common import extract_object_pose
from robot_backend_sim.common.tasks.base import (
    TaskChunk,
    TaskChunkView,
    TaskPlan,
    TaskPrepareResult,
    TaskPrepareSnapshot,
)
from robot_backend_sim.panda_sim.stack.config import (
    StackTaskConfig,
    StackTaskConfigError,
    load_task_config,
)
from robot_backend_sim.panda_sim.stack.evaluator import (
    ObjectState,
    StackTaskError,
    StackTaskRuntime,
    capture_task_observation,
    make_task_binding,
)
from robot_backend_sim.panda_sim.stack.gate import TeacherFeedback, TeacherProgress
from robot_backend_sim.panda_sim.stack.teacher import (
    TeacherSnapshot,
    build_teacher_plan,
    sample_teacher_chunk,
)

__all__ = ["StackPrepareSnapshot", "StackTaskPlugin"]


@dataclass(frozen=True, slots=True)
class StackPrepareSnapshot:
    """Immutable stack truth captured by the owner before any IK is attempted."""

    context: RunContext
    task_epoch: int
    sim_time_ns: int
    capture_seq: int
    qpos: tuple[float, ...]
    source: ObjectState
    target: ObjectState
    left_positions: tuple[float, ...]
    right_positions: tuple[float, ...]
    left_gripper_opening_m: float
    right_gripper_opening_m: float
    table_top_z_m: float
    seed: int
    controller_command_id: str = ""


@dataclass(frozen=True, slots=True)
class _EvaluationOnlyPlan:
    plan_digest: str
    duration_ns: int
    prepare_kind: str = "evaluation_only"
    key_poses: tuple[object, ...] = ()
    ik_results: tuple[object, ...] = ()
    collision_results: tuple[object, ...] = ()


def _digest(payload: dict[str, object]) -> str:
    text = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(text.encode()).hexdigest()


def _error(reason: str, message: str) -> StackTaskError:
    return StackTaskError(reason, message)


class StackTaskPlugin:
    """The stack task's explicit plugin implementation."""

    task_type = "stack"
    teacher_capability = "panda_stack_api_msgs.teacher_chunk.v1"

    def __init__(self, config: StackTaskConfig, *, task_mode: str | None) -> None:
        self.config = config
        self._task_mode = task_mode
        self._binding = None
        self._runtime: StackTaskRuntime | None = None
        self._teacher_progress: TeacherProgress | None = None
        self._teacher_feedback: dict[int, tuple[TeacherFeedback, object]] = {}
        self._last_observation: object | None = None
        self._prepared_command_id = ""
        self._chunk_cache: dict[str, tuple[str, TaskChunk]] = {}

    @classmethod
    def from_path(
        cls, path: object, *, task_mode: str | None, scene_config: SceneConfig
    ) -> StackTaskPlugin:
        config = load_task_config(path)  # type: ignore[arg-type]
        validate_scene_geometry(config, scene_config)
        return cls(config, task_mode=task_mode)

    @property
    def config_digest(self) -> str:
        return self.config.config_digest

    @property
    def teacher_enabled(self) -> bool:
        return self.config.teacher.enabled and self._task_mode == "teacher"

    @property
    def plan_timeout_s(self) -> float:
        return self.config.teacher.plan_wall_timeout_s

    def load_kinematics(self, scene_config: SceneConfig) -> object | None:
        if not self.config.teacher.enabled:
            return None
        from robot_backend_sim.panda_sim.kinematics_config import load_kinematics_config

        return load_kinematics_config(self.config.teacher.kinematics_config_path, scene_config)

    @property
    def state(self) -> TaskState:
        return self._require_runtime().state

    @property
    def binding(self) -> Any:
        return self._binding

    def capacity_requirements(self) -> dict[str, int]:
        capacity = self.config.capacity
        return {
            "chunk_history": capacity.max_episode_chunks,
            "command_history": capacity.max_episode_chunks,
            "event_queue": capacity.expected_events,
            "accepted_reference_queue": capacity.expected_references,
        }

    def segmentation_geoms(self, model: object) -> dict[str, str]:
        del model
        return {
            "source": f"{self.config.source_object}_geom",
            "target": f"{self.config.target_object}_geom",
            "table": "table_top",
        }

    def bind_scene(self, scene: object) -> None:
        self._binding = make_task_binding(
            scene.model,  # type: ignore[attr-defined]
            self.config,
            table_top_z_m=scene.config.table.top_z,  # type: ignore[attr-defined]
        )
        self._runtime = StackTaskRuntime(self.config, self._binding)

    def _require_runtime(self) -> StackTaskRuntime:
        if self._runtime is None or self._binding is None:
            raise _error("TASK_NOT_READY", "task plugin has no bound scene")
        return self._runtime

    def prepare_snapshot(
        self, *, scene: object, controller: object, context: RunContext, lease_id: str,
        request_id: str, content_digest: str, episode_id: str, seed: int,
        capture_seq: int, sim_time_ns: int, now: int, reset_seed: int,
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
            return TaskPrepareResult(
                task_type=self.task_type, task_epoch=fact.state.task_epoch,
                task_config_digest=self.config.config_digest, plan_digest=fact.plan_digest,
                idempotent=True, state=fact.state, prepare_kind=fact.prepare_kind,
                metrics={
                    "key_pose_count": fact.key_pose_count,
                    "collision_segment_count": fact.collision_segment_count,
                    "ik_max_position_error_m": fact.ik_max_position_error_m,
                    "ik_max_orientation_error_rad": fact.ik_max_orientation_error_rad,
                    "planned_duration_s": fact.plan_duration_s,
                },
            )
        runtime.require_prepare_slot()
        qpos = tuple(float(value) for value in scene.data.qpos)  # type: ignore[attr-defined]
        binding = self._binding
        source = extract_object_pose(qpos, binding.source_qpos_address, self.config.source_object)
        target = extract_object_pose(qpos, binding.target_qpos_address, self.config.target_object)
        indices = scene.indices  # type: ignore[attr-defined]
        left = tuple(qpos[address] for address in indices.robot.arm_joint_qpos["left"])
        right = tuple(qpos[address] for address in indices.robot.arm_joint_qpos["right"])
        return StackPrepareSnapshot(
            context=context, task_epoch=runtime.task_epoch + 1, sim_time_ns=sim_time_ns,
            capture_seq=capture_seq, qpos=qpos, source=source, target=target,
            left_positions=left, right_positions=right,
            left_gripper_opening_m=float(
                sum(qpos[address] for address in indices.robot.finger_joint_qpos["left"])
            ),
            right_gripper_opening_m=float(
                sum(qpos[address] for address in indices.robot.finger_joint_qpos["right"])
            ),
            table_top_z_m=float(scene.config.table.top_z),  # type: ignore[attr-defined]
            seed=seed,
            controller_command_id=(
                "" if controller.current_status() is None  # type: ignore[attr-defined]
                else controller.current_status().command_id  # type: ignore[attr-defined]
            ),
        )

    def build_plan(
        self, snapshot: TaskPrepareSnapshot, *, model: object, profile: RobotProfile,
        control_config: object, kinematics_config: object | None, deadline: float,
    ) -> TaskPlan:
        if not self.teacher_enabled:
            plan = _EvaluationOnlyPlan(
                plan_digest=_digest(
                    {
                        "kind": "evaluation_only",
                        "task_config_digest": self.config.config_digest,
                        "context": snapshot.context.__dict__,
                        "task_epoch": snapshot.task_epoch,
                        "source": snapshot.source.center,  # type: ignore[attr-defined]
                        "target": snapshot.target.center,  # type: ignore[attr-defined]
                    }
                ),
                duration_ns=round(self.config.success.max_task_sim_duration_s * 1e9),
            )
            return TaskPlan(plan, plan.plan_digest, plan.duration_ns, "evaluation_only")
        if model is None or kinematics_config is None:
            raise _error("INVALID_TASK_SNAPSHOT", "scene model is unavailable")
        teacher_snapshot = TeacherSnapshot(
            context=snapshot.context, task_epoch=snapshot.task_epoch,
            sim_time_ns=snapshot.sim_time_ns, qpos=snapshot.qpos,
            source=snapshot.source, target=snapshot.target,  # type: ignore[arg-type]
            source_half_extents_m=self.config.source_half_extents_m,
            target_half_extents_m=self.config.target_half_extents_m,
            left_positions=snapshot.left_positions,
            right_positions=snapshot.right_positions,
            left_gripper_opening_m=snapshot.left_gripper_opening_m,
            right_gripper_opening_m=snapshot.right_gripper_opening_m,
            table_top_z_m=snapshot.table_top_z_m,
        )
        plan = build_teacher_plan(
            model, teacher_snapshot, profile, control_config,  # type: ignore[arg-type]
            kinematics_config, self.config, deadline=deadline,  # type: ignore[arg-type]
        )
        return TaskPlan(plan, plan.plan_digest, plan.duration_ns, "teacher")

    def commit_prepare(
        self, *, scene: object, controller: object, snapshot: TaskPrepareSnapshot,
        plan: TaskPlan, request_id: str, content_digest: str, context: RunContext,
        lease_id: str, now: int,
    ) -> TaskPrepareResult:
        runtime = self._require_runtime()
        if not controller.lease_held(context, lease_id, now):  # type: ignore[attr-defined]
            raise _error("LEASE_EXPIRED", "prepare lease expired before commit")
        if context != controller.context:  # type: ignore[attr-defined]
            raise _error("STALE_CONTEXT", "prepare context changed before commit")
        if runtime.state.phase != "IDLE" or runtime.task_epoch != snapshot.task_epoch - 1:
            raise _error("STALE_TASK_PREPARE", "task generation changed before commit")
        status = controller.current_status()  # type: ignore[attr-defined]
        if ("" if status is None else status.command_id) != snapshot.controller_command_id:
            raise _error(
                "STALE_TASK_PREPARE", "another motion ran while task planning was pending"
            )
        if not controller.is_quiescent():  # type: ignore[attr-defined]
            raise _error("TASK_NOT_QUIESCENT", "robot moved before prepare commit")
        binding = self._binding
        current_source = tuple(
            float(value)
            for value in scene.data.qpos[  # type: ignore[attr-defined]
                binding.source_qpos_address : binding.source_qpos_address + 7
            ]
        )
        current_target = tuple(
            float(value)
            for value in scene.data.qpos[  # type: ignore[attr-defined]
                binding.target_qpos_address : binding.target_qpos_address + 7
            ]
        )
        snapshot_source = tuple(snapshot.qpos[binding.source_qpos_address :][:7])
        snapshot_target = tuple(snapshot.qpos[binding.target_qpos_address :][:7])
        if _moved(current_source, snapshot_source) or _moved(current_target, snapshot_target):
            raise _error("STALE_TASK_PREPARE", "scene objects moved between snapshot and commit")
        ik_results = getattr(plan.plan, "ik_results", ())
        collision_results = getattr(plan.plan, "collision_results", ())
        key_poses = getattr(plan.plan, "key_poses", ())
        max_position = max(
            (float(result.position_error_m) for result in ik_results), default=0.0
        )
        max_orientation = max(
            (float(result.orientation_error_rad) for result in ik_results), default=0.0
        )
        self._teacher_progress = None
        self._teacher_feedback.clear()
        self._last_observation = None
        outcome = runtime.commit_prepare(
            request_id=request_id, content_digest=content_digest, context=context,
            plan=plan.plan, plan_digest=plan.plan_digest, plan_duration_ns=plan.duration_ns,
            key_pose_count=len(key_poses), collision_segment_count=len(collision_results),
            ik_max_position_error_m=max_position,
            ik_max_orientation_error_rad=max_orientation, prepare_kind=plan.prepare_kind,
        )
        self._prepared_command_id = snapshot.controller_command_id
        self.clear_chunk_cache()
        return TaskPrepareResult(
            task_type=self.task_type, task_epoch=outcome.fact.state.task_epoch,
            task_config_digest=self.config.config_digest, plan_digest=plan.plan_digest,
            idempotent=outcome.idempotent, state=outcome.fact.state,
            prepare_kind=plan.prepare_kind,
            metrics={
                "key_pose_count": len(key_poses),
                "collision_segment_count": len(collision_results),
                "ik_max_position_error_m": max_position,
                "ik_max_orientation_error_rad": max_orientation,
                "planned_duration_s": plan.duration_ns / 1e9,
            },
        )

    def start(
        self, *, controller: object, context: RunContext, lease_id: str, request_id: str,
        content_digest: str, plan_digest: str, task_epoch: int, episode_id: str,
        sim_time_ns: int, wall_ns: int, command_id: str,
    ) -> tuple[TaskState, bool]:
        runtime = self._require_runtime()
        if not controller.lease_held(context, lease_id, wall_ns):  # type: ignore[attr-defined]
            raise _error("LEASE_EXPIRED", "start does not own the current lease")
        if context != controller.context or episode_id != context.episode_id:  # type: ignore[attr-defined]
            raise _error("STALE_CONTEXT", "start context/episode differs")
        if (
            runtime.state.phase == "PREPARED"
            and command_id != self._prepared_command_id
        ):
            raise _error("STALE_TASK_PREPARE", "robot command changed after prepare")
        state, idempotent = runtime.start(
            request_id=request_id, content_digest=content_digest, context=context,
            plan_digest=plan_digest, task_epoch=task_epoch, sim_time_ns=sim_time_ns,
            wall_ns=wall_ns,
        )
        if (
            not idempotent and self.teacher_enabled and runtime.plan is not None
            and hasattr(runtime.plan, "segments")
        ):
            self._teacher_progress = TeacherProgress(runtime.plan, self.config)  # type: ignore[arg-type]
            self._teacher_feedback.clear()
        return state, idempotent

    def capture_observation(self, scene: object) -> object:
        self._require_runtime()
        return capture_task_observation(scene, self._binding, self.config)

    def tick(self, observation: object, wall_ns: int) -> TaskState:
        return self._require_runtime().tick(observation, wall_ns)  # type: ignore[arg-type]

    def reset(self, reason: str) -> TaskState:
        runtime = self._require_runtime()
        self._teacher_progress = None
        self._teacher_feedback.clear()
        self._last_observation = None
        self.clear_chunk_cache()
        return runtime.reset(reason)

    def fail_control(self, reason: str) -> TaskState:
        return self._require_runtime().fail_control(reason)

    def on_capture(self, capture: object, observation: object) -> None:
        if (
            self._teacher_progress is None or self._runtime is None
            or observation is None
        ):
            return
        try:
            feedback = self._teacher_progress.feedback_for(capture, observation)  # type: ignore[arg-type]
            fact = self._teacher_progress.update(feedback, self._runtime.state.phase)
        except BaseException as error:  # noqa: BLE001 - gate evidence is required
            raise ContractError("TASK_GATE_FAILED", f"{type(error).__name__}: {error}") from error
        if len(self._teacher_feedback) >= 2048:
            self._teacher_feedback.clear()
        self._teacher_feedback[capture.capture_seq] = (feedback, fact)  # type: ignore[attr-defined]

    def chunk_view(
        self, *, context: RunContext, task_epoch: int, capture_seq: int,
        observation_time_ns: int, latest_capture_seq: int, capture_times: object,
        current_sim_time_ns: int, handoff_time_ns: int = 0,
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
        if plan is None:
            raise _error("TASK_NOT_PREPARED", "task has no immutable plan")
        feedback = self._teacher_feedback.get(capture_seq)
        if feedback is None or self._teacher_progress is None:
            raise _error(
                "CAPTURE_FEEDBACK_MISSING",
                "no immutable teacher feedback exists for this capture",
            )
        return TaskChunkView(
            plan=plan, task_epoch=state.task_epoch, task_start_sim_time_ns=start_ns,
            current_sim_time_ns=current_sim_time_ns, latest_capture_seq=latest_capture_seq,
            state=state, feedback=feedback[0], progress=feedback[1],
        )

    def sample_chunk(
        self, view: TaskChunkView, *, request_id: str, capture_seq: int,
        observation_time_ns: int, profile: RobotProfile, control_config: object,
    ) -> TaskChunk:
        content_digest = _digest(
            {
                "kind": "chunk", "context": view.state.context.__dict__
                if view.state.context is not None else None,
                "task_epoch": view.task_epoch, "request_id": request_id,
                "capture_seq": capture_seq, "observation_time_ns": observation_time_ns,
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
        chunk = sample_teacher_chunk(
            view.plan, gate=view.progress, feedback=view.feedback,
            request_id=request_id, task_epoch=view.task_epoch, capture_seq=capture_seq,
            observation_time_ns=observation_time_ns, chunk_size=self.config.teacher.chunk_size,
            period_ns=self.config.teacher.action_period_ns, profile=profile,
            control_config=control_config,  # type: ignore[arg-type]
        )
        self._chunk_cache[request_id] = (content_digest, chunk)
        return chunk

    def fill_chunk_response(self, response: object, chunk: TaskChunk) -> object:
        from robot_backend_sim.panda_sim.stack.ros_conv import fill_teacher_chunk_response

        return fill_teacher_chunk_response(response, chunk)

    def clear_chunk_cache(self) -> None:
        self._chunk_cache.clear()

    def clear_teacher_state(self) -> None:
        self._teacher_progress = None
        self._teacher_feedback.clear()
        self._last_observation = None


def _moved(current: tuple[float, ...], original: tuple[float, ...]) -> bool:
    if any(abs(a - b) > 1e-4 for a, b in zip(current[:3], original[:3], strict=True)):
        return True
    dot = abs(sum(a * b for a, b in zip(current[3:7], original[3:7], strict=True)))
    return abs(dot - 1.0) > 1e-6


def validate_scene_geometry(config: StackTaskConfig, scene_config: Any) -> None:
    """Require the task names/half extents to match the frozen scene exactly."""
    objects = {obj.name: obj for obj in scene_config.objects}
    for name, expected in (
        (config.source_object, config.source_half_extents_m),
        (config.target_object, config.target_half_extents_m),
    ):
        obj = objects.get(name)
        if obj is None:
            raise StackTaskConfigError(config.source, "objects",
                                       f"scene has no configured object {name!r}")
        if any(abs(float(a) - float(b)) > 1e-12
               for a, b in zip(obj.half_extents, expected, strict=True)):
            raise StackTaskConfigError(config.source, "objects",
                                       f"{name!r} half extents do not match task")
