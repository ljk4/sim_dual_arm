"""Marvin stack task plugin, configuration entry points and geometry validation."""

from __future__ import annotations

from typing import Any

from robot_contracts.io import canonical_sha256
from robot_contracts.models import ContractError, RobotProfile

from robot_backend_sim.common.tasks.base import (
    CAPTURE_HISTORY_LIMIT,
    TaskChunk,
    TaskChunkView,
    TaskPlan,
    TaskPrepareSnapshot,
)
from robot_backend_sim.common.tasks.plugin_base import BaseTeacherTaskPlugin
from robot_backend_sim.marvin_sim.robot_config import MARVIN_TEACHER_CAPABILITY
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
from robot_backend_sim.marvin_sim.stack.gate import TeacherFeedback
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

TEACHER_CAPABILITY = MARVIN_TEACHER_CAPABILITY


def _error(reason: str, message: str) -> ContractError:
    return ContractError(reason, message)


class StackTaskPlugin(BaseTeacherTaskPlugin):
    """Marvin single-arm (right) cube stack plugin implementing SimTaskPlugin."""

    task_type = "stack"
    teacher_capability = TEACHER_CAPABILITY

    def __init__(self, config: StackTaskConfig, *, task_mode: str | None) -> None:
        super().__init__(config, task_mode=task_mode)
        self._binding: TaskBinding | None = None
        self._runtime: StackTaskRuntime | None = None

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
    def binding(self) -> TaskBinding | None:
        return self._binding

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

    def _pose_check_addresses(self) -> tuple[int, ...]:
        assert self._binding is not None
        return (self._binding.source_qpos_address, self._binding.target_qpos_address)

    def _validate_plan_for_commit(self, plan_obj: object) -> tuple[int, float, float]:
        stack_plan = plan_obj if isinstance(plan_obj, StackPlan) else None
        if stack_plan is None:
            return (0, 0.0, 0.0)
        return (
            len(stack_plan.segments),
            stack_plan.ik_max_position_error_m,
            stack_plan.ik_max_orientation_error_rad,
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
            plan_digest = canonical_sha256(
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

    def capture_observation(self, scene: object) -> object:
        binding = self._binding
        if binding is None:
            raise _error("TASK_NOT_READY", "task plugin has no bound scene")
        return capture_task_observation(scene, binding, self.config)

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

    def _validate_prepared_plan(self, plan: Any) -> None:
        if plan is None or not isinstance(plan, StackPlan):
            raise _error("TASK_NOT_PREPARED", "task has no immutable stack plan")

    def _on_step_motion_start(self, step_index: int, motion_start: int) -> None:
        if self._runtime is not None and self._runtime.progress is not None:
            self._runtime.progress.bind_motion_start(step_index, motion_start)

    def _sample_step_chunk(
        self,
        view: TaskChunkView,
        *,
        request_id: str,
        capture_seq: int,
        observation_time_ns: int,
        profile: RobotProfile,
        control_config: object,
        motion_start: int,
    ) -> TaskChunk:
        return sample_teacher_chunk(
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
