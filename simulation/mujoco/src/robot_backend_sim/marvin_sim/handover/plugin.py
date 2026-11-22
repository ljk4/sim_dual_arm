"""Handover task plugin: lifecycle, measured gates and gated Teacher chunks."""

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
from robot_backend_sim.marvin_sim.handover.evaluator import HandoverTaskRuntime
from robot_backend_sim.marvin_sim.handover.gate import HandoverFeedback
from robot_backend_sim.marvin_sim.handover.observation import (
    HandoverBinding,
    capture_handover_observation,
    make_handover_binding,
)
from robot_backend_sim.marvin_sim.handover.plan import HandoverLayout, layout_from_config
from robot_backend_sim.marvin_sim.handover.task_config import (
    HandoverTaskConfig,
    load_handover_task_config,
)
from robot_backend_sim.marvin_sim.handover.teacher import (
    HandoverPlan,
    HandoverSnapshot,
    build_handover_plan,
    sample_handover_chunk,
)
from robot_backend_sim.marvin_sim.robot_config import MARVIN_TEACHER_CAPABILITY

__all__ = ["HandoverTaskPlugin", "TEACHER_CAPABILITY"]

TEACHER_CAPABILITY = MARVIN_TEACHER_CAPABILITY


def _error(reason: str, message: str) -> ContractError:
    return ContractError(reason, message)


class HandoverTaskPlugin(BaseTeacherTaskPlugin):
    """The handover task's explicit plugin implementation."""

    task_type = "handover"
    teacher_capability = TEACHER_CAPABILITY

    def __init__(self, config: HandoverTaskConfig, *, task_mode: str | None) -> None:
        super().__init__(config, task_mode=task_mode)
        self._binding: HandoverBinding | None = None
        self._layout: HandoverLayout | None = None
        self._runtime: HandoverTaskRuntime | None = None
        self._robot: Any = None
        self._robot_geometry: Any = None
        self._object_addresses: dict[str, int] = {}
        self._support_body_names: tuple[str, ...] = ()

    @classmethod
    def from_path(
        cls, path: object, *, task_mode: str | None, scene_config: object
    ) -> HandoverTaskPlugin:
        config = load_handover_task_config(path)  # type: ignore[arg-type]
        if getattr(scene_config, "task", None) is None:
            raise _error("SCENE_MISMATCH", "handover requires a Marvin handover scene")
        plugin = cls(config, task_mode=task_mode)
        plugin._scene_config = scene_config
        return plugin

    @property
    def binding(self) -> HandoverBinding | None:
        return self._binding

    def segmentation_geoms(self, model: object) -> dict[str, str]:
        del model
        binding = self._binding
        if binding is None:
            return {}
        return {
            "block": binding.block_geom_name,
            "table": "table_top",
            "box": "box_bottom",
        }

    def load_kinematics(self, scene_config: object) -> object | None:
        del scene_config
        return None

    def bind_scene(self, scene: object) -> None:
        config = self._scene_config
        if config is None:
            raise _error("SCENE_NOT_READY", "handover plugin has no scene recipe")
        address = int(scene.object_free_joint_qpos[config.task.block.name])  # type: ignore[attr-defined]
        layout = layout_from_config(config, scene.data.qpos, address)  # type: ignore[attr-defined]
        binding = make_handover_binding(
            scene, scene.indices.robot, layout=layout  # type: ignore[attr-defined]
        )
        self._binding = binding
        self._layout = layout
        self._robot = scene.indices.robot  # type: ignore[attr-defined]
        self._robot_geometry = scene.robot_geometry  # type: ignore[attr-defined]
        self._object_addresses = {
            name: int(value)
            for name, value in scene.object_free_joint_qpos.items()  # type: ignore[attr-defined]
        }
        self._support_body_names = tuple(scene.support_body_names)  # type: ignore[attr-defined]
        self._runtime = HandoverTaskRuntime(self.config, binding, layout)

    def _cached_prepare_duration_s(self, fact: Any) -> float:
        return float(fact.plan_duration_s)

    def _pose_check_addresses(self) -> tuple[int, ...]:
        assert self._binding is not None
        return (self._binding.block_qpos_address,)

    def _validate_plan_for_commit(self, plan_obj: Any) -> tuple[int, float, float]:
        assert isinstance(plan_obj, HandoverPlan)
        return super()._validate_plan_for_commit(plan_obj)

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
        del kinematics_config
        handover_snapshot = snapshot
        assert isinstance(handover_snapshot, HandoverSnapshot)
        if not self.teacher_enabled:
            layout = self._layout
            assert layout is not None
            plan = HandoverPlan(
                context=handover_snapshot.context,
                task_epoch=handover_snapshot.task_epoch,
                config_digest=self.config.config_digest,
                plan_digest=canonical_sha256(
                    {
                        "kind": "evaluation_only",
                        "task_config_digest": self.config.config_digest,
                        "context": handover_snapshot.context.__dict__,
                        "task_epoch": handover_snapshot.task_epoch,
                    }
                ),
                steps=(),
                duration_ns=round(self.config.success.stable_duration_s * 1e9),
                open_total_m=float(self._scene_config.robot.gripper_open_total),
                velocities=(0.0,) * 16,
                accelerations=(0.0,) * 16,
                ik_max_position_error_m=0.0,
                ik_max_orientation_error_rad=0.0,
                layout=layout,
            )
            return TaskPlan(plan, plan.plan_digest, plan.duration_ns, "evaluation_only")
        binding = self._binding
        if binding is None or self._robot is None or self._layout is None:
            raise _error("TASK_NOT_READY", "task plugin has no bound scene")
        plan = build_handover_plan(
            model,
            handover_snapshot,
            profile,
            control_config,  # type: ignore[arg-type]
            self.config,
            scene_config=self._scene_config,
            robot=self._robot,
            robot_geometry=self._robot_geometry,
            object_free_joint_qpos=self._object_addresses,
            support_body_names=self._support_body_names,
            deadline=deadline,
        )
        return TaskPlan(plan, plan.plan_digest, plan.duration_ns, "teacher")

    def capture_observation(self, scene: object) -> object:
        binding = self._binding
        if binding is None:
            raise _error("TASK_NOT_READY", "task plugin has no bound scene")
        return capture_handover_observation(scene, binding)

    def on_capture(self, capture: object, observation: object) -> None:
        if not self.teacher_enabled or self._runtime is None or observation is None:
            return
        feedback = HandoverFeedback(
            capture_seq=int(capture.capture_seq),  # type: ignore[attr-defined]
            sim_time_ns=int(capture.sim_time_ns),  # type: ignore[attr-defined]
            gate=self._runtime.gate_fact(),
        )
        self._teacher_feedback[feedback.capture_seq] = feedback
        if len(self._teacher_feedback) > CAPTURE_HISTORY_LIMIT:
            self._teacher_feedback.popitem(last=False)

    def _retained_feedback_and_gate(self, capture_seq: int) -> tuple[Any, Any] | None:
        feedback = self._teacher_feedback.get(capture_seq)
        if feedback is None:
            return None
        return (feedback, feedback.gate)

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
        return sample_handover_chunk(
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
