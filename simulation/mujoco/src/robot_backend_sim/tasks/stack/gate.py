"""Measured per-phase teacher gate state; never a static plan clock."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from .config import StackTaskConfig
from .evaluator import TaskObservation

if TYPE_CHECKING:
    from .teacher import TeacherPlan, TeacherSegment

__all__ = ["TeacherFeedback", "TeacherGateFact", "TeacherProgress"]


@dataclass(frozen=True, slots=True)
class TeacherFeedback:
    """Per-capture immutable evidence for gate decisions (no image arrays)."""

    capture_seq: int
    sim_time_ns: int
    right_positions: tuple[float, ...]
    right_velocities: tuple[float, ...]
    right_gripper_opening_m: float
    right_gripper_velocity_m_s: float
    right_finger_source_contacts: int
    right_finger_source_min_force_n: float
    right_finger_source_max_speed_m_s: float
    source_center: tuple[float, float, float]
    right_tcp_distance_m: float


@dataclass(frozen=True, slots=True)
class TeacherGateFact:
    phase_index: int
    phase: str
    status: str
    actual_start_sim_time_ns: int
    phase_wait_s: float
    stable_elapsed_s: float
    failure: str


class TeacherProgress:
    """Measured per-phase gate state; never a static plan clock."""

    def __init__(self, plan: TeacherPlan, config: StackTaskConfig) -> None:
        self.plan = plan
        self.config = config
        self.phase_index = 0
        self.actual_start_ns = -1
        self.stable_since_ns: int | None = None
        self.failure = ""
        self.actual_starts_ns: list[int] = []
        self.last_feedback: TeacherFeedback | None = None
        self.nominal_starts_s = [segment.start_s for segment in plan.segments]

    @property
    def phase(self) -> str:
        return self.plan.segments[min(self.phase_index, len(self.plan.segments) - 1)].phase

    def _arm_ok(self, feedback: TeacherFeedback, segment: TeacherSegment) -> bool:
        target = segment.end_positions[8:15]
        teacher = self.config.teacher
        return all(
            abs(measured - wanted) <= teacher.phase_joint_tolerance_rad
            for measured, wanted in zip(feedback.right_positions, target, strict=True)
        ) and all(
            abs(velocity) <= teacher.phase_velocity_tolerance_rad_s
            for velocity in feedback.right_velocities
        )

    def _contact_ok(self, feedback: TeacherFeedback) -> bool:
        teacher = self.config.teacher
        return (
            feedback.right_finger_source_contacts >= 2
            and feedback.right_finger_source_min_force_n >= teacher.contact_force_min_n
            and feedback.right_finger_source_max_speed_m_s <= teacher.finger_speed_max_m_s
        )

    def _condition(self, feedback: TeacherFeedback) -> bool:
        segment = self.plan.segments[self.phase_index]
        teacher = self.config.teacher
        phase = segment.phase
        if phase in ("pregrasp", "grasp"):
            return self._arm_ok(feedback, segment)
        if phase in ("close", "transport", "lower"):
            return self._arm_ok(feedback, segment) and self._contact_ok(feedback)
        if phase == "lift":
            return (
                self._arm_ok(feedback, segment)
                and self._contact_ok(feedback)
                and feedback.source_center[2] - self.plan.source_center[2] >= teacher.lift_height_m
            )
        if phase == "open":
            return (
                self._arm_ok(feedback, segment)
                and feedback.right_finger_source_contacts == 0
                and feedback.right_gripper_opening_m
                >= teacher.release_gripper_opening_m - teacher.gripper_position_tolerance_m
                and abs(feedback.right_gripper_velocity_m_s)
                <= teacher.gripper_velocity_tolerance_m_s
            )
        if phase == "retreat":
            return (
                self._arm_ok(feedback, segment)
                and feedback.right_tcp_distance_m >= teacher.retreat_tcp_distance_m
            )
        if phase == "stable":
            return True
        return False

    def update(self, feedback: TeacherFeedback, task_state: str) -> TeacherGateFact:
        self.last_feedback = feedback
        if self.failure:
            return self.fact()
        if task_state == "FAILED":
            self.failure = "TASK_FAILED"
            return self.fact()
        if self.phase_index >= len(self.plan.segments):
            return self.fact()
        if self.actual_start_ns < 0:
            self.actual_start_ns = feedback.sim_time_ns
            self.actual_starts_ns.append(self.actual_start_ns)
        segment = self.plan.segments[self.phase_index]
        teacher = self.config.teacher
        if self.phase != "stable":
            wait_ns = feedback.sim_time_ns - self.actual_start_ns
            timeout_ns = round((segment.duration_s + teacher.phase_gate_extra_timeout_s) * 1e9)
            if wait_ns > timeout_ns:
                self.failure = f"GATE_TIMEOUT:{segment.phase}"
                return self.fact()
        if self._condition(feedback):
            if self.stable_since_ns is None:
                self.stable_since_ns = feedback.sim_time_ns
            if (
                feedback.sim_time_ns - self.stable_since_ns
                >= round(teacher.gate_stable_duration_s * 1e9)
                and self.phase != "stable"
                # Contact may settle halfway through closing. Finish the raw
                # ramp before changing phases, so the next phase's start is C1.
                and feedback.sim_time_ns - self.actual_start_ns >= round(segment.duration_s * 1e9)
            ):
                self.phase_index += 1
                self.stable_since_ns = None
                if self.phase_index < len(self.plan.segments):
                    self.actual_start_ns = feedback.sim_time_ns
                    self.actual_starts_ns.append(self.actual_start_ns)
        else:
            self.stable_since_ns = None
        return self.fact()

    def fact(self) -> TeacherGateFact:
        stable = 0.0
        feedback = self.last_feedback
        if self.stable_since_ns is not None and feedback is not None:
            stable = max(0.0, (feedback.sim_time_ns - self.stable_since_ns) / 1e9)
        wait = 0.0
        elapsed_s = 0.0
        segment = self.plan.segments[min(self.phase_index, len(self.plan.segments) - 1)]
        if feedback is not None:
            elapsed_s = max(0.0, (feedback.sim_time_ns - self.actual_start_ns) / 1e9)
            wait = max(0.0, elapsed_s - segment.duration_s)
        if self.failure:
            status = "FAILED"
        elif segment.phase == "stable":
            status = "HOLDING"
        else:
            status = "TRACKING" if elapsed_s < segment.duration_s else "WAITING_FOR_FEEDBACK"
        return TeacherGateFact(
            phase_index=min(self.phase_index, len(self.plan.segments) - 1),
            phase=self.phase,
            status=status,
            actual_start_sim_time_ns=self.actual_start_ns,
            phase_wait_s=wait,
            stable_elapsed_s=stable,
            failure=self.failure,
        )

    def feedback_for(self, capture: Any, observation: TaskObservation) -> TeacherFeedback:
        return TeacherFeedback(
            capture_seq=int(capture.capture_seq),
            sim_time_ns=int(capture.sim_time_ns),
            right_positions=tuple(float(value) for value in capture.positions[8:15]),
            right_velocities=tuple(float(value) for value in capture.velocities[8:15]),
            right_gripper_opening_m=float(capture.positions[15]),
            right_gripper_velocity_m_s=float(capture.velocities[15]),
            right_finger_source_contacts=observation.right_finger_source_contacts,
            right_finger_source_min_force_n=observation.right_finger_source_min_force_n,
            right_finger_source_max_speed_m_s=observation.right_finger_source_max_speed_m_s,
            source_center=observation.source.center,
            right_tcp_distance_m=observation.right_tcp_to_source_m,
        )

