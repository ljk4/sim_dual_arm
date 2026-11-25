"""Immutable per-capture gate facts and progress state machine for Marvin stack Teacher."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

from robot_backend_sim.common.tasks.base import TaskGateFact
from robot_backend_sim.marvin.stack.config import StackTaskConfig

if TYPE_CHECKING:
    from robot_backend_sim.marvin.stack.teacher import StackPlan

__all__ = [
    "StackGateFact",
    "TeacherFeedback",
    "TeacherProgress",
]

StackGateFact = TaskGateFact


@dataclass(frozen=True, slots=True)
class TeacherFeedback:
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


class TeacherProgress:
    """Owner-thread gate tracker that advances phases only from measured feedback."""

    def __init__(self, plan: StackPlan, config: StackTaskConfig) -> None:
        self.plan = plan
        self.config = config
        self.phase_index = 0
        self.actual_start_ns = -1
        self.stable_since_ns: int | None = None
        self.hold_lost_since_ns: int | None = None
        self.last_wait_s = 0.0
        self.failure = ""
        self._status = "RUNNING"

    def bind_motion_start(self, step_index: int, start_ns: int) -> None:
        if step_index == self.phase_index and start_ns > 0:
            self.actual_start_ns = int(start_ns)
            self.stable_since_ns = None

    def fact(self) -> StackGateFact:
        segments = self.plan.segments
        idx = min(self.phase_index, len(segments) - 1) if segments else 0
        segment = segments[idx] if segments else None
        stable_s = 0.0
        if self.stable_since_ns is not None and self._last_sim_ns is not None:
            stable_s = max(0.0, (self._last_sim_ns - self.stable_since_ns) / 1e9)
        return StackGateFact(
            step_index=self.phase_index,
            step=segment.name if segment is not None else "",
            phase=segment.phase if segment is not None else "",
            status=self._status,
            active_start_sim_time_ns=self.actual_start_ns,
            phase_wait_s=self.last_wait_s,
            stable_elapsed_s=stable_s,
            failure=self.failure,
        )

    _last_sim_ns: int | None = None

    def update(self, feedback: TeacherFeedback, task_phase: str) -> StackGateFact:
        self._last_sim_ns = feedback.sim_time_ns
        if task_phase != "RUNNING" or self.failure:
            return self.fact()
        segments = self.plan.segments
        if self.phase_index >= len(segments):
            self._status = "COMPLETE"
            return self.fact()
        segment = segments[self.phase_index]
        if self.actual_start_ns < 0:
            self.actual_start_ns = feedback.sim_time_ns

        gates = self.config.gates
        duration_ns = round(segment.duration_s * 1e9)
        elapsed_ns = feedback.sim_time_ns - self.actual_start_ns
        self.last_wait_s = max(0.0, (elapsed_ns - duration_ns) / 1e9)

        if not segment.final:
            budget_ns = duration_ns + round(gates.phase_extra_timeout_s * 1e9)
            if elapsed_ns > budget_ns:
                self.failure = f"GATE_TIMEOUT:{segment.phase}"
                self._status = "FAILED"
                return self.fact()

        if segment.gate in ("lift", "carry", "place"):
            holding = (
                feedback.right_finger_source_contacts >= 2
                and feedback.right_finger_source_min_force_n >= gates.hold_min_force_n
            )
            if holding:
                self.hold_lost_since_ns = None
            else:
                if self.hold_lost_since_ns is None:
                    self.hold_lost_since_ns = feedback.sim_time_ns
                loss_s = (feedback.sim_time_ns - self.hold_lost_since_ns) / 1e9
                if loss_s > gates.hold_loss_tolerance_s:
                    self.failure = f"GATE_HOLD_LOST:{segment.phase}"
                    self._status = "FAILED"
                    return self.fact()

        if segment.final:
            self._status = "RUNNING"
            return self.fact()

        arm_target = segment.end_positions[8:15]
        arm_pos_err = max(
            abs(a - b)
            for a, b in zip(feedback.right_positions, arm_target, strict=True)
        )
        arm_vel_err = max(abs(v) for v in feedback.right_velocities)
        arm_ok = (
            arm_pos_err <= gates.arm_position_tolerance_rad
            and arm_vel_err <= gates.arm_velocity_tolerance_rad_s
        )

        gate_ok = False
        if segment.gate == "arm":
            gate_ok = arm_ok
        elif segment.gate == "close":
            gate_ok = (
                arm_ok
                and feedback.right_finger_source_contacts >= 2
                and feedback.right_finger_source_min_force_n >= gates.hold_min_force_n
                and feedback.right_finger_source_max_speed_m_s <= gates.finger_max_speed_m_s
            )
        elif segment.gate == "lift":
            lifted = (
                feedback.source_center[2] - self.plan.source_initial_center[2]
                >= gates.lift_height_m
            )
            gate_ok = (
                arm_ok
                and feedback.right_finger_source_contacts >= 2
                and feedback.right_finger_source_min_force_n >= gates.hold_min_force_n
                and lifted
            )
        elif segment.gate == "carry":
            gate_ok = (
                arm_ok
                and feedback.right_finger_source_contacts >= 2
                and feedback.right_finger_source_min_force_n >= gates.hold_min_force_n
            )
        elif segment.gate == "place":
            xy_err = max(
                abs(feedback.source_center[0] - self.plan.target_center[0]),
                abs(feedback.source_center[1] - self.plan.target_center[1]),
            )
            z_err = abs(feedback.source_center[2] - self.plan.expected_placed_source_z_m)
            gate_ok = (
                arm_ok
                and feedback.right_finger_source_contacts >= 2
                and feedback.right_finger_source_min_force_n >= gates.hold_min_force_n
                and xy_err <= gates.place_xy_tolerance_m
                and z_err <= gates.place_z_tolerance_m
            )
        elif segment.gate == "release":
            gate_ok = (
                arm_ok
                and feedback.right_finger_source_contacts == 0
                and feedback.right_gripper_opening_m >= gates.release_opening_m
                and abs(feedback.right_gripper_velocity_m_s)
                <= gates.gripper_velocity_tolerance_m_s
            )
        elif segment.gate == "retreat":
            gate_ok = (
                arm_ok
                and feedback.right_finger_source_contacts == 0
                and feedback.right_gripper_opening_m >= gates.release_opening_m
                and feedback.right_tcp_distance_m >= gates.retreat_min_tcp_to_source_m
            )

        if gate_ok:
            if self.stable_since_ns is None:
                self.stable_since_ns = feedback.sim_time_ns
        else:
            self.stable_since_ns = None

        elapsed_ok = elapsed_ns >= duration_ns
        stable_ns = round(gates.stable_duration_s * 1e9)
        if (
            elapsed_ok
            and gate_ok
            and self.stable_since_ns is not None
            and (feedback.sim_time_ns - self.stable_since_ns) >= stable_ns
        ):
            self.phase_index += 1
            self.actual_start_ns = feedback.sim_time_ns
            self.stable_since_ns = None
            self.hold_lost_since_ns = None
            self.last_wait_s = 0.0
            if self.phase_index >= len(segments):
                self._status = "COMPLETE"
        return self.fact()
