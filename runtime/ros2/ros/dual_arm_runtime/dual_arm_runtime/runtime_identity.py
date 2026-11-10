"""Stable identity tuple for one execution-feedback sample."""

from __future__ import annotations

from robot_contracts.execution_feedback import ExecutionFeedback


def execution_identity(execution: ExecutionFeedback) -> tuple[object, ...]:
    return (
        execution.phase,
        execution.reason,
        execution.stop_reason,
        execution.accepted_sim_time_ns,
        execution.accepted_wall_time_ns,
        execution.has_started,
        execution.started_sim_time_ns,
        execution.started_wall_time_ns,
        execution.has_finished,
        execution.finished_sim_time_ns,
        execution.finished_wall_time_ns,
        execution.motion_seq,
        execution.stream_id,
        execution.chunk_seq,
        execution.motion_phase,
        execution.reference_application_sim_time_ns,
        execution.measurement_sim_time_ns,
        execution.reference_elapsed_ns,
        execution.stable_for_ns,
        tuple(execution.reference_positions),
        tuple(execution.reference_velocities),
        tuple(execution.reference_accelerations),
        tuple(execution.measured_positions),
        tuple(execution.measured_velocities),
        tuple(execution.position_errors),
        tuple(execution.velocity_errors),
        execution.reference_kind,
        execution.tracking_quality,
        execution.tracking_violation_ns,
        tuple(
            (
                gripper.side,
                gripper.mode,
                gripper.position_ok,
                gripper.contact_ok,
                gripper.contact_object,
                gripper.ambiguous,
                gripper.target_position_m,
                gripper.initial_reference_m,
                gripper.final_reference_m,
                gripper.measured_position_m,
                gripper.measured_velocity_m_s,
                gripper.target_error_m,
                tuple(
                    (
                        finger.finger_index,
                        finger.body_name,
                        finger.joint_speed_m_s,
                        tuple(finger.object_names),
                        tuple(finger.normal_forces),
                        tuple(finger.contact_counts),
                    )
                    for finger in gripper.fingers
                ),
            )
            for gripper in execution.grippers
        ),
    )


__all__ = ["execution_identity"]
