"""Shared-DTO feedback adaptation for the backend worker.

Nothing here imports ROS, writes a file, steps physics, creates a thread or
reads a hidden clock.  The functions copy an already frozen
``ControlStatus``/``ChunkStatus`` snapshot or controller event into the public
transport DTO without rewriting terminal history.  Request precompilation and
accepted-reference records live in :mod:`robot_backend_sim.common.follower.follower_protocol`.
"""

from __future__ import annotations

from numbers import Integral

from robot_contracts.execution_feedback import (
    ControlEvent as ControlEventRecord,
)
from robot_contracts.execution_feedback import (
    ExecutionFeedback,
    FingerFeedback,
    GripperFeedback,
    StreamChunkStatus,
)
from robot_contracts.models import ContractError

from robot_backend_sim.common.control.control_state import ChunkStatus, ControlEvent, ControlStatus

__all__ = [
    "chunk_feedback",
    "control_event",
    "execution_feedback",
]


def _invalid(reason: str, detail: str) -> ContractError:
    return ContractError(reason, detail)


def _gripper_feedback(completion: object) -> GripperFeedback:
    fingers: list[FingerFeedback] = []
    for evidence in completion.fingers:  # type: ignore[attr-defined]
        object_names = tuple(item[0] for item in evidence.object_forces)
        normal_forces = tuple(item[1] for item in evidence.object_forces)
        contact_counts = tuple(item[2] for item in evidence.object_forces)
        fingers.append(
            FingerFeedback(
                side=evidence.side,
                finger_index=evidence.finger_index,
                body_name=evidence.body_name,
                joint_speed_m_s=evidence.joint_speed_m_s,
                object_names=object_names,
                normal_forces=normal_forces,
                contact_counts=contact_counts,
            )
        )
    return GripperFeedback(
        side=completion.side,  # type: ignore[attr-defined]
        mode=completion.mode,  # type: ignore[attr-defined]
        position_ok=completion.position_ok,  # type: ignore[attr-defined]
        contact_ok=completion.contact_ok,  # type: ignore[attr-defined]
        contact_object=completion.contact_object,  # type: ignore[attr-defined]
        ambiguous=completion.ambiguous,  # type: ignore[attr-defined]
        target_position_m=completion.target_position_m,  # type: ignore[attr-defined]
        initial_reference_m=completion.initial_reference_m,  # type: ignore[attr-defined]
        final_reference_m=completion.final_reference_m,  # type: ignore[attr-defined]
        measured_position_m=completion.measured_position_m,  # type: ignore[attr-defined]
        measured_velocity_m_s=completion.measured_velocity_m_s,  # type: ignore[attr-defined]
        target_error_m=completion.target_error_m,  # type: ignore[attr-defined]
        fingers=tuple(fingers),
    )


def _execution_grippers(snapshot: object) -> tuple[GripperFeedback, ...]:
    # The shared container accepts any bounded named gripper set.  The
    # two-sided/two-finger evidence rule is enforced by GripperContactTracker;
    # this adapter only copies the frozen snapshot without adding a new rule.
    return tuple(
        _gripper_feedback(completion)
        for completion in snapshot.grippers  # type: ignore[attr-defined]
    )


def execution_feedback(status: ControlStatus) -> ExecutionFeedback:
    """Copy one frozen overall status snapshot into shared execution feedback."""
    if not isinstance(status, ControlStatus):
        raise _invalid("INVALID_EXECUTION_FEEDBACK", "status must be a ControlStatus")
    snapshot = status.snapshot
    if snapshot is None:
        raise _invalid(
            "INVALID_EXECUTION_FEEDBACK", "status has no immutable snapshot to report"
        )
    if status.accepted_wall_ns is None:
        raise _invalid(
            "INVALID_EXECUTION_FEEDBACK", "accepted wall time is required for feedback"
        )
    velocity_errors = snapshot.tracking_velocity_errors
    if not velocity_errors:
        velocity_errors = tuple(
            reference - measured
            for reference, measured in zip(
                snapshot.reference_velocities,
                snapshot.measured_velocities,
                strict=True,
            )
        )
    return ExecutionFeedback(
        context=status.context,
        command_id=status.command_id,
        command_seq=status.command_seq,
        observation_id=status.observation_id,
        phase=status.phase,
        reason=status.reason,
        stop_reason=status.stop_reason or "",
        motion_seq=status.motion_seq,
        stream_id=status.stream_id or "",
        chunk_seq=0 if status.chunk_seq is None else status.chunk_seq,
        accepted_sim_time_ns=status.accepted_sim_ns,
        accepted_wall_time_ns=status.accepted_wall_ns,
        has_started=status.started_sim_ns is not None,
        started_sim_time_ns=status.started_sim_ns,
        started_wall_time_ns=status.started_wall_ns,
        has_finished=status.finished_sim_ns is not None,
        finished_sim_time_ns=status.finished_sim_ns,
        finished_wall_time_ns=status.finished_wall_ns,
        motion_phase=snapshot.phase.value,
        reference_application_sim_time_ns=snapshot.reference_application_sim_time_ns,
        measurement_sim_time_ns=snapshot.measurement_sim_time_ns,
        reference_elapsed_ns=snapshot.reference_elapsed_ns,
        stable_for_ns=snapshot.stable_for_ns,
        joint_names=snapshot.joint_names,
        reference_positions=snapshot.reference_positions,
        reference_velocities=snapshot.reference_velocities,
        reference_accelerations=snapshot.reference_accelerations,
        measured_positions=snapshot.measured_positions,
        measured_velocities=snapshot.measured_velocities,
        position_errors=snapshot.tracking_errors,
        velocity_errors=velocity_errors,
        reference_kind=snapshot.reference_kind,
        tracking_quality=snapshot.tracking_quality,
        tracking_violation_ns=snapshot.tracking_violation_ns,
        grippers=_execution_grippers(snapshot),
    )


def chunk_feedback(chunk: ChunkStatus) -> StreamChunkStatus:
    """Copy one frozen chunk status without relabelling RETIRED/terminal phases."""
    if not isinstance(chunk, ChunkStatus):
        raise _invalid("INVALID_STREAM_CHUNK", "chunk must be a ChunkStatus")
    if chunk.accepted_wall_ns is None:
        raise _invalid("INVALID_STREAM_CHUNK", "accepted wall time is required")
    return StreamChunkStatus(
        context=chunk.context,
        command_id=chunk.command_id,
        command_seq=chunk.command_seq,
        chunk_seq=chunk.chunk_seq,
        observation_id=chunk.observation_id,
        phase=chunk.phase,
        reason=chunk.reason,
        accepted_sim_time_ns=chunk.accepted_sim_ns,
        accepted_wall_time_ns=chunk.accepted_wall_ns,
        has_started=chunk.started_sim_ns is not None,
        started_sim_time_ns=chunk.started_sim_ns,
        started_wall_time_ns=chunk.started_wall_ns,
        has_finished=chunk.finished_sim_ns is not None,
        finished_sim_time_ns=chunk.finished_sim_ns,
        finished_wall_time_ns=chunk.finished_wall_ns,
    )


def control_event(event: ControlEvent) -> ControlEventRecord:
    """Copy one controller event verbatim, including its original identity/time."""
    if not isinstance(event, ControlEvent):
        raise _invalid("INVALID_CONTROL_EVENT", "event must be a ControlEvent")
    if event.chunk_seq is not None and (
        isinstance(event.chunk_seq, bool)
        or not isinstance(event.chunk_seq, Integral)
        or int(event.chunk_seq) < 1
    ):
        raise _invalid("INVALID_CONTROL_EVENT", "chunk_seq must be positive or absent")
    return ControlEventRecord(
        context=event.context,
        event_seq=event.event_seq,
        command_id=event.command_id,
        command_seq=event.command_seq,
        observation_id=event.observation_id,
        is_chunk=event.chunk_seq is not None,
        chunk_seq=event.chunk_seq,
        phase=event.phase,
        reason=event.reason,
        sim_time_ns=event.sim_time_ns,
    )
