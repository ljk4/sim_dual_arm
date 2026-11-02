"""Backend-side request precompilation and shared-DTO feedback adaptation.

Nothing here imports ROS, writes a file, steps physics, creates a thread or
reads a hidden clock.  ``prepare_request`` runs in the caller's planning phase
and returns the existing ``PreparedControlCommand``; the feedback functions
copy an already frozen ``ControlStatus``/``ChunkStatus`` snapshot into the
public transport DTO without rewriting terminal history.
"""

from __future__ import annotations

from collections.abc import Sequence
from numbers import Integral

from robot_contracts.models import ContractError, RobotProfile
from robot_contracts.native import (
    INTERPOLATION_VERSION,
    NATIVE_MOTION,
    AcceptedReference,
    NativeChunkStatus,
    NativeControlEvent,
    NativeExecution,
    NativeFinger,
    NativeGripper,
    NativeRequest,
    expected_final_velocities,
)

from .chunk_plan import PreparedChunk, prepare_chunk
from .control_state import (
    ChunkStatus,
    ControlEvent,
    ControlStatus,
    PreparedControlCommand,
)
from .motion import MotionSnapshot, PreparedMotion, prepare_motion
from .motion_config import PandaControlConfig

__all__ = [
    "accepted_reference",
    "accepted_reference_from_chunk_status",
    "accepted_reference_from_status",
    "chunk_feedback",
    "control_event",
    "execution_feedback",
    "prepare_request",
]


def _invalid(reason: str, detail: str) -> ContractError:
    return ContractError(reason, detail)


def _as_snapshot(value: object) -> MotionSnapshot:
    if not isinstance(value, MotionSnapshot):
        raise _invalid(
            "INVALID_ACCEPTED_SNAPSHOT", "accepted snapshot must be a MotionSnapshot"
        )
    return value


def prepare_request(
    request: NativeRequest,
    profile: RobotProfile,
    control_config: PandaControlConfig,
    *,
    initial_reference: Sequence[float] | None = None,
) -> PreparedControlCommand:
    """Precompile one request into the existing immutable command envelope.

    MOTION requires an explicit last-hold ``initial_reference``; streams derive
    their start position/velocity only from the request anchor.  No scene,
    measurement or hidden re-basing is consulted.
    """
    if not isinstance(request, NativeRequest):
        raise _invalid("INVALID_NATIVE_REQUEST", "request must be a NativeRequest")
    if not isinstance(profile, RobotProfile):
        raise _invalid("PROFILE_MISMATCH", "profile must be a RobotProfile")
    if not isinstance(control_config, PandaControlConfig):
        raise _invalid("INVALID_PREPARED_REQUEST", "control_config must be PandaControlConfig")
    if tuple(profile.joint_names) != tuple(control_config.joint_names):
        raise _invalid("PROFILE_MISMATCH", "profile joint order differs from control config")
    if request.trajectory.joint_names != tuple(profile.joint_names):
        raise _invalid("PROFILE_MISMATCH", "request joint order differs from profile")
    if request.trajectory.context.robot_profile_hash != profile.digest:
        raise _invalid("PROFILE_MISMATCH", "request profile digest differs from profile")

    command = request.command
    if request.kind == NATIVE_MOTION:
        if initial_reference is None:
            raise _invalid(
                "INVALID_PREPARED_REQUEST",
                "MOTION requires the caller's explicit last-hold initial_reference",
            )
        prepared: PreparedMotion | PreparedChunk = prepare_motion(
            initial_reference,
            request.trajectory.targets,
            profile,
            control_config,
        )
    else:
        if initial_reference is not None:
            raise _invalid(
                "INVALID_PREPARED_REQUEST",
                "stream requests must not accept a separate initial_reference",
            )
        assert request.anchor is not None
        window = request.window
        assert window is not None
        prepared = prepare_chunk(
            request.anchor,
            window,
            profile,
            control_config,
            final_velocities=request.final_velocities,
            settle_at_end=request.settle_at_end,
        )
    return PreparedControlCommand(command, prepared, request=request)


def _request_window_matches(request: NativeRequest, prepared: PreparedChunk) -> bool:
    window = request.window
    assert window is not None
    return (
        request.anchor == prepared.anchor
        and request.trajectory.observation_id == window.observation_id
        and request.observation_time_ns == window.observation_time_ns
        and request.trajectory.targets == window.source_targets
        and request.target_count == window.target_count
        and request.settle_at_end == prepared.settle_at_end
        and tuple(prepared.final_velocities)
        == tuple(expected_final_velocities(request))
    )


def _check_request_packet(request: NativeRequest, packet: PreparedControlCommand) -> None:
    if not isinstance(request, NativeRequest):
        raise _invalid("INVALID_NATIVE_REQUEST", "request must be a NativeRequest")
    if not isinstance(packet, PreparedControlCommand):
        raise _invalid("INVALID_PREPARED_COMMAND", "packet must be PreparedControlCommand")
    if request.command != packet.command:
        raise _invalid(
            "REQUEST_PACKET_MISMATCH", "request identity differs from the compiled packet"
        )
    if packet.request is not None and packet.request != request:
        raise _invalid(
            "REQUEST_PACKET_MISMATCH",
            "request differs from the complete request retained with the packet",
        )


def _build_accepted_reference(
    request: NativeRequest,
    packet: PreparedControlCommand,
    *,
    motion_seq: int,
    accepted_sim_time_ns: int,
) -> AcceptedReference:
    _check_request_packet(request, packet)
    if isinstance(motion_seq, bool) or not isinstance(motion_seq, int) or motion_seq < 1:
        raise _invalid("INVALID_ACCEPTED_SNAPSHOT", "accepted fact lacks a motion_seq")
    prepared = packet.prepared
    if request.kind == NATIVE_MOTION:
        if not isinstance(prepared, PreparedMotion):
            raise _invalid("REQUEST_PACKET_MISMATCH", "MOTION packet is not PreparedMotion")
        if (
            prepared.targets != request.trajectory.targets
            or prepared.joint_names != request.trajectory.joint_names
            or prepared.profile_digest != request.trajectory.context.robot_profile_hash
        ):
            raise _invalid(
                "REQUEST_PACKET_MISMATCH", "MOTION request does not match the compiled plan"
            )
        joint_count = len(prepared.joint_names)
        return AcceptedReference(
            request=request,
            motion_seq=motion_seq,
            accepted_sim_time_ns=accepted_sim_time_ns,
            joint_names=prepared.joint_names,
            plan=prepared.plan,
            initial_positions=prepared.initial_reference,
            initial_velocities=(0.0,) * joint_count,
            final_velocities=(0.0,) * joint_count,
            max_accelerations=prepared.max_accelerations,
            selected_indices=(),
            interpolation_version=INTERPOLATION_VERSION,
        )

    if not isinstance(prepared, PreparedChunk):
        raise _invalid("REQUEST_PACKET_MISMATCH", "stream packet is not PreparedChunk")
    if not _request_window_matches(request, prepared):
        raise _invalid(
            "REQUEST_PACKET_MISMATCH", "stream request does not match the compiled chunk"
        )
    if prepared.plan.joint_names != request.trajectory.joint_names:
        raise _invalid("REQUEST_PACKET_MISMATCH", "compiled plan joint order differs")
    return AcceptedReference(
        request=request,
        motion_seq=motion_seq,
        accepted_sim_time_ns=accepted_sim_time_ns,
        joint_names=prepared.joint_names,
        plan=prepared.plan,
        initial_positions=prepared.anchor.reference.positions,
        initial_velocities=prepared.anchor.reference.velocities,
        final_velocities=prepared.final_velocities,
        max_accelerations=prepared.max_accelerations,
        selected_indices=prepared.window.selected_indices,
        interpolation_version=INTERPOLATION_VERSION,
    )


def accepted_reference(
    request: NativeRequest,
    packet: PreparedControlCommand,
    accepted_snapshot: MotionSnapshot,
) -> AcceptedReference:
    """Bind a request to the actually compiled packet and acceptance fact."""
    snapshot = _as_snapshot(accepted_snapshot)
    if snapshot.motion_seq < 1:
        raise _invalid("INVALID_ACCEPTED_SNAPSHOT", "accepted snapshot lacks a motion_seq")
    return _build_accepted_reference(
        request,
        packet,
        motion_seq=snapshot.motion_seq,
        accepted_sim_time_ns=snapshot.measurement_sim_time_ns,
    )


def accepted_reference_from_status(
    request: NativeRequest,
    packet: PreparedControlCommand,
    status: ControlStatus,
) -> AcceptedReference:
    """Rebuild the same receipt from a frozen historical acceptance fact.

    This never substitutes the current ``latest_snapshot``: ``accepted_sim_ns``
    and ``motion_seq`` are the values saved when the command was accepted, so an
    idempotent retry returns the original digest/time/plan even after later
    physics stepped forward.
    """
    if not isinstance(status, ControlStatus):
        raise _invalid("INVALID_NATIVE_EXECUTION", "status must be a ControlStatus")
    if status.command_id != request.command.command_id:
        raise _invalid("REQUEST_PACKET_MISMATCH", "status command_id differs from request")
    if status.command_seq != request.command.command_seq:
        raise _invalid("REQUEST_PACKET_MISMATCH", "status command_seq differs from request")
    if status.motion_seq is None:
        raise _invalid("INVALID_ACCEPTED_SNAPSHOT", "accepted status lacks a motion_seq")
    return _build_accepted_reference(
        request,
        packet,
        motion_seq=status.motion_seq,
        accepted_sim_time_ns=status.accepted_sim_ns,
    )


def accepted_reference_from_chunk_status(
    request: NativeRequest,
    packet: PreparedControlCommand,
    chunk: ChunkStatus,
    *,
    motion_seq: int,
) -> AcceptedReference:
    """Rebuild a successor chunk receipt from its own accepted fact.

    A stream keeps one ``motion_seq`` for the whole command, but each chunk has
    its own acceptance time.  Using the overall command acceptance here would
    make a retry's digest misleading even though the plan is unchanged.
    """
    if not isinstance(chunk, ChunkStatus):
        raise _invalid("INVALID_NATIVE_CHUNK", "chunk must be a ChunkStatus")
    if chunk.command_id != request.command.command_id:
        raise _invalid("REQUEST_PACKET_MISMATCH", "chunk command_id differs from request")
    if chunk.command_seq != request.command.command_seq:
        raise _invalid("REQUEST_PACKET_MISMATCH", "chunk command_seq differs from request")
    prepared = packet.prepared
    if not isinstance(prepared, PreparedChunk) or chunk.chunk_seq != prepared.seq:
        raise _invalid("REQUEST_PACKET_MISMATCH", "chunk seq differs from compiled packet")
    return _build_accepted_reference(
        request,
        packet,
        motion_seq=motion_seq,
        accepted_sim_time_ns=chunk.accepted_sim_ns,
    )


def _gripper_feedback(completion: object) -> NativeGripper:
    fingers: list[NativeFinger] = []
    for evidence in completion.fingers:  # type: ignore[attr-defined]
        object_names = tuple(item[0] for item in evidence.object_forces)
        normal_forces = tuple(item[1] for item in evidence.object_forces)
        contact_counts = tuple(item[2] for item in evidence.object_forces)
        fingers.append(
            NativeFinger(
                side=evidence.side,
                finger_index=evidence.finger_index,
                body_name=evidence.body_name,
                joint_speed_m_s=evidence.joint_speed_m_s,
                object_names=object_names,
                normal_forces=normal_forces,
                contact_counts=contact_counts,
            )
        )
    return NativeGripper(
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


def _execution_grippers(snapshot: MotionSnapshot) -> tuple[NativeGripper, ...]:
    grippers = tuple(_gripper_feedback(completion) for completion in snapshot.grippers)
    if len(grippers) != 2:
        raise _invalid(
            "INVALID_NATIVE_EXECUTION", "snapshot must carry exactly two gripper decisions"
        )
    return grippers


def execution_feedback(status: ControlStatus) -> NativeExecution:
    """Copy one frozen overall status snapshot into shared execution feedback."""
    if not isinstance(status, ControlStatus):
        raise _invalid("INVALID_NATIVE_EXECUTION", "status must be a ControlStatus")
    snapshot = status.snapshot
    if snapshot is None:
        raise _invalid(
            "INVALID_NATIVE_EXECUTION", "status has no immutable snapshot to report"
        )
    if status.accepted_wall_ns is None:
        raise _invalid(
            "INVALID_NATIVE_EXECUTION", "accepted wall time is required for feedback"
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
    return NativeExecution(
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


def chunk_feedback(chunk: ChunkStatus) -> NativeChunkStatus:
    """Copy one frozen chunk status without relabelling RETIRED/terminal phases."""
    if not isinstance(chunk, ChunkStatus):
        raise _invalid("INVALID_NATIVE_CHUNK", "chunk must be a ChunkStatus")
    if chunk.accepted_wall_ns is None:
        raise _invalid("INVALID_NATIVE_CHUNK", "accepted wall time is required")
    return NativeChunkStatus(
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


def control_event(event: ControlEvent) -> NativeControlEvent:
    """Copy one controller event verbatim, including its original identity/time."""
    if not isinstance(event, ControlEvent):
        raise _invalid("INVALID_NATIVE_EVENT", "event must be a ControlEvent")
    if event.chunk_seq is not None and (
        isinstance(event.chunk_seq, bool)
        or not isinstance(event.chunk_seq, Integral)
        or int(event.chunk_seq) < 1
    ):
        raise _invalid("INVALID_NATIVE_EVENT", "chunk_seq must be positive or absent")
    return NativeControlEvent(
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
