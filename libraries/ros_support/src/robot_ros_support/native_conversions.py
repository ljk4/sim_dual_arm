"""Explicit ROS conversions for native requests, references and feedback.

Each public shared DTO has a named bidirectional function.  Generated classes
are never trusted: every field is range/type/finite/mode checked again by the
shared contract constructors.  Unsupported trajectory semantics (non-empty
frame, extra velocity/acceleration/effort, absent-anchor inconsistencies) raise
``ContractError`` instead of being silently dropped.
"""

from __future__ import annotations

from robot_api_msgs.msg import NativeChunkStatus as NativeChunkStatusMessage
from robot_api_msgs.msg import NativeCommand as NativeCommandMessage
from robot_api_msgs.msg import NativeControlEvent as NativeControlEventMessage
from robot_api_msgs.msg import NativeExecution as NativeExecutionMessage
from robot_api_msgs.msg import NativeFinger as NativeFingerMessage
from robot_api_msgs.msg import NativeGripper as NativeGripperMessage
from robot_api_msgs.msg import ReferenceAnchor as ReferenceAnchorMessage
from robot_contracts.action_chunks import ChunkAnchor
from robot_contracts.models import ContractError
from robot_contracts.native import (
    NativeChunkStatus,
    NativeControlEvent,
    NativeExecution,
    NativeFinger,
    NativeGripper,
    NativeRequest,
)
from robot_contracts.native_reference import Trajectory
from robot_contracts.trajectory import ReferenceSample

from .conversions import (
    context_from_message,
    context_message,
    stamp,
    time_ns,
    trajectory_from_message,
    trajectory_message,
)

__all__ = [
    "anchor_from_message",
    "anchor_message",
    "native_chunk_status_from_message",
    "native_chunk_status_message",
    "native_command_from_message",
    "native_command_message",
    "native_control_event_from_message",
    "native_control_event_message",
    "native_execution_from_message",
    "native_execution_message",
    "native_finger_from_message",
    "native_finger_message",
    "native_gripper_from_message",
    "native_gripper_message",
]


def anchor_message(value: ChunkAnchor) -> ReferenceAnchorMessage:
    if not isinstance(value, ChunkAnchor):
        raise ContractError("INVALID_CHUNK_ANCHOR", "value must be a ChunkAnchor")
    return ReferenceAnchorMessage(
        stream_id=value.stream_id,
        predecessor_seq=value.predecessor_seq,
        start_sim_time_ns=value.start_sim_time_ns,
        robot_profile_hash=value.robot_profile_hash,
        positions=list(value.reference.positions),
        velocities=list(value.reference.velocities),
        accelerations=list(value.reference.accelerations),
    )


def anchor_from_message(value: ReferenceAnchorMessage) -> ChunkAnchor:
    return ChunkAnchor(
        stream_id=value.stream_id,
        predecessor_seq=value.predecessor_seq,
        start_sim_time_ns=value.start_sim_time_ns,
        reference=ReferenceSample(
            tuple(value.positions),
            tuple(value.velocities),
            tuple(value.accelerations),
        ),
        robot_profile_hash=value.robot_profile_hash,
    )


def _anchor_absent(value: ReferenceAnchorMessage) -> bool:
    return (
        not value.stream_id
        and value.predecessor_seq == 0
        and value.start_sim_time_ns == 0
        and not value.robot_profile_hash
        and not value.positions
        and not value.velocities
        and not value.accelerations
    )


def native_command_message(value: NativeRequest) -> NativeCommandMessage:
    if not isinstance(value, NativeRequest):
        raise ContractError("INVALID_NATIVE_REQUEST", "value must be a NativeRequest")
    anchor = ReferenceAnchorMessage() if value.anchor is None else anchor_message(value.anchor)
    trajectory = value.trajectory
    return NativeCommandMessage(
        kind=value.kind,
        context=context_message(trajectory.context),
        lease_id=trajectory.lease_id,
        command_id=trajectory.command_id,
        command_seq=trajectory.command_seq,
        observation_id=trajectory.observation_id,
        accept_before=stamp(trajectory.accept_before_ns),
        trajectory=trajectory_message(trajectory),
        anchor=anchor,
        observation_time_ns=value.observation_time_ns,
        target_count=value.target_count,
        settle_at_end=value.settle_at_end,
        has_final_velocities=value.final_velocities is not None,
        final_velocities=[]
        if value.final_velocities is None
        else list(value.final_velocities),
    )


def native_command_from_message(value: NativeCommandMessage) -> NativeRequest:
    if not isinstance(value, NativeCommandMessage):
        raise ContractError("INVALID_NATIVE_REQUEST", "value is not a NativeCommand message")
    if value.kind not in (1, 2, 3):
        raise ContractError("INVALID_NATIVE_REQUEST", f"unknown kind: {value.kind}")
    joint_names, targets = trajectory_from_message(value.trajectory)
    trajectory = Trajectory(
        context_from_message(value.context),
        value.lease_id,
        value.command_id,
        value.command_seq,
        value.observation_id,
        time_ns(value.accept_before),
        joint_names,
        targets,
    )
    if _anchor_absent(value.anchor):
        anchor = None
    else:
        anchor = anchor_from_message(value.anchor)
    if not value.has_final_velocities:
        if value.final_velocities:
            raise ContractError(
                "INVALID_NATIVE_REQUEST",
                "has_final_velocities is false but final_velocities is not empty",
            )
        final_velocities = None
    else:
        final_velocities = tuple(value.final_velocities)
    return NativeRequest(
        trajectory=trajectory,
        kind=value.kind,
        anchor=anchor,
        observation_time_ns=value.observation_time_ns,
        target_count=value.target_count,
        settle_at_end=value.settle_at_end,
        final_velocities=final_velocities,
    )


def native_finger_message(value: NativeFinger) -> NativeFingerMessage:
    if not isinstance(value, NativeFinger):
        raise ContractError("INVALID_NATIVE_GRIPPER", "value must be a NativeFinger")
    return NativeFingerMessage(
        side=value.side,
        finger_index=value.finger_index,
        body_name=value.body_name,
        joint_speed_m_s=value.joint_speed_m_s,
        object_names=list(value.object_names),
        normal_forces=list(value.normal_forces),
        contact_counts=list(value.contact_counts),
    )


def native_finger_from_message(value: NativeFingerMessage) -> NativeFinger:
    return NativeFinger(
        side=value.side,
        finger_index=value.finger_index,
        body_name=value.body_name,
        joint_speed_m_s=value.joint_speed_m_s,
        object_names=tuple(value.object_names),
        normal_forces=tuple(value.normal_forces),
        contact_counts=tuple(value.contact_counts),
    )


def native_gripper_message(value: NativeGripper) -> NativeGripperMessage:
    if not isinstance(value, NativeGripper):
        raise ContractError("INVALID_NATIVE_GRIPPER", "value must be a NativeGripper")
    return NativeGripperMessage(
        side=value.side,
        mode=value.mode,
        position_ok=value.position_ok,
        contact_ok=value.contact_ok,
        contact_object="" if value.contact_object is None else value.contact_object,
        ambiguous=value.ambiguous,
        target_position_m=value.target_position_m,
        initial_reference_m=value.initial_reference_m,
        final_reference_m=value.final_reference_m,
        measured_position_m=value.measured_position_m,
        measured_velocity_m_s=value.measured_velocity_m_s,
        target_error_m=value.target_error_m,
        fingers=[native_finger_message(finger) for finger in value.fingers],
    )


def native_gripper_from_message(value: NativeGripperMessage) -> NativeGripper:
    return NativeGripper(
        side=value.side,
        mode=value.mode,
        position_ok=value.position_ok,
        contact_ok=value.contact_ok,
        contact_object=value.contact_object or None,
        ambiguous=value.ambiguous,
        target_position_m=value.target_position_m,
        initial_reference_m=value.initial_reference_m,
        final_reference_m=value.final_reference_m,
        measured_position_m=value.measured_position_m,
        measured_velocity_m_s=value.measured_velocity_m_s,
        target_error_m=value.target_error_m,
        fingers=tuple(native_finger_from_message(finger) for finger in value.fingers),
    )


def native_execution_message(value: NativeExecution) -> NativeExecutionMessage:
    if not isinstance(value, NativeExecution):
        raise ContractError("INVALID_NATIVE_EXECUTION", "value must be a NativeExecution")
    return NativeExecutionMessage(
        context=context_message(value.context),
        command_id=value.command_id,
        command_seq=value.command_seq,
        observation_id=value.observation_id,
        phase=value.phase,
        reason=value.reason,
        stop_reason=value.stop_reason,
        motion_seq=0 if value.motion_seq is None else value.motion_seq,
        stream_id=value.stream_id,
        chunk_seq=value.chunk_seq,
        accepted_sim_time_ns=value.accepted_sim_time_ns,
        accepted_wall_time_ns=value.accepted_wall_time_ns,
        has_started=value.has_started,
        started_sim_time_ns=0
        if value.started_sim_time_ns is None
        else value.started_sim_time_ns,
        started_wall_time_ns=0
        if value.started_wall_time_ns is None
        else value.started_wall_time_ns,
        has_finished=value.has_finished,
        finished_sim_time_ns=0
        if value.finished_sim_time_ns is None
        else value.finished_sim_time_ns,
        finished_wall_time_ns=0
        if value.finished_wall_time_ns is None
        else value.finished_wall_time_ns,
        motion_phase=value.motion_phase,
        reference_application_sim_time_ns=value.reference_application_sim_time_ns,
        measurement_sim_time_ns=value.measurement_sim_time_ns,
        reference_elapsed_ns=value.reference_elapsed_ns,
        stable_for_ns=value.stable_for_ns,
        joint_names=list(value.joint_names),
        reference_positions=list(value.reference_positions),
        reference_velocities=list(value.reference_velocities),
        reference_accelerations=list(value.reference_accelerations),
        measured_positions=list(value.measured_positions),
        measured_velocities=list(value.measured_velocities),
        position_errors=list(value.position_errors),
        velocity_errors=list(value.velocity_errors),
        reference_kind=value.reference_kind,
        tracking_quality=value.tracking_quality,
        tracking_violation_ns=value.tracking_violation_ns,
        grippers=[native_gripper_message(gripper) for gripper in value.grippers],
    )


def native_execution_from_message(value: NativeExecutionMessage) -> NativeExecution:
    if not isinstance(value, NativeExecutionMessage):
        raise ContractError("INVALID_NATIVE_EXECUTION", "value is not a NativeExecution message")
    if not value.has_started and (
        value.started_sim_time_ns or value.started_wall_time_ns
    ):
        raise ContractError(
            "INVALID_NATIVE_EXECUTION",
            "has_started is false but a started timestamp is nonzero",
        )
    if not value.has_finished and (
        value.finished_sim_time_ns or value.finished_wall_time_ns
    ):
        raise ContractError(
            "INVALID_NATIVE_EXECUTION",
            "has_finished is false but a finished timestamp is nonzero",
        )
    return NativeExecution(
        context=context_from_message(value.context),
        command_id=value.command_id,
        command_seq=value.command_seq,
        observation_id=value.observation_id,
        phase=value.phase,
        reason=value.reason,
        stop_reason=value.stop_reason,
        motion_seq=None if value.motion_seq == 0 else value.motion_seq,
        stream_id=value.stream_id,
        chunk_seq=value.chunk_seq,
        accepted_sim_time_ns=value.accepted_sim_time_ns,
        accepted_wall_time_ns=value.accepted_wall_time_ns,
        has_started=value.has_started,
        started_sim_time_ns=value.started_sim_time_ns if value.has_started else None,
        started_wall_time_ns=value.started_wall_time_ns if value.has_started else None,
        has_finished=value.has_finished,
        finished_sim_time_ns=value.finished_sim_time_ns if value.has_finished else None,
        finished_wall_time_ns=value.finished_wall_time_ns if value.has_finished else None,
        motion_phase=value.motion_phase,
        reference_application_sim_time_ns=value.reference_application_sim_time_ns,
        measurement_sim_time_ns=value.measurement_sim_time_ns,
        reference_elapsed_ns=value.reference_elapsed_ns,
        stable_for_ns=value.stable_for_ns,
        joint_names=tuple(value.joint_names),
        reference_positions=tuple(value.reference_positions),
        reference_velocities=tuple(value.reference_velocities),
        reference_accelerations=tuple(value.reference_accelerations),
        measured_positions=tuple(value.measured_positions),
        measured_velocities=tuple(value.measured_velocities),
        position_errors=tuple(value.position_errors),
        velocity_errors=tuple(value.velocity_errors),
        reference_kind=value.reference_kind,
        tracking_quality=value.tracking_quality,
        tracking_violation_ns=value.tracking_violation_ns,
        grippers=tuple(native_gripper_from_message(gripper) for gripper in value.grippers),
    )


def native_chunk_status_message(value: NativeChunkStatus) -> NativeChunkStatusMessage:
    if not isinstance(value, NativeChunkStatus):
        raise ContractError("INVALID_NATIVE_CHUNK", "value must be a NativeChunkStatus")
    return NativeChunkStatusMessage(
        context=context_message(value.context),
        command_id=value.command_id,
        command_seq=value.command_seq,
        chunk_seq=value.chunk_seq,
        observation_id=value.observation_id,
        phase=value.phase,
        reason=value.reason,
        accepted_sim_time_ns=value.accepted_sim_time_ns,
        accepted_wall_time_ns=value.accepted_wall_time_ns,
        has_started=value.has_started,
        started_sim_time_ns=0
        if value.started_sim_time_ns is None
        else value.started_sim_time_ns,
        started_wall_time_ns=0
        if value.started_wall_time_ns is None
        else value.started_wall_time_ns,
        has_finished=value.has_finished,
        finished_sim_time_ns=0
        if value.finished_sim_time_ns is None
        else value.finished_sim_time_ns,
        finished_wall_time_ns=0
        if value.finished_wall_time_ns is None
        else value.finished_wall_time_ns,
    )


def native_chunk_status_from_message(value: NativeChunkStatusMessage) -> NativeChunkStatus:
    if not isinstance(value, NativeChunkStatusMessage):
        raise ContractError("INVALID_NATIVE_CHUNK", "value is not a NativeChunkStatus message")
    if not value.has_started and (
        value.started_sim_time_ns or value.started_wall_time_ns
    ):
        raise ContractError(
            "INVALID_NATIVE_CHUNK",
            "has_started is false but a started timestamp is nonzero",
        )
    if not value.has_finished and (
        value.finished_sim_time_ns or value.finished_wall_time_ns
    ):
        raise ContractError(
            "INVALID_NATIVE_CHUNK",
            "has_finished is false but a finished timestamp is nonzero",
        )
    return NativeChunkStatus(
        context=context_from_message(value.context),
        command_id=value.command_id,
        command_seq=value.command_seq,
        chunk_seq=value.chunk_seq,
        observation_id=value.observation_id,
        phase=value.phase,
        reason=value.reason,
        accepted_sim_time_ns=value.accepted_sim_time_ns,
        accepted_wall_time_ns=value.accepted_wall_time_ns,
        has_started=value.has_started,
        started_sim_time_ns=value.started_sim_time_ns if value.has_started else None,
        started_wall_time_ns=value.started_wall_time_ns if value.has_started else None,
        has_finished=value.has_finished,
        finished_sim_time_ns=value.finished_sim_time_ns if value.has_finished else None,
        finished_wall_time_ns=value.finished_wall_time_ns if value.has_finished else None,
    )


def native_control_event_message(value: NativeControlEvent) -> NativeControlEventMessage:
    if not isinstance(value, NativeControlEvent):
        raise ContractError("INVALID_NATIVE_EVENT", "value must be a NativeControlEvent")
    return NativeControlEventMessage(
        context=context_message(value.context),
        event_seq=value.event_seq,
        command_id=value.command_id,
        command_seq=value.command_seq,
        observation_id=value.observation_id,
        is_chunk=value.is_chunk,
        chunk_seq=0 if value.chunk_seq is None else value.chunk_seq,
        phase=value.phase,
        reason=value.reason,
        sim_time_ns=value.sim_time_ns,
    )


def native_control_event_from_message(value: NativeControlEventMessage) -> NativeControlEvent:
    if not isinstance(value, NativeControlEventMessage):
        raise ContractError("INVALID_NATIVE_EVENT", "value is not a NativeControlEvent message")
    if not value.is_chunk and value.chunk_seq != 0:
        raise ContractError(
            "INVALID_NATIVE_EVENT", "is_chunk is false but chunk_seq is nonzero"
        )
    if value.is_chunk and value.chunk_seq == 0:
        raise ContractError(
            "INVALID_NATIVE_EVENT", "is_chunk is true but chunk_seq is zero"
        )
    return NativeControlEvent(
        context=context_from_message(value.context),
        event_seq=value.event_seq,
        command_id=value.command_id,
        command_seq=value.command_seq,
        observation_id=value.observation_id,
        is_chunk=value.is_chunk,
        chunk_seq=None if value.chunk_seq == 0 else value.chunk_seq,
        phase=value.phase,
        reason=value.reason,
        sim_time_ns=value.sim_time_ns,
    )
