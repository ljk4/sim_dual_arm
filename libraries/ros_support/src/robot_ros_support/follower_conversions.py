"""Explicit ROS conversions for the independent bounded-follower record.

Every field is copied by name and revalidated by the shared
:class:`robot_contracts.follower_reference.FollowerReference` constructor.  The
record is never converted into a strict ``AcceptedReference``.
"""

from __future__ import annotations

from robot_api_msgs.msg import FollowerReference as FollowerReferenceMessage
from robot_api_msgs.msg import ReferenceSegment as ReferenceSegmentMessage
from robot_contracts.action_chunks import ActionWindow
from robot_contracts.follower_reference import FollowerReference
from robot_contracts.models import ContractError
from robot_contracts.native import plan_from_segments
from robot_contracts.reference import CubicSegment

from .native_conversions import (
    anchor_from_message,
    anchor_message,
    native_command_from_message,
    native_command_message,
)

__all__ = [
    "follower_reference_from_message",
    "follower_reference_message",
]


def _invalid(detail: str) -> ContractError:
    return ContractError("INVALID_FOLLOWER_REFERENCE", detail)


def _segment_message(segment: CubicSegment) -> ReferenceSegmentMessage:
    return ReferenceSegmentMessage(
        duration_ns=segment.duration_ns,
        c0=list(segment.c0),
        c1=list(segment.c1),
        c2=list(segment.c2),
        c3=list(segment.c3),
    )


def follower_reference_message(value: FollowerReference) -> FollowerReferenceMessage:
    """Copy one accepted follower record to the wire without re-deriving it."""
    if not isinstance(value, FollowerReference):
        raise _invalid("value must be a FollowerReference")
    return FollowerReferenceMessage(
        request=native_command_message(value.request),
        motion_seq=value.motion_seq,
        accepted_sim_time_ns=value.accepted_sim_time_ns,
        accepted_wall_time_ns=value.accepted_wall_time_ns,
        joint_names=list(value.joint_names),
        anchor=anchor_message(value.anchor),
        observation_id=value.window.observation_id,
        observation_time_ns=value.window.observation_time_ns,
        handoff_time_ns=value.window.handoff_time_ns,
        target_count=value.window.target_count,
        selected_indices=list(value.window.selected_indices),
        segments=[_segment_message(segment) for segment in value.plan.segments],
        raw_terminal_velocities=list(value.raw_terminal_velocities),
        restricted_terminal_velocities=list(value.restricted_terminal_velocities),
        actual_terminal_positions=list(value.actual_terminal_positions),
        actual_terminal_velocities=list(value.actual_terminal_velocities),
        actual_terminal_accelerations=list(value.actual_terminal_accelerations),
        max_accelerations=list(value.max_accelerations),
        max_target_deviation_limits=list(value.max_target_deviation_limits),
        max_target_deviations=list(value.max_target_deviations),
        is_final=value.is_final,
        requested_window_ns=value.requested_window_ns,
        ruckig_duration_ns=value.ruckig_duration_ns,
        has_duration_grid=value.duration_grid_ns is not None,
        duration_grid_ns=0 if value.duration_grid_ns is None else value.duration_grid_ns,
        extension_ns=value.extension_ns,
        velocity_limited=list(value.velocity_limited),
        acceleration_limited=list(value.acceleration_limited),
        boundary_preserved=list(value.boundary_preserved),
        algorithm_version=value.algorithm_version,
        sampler_version=value.sampler_version,
        normalization_version=value.normalization_version,
        reference_digest=value.reference_digest or "",
    )


def follower_reference_from_message(value: object) -> FollowerReference:
    """Rebuild and fully re-verify one wire record."""
    if not isinstance(value, FollowerReferenceMessage):
        raise _invalid("value is not a FollowerReference message")
    if not value.has_duration_grid and value.duration_grid_ns != 0:
        raise _invalid("has_duration_grid is false but duration_grid_ns is nonzero")
    request = native_command_from_message(value.request)
    if request.anchor is None or request.window is None:
        raise _invalid("request lacks an anchor/window")
    if tuple(value.joint_names) != tuple(request.trajectory.joint_names):
        raise _invalid("joint_names differ from the raw request")
    anchor = anchor_from_message(value.anchor)
    window = ActionWindow(
        observation_id=value.observation_id,
        observation_time_ns=value.observation_time_ns,
        handoff_time_ns=value.handoff_time_ns,
        source_targets=request.trajectory.targets,
        target_count=value.target_count,
    )
    if tuple(value.selected_indices) != window.selected_indices:
        raise _invalid("selected_indices differ from the recomputed raw window")
    if request.window != window:
        raise _invalid("window fields differ from the raw request")
    if request.anchor != anchor:
        raise _invalid("anchor fields differ from the raw request")
    segments = tuple(
        CubicSegment(
            duration_ns=segment.duration_ns,
            c0=tuple(segment.c0),
            c1=tuple(segment.c1),
            c2=tuple(segment.c2),
            c3=tuple(segment.c3),
        )
        for segment in value.segments
    )
    plan = plan_from_segments(tuple(value.joint_names), segments)
    reference = FollowerReference(
        request=request,
        motion_seq=value.motion_seq,
        accepted_sim_time_ns=value.accepted_sim_time_ns,
        accepted_wall_time_ns=value.accepted_wall_time_ns,
        joint_names=tuple(value.joint_names),
        plan=plan,
        anchor=anchor,
        window=window,
        max_accelerations=tuple(value.max_accelerations),
        max_target_deviation_limits=tuple(value.max_target_deviation_limits),
        raw_terminal_velocities=tuple(value.raw_terminal_velocities),
        restricted_terminal_velocities=tuple(value.restricted_terminal_velocities),
        actual_terminal_positions=tuple(value.actual_terminal_positions),
        actual_terminal_velocities=tuple(value.actual_terminal_velocities),
        actual_terminal_accelerations=tuple(value.actual_terminal_accelerations),
        max_target_deviations=tuple(value.max_target_deviations),
        is_final=value.is_final,
        requested_window_ns=value.requested_window_ns,
        ruckig_duration_ns=value.ruckig_duration_ns,
        duration_grid_ns=value.duration_grid_ns
        if value.has_duration_grid
        else None,
        velocity_limited=tuple(value.velocity_limited),
        acceleration_limited=tuple(value.acceleration_limited),
        boundary_preserved=tuple(value.boundary_preserved),
        algorithm_version=value.algorithm_version,
        sampler_version=value.sampler_version,
        normalization_version=value.normalization_version,
        reference_digest=value.reference_digest or None,
    )
    if value.extension_ns != reference.extension_ns:
        raise _invalid("extension_ns differs from the retained durations")
    return reference
