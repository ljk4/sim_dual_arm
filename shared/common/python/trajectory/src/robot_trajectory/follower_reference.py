"""Follower record sampling, capture and independent numerical validation."""
from __future__ import annotations

import math
from collections.abc import Sequence

from robot_contracts.models import ContractError, JointSpec, RobotProfile
from robot_contracts.stream_request import StreamRequest
from robot_contracts.trajectory.follower_reference import (
    _DEVIATION_ATOL,
    FollowerAppliedSample,
    FollowerReference,
    FollowerValidationLimits,
    _invalid,
)
from robot_contracts.trajectory.spline import ReferenceSample

from robot_trajectory.sampling import sample_reference_interval

from .follower import PreparedFollowerPlan, normalize_follower_positions


def applied_follower_sample(
    record: FollowerReference,
    sim_time_ns: int,
    profile_or_joints: RobotProfile | Sequence[JointSpec],
) -> FollowerAppliedSample:
    """Reconstruct the physically applied reference and its corrections."""
    joints = _joints_from(profile_or_joints)
    sample = sample_follower_reference(record, sim_time_ns)
    normalized = normalize_follower_positions(sample.positions, joints)
    return FollowerAppliedSample(
        sampled_positions=normalized.sampled_positions,
        applied_positions=normalized.applied_positions,
        corrections=normalized.corrections,
        sampled_velocities=sample.velocities,
        sampled_accelerations=sample.accelerations,
    )

def sample_follower_reference(record: FollowerReference, sim_time_ns: int) -> ReferenceSample:
    if isinstance(sim_time_ns, bool) or not isinstance(sim_time_ns, int):
        raise _invalid("sim_time_ns", "must be an integer")
    if sim_time_ns < record.start_sim_time_ns or sim_time_ns > record.end_sim_time_ns:
        raise ContractError(
            "REFERENCE_EXPIRED",
            f"sim_time_ns {sim_time_ns} is outside "
            f"[{record.start_sim_time_ns}, {record.end_sim_time_ns}]")
    return sample_reference_interval(record.plan, sim_time_ns - record.start_sim_time_ns)

def follower_reference_from_prepared(
    prepared: PreparedFollowerPlan,
    request: StreamRequest,
    *,
    motion_seq: int,
    accepted_sim_time_ns: int,
    accepted_wall_time_ns: int,
    joint_names: Sequence[str] | None = None,
) -> FollowerReference:
    """Capture one prepared follower plan as an immutable accepted record."""
    if not isinstance(prepared, PreparedFollowerPlan):
        raise _invalid("prepared", "must be a PreparedFollowerPlan")
    if not isinstance(request, StreamRequest):
        raise _invalid("request", "must be a StreamRequest")
    names = prepared.joint_names if joint_names is None else tuple(joint_names)
    return FollowerReference(
        request=request,
        motion_seq=motion_seq,
        accepted_sim_time_ns=accepted_sim_time_ns,
        accepted_wall_time_ns=accepted_wall_time_ns,
        joint_names=names,
        plan=prepared.plan,
        anchor=prepared.anchor,
        window=prepared.window,
        max_accelerations=prepared.max_accelerations,
        max_target_deviation_limits=prepared.max_target_deviation_limits,
        raw_terminal_velocities=prepared.raw_terminal_velocities,
        restricted_terminal_velocities=prepared.restricted_terminal_velocities,
        actual_terminal_positions=prepared.actual_terminal_positions,
        actual_terminal_velocities=prepared.actual_terminal_velocities,
        actual_terminal_accelerations=prepared.actual_terminal_accelerations,
        max_target_deviations=prepared.max_target_deviations,
        is_final=prepared.is_final,
        requested_window_ns=prepared.requested_window_ns,
        curve_duration_ns=prepared.curve_duration_ns,
        duration_grid_ns=prepared.duration_grid_ns,
        algorithm_version=prepared.algorithm_version,
        velocity_limited=prepared.flags.velocity_limited,
        acceleration_limited=prepared.flags.acceleration_limited,
        boundary_preserved=prepared.flags.boundary_preserved,
    )

def _joints_from(profile_or_joints: RobotProfile | Sequence[JointSpec]) -> tuple[JointSpec, ...]:
    if isinstance(profile_or_joints, RobotProfile):
        return tuple(profile_or_joints.joints)
    joints = tuple(profile_or_joints)
    if not joints or any(not isinstance(joint, JointSpec) for joint in joints):
        raise _invalid("joints", "must contain JointSpec values")
    return joints

def _same_vector(left: Sequence[float], right: Sequence[float]) -> bool:
    return len(left) == len(right) and all(
        math.isclose(a, b, rel_tol=0.0, abs_tol=_DEVIATION_ATOL)
        for a, b in zip(left, right, strict=True)
    )

def validate(record: FollowerReference, profile: RobotProfile,
             limits: FollowerValidationLimits) -> PreparedFollowerPlan:
    if not isinstance(record, FollowerReference):
        raise _invalid("record", "must be a FollowerReference")
    if not isinstance(profile, RobotProfile):
        raise ContractError("PROFILE_MISMATCH", "profile must be a RobotProfile")
    if not isinstance(limits, FollowerValidationLimits):
        raise _invalid("limits", "must be a FollowerValidationLimits")
    if limits.joint_names != tuple(profile.joint_names):
        raise ContractError("PROFILE_MISMATCH", "limits joint order differs from profile")
    if record.joint_names != limits.joint_names:
        raise ContractError("PROFILE_MISMATCH", "record joint order differs from limits")
    if record.anchor.robot_profile_hash != profile.digest:
        raise ContractError("PROFILE_MISMATCH", "record anchor profile digest differs")
    if not _same_vector(limits.max_accelerations, record.max_accelerations):
        raise ContractError("FOLLOWER_LIMIT_MISMATCH",
                            "record max_accelerations differ from the validation limits")
    if not _same_vector(limits.max_target_deviations, record.max_target_deviation_limits):
        raise ContractError("FOLLOWER_LIMIT_MISMATCH",
                            "record deviation limits differ from the validation limits")
    if record.window.target_count != limits.target_count:
        raise ContractError("FOLLOWER_LIMIT_MISMATCH",
                            "record window.target_count differs from the validation limits")
    if record.request.target_count != limits.target_count:
        raise ContractError("FOLLOWER_LIMIT_MISMATCH",
                            "raw request target_count differs from the validation limits")
    if record.duration_grid_ns != limits.duration_grid_ns:
        raise ContractError("FOLLOWER_GRID_MISMATCH",
                            "record duration grid differs from the validation limits")
    if (record.requested_window_ns > limits.max_terminal_tail_duration_ns
            or record.plan.duration_ns > limits.max_terminal_tail_duration_ns):
        raise ContractError("FOLLOWER_HORIZON_EXCEEDED",
                            "record horizon exceeds the configured terminal tail")
    if record.is_final != record.request.settle_at_end:
        raise _invalid("is_final", "differs from the raw request settle_at_end flag")
    if not (record.request.observation_time_ns <= record.accepted_sim_time_ns
            <= record.anchor.start_sim_time_ns
            and record.accepted_sim_time_ns <= record.request.trajectory.accept_before_ns):
        raise _invalid("accepted_sim_time_ns",
                       "must follow the observation and precede both deadlines")
    rebuilt = PreparedFollowerPlan(
        anchor=record.anchor,
        window=record.window,
        plan=record.plan,
        joints=tuple(profile.joints),
        max_accelerations=record.max_accelerations,
        max_target_deviation_limits=record.max_target_deviation_limits,
        raw_terminal_velocities=record.raw_terminal_velocities,
        restricted_terminal_velocities=record.restricted_terminal_velocities,
        is_final=record.is_final,
        requested_window_ns=record.requested_window_ns,
        curve_duration_ns=record.curve_duration_ns,
        duration_grid_ns=record.duration_grid_ns,
        algorithm_version=record.algorithm_version,
    )
    if rebuilt.plan != record.plan:
        raise _invalid("plan", "reconstructed plan differs")
    for name in (
        "actual_terminal_positions", "actual_terminal_velocities",
        "actual_terminal_accelerations", "max_target_deviations",
        "raw_terminal_velocities", "restricted_terminal_velocities",
    ):
        if not _same_vector(getattr(record, name), getattr(rebuilt, name)):
            raise _invalid(name, "differs from the independently rebuilt DSH-030 plan")
    if record.flags != rebuilt.flags:
        raise _invalid("flags", "limit flags differ from the independently rebuilt plan")
    if record.extension_ns != rebuilt.extension_ns:
        raise _invalid("extension_ns", "extended horizon differs from the rebuilt plan")
    return rebuilt
