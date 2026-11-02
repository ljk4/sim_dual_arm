"""Canonical payload and digest for bounded-follower reference records.

This module owns only the versioned canonical document and SHA-256 computation.
It is standard-library-only and imports the record type lazily for the type
check, so the record module can import these functions without a cycle.
"""

from __future__ import annotations

import hashlib
import json
from typing import TYPE_CHECKING

from .action_chunks import ActionWindow, ChunkAnchor
from .models import ContractError, Target
from .native import NativeRequest
from .reference import CubicSegment

if TYPE_CHECKING:  # pragma: no cover - import for static type checking only
    from .follower_reference import FollowerReference

FOLLOWER_REFERENCE_DIGEST_VERSION = "follower-reference-v1"
DIGEST_HEX_LENGTH = 64


def _invalid(location: str, message: str) -> ContractError:
    return ContractError("INVALID_FOLLOWER_REFERENCE", f"{location}: {message}")


def segment_payload(segment: CubicSegment) -> dict[str, object]:
    return {
        "duration_ns": int(segment.duration_ns),
        "c0": list(segment.c0),
        "c1": list(segment.c1),
        "c2": list(segment.c2),
        "c3": list(segment.c3),
    }


def _target_payload(target: Target) -> dict[str, object]:
    return {
        "time_ns": int(target.time_ns),
        "positions": list(target.positions),
    }


def request_payload(request: NativeRequest) -> dict[str, object]:
    """Canonical copy of the complete raw request including the unused tail."""
    trajectory = request.trajectory
    context = trajectory.context
    anchor = request.anchor
    return {
        "kind": request.kind,
        "trajectory": {
            "context": {
                "robot_id": context.robot_id,
                "robot_profile_hash": context.robot_profile_hash,
                "backend_instance_id": context.backend_instance_id,
                "time_domain_id": context.time_domain_id,
                "clock_epoch": int(context.clock_epoch),
                "episode_id": context.episode_id,
                "control_epoch": int(context.control_epoch),
            },
            "lease_id": trajectory.lease_id,
            "command_id": trajectory.command_id,
            "command_seq": int(trajectory.command_seq),
            "observation_id": int(trajectory.observation_id),
            "accept_before_ns": int(trajectory.accept_before_ns),
            "joint_names": list(trajectory.joint_names),
            "targets": [_target_payload(target) for target in trajectory.targets],
        },
        "anchor": None
        if anchor is None
        else {
            "stream_id": anchor.stream_id,
            "predecessor_seq": int(anchor.predecessor_seq),
            "start_sim_time_ns": int(anchor.start_sim_time_ns),
            "robot_profile_hash": anchor.robot_profile_hash,
            "positions": list(anchor.reference.positions),
            "velocities": list(anchor.reference.velocities),
            "accelerations": list(anchor.reference.accelerations),
        },
        "observation_time_ns": int(request.observation_time_ns),
        "target_count": int(request.target_count),
        "settle_at_end": bool(request.settle_at_end),
        "final_velocities": None
        if request.final_velocities is None
        else list(request.final_velocities),
    }


def window_payload(window: ActionWindow) -> dict[str, object]:
    return {
        "observation_id": int(window.observation_id),
        "observation_time_ns": int(window.observation_time_ns),
        "handoff_time_ns": int(window.handoff_time_ns),
        "target_count": int(window.target_count),
        "selected_indices": [int(value) for value in window.selected_indices],
        "source_targets": [_target_payload(target) for target in window.source_targets],
    }


def anchor_payload(anchor: ChunkAnchor) -> dict[str, object]:
    return {
        "stream_id": anchor.stream_id,
        "predecessor_seq": int(anchor.predecessor_seq),
        "start_sim_time_ns": int(anchor.start_sim_time_ns),
        "robot_profile_hash": anchor.robot_profile_hash,
        "positions": list(anchor.reference.positions),
        "velocities": list(anchor.reference.velocities),
        "accelerations": list(anchor.reference.accelerations),
    }


def follower_reference_digest_payload(record: FollowerReference) -> dict[str, object]:
    """Return the complete canonical document covered by the record digest."""
    return {
        "digest_version": FOLLOWER_REFERENCE_DIGEST_VERSION,
        "request": request_payload(record.request),
        "motion_seq": record.motion_seq,
        "accepted_sim_time_ns": record.accepted_sim_time_ns,
        "accepted_wall_time_ns": record.accepted_wall_time_ns,
        "joint_names": list(record.joint_names),
        "anchor": anchor_payload(record.anchor),
        "window": window_payload(record.window),
        "plan": {
            "duration_ns": record.plan.duration_ns,
            "segments": [segment_payload(segment) for segment in record.plan.segments],
        },
        "max_accelerations": list(record.max_accelerations),
        "max_target_deviation_limits": list(record.max_target_deviation_limits),
        "raw_terminal_velocities": list(record.raw_terminal_velocities),
        "restricted_terminal_velocities": list(record.restricted_terminal_velocities),
        "actual_terminal_positions": list(record.actual_terminal_positions),
        "actual_terminal_velocities": list(record.actual_terminal_velocities),
        "actual_terminal_accelerations": list(record.actual_terminal_accelerations),
        "max_target_deviations": list(record.max_target_deviations),
        "is_final": record.is_final,
        "requested_window_ns": record.requested_window_ns,
        "ruckig_duration_ns": record.ruckig_duration_ns,
        "duration_grid_ns": record.duration_grid_ns,
        "extension_ns": record.extension_ns,
        "velocity_limited": list(record.velocity_limited),
        "acceleration_limited": list(record.acceleration_limited),
        "boundary_preserved": list(record.boundary_preserved),
        "algorithm_version": record.algorithm_version,
        "sampler_version": record.sampler_version,
        "normalization_version": record.normalization_version,
    }


def compute_follower_reference_digest(record: FollowerReference) -> str:
    """SHA-256 over every protected identity/source/curve/limit/time field."""
    from .follower_reference import FollowerReference

    if not isinstance(record, FollowerReference):
        raise _invalid("record", "must be a FollowerReference")
    try:
        encoded = json.dumps(
            follower_reference_digest_payload(record),
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
            allow_nan=False,
        ).encode("ascii")
    except (TypeError, ValueError) as error:  # pragma: no cover - constructor validates
        raise _invalid("reference_digest", "record is not finite JSON") from error
    return hashlib.sha256(encoded).hexdigest()


__all__ = [
    "DIGEST_HEX_LENGTH",
    "FOLLOWER_REFERENCE_DIGEST_VERSION",
    "anchor_payload",
    "compute_follower_reference_digest",
    "follower_reference_digest_payload",
    "request_payload",
    "segment_payload",
    "window_payload",
]
