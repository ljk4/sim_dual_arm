"""Planning and feedback adaptation for bounded-follower requests.

This module is the follower counterpart of :mod:`robot_backend_sim.protocol`.
It compiles one already validated raw :class:`StreamRequest` into the existing
``PreparedControlCommand`` envelope and freezes accepted evidence as a
:class:`FollowerReference`.  It never runs physics, reads a scene/clock or
publishes ROS messages, and it never mutates the frozen stream request.
"""

from __future__ import annotations

from robot_contracts.models import ContractError, RobotProfile
from robot_contracts.stream_request import REQUEST_STREAM_FIRST, REQUEST_STREAM_NEXT, StreamRequest
from robot_contracts.trajectory import (
    FollowerReference,
    FollowerValidationLimits,
    PreparedFollowerPlan,
)

from .control_state import ChunkStatus, ControlStatus, PreparedControlCommand
from .follower_config import PandaFollowerConfig
from .follower_plan import prepare_follower_plan
from .motion_config import PandaControlConfig

__all__ = [
    "follower_reference",
    "follower_reference_from_chunk_status",
    "follower_reference_from_status",
    "prepare_follower_request",
    "validation_limits",
]

_GRID_NS = 2_000_000


def _invalid(reason: str, detail: str) -> ContractError:
    return ContractError(reason, detail)


def validation_limits(
    control_config: PandaControlConfig,
    follower_config: PandaFollowerConfig,
    *,
    duration_grid_ns: int | None,
) -> FollowerValidationLimits:
    """Convert known backend configuration into the standard-library limits."""
    if not isinstance(control_config, PandaControlConfig):
        raise _invalid("INVALID_FOLLOWER_REQUEST", "control_config must be PandaControlConfig")
    if not isinstance(follower_config, PandaFollowerConfig):
        raise _invalid("INVALID_FOLLOWER_REQUEST", "follower_config must be PandaFollowerConfig")
    return FollowerValidationLimits(
        joint_names=follower_config.joint_names,
        max_accelerations=control_config.max_accelerations,
        max_target_deviations=follower_config.max_target_deviations,
        target_count=follower_config.target_count,
        max_terminal_tail_duration_ns=follower_config.max_terminal_tail_duration_ns,
        duration_grid_ns=duration_grid_ns,
    )


def prepare_follower_request(
    request: StreamRequest,
    profile: RobotProfile,
    control_config: PandaControlConfig,
    follower_config: PandaFollowerConfig,
    *,
    duration_grid_ns: int | None = _GRID_NS,
    is_final: bool | None = None,
) -> PreparedControlCommand:
    """Compile one raw follower request with exactly one Ruckig plan.

    ``request.kind`` must be ``STREAM_FIRST`` or ``STREAM_NEXT``.  By default
    ``request.settle_at_end`` is the explicit final-block flag: a true value
    asks the DSH-030 planner to extend to zero terminal velocity, exactly like a
    strict final stream chunk.  The caller may pass ``is_final`` explicitly when
    it has additional terminal knowledge; the two values must agree.
    """
    if not isinstance(request, StreamRequest):
        raise _invalid("INVALID_FOLLOWER_REQUEST", "request must be a StreamRequest")
    if request.kind not in (REQUEST_STREAM_FIRST, REQUEST_STREAM_NEXT):
        raise _invalid(
            "INVALID_FOLLOWER_REQUEST",
            "follower requests allow only STREAM_FIRST or STREAM_NEXT",
        )
    if request.anchor is None or request.window is None:
        raise _invalid("INVALID_FOLLOWER_REQUEST", "follower request needs an anchor/window")
    if not isinstance(profile, RobotProfile):
        raise _invalid("PROFILE_MISMATCH", "profile must be a RobotProfile")
    if not isinstance(control_config, PandaControlConfig):
        raise _invalid("INVALID_FOLLOWER_REQUEST", "control_config must be PandaControlConfig")
    if not isinstance(follower_config, PandaFollowerConfig):
        raise _invalid("INVALID_FOLLOWER_REQUEST", "follower_config must be PandaFollowerConfig")

    final = request.settle_at_end if is_final is None else is_final
    if not isinstance(final, bool):
        raise _invalid("INVALID_FOLLOWER_REQUEST", "is_final must be a bool")
    if final != request.settle_at_end:
        raise _invalid(
            "INVALID_FOLLOWER_REQUEST",
            "is_final must agree with the raw request settle_at_end flag",
        )
    if request.trajectory.joint_names != tuple(profile.joint_names):
        raise _invalid("PROFILE_MISMATCH", "request joint order differs from profile")
    if request.trajectory.context.robot_profile_hash != profile.digest:
        raise _invalid("PROFILE_MISMATCH", "request profile digest differs from profile")

    prepared = prepare_follower_plan(
        request.anchor,
        request.window,
        profile,
        control_config,
        follower_config,
        is_final=final,
        duration_grid_ns=duration_grid_ns,
    )
    return PreparedControlCommand(request.command, prepared, request=request)


def _check_request_packet(
    request: StreamRequest, packet: PreparedControlCommand
) -> PreparedFollowerPlan:
    if not isinstance(request, StreamRequest):
        raise _invalid("INVALID_STREAM_REQUEST", "request must be a StreamRequest")
    if request.kind not in (REQUEST_STREAM_FIRST, REQUEST_STREAM_NEXT):
        raise _invalid(
            "INVALID_FOLLOWER_REQUEST",
            "follower references allow only STREAM_FIRST or STREAM_NEXT",
        )
    if not isinstance(packet, PreparedControlCommand):
        raise _invalid("INVALID_PREPARED_COMMAND", "packet must be PreparedControlCommand")
    if packet.kind != "FOLLOWER" or not isinstance(packet.prepared, PreparedFollowerPlan):
        raise _invalid(
            "STREAM_MODE_MISMATCH",
            "packet is not a bounded-follower PreparedFollowerPlan",
        )
    if request.command != packet.command:
        raise _invalid(
            "REQUEST_PACKET_MISMATCH", "request identity differs from the compiled packet"
        )
    if packet.request is None or packet.request != request:
        raise _invalid(
            "REQUEST_PACKET_MISMATCH",
            "packet does not retain this complete raw follower request",
        )
    return packet.prepared


def _check_motion_seq(motion_seq: object) -> int:
    from numbers import Integral

    if isinstance(motion_seq, bool) or not isinstance(motion_seq, Integral):
        raise _invalid("INVALID_FOLLOWER_REFERENCE", "accepted fact lacks a motion_seq")
    value = int(motion_seq)
    if value < 1:
        raise _invalid("INVALID_FOLLOWER_REFERENCE", "accepted fact lacks a motion_seq")
    return value


def follower_reference(
    request: StreamRequest,
    packet: PreparedControlCommand,
    *,
    motion_seq: int,
    accepted_sim_time_ns: int,
    accepted_wall_time_ns: int,
) -> FollowerReference:
    """Build the canonical record for one explicitly supplied acceptance fact."""
    prepared = _check_request_packet(request, packet)
    seq = _check_motion_seq(motion_seq)
    return FollowerReference.from_prepared(
        prepared,
        request,
        motion_seq=seq,
        accepted_sim_time_ns=accepted_sim_time_ns,
        accepted_wall_time_ns=accepted_wall_time_ns,
    )


def follower_reference_from_status(
    request: StreamRequest,
    packet: PreparedControlCommand,
    status: ControlStatus,
) -> FollowerReference:
    """Rebuild a first-block receipt from the frozen overall acceptance fact."""
    prepared = _check_request_packet(request, packet)
    if not isinstance(status, ControlStatus):
        raise _invalid("INVALID_FOLLOWER_EXECUTION", "status must be a ControlStatus")
    if status.command_id != request.command.command_id:
        raise _invalid("REQUEST_PACKET_MISMATCH", "status command_id differs from request")
    if status.command_seq != request.command.command_seq:
        raise _invalid("REQUEST_PACKET_MISMATCH", "status command_seq differs from request")
    if status.motion_seq is None:
        raise _invalid("INVALID_FOLLOWER_REFERENCE", "accepted status lacks a motion_seq")
    if status.accepted_wall_ns is None:
        raise _invalid("INVALID_FOLLOWER_REFERENCE", "accepted status lacks a wall time")
    if status.chunk_seq is not None and status.chunk_seq != prepared.seq:
        raise _invalid("REQUEST_PACKET_MISMATCH", "status chunk_seq differs from the packet")
    return FollowerReference.from_prepared(
        prepared,
        request,
        motion_seq=status.motion_seq,
        accepted_sim_time_ns=status.accepted_sim_ns,
        accepted_wall_time_ns=status.accepted_wall_ns,
    )


def follower_reference_from_chunk_status(
    request: StreamRequest,
    packet: PreparedControlCommand,
    chunk: ChunkStatus,
    *,
    motion_seq: int,
) -> FollowerReference:
    """Rebuild one block receipt from its own frozen chunk acceptance fact.

    A stream keeps one ``motion_seq`` for the whole command, but every block has
    its own acceptance time.  A legal retry therefore returns the original
    acceptance time and digest instead of recomputing from later measurement.
    """
    prepared = _check_request_packet(request, packet)
    if not isinstance(chunk, ChunkStatus):
        raise _invalid("INVALID_FOLLOWER_REFERENCE", "chunk must be a ChunkStatus")
    if chunk.command_id != request.command.command_id:
        raise _invalid("REQUEST_PACKET_MISMATCH", "chunk command_id differs from request")
    if chunk.command_seq != request.command.command_seq:
        raise _invalid("REQUEST_PACKET_MISMATCH", "chunk command_seq differs from request")
    if chunk.chunk_seq != prepared.seq:
        raise _invalid("REQUEST_PACKET_MISMATCH", "chunk seq differs from the compiled packet")
    if chunk.accepted_wall_ns is None:
        raise _invalid("INVALID_FOLLOWER_REFERENCE", "chunk lacks an accepted wall time")
    return FollowerReference.from_prepared(
        prepared,
        request,
        motion_seq=_check_motion_seq(motion_seq),
        accepted_sim_time_ns=chunk.accepted_sim_ns,
        accepted_wall_time_ns=chunk.accepted_wall_ns,
    )
