"""Pure validation, anchor, timing and snapshot helpers for the motion loop.

Nothing here owns mutable state, reads a clock or steps physics; every helper
either validates immutable input or derives one value from explicit arguments.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from numbers import Integral, Real

from robot_contracts.action_chunks import ChunkAnchor
from robot_contracts.models import ContractError
from robot_contracts.trajectory.spline import ReferenceSample
from robot_trajectory.follower import PreparedFollowerPlan

from robot_backend_sim.common.config.motion_config import ControlConfig
from robot_backend_sim.common.config.stream_config import StreamConfig
from robot_backend_sim.common.motion.motion_types import BoundarySelection, Phase
from robot_backend_sim.common.motion.stream_schedule import (
    StreamSchedule,
    grid_aligned,
    next_grid_time,
    observation_rejection,
)
from robot_backend_sim.common.physics.robot_io import RobotIO
from robot_backend_sim.common.physics.scene_container import CompiledScene

__all__ = [
    "compute_active_chunk_anchor",
    "boundary_selection",
    "compute_idle_chunk_anchor",
    "deadline_decision",
    "finite_vector",
    "normalize_reference",
    "select_stream_reference",
    "sim_time_ns",
    "validate_controller_inputs",
    "validate_first_chunk",
    "zero_sample",
]

MAX_NS = 2**63 - 1


def invalid(reason: str, detail: str) -> ContractError:
    return ContractError(reason, detail)


def finite_vector(value: object, location: str, length: int) -> tuple[float, ...]:
    """Validate one fixed-length non-empty vector of finite real numbers."""
    if isinstance(value, str | bytes):
        raise invalid("INVALID_REFERENCE", f"{location}: must be an array of numbers")
    try:
        items = tuple(value)  # type: ignore[arg-type]
    except TypeError as error:
        raise invalid("INVALID_REFERENCE", f"{location}: must be an array of numbers") from error
    if len(items) != length:
        raise invalid(
            "INVALID_REFERENCE", f"{location}: expected {length} values, got {len(items)}"
        )
    result: list[float] = []
    for index, item in enumerate(items):
        if isinstance(item, bool) or not isinstance(item, Real):
            raise invalid(
                "INVALID_REFERENCE", f"{location}[{index}]: expected a finite real number"
            )
        try:
            number = float(item)
        except (OverflowError, TypeError, ValueError) as error:
            raise invalid("INVALID_REFERENCE", f"{location}[{index}]: not finite") from error
        if not math.isfinite(number):
            raise invalid("INVALID_REFERENCE", f"{location}[{index}]: must be finite")
        result.append(number)
    return tuple(result)


def zero_sample(positions: tuple[float, ...]) -> ReferenceSample:
    zero = (0.0,) * len(positions)
    return ReferenceSample(positions, zero, zero)


def sim_time_ns(data: object) -> int:
    value = data.time
    if isinstance(value, bool) or not isinstance(value, Real):
        raise invalid("INVALID_MEASUREMENT", "simulation time must be a real number")
    seconds = float(value)
    if not math.isfinite(seconds) or seconds < 0.0 or seconds > MAX_NS / 1e9:
        raise invalid("INVALID_MEASUREMENT", "simulation time is not a finite non-negative time")
    return round(seconds * 1_000_000_000)


def validate_controller_inputs(
    scene: CompiledScene,
    io: RobotIO,
    config: ControlConfig,
    stream_config: StreamConfig | None,
    initial_reference: Sequence[float] | None,
) -> tuple[int, tuple[float, ...]]:
    """Validate constructor inputs and return ``(step_ns, hold_positions)``."""
    if not isinstance(io, RobotIO):
        raise invalid("INVALID_MOTION", "io must implement the RobotIO contract")
    if not isinstance(config, ControlConfig):
        raise invalid("INVALID_MOTION", "config must be a ControlConfig")
    if not isinstance(scene, CompiledScene):
        raise invalid("INVALID_MOTION", "scene must be a CompiledScene")
    io.require_scene(scene)
    if tuple(io.joint_names) != config.joint_names:
        raise invalid("PROFILE_MISMATCH", "robot I/O joint order differs from the control config")
    if tuple(io.profile.joint_names) != config.joint_names:
        raise invalid(
            "PROFILE_MISMATCH", "robot profile joint order differs from the control config"
        )
    if not isinstance(io.profile.digest, str) or not io.profile.digest:
        raise invalid("PROFILE_MISMATCH", "robot profile digest is missing")
    if not scene.contact_object_names or any(
        not isinstance(name, str) or not name for name in scene.contact_object_names
    ):
        raise invalid("MODEL_MISMATCH", "scene must declare non-empty contact object names")
    timestep = float(scene.model.opt.timestep)
    if not math.isfinite(timestep) or timestep <= 0.0:
        raise invalid("MODEL_MISMATCH", "scene timestep must be finite and positive")
    step_ns = round(timestep * 1_000_000_000)
    if step_ns <= 0:
        raise invalid("MODEL_MISMATCH", "scene timestep must resolve to a positive grid")
    if stream_config is not None and not isinstance(stream_config, StreamConfig):
        raise invalid("INVALID_STREAM_CONFIG", "stream_config must be a StreamConfig")
    if stream_config is not None and stream_config.min_lead_ns < step_ns:
        raise invalid("INVALID_STREAM_CONFIG", "min_lead_s must be at least one physics step")
    default = io.profile.initial_positions
    hold = default if initial_reference is None else initial_reference
    hold_positions = finite_vector(hold, "initial_reference", len(config.joint_names))
    for index, (position, joint) in enumerate(
        zip(hold_positions, io.profile.joints, strict=True)
    ):
        if position < joint.minimum or position > joint.maximum:
            raise invalid(
                "JOINT_LIMIT",
                f"initial_reference[{index}] ({joint.name}) is outside profile limits",
            )
    return step_ns, hold_positions


def compute_idle_chunk_anchor(
    *,
    instance_id: str,
    motion_seq: int,
    hold_positions: tuple[float, ...],
    profile_digest: str,
    now_sim_ns: int,
    stream_config: StreamConfig,
    step_ns: int,
    start_backend_time_ns: int | None,
) -> ChunkAnchor:
    """Preview the next motion's first chunk from the current zero-velocity hold."""
    if start_backend_time_ns is None:
        start_ns = next_grid_time(now_sim_ns, stream_config.initial_lead_ns, step_ns)
    else:
        if isinstance(start_backend_time_ns, bool) or not isinstance(
            start_backend_time_ns, Integral
        ):
            raise invalid("INVALID_CHUNK_START", "start_backend_time_ns must be an integer")
        start_ns = int(start_backend_time_ns)
        if start_ns < now_sim_ns:
            raise invalid("INVALID_CHUNK_START", "start_backend_time_ns is in the past")
        if not grid_aligned(start_ns, step_ns):
            raise invalid("INVALID_CHUNK_START", "start_backend_time_ns is not on the physics grid")
    return ChunkAnchor(
        f"{instance_id}:motion-{motion_seq + 1}",
        0,
        start_ns,
        zero_sample(hold_positions),
        profile_digest,
    )


def compute_active_chunk_anchor(
    schedule: StreamSchedule,
    phase: Phase,
    start_backend_time_ns: int | None,
    profile_digest: str,
) -> ChunkAnchor:
    """Return the one successor anchor for a non-final active chunk."""
    if schedule.active is None:
        raise invalid("STREAM_NOT_STARTED", "stream has not applied its first chunk")
    if phase is Phase.SETTLING or schedule.active.settle_at_end:
        raise invalid("STREAM_FINAL_CHUNK", "final settling chunk has no successor")
    if schedule.pending is not None:
        raise invalid("STREAM_PENDING_EXISTS", "one successor is already pending")
    active = schedule.active
    end_ns = active.end_backend_time_ns
    if start_backend_time_ns is not None:
        if isinstance(start_backend_time_ns, bool) or not isinstance(
            start_backend_time_ns, Integral
        ):
            raise invalid("INVALID_CHUNK_START", "start_backend_time_ns must be an integer")
        if int(start_backend_time_ns) != end_ns:
            raise invalid(
                "INVALID_CHUNK_START", "explicit start does not equal the fixed active chunk end"
            )
    return ChunkAnchor(
        schedule.stream_id, active.seq, end_ns, active.sample_at(end_ns), profile_digest
    )


def validate_first_chunk(
    first: PreparedFollowerPlan,
    *,
    phase: Phase,
    instance_id: str,
    motion_seq: int,
    stream_config: StreamConfig,
    step_ns: int,
    now_sim_ns: int,
    hold_positions: tuple[float, ...],
    profile_digest: str,
    profile_joints: Sequence[str],
    max_accelerations: tuple[float, ...],
) -> None:
    """Raise the original stable error for every inadmissible first chunk."""
    if not isinstance(first, PreparedFollowerPlan):
        raise invalid("INVALID_PREPARED_CHUNK", "first must be a PreparedFollowerPlan")
    if phase in (
        Phase.ACCEPTED,
        Phase.RUNNING,
        Phase.SETTLING,
        Phase.STOPPING,
    ):
        raise invalid("MOTION_ACTIVE", "cannot overlap active/settling/stopping motion")
    if phase is Phase.FAILED:
        raise invalid("FAILED_LATCHED", "a failed motion must be reset before a new start")
    expected_stream_id = f"{instance_id}:motion-{motion_seq + 1}"
    if first.anchor.stream_id != expected_stream_id:
        raise invalid(
            "STALE_STREAM", "stream id is not the next generation of this motion instance"
        )
    if first.anchor.predecessor_seq != 0:
        raise invalid("INVALID_CHUNK_ORDER", "first chunk predecessor_seq must be zero")
    if first.anchor.robot_profile_hash != profile_digest:
        raise invalid("PROFILE_MISMATCH", "chunk profile digest differs from the current profile")
    if tuple(first.joints) != tuple(profile_joints):
        raise invalid("PROFILE_MISMATCH", "chunk joints differ from the current profile")
    if tuple(first.max_accelerations) != tuple(max_accelerations):
        raise invalid("PROFILE_MISMATCH", "chunk accelerations differ from the current config")
    if not grid_aligned(first.start_backend_time_ns, step_ns):
        raise invalid("INVALID_CHUNK_START", "chunk start is not on the physics grid")
    if not grid_aligned(first.end_backend_time_ns, step_ns):
        raise invalid("INVALID_CHUNK_GRID", "chunk end is not on the physics grid")
    if first.duration_grid_ns != step_ns:
        raise invalid(
            "FOLLOWER_GRID_REQUIRED",
            "physical follower chunks must be prepared on the scene physics grid",
        )
    if first.plan.duration_ns % step_ns != 0:
        raise invalid(
            "INVALID_CHUNK_GRID", "follower plan duration is not a multiple of the physics step"
        )
    if first.start_backend_time_ns < now_sim_ns:
        raise invalid("INVALID_CHUNK_START", "chunk start is in the past")
    observation = observation_rejection(
        first.window, now_sim_ns, stream_config.max_observation_age_ns
    )
    if observation is not None:
        raise invalid(observation, "first chunk observation is not admissible")
    if now_sim_ns > first.start_backend_time_ns - stream_config.min_lead_ns:
        raise invalid("CHUNK_LATE", "first chunk missed the admission lead deadline")
    for index, (initial, hold) in enumerate(
        zip(first.anchor.reference.positions, hold_positions, strict=True)
    ):
        if not math.isclose(initial, hold, rel_tol=0.0, abs_tol=1e-9):
            raise invalid(
                "INITIAL_REFERENCE_MISMATCH",
                f"chunk initial reference[{index}] does not match the current hold",
            )
    for value in first.anchor.reference.velocities:
        if not math.isclose(value, 0.0, rel_tol=0.0, abs_tol=1e-9):
            raise invalid(
                "INITIAL_REFERENCE_MISMATCH",
                "first chunk must start from a zero-velocity hold",
            )



def boundary_selection(
    schedule: StreamSchedule, active: PreparedFollowerPlan, application_sim_ns: int
) -> BoundarySelection:
    """Choose the endpoint, successor start, or underrun endpoint reference."""
    end_reference = active.sample_at(application_sim_ns)
    if active.settle_at_end:
        return BoundarySelection(end_reference, active.duration_ns, "CHUNK", active.seq, "settle")
    if schedule.pending is not None:
        successor = schedule.pending
        return BoundarySelection(
            successor.sample_at(application_sim_ns), 0, "CHUNK", successor.seq, "handoff"
        )
    return BoundarySelection(end_reference, active.duration_ns, "CHUNK", active.seq, "underrun")


@dataclass(frozen=True, slots=True)
class StreamReference:
    """One chosen pre-step reference; ``reference is None`` means underrun."""

    reference: ReferenceSample | None
    elapsed_ns: int
    reference_kind: str
    chunk_seq: int | None
    boundary: BoundarySelection | None = None


def select_stream_reference(
    schedule: StreamSchedule,
    phase: Phase,
    application_sim_ns: int,
    hold_positions: tuple[float, ...],
    final_positions: tuple[float, ...] | None,
) -> StreamReference:
    """Choose the reference for one active-chunk application instant."""
    active = schedule.active
    assert active is not None
    if phase is Phase.SETTLING:
        assert final_positions is not None
        return StreamReference(
            zero_sample(final_positions),
            application_sim_ns - active.start_backend_time_ns,
            "HOLD",
            active.seq,
        )
    if application_sim_ns > active.end_backend_time_ns:
        return StreamReference(None, 0, "BRAKE", active.seq)
    if application_sim_ns == active.end_backend_time_ns:
        boundary = boundary_selection(schedule, active, application_sim_ns)
        return StreamReference(
            boundary.reference,
            boundary.elapsed_ns,
            boundary.reference_kind,
            boundary.chunk_seq,
            boundary,
        )
    if application_sim_ns < active.start_backend_time_ns:
        return StreamReference(zero_sample(hold_positions), 0, "HOLD", active.seq)
    return StreamReference(
        active.sample_at(application_sim_ns),
        application_sim_ns - active.start_backend_time_ns,
        "CHUNK",
        active.seq,
    )


def deadline_decision(
    *,
    phase: Phase,
    wall: int,
    config: ControlConfig,
    stream_config: StreamConfig | None,
    accepted_wall_ns: int | None,
    stop_requested_wall_ns: int | None,
    last_progress_wall_ns: int | None,
    last_progress_sim_ns: int | None,
    now_sim_ns: int,
    applied_measurement_sim_ns: int | None,
    final_end_ns: int | None,
) -> tuple[str, str, str | None, bool] | None:
    """Return ``(action, reason, latched, exact_fail_return)`` or ``None``."""
    if phase in (Phase.ACCEPTED, Phase.RUNNING, Phase.SETTLING):
        assert accepted_wall_ns is not None and stream_config is not None
        if wall - accepted_wall_ns >= stream_config.stream_wall_timeout_ns:
            return "stop", "WALL_TIMEOUT", "WALL_TIMEOUT", False
        if (
            last_progress_wall_ns is not None
            and last_progress_sim_ns is not None
            and wall - last_progress_wall_ns >= stream_config.progress_wall_timeout_ns
            and now_sim_ns == last_progress_sim_ns
        ):
            return "stop", "PROGRESS_TIMEOUT", "PROGRESS_TIMEOUT", False
        if (
            phase in (Phase.RUNNING, Phase.SETTLING)
            and applied_measurement_sim_ns is not None
            and final_end_ns is not None
            and applied_measurement_sim_ns >= final_end_ns + config.settle_timeout_ns
        ):
            return "stop", "TRACKING_TIMEOUT", "TRACKING_TIMEOUT", True
    if phase is Phase.STOPPING:
        assert stop_requested_wall_ns is not None
        if wall - stop_requested_wall_ns >= config.stop_wall_timeout_ns:
            return "fail", "STOP_TIMEOUT", None, True
    return None


def normalize_reference(
    reference: ReferenceSample, joints: tuple[str, ...]
) -> tuple[ReferenceSample, tuple[float, ...]]:
    """Apply the shared follower boundary normalization to one reference."""
    from robot_trajectory.follower import normalize_follower_positions

    normalized = normalize_follower_positions(reference.positions, joints)
    if not normalized.has_correction:
        return reference, ()
    return (
        ReferenceSample(
            normalized.applied_positions, reference.velocities, reference.accelerations
        ),
        normalized.corrections,
    )


def validate_wall(value: object, previous_wall_ns: int | None) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise ContractError("INVALID_WALL_TIME", "wall_time_ns must be an integer")
    result = int(value)
    if result < 0 or result > MAX_NS:
        raise ContractError("INVALID_WALL_TIME", "wall_time_ns is outside the supported range")
    if previous_wall_ns is not None and result < previous_wall_ns:
        raise ContractError("WALL_TIME_REGRESSION", "wall_time_ns must not move backwards")
    return result
