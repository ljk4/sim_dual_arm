"""Explicit transition helpers shared by the motion controller.

These functions only assemble immutable facts from values the controller has
already measured.  They never step physics, mutate a schedule or read a clock.
"""

from __future__ import annotations

from dataclasses import dataclass, replace

from robot_contracts.action_chunks import ChunkAnchor
from robot_trajectory.sampling import sample_reference_interval

from robot_backend_sim.config.stream_config import StreamConfig
from robot_backend_sim.motion.motion_math import (
    compute_active_chunk_anchor,
    compute_idle_chunk_anchor,
    invalid,
    zero_sample,
)
from robot_backend_sim.motion.motion_snapshot import SnapshotContext, Snapshotter
from robot_backend_sim.motion.motion_types import (
    ACTIVE_PHASES,
    TERMINAL_PHASES,
    AppliedStep,
    MotionResult,
    MotionSnapshot,
    Phase,
)
from robot_backend_sim.motion.stream_schedule import StreamSchedule
from robot_backend_sim.physics.robot_io import Measurement

__all__ = [
    "failed_terminal",
    "resolve_chunk_anchor",
    "snapshot_for_call",
    "stop_terminal",
]


def resolve_chunk_anchor(
    *,
    phase: Phase,
    schedule: StreamSchedule | None,
    stream_config: StreamConfig | None,
    instance_id: str,
    motion_seq: int,
    hold_positions: tuple[float, ...],
    profile_digest: str,
    now_sim_ns: int,
    step_ns: int,
    start_sim_time_ns: int | None,
) -> ChunkAnchor:
    """Validate the current phase and return the one admissible anchor."""
    if stream_config is None:
        raise invalid("STREAM_NOT_CONFIGURED", "MotionController has no stream config")
    if phase is Phase.FAILED:
        raise invalid("STREAM_ANCHOR_UNAVAILABLE", "a failed motion has no successor")
    if phase is Phase.STOPPING:
        raise invalid("STREAM_ANCHOR_UNAVAILABLE", "stopping motion has no successor")
    if schedule is None and phase in ACTIVE_PHASES:
        raise invalid("MOTION_ACTIVE", "legacy motion is active")
    if schedule is not None and phase in (Phase.ACCEPTED, Phase.RUNNING, Phase.SETTLING):
        return compute_active_chunk_anchor(schedule, phase, start_sim_time_ns, profile_digest)
    return compute_idle_chunk_anchor(
        instance_id=instance_id,
        motion_seq=motion_seq,
        hold_positions=hold_positions,
        profile_digest=profile_digest,
        now_sim_ns=now_sim_ns,
        stream_config=stream_config,
        step_ns=step_ns,
        start_sim_time_ns=start_sim_time_ns,
    )


def snapshot_for_call(
    snapshotter: Snapshotter,
    *,
    context: SnapshotContext,
    last_snapshot: MotionSnapshot | None,
    phase: Phase,
    stop_reason: str | None,
    events: tuple,
    measurement: Measurement,
    hold_positions: tuple[float, ...],
    contacts: tuple,
    hold_completions: tuple,
) -> MotionSnapshot:
    """Return the caller's current view without inventing a measured step."""
    if last_snapshot is not None:
        changes: dict[str, object] = {
            "phase": phase,
            "reason": (
                stop_reason
                if phase is Phase.STOPPING
                else (last_snapshot.reason if phase in TERMINAL_PHASES else "")
            )
            or "",
            "chunk_events": events,
        }
        return replace(last_snapshot, **changes)  # type: ignore[arg-type]
    return snapshotter.build(
        context,
        phase=phase,
        reason="",
        reference=zero_sample(hold_positions),
        measurement=measurement,
        reference_application_sim_time_ns=measurement.sim_time_ns,
        elapsed_ns=0,
        stable_for_ns=0,
        contacts=contacts,
        completions=hold_completions,
        reference_kind="HOLD",
    )


def stop_terminal(
    snapshotter: Snapshotter,
    *,
    context: SnapshotContext,
    applied: AppliedStep,
    phase: Phase,
    reason: str,
    elapsed_ns: int,
    stable_for_ns: int,
    completions: tuple,
    wall: int,
    result_stop_reason: str | None,
) -> tuple[MotionSnapshot, MotionResult]:
    """Assemble one STOPPED/FAILED snapshot and its frozen result."""
    snapshot = snapshotter.build_applied(
        context,
        applied,
        phase=phase,
        reason=reason,
        chunk_seq=None if context.active_plan is None else context.active_plan.seq,
        elapsed_ns=elapsed_ns,
        stable_for_ns=stable_for_ns,
        completions=completions,
    )
    return snapshot, Snapshotter.freeze(snapshot, wall, result_stop_reason)


def failed_terminal(
    snapshotter: Snapshotter,
    *,
    context: SnapshotContext,
    applied: AppliedStep | None,
    reason: str,
    stable_for_ns: int,
    wall: int,
    stop_reason: str | None,
    measurement: Measurement,
    hold_positions: tuple[float, ...],
    contacts: tuple,
    hold_completions: tuple,
) -> tuple[MotionSnapshot, MotionResult]:
    """Assemble a FAILED snapshot with or without a prior physical application."""
    if applied is not None:
        snapshot = snapshotter.build_applied(
            context,
            applied,
            phase=Phase.FAILED,
            reason=reason,
            chunk_seq=None if context.active_plan is None else context.active_plan.seq,
            stable_for_ns=stable_for_ns,
        )
    else:
        snapshot = snapshotter.build(
            context,
            phase=Phase.FAILED,
            reason=reason,
            reference=zero_sample(hold_positions),
            measurement=measurement,
            reference_application_sim_time_ns=measurement.sim_time_ns,
            elapsed_ns=0,
            stable_for_ns=stable_for_ns,
            contacts=contacts,
            completions=hold_completions,
            reference_kind="HOLD",
        )
    return snapshot, Snapshotter.freeze(snapshot, wall, stop_reason)


@dataclass(frozen=True, slots=True)
class BoundaryEffect:
    """Immutable stream-boundary effect applied by the controller."""

    phase: Phase
    hold_positions: tuple[float, ...] | None
    final_positions: tuple[float, ...] | None
    final_targets: tuple[float, ...] | None
    final_end_ns: int | None
    retired: object | None
    started: object | None
    underrun: bool
    events: tuple[tuple[str, object, int, str], ...]


def stream_boundary(
    schedule: StreamSchedule, active: object, boundary: object, application_sim_ns: int
) -> BoundaryEffect:
    """Return the effect of one chosen chunk-boundary reference."""
    if boundary.action == "settle":
        return BoundaryEffect(
            Phase.SETTLING,
            boundary.reference.positions,
            boundary.reference.positions,
            tuple(active.window.targets[-1].positions),
            application_sim_ns,
            active,
            None,
            False,
            (("RETIRED", active, application_sim_ns, "FINAL_HOLD"),),
        )
    if boundary.action == "handoff":
        retired, started = schedule.handoff()
        final_positions = final_targets = final_end = None
        if started.settle_at_end:
            final_positions = sample_reference_interval(
                started.plan, started.plan.duration_ns
            ).positions
            final_targets = tuple(started.window.targets[-1].positions)
            final_end = started.end_sim_time_ns
        return BoundaryEffect(
            Phase.RUNNING,
            None,
            final_positions,
            final_targets,
            final_end,
            retired,
            started,
            False,
            (
                ("RETIRED", retired, application_sim_ns, "HANDOFF"),
                ("STARTED", started, application_sim_ns, "HANDOFF"),
            ),
        )
    return BoundaryEffect(
        Phase.STOPPING,
        None,
        None,
        None,
        None,
        active,
        None,
        True,
        (("RETIRED", active, application_sim_ns, "UNDERRUN"),),
    )


def begin_stop(
    braking: object,
    *,
    schedule: StreamSchedule,
    applied: AppliedStep | None,
    hold_positions: tuple[float, ...],
    profile_joints: tuple[str, ...],
    sim_time_ns: int,
    reason: str,
    latched: str | None,
) -> tuple[object | None, tuple[float, ...], str, str | None]:
    """Plan one brake and return ``(canceled_chunk, base_positions, reason, latched)``."""
    base = zero_sample(hold_positions) if applied is None else applied.reference
    canceled = schedule.cancel_pending()
    joints = schedule.active.joints if schedule.active is not None else profile_joints
    braking.begin(
        base_positions=base.positions,
        base_velocities=base.velocities,
        joints=joints,
        sim_time_ns=sim_time_ns,
        application_sim_time_ns=(
            None if applied is None else applied.application_sim_time_ns
        ),
    )
    return canceled, base.positions, str(reason), str(latched) if latched is not None else None
