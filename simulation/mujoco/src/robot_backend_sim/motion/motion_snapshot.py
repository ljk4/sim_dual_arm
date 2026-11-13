"""Explicit snapshot/result assembly and tracking classification.

``Snapshotter`` and ``TrackingState`` are components owned by the motion
controller.  They receive already measured values and never step physics,
write targets or read a clock.  ``SnapshotContext`` is the immutable view of
the controller identity needed to label one snapshot.
"""

from __future__ import annotations

from dataclasses import dataclass

from robot_contracts.trajectory.spline import ReferenceSample

from robot_backend_sim.motion.motion_types import AppliedStep, MotionResult, MotionSnapshot, Phase
from robot_backend_sim.motion.stream_schedule import ChunkEvent
from robot_backend_sim.physics.contacts import GripperCompletion, GripperContactEvidence
from robot_backend_sim.physics.gripper_contact import GripperContactTracker
from robot_backend_sim.physics.robot_io import Measurement

__all__ = ["SnapshotContext", "Snapshotter", "TrackingState", "tracking_errors"]


def tracking_errors(
    reference: ReferenceSample, measurement: Measurement
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """Return position and velocity command-minus-measurement errors."""
    positions = tuple(
        commanded - measured
        for commanded, measured in zip(reference.positions, measurement.positions, strict=True)
    )
    velocities = tuple(
        commanded - measured
        for commanded, measured in zip(reference.velocities, measurement.velocities, strict=True)
    )
    return positions, velocities


@dataclass(frozen=True, slots=True)
class SnapshotContext:
    """Identity and current stream facts needed to label one snapshot."""

    phase: Phase
    motion_seq: int
    accepted_wall_ns: int | None
    started_wall_ns: int | None
    started_sim_ns: int | None
    joint_names: tuple[str, ...]
    stream_id: str | None
    hold_positions: tuple[float, ...]
    stream_initial_positions: tuple[float, ...] | None
    stream_final_targets: tuple[float, ...] | None
    active_plan: object | None
    tracking_quality: str
    tracking_violation_ns: int
    events: tuple[ChunkEvent, ...]


class Snapshotter:
    """Assemble immutable snapshots and terminal results from explicit facts."""

    def __init__(self, gripper: GripperContactTracker) -> None:
        self._gripper = gripper

    def completion_bounds(
        self, context: SnapshotContext
    ) -> tuple[tuple[float, ...], tuple[float, ...]]:
        return self._gripper.completion_bounds(
            context.stream_initial_positions,
            context.stream_final_targets,
            context.active_plan,
            context.hold_positions,
        )

    def completions(
        self,
        context: SnapshotContext,
        measurement: Measurement,
        contacts: tuple[GripperContactEvidence, ...],
    ) -> tuple[GripperCompletion, ...]:
        return self._gripper.completions(measurement, contacts, *self.completion_bounds(context))

    def build(
        self,
        context: SnapshotContext,
        *,
        phase: Phase | None = None,
        reason: str,
        reference: ReferenceSample,
        measurement: Measurement,
        reference_application_sim_time_ns: int,
        elapsed_ns: int,
        stable_for_ns: int,
        contacts: tuple[GripperContactEvidence, ...],
        completions: tuple[GripperCompletion, ...] | None = None,
        reference_kind: str = "TRAJECTORY",
        chunk_seq: int | None = None,
        sampled_reference_positions: tuple[float, ...] | None = None,
        sampled_reference_velocities: tuple[float, ...] | None = None,
        reference_boundary_corrections: tuple[float, ...] = (),
    ) -> MotionSnapshot:
        if completions is None:
            completions = (
                self.completions(context, measurement, contacts)
                if context.stream_id is not None
                else self._gripper.hold_completions(
                    measurement, contacts, context.hold_positions
                )
            )
        errors, velocity_errors = tracking_errors(reference, measurement)
        return MotionSnapshot(
            phase=context.phase if phase is None else phase,
            motion_seq=context.motion_seq,
            reason=reason,
            accepted_wall_ns=context.accepted_wall_ns,
            started_wall_ns=context.started_wall_ns,
            started_sim_ns=context.started_sim_ns,
            reference_application_sim_time_ns=reference_application_sim_time_ns,
            reference_elapsed_ns=elapsed_ns,
            measurement_sim_time_ns=measurement.sim_time_ns,
            stable_for_ns=stable_for_ns,
            joint_names=context.joint_names,
            reference_positions=reference.positions,
            reference_velocities=reference.velocities,
            reference_accelerations=reference.accelerations,
            measured_positions=measurement.positions,
            measured_velocities=measurement.velocities,
            tracking_errors=errors,
            grippers=completions,
            contacts=contacts,
            stream_id=context.stream_id,
            chunk_seq=chunk_seq,
            reference_kind=reference_kind,
            tracking_velocity_errors=velocity_errors,
            tracking_quality=context.tracking_quality,
            tracking_violation_ns=context.tracking_violation_ns,
            chunk_events=context.events,
            sampled_reference_positions=(
                reference.positions
                if sampled_reference_positions is None
                else sampled_reference_positions
            ),
            sampled_reference_velocities=(
                reference.velocities
                if sampled_reference_velocities is None
                else sampled_reference_velocities
            ),
            reference_boundary_corrections=reference_boundary_corrections,
        )

    def build_applied(
        self,
        context: SnapshotContext,
        applied: AppliedStep,
        *,
        phase: Phase | None = None,
        reason: str,
        reference_kind: str | None = None,
        chunk_seq: int | None = None,
        stable_for_ns: int | None = None,
        completions: tuple[GripperCompletion, ...] | None = None,
        elapsed_ns: int | None = None,
    ) -> MotionSnapshot:
        return self.build(
            context,
            phase=phase,
            reason=reason,
            reference=applied.reference,
            measurement=applied.measurement,
            reference_application_sim_time_ns=applied.application_sim_time_ns,
            elapsed_ns=applied.elapsed_ns if elapsed_ns is None else elapsed_ns,
            stable_for_ns=stable_for_ns or 0,
            contacts=applied.contacts,
            completions=(
                self.completions(context, applied.measurement, applied.contacts)
                if completions is None
                else completions
            ),
            reference_kind=(
                applied.reference_kind if reference_kind is None else reference_kind
            ),
            chunk_seq=chunk_seq,
            sampled_reference_positions=applied.sampled_reference.positions,
            sampled_reference_velocities=applied.sampled_reference.velocities,
            reference_boundary_corrections=applied.reference_corrections,
        )

    @staticmethod
    def freeze(
        snapshot: MotionSnapshot, wall: int, stop_reason: str | None
    ) -> MotionResult:
        return MotionResult(
            motion_seq=snapshot.motion_seq,
            phase=snapshot.phase,
            reason=snapshot.reason,
            stop_reason=stop_reason,
            accepted_wall_ns=snapshot.accepted_wall_ns,
            started_wall_ns=snapshot.started_wall_ns,
            started_sim_ns=snapshot.started_sim_ns,
            finished_wall_ns=wall,
            snapshot=snapshot,
        )


class TrackingState:
    """Measured tracking classification; never invents a violation window."""

    def __init__(self) -> None:
        self.quality = "HOLD"
        self.violation_since_ns: int | None = None
        self.violation_observed_sim_ns = 0

    @property
    def violation_ns(self) -> int:
        if self.violation_since_ns is None:
            return 0
        return max(0, self.violation_observed_sim_ns - self.violation_since_ns)

    def reset(self) -> None:
        self.quality = "HOLD"
        self.violation_since_ns = None
        self.violation_observed_sim_ns = 0

    def update(
        self,
        reference: ReferenceSample,
        measurement: Measurement,
        constrained: frozenset[int],
        unloading: frozenset[int],
        position_tolerances: tuple[float, ...],
        velocity_tolerances: tuple[float, ...],
    ) -> None:
        violating = any(
            index not in constrained
            and (
                abs(reference.positions[index] - measurement.positions[index])
                > position_tolerances[index]
                or abs(reference.velocities[index] - measurement.velocities[index])
                > velocity_tolerances[index]
            )
            for index in range(len(reference.positions))
        )
        if violating:
            if self.violation_since_ns is None:
                self.violation_since_ns = measurement.sim_time_ns
            self.violation_observed_sim_ns = measurement.sim_time_ns
            self.quality = "OUTSIDE_TOLERANCE"
        else:
            self.violation_since_ns = None
            self.quality = (
                "CONTACT_UNLOADING"
                if unloading
                else "CONSTRAINED"
                if constrained
                else "FOLLOWING"
            )
