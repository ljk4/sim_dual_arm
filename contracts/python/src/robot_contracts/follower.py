"""Immutable bounded-target follower plan and exact target-deviation math.

The strict :class:`robot_contracts.reference.ReferencePlan` remains the only
coefficient representation.  A follower plan is a different explicit type: it
does not claim to pass through every original prediction.  It keeps the complete
immutable :class:`ActionWindow` and :class:`ChunkAnchor`, stores the actual
coefficient curve, the raw and limited terminal velocities, the exact continuous
maximum target deviation per axis, and the flags that explain what the planner
had to limit.

This module uses only the Python standard library and never imports Ruckig,
NumPy, SciPy or ROS.  The desired target is piecewise linear from the anchor at
``t = 0`` through the selected raw predictions at their hand-off-relative times.
A final plan may extend the last desired value as a tail.  Deviations are the
maximum of the absolute cubic difference over every continuous intersection of
a reference segment and a desired segment; control ticks are not sampled here.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from numbers import Integral

from .action_chunks import ActionWindow, ChunkAnchor
from .follower_math import (
    _DEVIATION_ATOL,
    _integer,
    _invalid,
    _limit_tolerance,
    _segment_state,
    _vector,
    compute_follower_deviations,
    estimate_raw_terminal_velocities,
    restrict_terminal_velocities,
)
from .models import ContractError, JointSpec
from .reference import (
    MAX_HORIZON_NS,
    MAX_JOINTS,
    POSITION_ATOL,
    RELATIVE_TOLERANCE,
    VELOCITY_ATOL,
    ReferenceExtremes,
    ReferencePlan,
    ReferenceSample,
    check_continuous_limits,
    compute_reference_extremes,
    validate_reference_inputs,
    validate_velocity_boundaries,
)

FOLLOWER_ALGORITHM_VERSION = "bounded-target-follower/ruckig-0.15.3/v1"


@dataclass(frozen=True)
class FollowerLimitFlags:
    """Per-axis explanation of what the bounded follower had to limit."""

    velocity_limited: tuple[bool, ...]
    acceleration_limited: tuple[bool, ...]
    boundary_preserved: tuple[bool, ...]

    def __post_init__(self) -> None:
        lengths: set[int] = set()
        for name in ("velocity_limited", "acceleration_limited", "boundary_preserved"):
            value = getattr(self, name)
            try:
                items = tuple(value)
            except TypeError as error:
                raise _invalid(name, "must be an array of bools") from error
            if not 1 <= len(items) <= MAX_JOINTS:
                raise _invalid(name, f"must contain 1..{MAX_JOINTS} values")
            for index, item in enumerate(items):
                if not isinstance(item, bool):
                    raise _invalid(f"{name}[{index}]", "must be a bool")
            object.__setattr__(self, name, items)
            lengths.add(len(items))
        if len(lengths) != 1:
            raise _invalid("flags", "all flag arrays must have the same length")

    @property
    def any_velocity_limited(self) -> bool:
        return any(self.velocity_limited)

    @property
    def any_acceleration_limited(self) -> bool:
        return any(self.acceleration_limited)

    @property
    def all_boundaries_preserved(self) -> bool:
        return all(self.boundary_preserved)


@dataclass(frozen=True)
class PreparedFollowerPlan:
    """One immutable bounded-follower chunk; never a :class:`PreparedChunk`.

    ``window`` and ``anchor`` are retained unchanged, including every original
    source prediction.  ``plan`` is the explicit cubic curve actually followed.
    ``raw_terminal_velocities`` are the backward differences of the selected raw
    predictions; ``restricted_terminal_velocities`` are what the planner asked
    Ruckig to target after speed and target-boundary braking feasibility limits.
    ``actual_terminal_*`` are the real exported curve endpoint, which can differ
    from the raw target for a non-final chunk and is inherited by its successor.
    ``duration_grid_ns`` is the optional Ruckig discrete grid used for the real
    physical duration; ``None`` preserves the accepted continuous planning.
    """

    anchor: ChunkAnchor
    window: ActionWindow
    plan: ReferencePlan
    joints: tuple[JointSpec, ...]
    max_accelerations: tuple[float, ...]
    max_target_deviation_limits: tuple[float, ...]
    raw_terminal_velocities: tuple[float, ...]
    restricted_terminal_velocities: tuple[float, ...]
    is_final: bool
    requested_window_ns: int
    ruckig_duration_ns: int
    duration_grid_ns: int | None = None
    actual_terminal_positions: tuple[float, ...] = field(init=False)
    actual_terminal_velocities: tuple[float, ...] = field(init=False)
    actual_terminal_accelerations: tuple[float, ...] = field(init=False)
    max_target_deviations: tuple[float, ...] = field(init=False)
    extremes: ReferenceExtremes = field(init=False)
    flags: FollowerLimitFlags = field(init=False)
    extension_ns: int = field(init=False)

    def __post_init__(self) -> None:
        if not isinstance(self.anchor, ChunkAnchor):
            raise _invalid("anchor", "must be a ChunkAnchor")
        if not isinstance(self.window, ActionWindow):
            raise _invalid("window", "must be an ActionWindow")
        if not isinstance(self.plan, ReferencePlan):
            raise _invalid("plan", "must be a ReferencePlan")
        if not isinstance(self.is_final, bool):
            raise _invalid("is_final", "must be a bool")

        try:
            joint_items = tuple(self.joints)
            acceleration_items = tuple(self.max_accelerations)
            deviation_limits = tuple(self.max_target_deviation_limits)
            raw_velocities = tuple(self.raw_terminal_velocities)
            restricted_velocities = tuple(self.restricted_terminal_velocities)
        except TypeError as error:
            raise _invalid("fields", "joint, limit and velocity arrays must be iterable") from error
        if not 1 <= len(joint_items) <= MAX_JOINTS:
            raise _invalid("joints", f"must contain 1..{MAX_JOINTS} joints")
        if any(not isinstance(joint, JointSpec) for joint in joint_items):
            raise _invalid("joints", "must contain JointSpec values")
        joint_count = len(joint_items)

        acceleration_items = _vector(acceleration_items, "max_accelerations", joint_count)
        for index, acceleration in enumerate(acceleration_items):
            if acceleration <= 0.0:
                raise _invalid(f"max_accelerations[{index}]", "must be positive")
        deviation_limits = _vector(deviation_limits, "max_target_deviation_limits", joint_count)
        for index, limit in enumerate(deviation_limits):
            if limit <= 0.0:
                raise _invalid(f"max_target_deviation_limits[{index}]", "must be positive")
        raw_velocities = _vector(raw_velocities, "raw_terminal_velocities", joint_count)
        restricted_velocities = validate_velocity_boundaries(
            restricted_velocities,
            joint_items,
            "restricted_terminal_velocities",
        )

        # Reuse the accepted input validator for joint order, the complete source
        # window (including points before hand-off), initial state and restricted
        # terminal velocity limits.  It is input validation, not a strict fit.
        inputs = validate_reference_inputs(
            self.anchor.reference.positions,
            self.window.source_targets,
            joint_items,
            acceleration_items,
            initial_velocities=self.anchor.reference.velocities,
            final_velocities=restricted_velocities,
        )
        derived_raw = estimate_raw_terminal_velocities(self.window.targets)
        if len(derived_raw) != joint_count:
            raise _invalid(
                "raw_terminal_velocities",
                "must have one value per joint derived from window.targets",
            )
        for axis, (stored, derived) in enumerate(zip(raw_velocities, derived_raw, strict=True)):
            if not math.isclose(
                stored,
                derived,
                rel_tol=0.0,
                abs_tol=_DEVIATION_ATOL,
            ):
                raise _invalid(
                    f"raw_terminal_velocities[{axis}]",
                    "must equal the backward difference of the last two raw targets",
                )
        expected_restricted = restrict_terminal_velocities(
            derived_raw,
            inputs.initial_positions,
            inputs.initial_velocities,
            self.window.targets[-1].positions,
            inputs.joints,
            inputs.max_accelerations,
            is_final=self.is_final,
        )
        for axis, (stored, expected) in enumerate(
            zip(restricted_velocities, expected_restricted, strict=True)
        ):
            if not math.isclose(
                stored,
                expected,
                rel_tol=0.0,
                abs_tol=_DEVIATION_ATOL,
            ):
                raise _invalid(
                    f"restricted_terminal_velocities[{axis}]",
                    "differs from the generic speed and joint-boundary rule",
                )

        if self.plan.joint_names != inputs.joint_names:
            raise _invalid("plan.joint_names", "must match the bounded follower joint order")
        if self.window.target_count != len(self.window.selected_indices):
            raise _invalid("window.target_count", "must select that many source targets")
        if self.window.handoff_time_ns != self.anchor.start_sim_time_ns:
            raise _invalid("anchor.start_sim_time_ns", "must equal window.handoff_time_ns")
        if self.anchor.robot_profile_hash == "":
            raise _invalid("anchor.robot_profile_hash", "must not be empty")

        requested_window_ns = _integer(
            self.requested_window_ns,
            "requested_window_ns",
            minimum=1,
            maximum=MAX_HORIZON_NS,
        )
        if requested_window_ns != self.window.end_time_ns:
            raise _invalid(
                "requested_window_ns",
                f"must equal window.end_time_ns {self.window.end_time_ns}",
            )
        ruckig_duration_ns = _integer(
            self.ruckig_duration_ns,
            "ruckig_duration_ns",
            minimum=1,
            maximum=MAX_HORIZON_NS,
        )
        if ruckig_duration_ns < self.plan.duration_ns:
            raise _invalid(
                "ruckig_duration_ns",
                "must cover the exported plan duration",
            )
        duration_grid_ns = self.duration_grid_ns
        if duration_grid_ns is not None:
            duration_grid_ns = _integer(
                duration_grid_ns,
                "duration_grid_ns",
                minimum=1,
                maximum=MAX_HORIZON_NS,
            )
            if self.is_final:
                if ruckig_duration_ns % duration_grid_ns != 0:
                    raise _invalid(
                        "ruckig_duration_ns",
                        "a discrete final duration must lie on duration_grid_ns",
                    )
            elif requested_window_ns % duration_grid_ns != 0:
                raise _invalid(
                    "requested_window_ns",
                    "a non-final fixed hand-off must lie on duration_grid_ns",
                )
            if self.plan.duration_ns % duration_grid_ns != 0:
                raise _invalid(
                    "plan.duration_ns",
                    "the exported plan duration must lie on duration_grid_ns",
                )
        if self.is_final:
            if self.plan.duration_ns < requested_window_ns:
                raise _invalid(
                    "plan.duration_ns",
                    "a final plan must cover the requested window",
                )
            if any(abs(velocity) > VELOCITY_ATOL for velocity in restricted_velocities):
                raise _invalid(
                    "restricted_terminal_velocities",
                    "must be zero for a final plan",
                )
        elif self.plan.duration_ns != requested_window_ns:
            raise _invalid(
                "plan.duration_ns",
                "a non-final plan must end exactly at the fixed hand-off",
            )

        start_positions, start_velocities, _ = _segment_state(
            self.plan.segments[0], 0.0, "segments[0]"
        )
        for index, (position, velocity) in enumerate(
            zip(start_positions, start_velocities, strict=True)
        ):
            if not math.isclose(
                position,
                inputs.initial_positions[index],
                rel_tol=RELATIVE_TOLERANCE,
                abs_tol=POSITION_ATOL,
            ):
                raise _invalid(
                    f"segments[0].start_positions[{index}]",
                    "does not match the anchor position",
                )
            if not math.isclose(
                velocity,
                inputs.initial_velocities[index],
                rel_tol=RELATIVE_TOLERANCE,
                abs_tol=VELOCITY_ATOL,
            ):
                raise _invalid(
                    f"segments[0].start_velocities[{index}]",
                    "does not match the anchor velocity",
                )

        for index in range(1, len(self.plan.segments)):
            previous = self.plan.segments[index - 1]
            current = self.plan.segments[index]
            previous_positions, previous_velocities, _ = _segment_state(
                previous, previous.duration_s, f"segments[{index - 1}]"
            )
            current_positions, current_velocities, _ = _segment_state(
                current, 0.0, f"segments[{index}]"
            )
            for axis in range(joint_count):
                if not math.isclose(
                    previous_positions[axis],
                    current_positions[axis],
                    rel_tol=RELATIVE_TOLERANCE,
                    abs_tol=POSITION_ATOL,
                ):
                    raise _invalid(
                        f"segments[{index}].start_positions[{axis}]",
                        "position is not C1 continuous at the merged boundary",
                    )
                if not math.isclose(
                    previous_velocities[axis],
                    current_velocities[axis],
                    rel_tol=RELATIVE_TOLERANCE,
                    abs_tol=VELOCITY_ATOL,
                ):
                    raise _invalid(
                        f"segments[{index}].start_velocities[{axis}]",
                        "velocity is not C1 continuous at the merged boundary",
                    )

        extremes = compute_reference_extremes(self.plan)
        check_continuous_limits(extremes, inputs.joints, inputs.max_accelerations)

        end = self.plan.sample_interval(self.plan.duration_ns)
        for axis, (joint, acceleration) in enumerate(
            zip(inputs.joints, inputs.max_accelerations, strict=True)
        ):
            velocity = end.velocities[axis]
            if velocity == 0.0:
                continue
            stop_distance = velocity * velocity / (2.0 * acceleration)
            if velocity > 0.0:
                upper = float(joint.maximum)
                if end.positions[axis] + stop_distance > upper + _limit_tolerance(upper):
                    raise ContractError(
                        "FOLLOWER_BRAKING_INFEASIBLE",
                        f"joint {joint.name!r} endpoint cannot brake inside its maximum {upper!r}",
                    )
            else:
                lower = float(joint.minimum)
                if end.positions[axis] - stop_distance < lower - _limit_tolerance(lower):
                    raise ContractError(
                        "FOLLOWER_BRAKING_INFEASIBLE",
                        f"joint {joint.name!r} endpoint cannot brake inside its minimum {lower!r}",
                    )
        if self.is_final:
            if any(abs(velocity) > VELOCITY_ATOL for velocity in end.velocities):
                raise _invalid("plan", "a final plan must end at zero velocity")
            last_target = self.window.targets[-1]
            for axis, (position, target) in enumerate(
                zip(end.positions, last_target.positions, strict=True)
            ):
                if not math.isclose(
                    position,
                    target,
                    rel_tol=RELATIVE_TOLERANCE,
                    abs_tol=POSITION_ATOL,
                ):
                    raise _invalid(
                        f"plan.actual_terminal_positions[{axis}]",
                        "a final plan must reach the original last target",
                    )

        deviations = compute_follower_deviations(
            self.plan,
            inputs.initial_positions,
            self.window.targets,
            self.is_final,
        )
        for axis, (deviation, limit) in enumerate(zip(deviations, deviation_limits, strict=True)):
            if deviation > limit + _DEVIATION_ATOL:
                raise ContractError(
                    "FOLLOWER_DEVIATION_EXCEEDED",
                    f"axis {inputs.joint_names[axis]!r} maximum target deviation "
                    f"{deviation!r} exceeds configured {limit!r}",
                )

        last_target_positions = self.window.targets[-1].positions
        velocity_limited: list[bool] = []
        acceleration_limited: list[bool] = []
        boundary_preserved: list[bool] = []
        for axis, (joint, joint_extremes) in enumerate(
            zip(inputs.joints, extremes.joints, strict=True)
        ):
            moving = (
                joint_extremes.maximum_velocity > VELOCITY_ATOL
                or joint_extremes.maximum_position - joint_extremes.minimum_position > POSITION_ATOL
            )
            terminal_restricted = not math.isclose(
                raw_velocities[axis],
                restricted_velocities[axis],
                rel_tol=0.0,
                abs_tol=_DEVIATION_ATOL,
            )
            velocity_limit_reached = (
                moving and joint_extremes.maximum_velocity >= float(joint.max_velocity) - 1e-9
            )
            velocity_limited.append(terminal_restricted or velocity_limit_reached)
            acceleration_limited.append(
                moving and joint_extremes.maximum_acceleration >= acceleration_items[axis] - 1e-9
            )
            boundary_preserved.append(
                math.isclose(
                    end.positions[axis],
                    last_target_positions[axis],
                    rel_tol=RELATIVE_TOLERANCE,
                    abs_tol=POSITION_ATOL,
                )
            )
        flags = FollowerLimitFlags(
            tuple(velocity_limited),
            tuple(acceleration_limited),
            tuple(boundary_preserved),
        )

        object.__setattr__(self, "joints", inputs.joints)
        object.__setattr__(self, "max_accelerations", inputs.max_accelerations)
        object.__setattr__(self, "max_target_deviation_limits", deviation_limits)
        object.__setattr__(self, "raw_terminal_velocities", raw_velocities)
        object.__setattr__(self, "restricted_terminal_velocities", restricted_velocities)
        object.__setattr__(self, "requested_window_ns", requested_window_ns)
        object.__setattr__(self, "ruckig_duration_ns", ruckig_duration_ns)
        object.__setattr__(self, "duration_grid_ns", duration_grid_ns)
        object.__setattr__(self, "actual_terminal_positions", end.positions)
        object.__setattr__(self, "actual_terminal_velocities", end.velocities)
        object.__setattr__(self, "actual_terminal_accelerations", end.accelerations)
        object.__setattr__(self, "max_target_deviations", deviations)
        object.__setattr__(self, "extremes", extremes)
        object.__setattr__(self, "flags", flags)
        object.__setattr__(
            self,
            "extension_ns",
            max(0, self.plan.duration_ns - requested_window_ns),
        )

    @property
    def seq(self) -> int:
        return self.anchor.seq

    @property
    def settle_at_end(self) -> bool:
        """Explicit stream-interface alias for the follower's real final flag."""
        return self.is_final

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self.plan.joint_names

    @property
    def joint_count(self) -> int:
        return self.plan.joint_count

    @property
    def duration_ns(self) -> int:
        return self.plan.duration_ns

    @property
    def start_sim_time_ns(self) -> int:
        return self.anchor.start_sim_time_ns

    @property
    def end_sim_time_ns(self) -> int:
        return self.anchor.start_sim_time_ns + self.plan.duration_ns

    def sample_at(self, sim_time_ns: int) -> ReferenceSample:
        """Sample strictly inside the absolute follower interval ``start..end``."""
        if isinstance(sim_time_ns, bool) or not isinstance(sim_time_ns, Integral):
            raise _invalid("sim_time_ns", "must be an integer")
        sim_time = int(sim_time_ns)
        if sim_time < self.start_sim_time_ns or sim_time > self.end_sim_time_ns:
            raise ContractError(
                "REFERENCE_EXPIRED",
                f"sim_time_ns {sim_time} is outside "
                f"[{self.start_sim_time_ns}, {self.end_sim_time_ns}]",
            )
        sample = self.plan.sample_interval(sim_time - self.start_sim_time_ns)
        return ReferenceSample(sample.positions, sample.velocities, sample.accelerations)

    def to_document(self) -> dict[str, object]:
        """Return a fresh JSON-serialisable, independently checkable summary."""
        return {
            "kind": "bounded_target_follower",
            "is_final": self.is_final,
            "joint_names": list(self.joint_names),
            "window": {
                "observation_id": self.window.observation_id,
                "observation_time_ns": self.window.observation_time_ns,
                "handoff_time_ns": self.window.handoff_time_ns,
                "target_count": self.window.target_count,
                "selected_indices": list(self.window.selected_indices),
                "source_targets": [
                    {
                        "time_ns": target.time_ns,
                        "positions": list(target.positions),
                    }
                    for target in self.window.source_targets
                ],
                "desired_targets": [
                    {
                        "time_ns": target.time_ns,
                        "positions": list(target.positions),
                    }
                    for target in self.window.targets
                ],
            },
            "anchor": {
                "stream_id": self.anchor.stream_id,
                "predecessor_seq": self.anchor.predecessor_seq,
                "start_sim_time_ns": self.anchor.start_sim_time_ns,
                "robot_profile_hash": self.anchor.robot_profile_hash,
                "positions": list(self.anchor.reference.positions),
                "velocities": list(self.anchor.reference.velocities),
                "accelerations": list(self.anchor.reference.accelerations),
            },
            "plan": {
                "duration_ns": self.plan.duration_ns,
                "segments": [
                    {
                        "duration_ns": segment.duration_ns,
                        "c0": list(segment.c0),
                        "c1": list(segment.c1),
                        "c2": list(segment.c2),
                        "c3": list(segment.c3),
                    }
                    for segment in self.plan.segments
                ],
            },
            "raw_terminal_velocities": list(self.raw_terminal_velocities),
            "restricted_terminal_velocities": list(self.restricted_terminal_velocities),
            "actual_terminal_positions": list(self.actual_terminal_positions),
            "actual_terminal_velocities": list(self.actual_terminal_velocities),
            "actual_terminal_accelerations": list(self.actual_terminal_accelerations),
            "max_target_deviations": list(self.max_target_deviations),
            "max_target_deviation_limits": list(self.max_target_deviation_limits),
            "requested_window_ns": self.requested_window_ns,
            "ruckig_duration_ns": self.ruckig_duration_ns,
            "duration_grid_ns": self.duration_grid_ns,
            "extension_ns": self.extension_ns,
            "flags": {
                "velocity_limited": list(self.flags.velocity_limited),
                "acceleration_limited": list(self.flags.acceleration_limited),
                "boundary_preserved": list(self.flags.boundary_preserved),
                "any_velocity_limited": self.flags.any_velocity_limited,
                "any_acceleration_limited": self.flags.any_acceleration_limited,
                "all_boundaries_preserved": self.flags.all_boundaries_preserved,
            },
            "extremes": [
                {
                    "joint_name": joint_extremes.joint_name,
                    "minimum_position": joint_extremes.minimum_position,
                    "maximum_position": joint_extremes.maximum_position,
                    "maximum_velocity": joint_extremes.maximum_velocity,
                    "maximum_acceleration": joint_extremes.maximum_acceleration,
                }
                for joint_extremes in self.extremes.joints
            ],
        }


__all__ = [
    "FOLLOWER_ALGORITHM_VERSION",
    "FollowerLimitFlags",
    "PreparedFollowerPlan",
    "compute_follower_deviations",
    "estimate_raw_terminal_velocities",
    "restrict_terminal_velocities",
]
