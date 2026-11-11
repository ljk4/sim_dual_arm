from __future__ import annotations

import math
from collections.abc import Sequence
from typing import Annotated, Self

from pydantic import BeforeValidator, Field, model_validator
from robot_contracts.action_chunks import ActionWindow, ChunkAnchor
from robot_contracts.models import ContractError, FloatVector, JointSpec
from robot_contracts.trajectory.follower import FollowerLimitFlags, NormalizedFollowerPositions
from robot_contracts.trajectory.spline import ReferencePlan, ReferenceSample
from robot_contracts.validation import StrictContractModel, as_tuple, contract_error

from robot_trajectory.sampling import sample_reference_interval

from .follower_math import (
    _DEVIATION_ATOL,
    _vector,
    compute_follower_deviations,
    estimate_raw_terminal_velocities,
    restrict_terminal_velocities,
)
from .spline_math import (
    MAX_HORIZON_NS,
    MAX_JOINTS,
    POSITION_ATOL,
    RELATIVE_TOLERANCE,
    VELOCITY_ATOL,
    ReferenceExtremes,
    _limit_tolerance,
    _state_at,
    check_continuous_limits,
    compute_reference_extremes,
    validate_reference_inputs,
    validate_velocity_boundaries,
)


def _invalid(location: str, message: str) -> ContractError:
    return ContractError("INVALID_FOLLOWER_PLAN", f"{location}: {message}")


def _close(left: float, right: float, atol: float) -> bool:
    return math.isclose(left, right, rel_tol=RELATIVE_TOLERANCE, abs_tol=atol)


def _require(condition: bool, field: str, message: str) -> None:
    if not condition:
        raise contract_error(field, message)






def normalize_follower_positions(
    positions: Sequence[float], joints: Sequence[JointSpec],
    location: str = "reference.positions",
) -> NormalizedFollowerPositions:
    joint_items = tuple(joints)
    if not 1 <= len(joint_items) <= MAX_JOINTS:
        raise _invalid("joints", f"must contain 1..{MAX_JOINTS} joints")
    sampled = _vector(positions, location, len(joint_items))
    applied: list[float] = []
    corrections: list[float] = []
    for index, (position, joint) in enumerate(zip(sampled, joint_items, strict=True)):
        minimum, maximum = float(joint.minimum), float(joint.maximum)
        if minimum <= position <= maximum:
            corrected = position
        elif position < minimum:
            if position < minimum - _limit_tolerance(minimum):
                raise ContractError(
                    "FOLLOWER_BOUNDARY_MISMATCH",
                    f"{location}[{index}] ({joint.name}) {position!r} is below the "
                    f"hard minimum {minimum!r} by more than the shared round-off tolerance")
            corrected = minimum
        elif position > maximum + _limit_tolerance(maximum):
            raise ContractError(
                "FOLLOWER_BOUNDARY_MISMATCH",
                f"{location}[{index}] ({joint.name}) {position!r} is above the "
                f"hard maximum {maximum!r} by more than the shared round-off tolerance")
        else:
            corrected = maximum
        applied.append(corrected)
        corrections.append(position - corrected)
    return NormalizedFollowerPositions(
        sampled_positions=sampled, applied_positions=tuple(applied),
        corrections=tuple(corrections))


class PreparedFollowerPlan(StrictContractModel):
    anchor: ChunkAnchor
    window: ActionWindow
    plan: ReferencePlan
    joints: Annotated[tuple[JointSpec, ...], BeforeValidator(as_tuple),
                      Field(min_length=1, max_length=MAX_JOINTS)]
    max_accelerations: FloatVector
    max_target_deviation_limits: FloatVector
    raw_terminal_velocities: FloatVector
    restricted_terminal_velocities: FloatVector
    is_final: bool
    requested_window_ns: Annotated[int, Field(strict=True, ge=1, le=MAX_HORIZON_NS)]
    ruckig_duration_ns: Annotated[int, Field(strict=True, ge=1, le=MAX_HORIZON_NS)]
    duration_grid_ns: Annotated[int, Field(strict=True, ge=1, le=MAX_HORIZON_NS)] | None = None
    actual_terminal_positions: tuple[float, ...] = Field(default=(), exclude=True)
    actual_terminal_velocities: tuple[float, ...] = Field(default=(), exclude=True)
    actual_terminal_accelerations: tuple[float, ...] = Field(default=(), exclude=True)
    max_target_deviations: tuple[float, ...] = Field(default=(), exclude=True)
    extremes: ReferenceExtremes | None = Field(default=None, exclude=True)
    flags: FollowerLimitFlags | None = Field(default=None, exclude=True)
    extension_ns: int = Field(default=0, exclude=True)

    def _validate_lengths(self) -> Self:
        count = len(self.joints)
        for name in (
            "max_accelerations",
            "max_target_deviation_limits",
            "raw_terminal_velocities",
            "restricted_terminal_velocities",
        ):
            values = getattr(self, name)
            if len(values) != count:
                raise contract_error(name, "must contain one value per joint")
        for name in ("max_accelerations", "max_target_deviation_limits"):
            if any(value <= 0.0 for value in getattr(self, name)):
                raise contract_error(name, "must be positive")
        return self

    def _validate_terminals(self):
        restricted = validate_velocity_boundaries(
            self.restricted_terminal_velocities,
            self.joints,
            "restricted_terminal_velocities",
        )
        inputs = validate_reference_inputs(
            self.anchor.reference.positions,
            self.window.source_targets,
            self.joints,
            self.max_accelerations,
            initial_velocities=self.anchor.reference.velocities,
            final_velocities=restricted,
        )
        derived_raw = estimate_raw_terminal_velocities(self.window.targets)
        for axis, (stored, derived) in enumerate(
            zip(self.raw_terminal_velocities, derived_raw, strict=True)
        ):
            if not math.isclose(stored, derived, rel_tol=0.0, abs_tol=_DEVIATION_ATOL):
                raise contract_error(
                    f"raw_terminal_velocities[{axis}]",
                    "must equal the backward difference of raw targets",
                )
        expected = restrict_terminal_velocities(
            derived_raw,
            inputs.initial_positions,
            inputs.initial_velocities,
            self.window.targets[-1].positions,
            inputs.joints,
            inputs.max_accelerations,
            is_final=self.is_final,
        )
        for axis, (stored, wanted) in enumerate(zip(restricted, expected, strict=True)):
            if not math.isclose(stored, wanted, rel_tol=0.0, abs_tol=_DEVIATION_ATOL):
                raise contract_error(
                    f"restricted_terminal_velocities[{axis}]",
                    "differs from the generic speed/boundary rule",
                )
        return inputs, restricted

    def _validate_timing(self, inputs) -> None:
        _require(
            self.plan.joint_names == inputs.joint_names,
            "plan.joint_names",
            "must match the follower joint order",
        )
        _require(
            self.window.handoff_time_ns == self.anchor.start_sim_time_ns,
            "anchor.start_sim_time_ns",
            "must equal window.handoff_time_ns",
        )
        _require(
            self.requested_window_ns == self.window.end_time_ns,
            "requested_window_ns",
            "must equal window.end_time_ns",
        )
        _require(
            self.ruckig_duration_ns >= self.plan.duration_ns,
            "ruckig_duration_ns",
            "must cover the exported plan duration",
        )
        if self.duration_grid_ns is not None:
            grid = self.duration_grid_ns
            _require(
                not self.is_final or self.ruckig_duration_ns % grid == 0,
                "ruckig_duration_ns",
                "a discrete final duration must lie on the grid",
            )
            _require(
                self.is_final or self.requested_window_ns % grid == 0,
                "requested_window_ns",
                "a non-final hand-off must lie on the grid",
            )
            _require(
                self.plan.duration_ns % grid == 0,
                "plan.duration_ns",
                "the exported plan duration must lie on the grid",
            )
        if self.is_final:
            if self.plan.duration_ns < self.requested_window_ns:
                raise contract_error("plan.duration_ns", "a final plan must cover the window")
        elif self.plan.duration_ns != self.requested_window_ns:
            raise contract_error("plan.duration_ns", "a non-final plan must end at the hand-off")

    def _validate_segments(self, count: int) -> None:
        for index in range(1, len(self.plan.segments)):
            previous, current = self.plan.segments[index - 1], self.plan.segments[index]
            previous_positions, previous_velocities, _ = _state_at(
                previous, previous.duration_s, f"segments[{index - 1}]"
            )
            current_positions, current_velocities, _ = _state_at(
                current, 0.0, f"segments[{index}]"
            )
            for axis in range(count):
                if not _close(
                    previous_positions[axis], current_positions[axis], POSITION_ATOL
                ):
                    raise contract_error(
                        f"segments[{index}].start_positions[{axis}]",
                        "position is not C1 continuous",
                    )
                if not _close(
                    previous_velocities[axis], current_velocities[axis], VELOCITY_ATOL
                ):
                    raise contract_error(
                        f"segments[{index}].start_velocities[{axis}]",
                        "velocity is not C1 continuous",
                    )

    def _validate_end(self, inputs) -> object:
        end = sample_reference_interval(self.plan, self.plan.duration_ns)
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
                        f"joint {joint.name!r} cannot brake inside maximum",
                    )
            else:
                lower = float(joint.minimum)
                if end.positions[axis] - stop_distance < lower - _limit_tolerance(lower):
                    raise ContractError(
                        "FOLLOWER_BRAKING_INFEASIBLE",
                        f"joint {joint.name!r} cannot brake inside minimum",
                    )
        if self.is_final:
            if any(abs(velocity) > VELOCITY_ATOL for velocity in end.velocities):
                raise contract_error("plan", "a final plan must end at zero velocity")
            for axis, (position, target) in enumerate(
                zip(end.positions, self.window.targets[-1].positions, strict=True)
            ):
                if not _close(position, target, POSITION_ATOL):
                    raise contract_error(
                        f"plan.actual_terminal_positions[{axis}]",
                        "a final plan must reach the last target",
                    )
        return end

    def _validate_deviations(self, inputs) -> tuple[float, ...]:
        deviations = compute_follower_deviations(
            self.plan, inputs.initial_positions, self.window.targets, self.is_final
        )
        for axis, (deviation, limit) in enumerate(
            zip(deviations, self.max_target_deviation_limits, strict=True)
        ):
            if deviation > limit + _DEVIATION_ATOL:
                raise ContractError(
                    "FOLLOWER_DEVIATION_EXCEEDED",
                    f"axis {inputs.joint_names[axis]!r} deviation "
                    f"{deviation!r} exceeds {limit!r}",
                )
        return deviations

    def _assign_evidence(self, inputs, restricted, end, deviations, extremes) -> None:
        last_target = self.window.targets[-1].positions
        velocity_flags: list[bool] = []
        acceleration_flags: list[bool] = []
        boundary_flags: list[bool] = []
        for axis, (joint, item) in enumerate(
            zip(inputs.joints, extremes.joints, strict=True)
        ):
            moving = (
                item.maximum_velocity > VELOCITY_ATOL
                or item.maximum_position - item.minimum_position > POSITION_ATOL
            )
            terminal = not _close(
                self.raw_terminal_velocities[axis], restricted[axis], _DEVIATION_ATOL
            )
            velocity_flags.append(
                terminal
                or (moving and item.maximum_velocity >= float(joint.max_velocity) - 1e-9)
            )
            acceleration_flags.append(
                moving and item.maximum_acceleration >= inputs.max_accelerations[axis] - 1e-9
            )
            boundary_flags.append(
                _close(end.positions[axis], last_target[axis], POSITION_ATOL)
            )
        flags = FollowerLimitFlags(
            tuple(velocity_flags), tuple(acceleration_flags), tuple(boundary_flags)
        )
        object.__setattr__(self, "actual_terminal_positions", end.positions)
        object.__setattr__(self, "actual_terminal_velocities", end.velocities)
        object.__setattr__(self, "actual_terminal_accelerations", end.accelerations)
        object.__setattr__(self, "max_target_deviations", deviations)
        object.__setattr__(self, "extremes", extremes)
        object.__setattr__(self, "flags", flags)
        object.__setattr__(
            self,
            "extension_ns",
            max(0, self.plan.duration_ns - self.requested_window_ns),
        )

    @model_validator(mode="after")
    def _plan(self) -> Self:
        count = len(self.joints)
        self._validate_lengths()
        inputs, restricted = self._validate_terminals()
        self._validate_timing(inputs)
        if self.is_final and any(abs(velocity) > VELOCITY_ATOL for velocity in restricted):
            raise contract_error(
                "restricted_terminal_velocities", "must be zero for a final plan"
            )
        self._validate_segments(count)
        extremes = compute_reference_extremes(self.plan)
        check_continuous_limits(extremes, inputs.joints, inputs.max_accelerations)
        end = self._validate_end(inputs)
        deviations = self._validate_deviations(inputs)
        self._assign_evidence(inputs, restricted, end, deviations, extremes)
        return self

    seq = property(lambda self: self.anchor.seq)
    settle_at_end = property(lambda self: self.is_final)
    joint_names = property(lambda self: self.plan.joint_names)
    joint_count = property(lambda self: self.plan.joint_count)
    duration_ns = property(lambda self: self.plan.duration_ns)
    start_sim_time_ns = property(lambda self: self.anchor.start_sim_time_ns)
    end_sim_time_ns = property(
        lambda self: self.anchor.start_sim_time_ns + self.plan.duration_ns)

    def sample_at(self, sim_time_ns: int) -> ReferenceSample:
        if isinstance(sim_time_ns, bool) or not isinstance(sim_time_ns, int):
            raise _invalid("sim_time_ns", "must be an integer")
        if sim_time_ns < self.start_sim_time_ns or sim_time_ns > self.end_sim_time_ns:
            raise ContractError("REFERENCE_EXPIRED",
                                f"sim_time_ns {sim_time_ns} is outside "
                                f"[{self.start_sim_time_ns}, {self.end_sim_time_ns}]")
        return sample_reference_interval(self.plan, sim_time_ns - self.start_sim_time_ns)

    def to_document(self) -> dict[str, object]:
        document = self.model_dump(mode="json")
        document["window"]["selected_indices"] = list(self.window.selected_indices)
        document["window"]["desired_targets"] = [
            target.model_dump(mode="json") for target in self.window.targets]
        document.update({name: list(getattr(self, name)) for name in (
            "actual_terminal_positions", "actual_terminal_velocities",
            "actual_terminal_accelerations", "max_target_deviations")})
        document.update({
            "kind": "bounded_target_follower", "extension_ns": self.extension_ns,
            "flags": self.flags.model_dump(mode="json") | {
                "any_velocity_limited": self.flags.any_velocity_limited,
                "any_acceleration_limited": self.flags.any_acceleration_limited,
                "all_boundaries_preserved": self.flags.all_boundaries_preserved},
            "extremes": [item.model_dump(mode="json") for item in self.extremes.joints]})
        return document
