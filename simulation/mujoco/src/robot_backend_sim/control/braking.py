"""Pure constant-acceleration braking plans for bounded motion stops.

Every axis starts from the last reference position/velocity actually applied to
the robot and decelerates at no more than its configured maximum until zero,
then holds.  The per-axis stop boundaries are merged into explicit quadratic
segments (stored as cubic coefficients with ``c3 == 0``) so position and
velocity remain continuous across the merged plan.  Acceleration may step at a
boundary; this module never claims jerk limits or a C2 braking profile.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from numbers import Integral, Real

from robot_contracts.models import ContractError, JointSpec
from robot_contracts.trajectory.spline import (
    CubicSegment,
    JointExtremes,
    ReferenceExtremes,
    ReferencePlan,
    ReferenceSample,
)
from robot_trajectory.spline_math import check_continuous_limits, compute_reference_extremes

from robot_backend_sim.config.motion_config import ControlConfig

__all__ = ["BrakingController", "BrakingPlan", "plan_braking"]

_MAX_NS = 2**63 - 1
_CONTINUITY_ATOL = 1e-9


def _invalid(message: str) -> ContractError:
    return ContractError("INVALID_BRAKING_INPUT", message)


def _finite(value: object, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise _invalid(f"{location}: must be a finite real number")
    try:
        result = float(value)
    except (OverflowError, TypeError, ValueError) as error:
        raise _invalid(f"{location}: cannot be represented as a finite float") from error
    if not math.isfinite(result):
        raise _invalid(f"{location}: must be finite")
    return result


def _vector(value: object, location: str, length: int) -> tuple[float, ...]:
    if isinstance(value, str | bytes):
        raise _invalid(f"{location}: must be an array of numbers")
    try:
        items = tuple(value)  # type: ignore[arg-type]
    except TypeError as error:
        raise _invalid(f"{location}: must be an array of numbers") from error
    if len(items) != length:
        raise _invalid(f"{location}: expected {length} values, got {len(items)}")
    return tuple(_finite(item, f"{location}[{index}]") for index, item in enumerate(items))


def _nonnegative_int(value: object, location: str) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise _invalid(f"{location}: must be a non-negative integer")
    result = int(value)
    if result < 0:
        raise _invalid(f"{location}: must not be negative")
    return result


@dataclass(frozen=True, slots=True)
class BrakingPlan:
    """Immutable merged braking plan with a clear hold path at the end."""

    joint_names: tuple[str, ...]
    segments: tuple[CubicSegment, ...]
    hold_positions: tuple[float, ...]
    stop_times_ns: tuple[int, ...]

    def __post_init__(self) -> None:
        names = tuple(self.joint_names)
        if not names or len(set(names)) != len(names):
            raise _invalid("joint_names: must be a non-empty unique array")
        hold = _vector(self.hold_positions, "hold_positions", len(names))
        segments = tuple(self.segments)
        if any(not isinstance(segment, CubicSegment) for segment in segments):
            raise _invalid("segments: must contain CubicSegment values")
        for index, segment in enumerate(segments):
            if segment.joint_count != len(names):
                raise _invalid(f"segments[{index}]: joint count differs from joint_names")
            if segment.duration_ns <= 0:
                raise _invalid(f"segments[{index}]: duration must be positive")
        stop_times = tuple(self.stop_times_ns)
        if len(stop_times) != len(names):
            raise _invalid("stop_times_ns: length must match joint_names")
        for index, value in enumerate(stop_times):
            if isinstance(value, bool) or not isinstance(value, Integral) or value < 0:
                raise _invalid(f"stop_times_ns[{index}]: must be a non-negative integer")
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "segments", segments)
        object.__setattr__(self, "hold_positions", hold)
        object.__setattr__(self, "stop_times_ns", tuple(int(value) for value in stop_times))

    @property
    def joint_count(self) -> int:
        return len(self.joint_names)

    @property
    def duration_ns(self) -> int:
        return sum(segment.duration_ns for segment in self.segments)

    def sample(self, elapsed_ns: int) -> ReferenceSample:
        """Sample at local elapsed time; hold at the braking endpoint thereafter."""
        elapsed = _nonnegative_int(elapsed_ns, "elapsed_ns")
        if elapsed >= self.duration_ns:
            zero = (0.0,) * self.joint_count
            return ReferenceSample(self.hold_positions, zero, zero)
        remaining = elapsed
        for segment in self.segments:
            if remaining < segment.duration_ns:
                local_s = remaining / 1_000_000_000
                positions = tuple(
                    ((c3 * local_s + c2) * local_s + c1) * local_s + c0
                    for c0, c1, c2, c3 in zip(
                        segment.c0, segment.c1, segment.c2, segment.c3, strict=True
                    )
                )
                velocities = tuple(
                    (3.0 * c3 * local_s + 2.0 * c2) * local_s + c1
                    for c1, c2, c3 in zip(segment.c1, segment.c2, segment.c3, strict=True)
                )
                accelerations = tuple(
                    6.0 * c3 * local_s + 2.0 * c2
                    for c2, c3 in zip(segment.c2, segment.c3, strict=True)
                )
                return ReferenceSample(positions, velocities, accelerations)
            remaining -= segment.duration_ns
        raise AssertionError("validated braking plan did not contain elapsed time")


def _check_joint_metadata(joints: Sequence[JointSpec]) -> tuple[JointSpec, ...]:
    try:
        items = tuple(joints)
    except TypeError as error:
        raise _invalid("joints: must be an array of JointSpec values") from error
    if not items:
        raise _invalid("joints: must not be empty")
    names: set[str] = set()
    for index, joint in enumerate(items):
        if not isinstance(joint, JointSpec):
            raise _invalid(f"joints[{index}]: must be a JointSpec")
        if not isinstance(joint.name, str) or not joint.name or joint.name in names:
            raise _invalid(f"joints[{index}].name: must be a unique non-empty string")
        names.add(joint.name)
        minimum = _finite(joint.minimum, f"joints[{index}].minimum")
        maximum = _finite(joint.maximum, f"joints[{index}].maximum")
        max_velocity = _finite(joint.max_velocity, f"joints[{index}].max_velocity")
        if minimum > maximum:
            raise _invalid(f"joints[{index}]: minimum exceeds maximum")
        if max_velocity <= 0.0:
            raise _invalid(f"joints[{index}].max_velocity: must be positive")
    return items


def _check_initial_limits(
    positions: tuple[float, ...],
    velocities: tuple[float, ...],
    joints: tuple[JointSpec, ...],
    accelerations: tuple[float, ...],
) -> None:
    extremes = ReferenceExtremes(
        tuple(
            JointExtremes(
                joint_name=joint.name,
                minimum_position=position,
                maximum_position=position,
                maximum_velocity=abs(velocity),
                maximum_acceleration=0.0,
            )
            for joint, position, velocity in zip(joints, positions, velocities, strict=True)
        )
    )
    check_continuous_limits(extremes, joints, accelerations)


def _quadratic_end(c0: float, c1: float, c2: float, duration_s: float) -> tuple[float, float]:
    end_position = ((c2 * duration_s) + c1) * duration_s + c0
    end_velocity = 2.0 * c2 * duration_s + c1
    return end_position, end_velocity


def _ceiling_ns(seconds: float, location: str) -> int:
    if not math.isfinite(seconds) or seconds < 0.0:
        raise _invalid(f"{location}: braking duration is not finite")
    scaled = seconds * 1_000_000_000
    if not math.isfinite(scaled) or scaled > _MAX_NS:
        raise _invalid(f"{location}: braking duration exceeds the integer-nanosecond range")
    value = math.ceil(scaled)
    if value < 0 or value > _MAX_NS:
        raise _invalid(f"{location}: braking duration exceeds the integer-nanosecond range")
    return value


def _braking_inputs(
    initial_positions: Sequence[float],
    initial_velocities: Sequence[float],
    joints: Sequence[JointSpec],
    max_accelerations: Sequence[float],
):
    joint_items = _check_joint_metadata(joints)
    count = len(joint_items)
    positions = _vector(initial_positions, "initial_positions", count)
    velocities = _vector(initial_velocities, "initial_velocities", count)
    accelerations = _vector(max_accelerations, "max_accelerations", count)
    for index, acceleration in enumerate(accelerations):
        if acceleration <= 0.0:
            raise _invalid(f"max_accelerations[{index}]: must be positive")
    if tuple(joint.name for joint in joint_items) == ():
        raise _invalid("joints: must not be empty")
    _check_initial_limits(positions, velocities, joint_items, accelerations)
    return joint_items, count, positions, velocities, accelerations


def _stop_plan(positions, velocities, accelerations) -> tuple[list[int], list[float]]:
    stop_times: list[int] = []
    stop_positions: list[float] = []
    for index, (position, velocity, acceleration) in enumerate(
        zip(positions, velocities, accelerations, strict=True)
    ):
        speed = abs(velocity)
        if speed == 0.0:
            stop_times.append(0)
            stop_positions.append(position)
            continue
        duration_ns = _ceiling_ns(speed / acceleration, f"joints[{index}]")
        duration_ns = max(1, duration_ns)
        duration_s = duration_ns / 1_000_000_000
        actual_deceleration = speed / duration_s
        tolerance = 1e-12 * max(1.0, acceleration)
        if actual_deceleration > acceleration + tolerance:
            raise _invalid(
                f"joints[{index}]: rounded stop time would exceed the acceleration limit"
            )
        stop_times.append(duration_ns)
        stop_positions.append(position + velocity * duration_s / 2.0)
    return stop_times, stop_positions


def _segment_coefficients(
    position: float, velocity: float, start_ns: int, stop_time: int
) -> tuple[float, float, float]:
    if velocity == 0.0 or start_ns >= stop_time:
        return position, 0.0, 0.0
    # Reconstruct velocity from the integer remaining time. Using
    # an accumulated residual over a final 1 ns interval amplifies
    # cancellation into an apparent acceleration-limit violation.
    remaining_ns = stop_time - start_ns
    c0 = position
    c1 = velocity * (remaining_ns / stop_time)
    signed_acceleration = -velocity / (stop_time / 1_000_000_000)
    return c0, c1, 0.5 * signed_acceleration


def _build_segments(
    count: int,
    positions,
    velocities,
    stop_times,
    boundaries,
) -> tuple[list[CubicSegment], list[float], list[float]]:
    segments: list[CubicSegment] = []
    current_positions = list(positions)
    current_velocities = list(velocities)
    previous_end: tuple[tuple[float, ...], tuple[float, ...]] | None = None
    start_ns = 0
    for end_ns in boundaries:
        duration_ns = end_ns - start_ns
        if duration_ns <= 0:
            raise _invalid("braking boundaries did not produce a positive segment")
        duration_s = duration_ns / 1_000_000_000
        c0_values: list[float] = []
        c1_values: list[float] = []
        c2_values: list[float] = []
        for index in range(count):
            c0, c1, c2 = _segment_coefficients(
                current_positions[index], velocities[index], start_ns, stop_times[index]
            )
            end_position, end_velocity = _quadratic_end(c0, c1, c2, duration_s)
            if not (math.isfinite(end_position) and math.isfinite(end_velocity)):
                raise _invalid("braking segment endpoint is not finite")
            c0_values.append(c0)
            c1_values.append(c1)
            c2_values.append(c2)
            current_positions[index] = end_position
            current_velocities[index] = end_velocity
        start_state = (tuple(c0_values), tuple(c1_values))
        if previous_end is not None:
            _check_boundary(previous_end, start_state, start_ns)
        previous_end = (tuple(current_positions), tuple(current_velocities))
        segments.append(
            CubicSegment(
                duration_ns=duration_ns,
                c0=tuple(c0_values),
                c1=tuple(c1_values),
                c2=tuple(c2_values),
                c3=(0.0,) * count,
            )
        )
        start_ns = end_ns
    return segments, current_positions, current_velocities


def plan_braking(
    initial_positions: Sequence[float],
    initial_velocities: Sequence[float],
    joints: Sequence[JointSpec],
    max_accelerations: Sequence[float],
) -> BrakingPlan:
    """Build one explicit merged braking plan from the last applied reference.

    Stop time for each moving axis is ``ceil(abs(v) / max_acceleration * 1e9)``
    nanoseconds, so the actual constant deceleration never exceeds the limit.
    Axes already at exactly zero velocity use the hold path.  Invalid input,
    limit overshoot or non-finite arithmetic raises a stable ``ContractError``;
    callers latch ``STOP_PLAN_REJECTED`` rather than claiming a bounded stop.
    """
    joint_items, count, positions, velocities, accelerations = _braking_inputs(
        initial_positions, initial_velocities, joints, max_accelerations
    )
    stop_times, stop_positions = _stop_plan(positions, velocities, accelerations)
    boundaries = sorted({value for value in stop_times if value > 0})
    names = tuple(joint.name for joint in joint_items)
    if not boundaries:
        plan = BrakingPlan(names, (), tuple(stop_positions), tuple(stop_times))
        _validate_hold_plan(plan, joint_items, accelerations)
        return plan
    segments, current_positions, current_velocities = _build_segments(
        count, positions, velocities, stop_times, boundaries
    )
    for index, velocity in enumerate(current_velocities):
        if abs(velocity) > _CONTINUITY_ATOL:
            raise _invalid(f"joints[{index}]: braking endpoint velocity is not zero")
    plan = BrakingPlan(names, tuple(segments), tuple(current_positions), tuple(stop_times))
    _validate_merged_plan(plan, joint_items, accelerations)
    return plan


def _check_boundary(
    previous_end: tuple[tuple[float, ...], tuple[float, ...]],
    start_state: tuple[tuple[float, ...], tuple[float, ...]],
    boundary_ns: int,
) -> None:
    for index, (left, right) in enumerate(zip(previous_end[0], start_state[0], strict=True)):
        if not math.isclose(left, right, rel_tol=0.0, abs_tol=_CONTINUITY_ATOL):
            raise _invalid(f"position discontinuity at {boundary_ns}ns, joint {index}")
    for index, (left, right) in enumerate(zip(previous_end[1], start_state[1], strict=True)):
        if not math.isclose(left, right, rel_tol=0.0, abs_tol=_CONTINUITY_ATOL):
            raise _invalid(f"velocity discontinuity at {boundary_ns}ns, joint {index}")


def _validate_hold_plan(
    plan: BrakingPlan, joints: tuple[JointSpec, ...], accelerations: tuple[float, ...]
) -> None:
    extremes = ReferenceExtremes(
        tuple(
            JointExtremes(
                joint_name=joint.name,
                minimum_position=position,
                maximum_position=position,
                maximum_velocity=0.0,
                maximum_acceleration=0.0,
            )
            for joint, position in zip(joints, plan.hold_positions, strict=True)
        )
    )
    check_continuous_limits(extremes, joints, accelerations)


def _validate_merged_plan(
    plan: BrakingPlan, joints: tuple[JointSpec, ...], accelerations: tuple[float, ...]
) -> None:
    reference = ReferencePlan(plan.joint_names, plan.segments)
    extremes = compute_reference_extremes(reference)
    check_continuous_limits(extremes, joints, accelerations)
    last = plan.segments[-1]
    duration_s = last.duration_s
    for index, (c0, c1, c2) in enumerate(zip(last.c0, last.c1, last.c2, strict=True)):
        end_position, end_velocity = _quadratic_end(c0, c1, c2, duration_s)
        if not math.isclose(
            end_position, plan.hold_positions[index], rel_tol=0.0, abs_tol=_CONTINUITY_ATOL
        ):
            raise _invalid("braking hold position differs from the segment endpoint")
        if abs(end_velocity) > _CONTINUITY_ATOL:
            raise _invalid(f"joints[{index}]: braking endpoint velocity is not zero")


class BrakingController:
    """Explicit stop component: plan one brake and own its stability window.

    The controller starts from the last reference that actually reached the
    actuators and never reads wall time or steps physics.  Stability is a
    physical fact measured on the owner thread: the plan duration has elapsed
    and every measured velocity is within its configured tolerance.
    """

    def __init__(self, config: ControlConfig) -> None:
        self._config = config
        self._plan: BrakingPlan | None = None
        self._stop_sim_start_ns: int | None = None
        self._stable_since_ns: int | None = None

    def reset(self) -> None:
        self._plan = None
        self._stop_sim_start_ns = None
        self._stable_since_ns = None

    @property
    def plan(self) -> BrakingPlan | None:
        return self._plan

    @property
    def duration_ns(self) -> int:
        if self._plan is None:
            raise AssertionError("braking has no plan")
        return self._plan.duration_ns

    @property
    def hold_positions(self) -> tuple[float, ...]:
        if self._plan is None:
            raise AssertionError("braking has no plan")
        return self._plan.hold_positions

    @property
    def stop_sim_start_ns(self) -> int | None:
        return self._stop_sim_start_ns

    @property
    def stable_since_ns(self) -> int | None:
        return self._stable_since_ns

    def begin(
        self,
        *,
        base_positions: tuple[float, ...],
        base_velocities: tuple[float, ...],
        joints: Sequence[JointSpec],
        sim_time_ns: int,
        application_sim_time_ns: int | None,
    ) -> BrakingPlan:
        """Plan a bounded brake; ``application_sim_time_ns`` is ``None`` only
        before the first physical application, where the current sim time is
        the brake origin."""
        plan = plan_braking(
            base_positions, base_velocities, joints, self._config.max_accelerations
        )
        self._plan = plan
        self._stop_sim_start_ns = (
            int(sim_time_ns)
            if application_sim_time_ns is None
            else int(application_sim_time_ns)
        )
        self._stable_since_ns = None
        return plan

    def sample(self, elapsed_ns: int) -> ReferenceSample:
        if self._plan is None:
            raise AssertionError("braking has no plan")
        return self._plan.sample(elapsed_ns)

    def update_stability(self, measurement: object, elapsed_ns: int) -> tuple[bool, int]:
        """Return ``(settled, stable_for_ns)`` for one post-step measurement."""
        if self._plan is None:
            raise AssertionError("braking has no plan")
        speeds_ok = all(
            abs(velocity) <= tolerance
            for velocity, tolerance in zip(
                measurement.velocities,  # type: ignore[attr-defined]
                self._config.velocity_tolerances,
                strict=True,
            )
        )
        if elapsed_ns >= self._plan.duration_ns and speeds_ok:
            if self._stable_since_ns is None:
                self._stable_since_ns = measurement.sim_time_ns  # type: ignore[attr-defined]
            stable_for = max(
                0, measurement.sim_time_ns - self._stable_since_ns  # type: ignore[attr-defined]
            )
            return stable_for >= self._config.stable_duration_ns, stable_for
        self._stable_since_ns = None
        return False, 0

    def stable_for_ns(self, measurement: object) -> int:
        if self._stable_since_ns is None:
            return 0
        return max(0, measurement.sim_time_ns - self._stable_since_ns)  # type: ignore[attr-defined]
