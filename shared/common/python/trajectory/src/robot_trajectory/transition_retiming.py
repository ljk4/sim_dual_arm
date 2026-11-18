"""Explicit per-axis execution-target shaping onto the reachable envelope.

The follower plans one bounded Ruckig curve from the inherited reference state to
the last selected target.  A predicted absolute target that is far ahead of the
physically reachable envelope at an early point cannot be followed even by a
maximal-velocity reference, so the unmodified request is rejected.  For a
triggered gripper axis this module keeps every target time and the selection,
replaces the selected-axis path by the straight line from the inherited state to
the predicted last target, and clamps that line point by point into the envelope
reachable under the declared position/velocity/acceleration limits.  This is
explicit execution-target shaping: it does not preserve the original intra-window
open/close sequence.

The original model targets stay with the caller as evidence.  The deviation from
those original targets and the deviation from the adapted execution targets are
distinct quantities; satisfying the follower budget for the adapted path says
nothing about the original one.  A point inside the envelope is only a necessary
condition for a feasible curve, so callers keep the real Ruckig plan,
continuous limit, braking and follower-deviation checks and propagate every
failure.  No speed, acceleration, braking or deviation budget is changed here.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from robot_contracts.action_chunks import ActionWindow, ChunkAnchor
from robot_contracts.models import JointSpec, Target, coordinated_axis_groups

__all__ = [
    "GRIPPER_JOINT_NAMES",
    "AxisRetiming",
    "RetimedWindow",
    "gripper_axes",
    "reachable_bounds",
    "retime_action_window",
]

# Public 16-joint contract names of the two parallel grippers.  Axis selection
# stays a profile/contract lookup; no scenario, robot or task name appears here.
GRIPPER_JOINT_NAMES = ("left_gripper_width", "right_gripper_width")


@dataclass(frozen=True, slots=True)
class AxisRetiming:
    """One retimed axis with the original and adapted selected positions."""

    axis: int
    name: str
    original_positions: tuple[float, ...]
    adapted_positions: tuple[float, ...]
    original_excess: float
    budget: float


@dataclass(frozen=True, slots=True)
class RetimedWindow:
    """A window whose selected gripper targets were retimed when infeasible."""

    window: ActionWindow
    axes: tuple[AxisRetiming, ...]

    @property
    def retimed(self) -> bool:
        return bool(self.axes)


def gripper_axes(joint_names: Sequence[str]) -> tuple[int, ...]:
    """Return the profile positions of the public gripper joints, by name."""
    names = tuple(joint_names)
    return tuple(
        index for index, name in enumerate(names) if name in GRIPPER_JOINT_NAMES
    )


def reachable_bounds(
    initial_position: float,
    initial_velocity: float,
    joint: JointSpec,
    max_acceleration: float,
    time_s: float,
) -> tuple[float, float]:
    """Loose reachable interval under the profile velocity/acceleration limits.

    This is the same necessary envelope used to prove a transition is
    impossible: every reference obeying the limits stays inside it.  Joint
    position bounds are intersected afterwards.
    """
    if not math.isfinite(time_s) or time_s <= 0.0:
        raise ValueError("time_s must be positive and finite")
    if not math.isfinite(max_acceleration) or max_acceleration <= 0.0:
        raise ValueError("max_acceleration must be positive and finite")
    lower_limit = float(joint.minimum)
    upper_limit = float(joint.maximum)
    max_velocity = float(joint.max_velocity)
    velocity = min(max(float(initial_velocity), -max_velocity), max_velocity)
    position = float(initial_position)
    if velocity >= max_velocity:
        upper = position + max_velocity * time_s
    else:
        ramp = (max_velocity - velocity) / max_acceleration
        if time_s <= ramp:
            upper = position + velocity * time_s + 0.5 * max_acceleration * time_s * time_s
        else:
            upper = (position + velocity * ramp + 0.5 * max_acceleration * ramp * ramp
                     + max_velocity * (time_s - ramp))
    if velocity <= -max_velocity:
        lower = position - max_velocity * time_s
    else:
        ramp = (velocity + max_velocity) / max_acceleration
        if time_s <= ramp:
            lower = position + velocity * time_s - 0.5 * max_acceleration * time_s * time_s
        else:
            lower = (position + velocity * ramp - 0.5 * max_acceleration * ramp * ramp
                     - max_velocity * (time_s - ramp))
    return max(lower, lower_limit), min(upper, upper_limit)


def _stop_bounds(
    initial_position: float,
    initial_velocity: float,
    joint: JointSpec,
    max_acceleration: float,
    time_s: float,
) -> tuple[float, float]:
    """Reachable interval at ``time_s`` that can also decelerate to zero velocity."""
    lower_limit = float(joint.minimum)
    upper_limit = float(joint.maximum)
    max_velocity = float(joint.max_velocity)
    velocity = min(max(float(initial_velocity), -max_velocity), max_velocity)
    position = float(initial_position)

    def _max_stop(pos: float, vel: float) -> float:
        t_Brake = abs(vel) / max_acceleration
        if time_s <= t_Brake:
            sign = 1.0 if vel >= 0.0 else -1.0
            return pos + vel * time_s - 0.5 * sign * max_acceleration * time_s * time_s
        t1 = max(0.0, 0.5 * time_s - vel / (2.0 * max_acceleration))
        v_peak = vel + max_acceleration * t1
        if v_peak <= max_velocity:
            t2 = max(0.0, time_s - t1)
            return (
                pos
                + vel * t1
                + 0.5 * max_acceleration * t1 * t1
                + v_peak * t2
                - 0.5 * max_acceleration * t2 * t2
            )
        t_acc = max(0.0, (max_velocity - vel) / max_acceleration)
        t_dec = max_velocity / max_acceleration
        if t_acc + t_dec <= time_s:
            t_cruise = time_s - t_acc - t_dec
            return (
                pos
                + 0.5 * (max_velocity + vel) * t_acc
                + max_velocity * t_cruise
                + 0.5 * max_velocity * t_dec
            )
        return reachable_bounds(pos, vel, joint, max_acceleration, time_s)[1]

    upper = min(_max_stop(position, velocity), upper_limit)
    lower = max(-_max_stop(-position, -velocity), lower_limit)
    return min(lower, upper), max(lower, upper)


def _excess(lower: float, upper: float, position: float) -> float:
    return max(lower - position, position - upper, 0.0)


@dataclass(slots=True)
class _AxisState:
    axis: int
    joint: JointSpec
    budget: float
    acceleration: float
    initial_position: float
    initial_velocity: float
    original: tuple[float, ...]
    bounds: tuple[tuple[float, float], ...]
    excess: float
    end: float
    clamped_end: float
    adapted: list[float]
    triggered: bool


def _line_adapted(
    initial_position: float,
    end: float,
    selected: Sequence[Target],
    bounds: Sequence[tuple[float, float]],
    duration_ns: int,
) -> list[float]:
    adapted: list[float] = []
    for target, (lower, upper) in zip(selected, bounds, strict=True):
        fraction = target.time_ns / duration_ns
        line = initial_position + (end - initial_position) * fraction
        adapted.append(min(max(line, lower), upper))
    return adapted


def _resolve_axis_groups(
    joint_items: Sequence[JointSpec],
    axes_set: set[int],
    axis_groups: Sequence[Sequence[int]] | None,
) -> tuple[tuple[int, ...], ...]:
    raw_groups = (
        coordinated_axis_groups(joint_items)
        if axis_groups is None
        else axis_groups
    )
    count = len(joint_items)
    resolved: list[tuple[int, ...]] = []
    for raw_group in raw_groups:
        for axis in raw_group:
            if axis < 0 or axis >= count:
                raise ValueError(f"axis_groups contains out-of-range axis {axis}")
        filtered = tuple(dict.fromkeys(axis for axis in raw_group if axis in axes_set))
        if len(filtered) > 1:
            resolved.append(filtered)
    return tuple(resolved)


def retime_action_window(
    window: ActionWindow,
    anchor: ChunkAnchor,
    joints: Sequence[JointSpec],
    max_accelerations: Sequence[float],
    max_target_deviations: Sequence[float],
    axes: Sequence[int],
    *,
    axis_groups: Sequence[Sequence[int]] | None = None,
) -> RetimedWindow:
    """Shape selected targets onto the reachable envelope, scaling arm groups together.

    An axis is triggered when some selected target exceeds the reachable envelope
    by more than its follower deviation budget (or the straight-line transition
    exceeds 90% of the budget).  Triggered revolute axes whose endpoints lie
    outside the window-end reachable/stop envelope clamp their endpoint to the
    stop envelope, and all moving axes in the same ``axis_groups`` limb scale
    their displacement proportionally by the most constrained axis ratio so the
    joint-space direction is preserved.  Independent axes (such as grippers)
    stay uncoupled.
    """
    joint_items = tuple(joints)
    count = len(joint_items)
    if len(max_accelerations) != count or len(max_target_deviations) != count:
        raise ValueError("per-axis limits must match the joint dimension")
    selected = window.targets
    if not selected:
        raise ValueError("window has no selected target")
    selected_indices = window.selected_indices
    duration_ns = window.end_time_ns
    if duration_ns <= 0:
        raise ValueError("window has no positive duration")
    source = list(window.source_targets)
    axis_order = tuple(dict.fromkeys(axes))
    states: dict[int, _AxisState] = {}
    for axis in axis_order:
        if axis < 0 or axis >= count:
            raise ValueError(f"axis {axis} is outside the joint dimension")
        joint = joint_items[axis]
        budget = float(max_target_deviations[axis])
        acceleration = float(max_accelerations[axis])
        initial_position = float(anchor.reference.positions[axis])
        initial_velocity = float(anchor.reference.velocities[axis])
        original = tuple(float(target.positions[axis]) for target in selected)
        bounds = tuple(
            reachable_bounds(
                initial_position, initial_velocity, joint, acceleration,
                target.time_ns / 1_000_000_000,
            )
            for target in selected
        )
        excess = max(
            _excess(lower, upper, position)
            for (lower, upper), position in zip(bounds, original, strict=True)
        )
        end = original[-1]
        adapted = _line_adapted(initial_position, end, selected, bounds, duration_ns)
        line_excess = max(
            abs(adapted_pos - orig_pos)
            for adapted_pos, orig_pos in zip(adapted, original, strict=True)
        )
        triggered = excess > budget or line_excess > 0.90 * budget
        clamped_end = end
        if joint.unit == "rad" and _excess(bounds[-1][0], bounds[-1][1], end) > 0.0:
            stop_lower, stop_upper = _stop_bounds(
                initial_position,
                initial_velocity,
                joint,
                acceleration,
                duration_ns / 1_000_000_000,
            )
            eff_lower = max(bounds[-1][0], stop_lower)
            eff_upper = min(bounds[-1][1], stop_upper)
            if eff_lower > eff_upper:
                eff_lower, eff_upper = stop_lower, stop_upper
            clamped_end = min(max(end, eff_lower), eff_upper)
        states[axis] = _AxisState(
            axis=axis,
            joint=joint,
            budget=budget,
            acceleration=acceleration,
            initial_position=initial_position,
            initial_velocity=initial_velocity,
            original=original,
            bounds=bounds,
            excess=excess,
            end=end,
            clamped_end=clamped_end,
            adapted=adapted,
            triggered=triggered,
        )

    grouped_axes: set[int] = set()
    for group in _resolve_axis_groups(joint_items, set(axis_order), axis_groups):
        grouped_axes.update(group)
        alpha_group = 1.0
        for axis in group:
            state = states[axis]
            delta = state.end - state.initial_position
            if state.triggered and abs(delta) > 1e-9 and state.clamped_end != state.end:
                ratio = (state.clamped_end - state.initial_position) / delta
                alpha_group = min(alpha_group, max(0.0, min(1.0, ratio)))
        if alpha_group < 1.0 - 1e-9:
            for axis in group:
                state = states[axis]
                delta = state.end - state.initial_position
                if abs(delta) > 1e-9:
                    coord_end = state.initial_position + alpha_group * delta
                    state.end = coord_end
                    state.adapted = _line_adapted(
                        state.initial_position, coord_end, selected, state.bounds, duration_ns
                    )
                    state.triggered = True
                elif state.triggered and state.clamped_end != state.end:
                    state.end = state.clamped_end
                    state.adapted = _line_adapted(
                        state.initial_position, state.clamped_end, selected,
                        state.bounds, duration_ns,
                    )
            continue
        for axis in group:
            state = states[axis]
            if state.triggered and state.clamped_end != state.end:
                state.end = state.clamped_end
                state.adapted = _line_adapted(
                    state.initial_position, state.clamped_end, selected, state.bounds, duration_ns
                )

    for axis in axis_order:
        if axis in grouped_axes:
            continue
        state = states[axis]
        if state.triggered and state.clamped_end != state.end:
            state.end = state.clamped_end
            state.adapted = _line_adapted(
                state.initial_position, state.clamped_end, selected, state.bounds, duration_ns
            )

    retimed: list[AxisRetiming] = []
    for axis in axis_order:
        state = states[axis]
        if not state.triggered:
            continue
        for index, value in zip(selected_indices, state.adapted, strict=True):
            positions = list(source[index].positions)
            positions[axis] = value
            source[index] = Target(source[index].time_ns, tuple(positions))
        retimed.append(AxisRetiming(
            axis=axis,
            name=state.joint.name,
            original_positions=state.original,
            adapted_positions=tuple(state.adapted),
            original_excess=state.excess,
            budget=state.budget,
        ))
    if not retimed:
        return RetimedWindow(window, ())
    adapted_window = ActionWindow(
        window.observation_id,
        window.observation_time_ns,
        window.handoff_time_ns,
        tuple(source),
        window.target_count,
    )
    return RetimedWindow(adapted_window, tuple(retimed))
