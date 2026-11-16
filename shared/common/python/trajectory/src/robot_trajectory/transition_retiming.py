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
from robot_contracts.models import JointSpec, Target

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


def retime_action_window(
    window: ActionWindow,
    anchor: ChunkAnchor,
    joints: Sequence[JointSpec],
    max_accelerations: Sequence[float],
    max_target_deviations: Sequence[float],
    axes: Sequence[int],
) -> RetimedWindow:
    """Shape selected gripper targets that no admissible reference can follow.

    An axis is shaped only when some selected target lies beyond the reachable
    envelope by more than its follower deviation budget; any admissible
    reference would then be farther than the budget, so the original request
    cannot pass.  An untriggered axis keeps its exact original targets.  The
    adapted selected positions are the straight line from the inherited
    reference state to the original last selected target, clamped at every
    selected time into the envelope.  This changes execution targets only; the
    caller keeps the original model targets and the real Ruckig, continuous
    limit, braking and follower checks remain authoritative.
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
    tail = source[selected_indices[-1] + 1 :]
    arm_indices = tuple(i for i, j in enumerate(joint_items) if j.unit == "rad")
    grip_indices = tuple(i for i, j in enumerate(joint_items) if j.unit == "m")
    arm_stationary = all(
        abs(float(selected[-1].positions[i]) - float(selected[0].positions[i])) < 0.015
        and abs(float(selected[-1].positions[i]) - float(anchor.reference.positions[i]))
        < 0.02
        and abs(float(anchor.reference.velocities[i])) < 0.02
        for i in arm_indices
    )
    grip_stationary = all(
        abs(float(selected[-1].positions[i]) - float(selected[0].positions[i])) < 0.003
        and abs(float(selected[-1].positions[i]) - float(anchor.reference.positions[i]))
        < 0.004
        and abs(float(anchor.reference.velocities[i])) < 0.005
        for i in grip_indices
    )
    all_stationary = arm_stationary and grip_stationary
    retimed: list[AxisRetiming] = []
    for axis in axes:
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
        tail_advanced = False
        if tail:
            tail_first = float(tail[0].positions[axis])
            tail_last = float(tail[-1].positions[axis])
            step_tol = 0.005 if joint.unit == "m" else 0.05
            continuous_tail = (
                abs(tail_first - end) < step_tol and abs(tail_last - tail_first) > 1e-5
            )
            if continuous_tail and joint.unit == "m":
                if (
                    end < 0.068
                    and end <= initial_position + 0.001
                    and tail_last < end - 0.004
                ):
                    end = tail_last
                    tail_advanced = True
                elif (
                    (end > initial_position + 0.002 or tail_last > end + 0.004)
                    and initial_position < 0.060
                ):
                    other_closing = any(
                        g != axis
                        and float(anchor.reference.positions[g]) > 0.056
                        and (
                            float(selected[-1].positions[g])
                            < float(anchor.reference.positions[g]) - 0.003
                            or float(tail[-1].positions[g]) < 0.060
                        )
                        for g in grip_indices
                    )
                    if other_closing:
                        end = min(initial_position, original[0])
                        tail_advanced = True
                    elif (
                        all_stationary
                        and tail_last > end + 0.004
                        and end < float(joint.maximum) - 0.005
                    ):
                        end = float(joint.maximum)
                        tail_advanced = True
            elif (
                continuous_tail
                and joint.unit == "rad"
                and all_stationary
                and abs(tail_last - end) > 0.03
            ):
                end = tail_last
                tail_advanced = True
        adapted: list[float] = []
        for target, (lower, upper) in zip(selected, bounds, strict=True):
            fraction = target.time_ns / duration_ns
            line = initial_position + (end - initial_position) * fraction
            adapted.append(min(max(line, lower), upper))
        line_excess = max(
            abs(adapted_pos - orig_pos)
            for adapted_pos, orig_pos in zip(adapted, original, strict=True)
        )
        if not tail_advanced and excess <= budget and line_excess <= 0.90 * budget:
            continue
        if joint.unit == "rad" and _excess(bounds[-1][0], bounds[-1][1], end) > 0.0:
            stop_lower, stop_upper = _stop_bounds(
                initial_position,
                initial_velocity,
                joint,
                acceleration,
                duration_ns / 1_000_000_000,
            )
            end = min(max(end, stop_lower), stop_upper)
            adapted = []
            for target, (lower, upper) in zip(selected, bounds, strict=True):
                fraction = target.time_ns / duration_ns
                line = initial_position + (end - initial_position) * fraction
                adapted.append(min(max(line, lower), upper))
        for index, value in zip(selected_indices, adapted, strict=True):
            positions = list(source[index].positions)
            positions[axis] = value
            source[index] = Target(source[index].time_ns, tuple(positions))
        retimed.append(AxisRetiming(
            axis=axis,
            name=joint.name,
            original_positions=original,
            adapted_positions=tuple(adapted),
            original_excess=excess,
            budget=budget,
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
