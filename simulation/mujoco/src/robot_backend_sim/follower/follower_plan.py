"""Prepare one bounded-target follower chunk with exactly one Ruckig call.

This module is a math/config adapter, not a controller.  It reuses the accepted
``ActionWindow``, ``ChunkAnchor``, ``ReferencePlan`` and ``PandaControlConfig``
inputs, asks Ruckig Community 0.15.3 once for a second-order time-optimal
trajectory, exports the public phase coefficients, and returns a distinct
:class:`PreparedFollowerPlan`.  It never calls the strict point-by-point
``verify_reference_plan`` and never re-bases from measured state.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from numbers import Integral

from robot_contracts.action_chunks import ActionWindow, ChunkAnchor
from robot_contracts.models import ContractError, JointSpec, RobotProfile, Target
from robot_contracts.trajectory.constants import MAX_HORIZON_NS, VELOCITY_ATOL
from robot_trajectory.follower import PreparedFollowerPlan
from robot_trajectory.follower_math import (
    estimate_raw_terminal_velocities,
    restrict_terminal_velocities,
)
from robot_trajectory.spline_math import validate_reference_inputs

from robot_backend_sim.config.follower_config import PandaFollowerConfig
from robot_backend_sim.config.motion_config import PandaControlConfig
from robot_backend_sim.motion.ruckig_export import export_ruckig_reference

__all__ = ["prepare_follower_plan"]

# The producer identity travels with every accepted record.  Public consumers
# retain it as recorded; they do not require this value.
RUCKIG_ALGORITHM_VERSION = "bounded-target-follower/ruckig-0.15.3/v1"

_NS_PER_S = 1_000_000_000
_VELOCITY_ATOL = 1e-12


def _invalid(location: str, message: str) -> ContractError:
    return ContractError("INVALID_FOLLOWER_REQUEST", f"{location}: {message}")


def _ruckig_input(
    anchor: ChunkAnchor,
    last_target: Target,
    joints: Sequence[JointSpec],
    max_accelerations: Sequence[float],
    target_velocities: Sequence[float],
    minimum_duration_s: float,
    duration_grid_ns: int | None,
):
    try:
        import ruckig
    except ImportError as error:
        raise ContractError(
            "RUCKIG_UNAVAILABLE",
            "Ruckig Community 0.15.3 is required to prepare a follower plan",
        ) from error

    count = len(joints)
    inp = ruckig.InputParameter(count)
    inp.current_position = list(anchor.reference.positions)
    # Only position and velocity are inherited across a hand-off.  Acceleration
    # continuity is not claimed by ChunkAnchor and this round does not add it.
    inp.current_velocity = list(anchor.reference.velocities)
    inp.current_acceleration = [0.0] * count
    inp.target_position = list(last_target.positions)
    inp.target_velocity = list(target_velocities)
    inp.target_acceleration = [0.0] * count
    inp.max_velocity = [float(joint.max_velocity) for joint in joints]
    inp.max_acceleration = [float(value) for value in max_accelerations]
    inp.max_jerk = [math.inf] * count
    inp.minimum_duration = minimum_duration_s
    if duration_grid_ns is not None:
        inp.duration_discretization = ruckig.DurationDiscretization.Discrete
    return ruckig, inp


def _calculate_trajectory(
    anchor: ChunkAnchor,
    last_target: Target,
    joints: Sequence[JointSpec],
    max_accelerations: Sequence[float],
    target_velocities: Sequence[float],
    minimum_duration_s: float,
    duration_grid_ns: int | None,
):
    ruckig, inp = _ruckig_input(
        anchor,
        last_target,
        joints,
        max_accelerations,
        target_velocities,
        minimum_duration_s,
        duration_grid_ns,
    )
    count = len(joints)
    trajectory = ruckig.Trajectory(count)
    try:
        if duration_grid_ns is None:
            calculator = ruckig.Ruckig(count)
        else:
            calculator = ruckig.Ruckig(count, duration_grid_ns / _NS_PER_S)
        result = calculator.calculate(inp, trajectory)
    except Exception as error:  # Ruckig raises RuckigError with a diagnostic string.
        raise ContractError(
            "FOLLOWER_RUCKIG_REJECTED",
            f"Ruckig rejected the bounded target: {error}",
        ) from error
    try:
        result_value = int(result)
    except (TypeError, ValueError) as error:
        raise ContractError(
            "FOLLOWER_RUCKIG_REJECTED",
            "Ruckig returned an unreadable result",
        ) from error
    if result_value < 0:
        raise ContractError(
            "FOLLOWER_RUCKIG_REJECTED",
            f"Ruckig returned error result {result!r}",
        )
    if not math.isfinite(trajectory.duration) or trajectory.duration <= 0.0:
        raise ContractError(
            "FOLLOWER_RUCKIG_REJECTED",
            "Ruckig returned a non-positive or non-finite duration",
        )
    return trajectory


def _validate_types(
    anchor: object,
    window: object,
    profile: object,
    control_config: object,
    follower_config: object,
    is_final: object,
    duration_grid_ns: object,
) -> int | None:
    if not isinstance(anchor, ChunkAnchor):
        raise _invalid("anchor", "must be a ChunkAnchor")
    if not isinstance(window, ActionWindow):
        raise _invalid("window", "must be an ActionWindow")
    if not isinstance(profile, RobotProfile):
        raise _invalid("profile", "must be a RobotProfile")
    if not isinstance(control_config, PandaControlConfig):
        raise _invalid("control_config", "must be a PandaControlConfig")
    if not isinstance(follower_config, PandaFollowerConfig):
        raise _invalid("follower_config", "must be a PandaFollowerConfig")
    if not isinstance(is_final, bool):
        raise _invalid("is_final", "must be a bool")
    if duration_grid_ns is None:
        return None
    if isinstance(duration_grid_ns, bool) or not isinstance(duration_grid_ns, Integral):
        raise _invalid("duration_grid_ns", "must be a positive integer or None")
    grid_ns = int(duration_grid_ns)
    if grid_ns <= 0 or grid_ns > MAX_HORIZON_NS:
        raise _invalid("duration_grid_ns", f"must be in 1..{MAX_HORIZON_NS}")
    return grid_ns


def _validate_identity(
    anchor: ChunkAnchor,
    window: ActionWindow,
    profile: RobotProfile,
    control_config: PandaControlConfig,
    follower_config: PandaFollowerConfig,
) -> None:
    if tuple(profile.joint_names) != tuple(control_config.joint_names):
        raise ContractError(
            "PROFILE_MISMATCH", "profile joint order differs from the control config"
        )
    if tuple(follower_config.joint_names) != tuple(control_config.joint_names):
        raise ContractError(
            "PROFILE_MISMATCH",
            "follower config joint order differs from the control config",
        )
    if anchor.robot_profile_hash != profile.digest:
        raise ContractError(
            "PROFILE_MISMATCH",
            "anchor robot_profile_hash differs from the profile digest",
        )
    if anchor.start_sim_time_ns != window.handoff_time_ns:
        raise _invalid("anchor.start_sim_time_ns", "must equal window.handoff_time_ns")
    if window.target_count != follower_config.target_count:
        raise _invalid(
            "window.target_count",
            f"is {window.target_count}, follower config expects {follower_config.target_count}",
        )


def _validate_window(
    window: ActionWindow,
    follower_config: PandaFollowerConfig,
    grid_ns: int | None,
    *,
    is_final: bool,
) -> int:
    end_time_ns = window.end_time_ns
    if end_time_ns > MAX_HORIZON_NS:
        raise _invalid("window.end_time_ns", "exceeds the shared three-second horizon")
    if end_time_ns > follower_config.max_terminal_tail_duration_ns:
        raise _invalid(
            "window.end_time_ns", "exceeds the configured terminal-tail duration"
        )
    if grid_ns is not None and not is_final and end_time_ns % grid_ns != 0:
        raise _invalid(
            "window.end_time_ns",
            "a non-final fixed hand-off must lie exactly on duration_grid_ns",
        )
    return end_time_ns


def _reference_inputs(
    anchor: ChunkAnchor,
    window: ActionWindow,
    profile: RobotProfile,
    control_config: PandaControlConfig,
    *,
    is_final: bool,
):
    inputs = validate_reference_inputs(
        anchor.reference.positions,
        window.source_targets,
        profile.joints,
        control_config.max_accelerations,
        initial_velocities=anchor.reference.velocities,
    )
    for index, acceleration in enumerate(inputs.max_accelerations):
        if acceleration <= 0.0:
            raise _invalid(f"max_accelerations[{index}]", "must be positive")
    if not window.targets:
        raise _invalid("window.targets", "must not be empty")
    raw_terminal_velocities = estimate_raw_terminal_velocities(window.targets)
    restricted_terminal_velocities = restrict_terminal_velocities(
        raw_terminal_velocities,
        inputs.initial_positions,
        inputs.initial_velocities,
        window.targets[-1].positions,
        inputs.joints,
        inputs.max_accelerations,
        is_final=is_final,
    )
    return inputs, raw_terminal_velocities, restricted_terminal_velocities


def _calculate(
    anchor: ChunkAnchor,
    window: ActionWindow,
    inputs,
    restricted_terminal_velocities,
    end_time_ns: int,
    grid_ns: int | None,
):
    trajectory = _calculate_trajectory(
        anchor,
        window.targets[-1],
        inputs.joints,
        inputs.max_accelerations,
        restricted_terminal_velocities,
        end_time_ns / _NS_PER_S,
        grid_ns,
    )
    profile_duration_ns = int(round(trajectory.duration * _NS_PER_S))
    if grid_ns is not None and profile_duration_ns % grid_ns != 0:
        raise ContractError(
            "FOLLOWER_GRID_MISMATCH",
            f"Ruckig duration {profile_duration_ns}ns is not a multiple of the "
            f"requested {grid_ns}ns grid",
        )
    if profile_duration_ns < end_time_ns:
        raise ContractError(
            "FOLLOWER_INFEASIBLE",
            f"Ruckig duration {profile_duration_ns}ns does not cover the requested "
            f"hand-off {end_time_ns}ns",
        )
    return trajectory, profile_duration_ns


def _truncation(
    profile_duration_ns: int,
    end_time_ns: int,
    follower_config: PandaFollowerConfig,
    grid_ns: int | None,
    *,
    is_final: bool,
) -> int:
    if is_final:
        truncate_ns = profile_duration_ns
        if truncate_ns > follower_config.max_terminal_tail_duration_ns:
            raise ContractError(
                "FOLLOWER_HORIZON_EXCEEDED",
                f"final reference duration {truncate_ns}ns exceeds the configured "
                f"terminal tail {follower_config.max_terminal_tail_duration_ns}ns",
            )
        return truncate_ns
    truncate_ns = end_time_ns
    if grid_ns is not None and truncate_ns % grid_ns != 0:
        raise ContractError(
            "FOLLOWER_GRID_MISMATCH",
            "non-final exported hand-off is not on the requested grid",
        )
    return truncate_ns


def _check_exported(exported, inputs, *, is_final: bool) -> None:
    # Braking feasibility is checked from the exported endpoint, not from a
    # sampled control tick.  The returned braking plan is discarded.
    from robot_backend_sim.control.braking import plan_braking

    try:
        plan_braking(
            exported.end_positions,
            exported.end_velocities,
            inputs.joints,
            inputs.max_accelerations,
        )
    except ContractError as error:
        raise ContractError(
            "FOLLOWER_BRAKING_INFEASIBLE",
            f"exported endpoint cannot be braked within joint limits: {error}",
        ) from error
    if is_final and any(
        abs(velocity) > VELOCITY_ATOL for velocity in exported.end_velocities
    ):
        raise ContractError(
            "FOLLOWER_INFEASIBLE",
            "final Ruckig export did not end at zero velocity",
        )


def prepare_follower_plan(
    anchor: ChunkAnchor,
    window: ActionWindow,
    profile: RobotProfile,
    control_config: PandaControlConfig,
    follower_config: PandaFollowerConfig,
    *,
    is_final: bool,
    duration_grid_ns: int | None = None,
) -> PreparedFollowerPlan:
    """Validate, limit terminal speed, call Ruckig once and export one plan.

    A non-final chunk is cut exactly at ``window.end_time_ns``.  Its exported
    endpoint is the actual reference state that a successor must inherit.  A
    final chunk asks for zero terminal velocity and may extend beyond the window,
    up to the configured terminal-tail bound and the shared three-second horizon.

    ``duration_grid_ns=None`` keeps the accepted continuous planning.  A positive
    integer asks Ruckig for a discrete total duration on that grid; the physical
    caller passes the scene's 2 ms step.  A non-final fixed hand-off must itself
    lie on that grid, and every exported plan duration must be an exact multiple.
    """
    grid_ns = _validate_types(
        anchor,
        window,
        profile,
        control_config,
        follower_config,
        is_final,
        duration_grid_ns,
    )
    _validate_identity(anchor, window, profile, control_config, follower_config)
    end_time_ns = _validate_window(
        window, follower_config, grid_ns, is_final=is_final
    )
    inputs, raw_terminal_velocities, restricted_terminal_velocities = _reference_inputs(
        anchor, window, profile, control_config, is_final=is_final
    )
    trajectory, profile_duration_ns = _calculate(
        anchor,
        window,
        inputs,
        restricted_terminal_velocities,
        end_time_ns,
        grid_ns,
    )
    truncate_ns = _truncation(
        profile_duration_ns,
        end_time_ns,
        follower_config,
        grid_ns,
        is_final=is_final,
    )
    exported = export_ruckig_reference(
        trajectory,
        tuple(inputs.joint_names),
        truncate_ns=truncate_ns,
    )
    _check_exported(exported, inputs, is_final=is_final)
    return PreparedFollowerPlan(
        anchor=anchor,
        window=window,
        plan=exported.plan,
        joints=inputs.joints,
        max_accelerations=inputs.max_accelerations,
        max_target_deviation_limits=follower_config.max_target_deviations,
        raw_terminal_velocities=raw_terminal_velocities,
        restricted_terminal_velocities=restricted_terminal_velocities,
        is_final=is_final,
        requested_window_ns=end_time_ns,
        curve_duration_ns=profile_duration_ns,
        duration_grid_ns=grid_ns,
        algorithm_version=RUCKIG_ALGORITHM_VERSION,
    )
