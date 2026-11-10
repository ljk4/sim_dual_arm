"""Concrete follower validation limits derived from one runtime configuration."""

from __future__ import annotations

from collections.abc import Sequence

from robot_contracts.models import ContractError
from robot_contracts.trajectory import FollowerValidationLimits

from dual_arm_runtime.runtime_config import RuntimeConfiguration


def follower_limits(
    configuration: RuntimeConfiguration, max_accelerations: Sequence[float]
) -> FollowerValidationLimits:
    """Build the concrete 031 validation limits from configuration only."""
    follower = configuration.recipe.follower
    if follower is None:
        raise ContractError("FOLLOWER_DISABLED", "follower constraints are absent")
    return FollowerValidationLimits(
        joint_names=configuration.profile.joint_names,
        max_accelerations=tuple(float(value) for value in max_accelerations),
        max_target_deviations=follower.max_target_deviations,
        target_count=follower.target_count,
        max_terminal_tail_duration_ns=follower.max_terminal_tail_duration_ns,
        duration_grid_ns=follower.duration_grid_ns,
    )


__all__ = ["follower_limits"]
