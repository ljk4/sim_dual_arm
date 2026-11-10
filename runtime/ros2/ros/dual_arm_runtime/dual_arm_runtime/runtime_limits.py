"""Runtime configuration checks for control limits and task monitoring."""

from __future__ import annotations

from robot_contracts.models import ContractError

from dual_arm_runtime.runtime_config import RuntimeConfiguration


def task_enabled(configuration: RuntimeConfiguration) -> bool:
    recipe = configuration.recipe
    stream = recipe.policy if recipe.action_source == "policy" else recipe.teacher
    return recipe.action_source == "teacher" or (
        stream is not None and stream.termination_mode == "task"
    )


def control_limits(configuration: RuntimeConfiguration) -> tuple[float, ...]:
    names = configuration.control_document.get("joint_names")
    limits = configuration.control_document.get("max_accelerations")
    if list(names or ()) != list(configuration.profile.joint_names):
        raise ContractError(
            "SCHEMA_MISMATCH", "control joint_names differ from the robot profile"
        )
    if not isinstance(limits, list) or len(limits) != len(configuration.profile.joints):
        raise ContractError(
            "SCHEMA_MISMATCH", "control max_accelerations must match the profile joints"
        )
    result: list[float] = []
    for index, value in enumerate(limits):
        if isinstance(value, bool) or not isinstance(value, int | float):
            raise ContractError(
                "SCHEMA_MISMATCH", f"max_accelerations[{index}] must be a number"
            )
        number = float(value)
        if not number >= 0.0:
            raise ContractError(
                "SCHEMA_MISMATCH", f"max_accelerations[{index}] must be non-negative"
            )
        result.append(number)
    return tuple(result)


__all__ = ["control_limits", "task_enabled"]
