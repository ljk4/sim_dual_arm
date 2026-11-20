"""Runtime control limits and common stream configuration bounds."""

from __future__ import annotations

from typing import TYPE_CHECKING

from robot_contracts.models import ContractError

if TYPE_CHECKING:
    from dual_arm_runtime.runtime_config import RuntimeConfiguration

INT64_MAX = 2**63 - 1
MAX_CHUNKS = 2048
MAX_TASK_BUDGET_NS = 300_000_000_000
STREAM_CHUNK_SIZE = 60
STREAM_PERIOD_NS = 50_000_000


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


__all__ = ["control_limits"]
