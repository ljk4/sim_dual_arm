"""Identify a recording's FollowerReference layout before decoding its payloads.

``robot_api_msgs`` 0.2.0 named the producer curve duration
``ruckig_duration_ns``; 0.3.0 names it ``curve_duration_ns``.  The field type,
order, bounds and every nested definition are unchanged, so old CDR bytes decode
with the current generated type.  The stored digest, however, covers the old
field name, so legacy records must be verified with the v1 payload.

The layout is identified from the definition stored in the recording.  A
recording is accepted when its definition is the installed one, or when it
differs from it only by that one known field name.
"""

from __future__ import annotations

from robot_contracts.models import ContractError

from dual_arm_runtime.offline.feedback_definition import (
    installed_definition,
    normalized_definition,
    recorded_definition,
)

__all__ = [
    "FOLLOWER_REFERENCE_TYPE",
    "is_legacy_follower_reference_definition",
    "require_current_follower_reference_definition",
]

FOLLOWER_REFERENCE_TYPE = "robot_api_msgs/msg/FollowerReference"

LEGACY_DURATION_FIELD = "uint64 ruckig_duration_ns"
CURRENT_DURATION_FIELD = "uint64 curve_duration_ns"


def is_legacy_follower_reference_definition(
    recorded: str, installed: str
) -> bool:
    """Whether ``recorded`` is the previous layout with the known field rename."""
    lines = normalized_definition(recorded)
    if lines.count(LEGACY_DURATION_FIELD) != 1:
        return False
    return lines.replace(LEGACY_DURATION_FIELD, CURRENT_DURATION_FIELD) == (
        normalized_definition(installed)
    )


def require_current_follower_reference_definition(
    reader: object, types: dict[str, str], rosbag2_py: object
) -> bool:
    """Return whether decode must use the legacy digest payload.

    Must run before any FollowerReference payload is deserialized.
    """
    if FOLLOWER_REFERENCE_TYPE not in types.values():
        return False
    recorded = recorded_definition(reader, FOLLOWER_REFERENCE_TYPE)
    installed = installed_definition(rosbag2_py, FOLLOWER_REFERENCE_TYPE)
    if normalized_definition(recorded) == normalized_definition(installed):
        return False
    if is_legacy_follower_reference_definition(recorded, installed):
        return True
    raise ContractError(
        "RECORDING_ERROR",
        "recorded FollowerReference definition is neither the installed layout "
        "nor the previous layout with the renamed curve-duration field",
    )
