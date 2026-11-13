"""Verify a recording's FollowerReference layout before decoding its payloads.

The layout is identified from the definition stored in the recording.  A
recording is accepted only when that definition is the currently installed
one; no earlier field-name layout is silently reinterpreted.
"""

from __future__ import annotations

from robot_contracts.models import ContractError

__all__ = [
    "FOLLOWER_REFERENCE_TYPE",
    "require_current_follower_reference_definition",
]

FOLLOWER_REFERENCE_TYPE = "robot_api_msgs/msg/FollowerReference"


def normalized_definition(text: str) -> str:
    """Return the significant lines of a ros2msg definition.

    Comments, blank lines and ``=`` section separators are not part of the
    wire layout; field lines keep their order, types and bounds.
    """
    lines = []
    for line in text.splitlines():
        content = line.split("#", 1)[0].strip()
        if not content or not content.strip("="):
            continue
        lines.append(" ".join(content.split()))
    return "\n".join(lines)


def recorded_definition(reader: object, type_name: str) -> str:
    """Return the message definition stored in the recording."""
    stored = {
        item.topic_type: item.encoded_message_definition
        for item in reader.get_all_message_definitions()  # type: ignore[attr-defined]
    }
    recorded = stored.get(type_name)
    if recorded is None:
        raise ContractError(
            "RECORDING_ERROR", f"recording has no stored definition for {type_name}"
        )
    return recorded


def installed_definition(rosbag2_py: object, type_name: str) -> str:
    """Return the definition of the currently installed generated type."""
    try:
        return rosbag2_py.LocalMessageDefinitionSource().get_full_text(  # type: ignore[attr-defined]
            type_name
        ).encoded_message_definition
    except Exception as error:
        raise ContractError(
            "RECORDING_ERROR",
            f"cannot read the installed {type_name} definition: {error}",
        ) from error


def require_current_follower_reference_definition(
    reader: object, types: dict[str, str], rosbag2_py: object
) -> None:
    """Raise unless the recorded layout equals the installed one.

    Must run before any FollowerReference payload is deserialized.
    """
    if FOLLOWER_REFERENCE_TYPE not in types.values():
        return
    recorded = recorded_definition(reader, FOLLOWER_REFERENCE_TYPE)
    installed = installed_definition(rosbag2_py, FOLLOWER_REFERENCE_TYPE)
    if normalized_definition(recorded) != normalized_definition(installed):
        raise ContractError(
            "RECORDING_ERROR",
            "recorded FollowerReference definition differs from the installed layout",
        )
