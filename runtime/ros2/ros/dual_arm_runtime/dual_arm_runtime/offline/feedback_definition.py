"""Identify a recording's ExecutionFeedback layout before decoding its payloads.

``robot_api_msgs`` 0.1.0 kept ``GripperFeedback`` and ``FingerFeedback`` as
fixed two-element arrays; 0.2.0 uses bounded ``[<=8]`` sequences, so the same
bytes decode to different values.  The layout is identified from the message
definition stored in the recording and compared with the installed definition
or with the known legacy definition.  Payloads are not touched here; legacy
payloads are decoded by the dedicated legacy reader rather than the current
generated type.
"""

from __future__ import annotations

from robot_contracts.models import ContractError

__all__ = [
    "EXECUTION_FEEDBACK_TYPE",
    "LEGACY_FEEDBACK_DEFINITION",
    "installed_definition",
    "is_legacy_feedback_definition",
    "normalized_definition",
    "recorded_definition",
    "require_supported_feedback_definition",
]

EXECUTION_FEEDBACK_TYPE = "robot_api_msgs/msg/ExecutionFeedback"

# The 0.1.0 layout recorded by the preserved MCAP samples.  ``normalized_definition``
# drops comments, blank lines and section separators; field order, field types
# and array bounds remain part of the comparison.
LEGACY_FEEDBACK_DEFINITION = """\
robot_api_msgs/RunContext context
string command_id
uint64 command_seq
uint64 observation_id
string phase
string reason
string stop_reason
uint64 motion_seq
string stream_id
uint64 chunk_seq
int64 accepted_sim_time_ns
int64 accepted_wall_time_ns
bool has_started
int64 started_sim_time_ns
int64 started_wall_time_ns
bool has_finished
int64 finished_sim_time_ns
int64 finished_wall_time_ns
string motion_phase
int64 reference_application_sim_time_ns
int64 measurement_sim_time_ns
int64 reference_elapsed_ns
int64 stable_for_ns
string[] joint_names
float64[<=64] reference_positions
float64[<=64] reference_velocities
float64[<=64] reference_accelerations
float64[<=64] measured_positions
float64[<=64] measured_velocities
float64[<=64] position_errors
float64[<=64] velocity_errors
string reference_kind
string tracking_quality
int64 tracking_violation_ns
robot_api_msgs/GripperFeedback[2] grippers
MSG: robot_api_msgs/GripperFeedback
string side
string mode
bool position_ok
bool contact_ok
string contact_object
bool ambiguous
float64 target_position_m
float64 initial_reference_m
float64 final_reference_m
float64 measured_position_m
float64 measured_velocity_m_s
float64 target_error_m
robot_api_msgs/FingerFeedback[2] fingers
MSG: robot_api_msgs/FingerFeedback
string side
uint8 finger_index
string body_name
float64 joint_speed_m_s
string[<=8] object_names
float64[<=8] normal_forces
uint32[<=8] contact_counts
MSG: robot_api_msgs/RunContext
string robot_id
string robot_profile_hash
string backend_instance_id
string time_domain_id
uint64 clock_epoch
string episode_id
uint64 control_epoch
"""


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


def is_legacy_feedback_definition(text: str) -> bool:
    """Whether ``text`` is the complete known fixed-array layout."""
    return normalized_definition(text) == normalized_definition(LEGACY_FEEDBACK_DEFINITION)


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


def require_supported_feedback_definition(
    reader: object, types: dict[str, str], rosbag2_py: object
) -> bool:
    """Identify the feedback layout before any payload is deserialized.

    Returns ``True`` when the recording uses the known legacy fixed-array
    layout; callers must decode those payloads with
    :func:`dual_arm_runtime.offline.legacy_feedback.decode_legacy_execution_feedback`
    instead of the current generated type.  Returns ``False`` for the installed
    layout.  Every other layout is rejected.
    """
    if EXECUTION_FEEDBACK_TYPE not in types.values():
        return False
    recorded = recorded_definition(reader, EXECUTION_FEEDBACK_TYPE)
    installed = installed_definition(rosbag2_py, EXECUTION_FEEDBACK_TYPE)
    if normalized_definition(recorded) == normalized_definition(installed):
        return False
    if is_legacy_feedback_definition(recorded):
        return True
    raise ContractError(
        "RECORDING_ERROR",
        "recorded ExecutionFeedback definition is neither the installed layout "
        "nor the known legacy layout",
    )
