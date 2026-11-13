"""Streaming recording decoder and dispatcher for the pure audit rules."""
from __future__ import annotations

from pathlib import Path

from robot_contracts.models import ContractError

from dual_arm_runtime.offline.audit.audit_common import record_error
from dual_arm_runtime.offline.audit.episode_audit import _Audit
from dual_arm_runtime.offline.audit.records import BackendInfo, CameraFrame, Observation, State
from dual_arm_runtime.offline.follower_definition import (
    require_current_follower_reference_definition,
)


def decode_message(
    topic: str,
    message: object,
    root: str,
    reference_topic: str,
):
    from robot_ros_support.conversions import context_from_message, time_ns
    from robot_ros_support.follower_conversions import follower_reference_from_message
    from robot_ros_support.stream_conversions import (
        control_event_from_message,
        execution_feedback_from_message,
        stream_command_from_message,
    )
    if topic == "/runtime/commands":
        return stream_command_from_message(message)
    if topic == reference_topic:
        return follower_reference_from_message(message)
    if topic == f"{root}/control_events":
        return control_event_from_message(message)
    if topic == f"{root}/execution_feedback":
        return execution_feedback_from_message(message)
    context = context_from_message(message.context)
    if topic == f"{root}/info":
        return BackendInfo(context, message.backend_kind, bool(message.test_only),
                           tuple(message.joint_names), tuple(message.camera_names),
                           tuple(message.capabilities))
    if topic == f"{root}/state":
        return State(context, int(message.state_seq), time_ns(message.stamp),
                     time_ns(message.joints.header.stamp), tuple(message.joints.name),
                     tuple(message.joints.position), tuple(message.joints.velocity))
    if topic == "/runtime/observation_index":
        return Observation(context, int(message.observation_id), int(message.state_seq),
                           tuple(message.camera_names), tuple(message.frame_seqs),
                           time_ns(message.reference_time))
    image, info = message.image, message.camera_info
    return CameraFrame(context, int(message.frame_seq), time_ns(image.header.stamp),
                       int(image.width), int(image.height), image.encoding, bytes(image.data),
                       int(image.step), image.header.frame_id, int(info.width), int(info.height),
                       info.header.frame_id, message.calibration_digest)


def _run_audit(audit: _Audit, directory: Path) -> dict:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=str(directory), storage_id="mcap"),
            rosbag2_py.ConverterOptions("", ""),
        )
    except Exception as error:
        raise record_error(f"cannot open recording: {error}") from error
    try:
        types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
        audit.topics = set(types)
        required = {
            f"{audit.root}/info",
            f"{audit.root}/state",
            f"{audit.root}/control_events",
            f"{audit.root}/execution_feedback",
            audit.reference_topic,
            "/runtime/commands",
            "/runtime/observation_index",
        } | {f"{audit.root}/cameras/{name}/frame" for name in audit.camera_names}
        missing = required - audit.topics
        if missing:
            raise record_error("recording is missing topics: " + ", ".join(sorted(missing)))
        require_current_follower_reference_definition(reader, types, rosbag2_py)
        decoded_types = {topic: get_message(types[topic]) for topic in required}
        camera_topics = {f"{audit.root}/cameras/{name}/frame" for name in audit.camera_names}
        while reader.has_next():
            topic, payload, _bag_time_ns = reader.read_next()
            audit.counts[topic] += 1
            if topic not in required:
                continue
            try:
                message = deserialize_message(payload, decoded_types[topic])
                # The original rules skip foreign contexts before decoding event/feedback bodies.
                if topic in {f"{audit.root}/control_events", f"{audit.root}/execution_feedback"}:
                    from robot_ros_support.conversions import context_from_message
                    if not audit.context_value(context_from_message(message.context)):
                        continue
                message = decode_message(
                    topic, message, audit.root, audit.reference_topic,
                )
                if topic == f"{audit.root}/info":
                    audit.info(message)
                elif topic == f"{audit.root}/state":
                    audit.capture_payload(topic, message, payload)
                    audit.state(message)
                elif topic in camera_topics:
                    audit.capture_payload(topic, message, payload)
                    audit.frame(topic, message)
                elif topic == "/runtime/observation_index":
                    audit.observation(message)
                elif topic == "/runtime/commands":
                    audit.command(message)
                elif topic == audit.reference_topic:
                    audit.reference(message)
                elif topic == f"{audit.root}/control_events":
                    audit.event(message)
                elif topic == f"{audit.root}/execution_feedback":
                    audit.feedback_message(message)
            except ContractError:
                raise
            except Exception as error:
                raise record_error(f"{topic}: {type(error).__name__}: {error}") from error
        return audit.finish()
    finally:
        reader.close()


def audit_follower_recording(
    directory: Path,
    episode_id: str,
    configuration: dict,
) -> dict:
    """Validate one MCAP follower recording and return compact policy counters."""
    from dual_arm_runtime.offline.audit.follower_audit import _FollowerAudit

    return _run_audit(_FollowerAudit(episode_id, configuration), directory)


__all__ = ["audit_follower_recording"]


def task_fact_from_message(fact):
    """Freeze just the task metadata consumed by the pure sidecar rules."""
    import json

    from dual_arm_runtime.offline.audit.records import TaskFact

    if fact is None:
        return None
    try:
        metrics = json.loads(str(getattr(fact, "metrics_json", "") or "{}"))
    except (TypeError, ValueError):
        metrics = {}
    stamp = getattr(fact, "sample_time", None)
    return TaskFact(
        phase=str(getattr(fact, "phase", "")),
        failure_reason=str(getattr(fact, "failure_reason", "")),
        task_type=str(getattr(fact, "task_type", "")),
        task_epoch=int(getattr(fact, "task_epoch", 0)),
        state_seq=int(getattr(fact, "state_seq", 0)),
        sample_time_ns=int(getattr(stamp, "sec", 0)) * 1_000_000_000
                       + int(getattr(stamp, "nanosec", 0)),
        task_config_digest=str(getattr(fact, "task_config_digest", "")),
        plan_digest=str(getattr(fact, "plan_digest", "")),
        metrics=metrics if isinstance(metrics, dict) else {},
    )
