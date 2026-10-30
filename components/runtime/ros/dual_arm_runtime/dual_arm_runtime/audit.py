"""Offline verification of native ROS MCAP recordings and committed manifests."""

import argparse
import hashlib
import json
import math
from collections import Counter
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from robot_contracts.models import ContractError
from robot_contracts.profiles import semantic_hash, validate_document
from robot_ros_support.conversions import time_ns
from rosidl_runtime_py.utilities import get_message


def audit_recording(directory: Path, episode_id: str, configuration: dict) -> dict:
    max_skew_ns = configuration["observation"]["max_skew_ns"]
    expected_context = configuration["run_context"]
    expected_names = [joint["name"] for joint in configuration["profile"]["joints"]]
    expected_camera = configuration["profile"]["camera"]
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(directory), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {topic.name: get_message(topic.type) for topic in reader.get_all_topics_and_types()}
    counts: Counter = Counter()
    states, frames, indices, commands, events = {}, {}, [], {}, {}
    positions = []
    event_sequences = []
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        message = deserialize_message(payload, types[topic])
        counts[topic] += 1
        context = getattr(message, "context", None)
        if context is None or context.episode_id != episode_id:
            continue
        if any(getattr(context, key) != value for key, value in expected_context.items()):
            raise ContractError("RECORDING_ERROR", "Mixed backend/clock/control context")
        if topic.endswith("/state"):
            if list(message.joints.name) != expected_names:
                raise ContractError("RECORDING_ERROR", "Recorded joint ordering differs")
            if len(message.joints.position) != len(expected_names) or not all(
                math.isfinite(value) for value in message.joints.position
            ):
                raise ContractError("RECORDING_ERROR", "Invalid measured positions")
            states[message.state_seq] = message
            positions.append(tuple(message.joints.position))
        elif topic.endswith("/frame"):
            image = message.image
            if (image.width, image.height) != (expected_camera["width"], expected_camera["height"]):
                raise ContractError("RECORDING_ERROR", "Camera dimensions differ")
            if image.encoding != "rgb8" or len(image.data) != image.width * image.height * 3:
                raise ContractError("RECORDING_ERROR", "Invalid RGB frame")
            frames[(topic.split("/")[-2], message.frame_seq)] = message
        elif topic.endswith("/observation_index"):
            indices.append(message)
        elif topic.endswith("/commands"):
            commands[message.command_id] = message
        elif topic.endswith("/command_events"):
            events.setdefault(message.command_id, []).append(message)
            event_sequences.append(message.event_seq)
    if not states or not frames or not indices or not commands:
        raise ContractError("RECORDING_ERROR", "Missing states, frames, indices or commands")
    for index in indices:
        state = states.get(index.state_seq)
        if state is None:
            raise ContractError("RECORDING_ERROR", "Observation references missing state")
        for name, sequence in zip(index.camera_names, index.frame_seqs, strict=True):
            frame = frames.get((name, sequence))
            if frame is None:
                raise ContractError("RECORDING_ERROR", "Observation references missing frame")
            skew = time_ns(state.stamp) - time_ns(frame.image.header.stamp)
            if not 0 <= skew <= max_skew_ns:
                raise ContractError("RECORDING_ERROR", "Observation is not causal/aligned")
    for command_id, command in commands.items():
        if command.observation_id not in {index.observation_id for index in indices}:
            raise ContractError("RECORDING_ERROR", "Command references missing observation")
        history = events.get(command_id, [])
        if [event.phase for event in history] != ["ACCEPTED", "STARTED", "COMPLETED"]:
            raise ContractError("RECORDING_ERROR", f"Incomplete command events: {command_id}")
        if any(event.trajectory != command.trajectory for event in history):
            raise ContractError("RECORDING_ERROR", "Accepted trajectory differs from sent target")
    if any(
        right != left + 1
        for left, right in zip(
            event_sequences,
            event_sequences[1:],
            strict=False,
        )
    ):
        raise ContractError("RECORDING_ERROR", "Command event sequence gap")
    motion = max(max(values) - min(values) for values in zip(*positions, strict=True))
    if motion <= 1e-6:
        raise ContractError("RECORDING_ERROR", "Mock joints never moved")
    return {
        "message_counts": dict(counts),
        "observations": len(indices),
        "commands": len(commands),
        "frames": len(frames),
        "max_joint_motion": motion,
    }


def inspect_episode(
    directory: Path,
    schema_root: Path = Path("/workspace/contracts/schemas/v1"),
) -> dict:
    manifest = json.loads((directory / "manifest.json").read_text())
    validate_document(manifest, schema_root / "episode_manifest.schema.json")
    if not manifest["complete"]:
        raise ContractError("RECORDING_ERROR", manifest["failure_reason"])
    configuration = manifest["configuration"]
    for name, field in (
        ("profile", "robot_profile_hash"),
        ("observation", "observation_schema_hash"),
        ("action", "action_schema_hash"),
    ):
        if semantic_hash(configuration[name]) != manifest[field]:
            raise ContractError("RECORDING_ERROR", f"Semantic digest mismatch: {name}")
    for relative, digest in manifest["files"].items():
        path = (directory / relative).resolve()
        if not path.is_relative_to(directory.resolve()) or not path.is_file():
            raise ContractError("RECORDING_ERROR", f"Invalid recorded path: {relative}")
        if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ContractError("RECORDING_ERROR", f"Checksum mismatch: {relative}")
    result = audit_recording(
        directory / "recording",
        manifest["episode_id"],
        configuration,
    )
    if result["message_counts"] != manifest["message_counts"]:
        raise ContractError("RECORDING_ERROR", "Manifest counts disagree with recording")
    return {"episode_id": manifest["episode_id"], "complete": True, "test_only": True, **result}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    print(json.dumps(inspect_episode(args.directory), indent=2))


if __name__ == "__main__":
    main()
