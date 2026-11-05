"""Offline streaming verification of native ROS MCAP recordings and manifests."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from collections import Counter
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from robot_contracts.episodes import (
    SUPPORTED_EPISODE_SCHEMA_VERSIONS,
    manifest_schema_path,
)
from robot_contracts.models import CommandStatus, ContractError, Target
from robot_contracts.profiles import (
    normalize_cameras,
    profile_schema_version,
    semantic_hash,
    validate_document,
)
from robot_contracts.status import (
    TERMINAL_PHASES,
    validate_status_against_trajectory,
    validate_status_transition,
)
from robot_ros_support.conversions import (
    command_status_from_message,
    context_from_message,
    time_ns,
)
from rosidl_runtime_py.utilities import get_message

_HASH_CHUNK_BYTES = 1024 * 1024


def sha256_file(path: Path) -> str:
    """Hash a file in fixed-size chunks; callers never read the whole file at once."""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(_HASH_CHUNK_BYTES):
            digest.update(chunk)
    return digest.hexdigest()


def _trajectory_signature(trajectory) -> tuple:
    """Compact equality-preserving semantic value without retaining ROS messages."""
    return (
        (
            str(trajectory.header.frame_id),
            trajectory.header.stamp.sec,
            trajectory.header.stamp.nanosec,
        ),
        tuple(trajectory.joint_names),
        tuple(
            (
                tuple(point.positions),
                tuple(point.velocities),
                tuple(point.accelerations),
                tuple(point.effort),
                point.time_from_start.sec,
                point.time_from_start.nanosec,
            )
            for point in trajectory.points
        ),
    )


def _command_has_terminal(phases: list[str], command_id: str, version: int) -> bool:
    """Validate one command history and report whether it has a terminal event.

    V1 keeps its old stricter rule.  V2 accepts every prefix of
    ACCEPTED -> STARTED -> terminal, plus an ACCEPTED-then-terminal pre-start
    cancellation.  An empty history is an unconfirmed intent, not an execution
    acknowledgement.
    """
    if version == 1:
        if phases != ["ACCEPTED", "STARTED", "COMPLETED"]:
            raise ContractError("RECORDING_ERROR", f"Incomplete command events: {command_id}")
        return True
    if not phases:
        return False
    if phases[0] != "ACCEPTED":
        raise ContractError(
            "RECORDING_ERROR",
            f"Illegal command event order for {command_id}: {phases}",
        )
    if len(phases) == 1:
        return False
    if len(phases) == 2:
        if phases[1] == "STARTED":
            return False
        if phases[1] in TERMINAL_PHASES:
            return True
        raise ContractError(
            "RECORDING_ERROR",
            f"Illegal command event order for {command_id}: {phases}",
        )
    if len(phases) == 3 and phases[1] == "STARTED" and phases[2] in TERMINAL_PHASES:
        return True
    raise ContractError(
        "RECORDING_ERROR",
        f"Command event history after terminal or out of order for {command_id}: {phases}",
    )


def _validate_camera_frame(message, camera, profile_version: int) -> None:
    """Validate one recorded camera frame against its declared profile entry."""
    name = camera.name
    image = message.image
    if (image.width, image.height) != (camera.width, camera.height):
        raise ContractError("RECORDING_ERROR", f"Camera dimensions differ: {name}")
    if image.encoding != "rgb8" or len(image.data) != image.width * image.height * 3:
        raise ContractError("RECORDING_ERROR", f"Invalid RGB frame: {name}")
    if profile_version != 2:
        # Historical v1 bags may omit frame ids, calibration and CameraInfo.
        return
    if image.step != image.width * 3:
        raise ContractError("RECORDING_ERROR", f"Camera RGB step differs: {name}")
    if image.header.frame_id != camera.frame_id:
        raise ContractError("RECORDING_ERROR", f"Camera frame_id differs: {name}")
    camera_info = message.camera_info
    if (camera_info.width, camera_info.height) != (camera.width, camera.height):
        raise ContractError("RECORDING_ERROR", f"CameraInfo dimensions differ: {name}")
    if camera_info.header.frame_id != camera.frame_id:
        raise ContractError("RECORDING_ERROR", f"CameraInfo frame_id differs: {name}")
    if message.calibration_digest != camera.calibration_digest:
        raise ContractError("RECORDING_ERROR", f"Camera calibration differs: {name}")


def audit_recording(
    directory: Path,
    episode_id: str,
    configuration: dict,
    *,
    version: int = 1,
) -> dict:
    """Stream one recording and validate its real content for the selected version.

    The default remains the historical v1 rule for direct callers; manifest-aware
    callers pass the explicit version after reading ``schema_version``.
    """
    if isinstance(version, bool) or version not in SUPPORTED_EPISODE_SCHEMA_VERSIONS:
        raise ContractError(
            "UNSUPPORTED_SCHEMA_VERSION",
            f"episode manifest version {version!r} is not supported",
        )
    if version == 3:
        from dual_arm_runtime.native_audit import audit_native_recording

        return audit_native_recording(directory, episode_id, configuration)
    if version == 4:
        from dual_arm_runtime.native_follower_audit import audit_follower_recording

        return audit_follower_recording(directory, episode_id, configuration)
    max_skew_ns = configuration["observation"]["max_skew_ns"]
    expected_context = configuration["run_context"]
    expected_names = [joint["name"] for joint in configuration["profile"]["joints"]]
    try:
        profile_version = profile_schema_version(configuration["profile"], "<recorded profile>")
        cameras = normalize_cameras(configuration["profile"], "<recorded profile>")
    except ContractError as error:
        raise ContractError("RECORDING_ERROR", f"Invalid recorded profile: {error}") from error
    expected_cameras = tuple(camera.name for camera in cameras)
    cameras_by_name = {camera.name: camera for camera in cameras}
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(directory), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {topic.name: get_message(topic.type) for topic in reader.get_all_topics_and_types()}
    counts: Counter = Counter()
    states: dict[int, int] = {}
    frames: dict[tuple[str, int], int] = {}
    last_frame_sequences: dict[str, int] = {}
    indices: list[tuple[int, int, tuple[str, ...], tuple[int, ...]]] = []
    commands: dict[str, tuple[int, tuple]] = {}
    command_contexts = {}
    command_sequences: dict[str, int] = {}
    command_targets: dict[str, tuple[Target, ...]] = {}
    events: dict[str, list[tuple[str, tuple, int]]] = {}
    event_sequences: list[int] = []
    feedback_latest: dict[str, CommandStatus] = {}
    feedback_declared = any(name.endswith("/command_feedback") for name in types)
    motion_min: list[float] | None = None
    motion_max: list[float] | None = None
    while reader.has_next():
        topic, payload, bag_time_ns = reader.read_next()
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
            positions = tuple(message.joints.position)
            if motion_min is None:
                motion_min = list(positions)
                motion_max = list(positions)
            else:
                for index, value in enumerate(positions):
                    motion_min[index] = min(motion_min[index], value)
                    motion_max[index] = max(motion_max[index], value)
            states[message.state_seq] = time_ns(message.stamp)
        elif topic.endswith("/frame"):
            camera_name = topic.split("/")[-2]
            camera = cameras_by_name.get(camera_name)
            if camera is None:
                raise ContractError(
                    "RECORDING_ERROR",
                    f"Frame topic has an unknown camera: {camera_name}",
                )
            _validate_camera_frame(message, camera, profile_version)
            image = message.image
            if message.frame_seq < 1:
                raise ContractError("RECORDING_ERROR", f"Invalid frame sequence: {camera_name}")
            previous_sequence = last_frame_sequences.get(camera_name)
            if previous_sequence is not None and message.frame_seq <= previous_sequence:
                raise ContractError(
                    "RECORDING_ERROR",
                    f"Frame sequence is not increasing: {camera_name}",
                )
            key = (camera_name, message.frame_seq)
            if key in frames:
                raise ContractError("RECORDING_ERROR", f"Duplicate frame reference: {camera_name}")
            last_frame_sequences[camera_name] = message.frame_seq
            frames[key] = time_ns(image.header.stamp)
        elif topic.endswith("/observation_index"):
            camera_names = tuple(message.camera_names)
            frame_sequences = tuple(message.frame_seqs)
            if camera_names != expected_cameras:
                raise ContractError(
                    "RECORDING_ERROR",
                    "Observation camera names/order differ from profile",
                )
            if len(frame_sequences) != len(camera_names):
                raise ContractError(
                    "RECORDING_ERROR", "Observation frame references are incomplete"
                )
            indices.append(
                (
                    message.observation_id,
                    message.state_seq,
                    camera_names,
                    frame_sequences,
                )
            )
        elif topic.endswith("/commands"):
            commands[message.command_id] = (
                message.observation_id,
                _trajectory_signature(message.trajectory),
            )
            command_contexts[message.command_id] = context_from_message(message.context)
            command_sequences[message.command_id] = int(message.command_seq)
            command_targets[message.command_id] = tuple(
                Target(
                    time_ns(point.time_from_start),
                    tuple(float(value) for value in point.positions),
                )
                for point in message.trajectory.points
            )
        elif topic.endswith("/command_events"):
            events.setdefault(message.command_id, []).append(
                (message.phase, _trajectory_signature(message.trajectory), time_ns(message.stamp))
            )
            event_sequences.append(message.event_seq)
        elif topic.endswith("/command_feedback"):
            status = command_status_from_message(message)
            command_id = status.command_id
            if command_id not in commands:
                raise ContractError(
                    "RECORDING_ERROR", f"Feedback precedes or lacks command: {command_id}"
                )
            try:
                validate_status_against_trajectory(
                    status,
                    command_targets[command_id],
                    expected_context=command_contexts[command_id],
                    expected_command_id=command_id,
                    expected_command_seq=command_sequences[command_id],
                    expected_observation_id=commands[command_id][0],
                    expected_joint_names=expected_names,
                )
            except ContractError as error:
                raise ContractError("RECORDING_ERROR", f"Invalid feedback: {error}") from error
            if bag_time_ns != status.sample_time_ns:
                raise ContractError(
                    "RECORDING_ERROR", "Feedback timestamp differs from sample_time"
                )
            try:
                validate_status_transition(feedback_latest.get(command_id), status)
            except ContractError as error:
                raise ContractError(
                    "RECORDING_ERROR", f"Invalid feedback transition: {error}"
                ) from error
            feedback_latest[command_id] = status
    if version == 1 and (not states or not frames or not indices or not commands):
        raise ContractError("RECORDING_ERROR", "Missing states, frames, indices or commands")
    if profile_version == 2:
        missing_cameras = [name for name in expected_cameras if name not in last_frame_sequences]
        if missing_cameras:
            raise ContractError(
                "RECORDING_ERROR", f"Missing camera frames: {', '.join(missing_cameras)}"
            )
    observation_ids = {index[0] for index in indices}
    for observation_id, state_seq, camera_names, frame_seqs in indices:
        del observation_id
        state_stamp = states.get(state_seq)
        if state_stamp is None:
            raise ContractError("RECORDING_ERROR", "Observation references missing state")
        for name, sequence in zip(camera_names, frame_seqs, strict=True):
            frame_stamp = frames.get((name, sequence))
            if frame_stamp is None:
                raise ContractError("RECORDING_ERROR", "Observation references missing frame")
            skew = state_stamp - frame_stamp
            if not 0 <= skew <= max_skew_ns:
                raise ContractError("RECORDING_ERROR", "Observation is not causal/aligned")
    orphan_events = sorted(set(events) - set(commands))
    if version == 2 and orphan_events:
        raise ContractError(
            "RECORDING_ERROR",
            f"Command events reference missing command: {orphan_events[0]}",
        )
    commands_without_terminal = 0
    terminal_events = 0
    for command_id, (observation_id, signature) in commands.items():
        if observation_id not in observation_ids:
            raise ContractError("RECORDING_ERROR", "Command references missing observation")
        history = events.get(command_id, [])
        phases = [phase for phase, _, _ in history]
        if not _command_has_terminal(phases, command_id, version):
            commands_without_terminal += 1
        if any(phase in TERMINAL_PHASES for phase in phases):
            terminal_events += 1
        if any(event_signature != signature for _, event_signature, _ in history):
            raise ContractError("RECORDING_ERROR", "Accepted trajectory differs from sent target")
    if feedback_declared:
        _validate_feedback_history(
            feedback_latest,
            commands,
            command_contexts,
            command_sequences,
            events,
            observation_ids,
        )
    if any(
        right != left + 1
        for left, right in zip(
            event_sequences,
            event_sequences[1:],
            strict=False,
        )
    ):
        raise ContractError("RECORDING_ERROR", "Command event sequence gap")
    if motion_min is None or motion_max is None:
        motion = 0.0
    else:
        motion = max(
            maximum - minimum for minimum, maximum in zip(motion_min, motion_max, strict=True)
        )
    if version == 1 and motion <= 1e-6:
        raise ContractError("RECORDING_ERROR", "Mock joints never moved")
    return {
        "message_counts": dict(counts),
        "observations": len(indices),
        "commands": len(commands),
        "frames": len(frames),
        "commands_without_terminal": commands_without_terminal,
        "terminal_events": terminal_events,
        "feedback_commands": len(feedback_latest) if feedback_declared else 0,
        "max_joint_motion": motion,
    }


def _positions_match(left: tuple[float, ...], right: tuple[float, ...]) -> bool:
    return len(left) == len(right) and all(
        math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-12) for a, b in zip(left, right, strict=True)
    )


def _validate_feedback_history(
    feedback: dict[str, CommandStatus],
    commands: dict[str, tuple[int, tuple]],
    contexts: dict,
    sequences: dict[str, int],
    events: dict[str, list[tuple[str, tuple, int]]],
    observation_ids: set[int],
) -> None:
    """Match each retained terminal snapshot to the recorded command and ACKs.

    v2 diagnostic recordings can retain a missing terminal event as before;
    commands_without_terminal reports that failure separately. A declared
    feedback stream must still retain every accepted command's terminal sample.
    """
    previous_reference: tuple[float, ...] | None = None
    for command_id, (observation_id, _signature) in commands.items():
        status = feedback.get(command_id)
        history = events.get(command_id, [])
        phases = {phase for phase, _, _ in history}
        if status is None:
            if phases & {"ACCEPTED", "STARTED"}:
                raise ContractError(
                    "RECORDING_ERROR", f"Accepted command has no terminal feedback: {command_id}"
                )
            previous_reference = None
            continue
        if not {"ACCEPTED", "STARTED"} <= phases:
            raise ContractError("RECORDING_ERROR", "Feedback lacks ACCEPTED/STARTED events")
        if status.phase not in TERMINAL_PHASES:
            raise ContractError(
                "RECORDING_ERROR", f"Accepted command has no terminal feedback: {command_id}"
            )
        if (
            status.command_seq != sequences[command_id]
            or status.observation_id != observation_id
            or status.context != contexts[command_id]
        ):
            raise ContractError("RECORDING_ERROR", "Feedback command identity differs")
        if observation_id not in observation_ids:
            raise ContractError("RECORDING_ERROR", "Feedback command observation is missing")
        if previous_reference is not None and not _positions_match(
            status.initial_positions, previous_reference
        ):
            raise ContractError("RECORDING_ERROR", "Feedback start differs from previous terminal")
        stamps = {phase: stamp for phase, _, stamp in history}
        if status.accepted_at_ns != stamps["ACCEPTED"] or status.started_at_ns != stamps["STARTED"]:
            raise ContractError("RECORDING_ERROR", "Feedback ACCEPTED/STARTED time differs")
        terminals = [
            (phase, timestamp) for phase, _, timestamp in history if phase in TERMINAL_PHASES
        ]
        if terminals and (status.phase, status.sample_time_ns) != terminals[-1]:
            raise ContractError("RECORDING_ERROR", "Feedback terminal phase/time differs")
        previous_reference = status.reference_positions


def _check_configuration_and_files(directory: Path, manifest: dict) -> dict:
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
        if sha256_file(path) != digest:
            raise ContractError("RECORDING_ERROR", f"Checksum mismatch: {relative}")
    return configuration


def inspect_episode(
    directory: Path,
    schema_root: Path = Path("/workspace/contracts/schemas"),
) -> dict:
    """Inspect one committed recording, selecting strict rules by manifest version."""
    manifest = json.loads((directory / "manifest.json").read_text())
    if not isinstance(manifest, dict):
        raise ContractError("SCHEMA_MISMATCH", "episode manifest must be an object")
    version = manifest.get("schema_version")
    validate_document(manifest, manifest_schema_path(schema_root, version))
    if version == 1:
        if not manifest["complete"]:
            raise ContractError("RECORDING_ERROR", manifest["failure_reason"])
    elif not manifest["recording"]["complete"]:
        raise ContractError("RECORDING_ERROR", manifest["recording"]["failure_reason"])
    configuration = _check_configuration_and_files(directory, manifest)
    result = audit_recording(
        directory / "recording",
        manifest["episode_id"],
        configuration,
        version=version,
    )
    if result["message_counts"] != manifest["message_counts"]:
        raise ContractError("RECORDING_ERROR", "Manifest counts disagree with recording")
    if version == 4:
        runtime = manifest["runtime"]
        if bool(result.get("test_only", False)) and not bool(manifest["test_only"]):
            raise ContractError(
                "RECORDING_ERROR",
                "test-only policy recording is not marked test_only in the manifest",
            )
        if runtime["outcome"] == "completed":
            if result["policy_chunks"] != result["configured_chunks"]:
                raise ContractError(
                    "RECORDING_ERROR",
                    "completed policy runtime lacks every configured diagnostic chunk",
                )
            if result["final_chunks"] != 1:
                raise ContractError(
                    "RECORDING_ERROR",
                    "completed policy runtime does not end in one explicit final block",
                )
            if result["raw_prediction_points"] < (
                result["configured_chunks"] * result["chunk_size"]
            ):
                raise ContractError(
                    "RECORDING_ERROR",
                    "completed policy runtime did not retain every raw prediction",
                )
            if result["applied_chunks"] < result["policy_chunks"]:
                raise ContractError(
                    "RECORDING_ERROR",
                    "completed policy runtime has an accepted but unapplied chunk",
                )
            if result["max_joint_motion"] <= 1e-6:
                raise ContractError(
                    "RECORDING_ERROR", "completed policy runtime never moved the joints"
                )
        return {
            "schema_version": 4,
            "episode_id": manifest["episode_id"],
            "source": manifest["source"],
            "test_only": manifest["test_only"],
            "recording": dict(manifest["recording"]),
            "runtime": dict(runtime),
            "task": dict(manifest["task"]),
            "native_format": "v4",
            "policy_test_only": bool(result.get("test_only", False)),
            **{key: value for key, value in result.items() if key != "test_only"},
        }
    if version == 3:
        runtime = manifest["runtime"]
        if runtime["outcome"] == "completed":
            required = int(result["configured_motions"])
            if result["commands"] < required or result["completed_commands"] < required:
                raise ContractError(
                    "RECORDING_ERROR",
                    "completed native runtime lacks every configured diagnostic motion",
                )
            if result["max_joint_motion"] <= 1e-6:
                raise ContractError(
                    "RECORDING_ERROR", "completed native runtime never moved the joints"
                )
        return {
            "schema_version": 3,
            "episode_id": manifest["episode_id"],
            "source": manifest["source"],
            "test_only": manifest["test_only"],
            "recording": dict(manifest["recording"]),
            "runtime": dict(runtime),
            "task": dict(manifest["task"]),
            "native_format": "v3",
            "runtime_completed": runtime["outcome"] == "completed",
            **result,
        }
    if version == 1:
        # Keep the historical call meaning: complete=true means the old all-in-one
        # success rule passed.  The v2 dimensions are exposed only as clearly
        # inferred compatibility values, not as newly claimed task labels.
        return {
            "schema_version": 1,
            "episode_id": manifest["episode_id"],
            "complete": True,
            "test_only": True,
            "recording": {"complete": True, "failure_reason": ""},
            "runtime": {"outcome": "completed", "reason": ""},
            "task": {"outcome": "not_evaluated", "reason": ""},
            **result,
        }
    return {
        "schema_version": 2,
        "episode_id": manifest["episode_id"],
        "source": manifest["source"],
        "test_only": manifest["test_only"],
        "recording": dict(manifest["recording"]),
        "runtime": dict(manifest["runtime"]),
        "task": dict(manifest["task"]),
        **result,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    print(json.dumps(inspect_episode(args.directory), indent=2))


if __name__ == "__main__":
    main()
