"""Native MCAP and atomic commit tests; run inside the ROS test image."""

import json
from dataclasses import asdict
from pathlib import Path

import pytest
from dual_arm_runtime.audit import inspect_episode
from dual_arm_runtime.configuration import load_configuration
from dual_arm_runtime.recording import EpisodeRecorder
from robot_api_msgs.msg import CameraFrame, CommandEvent, ObservationIndex, RobotState
from robot_contracts.models import ContractError, RunContext, Target, Trajectory
from robot_ros_support.conversions import (
    command_message,
    context_message,
    stamp,
    trajectory_message,
)
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Header

ROOT = Path("/workspace")
SCHEMAS = ROOT / "contracts/schemas/v1"


def write_episode(output: Path, *, missing_frame: bool = False, fail: str = "") -> Path:
    config = load_configuration(ROOT, ROOT / "configs/runtime/mock_collect.yaml")
    context = RunContext(
        config.profile.robot_id,
        config.profile.digest,
        "backend-test",
        "mock-system",
        0,
        "episode-test",
        1,
    )
    topics = {
        "/robot/state": "robot_api_msgs/msg/RobotState",
        "/robot/cameras/front/frame": "robot_api_msgs/msg/CameraFrame",
        "/runtime/observation_index": "robot_api_msgs/msg/ObservationIndex",
        "/runtime/control/commands": "robot_api_msgs/msg/TrajectoryCommand",
        "/robot/command_events": "robot_api_msgs/msg/CommandEvent",
    }
    recorder = EpisodeRecorder(
        output,
        context.episode_id,
        {**config.snapshot, "run_context": asdict(context)},
        SCHEMAS,
        topics,
    )
    positions = list(config.profile.initial_positions)
    for sequence in (1, 2):
        positions[0] = (sequence - 1) * 0.2
        timestamp = sequence * 100_000_000
        state = RobotState(
            context=context_message(context),
            state_seq=sequence,
            stamp=stamp(timestamp),
            joints=JointState(
                name=list(config.profile.joint_names),
                position=positions,
            ),
            status="READY",
        )
        recorder.append("/robot/state", state, timestamp)
        image = Image(
            header=Header(stamp=stamp(timestamp)),
            width=64,
            height=48,
            encoding="rgb8",
            step=192,
            data=bytes([1, 2, 3]) * 64 * 48,
        )
        recorder.append(
            "/robot/cameras/front/frame",
            CameraFrame(
                context=context_message(context),
                frame_seq=sequence,
                image=image,
                camera_info=CameraInfo(),
                calibration_digest="test",
            ),
            timestamp,
        )
        recorder.append(
            "/runtime/observation_index",
            ObservationIndex(
                context=context_message(context),
                observation_id=sequence,
                state_seq=sequence,
                camera_names=["front"],
                frame_seqs=[999 if missing_frame else sequence],
                reference_time=stamp(timestamp),
            ),
            timestamp,
        )
    trajectory = Trajectory(
        context,
        "lease",
        "command-test",
        1,
        1,
        1_000_000_000,
        config.profile.joint_names,
        (Target(800_000_000, tuple(positions)),),
    )
    recorder.append("/runtime/control/commands", command_message(trajectory), 100_000_000)
    for sequence, phase in enumerate(("ACCEPTED", "STARTED", "COMPLETED"), start=1):
        recorder.append(
            "/robot/command_events",
            CommandEvent(
                context=context_message(context),
                event_seq=sequence,
                stamp=stamp(sequence * 100_000_000),
                command_id=trajectory.command_id,
                command_seq=1,
                phase=phase,
                trajectory=trajectory_message(trajectory),
            ),
            sequence * 100_000_000,
        )
    return recorder.finish(fail)


def test_checked_recording_commits_and_reopens(tmp_path):
    result = write_episode(tmp_path)
    assert result.name == "episode-test"
    report = inspect_episode(result, SCHEMAS)
    assert report["complete"] is True
    assert report["observations"] == 2
    assert report["commands"] == 1


def test_missing_reference_never_commits(tmp_path):
    result = write_episode(tmp_path, missing_frame=True)
    assert result.name.startswith(".partial-")
    manifest = json.loads((result / "manifest.json").read_text())
    assert not manifest["complete"]
    with pytest.raises(ContractError, match="RECORDING_ERROR"):
        inspect_episode(result, SCHEMAS)


def test_known_failure_preserves_incomplete_recording(tmp_path):
    result = write_episode(tmp_path, fail="EPISODE_STOPPED")
    assert result.name.startswith(".partial-")
    assert json.loads((result / "manifest.json").read_text())["failure_reason"] == "EPISODE_STOPPED"


def test_recording_corruption_is_detected(tmp_path):
    result = write_episode(tmp_path)
    file = next((result / "recording").glob("*.mcap"))
    with file.open("ab") as stream:
        stream.write(b"corrupted")
    with pytest.raises(ContractError, match="Checksum mismatch"):
        inspect_episode(result, SCHEMAS)


def test_semantic_metadata_corruption_is_detected(tmp_path):
    result = write_episode(tmp_path)
    manifest_path = result / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    manifest["configuration"]["profile"]["joints"][0]["unit"] = "m"
    manifest_path.write_text(json.dumps(manifest))
    with pytest.raises(ContractError, match="Semantic digest mismatch"):
        inspect_episode(result, SCHEMAS)
