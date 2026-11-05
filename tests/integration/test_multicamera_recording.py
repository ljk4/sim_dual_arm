"""Real-MCAP three-camera recording and audit checks; run inside ROS Jazzy."""

import json
from dataclasses import asdict
from pathlib import Path

import pytest
import rosbag2_py
import yaml
from dual_arm_runtime.audit import inspect_episode
from dual_arm_runtime.configuration import load_configuration
from dual_arm_runtime.recording import EpisodeRecorder
from rclpy.serialization import deserialize_message
from robot_api_msgs.msg import CameraFrame, CommandEvent, ObservationIndex, RobotState
from robot_contracts.models import ContractError, RunContext, Target, Trajectory
from robot_contracts.profiles import load_document
from robot_ros_support.conversions import (
    command_message,
    context_message,
    stamp,
    trajectory_message,
)
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Header

ROOT = Path("/workspace")
SCHEMAS = ROOT / "contracts/schemas"
PANDA_PROFILE = ROOT / "profiles/robots/panda_stack_v1/profile.json"
PANDA_OBSERVATION = ROOT / "profiles/observations/panda_rgb_joint_v1.json"
PANDA_ACTION = ROOT / "profiles/actions/panda_joint_position_v1.json"
BASE_CONFIG = ROOT / "runtime/ros2/configs/mock_collect.yaml"
EPISODE_ID = "episode-multicamera"


def write_three_camera_config(tmp_path):
    """Build an explicitly test-only v2 profile and its otherwise normal recipe."""
    profile = load_document(PANDA_PROFILE)
    profile["test_only"] = True
    profile_path = tmp_path / "panda_test_profile.json"
    profile_path.write_text(json.dumps(profile))
    recipe = yaml.safe_load(BASE_CONFIG.read_text())
    recipe["robot_profile"] = str(profile_path)
    recipe["observation_schema"] = str(PANDA_OBSERVATION)
    recipe["action_schema"] = str(PANDA_ACTION)
    recipe["script_targets"] = [
        {
            "duration_ns": 100_000_000,
            "positions": [joint["initial"] for joint in profile["joints"]],
        }
    ]
    recipe["command_timeout_s"] = 1.0
    config_path = tmp_path / "runtime.yaml"
    config_path.write_text(yaml.safe_dump(recipe))
    return load_configuration(ROOT, config_path)


def _topics(configuration):
    topics = {
        "/robot/state": "robot_api_msgs/msg/RobotState",
        "/runtime/observation_index": "robot_api_msgs/msg/ObservationIndex",
        "/runtime/control/commands": "robot_api_msgs/msg/TrajectoryCommand",
        "/robot/command_events": "robot_api_msgs/msg/CommandEvent",
    }
    for camera in configuration.profile.cameras:
        topics[f"/robot/cameras/{camera.name}/frame"] = "robot_api_msgs/msg/CameraFrame"
    return topics


def _read_messages(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {topic.name: get_message(topic.type) for topic in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        yield topic, deserialize_message(payload, types[topic])


def write_episode(tmp_path, configuration, fault=""):
    context = RunContext(
        configuration.profile.robot_id,
        configuration.profile.digest,
        "backend-multicamera",
        "mock-system",
        0,
        EPISODE_ID,
        1,
    )
    output = tmp_path / "raw"
    recorder = EpisodeRecorder(
        output,
        EPISODE_ID,
        {**configuration.snapshot, "run_context": asdict(context)},
        SCHEMAS,
        _topics(configuration),
        run_id="test-multicamera",
        source="mock",
        test_only=True,
    )
    positions = list(configuration.profile.initial_positions)
    for sequence in (1, 2):
        timestamp = sequence * 100_000_000
        positions[0] = 0.01 * sequence
        recorder.append(
            "/robot/state",
            RobotState(
                context=context_message(context),
                state_seq=sequence,
                stamp=stamp(timestamp),
                joints=JointState(
                    name=list(configuration.profile.joint_names),
                    position=positions,
                ),
                status="READY",
            ),
            timestamp,
        )
        for camera_index, camera in enumerate(configuration.profile.cameras):
            if fault == "missing" and camera.name == "left_wrist":
                continue
            frame_id = camera.frame_id
            if fault == "frame_id" and camera.name == "left_wrist":
                frame_id = "right_wrist_optical_frame"
            calibration = camera.calibration_digest
            if fault == "calibration" and camera.name == "right_wrist":
                calibration = "0" * 64
            step = camera.width * 3
            if fault == "step" and camera.name == "left_wrist":
                step += 3
            palette = bytes([10 + camera_index, 20 + camera_index, 30 + camera_index])
            image_header = Header(stamp=stamp(timestamp), frame_id=frame_id)
            image = Image(
                header=image_header,
                width=camera.width,
                height=camera.height,
                encoding="rgb8",
                step=step,
                data=palette * (camera.width * camera.height),
            )
            info_width, info_height = camera.width, camera.height
            if fault == "camera_info_size" and camera.name == "right_wrist":
                info_width += 1
                info_height += 1
            info_frame_id = camera.frame_id
            if fault == "camera_info_frame" and camera.name == "left_wrist":
                info_frame_id = "wrong_optical_frame"
            camera_info_header = Header(stamp=stamp(timestamp), frame_id=info_frame_id)
            recorder.append(
                f"/robot/cameras/{camera.name}/frame",
                CameraFrame(
                    context=context_message(context),
                    frame_seq=sequence,
                    image=image,
                    camera_info=CameraInfo(
                        header=camera_info_header,
                        width=info_width,
                        height=info_height,
                    ),
                    calibration_digest=calibration,
                ),
                timestamp,
            )
        camera_names = list(configuration.profile.camera_names)
        if fault == "cross_camera":
            camera_names[1], camera_names[2] = camera_names[2], camera_names[1]
        recorder.append(
            "/runtime/observation_index",
            ObservationIndex(
                context=context_message(context),
                observation_id=sequence,
                state_seq=sequence,
                camera_names=camera_names,
                frame_seqs=[sequence] * len(camera_names),
                reference_time=stamp(timestamp),
            ),
            timestamp,
        )
    trajectory = Trajectory(
        context,
        "lease",
        "command-multicamera",
        1,
        2,
        1_000_000_000,
        configuration.profile.joint_names,
        (Target(100_000_000, tuple(configuration.profile.initial_positions)),),
    )
    recorder.append("/runtime/control/commands", command_message(trajectory), 100_000_000)
    event_trajectory = trajectory_message(trajectory)
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
                trajectory=event_trajectory,
            ),
            sequence * 100_000_000,
        )
    return recorder.finish()


def test_three_camera_recording_commits_and_audits_every_stream(tmp_path):
    configuration = write_three_camera_config(tmp_path)
    assert configuration.profile.test_only is True
    assert configuration.profile.camera_names == ("global", "left_wrist", "right_wrist")

    result = write_episode(tmp_path, configuration)
    assert result.name == EPISODE_ID
    report = inspect_episode(result, SCHEMAS)
    assert report["recording"]["complete"] is True
    assert report["observations"] == 2
    assert report["frames"] == 6

    by_camera = {}
    for topic, message in _read_messages(result / "recording"):
        if topic.endswith("/frame"):
            by_camera.setdefault(topic, []).append(message)
    assert set(by_camera) == {
        f"/robot/cameras/{camera.name}/frame" for camera in configuration.profile.cameras
    }
    palettes = set()
    for camera in configuration.profile.cameras:
        messages = by_camera[f"/robot/cameras/{camera.name}/frame"]
        assert len(messages) == 2
        for message in messages:
            assert message.image.header.frame_id == camera.frame_id
            assert message.calibration_digest == camera.calibration_digest
            assert (message.image.width, message.image.height) == (camera.width, camera.height)
            assert len(message.image.data) == camera.width * camera.height * 3
        palettes.add(bytes(messages[0].image.data[:3]))
    assert len(palettes) == 3


@pytest.mark.parametrize(
    ("fault", "reason"),
    [
        ("missing", "Missing camera frames"),
        ("frame_id", "Camera frame_id differs"),
        ("calibration", "Camera calibration differs"),
        ("step", "Camera RGB step differs"),
        ("camera_info_size", "CameraInfo dimensions differ"),
        ("camera_info_frame", "CameraInfo frame_id differs"),
        ("cross_camera", "Observation camera names/order differ"),
    ],
)
def test_bad_camera_stream_never_commits(tmp_path, fault, reason):
    configuration = write_three_camera_config(tmp_path)
    result = write_episode(tmp_path, configuration, fault)
    assert result.name.startswith(".partial-")
    manifest = json.loads((result / "manifest.json").read_text())
    assert manifest["recording"]["complete"] is False
    assert reason in manifest["recording"]["failure_reason"]
    with pytest.raises(ContractError, match=reason):
        inspect_episode(result, SCHEMAS)
