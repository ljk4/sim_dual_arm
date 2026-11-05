"""Native MCAP and atomic commit tests; run inside the ROS test image."""

import hashlib
import json
import threading
import time
from dataclasses import asdict
from pathlib import Path

import pytest
from dual_arm_runtime import recording as recording_module
from dual_arm_runtime.audit import inspect_episode, sha256_file
from dual_arm_runtime.configuration import load_configuration
from dual_arm_runtime.recording import EpisodeRecorder
from robot_api_msgs.msg import (
    CameraFrame,
    CommandEvent,
    CommandStatus,
    ObservationIndex,
    RobotState,
)
from robot_contracts.episodes import RuntimeOutcome, RuntimeResult
from robot_contracts.models import ContractError, RunContext, Target, Trajectory
from robot_ros_support.conversions import (
    command_message,
    context_message,
    stamp,
    time_ns,
    trajectory_message,
)
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Header

ROOT = Path("/workspace")
SCHEMAS = ROOT / "contracts/schemas"


def write_episode(
    output: Path,
    *,
    missing_frame: bool = False,
    runtime: RuntimeResult | None = None,
    event_frame_id: str = "",
    phases: tuple[str, ...] = ("ACCEPTED", "STARTED", "COMPLETED"),
    moving: bool = True,
    with_commands: bool = True,
    on_recorder=None,
    feedback: tuple[CommandStatus, ...] = (),
) -> Path:
    config = load_configuration(ROOT, ROOT / "runtime/ros2/configs/mock_collect.yaml")
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
    if feedback:
        topics["/robot/command_feedback"] = "robot_api_msgs/msg/CommandStatus"
    recorder = EpisodeRecorder(
        output,
        context.episode_id,
        {**config.snapshot, "run_context": asdict(context)},
        SCHEMAS,
        topics,
        run_id="test-recording",
        source="mock",
        test_only=True,
    )
    if on_recorder is not None:
        on_recorder(recorder)
    positions = list(config.profile.initial_positions)
    for sequence in (1, 2):
        positions[0] = (sequence - 1) * 0.2 if moving else positions[0]
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
    if not with_commands:
        return recorder.finish(runtime)
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
    event_trajectory = trajectory_message(trajectory)
    if event_frame_id:
        event_trajectory.header.frame_id = event_frame_id
    for sequence, phase in enumerate(phases, start=1):
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
    for message in feedback:
        recorder.append(
            "/robot/command_feedback",
            message,
            time_ns(message.sample_time),
        )
    return recorder.finish(runtime)


def test_checked_recording_commits_and_reopens(tmp_path):
    result = write_episode(tmp_path)
    assert result.name == "episode-test"
    report = inspect_episode(result, SCHEMAS)
    assert report["schema_version"] == 2
    assert report["recording"] == {"complete": True, "failure_reason": ""}
    assert report["runtime"] == {"outcome": "completed", "reason": ""}
    assert report["task"] == {"outcome": "not_evaluated", "reason": ""}
    assert report["observations"] == 2
    assert report["commands"] == 1
    assert report["commands_without_terminal"] == 0
    assert report["terminal_events"] == 1


def test_missing_reference_never_commits(tmp_path):
    result = write_episode(tmp_path, missing_frame=True)
    assert result.name.startswith(".partial-")
    manifest = json.loads((result / "manifest.json").read_text())
    assert manifest["recording"]["complete"] is False
    assert manifest["runtime"]["outcome"] == "failed"
    assert manifest["task"]["outcome"] == "not_evaluated"
    with pytest.raises(ContractError, match="RECORDING_ERROR"):
        inspect_episode(result, SCHEMAS)


def test_stopped_runtime_keeps_complete_recording_and_distinct_dimensions(tmp_path):
    result = write_episode(
        tmp_path,
        runtime=RuntimeResult(RuntimeOutcome.STOPPED, "operator stopped"),
    )
    assert result.name == "episode-test"
    manifest = json.loads((result / "manifest.json").read_text())
    assert manifest["recording"] == {"complete": True, "failure_reason": ""}
    assert manifest["runtime"] == {"outcome": "stopped", "reason": "operator stopped"}
    assert manifest["task"] == {"outcome": "not_evaluated", "reason": ""}
    report = inspect_episode(result, SCHEMAS)
    assert report["recording"]["complete"] is True
    assert report["runtime"]["outcome"] == "stopped"
    assert report["task"]["outcome"] == "not_evaluated"


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


def test_large_file_hash_is_chunked_and_matches_reference(tmp_path):
    payload = b"x" * (2 * 1024 * 1024 + 17)
    path = tmp_path / "large.bin"
    path.write_bytes(payload)
    assert sha256_file(path) == hashlib.sha256(payload).hexdigest()


def test_trajectory_frame_id_mismatch_never_commits(tmp_path):
    result = write_episode(tmp_path, event_frame_id="unexpected-trajectory-frame")
    assert result.name.startswith(".partial-")
    manifest = json.loads((result / "manifest.json").read_text())
    assert manifest["recording"]["complete"] is False
    assert manifest["runtime"]["outcome"] == "failed"
    assert "Accepted trajectory differs" in manifest["recording"]["failure_reason"]


def test_duplicate_finish_rejects_while_first_audit_is_blocked(tmp_path, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    original_audit = recording_module.audit_recording
    captured = []

    def held_audit(*args, **kwargs):
        entered.set()
        assert release.wait(2.0)
        return original_audit(*args, **kwargs)

    monkeypatch.setattr(recording_module, "audit_recording", held_audit)
    errors = []

    def run_first():
        try:
            write_episode(tmp_path, on_recorder=captured.append)
        except Exception as error:
            errors.append(error)

    first = threading.Thread(target=run_first)
    first.start()
    try:
        assert entered.wait(2.0)
        started = time.monotonic()
        with pytest.raises(RuntimeError, match="already been finalized"):
            captured[0].finish()
        assert time.monotonic() - started < 0.2
    finally:
        release.set()
        first.join(5.0)
    assert not first.is_alive()
    assert errors == []
