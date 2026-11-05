"""Real-MCAP backpressure, immutability and streaming-audit checks."""

from __future__ import annotations

import json
import os
import random
import subprocess
import sys
import threading
import time
import tracemalloc
from dataclasses import asdict
from pathlib import Path

import pytest
import rosbag2_py
import yaml
from dual_arm_runtime import recording as recording_module
from dual_arm_runtime.audit import audit_recording, inspect_episode
from dual_arm_runtime.configuration import load_configuration
from dual_arm_runtime.recording import EpisodeRecorder
from dual_arm_runtime.recording_queue import RecordingSettings
from rclpy.serialization import deserialize_message
from robot_api_msgs.msg import CameraFrame, CommandEvent, ObservationIndex, RobotState
from robot_contracts.models import ContractError, RunContext, Target, Trajectory
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
CONFIG_SCHEMAS = ROOT / "contracts/schemas/v1"
CONFIG = ROOT / "runtime/ros2/configs/mock_collect.yaml"
PROFILE = ROOT / "profiles/robots/mock_dual_arm_v1/profile.json"
RUNTIME_SOURCE = ROOT / "runtime/ros2/ros/dual_arm_runtime"
DOUBLES_SOURCE = ROOT / "tests/doubles/ros/mock_robot_backend"
BOOTSTRAP = ROOT / "tests/integration/runtime_shutdown_bootstrap.py"
TOPICS = {
    "/robot/state": "robot_api_msgs/msg/RobotState",
    "/robot/cameras/front/frame": "robot_api_msgs/msg/CameraFrame",
    "/runtime/observation_index": "robot_api_msgs/msg/ObservationIndex",
    "/runtime/control/commands": "robot_api_msgs/msg/TrajectoryCommand",
    "/robot/command_events": "robot_api_msgs/msg/CommandEvent",
}


def _configuration():
    return load_configuration(ROOT, CONFIG)


def _context(configuration, episode_id="episode-backpressure"):
    return RunContext(
        configuration.profile.robot_id,
        configuration.profile.digest,
        "backend-test",
        "mock-system",
        0,
        episode_id,
        1,
    )


def _recorder(output, configuration, context, topics=None, settings=None):
    return EpisodeRecorder(
        output,
        context.episode_id,
        {**configuration.snapshot, "run_context": asdict(context)},
        SCHEMAS,
        topics or TOPICS,
        settings=settings,
        run_id="test-backpressure",
        source="mock",
        test_only=True,
    )


def _state(configuration, context, sequence, positions, timestamp):
    return RobotState(
        context=context_message(context),
        state_seq=sequence,
        stamp=stamp(timestamp),
        joints=JointState(
            name=list(configuration.profile.joint_names),
            position=positions,
        ),
        status="READY",
    )


def _frame(configuration, context, sequence, timestamp, data):
    camera = configuration.profile.cameras[0]
    return CameraFrame(
        context=context_message(context),
        frame_seq=sequence,
        image=Image(
            header=Header(stamp=stamp(timestamp)),
            width=camera.width,
            height=camera.height,
            encoding="rgb8",
            step=camera.width * 3,
            data=data,
        ),
        camera_info=CameraInfo(),
        calibration_digest="test",
    )


def _read_messages(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {topic.name: get_message(topic.type) for topic in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, payload, timestamp = reader.read_next()
        yield topic, deserialize_message(payload, types[topic]), timestamp


def test_recorder_oversize_rejection_latches_health_and_finish(tmp_path):
    configuration = _configuration()
    context = _context(configuration, "episode-reject-size")
    recorder = _recorder(
        tmp_path,
        configuration,
        context,
        {"/robot/state": "robot_api_msgs/msg/RobotState"},
        settings=RecordingSettings(max_pending_bytes=1),
    )
    state = _state(configuration, context, 1, list(configuration.profile.initial_positions), 1)
    with pytest.raises(ContractError, match="max_pending_bytes"):
        recorder.append("/robot/state", state, 1)
    with pytest.raises(ContractError, match="max_pending_bytes"):
        recorder.check_health()
    with pytest.raises(ContractError, match="max_pending_bytes"):
        recorder.finish()
    assert recorder.directory.name.startswith(".partial-")
    assert not (recorder.directory / "manifest.json").exists()


def test_recorder_serialization_failure_latches_health_and_finish(tmp_path):
    configuration = _configuration()
    context = _context(configuration, "episode-reject-serialize")
    recorder = _recorder(
        tmp_path,
        configuration,
        context,
        {"/robot/state": "robot_api_msgs/msg/RobotState"},
    )
    with pytest.raises(ContractError, match="serialize"):
        recorder.append("/robot/state", object(), 1)
    with pytest.raises(ContractError, match="serialize"):
        recorder.check_health()
    with pytest.raises(ContractError, match="serialize"):
        recorder.finish()
    assert recorder.directory.name.startswith(".partial-")
    assert not (recorder.directory / "manifest.json").exists()


def test_accepted_payload_is_immutable(tmp_path):
    configuration = _configuration()
    context = _context(configuration, "episode-immutable")
    recorder = _recorder(
        tmp_path,
        configuration,
        context,
        {"/robot/state": "robot_api_msgs/msg/RobotState"},
    )
    positions = list(configuration.profile.initial_positions)
    positions[0] = 0.1
    message = _state(configuration, context, 1, positions, 100_000_000)
    recorder.append("/robot/state", message, 100_000_000)
    message.joints.position[0] = 9.9
    message.state_seq = 99
    path = recorder.finish()

    recorded = next(
        value for topic, value, _ in _read_messages(path / "recording") if topic == "/robot/state"
    )
    assert recorded.state_seq == 1
    assert recorded.joints.position[0] == pytest.approx(0.1)
    assert recorded.stamp.sec * 1_000_000_000 + recorded.stamp.nanosec == 100_000_000


def test_append_returns_while_native_write_is_slow(tmp_path, monkeypatch):
    configuration = _configuration()
    context = _context(configuration, "episode-slow")
    entered, release = threading.Event(), threading.Event()
    original_write = recording_module._NativeWriter.write

    def slow_write(self, topic, payload, timestamp_ns):
        if not entered.is_set():
            entered.set()
            assert release.wait(5.0)
        return original_write(self, topic, payload, timestamp_ns)

    monkeypatch.setattr(recording_module._NativeWriter, "write", slow_write)
    recorder = _recorder(
        tmp_path,
        configuration,
        context,
        {"/robot/state": "robot_api_msgs/msg/RobotState"},
    )
    try:
        positions = list(configuration.profile.initial_positions)
        recorder.append(
            "/robot/state",
            _state(configuration, context, 1, positions, 1),
            1,
        )
        assert entered.wait(2.0)
        started = time.monotonic()
        for sequence in range(2, 22):
            recorder.append(
                "/robot/state",
                _state(configuration, context, sequence, positions, sequence),
                sequence,
            )
        elapsed = time.monotonic() - started
        assert elapsed < 1.0
        pending = recorder.stats()
        assert pending.accepted >= 21
        assert pending.written < pending.accepted
    finally:
        release.set()
    path = recorder.finish()
    manifest = json.loads((path / "manifest.json").read_text())
    assert path.name == "episode-slow"
    assert manifest["recording"] == {"complete": True, "failure_reason": ""}
    assert manifest["runtime"] == {"outcome": "completed", "reason": ""}
    assert manifest["task"] == {"outcome": "not_evaluated", "reason": ""}
    assert recorder.stats().pending_items == 0
    assert recorder.stats().pending_bytes == 0


def _write_large_episode(output, configuration, context, frames):
    # This fixture measures audit memory after close, not producer overload.
    # Allow its complete finite burst even when the writer gets no CPU meanwhile.
    recorder = _recorder(
        output,
        configuration,
        context,
        settings=RecordingSettings(queue_capacity=3 * frames + 4),
    )
    camera = configuration.profile.cameras[0]
    pixel_count = camera.width * camera.height * 3
    image_data = bytes([1, 2, 3]) * (camera.width * camera.height)
    initial = list(configuration.profile.initial_positions)
    for sequence in range(1, frames + 1):
        timestamp = sequence * 1_000_000
        positions = list(initial)
        positions[0] = 0.001 * (sequence % 100)
        recorder.append(
            "/robot/state",
            _state(configuration, context, sequence, positions, timestamp),
            timestamp,
        )
        recorder.append(
            "/robot/cameras/front/frame",
            _frame(configuration, context, sequence, timestamp, image_data),
            timestamp,
        )
        recorder.append(
            "/runtime/observation_index",
            ObservationIndex(
                context=context_message(context),
                observation_id=sequence,
                state_seq=sequence,
                camera_names=["front"],
                frame_seqs=[sequence],
                reference_time=stamp(timestamp),
            ),
            timestamp,
        )
    trajectory = Trajectory(
        context,
        "lease",
        "command-large",
        1,
        1,
        1_000_000_000,
        configuration.profile.joint_names,
        (Target(800_000_000, tuple(positions)),),
    )
    recorder.append("/runtime/control/commands", command_message(trajectory), 1_000_000)
    for sequence, phase in enumerate(("ACCEPTED", "STARTED", "COMPLETED"), start=1):
        recorder.append(
            "/robot/command_events",
            CommandEvent(
                context=context_message(context),
                event_seq=sequence,
                stamp=stamp(sequence * 1_000_000),
                command_id=trajectory.command_id,
                command_seq=1,
                phase=phase,
                trajectory=trajectory_message(trajectory),
            ),
            sequence * 1_000_000,
        )
    return recorder.finish(), pixel_count * frames


def test_streaming_audit_memory_does_not_scale_with_pixel_bytes(tmp_path):
    configuration = _configuration()
    context = _context(configuration, "episode-memory")
    path, pixel_bytes = _write_large_episode(tmp_path, configuration, context, frames=400)
    assert pixel_bytes > 3 * 1024 * 1024
    assert inspect_episode(path, SCHEMAS)["frames"] == 400

    tracemalloc.start()
    try:
        report = audit_recording(
            path / "recording",
            context.episode_id,
            {**configuration.snapshot, "run_context": asdict(context)},
        )
        _, peak = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()
    assert report["frames"] == 400
    # The old implementation retained every CameraFrame/Image; this bound keeps
    # reference metadata only, far below the multi-megabyte pixel payload.
    assert peak < 2 * 1024 * 1024, peak


# ---- Real runtime subprocesses: slow writer and asynchronous failure ----------------


def _environment(domain_id):
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = str(domain_id)
    environment["PYTHONUNBUFFERED"] = "1"
    paths = [str(RUNTIME_SOURCE), str(DOUBLES_SOURCE)]
    if environment.get("PYTHONPATH"):
        paths.append(environment["PYTHONPATH"])
    environment["PYTHONPATH"] = os.pathsep.join(paths)
    return environment


class _RuntimeHarness:
    def __init__(self, tmp_path):
        self.tmp = tmp_path
        self.output = tmp_path / "raw"
        self.environment = _environment(random.SystemRandom().randint(1, 200))
        self.backend = None
        self.runtime = None
        self.runtime_log = None
        self._handles = []
        self._processes = []

    def _spawn(self, arguments, name):
        path = self.tmp / name
        handle = path.open("w")
        self._handles.append(handle)
        process = subprocess.Popen(
            arguments,
            cwd=ROOT,
            env=self.environment,
            stdout=handle,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        self._processes.append(process)
        return process, path

    def start_backend(self):
        self.backend, _ = self._spawn(
            [
                sys.executable,
                "-m",
                "mock_robot_backend.node",
                "--profile",
                str(PROFILE),
                "--schemas",
                str(CONFIG_SCHEMAS),
            ],
            "backend.log",
        )

    def start_runtime(self, *, config=CONFIG, extra_environment=None):
        if extra_environment:
            self.environment.update(extra_environment)
        self.runtime, self.runtime_log = self._spawn(
            [
                sys.executable,
                str(BOOTSTRAP),
                "--project-root",
                str(ROOT),
                "--config",
                str(config),
                "--output",
                str(self.output),
                "--shutdown-timeout",
                "15",
                "--once",
            ],
            "runtime.log",
        )

    def wait_runtime(self, timeout=60):
        return self.runtime.wait(timeout=timeout)

    def log(self):
        return self.runtime_log.read_text() if self.runtime_log.exists() else ""

    def stop(self):
        for process in reversed(self._processes):
            if process.poll() is None:
                process.kill()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        for handle in self._handles:
            handle.close()


def _episodes(output):
    return sorted(output.glob("episode-*")), sorted(output.glob(".partial-*"))


def test_slow_disk_keeps_control_and_renewal_running(tmp_path):
    harness = _RuntimeHarness(tmp_path)
    marker = tmp_path / "commands"
    try:
        harness.start_backend()
        harness.start_runtime(
            extra_environment={
                "DSH009_TEST_SLOW_WRITE_TOPIC": "/runtime/control/commands",
                "DSH009_TEST_SLOW_WRITE_HOLD_S": "2.5",
                "DSH008_TEST_COMMAND_MARKER": str(marker),
            }
        )
        assert harness.wait_runtime(timeout=60) == 0, harness.log()
        committed, partial = _episodes(harness.output)
        assert partial == []
        assert len(committed) == 1
        commands = [line for line in marker.read_text().splitlines() if line]
        assert len(commands) == 3
        assert inspect_episode(committed[0], SCHEMAS)["recording"]["complete"] is True
        assert inspect_episode(committed[0], SCHEMAS)["task"]["outcome"] == "not_evaluated"
    finally:
        harness.stop()


def test_async_writer_failure_stops_next_command_and_still_cleans_up(tmp_path):
    harness = _RuntimeHarness(tmp_path)
    command_marker = tmp_path / "commands"
    cleanup_marker = tmp_path / "cleanup"
    try:
        harness.start_backend()
        harness.start_runtime(
            extra_environment={
                "DSH009_TEST_FAIL_WRITE_TOPIC": "/runtime/control/commands",
                "DSH008_TEST_COMMAND_MARKER": str(command_marker),
                "DSH003_TEST_CLEANUP_MARKER": str(cleanup_marker),
            }
        )
        assert harness.wait_runtime(timeout=60) == 1, harness.log()
        assert "injected writer failure" in harness.log()
        assert len(command_marker.read_text().splitlines()) == 1
        assert cleanup_marker.read_text() == "cleanup-ran\n"
        committed, partial = _episodes(harness.output)
        assert committed == []
        assert len(partial) == 1
        assert (partial[0] / "recording").is_dir()
        assert not (partial[0] / "manifest.json").exists()
    finally:
        harness.stop()


def test_close_timeout_keeps_partial_without_success_manifest(tmp_path):
    harness = _RuntimeHarness(tmp_path)
    document = yaml.safe_load(CONFIG.read_text())
    document["recording"]["close_timeout_s"] = 0.2
    config = tmp_path / "runtime-fast-close.yaml"
    config.write_text(yaml.safe_dump(document))
    try:
        harness.start_backend()
        harness.start_runtime(
            config=config,
            extra_environment={"DSH009_TEST_CLOSE_HOLD_S": "10"},
        )
        assert harness.wait_runtime(timeout=30) == 1, harness.log()
        assert "writer close failed" in harness.log()
        assert "timed out" in harness.log()
        committed, partial = _episodes(harness.output)
        assert committed == []
        assert len(partial) == 1
        assert (partial[0] / "recording").is_dir()
        assert not (partial[0] / "manifest.json").exists()
    finally:
        harness.stop()
