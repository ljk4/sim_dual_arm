"""Audit a v3 MCAP built from real three-chunk controller evidence.

No reference is copied into a measurement field: the state images and feedback
come from ``native_physical_run``'s actual ``NativeController`` stepping.  Real
EGL rendering is required for the MCAP test; without it the test skips instead of
substituting synthetic images.
"""

from __future__ import annotations

import sys
from dataclasses import asdict, replace
from pathlib import Path

import pytest

pytest.importorskip("numpy")
pytest.importorskip("scipy")
pytest.importorskip("mujoco")
rosbag2_py = pytest.importorskip("rosbag2_py")
rclpy = pytest.importorskip("rclpy")
pytest.importorskip("robot_api_msgs")

from rclpy.serialization import serialize_message  # noqa: E402
from robot_api_msgs.msg import (  # noqa: E402
    BackendInfo,
    CameraFrame,
    ObservationIndex,
    RobotState,
)
from sensor_msgs.msg import CameraInfo, Image, JointState  # noqa: E402
from std_msgs.msg import Header  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
for source in (
    ROOT / "components/backend_sim/src",
    ROOT / "contracts/python/src",
    ROOT / "libraries/ros_support/src",
):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

from native_physical_run import NATIVE_CONFIG, run_three_physical_chunks  # noqa: E402
from robot_ros_support.conversions import context_message, stamp  # noqa: E402
from robot_ros_support.native_conversions import (  # noqa: E402
    accepted_reference_message,
    native_command_message,
    native_control_event_message,
    native_execution_message,
)


def _state(run, capture) -> RobotState:
    header = Header(stamp=stamp(capture.sim_time_ns))
    return RobotState(
        context=context_message(run.context),
        state_seq=capture.sequence,
        stamp=stamp(capture.sim_time_ns),
        joints=JointState(
            header=header,
            name=list(run.profile.joint_names),
            position=list(capture.positions),
            velocity=list(capture.velocities),
        ),
        status="ACTIVE",
        fault_reason="",
    )


def _frame(run, camera, capture) -> CameraFrame:
    header = Header(stamp=stamp(capture.sim_time_ns), frame_id=camera.frame_id)
    data = capture.frames[camera.name]
    return CameraFrame(
        context=context_message(run.context),
        frame_seq=capture.sequence,
        image=Image(
            header=header,
            height=camera.height,
            width=camera.width,
            encoding="rgb8",
            is_bigendian=0,
            step=camera.width * 3,
            data=data,
        ),
        camera_info=CameraInfo(
            header=header,
            width=camera.width,
            height=camera.height,
            distortion_model="plumb_bob",
            d=[0.0] * 5,
            k=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            p=[1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        ),
        calibration_digest=camera.calibration_digest,
    )


def _write_recording(path: Path, run, *, raw_requests=None, capture_corruption=None) -> None:
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topics = {
        "/robot/info": "robot_api_msgs/msg/BackendInfo",
        "/robot/state": "robot_api_msgs/msg/RobotState",
        "/robot/native_events": "robot_api_msgs/msg/NativeControlEvent",
        "/robot/native_feedback": "robot_api_msgs/msg/NativeExecution",
        "/robot/accepted_references": "robot_api_msgs/msg/AcceptedReference",
        "/runtime/native/commands": "robot_api_msgs/msg/NativeCommand",
        "/runtime/observation_index": "robot_api_msgs/msg/ObservationIndex",
        **{
            f"/robot/cameras/{camera.name}/frame": "robot_api_msgs/msg/CameraFrame"
            for camera in run.profile.cameras
        },
    }
    for index, (name, type_name) in enumerate(topics.items(), start=1):
        writer.create_topic(
            rosbag2_py.TopicMetadata(
                id=index, name=name, type=type_name, serialization_format="cdr"
            )
        )

    def write(topic: str, message, timestamp: int) -> None:
        writer.write(topic, serialize_message(message), timestamp)

    info = BackendInfo(
        protocol_major=1,
        context=context_message(run.context),
        backend_kind="mujoco",
        ready=True,
        test_only=False,
        joint_names=list(run.profile.joint_names),
        joint_units=[joint.unit for joint in run.profile.joints],
        camera_names=list(run.profile.camera_names),
        capabilities=[
            "native_motion",
            "exact_capture",
            "simulation_reset",
            "lease",
            "stop",
            "command_query",
            "native_events",
            "accepted_reference",
            "continuous_chunks",
        ],
    )
    write("/robot/info", info, 0)
    for capture in run.captures:
        write("/robot/state", _state(run, capture), capture.sim_time_ns)
        if capture_corruption == "state_conflict":
            changed = _state(run, capture)
            changed.joints.position[8] += 0.001
            write("/robot/state", changed, capture.sim_time_ns)
        for camera in run.profile.cameras:
            write(
                f"/robot/cameras/{camera.name}/frame",
                _frame(run, camera, capture),
                capture.sim_time_ns,
            )
        if capture_corruption == "frame_conflict":
            camera = run.profile.cameras[0]
            changed = _frame(run, camera, capture)
            changed.image.data[0] ^= 1
            write(f"/robot/cameras/{camera.name}/frame", changed, capture.sim_time_ns)
        write(
            "/runtime/observation_index",
            ObservationIndex(
                context=context_message(run.context),
                observation_id=capture.sequence,
                state_seq=capture.sequence,
                camera_names=list(run.profile.camera_names),
                frame_seqs=[capture.sequence] * len(run.profile.cameras),
                reference_time=stamp(
                    capture.sim_time_ns + (1 if capture_corruption == "index_time" else 0)
                ),
            ),
            capture.sim_time_ns,
        )
    requests = raw_requests or tuple(record.request for record in run.references)
    for request in requests:
        write("/runtime/native/commands", native_command_message(request), 0)
    for record in run.references:
        write(
            "/robot/accepted_references",
            accepted_reference_message(record),
            record.accepted_sim_time_ns,
        )
    for event in run.events:
        write(
            "/robot/native_events",
            native_control_event_message(event),
            event.sim_time_ns,
        )
    for execution in run.feedback:
        write(
            "/robot/native_feedback",
            native_execution_message(execution),
            execution.measurement_sim_time_ns,
        )
    writer.close()


def test_three_physical_chunk_references_are_saved_retired_and_reconstructable(tmp_path):
    from dual_arm_runtime.audit import audit_recording
    from dual_arm_runtime.native_config import load_native_configuration

    run = run_three_physical_chunks()
    if any(capture.frames is None for capture in run.captures):
        pytest.skip(f"real EGL rendering unavailable: {run.render_error}")
    assert [len(record.request.trajectory.targets) for record in run.references] == [60, 60, 60]
    assert [len(record.selected_indices) for record in run.references] == [20, 20, 18]
    recording = tmp_path / "recording"
    _write_recording(recording, run)
    configuration = load_native_configuration(NATIVE_CONFIG, ROOT).snapshot
    configuration["run_context"] = asdict(run.context)
    report = audit_recording(recording, run.context.episode_id, configuration, version=3)
    assert report["accepted_references"] == 3
    assert report["commands"] == 1
    assert report["commands_without_terminal"] == 0
    # RETIRED is a per-chunk hand-off fact, never a second overall terminal.
    assert report["terminal_events"] == 1
    assert report["completed_commands"] == 1
    assert report["feedback_commands"] == 1
    assert report["reference_duplicates"] == 0
    assert report["max_joint_motion"] > 1e-4


@pytest.mark.parametrize("corruption", ["raw_reference_mismatch", "duplicate_chunk_conflict"])
def test_chunk_raw_intent_corruption_is_rejected(tmp_path, corruption):
    from dual_arm_runtime.audit import audit_recording
    from dual_arm_runtime.native_config import load_native_configuration
    from robot_contracts.models import ContractError

    run = run_three_physical_chunks()
    if any(capture.frames is None for capture in run.captures):
        pytest.skip(f"real EGL rendering unavailable: {run.render_error}")
    requests = [record.request for record in run.references]
    request = requests[1]
    targets = list(request.trajectory.targets)
    positions = list(targets[-1].positions)
    positions[8] += 0.001
    targets[-1] = replace(targets[-1], positions=tuple(positions))
    changed = replace(request, trajectory=replace(request.trajectory, targets=tuple(targets)))
    if corruption == "raw_reference_mismatch":
        requests[1] = changed
    else:
        requests.append(changed)
    recording = tmp_path / "recording"
    _write_recording(recording, run, raw_requests=requests)
    configuration = load_native_configuration(NATIVE_CONFIG, ROOT).snapshot
    configuration["run_context"] = asdict(run.context)
    with pytest.raises(ContractError, match="different content|raw chunk request"):
        audit_recording(recording, run.context.episode_id, configuration, version=3)


@pytest.mark.parametrize("corruption", ["state_conflict", "frame_conflict", "index_time"])
def test_exact_capture_payload_and_reference_time_corruption_is_rejected(tmp_path, corruption):
    from dual_arm_runtime.audit import audit_recording
    from dual_arm_runtime.native_config import load_native_configuration
    from robot_contracts.models import ContractError

    run = run_three_physical_chunks()
    if any(capture.frames is None for capture in run.captures):
        pytest.skip(f"real EGL rendering unavailable: {run.render_error}")
    recording = tmp_path / "recording"
    _write_recording(recording, run, capture_corruption=corruption)
    configuration = load_native_configuration(NATIVE_CONFIG, ROOT).snapshot
    configuration["run_context"] = asdict(run.context)
    with pytest.raises(ContractError, match="conflicting payloads|reference time differs"):
        audit_recording(recording, run.context.episode_id, configuration, version=3)
