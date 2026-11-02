"""Real ROS serialization/deserialization evidence for the native transport."""

from __future__ import annotations

import hashlib
from pathlib import Path

import pytest

try:
    from rclpy.serialization import deserialize_message, serialize_message
    from robot_api_msgs.msg import (
        AcceptedReference as AcceptedReferenceMessage,
    )
    from robot_api_msgs.msg import NativeChunkStatus as NativeChunkStatusMessage
    from robot_api_msgs.msg import NativeCommand as NativeCommandMessage
    from robot_api_msgs.msg import NativeControlEvent as NativeControlEventMessage
    from robot_api_msgs.msg import NativeExecution as NativeExecutionMessage
    from robot_api_msgs.srv import (
        GetNativeAnchor,
        QueryNativeChunk,
        QueryNativeCommand,
        ResetSimulation,
        SubmitNative,
    )
    from robot_contracts.action_chunks import ChunkAnchor
    from robot_contracts.models import (
        ContractError,
        RunContext,
        Target,
        Trajectory,
    )
    from robot_contracts.native import (
        NATIVE_MOTION,
        NATIVE_STREAM_FIRST,
        AcceptedReference,
        NativeChunkStatus,
        NativeControlEvent,
        NativeExecution,
        NativeFinger,
        NativeGripper,
        NativeRequest,
    )
    from robot_contracts.reference import (
        CubicSegment,
        ReferencePlan,
        ReferenceSample,
    )
    from robot_ros_support.native_conversions import (
        accepted_reference_from_message,
        accepted_reference_message,
        native_chunk_status_from_message,
        native_chunk_status_message,
        native_command_from_message,
        native_command_message,
        native_control_event_from_message,
        native_control_event_message,
        native_execution_from_message,
        native_execution_message,
    )
except ImportError as error:  # pragma: no cover - host without ROS
    pytest.skip(f"ROS unavailable: {error}", allow_module_level=True)

ROOT = Path(__file__).resolve().parents[2]
PROFILE_DIGEST = "profile-digest"
CTX = RunContext(
    "panda-stack",
    PROFILE_DIGEST,
    "backend-instance",
    "mujoco-sim",
    0,
    episode_id="episode-1",
    control_epoch=1,
)

OLD_IDL_DIGESTS = {
    "BackendInfo.msg": "359cba42f2cd9189fd2418c0c317a8749aef91d4809c611e89fb035a633993a2",
    "CameraFrame.msg": "b9c6c46a196bdd2def564cf0c2083f43b52d9ee834c26381a29cfbf910ee2735",
    "CommandEvent.msg": "eac72bfa2b2403d6e137227a6145fd6866dcb8b4f54933fc9b8fc3d77d1a62d7",
    "CommandStatus.msg": "525eee3dcaa4265467a68b9c53c46cd20f308cc3f12146a5dcf1e56f02dc1a29",
    "ObservationIndex.msg": "80122678b8df5ec086835049823a8cc790a2bb5c93dcf2db0167c1acbc10ce8f",
    "RobotState.msg": "4fa3ca997f859efdbf3f9c7f5832e00c49055ffd811fc37eb6a99546f146364b",
    "RunContext.msg": "2b3ac8b3e9d62e21cc872720cf1f1adb68af442b92930d36a4c6d50796e55f28",
    "RuntimeStatus.msg": "076a1c6380744d2d34906a14d16e773442c047bf44252082429a45bf025b2904",
    "TrajectoryCommand.msg": "a3425c62f4f5df2d95cd39ccf4684cb5f5266115e1b38b128a0c8f6bf32f1341",
    "AcquireControl.srv": "697b3b383c86c9058583e00c4fbabe2ac8d01812081138f0c212b3b5aedec96d",
    "EndEpisode.srv": "e0231330eeb58c968f10eed0dc1f6428c2e3ff6d31922212f0887e7b17fe20e7",
    "QueryCommand.srv": "9ec4a0cbfc915253ccb7f084f03c99e12892ea3af172741674530c3d015afb35",
    "ReleaseControl.srv": "c5cbf93e9e844b66da22cbd84745bef90ec3ddfd6b06b1ee6df5ee92a5a9b1c3",
    "RenewControl.srv": "788f6babb1c9f93c051bdaaa57ae47fe594fae64a20cbcb7769ef3608abee991",
    "StartEpisode.srv": "2022c786b2431c26db5df35af855e47676c03b9e60f7cd72e004aae201dd4b10",
    "Stop.srv": "71590fec3834816dc72fe9c97b09977501b87d60c42050f43f930965d5e08c4d",
    "ExecuteTrajectory.action": "a7b09037efe628dedb2a94bc10c305f5119095d451e93b0e62f42fe79ba50b67",
}


def motion_request() -> NativeRequest:
    return NativeRequest(
        trajectory=Trajectory(
            CTX,
            "lease-1",
            "command-1",
            1,
            1,
            5_000_000_000,
            ("j1",),
            (Target(1_000_000_000, (1.0,)),),
        ),
        kind=NATIVE_MOTION,
    )


def stream_request() -> NativeRequest:
    return NativeRequest(
        trajectory=Trajectory(
            CTX,
            "lease-1",
            "command-1",
            1,
            2,
            5_000_000_000,
            ("j1",),
            (
                Target(200_000_000, (0.1,)),
                Target(600_000_000, (0.2,)),
                Target(800_000_000, (0.3,)),
            ),
        ),
        kind=NATIVE_STREAM_FIRST,
        anchor=ChunkAnchor(
            "stream-1",
            0,
            1_000_000_000,
            ReferenceSample((0.0,), (0.0,), (0.0,)),
            PROFILE_DIGEST,
        ),
        observation_time_ns=500_000_000,
        target_count=2,
        settle_at_end=False,
        final_velocities=(0.5,),
    )


def accepted_record() -> AcceptedReference:
    request = motion_request()
    return AcceptedReference(
        request=request,
        motion_seq=1,
        accepted_sim_time_ns=100_000_000,
        joint_names=("j1",),
        plan=ReferencePlan(
            ("j1",),
            (CubicSegment(1_000_000_000, (0.0,), (0.0,), (3.0,), (-2.0,)),),
        ),
        initial_positions=(0.0,),
        initial_velocities=(0.0,),
        final_velocities=(0.0,),
        max_accelerations=(1.0,),
        selected_indices=(),
    )


def finger(side: str, index: int) -> NativeFinger:
    return NativeFinger(
        side,
        index,
        f"{side}_finger{index}",
        0.0,
        ("source",),
        (1.25,),
        (2,),
    )


def gripper(side: str, mode: str = "NONE") -> NativeGripper:
    contact = mode == "CONTACT"
    return NativeGripper(
        side=side,
        mode=mode,
        position_ok=mode == "POSITION",
        contact_ok=contact,
        contact_object="source" if contact else None,
        ambiguous=False,
        target_position_m=0.02,
        initial_reference_m=0.04,
        final_reference_m=0.0,
        measured_position_m=0.02 if contact else 0.04,
        measured_velocity_m_s=0.0,
        target_error_m=0.0 if contact else -0.02,
        fingers=(finger(side, 1), finger(side, 2)),
    )


def execution() -> NativeExecution:
    return NativeExecution(
        context=CTX,
        command_id="command-1",
        command_seq=1,
        observation_id=1,
        phase="FAILED",
        reason="STOP_TIMEOUT",
        stop_reason="STOP_REQUESTED",
        motion_seq=1,
        stream_id="stream-1",
        chunk_seq=2,
        accepted_sim_time_ns=0,
        accepted_wall_time_ns=10,
        has_started=True,
        started_sim_time_ns=20,
        started_wall_time_ns=20,
        has_finished=True,
        finished_sim_time_ns=100,
        finished_wall_time_ns=110,
        motion_phase="FAILED",
        reference_application_sim_time_ns=90,
        measurement_sim_time_ns=92,
        reference_elapsed_ns=70,
        stable_for_ns=5,
        joint_names=("j1",),
        reference_positions=(0.3,),
        reference_velocities=(0.2,),
        reference_accelerations=(0.0,),
        measured_positions=(0.25,),
        measured_velocities=(0.15,),
        position_errors=(0.05,),
        velocity_errors=(0.05,),
        reference_kind="BRAKE",
        tracking_quality="OUTSIDE_TOLERANCE",
        tracking_violation_ns=15,
        grippers=(gripper("left"), gripper("right", "CONTACT")),
    )


def chunk_status() -> NativeChunkStatus:
    return NativeChunkStatus(
        context=CTX,
        command_id="command-1",
        command_seq=1,
        chunk_seq=2,
        observation_id=2,
        phase="RETIRED",
        reason="HANDOFF",
        accepted_sim_time_ns=0,
        accepted_wall_time_ns=10,
        has_started=True,
        started_sim_time_ns=20,
        started_wall_time_ns=20,
        has_finished=True,
        finished_sim_time_ns=100,
        finished_wall_time_ns=100,
    )


def event() -> NativeControlEvent:
    return NativeControlEvent(
        context=CTX,
        event_seq=4,
        command_id="command-1",
        command_seq=1,
        observation_id=2,
        is_chunk=True,
        chunk_seq=2,
        phase="STARTED",
        reason="HANDOFF",
        sim_time_ns=1_100_000_000,
    )


def roundtrip(value, message_class, to_message, from_message):
    wire = serialize_message(to_message(value))
    decoded = deserialize_message(wire, message_class)
    restored = from_message(decoded)
    assert restored == value
    return decoded, wire


def test_native_messages_really_serialize_and_roundtrip():
    command, wire = roundtrip(
        motion_request(),
        NativeCommandMessage,
        native_command_message,
        native_command_from_message,
    )
    assert command.kind == NATIVE_MOTION
    assert list(command.command.trajectory.joint_names) == ["j1"]
    assert bytes(wire)

    stream, _ = roundtrip(
        stream_request(),
        NativeCommandMessage,
        native_command_message,
        native_command_from_message,
    )
    assert stream.anchor.predecessor_seq == 0
    assert stream.kind == NATIVE_STREAM_FIRST

    record, _ = roundtrip(
        accepted_record(),
        AcceptedReferenceMessage,
        accepted_reference_message,
        accepted_reference_from_message,
    )
    assert list(record.segments[0].c2) == [3.0]

    feedback, _ = roundtrip(
        execution(),
        NativeExecutionMessage,
        native_execution_message,
        native_execution_from_message,
    )
    assert feedback.has_finished and feedback.finished_wall_time_ns == 110
    assert feedback.grippers[1].mode == "CONTACT"
    assert list(feedback.grippers[1].fingers[0].object_names) == ["source"]

    chunk, _ = roundtrip(
        chunk_status(),
        NativeChunkStatusMessage,
        native_chunk_status_message,
        native_chunk_status_from_message,
    )
    assert chunk.phase == "RETIRED"

    control_event, _ = roundtrip(
        event(),
        NativeControlEventMessage,
        native_control_event_message,
        native_control_event_from_message,
    )
    assert control_event.is_chunk and control_event.event_seq == 4


def test_new_services_deserialize_real_requests_and_responses():
    submit = SubmitNative.Request(request=native_command_message(motion_request()))
    assert deserialize_message(serialize_message(submit), SubmitNative.Request).request.kind == 1
    submit_response = SubmitNative.Response(
        accepted=True,
        reason="ACCEPTED",
        has_execution=True,
        execution=native_execution_message(execution()),
        has_chunk=True,
        chunk=native_chunk_status_message(chunk_status()),
        has_reference=True,
        reference=accepted_reference_message(accepted_record()),
    )
    decoded = deserialize_message(serialize_message(submit_response), SubmitNative.Response)
    assert decoded.has_execution and decoded.has_chunk and decoded.has_reference

    anchor_request = GetNativeAnchor.Request(
        context=submit.request.command.context,
        lease_id="lease-1",
        has_start=True,
        start_sim_time_ns=1_100_000_000,
    )
    assert (
        deserialize_message(serialize_message(anchor_request), GetNativeAnchor.Request)
        .start_sim_time_ns
        == 1_100_000_000
    )
    assert deserialize_message(
        serialize_message(QueryNativeCommand.Request(context=submit.request.command.context,
                                                    command_id="command-1")),
        QueryNativeCommand.Request,
    ).command_id == "command-1"
    assert deserialize_message(
        serialize_message(QueryNativeChunk.Request(
            context=submit.request.command.context, command_id="command-1", chunk_seq=2
        )),
        QueryNativeChunk.Request,
    ).chunk_seq == 2
    assert deserialize_message(
        serialize_message(ResetSimulation.Request(
            context=submit.request.command.context, lease_id="lease-1", seed=7
        )),
        ResetSimulation.Request,
    ).seed == 7


def test_error_payloads_are_rejected_not_silently_dropped():
    message = native_command_message(motion_request())
    message.command.trajectory.points[0].velocities = [0.1]
    with pytest.raises(ContractError) as caught:
        native_command_from_message(message)
    assert caught.value.reason == "UNSUPPORTED_CAPABILITY"

    message = native_command_message(motion_request())
    message.command.trajectory.header.frame_id = "world"
    with pytest.raises(ContractError) as caught:
        native_command_from_message(message)
    assert caught.value.reason == "UNSUPPORTED_CAPABILITY"

    message = native_command_message(stream_request())
    message.has_final_velocities = False
    with pytest.raises(ContractError) as caught:
        native_command_from_message(message)
    assert caught.value.reason == "INVALID_NATIVE_REQUEST"

    message = native_command_message(motion_request())
    message.kind = 9
    with pytest.raises(ContractError) as caught:
        native_command_from_message(message)
    assert caught.value.reason == "INVALID_NATIVE_REQUEST"

    message = accepted_reference_message(accepted_record())
    message.reference_digest = "0" * 64
    with pytest.raises(ContractError) as caught:
        accepted_reference_from_message(message)
    assert caught.value.reason == "REFERENCE_DIGEST_MISMATCH"

    message = native_execution_message(execution())
    message.has_started = False
    with pytest.raises(ContractError) as caught:
        native_execution_from_message(message)
    assert caught.value.reason == "INVALID_NATIVE_EXECUTION"

    message = native_control_event_message(event())
    message.is_chunk = False
    with pytest.raises(ContractError) as caught:
        native_control_event_from_message(message)
    assert caught.value.reason == "INVALID_NATIVE_EVENT"


def test_old_idl_source_bytes_are_unchanged():
    root = ROOT / "contracts/ros/robot_api_msgs"
    for name, expected in OLD_IDL_DIGESTS.items():
        if name.endswith(".action"):
            path = root / "action" / name
        elif name.endswith(".srv"):
            path = root / "srv" / name
        else:
            path = root / "msg" / name
        assert hashlib.sha256(path.read_bytes()).hexdigest() == expected, name
