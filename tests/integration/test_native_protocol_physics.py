"""Native request -> compile -> record -> ROS -> independent rebuild evidence."""

from __future__ import annotations

import os
import sys
from pathlib import Path

import pytest

mujoco = pytest.importorskip("mujoco")
pytest.importorskip("scipy")
np = pytest.importorskip("numpy")

ROOT = Path(__file__).resolve().parents[2]
for source in (ROOT / "components/backend_sim/src", ROOT / "contracts/python/src"):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

from robot_backend_sim.control_state import ControlStatus  # noqa: E402
from robot_backend_sim.motion import NativeMotion, Phase  # noqa: E402
from robot_backend_sim.motion_config import load_control_config  # noqa: E402
from robot_backend_sim.panda_io import PandaIO  # noqa: E402
from robot_backend_sim.protocol import (  # noqa: E402
    accepted_reference,
    execution_feedback,
    prepare_request,
)
from robot_backend_sim.scene import build_scene  # noqa: E402
from robot_backend_sim.scene_config import load_scene_config  # noqa: E402
from robot_backend_sim.stream_config import StreamConfig  # noqa: E402
from robot_contracts.models import RunContext, Target, Trajectory  # noqa: E402
from robot_contracts.native import (  # noqa: E402
    NATIVE_MOTION,
    NATIVE_STREAM_FIRST,
    NATIVE_STREAM_NEXT,
    NativeRequest,
    sample_accepted_reference,
    validate_accepted_reference,
)
from robot_contracts.profiles import load_profile  # noqa: E402
from test_native_motion import install_contact_source  # noqa: E402
from test_native_stream import original_plan, window_for  # noqa: E402

CONFIG_PATH = ROOT / "configs/simulation/panda_stack.yaml"
CONTROL_PATH = ROOT / "configs/simulation/panda_control.yaml"
PROFILE_PATH = ROOT / "profiles/robots/panda_stack_v1/profile.json"
SCHEMAS = ROOT / "contracts/schemas"
ASSETS_ROOT = (
    Path(os.environ["SIM_ASSETS_ROOT"])
    if "SIM_ASSETS_ROOT" in os.environ
    else (ROOT / "third_party/src/mujoco_menagerie")
)
SUMMARY_PATH = Path(os.environ.get("SIM_ASSET_SUMMARY", ROOT / ".build/assets/panda.json"))
SOURCE_PATH = Path(os.environ.get("SIM_ASSET_SOURCE", ROOT / "third_party/panda-source.json"))
DT_NS = 2_000_000
START_NS = 100_000_000
TERMINAL = frozenset({Phase.COMPLETED, Phase.STOPPED, Phase.FAILED})

try:
    from rclpy.serialization import deserialize_message, serialize_message
    from robot_api_msgs.msg import AcceptedReference as AcceptedReferenceMessage
    from robot_api_msgs.msg import NativeExecution as NativeExecutionMessage
    from robot_ros_support.native_conversions import (
        accepted_reference_from_message,
        accepted_reference_message,
        native_execution_from_message,
        native_execution_message,
    )
except ImportError:  # pragma: no cover - native host may not have ROS
    serialize_message = None


@pytest.fixture(scope="module")
def scene_config():
    return load_scene_config(CONFIG_PATH)


@pytest.fixture(scope="module")
def scene(scene_config):
    return build_scene(
        scene_config,
        assets_root=ASSETS_ROOT,
        summary_path=SUMMARY_PATH,
        source_path=SOURCE_PATH,
    )


@pytest.fixture(scope="module")
def profile():
    return load_profile(PROFILE_PATH, SCHEMAS)


@pytest.fixture(scope="module")
def control_config():
    return load_control_config(CONTROL_PATH)


@pytest.fixture(scope="module")
def stream_config():
    return StreamConfig(0.25, 0.05, 0.5, 120.0, 3.0, 2.0)


def fresh_io(scene, profile) -> PandaIO:
    scene.reset(0)
    return PandaIO(scene, profile)


def context(profile) -> RunContext:
    return RunContext(
        "panda-stack",
        profile.digest,
        "native-protocol-test",
        "mujoco-sim",
        0,
        episode_id="episode-1",
        control_epoch=1,
    )


def request_for(
    profile,
    observation_id: int,
    targets: tuple[Target, ...],
    *,
    kind: int,
    anchor=None,
    observation_time_ns: int = 0,
    target_count: int = 0,
    settle_at_end: bool = True,
    final_velocities=None,
) -> NativeRequest:
    return NativeRequest(
        trajectory=Trajectory(
            context(profile),
            "lease-1",
            "native-command-1",
            1,
            observation_id,
            5_000_000_000,
            tuple(profile.joint_names),
            targets,
        ),
        kind=kind,
        anchor=anchor,
        observation_time_ns=observation_time_ns,
        target_count=target_count,
        settle_at_end=settle_at_end,
        final_velocities=final_velocities,
    )


def status_for(snapshot, profile, *, accepted_sim: int = 0) -> ControlStatus:
    terminal = snapshot.phase in TERMINAL
    if snapshot.phase is Phase.COMPLETED:
        phase = "COMPLETED"
    elif snapshot.phase is Phase.FAILED:
        phase = "FAILED"
    elif snapshot.phase is Phase.STOPPED:
        phase = "CANCELED"
    elif snapshot.phase is Phase.ACCEPTED:
        phase = "ACCEPTED"
    else:
        phase = "STARTED"
    return ControlStatus(
        context=snapshot_context(profile),
        command_id="native-command-1",
        command_seq=1,
        observation_id=1,
        phase=phase,
        reason=snapshot.reason if terminal else "",
        stop_reason=snapshot.reason if terminal and snapshot.reason else None,
        motion_seq=snapshot.motion_seq,
        stream_id=snapshot.stream_id,
        chunk_seq=snapshot.chunk_seq,
        accepted_sim_ns=accepted_sim,
        started_sim_ns=snapshot.started_sim_ns,
        finished_sim_ns=snapshot.measurement_sim_time_ns if terminal else None,
        accepted_wall_ns=0,
        started_wall_ns=snapshot.started_wall_ns,
        finished_wall_ns=snapshot.measurement_sim_time_ns if terminal else None,
        snapshot=snapshot,
    )


def snapshot_context(profile) -> RunContext:
    # NativeMotion has no context; the accepted request identity is shared.
    return RunContext(
        "panda-stack",
        profile.digest,
        "native-protocol-test",
        "mujoco-sim",
        0,
        episode_id="episode-1",
        control_epoch=1,
    )


def ros_roundtrip_record(record):
    if serialize_message is None:
        return
    wire = serialize_message(accepted_reference_message(record))
    decoded = deserialize_message(wire, AcceptedReferenceMessage)
    assert accepted_reference_from_message(decoded) == record


def ros_roundtrip_execution(feedback):
    if serialize_message is None:
        return
    wire = serialize_message(native_execution_message(feedback))
    decoded = deserialize_message(wire, NativeExecutionMessage)
    assert native_execution_from_message(decoded) == feedback


def compare_record(record, snapshot) -> None:
    rebuilt = sample_accepted_reference(
        record,
        snapshot.reference_application_sim_time_ns,
        actual_started_sim_ns=(
            snapshot.started_sim_ns if record.request.kind == NATIVE_MOTION else None
        ),
    )
    np.testing.assert_allclose(rebuilt.positions, snapshot.reference_positions, atol=1e-12)
    np.testing.assert_allclose(rebuilt.velocities, snapshot.reference_velocities, atol=1e-12)
    np.testing.assert_allclose(rebuilt.accelerations, snapshot.reference_accelerations, atol=1e-12)


def test_motion_request_compile_record_feedback_and_ros(scene, profile, control_config):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    targets = list(initial)
    targets[9] += 0.05
    # 0.8 s keeps the same 1.0 rad/s^2 acceleration limit: the smooth-step
    # peak is 6 * 0.05 / 0.8^2 = 0.46875 rad/s^2.
    request = request_for(profile, 1, (Target(800_000_000, tuple(targets)),), kind=NATIVE_MOTION)
    packet = prepare_request(request, profile, control_config, initial_reference=initial)
    motion = NativeMotion(scene, io, control_config, initial_reference=initial)
    accepted = motion.start(packet.prepared, 0)
    assert accepted.reference_kind == "HOLD"
    assert accepted.started_sim_ns is None
    accepted_feedback = execution_feedback(status_for(accepted, profile))
    assert accepted_feedback.phase == "ACCEPTED"
    assert accepted_feedback.reference_kind == "HOLD"
    assert not accepted_feedback.has_started
    assert accepted_feedback.motion_phase == "ACCEPTED"
    ros_roundtrip_execution(accepted_feedback)

    record = accepted_reference(request, packet, accepted)
    validate_accepted_reference(record, profile, control_config.max_accelerations)
    ros_roundtrip_record(record)

    first = motion.step(0)
    assert first.reference_kind == "TRAJECTORY"
    assert first.started_sim_ns == 0
    started_feedback = execution_feedback(status_for(first, profile))
    assert started_feedback.phase == "STARTED"
    assert started_feedback.has_started and started_feedback.started_sim_time_ns == 0
    assert started_feedback.reference_kind == "TRAJECTORY"
    ros_roundtrip_execution(started_feedback)

    snapshot = first
    settling = None
    for index in range(1, 1600):
        snapshot = motion.step(index * DT_NS)
        if snapshot.phase is Phase.SETTLING:
            settling = snapshot
        if snapshot.reference_kind in ("TRAJECTORY", "HOLD"):
            compare_record(record, snapshot)
        if snapshot.phase in TERMINAL:
            break
    assert settling is not None
    assert settling.reference_kind == "HOLD"
    assert (
        settling.reference_application_sim_time_ns >= first.started_sim_ns + record.plan.duration_ns
    )
    held = sample_accepted_reference(
        record,
        settling.reference_application_sim_time_ns,
        actual_started_sim_ns=first.started_sim_ns,
    )
    np.testing.assert_allclose(held.positions, settling.reference_positions, atol=1e-12)
    np.testing.assert_allclose(held.velocities, (0.0,) * len(held.velocities), atol=1e-12)
    np.testing.assert_allclose(held.accelerations, (0.0,) * len(held.accelerations), atol=1e-12)
    np.testing.assert_allclose(settling.reference_velocities, 0.0, atol=1e-12)
    np.testing.assert_allclose(settling.reference_accelerations, 0.0, atol=1e-12)
    assert snapshot.phase is Phase.COMPLETED
    feedback = execution_feedback(status_for(snapshot, profile))
    assert feedback.phase == "COMPLETED"
    assert feedback.has_finished and feedback.motion_phase == "COMPLETED"
    np.testing.assert_allclose(feedback.measured_positions, snapshot.measured_positions)
    np.testing.assert_allclose(feedback.position_errors, snapshot.tracking_errors)
    ros_roundtrip_execution(feedback)


def test_three_chunk_requests_match_real_stream_reference(
    scene, profile, control_config, stream_config
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    plan = original_plan(profile, control_config, initial)
    motion = NativeMotion(
        scene, io, control_config, initial_reference=initial, stream_config=stream_config
    )

    first_anchor = motion.chunk_anchor(START_NS)
    first_window = window_for(
        plan,
        stream_start_ns=START_NS,
        observation_ns=0,
        handoff_ns=START_NS,
        observation_id=1,
    )
    first_request = request_for(
        profile,
        1,
        tuple(first_window.source_targets),
        kind=NATIVE_STREAM_FIRST,
        anchor=first_anchor,
        observation_time_ns=0,
        target_count=first_window.target_count,
        settle_at_end=False,
        final_velocities=plan.sample_interval(1_000_000_000).velocities,
    )
    first_packet = prepare_request(first_request, profile, control_config)
    accepted = motion.start_stream(first_packet.prepared, 0)
    records = {
        1: accepted_reference(first_request, first_packet, accepted),
    }

    records_ros = {1: records[1]}
    wall = 0
    terminal = None
    snapshots = []
    for _index in range(4000):
        application_ns = round(float(scene.data.time) * 1e9)
        if application_ns == 400_000_000:
            anchor = motion.chunk_anchor()
            window = window_for(
                plan,
                stream_start_ns=START_NS,
                observation_ns=application_ns,
                handoff_ns=1_100_000_000,
                observation_id=2,
            )
            request = request_for(
                profile,
                2,
                tuple(window.source_targets),
                kind=NATIVE_STREAM_NEXT,
                anchor=anchor,
                observation_time_ns=application_ns,
                target_count=window.target_count,
                settle_at_end=False,
                final_velocities=plan.sample_interval(2_000_000_000).velocities,
            )
            packet = prepare_request(request, profile, control_config)
            submission = motion.queue_chunk(packet.prepared, application_ns)
            assert submission.accepted, submission.reason
            records[2] = accepted_reference(request, packet, submission.snapshot)
            records_ros[2] = records[2]
        if application_ns == 1_400_000_000:
            anchor = motion.chunk_anchor()
            window = window_for(
                plan,
                stream_start_ns=START_NS,
                observation_ns=application_ns,
                handoff_ns=2_100_000_000,
                observation_id=3,
            )
            request = request_for(
                profile,
                3,
                tuple(window.source_targets),
                kind=NATIVE_STREAM_NEXT,
                anchor=anchor,
                observation_time_ns=application_ns,
                target_count=window.target_count,
                settle_at_end=True,
                final_velocities=None,
            )
            packet = prepare_request(request, profile, control_config)
            submission = motion.queue_chunk(packet.prepared, application_ns)
            assert submission.accepted, submission.reason
            records[3] = accepted_reference(request, packet, submission.snapshot)
            records_ros[3] = records[3]

        snapshot = motion.step(wall)
        snapshots.append(snapshot)
        if (
            snapshot.chunk_seq in records
            and snapshot.reference_kind == "CHUNK"
            and snapshot.reference_application_sim_time_ns
            <= records[snapshot.chunk_seq].request.anchor.start_sim_time_ns
            + records[snapshot.chunk_seq].plan.duration_ns
        ):
            compare_record(records[snapshot.chunk_seq], snapshot)
        if snapshot.phase in TERMINAL:
            terminal = snapshot
            break
        wall += DT_NS

    assert terminal is not None and terminal.phase is Phase.COMPLETED
    assert set(records) == {1, 2, 3}
    for record in records.values():
        validate_accepted_reference(record, profile, control_config.max_accelerations)
        ros_roundtrip_record(record)
    feedback = execution_feedback(status_for(terminal, profile))
    assert feedback.phase == "COMPLETED"
    assert feedback.stream_id
    assert feedback.chunk_seq == 3
    ros_roundtrip_execution(feedback)
    assert any(
        snapshot.reference_kind == "CHUNK" and snapshot.chunk_seq == 2 for snapshot in snapshots
    )
    assert any(snapshot.reference_kind == "BRAKE" for snapshot in snapshots) is False


def test_contact_evidence_reaches_frozen_feedback(scene, profile, control_config):
    io = fresh_io(scene, profile)
    install_contact_source(scene)
    closed = list(profile.initial_positions)
    closed[15] = 0.0
    # Pre-run free-object fixture: establish a real, stable grasp before the
    # command. A slow reference ramp would let an unsupported block fall out.
    for _ in range(500):
        io.apply_targets(closed)
        mujoco.mj_step(scene.model, scene.data)
        mujoco.mj_forward(scene.model, scene.data)
    assert io.measure().positions[15] == pytest.approx(0.04, abs=0.002)
    initial = tuple(closed)
    request = request_for(profile, 1, (Target(1_000_000_000, initial),), kind=NATIVE_MOTION)
    packet = prepare_request(request, profile, control_config, initial_reference=initial)
    motion = NativeMotion(scene, io, control_config, initial_reference=initial)
    motion.start(packet.prepared, 0)
    terminal = None
    for index in range(5000):
        snapshot = motion.step(index * DT_NS)
        if snapshot.phase in TERMINAL:
            terminal = snapshot
            break
    assert terminal is not None and terminal.phase is Phase.COMPLETED
    feedback = execution_feedback(status_for(terminal, profile))
    right = next(gripper for gripper in feedback.grippers if gripper.side == "right")
    assert right.mode == "CONTACT"
    assert right.contact_ok and right.contact_object == "source"
    assert right.target_position_m == 0.0
    assert 0.03 < right.measured_position_m < 0.05
    assert right.target_error_m == pytest.approx(-right.measured_position_m)
    assert right.fingers[0].object_names == ("source",)
    assert right.fingers[0].contact_counts[0] >= 1
    assert right.fingers[0].normal_forces[0] > 0.0
    ros_roundtrip_execution(feedback)


def test_failed_terminal_keeps_real_applied_kind_in_feedback(scene, profile, control_config):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    targets = list(initial)
    targets[9] += 0.1
    request = request_for(profile, 1, (Target(3_000_000_000, tuple(targets)),), kind=NATIVE_MOTION)
    packet = prepare_request(request, profile, control_config, initial_reference=initial)
    motion = NativeMotion(scene, io, control_config, initial_reference=initial)
    motion.start(packet.prepared, 0)
    snapshot = None
    wall = 0
    for index in range(300):
        snapshot = motion.step(index * DT_NS)
        wall = index * DT_NS
    assert snapshot is not None and snapshot.phase is Phase.RUNNING
    stopping = motion.request_stop("MANUAL_BRAKE", wall + DT_NS)
    assert stopping.phase is Phase.STOPPING
    terminal = motion.poll(wall + DT_NS + 3_100_000_000)
    assert terminal.phase is Phase.FAILED
    assert terminal.reason == "STOP_TIMEOUT"
    assert terminal.reference_kind == "TRAJECTORY"
    feedback = execution_feedback(status_for(terminal, profile))
    assert feedback.reference_kind == "TRAJECTORY"
    assert feedback.phase == "FAILED"
    assert feedback.motion_phase == "FAILED"
    ros_roundtrip_execution(feedback)
