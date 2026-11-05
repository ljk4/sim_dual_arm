from __future__ import annotations

import sys
import threading
from dataclasses import replace
from pathlib import Path

import pytest

mujoco = pytest.importorskip("mujoco")
pytest.importorskip("scipy")
np = pytest.importorskip("numpy")

ROOT = Path(__file__).resolve().parents[2]
for source in (ROOT / "simulation/mujoco/src", ROOT / "contracts/python/src"):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

from robot_backend_sim.chunk_plan import prepare_chunk  # noqa: E402
from robot_backend_sim.motion import NativeMotion, Phase, prepare_motion  # noqa: E402
from robot_backend_sim.motion_config import load_control_config  # noqa: E402
from robot_backend_sim.panda_io import PandaIO  # noqa: E402
from robot_backend_sim.reference_compiler import compile_reference_plan  # noqa: E402
from robot_backend_sim.scene import build_scene  # noqa: E402
from robot_backend_sim.scene_config import load_scene_config  # noqa: E402
from robot_backend_sim.stream_config import StreamConfig  # noqa: E402
from robot_contracts.action_chunks import ActionWindow  # noqa: E402
from robot_contracts.models import ContractError, Target  # noqa: E402
from robot_contracts.profiles import load_profile  # noqa: E402

CONFIG_PATH = ROOT / "simulation/mujoco/configs/scene/panda_stack.yaml"
CONTROL_PATH = ROOT / "simulation/mujoco/configs/control/panda_control.yaml"
STREAM_PATH = ROOT / "simulation/mujoco/configs/control/panda_stream.yaml"
PROFILE_PATH = ROOT / "profiles/robots/panda_stack_v1/profile.json"
SCHEMAS = ROOT / "contracts/schemas"
DT_NS = 2_000_000
SAMPLE_NS = 50_000_000
START_NS = 100_000_000
DURATION_NS = 3_000_000_000
RIGHT_JOINT1 = 8
TERMINAL = frozenset({Phase.COMPLETED, Phase.STOPPED, Phase.FAILED})
LEFT_ARM = tuple(range(7))


@pytest.fixture(scope="module")
def scene_config():
    return load_scene_config(CONFIG_PATH)


@pytest.fixture(scope="module")
def scene(scene_config):
    return build_scene(
        scene_config,
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


def smoothstep(delta: float, duration_s: float, time_s: float) -> float:
    u = time_s / duration_s
    return delta * (3.0 * u * u - 2.0 * u * u * u)


def original_plan(profile, control_config, initial, *, duration_s=3.0, delta=0.15):
    targets = tuple(
        Target(
            time_ns,
            tuple(
                value + smoothstep(delta, duration_s, time_ns / 1_000_000_000)
                if index == RIGHT_JOINT1
                else value
                for index, value in enumerate(initial)
            ),
        )
        for time_ns in range(SAMPLE_NS, int(duration_s * 1e9) + 1, SAMPLE_NS)
    )
    return compile_reference_plan(
        initial,
        targets,
        profile.joints,
        control_config.max_accelerations,
        initial_velocities=(0.0,) * len(initial),
        final_velocities=(0.0,) * len(initial),
    )


def window_for(plan, *, stream_start_ns, observation_ns, handoff_ns, observation_id):
    targets = []
    for offset in range(SAMPLE_NS, 1_000_000_001, SAMPLE_NS):
        absolute_ns = handoff_ns + offset
        sample = plan.sample_interval(absolute_ns - stream_start_ns)
        targets.append(Target(absolute_ns - observation_ns, sample.positions))
    return ActionWindow(
        observation_id,
        observation_ns,
        handoff_ns,
        tuple(targets),
        len(targets),
    )


def prepared_chunks(motion, profile, control_config, plan, scene, *, stream_start_ns):
    """Build all three 1 s chunks with realistic observation times."""
    first_base = motion.chunk_anchor(stream_start_ns)
    first = prepare_chunk(
        first_base,
        window_for(
            plan,
            stream_start_ns=stream_start_ns,
            observation_ns=0,
            handoff_ns=stream_start_ns,
            observation_id=1,
        ),
        profile,
        control_config,
        final_velocities=plan.sample_interval(1_000_000_000).velocities,
    )
    accepted = motion.start_stream(first, 0)

    # The test loop prepares each successor a little before it is needed, using
    # the real current simulation time as the observation time.
    return first, accepted


def collect_events(events, snapshot):
    for event in snapshot.chunk_events:
        events.setdefault(event.event_seq, event)


def drive_stream_three_chunks(scene, motion, profile, control_config, plan, snapshots):
    events = {}
    chunk2 = None
    chunk3 = None
    terminal = None
    for _index in range(3000):
        application_ns = round(float(scene.data.time) * 1e9)
        if application_ns == 400_000_000:
            anchor = motion.chunk_anchor()
            chunk2 = prepare_chunk(
                anchor,
                window_for(
                    plan,
                    stream_start_ns=START_NS,
                    observation_ns=application_ns,
                    handoff_ns=1_100_000_000,
                    observation_id=2,
                ),
                profile,
                control_config,
                final_velocities=plan.sample_interval(2_000_000_000).velocities,
            )
            submission = motion.queue_chunk(chunk2, application_ns)
            assert submission.accepted, submission.reason
            collect_events(events, submission.snapshot)
        if application_ns == 1_400_000_000:
            anchor = motion.chunk_anchor()
            chunk3 = prepare_chunk(
                anchor,
                window_for(
                    plan,
                    stream_start_ns=START_NS,
                    observation_ns=application_ns,
                    handoff_ns=2_100_000_000,
                    observation_id=3,
                ),
                profile,
                control_config,
                settle_at_end=True,
            )
            submission = motion.queue_chunk(chunk3, application_ns)
            assert submission.accepted, submission.reason
            collect_events(events, submission.snapshot)
        snapshot = motion.step(application_ns)
        snapshots.append(snapshot)
        collect_events(events, snapshot)
        if snapshot.phase in TERMINAL:
            terminal = snapshot
            break
    assert terminal is not None
    return chunk2, chunk3, terminal, events


def install_contact_source(scene) -> None:
    """One pre-run fixture: free source between the right fingers at the TCP."""
    model = scene.model
    data = scene.data
    mujoco.mj_forward(model, data)
    tcp = int(scene.indices.tcp_site_ids["right_tcp"])
    position = np.asarray(data.site_xpos[tcp], dtype=float)
    matrix = np.asarray(data.site_xmat[tcp], dtype=float)
    quaternion = np.zeros(4, dtype=float)
    mujoco.mju_mat2Quat(quaternion, matrix)
    address = int(scene.indices.object_free_joint_qpos["source"])
    data.qpos[address : address + 7] = (*position, *quaternion)
    free_dof = int(model.joint("source_free").dofadr[0])
    data.qvel[free_dof : free_dof + 6] = 0.0
    for name in ("right_finger_joint1", "right_finger_joint2"):
        joint = model.joint(name)
        data.qpos[int(joint.qposadr[0])] = 0.02
        data.qvel[int(joint.dofadr[0])] = 0.0
    mujoco.mj_forward(model, data)


def test_three_chunks_share_one_stream_step_and_replay_exactly(
    scene, profile, control_config, stream_config
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    plan = original_plan(profile, control_config, initial)
    motion = NativeMotion(
        scene, io, control_config, initial_reference=initial, stream_config=stream_config
    )
    first, accepted = prepared_chunks(
        motion, profile, control_config, plan, scene, stream_start_ns=START_NS
    )
    snapshots = []
    events = {}
    collect_events(events, accepted)
    first_started = motion.step(0)
    snapshots.append(first_started)
    collect_events(events, first_started)
    chunk2, chunk3, terminal, more = drive_stream_three_chunks(
        scene, motion, profile, control_config, plan, snapshots
    )
    events.update(more)
    assert chunk2 is not None and chunk3 is not None
    assert terminal.phase is Phase.COMPLETED
    assert motion.result is not None and motion.result.phase is Phase.COMPLETED
    assert all(
        snapshot.phase not in (Phase.STOPPING, Phase.STOPPED, Phase.FAILED)
        for snapshot in snapshots
    )

    # Boundary p/v continuity comes from the plans themselves and from the one
    # actual application at each shared physics instant.
    for left, right in ((first, chunk2), (chunk2, chunk3)):
        at_end = left.sample_at(left.end_sim_time_ns)
        at_start = right.sample_at(right.start_sim_time_ns)
        np.testing.assert_allclose(at_end.positions, at_start.positions, atol=1e-12, rtol=0)
        np.testing.assert_allclose(at_end.velocities, at_start.velocities, atol=1e-12, rtol=0)
    for snapshot in snapshots:
        app = snapshot.reference_application_sim_time_ns
        if START_NS <= app < 3_100_000_000:
            assert snapshot.reference_kind == "CHUNK"

    started = [event.chunk_seq for event in events.values() if event.phase == "STARTED"]
    retired = [event.chunk_seq for event in events.values() if event.phase == "RETIRED"]
    assert started == [1, 2, 3]
    assert retired == [1, 2, 3]
    assert [
        event.event_seq for event in sorted(events.values(), key=lambda e: e.event_seq)
    ] == list(range(1, len(events) + 1))
    # The three chunks belong to one motion id, and only the final settling
    # phase can complete the motion.
    assert {event.motion_seq for event in events.values()} == {motion.result.motion_seq}
    assert motion.result.motion_seq == 1

    final = terminal
    for index in LEFT_ARM:
        assert (
            abs(final.measured_positions[index] - initial[index])
            <= (control_config.position_tolerances[index])
        )
    for index, tolerance in enumerate(control_config.velocity_tolerances):
        assert abs(final.measured_velocities[index]) <= tolerance
    assert final.measured_positions[RIGHT_JOINT1] > initial[RIGHT_JOINT1] + 0.1
    with pytest.raises(ContractError) as stale_after_end:
        motion.start_stream(first, 10**12)
    assert stale_after_end.value.reason == "STALE_STREAM"

    # Independent raw ctrl/step/forward replay of every applied reference,
    # including free objects, must reproduce qpos/qvel exactly.
    expected_qpos = scene.data.qpos.copy()
    expected_qvel = scene.data.qvel.copy()
    references = [snapshot.reference_positions for snapshot in snapshots]
    scene.reset(0)
    replay_io = PandaIO(scene, profile)
    for positions in references:
        replay_io.apply_targets(positions)
        mujoco.mj_step(scene.model, scene.data)
        mujoco.mj_forward(scene.model, scene.data)
    np.testing.assert_array_equal(scene.data.qpos, expected_qpos)
    np.testing.assert_array_equal(scene.data.qvel, expected_qvel)


def test_missing_successor_applies_endpoint_then_underrun_brakes(
    scene, profile, control_config, stream_config
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    plan = original_plan(profile, control_config, initial)
    motion = NativeMotion(
        scene, io, control_config, initial_reference=initial, stream_config=stream_config
    )
    anchor = motion.chunk_anchor(START_NS)
    chunk = prepare_chunk(
        anchor,
        window_for(
            plan,
            stream_start_ns=START_NS,
            observation_ns=0,
            handoff_ns=START_NS,
            observation_id=1,
        ),
        profile,
        control_config,
        final_velocities=plan.sample_interval(1_000_000_000).velocities,
    )
    motion.start_stream(chunk, 0)
    terminal = None
    end_step = None
    for _index in range(1500):
        application_ns = round(float(scene.data.time) * 1e9)
        snapshot = motion.step(application_ns)
        if snapshot.reference_application_sim_time_ns == chunk.end_sim_time_ns:
            end_step = snapshot
        if snapshot.phase in TERMINAL:
            terminal = snapshot
            break
    assert end_step is not None
    assert end_step.reference_kind == "CHUNK"
    assert any(
        event.phase == "RETIRED" and event.reason == "UNDERRUN" for event in end_step.chunk_events
    )
    assert terminal is not None and terminal.phase is Phase.FAILED
    assert terminal.reason == "UNDERRUN"
    assert motion.result is not None and motion.result.stop_reason == "UNDERRUN"


def test_queue_full_and_stop_cancels_pending_without_starting_it(
    scene, profile, control_config, stream_config
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    plan = original_plan(profile, control_config, initial)
    motion = NativeMotion(
        scene, io, control_config, initial_reference=initial, stream_config=stream_config
    )
    prepared_chunks(motion, profile, control_config, plan, scene, stream_start_ns=START_NS)
    for _index in range(250):
        application_ns = round(float(scene.data.time) * 1e9)
        snapshot = motion.step(application_ns)
    assert snapshot.phase is Phase.RUNNING
    application_ns = round(float(scene.data.time) * 1e9)
    anchor = motion.chunk_anchor()
    successor = prepare_chunk(
        anchor,
        window_for(
            plan,
            stream_start_ns=START_NS,
            observation_ns=application_ns,
            handoff_ns=1_100_000_000,
            observation_id=2,
        ),
        profile,
        control_config,
        final_velocities=plan.sample_interval(2_000_000_000).velocities,
    )
    accepted = motion.queue_chunk(successor, application_ns)
    assert accepted.accepted
    full = motion.queue_chunk(successor, application_ns)
    assert not full.accepted and full.reason == "QUEUE_FULL"
    stopping = motion.request_stop("OPERATOR_STOP", application_ns)
    assert stopping.phase is Phase.STOPPING
    assert any(
        event.phase == "CANCELED" and event.chunk_seq == successor.seq
        for event in stopping.chunk_events
    )
    terminal = None
    for _index in range(800):
        application_ns = round(float(scene.data.time) * 1e9)
        snapshot = motion.step(application_ns)
        if snapshot.phase in TERMINAL:
            terminal = snapshot
            break
    assert terminal is not None and terminal.phase is Phase.STOPPED
    assert motion.result is not None and motion.result.reason == "OPERATOR_STOP"
    assert not any(
        event.phase == "STARTED" and event.chunk_seq == successor.seq
        for snapshot in [stopping, terminal]
        for event in snapshot.chunk_events
    )


def test_stale_successor_rejects_and_latches_bounded_failure(
    scene, profile, control_config, stream_config
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    plan = original_plan(profile, control_config, initial)
    motion = NativeMotion(
        scene, io, control_config, initial_reference=initial, stream_config=stream_config
    )
    prepared_chunks(motion, profile, control_config, plan, scene, stream_start_ns=START_NS)
    # Advance to 0.2 s and build a legal successor with that observation; queue
    # it only at 0.8 s, when the observation is older than max_observation_age.
    while round(float(scene.data.time) * 1e9) < 200_000_000:
        motion.step(round(float(scene.data.time) * 1e9))
    observation_ns = 200_000_000
    anchor = motion.chunk_anchor()
    successor = prepare_chunk(
        anchor,
        window_for(
            plan,
            stream_start_ns=START_NS,
            observation_ns=observation_ns,
            handoff_ns=1_100_000_000,
            observation_id=2,
        ),
        profile,
        control_config,
        final_velocities=plan.sample_interval(2_000_000_000).velocities,
    )
    while round(float(scene.data.time) * 1e9) < 800_000_000:
        motion.step(round(float(scene.data.time) * 1e9))
    submission = motion.queue_chunk(successor, 800_000_000)
    assert not submission.accepted and submission.reason == "OBSERVATION_STALE"
    assert submission.snapshot.phase is Phase.STOPPING
    terminal = None
    while round(float(scene.data.time) * 1e9) < 1_200_000_000:
        snapshot = motion.step(round(float(scene.data.time) * 1e9))
        if snapshot.phase in TERMINAL:
            terminal = snapshot
            break
    assert terminal is not None and terminal.phase is Phase.FAILED
    assert terminal.reason == "OBSERVATION_STALE"


def test_new_stream_entries_reject_other_threads(scene, profile, control_config, stream_config):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    motion = NativeMotion(
        scene, io, control_config, initial_reference=initial, stream_config=stream_config
    )
    calls = (
        lambda: motion.chunk_anchor(),
        lambda: motion.start_stream(object(), 0),
        lambda: motion.queue_chunk(object(), 0),
    )
    for call in calls:
        outcome = []

        def target(function=call, outcome=outcome):
            try:
                outcome.append(function())
            except BaseException as exc:
                outcome.append(exc)

        thread = threading.Thread(target=target)
        thread.start()
        thread.join()
        error = outcome[0]
        assert isinstance(error, ContractError)
        assert error.reason == "WRONG_THREAD"


def test_free_block_contact_stream_moves_slightly_and_completes_in_contact(
    scene, profile, control_config, stream_config
):
    io = fresh_io(scene, profile)
    install_contact_source(scene)
    initial = list(profile.initial_positions)
    initial[15] = 0.0
    initial = tuple(initial)
    close = list(initial)
    close[15] = 0.0
    for _ in range(500):
        io.apply_targets(close)
        mujoco.mj_step(scene.model, scene.data)
        mujoco.mj_forward(scene.model, scene.data)
    measured = io.measure()
    assert measured.positions[15] == pytest.approx(0.03985, abs=0.0015)

    # One second of a small right-arm motion while the gripper keeps asking for
    # the original zero target; the free block is constrained by both fingers.
    targets = tuple(
        Target(
            time_ns,
            tuple(
                value + smoothstep(0.03, 1.0, time_ns / 1_000_000_000) if index == 9 else value
                for index, value in enumerate(initial)
            ),
        )
        for time_ns in range(SAMPLE_NS, 1_000_000_001, SAMPLE_NS)
    )
    plan = compile_reference_plan(
        initial,
        targets,
        profile.joints,
        control_config.max_accelerations,
        initial_velocities=(0.0,) * len(initial),
        final_velocities=(0.0,) * len(initial),
    )
    observation_ns = round(float(scene.data.time) * 1e9)
    start_ns = observation_ns + START_NS
    assert start_ns % DT_NS == 0
    motion = NativeMotion(
        scene, io, control_config, initial_reference=initial, stream_config=stream_config
    )

    # window_for builds 20 points (1 s); this stream uses two 0.5 s chunks, so
    # build the two ten-point windows directly.
    def half_window(observation_ns, handoff_ns, observation_id):
        points = tuple(
            Target(
                handoff_ns + offset - observation_ns,
                plan.sample_interval(handoff_ns + offset - start_ns).positions,
            )
            for offset in range(SAMPLE_NS, 500_000_001, SAMPLE_NS)
        )
        return ActionWindow(observation_id, observation_ns, handoff_ns, points, len(points))

    first = prepare_chunk(
        motion.chunk_anchor(start_ns),
        half_window(observation_ns, start_ns, 1),
        profile,
        control_config,
        final_velocities=plan.sample_interval(500_000_000).velocities,
    )
    events = {}
    collect_events(events, motion.start_stream(first, 0))
    terminal = None
    second = None
    for _index in range(1200):
        application_ns = round(float(scene.data.time) * 1e9)
        if second is None and application_ns >= start_ns + 300_000_000:
            second = prepare_chunk(
                motion.chunk_anchor(),
                half_window(application_ns, start_ns + 500_000_000, 2),
                profile,
                control_config,
                settle_at_end=True,
            )
            submission = motion.queue_chunk(second, application_ns)
            assert submission.accepted, submission.reason
            collect_events(events, submission.snapshot)
        snapshot = motion.step(application_ns)
        collect_events(events, snapshot)
        if snapshot.phase in TERMINAL:
            terminal = snapshot
            break
    assert second is not None
    assert terminal is not None and terminal.phase is Phase.COMPLETED
    right = terminal.grippers[1]
    assert right.mode == "CONTACT"
    assert right.contact_object == "source"
    assert right.target_position_m == pytest.approx(0.0, abs=1e-12)
    assert right.target_error_m == pytest.approx(-0.03985, abs=0.002)
    for finger in right.fingers:
        assert finger.force_for("source") >= control_config.contact_force_min_n
        assert finger.joint_speed_m_s <= control_config.finger_velocity_max_m_s
    assert (
        any(event.phase == "RETIRED" and event.chunk_seq == 2 for event in terminal.chunk_events)
        or motion.result is not None
    )


@pytest.mark.parametrize("fault_start_ns", [300_000_000, 1_500_000_000])
def test_tracking_failure_across_chunk_boundary_does_not_reset(
    scene, profile, control_config, stream_config, monkeypatch, fault_start_ns
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    plan = original_plan(profile, control_config, initial)
    motion = NativeMotion(
        scene, io, control_config, initial_reference=initial, stream_config=stream_config
    )
    real_apply = io.apply_targets
    frozen_right_joint1 = initial[RIGHT_JOINT1]

    def lossy_apply(positions):
        values = list(positions)
        if round(float(scene.data.time) * 1e9) >= fault_start_ns:
            # Real physics still runs; only one actuator target is lost, so the
            # measured state genuinely diverges from the reference.
            values[RIGHT_JOINT1] = frozen_right_joint1
        real_apply(tuple(values))

    monkeypatch.setattr(io, "apply_targets", lossy_apply)
    prepared_chunks(motion, profile, control_config, plan, scene, stream_start_ns=START_NS)
    snapshots = [motion.step(0)]
    _chunk2, _chunk3, terminal, _events = drive_stream_three_chunks(
        scene, motion, profile, control_config, plan, snapshots
    )
    boundary_ns = 1_100_000_000 if fault_start_ns < 1_000_000_000 else 2_100_000_000
    boundary = next(
        snapshot
        for snapshot in snapshots
        if snapshot.reference_application_sim_time_ns == boundary_ns
        and any(
            event.phase == "STARTED"
            and event.chunk_seq == (2 if boundary_ns == 1_100_000_000 else 3)
            for event in snapshot.chunk_events
        )
    )
    assert boundary.tracking_violation_ns >= 500_000_000
    assert terminal.phase is Phase.FAILED
    assert terminal.reason == "TRACKING_TIMEOUT"
    assert motion.result is not None
    assert motion.result.stop_reason == "TRACKING_TIMEOUT"
    assert terminal.reference_application_sim_time_ns < fault_start_ns + 2_500_000_000
    if fault_start_ns > 1_000_000_000:
        # The same violation timer survives the transition into final HOLD.
        settling = next(snapshot for snapshot in snapshots if snapshot.phase is Phase.SETTLING)
        assert settling.tracking_violation_ns >= 1_500_000_000


def test_progress_stall_poll_then_stop_timeout_latches(
    scene, profile, control_config, stream_config
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    plan = original_plan(profile, control_config, initial)
    motion = NativeMotion(
        scene, io, control_config, initial_reference=initial, stream_config=stream_config
    )
    prepared_chunks(motion, profile, control_config, plan, scene, stream_start_ns=START_NS)
    stopping = motion.poll(3_000_000_000)
    assert stopping.phase is Phase.STOPPING
    assert stopping.reason == "PROGRESS_TIMEOUT"
    terminal = motion.poll(6_000_000_001)
    assert terminal.phase is Phase.FAILED
    assert terminal.reason == "STOP_TIMEOUT"
    assert motion.result is not None
    assert motion.result.stop_reason == "PROGRESS_TIMEOUT"


def test_stream_overall_wall_timeout_uses_new_limit_not_legacy_ten_seconds(
    scene, profile, control_config
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    plan = original_plan(profile, control_config, initial)
    config = StreamConfig(0.25, 0.05, 0.5, 0.5, 3.0, 2.0)
    motion = NativeMotion(
        scene, io, control_config, initial_reference=initial, stream_config=config
    )
    anchor = motion.chunk_anchor(START_NS)
    chunk = prepare_chunk(
        anchor,
        window_for(
            plan,
            stream_start_ns=START_NS,
            observation_ns=0,
            handoff_ns=START_NS,
            observation_id=1,
        ),
        profile,
        control_config,
        final_velocities=plan.sample_interval(1_000_000_000).velocities,
    )
    motion.start_stream(chunk, 0)
    terminal = None
    for _index in range(1000):
        application_ns = round(float(scene.data.time) * 1e9)
        snapshot = motion.step(application_ns)
        if snapshot.phase in TERMINAL:
            terminal = snapshot
            break
    assert terminal is not None and terminal.phase is Phase.FAILED
    assert terminal.reason == "WALL_TIMEOUT"
    assert motion.result is not None and motion.result.stop_reason == "WALL_TIMEOUT"


def _first_chunk(motion, profile, control_config, plan, *, start_ns, observation_ns=0):
    return prepare_chunk(
        motion.chunk_anchor(start_ns),
        window_for(
            plan,
            stream_start_ns=start_ns,
            observation_ns=observation_ns,
            handoff_ns=start_ns,
            observation_id=1,
        ),
        profile,
        control_config,
        final_velocities=plan.sample_interval(1_000_000_000).velocities,
    )


def test_stream_generation_must_be_exact_next_motion_of_this_instance(
    scene, profile, control_config, stream_config
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    plan = original_plan(profile, control_config, initial)
    motion = NativeMotion(
        scene, io, control_config, initial_reference=initial, stream_config=stream_config
    )
    good = _first_chunk(motion, profile, control_config, plan, start_ns=START_NS)
    instance = good.anchor.stream_id.split(":motion-", 1)[0]
    fake_anchor = replace(good.anchor, stream_id=f"{instance}:motion-999")
    fake = prepare_chunk(
        fake_anchor,
        window_for(
            plan,
            stream_start_ns=START_NS,
            observation_ns=0,
            handoff_ns=START_NS,
            observation_id=1,
        ),
        profile,
        control_config,
        final_velocities=plan.sample_interval(1_000_000_000).velocities,
    )
    with pytest.raises(ContractError) as rejected:
        motion.start_stream(fake, 0)
    assert rejected.value.reason == "STALE_STREAM"
    assert motion.phase is Phase.IDLE
    assert motion.start_stream(good, 0).phase is Phase.ACCEPTED


def test_new_instance_after_old_object_destroyed_rejects_old_stream(
    scene, profile, control_config, stream_config
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    plan = original_plan(profile, control_config, initial)
    old = NativeMotion(
        scene, io, control_config, initial_reference=initial, stream_config=stream_config
    )
    old_anchor = old.chunk_anchor(START_NS)
    old_chunk = _first_chunk(old, profile, control_config, plan, start_ns=START_NS)
    del old
    new = NativeMotion(
        scene, io, control_config, initial_reference=initial, stream_config=stream_config
    )
    new_anchor = new.chunk_anchor(START_NS)
    assert new_anchor.stream_id != old_anchor.stream_id
    with pytest.raises(ContractError) as rejected:
        new.start_stream(old_chunk, 0)
    assert rejected.value.reason == "STALE_STREAM"
    assert new.phase is Phase.IDLE


def test_first_chunk_admission_lead_is_enforced_at_the_boundary(
    scene, profile, control_config, stream_config
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    plan = original_plan(profile, control_config, initial)
    motion = NativeMotion(
        scene, io, control_config, initial_reference=initial, stream_config=stream_config
    )
    early = _first_chunk(motion, profile, control_config, plan, start_ns=20_000_000)
    with pytest.raises(ContractError) as late:
        motion.start_stream(early, 0)
    assert late.value.reason == "CHUNK_LATE"
    assert motion.phase is Phase.IDLE
    at_deadline = _first_chunk(motion, profile, control_config, plan, start_ns=50_000_000)
    assert motion.start_stream(at_deadline, 0).phase is Phase.ACCEPTED


def test_queue_applies_expired_stream_wall_before_admission(
    scene, profile, control_config, stream_config
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    plan = original_plan(profile, control_config, initial)
    motion = NativeMotion(
        scene, io, control_config, initial_reference=initial, stream_config=stream_config
    )
    first = _first_chunk(motion, profile, control_config, plan, start_ns=START_NS)
    motion.start_stream(first, 109_000_000_000)
    successor = prepare_chunk(
        motion.chunk_anchor(),
        window_for(
            plan,
            stream_start_ns=START_NS,
            observation_ns=0,
            handoff_ns=1_100_000_000,
            observation_id=2,
        ),
        profile,
        control_config,
        final_velocities=plan.sample_interval(2_000_000_000).velocities,
    )
    submission = motion.queue_chunk(successor, 229_000_000_000)
    assert submission.accepted is False
    assert submission.reason == "WALL_TIMEOUT"
    assert submission.snapshot.phase is Phase.STOPPING
    assert submission.snapshot.reason == "WALL_TIMEOUT"


def test_chunk_anchor_rejects_legacy_active_and_bad_explicit_active_start(
    scene, profile, control_config, stream_config
):
    io = fresh_io(scene, profile)
    initial = tuple(profile.initial_positions)
    targets = (Target(400_000_000, initial),)
    legacy = prepare_motion(initial, targets, profile, control_config)
    legacy_motion = NativeMotion(
        scene, io, control_config, initial_reference=initial, stream_config=stream_config
    )
    legacy_motion.start(legacy, 0)
    with pytest.raises(ContractError) as active:
        legacy_motion.chunk_anchor()
    assert active.value.reason == "MOTION_ACTIVE"

    io2 = fresh_io(scene, profile)
    plan = original_plan(profile, control_config, initial)
    motion = NativeMotion(
        scene, io2, control_config, initial_reference=initial, stream_config=stream_config
    )
    first = _first_chunk(motion, profile, control_config, plan, start_ns=START_NS)
    motion.start_stream(first, 0)
    before_time = float(scene.data.time)
    before_seq = motion._motion_seq
    with pytest.raises(ContractError) as bad:
        motion.chunk_anchor(123)
    assert bad.value.reason == "INVALID_CHUNK_START"
    assert motion.phase is Phase.ACCEPTED
    assert motion._motion_seq == before_seq
    assert float(scene.data.time) == before_time
    assert motion.chunk_anchor().start_sim_time_ns == 1_100_000_000


def test_contact_cannot_exempt_a_failed_opening_command(
    scene, profile, control_config, stream_config, monkeypatch
):
    io = fresh_io(scene, profile)
    install_contact_source(scene)
    initial = tuple(profile.initial_positions)  # desired width remains 0.08 m
    closed_targets = list(initial)
    closed_targets[15] = 0.0
    for _ in range(500):
        io.apply_targets(closed_targets)
        mujoco.mj_step(scene.model, scene.data)
        mujoco.mj_forward(scene.model, scene.data)
    assert io.measure().positions[15] == pytest.approx(0.04, abs=0.002)
    real_apply = io.apply_targets

    def actuator_stays_closed(positions):
        # Test-only actuator failure: physics and all measurements stay real.
        values = list(positions)
        values[15] = 0.0
        real_apply(values)

    monkeypatch.setattr(io, "apply_targets", actuator_stays_closed)
    motion = NativeMotion(
        scene, io, control_config, initial_reference=initial, stream_config=stream_config
    )
    plan = original_plan(profile, control_config, initial, delta=0.01)
    observation_ns = round(scene.data.time * 1e9)
    start_ns = observation_ns + START_NS
    first = prepare_chunk(
        motion.chunk_anchor(start_ns),
        window_for(
            plan,
            stream_start_ns=start_ns,
            observation_ns=observation_ns,
            handoff_ns=start_ns,
            observation_id=1,
        ),
        profile,
        control_config,
        final_velocities=plan.sample_interval(1_000_000_000).velocities,
    )
    motion.start_stream(first, 0)
    queued = set()
    outside_at_start = False
    for tick in range(2500):
        application_ns = round(scene.data.time * 1e9)
        for seq in (2, 3):
            if seq in queued or application_ns < start_ns + (seq - 2) * 1_000_000_000 + 200_000_000:
                continue
            anchor = motion.chunk_anchor()
            chunk = prepare_chunk(
                anchor,
                window_for(
                    plan,
                    stream_start_ns=start_ns,
                    observation_ns=application_ns,
                    handoff_ns=anchor.start_sim_time_ns,
                    observation_id=seq,
                ),
                profile,
                control_config,
                final_velocities=plan.sample_interval(seq * 1_000_000_000).velocities,
                settle_at_end=seq == 3,
            )
            assert motion.queue_chunk(chunk, tick * DT_NS).accepted
            queued.add(seq)
        snapshot = motion.step(tick * DT_NS)
        if application_ns == start_ns:
            outside_at_start = snapshot.tracking_quality == "OUTSIDE_TOLERANCE"
            assert snapshot.tracking_errors[15] > 0.035
            assert snapshot.grippers[1].mode == "NONE"
            assert snapshot.contacts[1].candidate_object == "source"
        if snapshot.phase in TERMINAL:
            break
    assert outside_at_start
    assert motion.result is not None and motion.result.reason == "TRACKING_TIMEOUT"
    assert snapshot.reference_application_sim_time_ns < start_ns + DURATION_NS
