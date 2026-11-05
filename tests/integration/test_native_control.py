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

import robot_backend_sim.motion as motion_module  # noqa: E402
from robot_backend_sim.chunk_plan import prepare_chunk  # noqa: E402
from robot_backend_sim.control import TIME_DOMAIN_ID, NativeController  # noqa: E402
from robot_backend_sim.control_state import PreparedControlCommand  # noqa: E402
from robot_backend_sim.motion import (  # noqa: E402
    NativeMotion,
    Phase,
    prepare_motion,
)
from robot_backend_sim.motion_config import load_control_config  # noqa: E402
from robot_backend_sim.panda_io import PandaIO  # noqa: E402
from robot_backend_sim.reference_compiler import compile_reference_plan  # noqa: E402
from robot_backend_sim.scene import build_scene  # noqa: E402
from robot_backend_sim.scene_config import load_scene_config  # noqa: E402
from robot_backend_sim.stream_config import StreamConfig  # noqa: E402
from robot_contracts.action_chunks import ActionWindow  # noqa: E402
from robot_contracts.control import ControlCommand  # noqa: E402
from robot_contracts.models import ContractError, Target  # noqa: E402
from robot_contracts.profiles import load_profile  # noqa: E402

CONFIG_PATH = ROOT / "simulation/mujoco/configs/scene/panda_stack.yaml"
CONTROL_PATH = ROOT / "simulation/mujoco/configs/control/panda_control.yaml"
STREAM_PATH = ROOT / "simulation/mujoco/configs/control/panda_stream.yaml"
PROFILE_PATH = ROOT / "profiles/robots/panda_stack_v1/profile.json"
SCHEMAS = ROOT / "contracts/schemas"
DT_NS = 2_000_000
LEASE_NS = 10_000_000_000
START_NS = 100_000_000
SAMPLE_NS = 50_000_000
TERMINAL = frozenset({"COMPLETED", "CANCELED", "FAILED"})


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


def controller_for(scene, profile, control_config, stream_config, **limits):
    io = fresh_io(scene, profile)
    return io, NativeController(
        scene,
        io,
        control_config,
        stream_config=stream_config,
        **limits,
    )


def context_for(controller, episode="episode-1", control_epoch=1):
    return replace(
        controller.context,
        episode_id=episode,
        control_epoch=control_epoch,
    )


def motion_for(profile, control_config, initial, target_positions, duration_ns):
    return prepare_motion(
        tuple(initial),
        (Target(duration_ns, tuple(target_positions)),),
        profile,
        control_config,
    )


def changed(positions, index: int, delta: float):
    result = list(positions)
    result[index] += delta
    return tuple(result)


def motion_packet(
    context,
    lease,
    prepared,
    *,
    command_id,
    command_seq,
    observation_id=1,
):
    command = ControlCommand(
        context,
        lease,
        command_id,
        command_seq,
        observation_id,
        10_000_000_000,
    )
    return PreparedControlCommand(command, prepared)


def chunk_packet(context, lease, prepared, command):
    successor = replace(command, observation_id=prepared.window.observation_id)
    return PreparedControlCommand(successor, prepared)


def drive_terminal(controller, wall, limit=3000):
    reply = None
    for _ in range(limit):
        reply = controller.step(wall)
        if reply.status is not None and reply.status.phase in TERMINAL:
            return reply, wall
        wall += DT_NS
    raise AssertionError(f"motion did not terminate, last phase {reply.status}")


def run_in_thread(function):
    outcome = []

    def target():
        try:
            outcome.append(function())
        except BaseException as error:  # keep the worker exception for the assertion
            outcome.append(error)

    thread = threading.Thread(target=target)
    thread.start()
    thread.join()
    return outcome[0]


def expect_reason(function, reason: str):
    with pytest.raises(ContractError) as caught:
        function()
    assert caught.value.reason == reason
    return caught.value


def smoothstep(delta: float, duration_s: float, time_s: float) -> float:
    u = time_s / duration_s
    return delta * (3.0 * u * u - 2.0 * u * u * u)


def original_plan(profile, control_config, initial, *, duration_s=3.0, delta=0.15):
    targets = tuple(
        Target(
            time_ns,
            tuple(
                value + smoothstep(delta, duration_s, time_ns / 1_000_000_000)
                if index == 8
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


def test_accepted_is_not_started_and_terminal_status_is_frozen(
    scene, profile, control_config, stream_config, monkeypatch
):
    io, controller = controller_for(scene, profile, control_config, stream_config)
    context = context_for(controller)
    lease = controller.acquire(context, "owner", LEASE_NS, 0)
    initial = tuple(profile.initial_positions)
    target = changed(initial, 9, 0.03)
    prepared = motion_for(profile, control_config, initial, target, 800_000_000)
    packet = motion_packet(context, lease, prepared, command_id="c1", command_seq=1)

    monkeypatch.setattr(
        motion_module,
        "compile_reference_plan",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("control entry fitted a curve")
        ),
    )
    admitted = controller.submit(packet, 0)
    assert admitted.status is not None
    assert admitted.status.phase == "ACCEPTED"
    assert admitted.snapshot.phase is Phase.ACCEPTED
    assert admitted.status.started_sim_ns is None
    assert [event.phase for event in admitted.events] == ["ACCEPTED"]

    wall = 0
    started = controller.step(wall)
    assert started.status is not None
    assert started.status.phase == "STARTED"
    assert started.status.started_sim_ns == 0
    assert started.snapshot.measured_positions == io.measure().positions
    assert started.snapshot.tracking_errors == tuple(
        reference - measured
        for reference, measured in zip(
            started.snapshot.reference_positions,
            started.snapshot.measured_positions,
            strict=True,
        )
    )
    assert any(event.phase == "STARTED" and event.chunk_seq is None for event in started.events)
    wall += DT_NS
    terminal, wall = drive_terminal(controller, wall)
    assert terminal.status is not None
    assert terminal.status.phase == "COMPLETED"
    frozen = terminal.status
    assert frozen.finished_sim_ns is not None
    assert frozen.finished_wall_ns is not None
    assert frozen.snapshot is not None
    frozen_time = frozen.snapshot.measurement_sim_time_ns
    wall += DT_NS
    live = controller.step(wall)
    assert live.status == frozen
    assert live.snapshot.measurement_sim_time_ns > frozen_time
    assert live.snapshot.phase is Phase.COMPLETED


def test_three_chunks_keep_one_overall_command_and_idempotent_successors(
    scene, profile, control_config, stream_config
):
    io, controller = controller_for(scene, profile, control_config, stream_config)
    context = context_for(controller)
    lease = controller.acquire(context, "owner", LEASE_NS, 0)
    initial = tuple(profile.initial_positions)
    plan = original_plan(profile, control_config, initial)
    anchor = controller.chunk_anchor(context, lease, 0, START_NS)
    first = prepare_chunk(
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
    command = ControlCommand(
        context,
        lease,
        "stream-1",
        1,
        1,
        10_000_000_000,
    )
    first_packet = PreparedControlCommand(command, first)
    admitted = controller.submit(first_packet, 0)
    assert admitted.status is not None
    assert admitted.status.phase == "ACCEPTED"
    assert admitted.status.motion_seq == 1

    chunk2 = None
    chunk3 = None
    prompt = None
    events = list(admitted.events)
    terminal = None
    wall = 0
    for _index in range(4000):
        application_ns = round(float(scene.data.time) * 1e9)
        if application_ns == 400_000_000:
            anchor2 = controller.chunk_anchor(context, lease, application_ns)
            chunk2 = prepare_chunk(
                anchor2,
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
            prompt = chunk_packet(context, lease, chunk2, command)
            queued = controller.queue_chunk(prompt, application_ns)
            assert queued.accepted, queued.reason
            events.extend(queued.events)
            duplicate = controller.queue_chunk(prompt, application_ns)
            assert duplicate.accepted
            assert duplicate.events == ()
            assert duplicate.status == queued.status
            conflict_command = replace(command, command_seq=2)
            conflict = chunk_packet(context, lease, chunk2, conflict_command)
            expect_reason(
                lambda packet=conflict, now=application_ns: controller.queue_chunk(packet, now),
                "COMMAND_ID_CONFLICT",
            )
        if application_ns == 1_400_000_000:
            anchor3 = controller.chunk_anchor(context, lease, application_ns)
            chunk3 = prepare_chunk(
                anchor3,
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
            prompt3 = chunk_packet(context, lease, chunk3, command)
            queued = controller.queue_chunk(prompt3, application_ns)
            assert queued.accepted, queued.reason
            events.extend(queued.events)
        assert application_ns >= wall
        wall = application_ns
        reply = controller.step(wall)
        events.extend(reply.events)
        if any(event.phase == "RETIRED" and event.chunk_seq == 1 for event in reply.events):
            assert reply.status is not None
            assert reply.status.phase == "STARTED"
            assert reply.status.phase != "COMPLETED"
        if reply.status is not None and reply.status.phase in TERMINAL:
            terminal = reply
            break
    assert chunk2 is not None and chunk3 is not None
    assert terminal is not None
    assert terminal.status is not None
    assert terminal.status.phase == "COMPLETED"
    assert terminal.status.motion_seq == 1
    assert terminal.status.command_id == "stream-1"
    assert terminal.status.observation_id == 1
    assert terminal.status.chunk_seq == 3
    assert not any(event.phase == "COMPLETED" and event.scope == "CHUNK" for event in events)
    assert (
        len([event for event in events if event.phase == "STARTED" and event.chunk_seq is None])
        == 1
    )
    assert (
        len([event for event in events if event.phase == "COMPLETED" and event.chunk_seq is None])
        == 1
    )
    retired = {event.chunk_seq for event in events if event.phase == "RETIRED"}
    assert {1, 2, 3}.issubset(retired)
    assert controller.query_chunk(context, "stream-1", 1).phase == "RETIRED"
    assert controller.query_chunk(context, "stream-1", 2).phase == "RETIRED"
    assert controller.query_chunk(context, "stream-1", 3).phase == "RETIRED"
    assert controller.query_command(context, "stream-1") == terminal.status


def test_expired_envelope_still_latches_late_successor(
    scene, profile, control_config, stream_config
):
    _io, controller = controller_for(scene, profile, control_config, stream_config)
    context = context_for(controller)
    lease = controller.acquire(context, "owner", LEASE_NS, 0)
    initial = tuple(profile.initial_positions)
    plan = original_plan(profile, control_config, initial)
    anchor = controller.chunk_anchor(context, lease, 0, START_NS)
    first = prepare_chunk(
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
    command = ControlCommand(
        context,
        lease,
        "late-flow",
        1,
        1,
        10_000_000_000,
    )
    controller.submit(PreparedControlCommand(command, first), 0)
    wall = 0
    while round(float(scene.data.time) * 1e9) < 1_080_000_000:
        wall = round(float(scene.data.time) * 1e9)
        controller.step(wall)
    application_ns = round(float(scene.data.time) * 1e9)
    anchor2 = controller.chunk_anchor(context, lease, application_ns)
    second = prepare_chunk(
        anchor2,
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
    late_command = replace(
        command,
        observation_id=2,
        accept_before_ns=1_050_000_000,
    )
    reply = controller.queue_chunk(
        PreparedControlCommand(late_command, second),
        application_ns,
    )
    assert not reply.accepted
    assert reply.status is not None
    assert reply.status.phase in ("STOPPING", "FAILED")
    assert any(event.phase == "STOPPING" and event.chunk_seq is None for event in reply.events)

    terminal, wall = drive_terminal(controller, application_ns)
    assert terminal.status is not None
    assert terminal.status.phase in ("CANCELED", "FAILED")
    assert terminal.status.phase != "COMPLETED"
    assert terminal.status.reason in ("CHUNK_LATE", "COMMAND_EXPIRED")
    assert not any(
        event.phase == "COMPLETED" and event.chunk_seq is None
        for event in controller.poll(wall).events
    )


def test_release_revokes_authority_immediately_but_waits_for_real_stop(
    scene, profile, control_config, stream_config
):
    _io, controller = controller_for(scene, profile, control_config, stream_config)
    context = context_for(controller)
    lease = controller.acquire(context, "owner", 5_000_000_000, 0)
    initial = tuple(profile.initial_positions)
    prepared = motion_for(
        profile,
        control_config,
        initial,
        changed(initial, 9, 0.05),
        2_000_000_000,
    )
    packet = motion_packet(context, lease, prepared, command_id="c1", command_seq=1)
    controller.submit(packet, 0)
    wall = 0
    for _ in range(30):
        controller.step(wall)
        wall += DT_NS

    released = controller.release(context, lease, wall)
    assert released.status is not None
    assert released.status.phase == "STOPPING"
    assert released.snapshot.phase is Phase.STOPPING
    assert released.status.finished_sim_ns is None
    expect_reason(lambda: controller.renew(context, lease, LEASE_NS, wall), "LEASE_EXPIRED")
    expect_reason(
        lambda: controller.acquire(context, "second", LEASE_NS, wall),
        "CONTROL_BUSY",
    )
    terminal, wall = drive_terminal(controller, wall)
    assert terminal.status is not None
    assert terminal.status.phase == "CANCELED"
    assert terminal.status.stop_reason == "CONTROL_RELEASED"
    assert terminal.snapshot.phase is Phase.STOPPED


def test_frozen_sim_clock_expires_lease_then_stop_timeout_keeps_reason(
    scene, profile, control_config, stream_config
):
    _io, controller = controller_for(scene, profile, control_config, stream_config)
    context = context_for(controller)
    lease = controller.acquire(context, "owner", 100_000_000, 0)
    initial = tuple(profile.initial_positions)
    prepared = motion_for(
        profile,
        control_config,
        initial,
        changed(initial, 9, 0.05),
        2_000_000_000,
    )
    packet = motion_packet(context, lease, prepared, command_id="c1", command_seq=1)
    controller.submit(packet, 0)
    wall = 0
    for _ in range(4):
        controller.step(wall)
        wall += DT_NS

    expiry = controller.poll(100_000_000)
    assert expiry.status is not None
    assert expiry.status.phase == "STOPPING"
    assert expiry.status.reason == "LEASE_EXPIRED"
    assert expiry.snapshot.phase is Phase.STOPPING
    sim_time = expiry.snapshot.measurement_sim_time_ns
    stopped = None
    for poll_wall in (200_000_000, 1_000_000_000, 3_000_000_000):
        stopped = controller.poll(poll_wall)
        assert stopped.status is not None
        assert stopped.status.phase == "STOPPING"
        assert stopped.snapshot.measurement_sim_time_ns == sim_time
        assert stopped.events == ()

    failed = controller.poll(3_100_000_000)
    assert failed.status is not None
    assert failed.status.phase == "FAILED"
    assert failed.status.reason == "STOP_TIMEOUT"
    assert failed.status.stop_reason == "LEASE_EXPIRED"
    assert failed.snapshot.phase is Phase.FAILED
    frozen = failed.status
    again = controller.poll(4_000_000_000)
    assert again.status == frozen
    assert again.events == ()
    assert again.status.finished_wall_ns == frozen.finished_wall_ns

    reset_context = controller.acquire(context, "reset-owner", 1_000_000_000, 4_000_000_000)
    new_context = controller.reset(
        context,
        reset_context,
        7,
        4_000_000_000,
    )
    assert new_context.clock_epoch == context.clock_epoch + 1
    assert new_context.backend_instance_id == context.backend_instance_id
    assert new_context.time_domain_id == TIME_DOMAIN_ID
    assert new_context.episode_id == ""
    assert new_context.control_epoch == 0
    expect_reason(
        lambda: controller.query_command(context, "c1"),
        "STALE_CONTEXT",
    )


def test_same_command_id_starts_once_and_bad_request_does_not_consume_sequence(
    scene, profile, control_config, stream_config
):
    _io, controller = controller_for(scene, profile, control_config, stream_config)
    context = context_for(controller)
    lease = controller.acquire(context, "owner", LEASE_NS, 0)
    initial = tuple(profile.initial_positions)
    target = changed(initial, 9, 0.01)
    prepared = motion_for(profile, control_config, initial, target, 300_000_000)
    packet = motion_packet(context, lease, prepared, command_id="c1", command_seq=1)
    admitted = controller.submit(packet, 0)
    duplicate = controller.submit(packet, 0)
    assert duplicate.events == ()
    assert duplicate.status == admitted.status
    started = controller.step(0)
    assert started.status is not None
    assert started.status.phase == "STARTED"
    sequence = started.status.motion_seq
    retried = controller.submit(packet, DT_NS)
    assert retried.status is not None
    assert retried.status.phase == "STARTED"
    assert retried.status.motion_seq == sequence
    assert retried.events == ()

    altered = replace(packet.command, observation_id=2)
    expect_reason(
        lambda: controller.submit(PreparedControlCommand(altered, prepared), DT_NS),
        "COMMAND_ID_CONFLICT",
    )
    expect_reason(
        lambda: controller.submit(
            PreparedControlCommand(replace(packet.command, command_seq=2), prepared),
            DT_NS,
        ),
        "COMMAND_ID_CONFLICT",
    )
    expect_reason(
        lambda: controller.submit(
            PreparedControlCommand(replace(packet.command, lease_id="other"), prepared),
            DT_NS,
        ),
        "COMMAND_ID_CONFLICT",
    )
    other_plan = motion_for(
        profile,
        control_config,
        initial,
        changed(initial, 9, 0.01),
        400_000_000,
    )
    expect_reason(
        lambda: controller.submit(
            motion_packet(context, lease, other_plan, command_id="c1", command_seq=1),
            DT_NS,
        ),
        "COMMAND_ID_CONFLICT",
    )

    wall = DT_NS
    terminal, wall = drive_terminal(controller, wall)
    assert terminal.status is not None
    assert terminal.status.phase == "COMPLETED"
    saved = controller.submit(packet, wall)
    assert saved.status == terminal.status
    assert saved.events == ()

    bad_initial = changed(initial, 9, 0.005)
    bad_prepared = motion_for(
        profile,
        control_config,
        bad_initial,
        bad_initial,
        300_000_000,
    )
    bad_packet = motion_packet(
        context,
        lease,
        bad_prepared,
        command_id="c2",
        command_seq=2,
    )
    expect_reason(lambda: controller.submit(bad_packet, wall), "INITIAL_REFERENCE_MISMATCH")
    expect_reason(lambda: controller.query_command(context, "c2"), "RESULT_EXPIRED")

    good_prepared = motion_for(
        profile,
        control_config,
        target,
        changed(target, 9, 0.01),
        300_000_000,
    )
    good_packet = motion_packet(
        context,
        lease,
        good_prepared,
        command_id="c3",
        command_seq=2,
    )
    accepted = controller.submit(good_packet, wall)
    assert accepted.status is not None
    assert accepted.status.phase == "ACCEPTED"
    assert accepted.status.command_id == "c3"


def test_old_context_queries_stay_readable_but_cannot_control_and_history_expires(
    scene, profile, control_config, stream_config
):
    _io, controller = controller_for(
        scene,
        profile,
        control_config,
        stream_config,
        max_history=2,
        max_chunk_history=2,
    )
    context1 = context_for(controller, "episode-1", 1)
    lease1 = controller.acquire(context1, "owner-1", LEASE_NS, 0)
    initial = tuple(profile.initial_positions)
    target1 = changed(initial, 9, 0.01)
    first = motion_for(profile, control_config, initial, target1, 300_000_000)
    controller.submit(
        motion_packet(context1, lease1, first, command_id="c1", command_seq=1),
        0,
    )
    first_terminal, wall = drive_terminal(controller, 0)
    assert first_terminal.status is not None
    assert first_terminal.status.phase == "COMPLETED"
    controller.release(context1, lease1, wall)

    context2 = context_for(controller, "episode-2", 1)
    lease2 = controller.acquire(context2, "owner-2", LEASE_NS, wall)
    old_status = controller.query_command(context1, "c1")
    assert old_status.phase == "COMPLETED"
    expect_reason(lambda: controller.query_command(context2, "c1"), "STALE_CONTEXT")

    target2 = changed(target1, 9, 0.01)
    second = motion_for(profile, control_config, target1, target2, 300_000_000)
    old_control = motion_packet(
        context1,
        lease1,
        second,
        command_id="old-control",
        command_seq=2,
    )
    expect_reason(lambda: controller.submit(old_control, wall), "STALE_CONTEXT")
    controller.submit(
        motion_packet(context2, lease2, second, command_id="c2", command_seq=2),
        wall,
    )
    second_terminal, wall = drive_terminal(controller, wall)
    assert second_terminal.status is not None
    assert second_terminal.status.phase == "COMPLETED"

    target3 = changed(target2, 9, 0.01)
    third = motion_for(profile, control_config, target2, target3, 300_000_000)
    controller.submit(
        motion_packet(context2, lease2, third, command_id="c3", command_seq=3),
        wall,
    )
    third_terminal, wall = drive_terminal(controller, wall)
    assert third_terminal.status is not None
    assert third_terminal.status.phase == "COMPLETED"
    expect_reason(lambda: controller.query_command(context1, "c1"), "RESULT_EXPIRED")
    assert controller.query_command(context2, "c2").phase == "COMPLETED"
    assert controller.query_command(context2, "c3").phase == "COMPLETED"


def test_reset_rejects_motion_then_restores_new_generation_deterministically(
    scene, profile, control_config, stream_config
):
    _io, controller = controller_for(scene, profile, control_config, stream_config)
    context = context_for(controller)
    lease = controller.acquire(context, "owner", LEASE_NS, 0)
    initial = tuple(profile.initial_positions)
    prepared = motion_for(
        profile,
        control_config,
        initial,
        changed(initial, 9, 0.05),
        2_000_000_000,
    )
    packet = motion_packet(context, lease, prepared, command_id="c1", command_seq=1)
    controller.submit(packet, 0)
    wall = 0
    for _ in range(80):
        controller.step(wall)
        wall += DT_NS
    expect_reason(
        lambda: controller.reset(context, lease, 7, wall),
        "RESET_NOT_ALLOWED",
    )
    controller.stop(context, lease, "OPERATOR_STOP", wall)
    terminal, wall = drive_terminal(controller, wall)
    assert terminal.status is not None
    assert terminal.status.phase == "CANCELED"

    new_context = controller.reset(context, lease, 7, wall)
    assert new_context.clock_epoch == 1
    assert new_context.backend_instance_id == context.backend_instance_id
    assert new_context.time_domain_id == context.time_domain_id
    assert new_context.episode_id == ""
    assert new_context.control_epoch == 0
    first_positions = scene.data.qpos.copy()
    assert float(scene.data.time) == 0.0
    expect_reason(lambda: controller.query_command(context, "c1"), "STALE_CONTEXT")
    expect_reason(lambda: controller.renew(context, lease, LEASE_NS, wall), "STALE_CONTEXT")
    expect_reason(
        lambda: controller.submit(packet, wall),
        "STALE_CONTEXT",
    )
    expect_reason(
        lambda: controller.chunk_anchor(context, lease, wall),
        "STALE_CONTEXT",
    )

    same_seed_context = context_for(controller, "episode-reset-2", 1)
    same_seed_lease = controller.acquire(same_seed_context, "owner", LEASE_NS, wall)
    second_context = controller.reset(same_seed_context, same_seed_lease, 7, wall)
    assert second_context.clock_epoch == 2
    assert np.array_equal(first_positions, scene.data.qpos)
    assert float(scene.data.time) == 0.0


def test_expiry_event_survives_rejected_renew_acquire_anchor_and_reset(
    scene, profile, control_config, stream_config
):
    _io, controller = controller_for(scene, profile, control_config, stream_config)
    context = context_for(controller)
    lease = controller.acquire(context, "owner", 100_000_000, 0)
    initial = tuple(profile.initial_positions)
    prepared = motion_for(
        profile,
        control_config,
        initial,
        changed(initial, 9, 0.15),
        3_000_000_000,
    )
    packet = motion_packet(context, lease, prepared, command_id="c1", command_seq=1)
    admitted = controller.submit(packet, 0)
    events = list(admitted.events)
    wall = 0
    for _ in range(300):
        reply = controller.step(wall)
        events.extend(reply.events)
    assert round(float(scene.data.time) * 1e9) >= 600_000_000

    expect_reason(
        lambda: controller.renew(context, lease, 1_000_000_000, 100_000_000),
        "LEASE_EXPIRED",
    )
    assert controller.query_command(context, "c1").phase == "STOPPING"
    expect_reason(
        lambda: controller.acquire(
            replace(context, episode_id="episode-2", control_epoch=2),
            "other",
            1_000_000_000,
            100_000_000,
        ),
        "CONTROL_BUSY",
    )
    expect_reason(
        lambda: controller.chunk_anchor(context, lease, 100_000_000),
        "LEASE_EXPIRED",
    )
    expect_reason(
        lambda: controller.reset(context, lease, 7, 100_000_000),
        "LEASE_EXPIRED",
    )

    delivered = controller.poll(100_000_000)
    events.extend(delivered.events)
    stopping = [
        event for event in delivered.events if event.chunk_seq is None and event.phase == "STOPPING"
    ]
    assert len(stopping) == 1
    assert controller.poll(100_000_000).events == ()
    terminal = None
    for _ in range(1500):
        reply = controller.step(100_000_000)
        events.extend(reply.events)
        if reply.status is not None and reply.status.phase in TERMINAL:
            terminal = reply
            break
    assert terminal is not None
    assert terminal.status is not None
    assert terminal.status.phase == "CANCELED"
    own = [event for event in events if event.chunk_seq is None]
    assert [event.event_seq for event in own] == list(range(1, len(own) + 1))
    assert [event.phase for event in own] == [
        "ACCEPTED",
        "STARTED",
        "STOPPING",
        "CANCELED",
    ]


def test_all_public_entries_reject_other_threads_and_measurements_are_real(
    scene, profile, control_config, stream_config
):
    io, controller = controller_for(scene, profile, control_config, stream_config)
    context = context_for(controller)
    lease = controller.acquire(context, "owner", LEASE_NS, 0)
    initial = tuple(profile.initial_positions)
    prepared = motion_for(
        profile,
        control_config,
        initial,
        changed(initial, 9, 0.02),
        800_000_000,
    )
    packet = motion_packet(context, lease, prepared, command_id="c1", command_seq=1)

    calls = [
        lambda: controller.context,
        lambda: controller.phase,
        lambda: controller.submit(packet, 0),
        lambda: controller.queue_chunk(packet, 0),
        lambda: controller.step(0),
        lambda: controller.poll(0),
        lambda: controller.query_command(context, "c1"),
        lambda: controller.query_chunk(context, "c1", 1),
        lambda: controller.chunk_anchor(context, lease, 0),
        lambda: controller.acquire(context, "other", LEASE_NS, 0),
        lambda: controller.reset(context, lease, 0, 0),
    ]
    for call in calls:
        error = run_in_thread(call)
        assert isinstance(error, ContractError), error
        assert error.reason == "WRONG_THREAD"

    controller.submit(packet, 0)
    wall = 0
    reply = None
    for _ in range(80):
        reply = controller.step(wall)
        wall += DT_NS
    assert reply is not None
    assert reply.snapshot.measured_positions == io.measure().positions
    assert reply.snapshot.measured_positions != reply.snapshot.reference_positions
    assert reply.snapshot.tracking_errors == tuple(
        reference - measured
        for reference, measured in zip(
            reply.snapshot.reference_positions,
            reply.snapshot.measured_positions,
            strict=True,
        )
    )
    assert controller.phase in (Phase.RUNNING, Phase.SETTLING)
    assert NativeMotion is motion_module.NativeMotion


def test_new_lease_restarts_sequence_without_losing_old_terminal_history(
    scene, profile, control_config, stream_config
):
    _io, controller = controller_for(scene, profile, control_config, stream_config)
    old_context = context_for(controller)
    old_lease = controller.acquire(old_context, "old-owner", LEASE_NS, 0)
    initial = tuple(profile.initial_positions)
    first = motion_for(profile, control_config, initial, changed(initial, 8, 0.02), 1_000_000_000)
    old_packet = motion_packet(
        old_context, old_lease, first, command_id="old-lease-command", command_seq=1
    )
    controller.submit(old_packet, 0)
    terminal, wall = drive_terminal(controller, 0)
    assert terminal.status.phase == "COMPLETED"
    saved = controller.query_command(old_context, "old-lease-command")
    controller.release(old_context, old_lease, wall)
    new_context = context_for(controller, episode="episode-2", control_epoch=2)
    new_lease = controller.acquire(new_context, "new-owner", LEASE_NS, wall)
    next_initial = terminal.snapshot.reference_positions
    prepared = motion_for(
        profile, control_config, next_initial, changed(next_initial, 8, 0.01), 1_000_000_000
    )
    # A reused id is still a conflict even though the sequence namespace resets.
    conflicting = motion_packet(
        new_context, new_lease, prepared, command_id="old-lease-command", command_seq=1
    )
    expect_reason(lambda: controller.submit(conflicting, wall), "COMMAND_ID_CONFLICT")
    fresh = motion_packet(
        new_context, new_lease, prepared, command_id="new-lease-command", command_seq=1
    )
    assert controller.submit(fresh, wall).status.phase == "ACCEPTED"
    assert controller.query_command(old_context, "old-lease-command") == saved
