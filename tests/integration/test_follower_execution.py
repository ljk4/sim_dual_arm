"""Direct no-ROS physical execution of bounded-follower chunks.

The test drives the production ``ControlSession`` / ``MotionController`` loop on
the real MuJoCo scene.  It never substitutes a measured state for an anchor and
never copies reference values into the measurement fields.
"""

from __future__ import annotations

import json
import math
import os
import sys
from dataclasses import dataclass
from dataclasses import replace as _dataclass_replace
from pathlib import Path

import pytest

mujoco = pytest.importorskip("mujoco")
pytest.importorskip("scipy")
np = pytest.importorskip("numpy")

ROOT = Path(__file__).resolve().parents[2]
for source in (
    ROOT / "simulation/mujoco/src",
    ROOT / "shared/common/python/contracts/src",
    ROOT / "tests/unit/fixtures",
):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

from follower_fixtures import generate_prepared_follower  # noqa: E402
from robot_backend_sim.common.config.backend_config import load_backend_config  # noqa: E402
from robot_backend_sim.common.config.follower_config import load_follower_config  # noqa: E402
from robot_backend_sim.common.config.motion_config import load_control_config  # noqa: E402
from robot_backend_sim.common.config.stream_config import StreamConfig  # noqa: E402
from robot_backend_sim.common.control.control import ControlSession  # noqa: E402
from robot_backend_sim.common.control.control_state import PreparedControlCommand  # noqa: E402
from robot_backend_sim.common.motion.motion_controller import MotionController, Phase  # noqa: E402
from robot_backend_sim.common.motion.stream_schedule import next_grid_time  # noqa: E402
from robot_backend_sim.marvin.io import MarvinIO  # noqa: E402
from robot_backend_sim.marvin.stack.scene import build_marvin_stack_scene  # noqa: E402
from robot_backend_sim.marvin.stack.scene_config import (  # noqa: E402
    load_marvin_stack_scene_config,
)
from robot_contracts.action_chunks import ActionWindow  # noqa: E402
from robot_contracts.control import ControlCommand  # noqa: E402
from robot_contracts.models import ContractError, RunContext, Target  # noqa: E402
from robot_contracts.profiles import load_profile  # noqa: E402
from robot_trajectory.follower import (  # noqa: E402
    PreparedFollowerPlan,
    normalize_follower_positions,
)


def replace(value, **changes):
    """Replace fields on a Pydantic contract model or a plain dataclass."""
    if hasattr(value, "model_copy"):
        return value.model_copy(update=changes)
    return _dataclass_replace(value, **changes)


SCENARIO = ROOT / "shared/scenarios/marvin/mujoco/stack"
BACKEND = load_backend_config(SCENARIO / "configs/simulation/backend/backend_stack_teacher.yaml")
DT_NS = 2_000_000
LEASE_NS = 10_000_000_000
FIRST_START_NS = 250_000_000
FINAL_CHUNK_SEQ = 11
TERMINAL = frozenset({"COMPLETED", "CANCELED", "FAILED"})
REPORT_DIR = Path(
    os.environ.get("FOLLOWER_REPORT_DIR", ROOT / "output/marvin/mujoco/stack/debug/follower-execution")
)
PROFILE = load_profile(BACKEND.profile_path, BACKEND.schemas_path)
BASE_POSITIONS = PROFILE.initial_positions


@pytest.fixture(scope="module")
def scene_config():
    return load_marvin_stack_scene_config(BACKEND.scene_path)


@pytest.fixture(scope="module")
def profile():
    return PROFILE


@pytest.fixture(scope="module")
def control_config():
    return load_control_config(BACKEND.control_path)


@pytest.fixture(scope="module")
def follower_config():
    # This regression exercises eleven 5-target handoffs over a 60-point source.
    # Production now uses ten targets; keep this fixture independent of that recipe.
    return replace(load_follower_config(BACKEND.follower_path), target_count=5)


@pytest.fixture(scope="module")
def scene(scene_config):
    return build_marvin_stack_scene(scene_config)


@dataclass(frozen=True)
class TickRecord:
    sim_ns: int
    phase: str
    chunk_seq: int | None
    reference_kind: str
    reference_elapsed_ns: int
    sampled_positions: tuple[float, ...]
    sampled_velocities: tuple[float, ...]
    reference_positions: tuple[float, ...]
    reference_velocities: tuple[float, ...]
    reference_corrections: tuple[float, ...]
    measured_positions: tuple[float, ...]
    measured_velocities: tuple[float, ...]


@dataclass(frozen=True)
class FollowerRun:
    context: RunContext
    command_id: str
    packets: dict[int, PreparedControlCommand]
    plans: dict[int, PreparedFollowerPlan]
    windows: dict[int, ActionWindow]
    ticks: tuple[TickRecord, ...]
    events: tuple[object, ...]
    chunk_statuses: dict[int, object]
    terminal: object
    terminal_status: object


def linear_source(
    amplitude: float = 0.3,
    *,
    duration_ns: int = 3_000_000_000,
) -> tuple[Target, ...]:
    targets = []
    for index in range(60):
        time_ns = (index + 1) * 50_000_000
        values = list(BASE_POSITIONS)
        values[8] += amplitude * min(1.0, time_ns / duration_ns)
        targets.append(Target(time_ns, tuple(values)))
    return tuple(targets)


def cubic_source(amplitude: float = 0.2) -> tuple[Target, ...]:
    targets = []
    for index in range(60):
        time_ns = (index + 1) * 50_000_000
        values = list(BASE_POSITIONS)
        values[8] += amplitude * (time_ns / 3_000_000_000) ** 3
        targets.append(Target(time_ns, tuple(values)))
    return tuple(targets)


def new_controller(
    scene,
    profile,
    control,
    episode: str,
    *,
    max_observation_age_s: float = 5.0,
):
    scene.reset(1234)
    io = MarvinIO(scene, profile)
    controller = ControlSession(
        scene,
        io,
        control,
        stream_config=StreamConfig(0.25, 0.05, max_observation_age_s, 120.0, 3.0, 2.0),
    )
    context = replace(controller.context, episode_id=episode, control_epoch=1)
    lease = controller.acquire(context, "follower-physical", LEASE_NS, 0)
    return controller, io, context, lease


def make_packet(
    controller,
    context,
    lease,
    profile,
    control,
    follower,
    source,
    *,
    wall_ns: int,
    command_id: str,
    start_ns: int | None,
    observation_id: int = 1,
    is_final: bool = False,
) -> tuple[PreparedControlCommand, PreparedFollowerPlan, ActionWindow]:
    anchor = controller.chunk_anchor(context, lease, wall_ns, start_ns)
    window = ActionWindow(
        observation_id,
        0,
        anchor.start_backend_time_ns,
        source,
        follower.target_count,
    )
    plan = generate_prepared_follower(
        anchor,
        window,
        profile,
        control,
        follower,
        is_final=is_final,
        duration_grid_ns=DT_NS,
    )
    command = ControlCommand(
        context,
        lease,
        command_id,
        1,
        observation_id,
        10_000_000_000,
    )
    return PreparedControlCommand(command, plan), plan, window


def record_tick(snapshot) -> TickRecord:
    return TickRecord(
        sim_ns=snapshot.measurement_sim_time_ns,
        phase=str(snapshot.phase),
        chunk_seq=snapshot.chunk_seq,
        reference_kind=snapshot.reference_kind,
        reference_elapsed_ns=snapshot.reference_elapsed_ns,
        sampled_positions=snapshot.sampled_reference_positions,
        sampled_velocities=snapshot.sampled_reference_velocities,
        reference_positions=snapshot.reference_positions,
        reference_velocities=snapshot.reference_velocities,
        reference_corrections=snapshot.reference_boundary_corrections,
        measured_positions=snapshot.measured_positions,
        measured_velocities=snapshot.measured_velocities,
    )


def eval_plan(plan, elapsed_ns: int) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """Independent polynomial reconstruction from the saved cubic coefficients."""
    remaining = elapsed_ns
    segment = plan.segments[-1]
    for candidate in plan.segments:
        if remaining <= candidate.duration_ns:
            segment = candidate
            break
        remaining -= candidate.duration_ns
    local = remaining / 1_000_000_000
    positions = tuple(
        ((c3 * local + c2) * local + c1) * local + c0
        for c0, c1, c2, c3 in zip(
            segment.c0, segment.c1, segment.c2, segment.c3, strict=True
        )
    )
    velocities = tuple(
        (3.0 * c3 * local + 2.0 * c2) * local + c1
        for c1, c2, c3 in zip(segment.c1, segment.c2, segment.c3, strict=True)
    )
    return positions, velocities


def assert_close(actual, expected, *, atol=2e-9) -> None:
    assert len(actual) == len(expected)
    for index, (left, right) in enumerate(zip(actual, expected, strict=True)):
        assert math.isclose(left, right, rel_tol=0.0, abs_tol=atol), (
            index,
            left,
            right,
        )


def step_until_terminal(controller, wall: int, *, max_steps: int = 4000):
    replies = []
    while len(replies) < max_steps:
        reply = controller.step(wall)
        replies.append(reply)
        wall += DT_NS
        if reply.status is not None and reply.status.phase in TERMINAL:
            return reply.status, replies, wall
    raise AssertionError("physical run did not reach a terminal phase")


def run_full_follower_stream(scene, profile, control, follower) -> FollowerRun:
    controller, _io, context, lease = new_controller(
        scene, profile, control, "follower-full"
    )
    source = cubic_source()
    command_id = "cmd-follower-full"
    first, first_plan, first_window = make_packet(
        controller,
        context,
        lease,
        profile,
        control,
        follower,
        source,
        wall_ns=0,
        command_id=command_id,
        start_ns=FIRST_START_NS,
    )
    packets = {1: first}
    plans = {1: first_plan}
    windows = {1: first_window}
    reply = controller.submit(first, 0)
    assert reply.accepted, reply.reason
    ticks = [record_tick(reply.snapshot)]
    events = list(reply.events)
    terminal = None
    wall = 0
    while terminal is None:
        reply = controller.step(wall)
        ticks.append(record_tick(reply.snapshot))
        events.extend(reply.events)
        for event in reply.events:
            if event.phase != "STARTED" or event.chunk_seq is None:
                continue
            seq = event.chunk_seq
            if seq >= FINAL_CHUNK_SEQ:
                continue
            next_seq = seq + 1
            successor, successor_plan, successor_window = make_packet(
                controller,
                context,
                lease,
                profile,
                control,
                follower,
                source,
                wall_ns=wall,
                command_id=command_id,
                start_ns=None,
                is_final=next_seq == FINAL_CHUNK_SEQ,
            )
            queued = controller.queue_chunk(successor, wall)
            assert queued.accepted, queued.reason
            packets[next_seq] = successor
            plans[next_seq] = successor_plan
            windows[next_seq] = successor_window
            events.extend(queued.events)
            active = controller.query_chunk(context, command_id, seq)
            pending = controller.query_chunk(context, command_id, next_seq)
            assert active.phase == "STARTED"
            assert pending.phase == "ACCEPTED"
            assert pending.reason == "PENDING"
        wall += DT_NS
        if reply.status is not None and reply.status.phase in TERMINAL:
            terminal = reply.status
    assert terminal is not None
    assert len(packets) == FINAL_CHUNK_SEQ
    chunk_statuses = {
        seq: controller.query_chunk(context, command_id, seq)
        for seq in range(1, FINAL_CHUNK_SEQ + 1)
    }
    terminal_status = controller.query_command(context, command_id)
    return FollowerRun(
        context=context,
        command_id=command_id,
        packets=packets,
        plans=plans,
        windows=windows,
        ticks=tuple(ticks),
        events=tuple(events),
        chunk_statuses=chunk_statuses,
        terminal=terminal,
        terminal_status=terminal_status,
    )


def write_run_artifact(run: FollowerRun, profile) -> Path:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    chunks = []
    for seq in sorted(run.packets):
        prepared = run.plans[seq]
        chunks.append(
            {
                "seq": seq,
                "fingerprint": run.packets[seq].fingerprint,
                "is_final": prepared.is_final,
                "duration_grid_ns": prepared.duration_grid_ns,
                "document": prepared.to_document(),
            }
        )
    ticks = [
        {
            "sim_time_ns": tick.sim_ns,
            "phase": tick.phase,
            "chunk_seq": tick.chunk_seq,
            "reference_kind": tick.reference_kind,
            "reference_elapsed_ns": tick.reference_elapsed_ns,
            "sampled_positions": list(tick.sampled_positions),
            "sampled_velocities": list(tick.sampled_velocities),
            "reference_positions": list(tick.reference_positions),
            "reference_velocities": list(tick.reference_velocities),
            "reference_corrections": list(tick.reference_corrections),
            "measured_positions": list(tick.measured_positions),
            "measured_velocities": list(tick.measured_velocities),
        }
        for tick in run.ticks
    ]
    document = {
        "kind": "dsh-030-follower-execution-run",
        "seed": 1234,
        "dt_ns": DT_NS,
        "joint_names": list(profile.joint_names),
        "source_targets": [
            {"time_ns": target.time_ns, "positions": list(target.positions)}
            for target in run.windows[1].source_targets
        ],
        "chunks": chunks,
        "ticks": ticks,
        "events": [
            {
                "event_seq": event.event_seq,
                "chunk_seq": event.chunk_seq,
                "phase": event.phase,
                "reason": event.reason,
                "sim_time_ns": event.sim_time_ns,
                "planned_start_sim_ns": getattr(event, "planned_start_sim_ns", None),
                "planned_end_sim_ns": getattr(event, "planned_end_sim_ns", None),
            }
            for event in run.events
        ],
        "terminal": {
            "phase": run.terminal.phase,
            "reason": run.terminal.reason,
            "finished_sim_ns": run.terminal.finished_sim_ns,
            "measured_positions": list(run.terminal.snapshot.measured_positions),
            "measured_velocities": list(run.terminal.snapshot.measured_velocities),
            "reference_positions": list(run.terminal.snapshot.reference_positions),
            "stable_for_ns": run.terminal.snapshot.stable_for_ns,
            "grippers": [
                {
                    "side": completion.side,
                    "mode": completion.mode,
                    "contact_object": completion.contact_object,
                }
                for completion in run.terminal.snapshot.grippers
            ],
        },
    }
    path = REPORT_DIR / "follower_run_summary.json"
    path.write_text(json.dumps(document, allow_nan=False, indent=1) + "\n")
    return path


# --------------------------------------------------------------------------- run


def test_first_window_follower_executes(
    scene, profile, control_config, follower_config
):
    controller, _io, context, lease = new_controller(
        scene, profile, control_config, "follower-first-window"
    )
    source = linear_source()
    packet, prepared, _window = make_packet(
        controller,
        context,
        lease,
        profile,
        control_config,
        follower_config,
        source,
        wall_ns=0,
        command_id="cmd-follower-counterexample",
        start_ns=FIRST_START_NS,
    )
    reply = controller.submit(packet, 0)
    assert reply.accepted, reply.reason
    wall = 0
    moved = False
    for _ in range(200):
        reply = controller.step(wall)
        assert reply.status is not None
        assert reply.status.phase in ("ACCEPTED", "STARTED")
        if reply.snapshot.chunk_seq == 1 and reply.snapshot.reference_kind == "CHUNK":
            moved = True
        wall += DT_NS
    assert moved
    stopped = controller.stop(context, lease, "COUNTEREXAMPLE_DONE", wall)
    assert stopped.accepted
    terminal, replies, _ = step_until_terminal(controller, wall)
    assert terminal.phase == "CANCELED"
    assert terminal.reason == "COUNTEREXAMPLE_DONE"
    assert any(reply.snapshot.reference_kind == "BRAKE" for reply in replies)
    assert prepared.window.source_targets == source


def test_eleven_continuous_follower_chunks_retire_and_final_extension(
    scene, profile, control_config, follower_config
):
    run = run_full_follower_stream(scene, profile, control_config, follower_config)
    assert run.terminal.phase == "COMPLETED"
    assert run.terminal.reason == "COMPLETED"
    assert run.terminal_status.phase == "COMPLETED"
    assert all(
        status.phase == "RETIRED" for status in run.chunk_statuses.values()
    )
    assert run.chunk_statuses[FINAL_CHUNK_SEQ].reason == "FINAL_HOLD"
    assert all(
        run.chunk_statuses[seq].phase == "RETIRED"
        for seq in range(1, FINAL_CHUNK_SEQ)
    )

    final_plan = run.plans[FINAL_CHUNK_SEQ]
    final_window = run.windows[FINAL_CHUNK_SEQ]
    previous_window = run.windows[FINAL_CHUNK_SEQ - 1]
    assert final_plan.is_final is True
    assert final_plan.extension_ns > 0
    assert final_plan.requested_window_ns == 250_000_000
    assert final_plan.plan.duration_ns % DT_NS == 0
    assert final_plan.plan.duration_ns == final_plan.requested_window_ns + final_plan.extension_ns
    assert final_window.selected_indices[-1] == 59
    assert previous_window.selected_indices[-1] == 54
    assert final_window.targets[-1].positions == run.windows[1].source_targets[-1].positions
    assert final_window.targets[-1].time_ns == 250_000_000
    for window in run.windows.values():
        assert len(window.source_targets) == 60
        assert window.observation_time_ns == 0
        assert all(len(target.positions) == 16 for target in window.source_targets)

    retired = [event for event in run.events if event.phase == "RETIRED"]
    assert {event.chunk_seq for event in retired} == set(range(1, FINAL_CHUNK_SEQ + 1))
    assert any(
        event.phase == "RETIRED"
        and event.chunk_seq == FINAL_CHUNK_SEQ
        and event.reason == "FINAL_HOLD"
        for event in retired
    )
    assert (
        sum(
            1
            for event in run.events
            if event.phase == "STARTED" and event.chunk_seq is not None
        )
        == FINAL_CHUNK_SEQ
    )
    overall = [event for event in run.events if event.chunk_seq is None]
    assert [event.phase for event in overall].count("COMPLETED") == 1

    # Every saved coefficient sample, actual applied reference and correction is
    # independently rebuilt from the JSON-serialisable plan document.
    for tick in run.ticks:
        if tick.reference_kind != "CHUNK" or tick.chunk_seq is None:
            continue
        plan = run.plans[tick.chunk_seq]
        assert tick.reference_elapsed_ns <= plan.plan.duration_ns
        expected_positions, expected_velocities = eval_plan(plan.plan, tick.reference_elapsed_ns)
        assert_close(tick.sampled_positions, expected_positions)
        # Snapshot velocity is the reference velocity; the reconstructed value
        # must match it even when a boundary position was normalized.
        assert_close(tick.reference_velocities, expected_velocities)
        normalized = normalize_follower_positions(expected_positions, profile.joints)
        assert_close(tick.reference_positions, normalized.applied_positions, atol=1e-12)
        if normalized.has_correction:
            assert_close(tick.reference_corrections, normalized.corrections, atol=1e-12)
        else:
            # The controller already encodes an unchanged reference as an empty correction tuple.
            assert tick.reference_corrections == ()
        for position, joint in zip(tick.reference_positions, profile.joints, strict=True):
            assert joint.minimum <= position <= joint.maximum
        assert any(
            abs(left - right) > 0.0
            for left, right in zip(tick.reference_positions, tick.measured_positions, strict=True)
        )

    terminal_snapshot = run.terminal.snapshot
    assert terminal_snapshot.stable_for_ns >= 200_000_000
    final_target = final_window.targets[-1].positions
    assert terminal_snapshot.measured_positions[8] == pytest.approx(final_target[8], abs=0.012)
    assert abs(terminal_snapshot.measured_velocities[8]) <= 0.025
    assert terminal_snapshot.reference_positions[8] == pytest.approx(final_target[8], abs=1e-9)
    assert all(
        completion.mode != "NONE" for completion in terminal_snapshot.grippers
    )
    path = write_run_artifact(run, profile)
    assert path.exists()


def test_underrun_really_brakes_with_explicit_reason(
    scene, profile, control_config, follower_config
):
    controller, _io, context, lease = new_controller(
        scene, profile, control_config, "follower-underrun"
    )
    source = linear_source(0.2)
    packet, _plan, _window = make_packet(
        controller,
        context,
        lease,
        profile,
        control_config,
        follower_config,
        source,
        wall_ns=0,
        command_id="cmd-underrun",
        start_ns=FIRST_START_NS,
    )
    assert controller.submit(packet, 0).accepted
    terminal, replies, _wall = step_until_terminal(controller, 0)
    assert terminal.phase == "FAILED"
    assert terminal.reason == "UNDERRUN"
    assert any(reply.snapshot.reference_kind == "BRAKE" for reply in replies)
    assert terminal.snapshot.stable_for_ns >= 200_000_000
    assert all(
        abs(velocity) <= tolerance
        for velocity, tolerance in zip(
            terminal.snapshot.measured_velocities,
            control_config.velocity_tolerances,
            strict=True,
        )
    )


def test_runtime_tracking_violation_brakes_with_tracking_timeout(
    scene, profile, control_config, follower_config
):
    scene.reset(1234)
    io = MarvinIO(scene, profile)
    controller = ControlSession(
        scene,
        io,
        control_config,
        stream_config=StreamConfig(0.25, 0.05, 5.0, 120.0, 3.0, 0.5),
    )
    context = replace(controller.context, episode_id="follower-tracking", control_epoch=1)
    lease = controller.acquire(context, "follower-tracking", LEASE_NS, 0)
    tracked_follower = replace(follower_config, target_count=40)
    source = linear_source()
    first, _plan, _window = make_packet(
        controller,
        context,
        lease,
        profile,
        control_config,
        tracked_follower,
        source,
        wall_ns=0,
        command_id="cmd-tracking",
        start_ns=FIRST_START_NS,
    )
    assert controller.submit(first, 0).accepted
    # Simulate loss of torque on the moving axis while retaining measured
    # MuJoCo feedback and the production tracking limits.
    actuator = scene.indices.robot.arm_actuator_ids["right"][0]
    force_range = scene.model.actuator_forcerange[actuator]
    original_range = force_range.copy()
    try:
        force_range[:] = 0.0
        terminal, replies, _wall = step_until_terminal(controller, 0, max_steps=3000)
    finally:
        force_range[:] = original_range
    assert terminal.phase == "FAILED"
    assert terminal.reason == "TRACKING_TIMEOUT"
    assert any(reply.snapshot.reference_kind == "BRAKE" for reply in replies)
    assert terminal.snapshot.stable_for_ns >= 200_000_000


def test_late_prediction_latches_and_brakes(
    scene, profile, control_config, follower_config
):
    controller, _io, context, lease = new_controller(
        scene, profile, control_config, "follower-late"
    )
    source = linear_source(0.2)
    first, _plan, _window = make_packet(
        controller,
        context,
        lease,
        profile,
        control_config,
        follower_config,
        source,
        wall_ns=0,
        command_id="cmd-late",
        start_ns=FIRST_START_NS,
    )
    assert controller.submit(first, 0).accepted
    wall = 0
    while round(float(scene.data.time) * 1_000_000_000) < 460_000_000:
        controller.step(wall)
        wall += DT_NS
    successor, _plan, _window = make_packet(
        controller,
        context,
        lease,
        profile,
        control_config,
        follower_config,
        source,
        wall_ns=wall,
        command_id="cmd-late",
        start_ns=None,
    )
    queued = controller.queue_chunk(successor, wall)
    assert queued.accepted is False
    assert queued.reason == "CHUNK_LATE"
    assert queued.snapshot.phase in (Phase.STOPPING, Phase.FAILED)
    terminal, replies, _wall = step_until_terminal(controller, wall)
    assert terminal.phase == "FAILED"
    assert terminal.reason == "CHUNK_LATE"
    assert any(reply.snapshot.reference_kind == "BRAKE" for reply in replies)


def test_active_follower_accepts_a_successor_and_stops(
    scene, profile, control_config, follower_config
):
    controller, _io, context, lease = new_controller(
        scene, profile, control_config, "follower-mode-mix"
    )
    source = linear_source(0.01)
    first, _plan, _window = make_packet(
        controller,
        context,
        lease,
        profile,
        control_config,
        follower_config,
        source,
        wall_ns=0,
        command_id="cmd-mode-mix",
        start_ns=FIRST_START_NS,
    )
    assert controller.submit(first, 0).accepted
    wall = 0
    while round(float(scene.data.time) * 1_000_000_000) < FIRST_START_NS:
        controller.step(wall)
        wall += DT_NS
    follower_packet, _plan, _window = make_packet(
        controller,
        context,
        lease,
        profile,
        control_config,
        follower_config,
        source,
        wall_ns=wall,
        command_id="cmd-mode-mix",
        start_ns=None,
    )
    assert controller.queue_chunk(follower_packet, wall).accepted
    stopped = controller.stop(context, lease, "MODE_MIX_DONE", wall)
    assert stopped.accepted
    terminal, _replies, _wall = step_until_terminal(controller, wall)
    assert terminal.phase == "CANCELED"
    assert terminal.reason == "MODE_MIX_DONE"


def test_same_fingerprint_retry_and_modified_retry_conflicts(
    scene, profile, control_config, follower_config
):
    controller, _io, context, lease = new_controller(
        scene, profile, control_config, "follower-retry"
    )
    source = linear_source(0.2)
    first, _plan, _window = make_packet(
        controller,
        context,
        lease,
        profile,
        control_config,
        follower_config,
        source,
        wall_ns=0,
        command_id="cmd-retry",
        start_ns=FIRST_START_NS,
    )
    changed, _plan, _window = make_packet(
        controller,
        context,
        lease,
        profile,
        control_config,
        follower_config,
        linear_source(0.25),
        wall_ns=0,
        command_id="cmd-retry",
        start_ns=FIRST_START_NS,
    )
    accepted = controller.submit(first, 0)
    assert accepted.accepted
    motion_seq = accepted.status.motion_seq
    retried = controller.submit(first, 0)
    assert retried.accepted
    assert retried.reason == "ACCEPTED"
    assert retried.status.motion_seq == motion_seq

    with pytest.raises(ContractError) as caught:
        controller.submit(changed, 0)
    assert caught.value.reason == "COMMAND_ID_CONFLICT"

    wall = 0
    while round(float(scene.data.time) * 1_000_000_000) < FIRST_START_NS:
        controller.step(wall)
        wall += DT_NS
    successor, _plan, _window = make_packet(
        controller,
        context,
        lease,
        profile,
        control_config,
        follower_config,
        source,
        wall_ns=wall,
        command_id="cmd-retry",
        start_ns=None,
    )
    modified_successor, _plan, _window = make_packet(
        controller,
        context,
        lease,
        profile,
        control_config,
        follower_config,
        linear_source(0.21),
        wall_ns=wall,
        command_id="cmd-retry",
        start_ns=None,
    )
    assert controller.queue_chunk(successor, wall).accepted
    retried_successor = controller.queue_chunk(successor, wall)
    assert retried_successor.accepted
    with pytest.raises(ContractError) as caught:
        controller.queue_chunk(modified_successor, wall)
    assert caught.value.reason == "CHUNK_ID_CONFLICT"
    controller.stop(context, lease, "RETRY_DONE", wall)
    terminal, _replies, _wall = step_until_terminal(controller, wall)
    assert terminal.phase == "CANCELED"


def test_wrong_context_lease_expiry_reset_and_new_stream_mode(
    scene, profile, control_config, follower_config
):
    controller, _io, context, lease = new_controller(
        scene, profile, control_config, "follower-context"
    )
    source = linear_source(0.2)
    first, _plan, _window = make_packet(
        controller,
        context,
        lease,
        profile,
        control_config,
        follower_config,
        source,
        wall_ns=0,
        command_id="cmd-context",
        start_ns=FIRST_START_NS,
    )
    assert controller.submit(first, 0).accepted
    wrong_context = replace(context, clock_epoch=context.clock_epoch + 1)
    wrong_anchor = controller.chunk_anchor(context, lease, 0)
    wrong_window = ActionWindow(
        1,
        0,
        wrong_anchor.start_backend_time_ns,
        source,
        follower_config.target_count,
    )
    wrong_plan = generate_prepared_follower(
        wrong_anchor,
        wrong_window,
        profile,
        control_config,
        follower_config,
        is_final=False,
        duration_grid_ns=DT_NS,
    )
    wrong_packet = PreparedControlCommand(
        ControlCommand(wrong_context, lease, "cmd-context", 1, 1, 10_000_000_000),
        wrong_plan,
    )
    with pytest.raises(ContractError) as caught:
        controller.queue_chunk(wrong_packet, 0)
    assert caught.value.reason == "STALE_CONTEXT"
    with pytest.raises(ContractError) as caught:
        controller.reset(context, lease, 42, 0)
    assert caught.value.reason == "RESET_NOT_ALLOWED"

    stopped = controller.stop(context, lease, "CONTEXT_DONE", 0)
    assert stopped.accepted
    terminal, _replies, wall = step_until_terminal(controller, 0)
    assert terminal.phase == "CANCELED"
    new_context = controller.reset(context, lease, 42, wall)
    assert new_context.clock_epoch == context.clock_epoch + 1
    with pytest.raises(ContractError) as caught:
        controller.query_command(context, "cmd-context")
    assert caught.value.reason == "STALE_CONTEXT"


def test_short_lease_expiry_brakes_with_lease_reason(
    scene, profile, control_config, follower_config
):
    scene.reset(1234)
    io = MarvinIO(scene, profile)
    controller = ControlSession(
        scene,
        io,
        control_config,
        stream_config=StreamConfig(0.25, 0.05, 5.0, 120.0, 3.0, 2.0),
    )
    context = replace(controller.context, episode_id="follower-lease", control_epoch=1)
    lease = controller.acquire(context, "follower-lease", 300_000_000, 0)
    source = linear_source(0.2)
    first, _plan, _window = make_packet(
        controller,
        context,
        lease,
        profile,
        control_config,
        follower_config,
        source,
        wall_ns=0,
        command_id="cmd-lease",
        start_ns=FIRST_START_NS,
    )
    assert controller.submit(first, 0).accepted
    wall = 0
    expired_reply = None
    while wall < 400_000_000:
        reply = controller.step(wall)
        if wall >= 300_000_000 and expired_reply is None:
            expired_reply = reply
        wall += DT_NS
    assert expired_reply is not None
    assert expired_reply.snapshot.phase in (Phase.STOPPING, Phase.FAILED)
    terminal, replies, _wall = step_until_terminal(controller, wall)
    assert terminal.phase == "CANCELED"
    assert terminal.reason == "LEASE_EXPIRED"
    assert any(reply.snapshot.reference_kind == "BRAKE" for reply in replies)


def test_follower_numeric_boundary_is_explicitly_normalized_not_clipped(
    scene, profile, control_config, follower_config
):
    scene.reset(1234)
    io = MarvinIO(scene, profile)
    raw_closed = list(profile.initial_positions)
    raw_closed[7] = profile.joints[7].minimum - 2.849e-11
    with pytest.raises(ContractError) as caught:
        io.apply_targets(raw_closed)
    assert caught.value.reason == "JOINT_LIMIT"
    closed = normalize_follower_positions(tuple(raw_closed), profile.joints)
    io.apply_targets(closed.applied_positions)
    assert closed.corrections[7] == pytest.approx(-2.849e-11)

    raw_open = list(profile.initial_positions)
    raw_open[15] = profile.joints[15].maximum + 2.849e-11
    with pytest.raises(ContractError) as caught:
        io.apply_targets(raw_open)
    assert caught.value.reason == "JOINT_LIMIT"
    opened = normalize_follower_positions(tuple(raw_open), profile.joints)
    io.apply_targets(opened.applied_positions)
    assert opened.corrections[15] == pytest.approx(2.849e-11)

    # A real follower plan whose raw endpoint is inside the hard range still
    # goes through the same shared correction function at every application.
    source = linear_source(0.2)
    controller, _io, context, lease = new_controller(
        scene, profile, control_config, "follower-boundary"
    )
    packet, prepared, _window = make_packet(
        controller,
        context,
        lease,
        profile,
        control_config,
        follower_config,
        source,
        wall_ns=0,
        command_id="cmd-boundary",
        start_ns=FIRST_START_NS,
    )
    normalized_endpoint = normalize_follower_positions(
        prepared.actual_terminal_positions, profile.joints
    )
    for position, joint in zip(
        normalized_endpoint.applied_positions, profile.joints, strict=True
    ):
        assert joint.minimum <= position <= joint.maximum
    assert max(abs(value) for value in normalized_endpoint.corrections) <= 1e-9
    assert controller.submit(packet, 0).accepted
    controller.stop(context, lease, "BOUNDARY_DONE", 0)
    step_until_terminal(controller, 0)


def test_raw_out_of_bounds_and_excess_deviation_are_rejected(
    scene, profile, control_config, follower_config
):
    controller, _io, context, lease = new_controller(
        scene, profile, control_config, "follower-invalid"
    )
    out_of_bounds = list(profile.initial_positions)
    out_of_bounds[0] = 99.0
    source_bad = list(linear_source(0.2))
    source_bad[0] = Target(source_bad[0].time_ns, tuple(out_of_bounds))
    anchor = controller.chunk_anchor(context, lease, 0, FIRST_START_NS)
    window_bad = ActionWindow(
        1,
        0,
        anchor.start_backend_time_ns,
        tuple(source_bad),
        follower_config.target_count,
    )
    with pytest.raises(ContractError) as caught:
        generate_prepared_follower(
            anchor,
            window_bad,
            profile,
            control_config,
            follower_config,
            is_final=False,
            duration_grid_ns=DT_NS,
        )
    assert caught.value.reason == "JOINT_LIMIT"

    fast = []
    for index in range(60):
        values = list(profile.initial_positions)
        values[0] += min(1.0, (index + 1) * 0.05 / 0.25)
        fast.append(Target((index + 1) * 50_000_000, tuple(values)))
    window_fast = ActionWindow(
        1,
        0,
        anchor.start_backend_time_ns,
        tuple(fast),
        follower_config.target_count,
    )
    with pytest.raises(ContractError) as caught:
        generate_prepared_follower(
            anchor,
            window_fast,
            profile,
            control_config,
            follower_config,
            is_final=False,
            duration_grid_ns=DT_NS,
        )
    assert caught.value.reason == "FOLLOWER_DEVIATION_EXCEEDED"
    assert controller.current_status() is None


def test_follower_final_contact_keeps_raw_closing_target_and_real_opening(
    scene, profile, control_config, follower_config
):
    scene.reset(1234)
    install_contact_source(scene)
    io = MarvinIO(scene, profile)
    closed_width = profile.joints[15].minimum
    close = (*profile.initial_positions[:15], closed_width)
    source_body = scene.model.body(scene.config.task.source.name).id
    # Support the floating cube while the fingers close, then let the physical
    # grasp support it for the entire follower stream.
    gravity_compensation = scene.model.body_gravcomp[source_body]
    scene.model.body_gravcomp[source_body] = 1.0
    try:
        for _ in range(500):
            io.apply_targets(close)
            mujoco.mj_step(scene.model, scene.data)
            mujoco.mj_forward(scene.model, scene.data)
    finally:
        scene.model.body_gravcomp[source_body] = gravity_compensation
    measured_contact = io.measure()
    measured_width = measured_contact.positions[15]
    assert closed_width + control_config.position_tolerances[15] < measured_width
    assert measured_width < profile.joints[15].maximum

    stream_config = StreamConfig(0.25, 0.05, 5.0, 120.0, 3.0, 2.0)
    motion = MotionController(
        scene,
        io,
        control_config,
        initial_reference=close,
        stream_config=stream_config,
    )
    now_ns = round(float(scene.data.time) * 1_000_000_000)
    start_ns = next_grid_time(now_ns, stream_config.initial_lead_ns, DT_NS)
    anchor = motion.chunk_anchor(start_ns)
    source = tuple(Target(index * 50_000_000, close) for index in range(1, 61))
    window = ActionWindow(
        1,
        0,
        anchor.start_backend_time_ns,
        source,
        follower_config.target_count,
    )
    prepared = generate_prepared_follower(
        anchor,
        window,
        profile,
        control_config,
        follower_config,
        is_final=True,
        duration_grid_ns=DT_NS,
    )
    assert window.targets[-1].positions[15] == closed_width
    assert prepared.actual_terminal_positions[15] == pytest.approx(closed_width, abs=1e-9)
    motion.start_stream(prepared, 0)
    wall = 0
    snapshot = motion.step(wall)
    while snapshot.phase not in (Phase.COMPLETED, Phase.STOPPED, Phase.FAILED):
        wall += DT_NS
        snapshot = motion.step(wall)
    assert motion.result is not None
    assert motion.result.phase is Phase.COMPLETED
    assert snapshot.phase is Phase.COMPLETED
    right = next(item for item in snapshot.grippers if item.side == "right")
    assert right.mode == "CONTACT"
    assert right.contact_object == "source"
    assert right.final_reference_m == closed_width
    assert snapshot.measured_positions[15] == pytest.approx(measured_width, abs=0.002)
    assert snapshot.reference_positions[15] == pytest.approx(closed_width, abs=1e-9)
    assert right.target_error_m < -control_config.position_tolerances[15]
    for finger in right.fingers:
        assert finger.force_for("source") >= control_config.contact_force_min_n
        assert finger.joint_speed_m_s <= control_config.finger_velocity_max_m_s


def install_contact_source(scene) -> None:
    """Place one source object between the right fingers at the home TCP."""
    model = scene.model
    data = scene.data
    mujoco.mj_forward(model, data)
    tcp = int(scene.indices.robot.tcp_site_ids["right"])
    position = np.asarray(data.site_xpos[tcp], dtype=float)
    matrix = np.asarray(data.site_xmat[tcp], dtype=float)
    quaternion = np.zeros(4, dtype=float)
    mujoco.mju_mat2Quat(quaternion, matrix)
    address = int(scene.indices.environment.object_free_joint_qpos["source"])
    data.qpos[address : address + 7] = (*position, *quaternion)
    free_dof = int(model.joint("source_free").dofadr[0])
    data.qvel[free_dof : free_dof + 6] = 0.0
    mujoco.mj_forward(model, data)
