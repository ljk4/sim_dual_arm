"""Unit evidence for the follower backend protocol adapter."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest
from robot_contracts.action_chunks import ChunkAnchor
from robot_contracts.control import ControlCommand
from robot_contracts.models import ContractError, RunContext, Target
from robot_contracts.profiles import load_profile
from robot_contracts.stream_request import (
    REQUEST_MOTION,
    REQUEST_STREAM_FIRST,
    StreamRequest,
    Trajectory,
)
from robot_contracts.trajectory.spline import ReferenceSample
from robot_trajectory.follower_reference import validate

ROOT = Path(__file__).resolve().parents[2]
PROFILES = ROOT / "shared/scenarios/marvin_sim/stack/profiles"
for source in (ROOT / "simulation/mujoco/src", ROOT / "shared/common/python/contracts/src"):
    if str(source) not in sys.path:
        sys.path.insert(0, str(source))

from robot_backend_sim.common.config.follower_config import load_follower_config  # noqa: E402
from robot_backend_sim.common.config.motion_config import load_control_config  # noqa: E402
from robot_backend_sim.common.control.control_state import ChunkStatus, ControlStatus  # noqa: E402
from robot_backend_sim.common.follower.follower_plan import RUCKIG_ALGORITHM_VERSION  # noqa: E402
from robot_backend_sim.common.follower.follower_protocol import (  # noqa: E402
    follower_reference,
    follower_reference_from_chunk_status,
    follower_reference_from_status,
    prepare_follower_request,
    validation_limits,
)

ruckig = pytest.importorskip("ruckig")

PROFILE = load_profile(
    PROFILES / "robots/marvin_stack_v1/profile.json",
    ROOT / "shared/common/interfaces/schemas/v1",
)
CONTROL = load_control_config(PROFILES / "control/marvin_control_v1.yaml")
FOLLOWER = load_follower_config(PROFILES / "control/marvin_follower_v1.yaml")
BASE = PROFILE.initial_positions
COUNT = len(BASE)
GRID_NS = 2_000_000
FIRST_START_NS = 250_000_000


def context() -> RunContext:
    return RunContext(
        PROFILE.robot_id,
        PROFILE.digest,
        "backend-instance",
        "mujoco-sim",
        0,
        episode_id="episode-follower",
        control_epoch=1,
    )


def source_targets() -> tuple[Target, ...]:
    target = list(BASE)
    target[0] += 0.2
    targets = []
    for index in range(60):
        seconds = (index + 1) * 0.05
        fraction = min(1.0, seconds / 3.0)
        smooth = 3.0 * fraction * fraction - 2.0 * fraction * fraction * fraction
        positions = list(BASE)
        positions[0] = BASE[0] + (target[0] - BASE[0]) * smooth
        targets.append(Target(round(seconds * 1_000_000_000), tuple(positions)))
    return tuple(targets)


def anchor() -> ChunkAnchor:
    return ChunkAnchor(
        "follower-unit-stream",
        0,
        FIRST_START_NS,
        ReferenceSample(BASE, (0.0,) * COUNT, (0.0,) * COUNT),
        PROFILE.digest,
    )


def request(*, kind: int = REQUEST_STREAM_FIRST, final: bool = False) -> StreamRequest:
    return StreamRequest(
        trajectory=Trajectory(
            context(),
            "lease-1",
            "command-follower",
            1,
            1,
            10_000_000_000,
            PROFILE.joint_names,
            source_targets(),
        ),
        kind=kind,
        anchor=anchor(),
        observation_time_ns=0,
        target_count=FOLLOWER.target_count,
        settle_at_end=final,
    )


def command() -> ControlCommand:
    value = request().command
    return value


def test_follower_protocol_compiles_a_distinct_packet():
    raw = request()
    packet = prepare_follower_request(
        raw,
        PROFILE,
        CONTROL,
        FOLLOWER,
        duration_grid_ns=GRID_NS,
    )
    assert packet.kind == "FOLLOWER"
    assert packet.request == raw
    assert packet.prepared.is_final is False
    assert packet.prepared.duration_grid_ns == GRID_NS
    assert packet.command == raw.command
    assert packet.prepared.algorithm_version == RUCKIG_ALGORITHM_VERSION


def test_ruckig_curve_duration_and_sampling_semantics_are_preserved():
    raw = request()
    packet = prepare_follower_request(
        raw, PROFILE, CONTROL, FOLLOWER, duration_grid_ns=GRID_NS
    )
    prepared = packet.prepared
    assert prepared.curve_duration_ns >= prepared.plan.duration_ns
    assert prepared.curve_duration_ns % GRID_NS == 0
    assert prepared.plan.duration_ns == prepared.requested_window_ns

    chunk = ChunkStatus(
        context=raw.command.context,
        command_id=raw.command.command_id,
        command_seq=raw.command.command_seq,
        chunk_seq=1,
        observation_id=raw.command.observation_id,
        phase="ACCEPTED",
        reason="PENDING",
        accepted_sim_ns=100_000_000,
        started_sim_ns=None,
        finished_sim_ns=None,
        accepted_wall_ns=1_900_000_000,
        started_wall_ns=None,
        finished_wall_ns=None,
        snapshot=None,
    )
    record = follower_reference_from_chunk_status(raw, packet, chunk, motion_seq=3)
    assert record.curve_duration_ns == prepared.curve_duration_ns
    rebuilt = validate(
        record,
        PROFILE,
        validation_limits(CONTROL, FOLLOWER, duration_grid_ns=GRID_NS),
    )
    assert rebuilt.plan == record.plan
    assert rebuilt.actual_terminal_positions == record.actual_terminal_positions
    assert rebuilt.flags == record.flags


def test_retry_returns_the_original_acceptance_times_and_digest():
    raw = request()
    packet = prepare_follower_request(
        raw, PROFILE, CONTROL, FOLLOWER, duration_grid_ns=GRID_NS
    )
    chunk = ChunkStatus(
        context=raw.command.context,
        command_id=raw.command.command_id,
        command_seq=raw.command.command_seq,
        chunk_seq=1,
        observation_id=raw.command.observation_id,
        phase="ACCEPTED",
        reason="PENDING",
        accepted_sim_ns=100_000_000,
        started_sim_ns=None,
        finished_sim_ns=None,
        accepted_wall_ns=1_900_000_000,
        started_wall_ns=None,
        finished_wall_ns=None,
        snapshot=None,
    )
    first = follower_reference_from_chunk_status(
        raw, packet, chunk, motion_seq=3
    )
    second = follower_reference_from_chunk_status(
        raw, packet, chunk, motion_seq=3
    )
    assert first == second
    assert first.accepted_sim_time_ns == 100_000_000
    assert first.accepted_wall_time_ns == 1_900_000_000
    assert first.motion_seq == 3
    assert first.reference_digest == second.reference_digest
    rebuilt = validate(
        first,
        PROFILE,
        validation_limits(CONTROL, FOLLOWER, duration_grid_ns=GRID_NS),
    )
    assert rebuilt.plan == first.plan

    status = ControlStatus(
        context=raw.command.context,
        command_id=raw.command.command_id,
        command_seq=raw.command.command_seq,
        observation_id=raw.command.observation_id,
        phase="ACCEPTED",
        reason="",
        stop_reason=None,
        motion_seq=3,
        stream_id=raw.anchor.stream_id,  # type: ignore[union-attr]
        chunk_seq=1,
        accepted_sim_ns=100_000_000,
        started_sim_ns=None,
        finished_sim_ns=None,
        accepted_wall_ns=1_900_000_000,
        started_wall_ns=None,
        finished_wall_ns=None,
        snapshot=None,
    )
    from_status = follower_reference_from_status(raw, packet, status)
    assert from_status.reference_digest == first.reference_digest


def test_follower_protocol_rejects_motion_and_mismatched_final_flag():
    motion = StreamRequest(
        trajectory=Trajectory(
            context(),
            "lease-1",
            "command-follower",
            1,
            1,
            10_000_000_000,
            PROFILE.joint_names,
            source_targets(),
        ),
        kind=REQUEST_MOTION,
    )
    with pytest.raises(ContractError) as caught:
        prepare_follower_request(motion, PROFILE, CONTROL, FOLLOWER)
    assert caught.value.reason == "INVALID_FOLLOWER_REQUEST"

    with pytest.raises(ContractError) as caught:
        prepare_follower_request(
            request(), PROFILE, CONTROL, FOLLOWER, is_final=True
        )
    assert caught.value.reason == "INVALID_FOLLOWER_REQUEST"


def test_successor_fingerprint_is_separate_from_strict_chunks():
    first = request()
    packet = prepare_follower_request(first, PROFILE, CONTROL, FOLLOWER, duration_grid_ns=GRID_NS)
    successor_request = StreamRequest(
        trajectory=Trajectory(
            context(),
            "lease-1",
            first.trajectory.command_id,
            1,
            2,
            10_000_000_000,
            PROFILE.joint_names,
            source_targets(),
        ),
        kind=3,
        anchor=ChunkAnchor(
            "follower-unit-stream",
            1,
            FIRST_START_NS + 250_000_000,
            ReferenceSample(
                packet.prepared.actual_terminal_positions,  # type: ignore[union-attr]
                packet.prepared.actual_terminal_velocities,  # type: ignore[union-attr]
                (0.0,) * COUNT,
            ),
            PROFILE.digest,
        ),
        observation_time_ns=250_000_000,
        target_count=FOLLOWER.target_count,
        settle_at_end=False,
    )
    successor = prepare_follower_request(
        successor_request, PROFILE, CONTROL, FOLLOWER, duration_grid_ns=GRID_NS
    )
    assert successor.kind == "FOLLOWER"
    assert successor.prepared.seq == 2  # type: ignore[union-attr]
    assert successor.fingerprint != packet.fingerprint


def test_real_final_counterexample_checks_executed_tail_and_target_count():
    """The accepted DSH-031 counterexample: 0.4 s plan must fail a 0.3 s tail."""
    from model_helpers import replace

    raw_targets = []
    right_joint1 = 8
    for index in range(1, 61):
        positions = list(BASE)
        positions[right_joint1] = BASE[right_joint1] + 0.04
        raw_targets.append(Target(index * 50_000_000, tuple(positions)))
    final_anchor = ChunkAnchor(
        "follower-counterexample",
        0,
        0,
        ReferenceSample(BASE, (0.0,) * COUNT, (0.0,) * COUNT),
        PROFILE.digest,
    )
    raw = StreamRequest(
        trajectory=Trajectory(
            context(),
            "lease-1",
            "command-follower-counterexample",
            1,
            1,
            10_000_000_000,
            PROFILE.joint_names,
            tuple(raw_targets),
        ),
        kind=REQUEST_STREAM_FIRST,
        anchor=final_anchor,
        observation_time_ns=0,
        target_count=FOLLOWER.target_count,
        settle_at_end=True,
    )
    packet = prepare_follower_request(raw, PROFILE, CONTROL, FOLLOWER, duration_grid_ns=GRID_NS)
    prepared = packet.prepared
    assert prepared.is_final is True
    assert prepared.requested_window_ns == FOLLOWER.target_count * 50_000_000
    assert prepared.duration_ns > 300_000_000
    record = follower_reference(
        raw,
        packet,
        motion_seq=1,
        accepted_sim_time_ns=0,
        accepted_wall_time_ns=0,
    )
    limits = validation_limits(CONTROL, FOLLOWER, duration_grid_ns=GRID_NS)
    assert validate(record, PROFILE, limits).plan == record.plan

    short_tail = replace(FOLLOWER, max_terminal_tail_duration_s=0.3)
    with pytest.raises(ContractError) as caught:
        validate(
            record,
            PROFILE,
            validation_limits(CONTROL, short_tail, duration_grid_ns=GRID_NS),
        )
    assert caught.value.reason == "FOLLOWER_HORIZON_EXCEEDED"

    six_targets = replace(FOLLOWER, target_count=6)
    with pytest.raises(ContractError) as caught:
        validate(
            record,
            PROFILE,
            validation_limits(CONTROL, six_targets, duration_grid_ns=GRID_NS),
        )
    assert caught.value.reason == "FOLLOWER_LIMIT_MISMATCH"
