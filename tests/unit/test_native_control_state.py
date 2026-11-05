import sys
from dataclasses import replace
from pathlib import Path

import pytest
from robot_contracts.action_chunks import ActionWindow, ChunkAnchor
from robot_contracts.control import ControlCommand
from robot_contracts.models import ContractError, JointSpec, RunContext, Target
from robot_contracts.reference import CubicSegment, ReferencePlan, ReferenceSample

SIM_SRC = Path(__file__).resolve().parents[2] / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.chunk_plan import PreparedChunk  # noqa: E402
from robot_backend_sim.control_state import (  # noqa: E402
    ChunkHistory,
    ChunkRecord,
    CommandHistory,
    CommandRecord,
    ControlEvent,
    DeferredEvents,
    LeaseBook,
    PreparedControlCommand,
)

PROFILE_DIGEST = "profile-digest"
JOINTS = (
    JointSpec("j1", "rad", -1.0, 1.0, 2.0, 0.0),
    JointSpec("j2", "rad", -1.0, 1.0, 2.0, 0.0),
)


def context(**changes) -> RunContext:
    value = RunContext(
        "panda-stack",
        PROFILE_DIGEST,
        "backend-instance",
        "mujoco-sim",
        0,
        episode_id="episode-1",
        control_epoch=1,
    )
    return replace(value, **changes)


def command(**changes) -> ControlCommand:
    value = ControlCommand(context(), "lease-1", "command-1", 1, 1, 5_000_000_000)
    return replace(value, **changes)


def chunk_at(
    position: float,
    *,
    observation_id: int = 1,
    settle_at_end: bool = True,
) -> PreparedChunk:
    positions = (position, position)
    plan = ReferencePlan(
        ("j1", "j2"),
        (
            CubicSegment(100_000_000, positions, (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)),
            CubicSegment(900_000_000, positions, (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)),
        ),
    )
    anchor = ChunkAnchor(
        "stream-1",
        0,
        0,
        ReferenceSample(positions, (0.0, 0.0), (0.0, 0.0)),
        PROFILE_DIGEST,
    )
    window = ActionWindow(
        observation_id,
        0,
        0,
        (
            Target(100_000_000, positions),
            Target(1_000_000_000, positions),
        ),
        2,
    )
    return PreparedChunk(
        anchor=anchor,
        window=window,
        plan=plan,
        joints=JOINTS,
        max_accelerations=(1.0, 1.0),
        final_velocities=(0.0, 0.0),
        settle_at_end=settle_at_end,
    )


def packet(
    prepared: PreparedChunk | None = None,
    *,
    command_value: ControlCommand | None = None,
) -> PreparedControlCommand:
    prepared = chunk_at(0.0) if prepared is None else prepared
    command_value = command() if command_value is None else command_value
    return PreparedControlCommand(command_value, prepared)


def command_record(
    packet_value: PreparedControlCommand,
    *,
    phase: str = "COMPLETED",
) -> CommandRecord:
    return CommandRecord(
        packet=packet_value,
        phase=phase,
        reason="",
        stop_reason=None,
        accepted_sim_ns=0,
        accepted_wall_ns=0,
        latest_snapshot=None,
    )


def chunk_record(
    packet_value: PreparedControlCommand,
    *,
    phase: str = "RETIRED",
) -> ChunkRecord:
    return ChunkRecord(
        packet=packet_value,
        phase=phase,
        reason="HANDOFF",
        accepted_sim_ns=0,
        accepted_wall_ns=0,
    )


def test_prepared_command_hashes_complete_request_not_only_endpoint() -> None:
    first = packet(chunk_at(0.0))
    same = packet(chunk_at(0.0))
    different_position = packet(chunk_at(0.1))
    different_settle = packet(chunk_at(0.0, settle_at_end=False))
    different_observation = packet(
        chunk_at(0.0, observation_id=2),
        command_value=command(observation_id=2),
    )
    different_sequence = packet(command_value=command(command_seq=2))
    assert first.fingerprint == same.fingerprint
    assert first.fingerprint != different_position.fingerprint
    assert first.fingerprint != different_settle.fingerprint
    assert first.fingerprint != different_observation.fingerprint
    assert first.fingerprint != different_sequence.fingerprint
    assert len(first.fingerprint) == 64


def test_chunk_observation_must_match_command() -> None:
    with pytest.raises(ContractError) as caught:
        PreparedControlCommand(command(), chunk_at(0.0, observation_id=2))
    assert caught.value.reason == "INVALID_OBSERVATION"


def test_chunk_profile_must_match_context() -> None:
    prepared = chunk_at(0.0)
    wrong = replace(
        command(),
        context=replace(context(), robot_profile_hash="other"),
    )
    with pytest.raises(ContractError) as caught:
        PreparedControlCommand(wrong, prepared)
    assert caught.value.reason == "PROFILE_MISMATCH"


def test_lease_book_uses_explicit_wall_and_exact_identity() -> None:
    book = LeaseBook()
    lease = book.acquire(context(), "owner-1", 100_000_000, 50_000_000)
    assert lease.expires_at_ns == 150_000_000
    assert book.expire(149_999_999) is None
    renewed = book.renew(lease.lease_id, 200_000_000, 100_000_000)
    assert renewed.expires_at_ns == 300_000_000
    assert book.expire(300_000_000) is not None
    assert book.current is None
    with pytest.raises(ContractError) as caught:
        book.renew(lease.lease_id, 100_000_000, 400_000_000)
    assert caught.value.reason == "LEASE_EXPIRED"


def test_lease_book_rejects_bool_and_overflow_without_changing_lease() -> None:
    book = LeaseBook()
    with pytest.raises(ContractError) as caught:
        book.acquire(context(), "owner", True, 0)
    assert caught.value.reason == "INVALID_LEASE_DURATION"
    lease = book.acquire(context(), "owner", 100_000_000, 0)
    with pytest.raises(ContractError) as caught:
        book.renew(lease.lease_id, 100_000_000, 2**63 - 1)
    assert caught.value.reason == "INVALID_WALL_TIME"
    assert book.current is not None
    with pytest.raises(ContractError):
        book.acquire(context(), "other", 100_000_000, 1)
    assert book.current.lease_id == lease.lease_id


def test_command_history_evicts_oldest_terminal_but_never_active() -> None:
    history = CommandHistory(2)
    first = command_record(packet(command_value=command(command_id="c1")))
    second = command_record(packet(command_value=command(command_id="c2")))
    third = command_record(packet(command_value=command(command_id="c3")))
    history.add(first)
    history.add(second)
    history.add(third)
    assert history.get("c1") is None
    assert history.get("c2") is not None
    assert history.get("c3") is not None

    active_history = CommandHistory(1)
    active = command_record(
        packet(command_value=command(command_id="active")),
        phase="STARTED",
    )
    active_history.add(active)
    assert not active_history.has_capacity()
    with pytest.raises(ContractError) as caught:
        active_history.add(third)
    assert caught.value.reason == "HISTORY_FULL"


def test_chunk_history_evicts_terminal_chunks_and_keeps_active() -> None:
    history = ChunkHistory(2)
    first = chunk_record(packet(command_value=command(command_id="c1")))
    second = chunk_record(packet(command_value=command(command_id="c2")))
    third = chunk_record(packet(command_value=command(command_id="c3")))
    history.add(first)
    history.add(second)
    history.add(third)
    assert history.get("c1", 1) is None
    assert history.get("c3", 1) is not None

    active_history = ChunkHistory(1)
    active = chunk_record(
        packet(command_value=command(command_id="active")),
        phase="ACCEPTED",
    )
    active_history.add(active)
    assert not active_history.has_capacity()
    with pytest.raises(ContractError) as caught:
        active_history.add(third)
    assert caught.value.reason == "CHUNK_HISTORY_FULL"


@pytest.mark.parametrize("bound", [True, 0, -1, 4097, 1.5])
def test_history_bounds_reject_bool_zero_and_oversize(bound) -> None:
    with pytest.raises(ContractError) as caught:
        CommandHistory(bound)
    assert caught.value.reason == "INVALID_HISTORY_BOUND"
    with pytest.raises(ContractError) as caught:
        ChunkHistory(bound)
    assert caught.value.reason == "INVALID_HISTORY_BOUND"


def test_prepared_command_rejects_unknown_prepared_type() -> None:
    with pytest.raises(ContractError) as caught:
        PreparedControlCommand(command(), object())
    assert caught.value.reason == "INVALID_PREPARED_COMMAND"


def event(sequence: int) -> ControlEvent:
    return ControlEvent(
        event_seq=sequence,
        context=context(),
        command_id="command-1",
        command_seq=1,
        observation_id=1,
        chunk_seq=None,
        phase="STOPPING",
        reason="LEASE_EXPIRED",
        sim_time_ns=sequence * 1_000_000,
    )


def test_deferred_events_are_bounded_fifo_and_drain_once() -> None:
    deferred = DeferredEvents(2)
    first, second, third = event(1), event(2), event(3)
    deferred.defer([first])
    deferred.defer([second])
    assert len(deferred) == 2
    with pytest.raises(ContractError) as caught:
        deferred.defer([third])
    assert caught.value.reason == "DEFERRED_EVENTS_FULL"
    assert len(deferred) == 2
    assert deferred.drain() == (first, second)
    assert deferred.drain() == ()
    deferred.defer([third])
    assert deferred.drain() == (third,)


@pytest.mark.parametrize("bound", [True, 0, -1, 257, 1.5])
def test_deferred_events_bound_rejects_bool_zero_and_oversize(bound) -> None:
    with pytest.raises(ContractError) as caught:
        DeferredEvents(bound)
    assert caught.value.reason == "INVALID_DEFERRED_BOUND"
