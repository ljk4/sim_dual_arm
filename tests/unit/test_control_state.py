import sys
from pathlib import Path

import pytest
from model_helpers import replace
from robot_contracts.action_chunks import ActionWindow, ChunkAnchor
from robot_contracts.control import ControlCommand
from robot_contracts.models import ContractError, RunContext, Target
from robot_contracts.profiles import load_profile
from robot_contracts.trajectory.spline import ReferenceSample
from robot_trajectory.follower import PreparedFollowerPlan

ROOT = Path(__file__).resolve().parents[2]
PROFILES = ROOT / "shared/scenarios/marvin_sim/stack/profiles"
SIM_SRC = ROOT / "simulation/mujoco/src"
if str(SIM_SRC) not in sys.path:
    sys.path.insert(0, str(SIM_SRC))

from robot_backend_sim.common.config.follower_config import load_follower_config  # noqa: E402
from robot_backend_sim.common.config.motion_config import load_control_config  # noqa: E402
from robot_backend_sim.common.control.control_state import (  # noqa: E402
    ChunkHistory,
    ChunkRecord,
    CommandHistory,
    CommandRecord,
    ControlEvent,
    DeferredEvents,
    LeaseBook,
    PreparedControlCommand,
)
from robot_backend_sim.common.follower.follower_plan import prepare_follower_plan  # noqa: E402
from scenario_fixtures import MARVIN_SHARED_PROFILES  # noqa: E402

ruckig = pytest.importorskip("ruckig")

PROFILE = load_profile(
    PROFILES / "robots/marvin_stack_v1/profile.json",
    ROOT / "shared/common/interfaces/schemas/v1",
)
CONTROL = load_control_config(MARVIN_SHARED_PROFILES / "control/marvin_control_v1.yaml")
FOLLOWER = load_follower_config(MARVIN_SHARED_PROFILES / "control/marvin_follower_v1.yaml")
PROFILE_DIGEST = PROFILE.digest
BASE = PROFILE.initial_positions
JOINT_COUNT = len(BASE)


def context(**changes) -> RunContext:
    value = RunContext(
        PROFILE.robot_id,
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
    displacement: float,
    *,
    observation_id: int = 1,
    settle_at_end: bool = True,
) -> PreparedFollowerPlan:
    targets = []
    for index in range(FOLLOWER.target_count):
        values = list(BASE)
        values[0] += displacement * (index + 1) / FOLLOWER.target_count
        targets.append(Target((index + 1) * 50_000_000, tuple(values)))
    window = ActionWindow(
        observation_id,
        0,
        0,
        tuple(targets),
        FOLLOWER.target_count,
    )
    anchor = ChunkAnchor(
        "stream-1",
        0,
        0,
        ReferenceSample(BASE, (0.0,) * JOINT_COUNT, (0.0,) * JOINT_COUNT),
        PROFILE_DIGEST,
    )
    return prepare_follower_plan(
        anchor,
        window,
        PROFILE,
        CONTROL,
        FOLLOWER,
        is_final=settle_at_end,
        duration_grid_ns=2_000_000,
    )


def packet(
    prepared: PreparedFollowerPlan | None = None,
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
