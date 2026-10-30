from dataclasses import replace

import pytest
from conftest import command
from robot_backend_core.engine import BackendEngine
from robot_contracts.models import ContractError, Target


def test_trajectory_interpolates_then_completes(controlled):
    engine, context, lease = controlled
    engine.submit(command(context, lease), 0, 0)
    engine.tick(500_000_000)
    assert engine.positions[0] == pytest.approx(0.1)
    engine.tick(1_000_000_000)
    assert engine.positions[0] == pytest.approx(0.2)
    assert engine.execution_status("command-1") == ("COMPLETED", "")
    assert [event.phase for event in engine.drain_events()] == [
        "ACCEPTED",
        "STARTED",
        "COMPLETED",
    ]


def test_duplicate_returns_original_without_reexecution(controlled):
    engine, context, lease = controlled
    first = engine.submit(command(context, lease), 0, 0)
    engine.tick(1_000_000_001)
    engine.drain_events()
    duplicate = engine.submit(command(context, lease), 900_000_000, 1_100_000_000)
    assert duplicate is first
    assert not engine.drain_events()


def test_same_id_with_changed_payload_is_rejected(controlled):
    engine, context, lease = controlled
    engine.submit(command(context, lease), 0, 0)
    altered = command(context, lease, targets=(Target(1_000_000_000, (0.0,) * 6),))
    with pytest.raises(ContractError, match="COMMAND_ID_CONFLICT"):
        engine.submit(altered, 0, 1)


@pytest.mark.parametrize(
    "field,value",
    [
        ("backend_instance_id", "old-instance"),
        ("clock_epoch", 1),
        ("episode_id", "old-episode"),
        ("control_epoch", 2),
        ("robot_profile_hash", "wrong"),
        ("time_domain_id", "wrong"),
    ],
)
def test_stale_context_rejected(controlled, field, value):
    engine, context, lease = controlled
    with pytest.raises(ContractError, match="STALE_CONTEXT"):
        engine.submit(command(replace(context, **{field: value}), lease), 0, 0)


@pytest.mark.parametrize(
    "targets,reason",
    [
        ((Target(0, (0.0,) * 6),), "INVALID_TRAJECTORY"),
        ((Target(1_000_000_000, (0.0,) * 5),), "INVALID_TRAJECTORY"),
        ((Target(1_000_000_000, (float("nan"),) + (0.0,) * 5),), "INVALID_TRAJECTORY"),
        ((Target(1_000_000_000, (2.0,) + (0.0,) * 5),), "JOINT_LIMIT"),
        ((Target(1_000_000, (0.2,) + (0.0,) * 5),), "VELOCITY_LIMIT"),
        ((Target(4_000_000_000, (0.0,) * 6),), "INVALID_TRAJECTORY"),
        ((), "INVALID_TRAJECTORY"),
    ],
)
def test_invalid_trajectory_rejected_without_side_effects(controlled, targets, reason):
    engine, context, lease = controlled
    with pytest.raises(ContractError, match=reason):
        engine.submit(command(context, lease, targets=targets), 0, 0)
    assert engine.positions == engine.profile.initial_positions
    assert not engine.drain_events()


def test_expired_command_rejected(controlled):
    engine, context, lease = controlled
    with pytest.raises(ContractError, match="COMMAND_EXPIRED"):
        engine.submit(command(context, lease), 500_000_001, 0)


def test_expired_lease_holds_position(controlled):
    engine, context, lease = controlled
    engine.submit(command(context, lease), 0, 0)
    engine.tick(100_000_000)
    position = engine.positions
    engine.tick(2_000_000_001)
    assert engine.positions == position
    assert engine.execution_status("command-1") == ("CANCELED", "LEASE_EXPIRED")
    with pytest.raises(ContractError, match="LEASE_EXPIRED"):
        engine.renew(context, lease, 2_000_000_000, 2_000_000_002)


def test_renewal_preserves_control(controlled):
    engine, context, lease = controlled
    engine.renew(context, lease, 2_000_000_000, 1_500_000_000)
    engine.expire(2_500_000_000)
    engine.submit(command(context, lease), 0, 2_500_000_000)


def test_stop_cancels_and_prevents_remaining_motion(controlled):
    engine, context, lease = controlled
    engine.submit(command(context, lease), 0, 0)
    engine.tick(100_000_000)
    engine.stop(context, lease, 100_000_000, "TEST_STOP")
    position = engine.positions
    engine.tick(900_000_000)
    assert engine.positions == position
    assert engine.execution_status("command-1") == ("CANCELED", "TEST_STOP")


def test_exclusive_control_and_new_lease(controlled):
    engine, context, lease = controlled
    with pytest.raises(ContractError, match="CONTROL_BUSY"):
        engine.acquire(context, "other", 1_000_000_000, 1)
    engine.release(context, lease, 2)
    new_context = replace(context, episode_id="episode-2", control_epoch=2)
    new_lease = engine.acquire(new_context, "runtime-test", 1_000_000_000, 3)
    assert new_lease != lease
    with pytest.raises(ContractError, match="STALE_CONTEXT"):
        engine.submit(command(context, lease), 0, 4)


def test_old_sequence_cannot_reexecute_after_history_eviction(profile):
    engine = BackendEngine(profile, max_history=1)
    context = replace(engine.context, episode_id="test", control_epoch=1)
    lease = engine.acquire(context, "test", 10_000_000_000, 0)
    first = command(context, lease)
    engine.submit(first, 0, 0)
    engine.tick(1_000_000_001)
    second = command(context, lease, command_id="command-2", command_seq=2)
    engine.submit(second, 0, 1_000_000_002)
    engine.tick(2_000_000_003)
    with pytest.raises(ContractError, match="COMMAND_OUT_OF_ORDER"):
        engine.submit(first, 0, 2_000_000_004)


def test_cancel_old_command_does_not_cancel_new_command(controlled):
    engine, context, lease = controlled
    engine.submit(command(context, lease), 0, 0)
    engine.tick(1_000_000_001)
    engine.submit(command(context, lease, command_id="command-2", command_seq=2), 0, 1_000_000_002)
    engine.cancel(context, lease, "command-1", 1_000_000_003)
    assert engine.execution_status("command-2")[0] == "STARTED"


def test_lease_handoff_preserves_pending_action_result(controlled):
    engine, context, lease = controlled
    engine.submit(command(context, lease), 0, 0)
    engine.release(context, lease, 100_000_000)
    newer = replace(context, episode_id="next-episode")
    newer_lease = engine.acquire(newer, "next-runtime", 1_000_000_000, 100_000_001)
    assert engine.execution_status("command-1") == ("CANCELED", "CONTROL_RELEASED")
    with pytest.raises(ContractError, match="COMMAND_ID_CONFLICT"):
        engine.submit(command(newer, newer_lease), 0, 100_000_002)
