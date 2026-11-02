from dataclasses import replace

import pytest
from conftest import command
from robot_backend_core.engine import BackendEngine
from robot_contracts.models import CommandStatus, ContractError, RunContext
from robot_contracts.status import (
    interpolate_reference,
    validate_command_status,
    validate_status_against_trajectory,
)


def _status(engine, context, command_id="command-1"):
    return engine.query_command(context, command_id)


def test_active_status_uses_robot_clock_and_freezes_terminal(controlled):
    engine, context, lease = controlled
    engine.submit(command(context, lease), 10_000, 0)
    active = _status(engine, context)
    assert active.phase == "STARTED"
    assert active.accepted_at_ns == active.started_at_ns == 10_000
    assert active.sample_time_ns == 10_000
    assert active.elapsed_ns == 0
    assert active.initial_positions == engine.profile.initial_positions
    assert active.reference_positions == active.measured_positions
    assert active.tracking_error == (0.0,) * 6
    assert active.progress == 0.0

    engine.tick(500_000_000, 500_010_000)
    middle = _status(engine, context)
    assert middle.phase == "STARTED"
    assert middle.sample_time_ns == 500_010_000
    assert middle.elapsed_ns == 500_000_000
    assert middle.progress == pytest.approx(0.5)
    assert middle.reference_positions == engine.positions
    assert middle.reference_positions[0] == pytest.approx(0.1)
    assert middle.initial_positions == engine.profile.initial_positions

    engine.tick(1_000_000_000, 1_000_010_000)
    terminal = _status(engine, context)
    assert terminal.phase == "COMPLETED"
    assert terminal.progress == 1.0
    assert terminal.reference_positions == terminal.measured_positions
    assert terminal.reference_positions == (0.2, -0.2, 0.1, -0.1, 0.02, 0.02)
    held = engine.positions

    engine.tick(2_000_000_000, 2_000_010_000)
    assert engine.positions == held
    assert _status(engine, context) == terminal


def test_query_is_read_only_and_does_not_reemit_events(controlled):
    engine, context, lease = controlled
    engine.submit(command(context, lease), 0, 0)
    events = engine.drain_events()
    assert [event.phase for event in events] == ["ACCEPTED", "STARTED"]
    position = engine.positions
    first = _status(engine, context)
    second = _status(engine, context)
    assert first == second
    assert engine.positions == position
    assert not engine.drain_events()


def test_lease_expiry_does_not_need_robot_time_to_advance(controlled):
    engine, context, lease = controlled
    engine.submit(command(context, lease), 1_000, 0)
    engine.tick(100_000_000, 1_000)
    held = engine.positions
    assert held[0] == pytest.approx(0.02)
    engine.tick(2_000_000_001, 1_000)
    status = _status(engine, context)
    assert status.phase == "CANCELED"
    assert status.reason == "LEASE_EXPIRED"
    assert status.sample_time_ns == 1_000
    assert status.elapsed_ns == 100_000_000
    assert status.progress == pytest.approx(0.1)
    assert status.reference_positions == held
    assert engine.positions == held
    assert engine.drain_events()[-1].stamp_ns == 1_000


def test_history_eviction_and_context_mismatch_are_explicit(profile):
    engine = BackendEngine(profile, max_history=1)
    context = replace(engine.context, episode_id="episode-a", control_epoch=1)
    lease = engine.acquire(context, "runtime", 10_000_000_000, 0)
    first = command(context, lease)
    engine.submit(first, 0, 0)
    engine.tick(1_000_000_001, 1_000_000_001)
    second = command(context, lease, command_id="command-2", command_seq=2)
    engine.submit(second, 0, 1_000_000_002)
    engine.tick(2_000_000_003, 2_000_000_003)
    with pytest.raises(ContractError, match="RESULT_EXPIRED"):
        engine.query_command(context, "command-1")
    assert engine.query_command(context, "command-2").phase == "COMPLETED"
    with pytest.raises(ContractError, match="STALE_CONTEXT"):
        engine.query_command(replace(context, episode_id="episode-b"), "command-2")
    with pytest.raises(ContractError, match="STALE_CONTEXT"):
        engine.query_command(replace(context, clock_epoch=2), "command-2")


def test_terminal_history_survives_same_generation_lease_handoff(controlled):
    engine, context, lease = controlled
    engine.submit(command(context, lease), 0, 0)
    engine.tick(1_000_000_001, 1_000_000_001)
    engine.release(context, lease, 1_000_000_002, robot_now_ns=1_000_000_002)
    newer = replace(context, episode_id="next-episode", control_epoch=2)
    engine.acquire(newer, "next-runtime", 10_000_000_000, 1_000_000_003)
    assert engine.query_command(context, "command-1").phase == "COMPLETED"
    with pytest.raises(ContractError, match="STALE_CONTEXT"):
        engine.query_command(newer, "command-1")


def test_duplicate_command_does_not_reemit_events(controlled):
    engine, context, lease = controlled
    first = engine.submit(command(context, lease), 0, 0)
    engine.tick(1_000_000_001, 1_000_000_001)
    engine.drain_events()
    duplicate = engine.submit(command(context, lease), 0, 1_000_000_002)
    assert duplicate is first
    assert not engine.drain_events()
    assert engine.query_command(context, "command-1").phase == "COMPLETED"


def test_separate_robot_and_steady_clocks(controlled):
    engine, context, lease = controlled
    engine.submit(command(context, lease, accept_before_ns=10_000_000_000), 5_000_000_000, 0)
    engine.tick(400_000_000, 6_000_000_000)
    status = _status(engine, context)
    assert status.accepted_at_ns == status.started_at_ns == 5_000_000_000
    assert status.sample_time_ns == 6_000_000_000
    assert status.elapsed_ns == 400_000_000
    assert status.execution_clock == "steady"
    assert status.interpolation_version == "linear_v1"
    assert status.reference_positions[0] == pytest.approx(0.08)
    assert status.progress == pytest.approx(0.4)
    events = engine.drain_events()
    assert [event.stamp_ns for event in events] == [5_000_000_000, 5_000_000_000]


def test_status_validation_rejects_tracking_and_progress_errors(controlled):
    engine, context, lease = controlled
    engine.submit(command(context, lease), 0, 0)
    status = _status(engine, context)
    validate_command_status(status, expected_context=context, expected_command_id="command-1")
    with pytest.raises(ContractError, match="INVALID_COMMAND_STATUS"):
        validate_command_status(replace(status, tracking_error=(1.0,) + status.tracking_error[1:]))
    with pytest.raises(ContractError, match="INVALID_COMMAND_STATUS"):
        validate_command_status(replace(status, progress=float("nan")))
    with pytest.raises(ContractError, match="INVALID_COMMAND_STATUS"):
        validate_command_status(replace(status, execution_clock="wall"))
    with pytest.raises(ContractError, match="INVALID_COMMAND_STATUS"):
        validate_command_status(replace(status, interpolation_version="cubic_v1"))
    with pytest.raises(ContractError, match="INVALID_COMMAND_STATUS"):
        validate_command_status(replace(status, joint_names=("wrong",) * 6))
    with pytest.raises(ContractError, match="STALE_CONTEXT"):
        validate_command_status(status, expected_context=replace(context, episode_id="other"))


def test_status_arrays_are_tuples_even_when_constructed_from_lists(controlled):
    engine, context, lease = controlled
    engine.submit(command(context, lease), 0, 0)
    status = _status(engine, context)
    mutable = CommandStatus(
        context=status.context,
        command_id=status.command_id,
        command_seq=status.command_seq,
        observation_id=status.observation_id,
        phase=status.phase,
        reason=status.reason,
        sample_time_ns=status.sample_time_ns,
        accepted_at_ns=status.accepted_at_ns,
        started_at_ns=status.started_at_ns,
        elapsed_ns=status.elapsed_ns,
        execution_clock=status.execution_clock,
        interpolation_version=status.interpolation_version,
        joint_names=list(status.joint_names),
        initial_positions=list(status.initial_positions),
        reference_positions=list(status.reference_positions),
        measured_positions=list(status.measured_positions),
        tracking_error=list(status.tracking_error),
        progress=status.progress,
    )
    assert isinstance(mutable.joint_names, tuple)
    assert isinstance(mutable.reference_positions, tuple)
    assert isinstance(mutable.tracking_error, tuple)
    validate_command_status(mutable)


def test_run_context_identity_is_part_of_status(controlled):
    engine, context, lease = controlled
    engine.submit(command(context, lease), 0, 0)
    status = _status(engine, context)
    assert isinstance(status.context, RunContext)
    assert status.command_id == "command-1"
    assert (status.command_seq, status.observation_id) == (1, 1)


def test_paused_robot_clock_uses_steady_execution_elapsed(controlled):
    engine, context, lease = controlled
    engine.submit(command(context, lease, accept_before_ns=10_000_000_000), 5_000_000_000, 0)
    engine.tick(50_000_000, 5_000_000_000)
    status = _status(engine, context)
    assert status.sample_time_ns == 5_000_000_000
    assert status.started_at_ns == 5_000_000_000
    assert status.elapsed_ns == 50_000_000
    assert status.elapsed_ns != status.sample_time_ns - status.started_at_ns
    assert status.reference_positions[0] == pytest.approx(0.01)
    assert status.progress == pytest.approx(0.05)

    engine.stop(context, lease, 200_000_000, "TEST_STOP", robot_now_ns=5_000_000_000)
    terminal = _status(engine, context)
    assert terminal.phase == "CANCELED"
    assert terminal.reference_positions == status.reference_positions
    assert terminal.elapsed_ns == status.elapsed_ns
    assert terminal.progress == status.progress
    assert terminal.sample_time_ns == 5_000_000_000
    assert engine.drain_events()[-1].stamp_ns == 5_000_000_000


def test_shared_trajectory_validation_reconstructs_each_sample(controlled):
    engine, context, lease = controlled
    command_value = command(context, lease)
    engine.submit(command_value, 0, 0)
    engine.tick(50_000_000, 50_000_000)
    status = _status(engine, context)
    validate_status_against_trajectory(
        status,
        command_value.targets,
        expected_context=context,
        expected_command_id="command-1",
        expected_command_seq=1,
        expected_observation_id=1,
        expected_joint_names=command_value.joint_names,
    )
    expected = interpolate_reference(
        status.initial_positions,
        command_value.targets,
        50_000_000,
        "linear_v1",
    )
    assert status.reference_positions == expected
    with pytest.raises(ContractError, match="INVALID_COMMAND_STATUS"):
        validate_status_against_trajectory(
            replace(
                status,
                reference_positions=(
                    status.reference_positions[0] + 0.005,
                    *status.reference_positions[1:],
                ),
                measured_positions=(
                    status.reference_positions[0] + 0.005,
                    *status.reference_positions[1:],
                ),
            ),
            command_value.targets,
        )
    with pytest.raises(ContractError, match="INVALID_COMMAND_STATUS"):
        validate_status_against_trajectory(replace(status, progress=0.9), command_value.targets)


@pytest.mark.parametrize(
    "field,value",
    [
        ("execution_clock", "robot"),
        ("phase", "ACCEPTED"),
        ("elapsed_ns", 0),
        ("sample_time_ns", 1),
    ],
)
def test_status_history_rejects_redefinition_or_reversal(controlled, field, value):
    from robot_contracts.status import validate_status_transition

    engine, context, lease = controlled
    engine.submit(command(context, lease), 0, 0)
    engine.tick(500_000_000, 500_000_000)
    previous = engine.query_command(context, "command-1")
    with pytest.raises(ContractError, match="INVALID_COMMAND_STATUS"):
        validate_status_transition(previous, replace(previous, **{field: value}))


def test_terminal_history_cannot_change_measurement_or_reason(controlled):
    from robot_contracts.status import validate_status_transition

    engine, context, lease = controlled
    engine.submit(command(context, lease), 0, 0)
    engine.tick(1_000_000_000, 1_000_000_000)
    terminal = engine.query_command(context, "command-1")
    validate_status_transition(terminal, terminal)
    for changed in (
        replace(terminal, measured_positions=(0.21, *terminal.measured_positions[1:])),
        replace(terminal, reason="changed"),
    ):
        with pytest.raises(ContractError, match="terminal snapshot changed"):
            validate_status_transition(terminal, changed)
