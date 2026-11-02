from dataclasses import FrozenInstanceError, replace

import pytest
from robot_contracts.control import (
    INT64_MAX,
    ControlCommand,
    check_generation,
    check_lease_context,
    validate_context,
    validate_lease_duration,
    validate_seed,
    validate_wall,
)
from robot_contracts.models import ContractError, RunContext


def context(**changes) -> RunContext:
    value = RunContext(
        "panda-stack",
        "profile-digest",
        "backend-instance",
        "mujoco-sim",
        0,
        episode_id="episode-1",
        control_epoch=1,
    )
    return replace(value, **changes)


def command(**changes) -> ControlCommand:
    value = ControlCommand(
        context(),
        "lease-1",
        "command-1",
        1,
        1,
        5_000_000_000,
    )
    return replace(value, **changes)


def test_contract_values_are_frozen_and_normalised() -> None:
    value = command()
    assert value.generation == ("backend-instance", 0)
    with pytest.raises(FrozenInstanceError):
        value.command_seq = 2  # type: ignore[misc]


@pytest.mark.parametrize(
    "field,value,reason",
    [
        ("lease_id", "", "INVALID_CONTROL_IDENTITY"),
        ("command_id", "", "INVALID_CONTROL_IDENTITY"),
        ("lease_id", "x" * 129, "INVALID_CONTROL_IDENTITY"),
        ("command_seq", True, "INVALID_CONTROL_IDENTITY"),
        ("command_seq", 0, "INVALID_CONTROL_IDENTITY"),
        ("observation_id", False, "INVALID_CONTROL_IDENTITY"),
        ("observation_id", 0, "INVALID_CONTROL_IDENTITY"),
        ("accept_before_ns", -1, "INVALID_CONTROL_IDENTITY"),
        ("accept_before_ns", INT64_MAX + 1, "INVALID_CONTROL_IDENTITY"),
    ],
)
def test_command_rejects_bad_identity(field: str, value, reason: str) -> None:
    with pytest.raises(ContractError) as caught:
        command(**{field: value})
    assert caught.value.reason == reason


@pytest.mark.parametrize(
    "field,value",
    [
        ("robot_id", ""),
        ("robot_profile_hash", ""),
        ("backend_instance_id", "x" * 129),
        ("time_domain_id", ""),
        ("clock_epoch", True),
        ("control_epoch", -1),
    ],
)
def test_context_is_strictly_validated(field: str, value) -> None:
    with pytest.raises(ContractError) as caught:
        validate_context(context(**{field: value}))
    assert caught.value.reason == "INVALID_CONTROL_IDENTITY"


def test_generation_ignores_episode_and_control_epoch() -> None:
    expected = context()
    changed = replace(expected, episode_id="other", control_epoch=7)
    check_generation(changed, expected)
    with pytest.raises(ContractError) as caught:
        check_lease_context(changed, expected)
    assert caught.value.reason == "STALE_CONTEXT"


@pytest.mark.parametrize(
    "field,value",
    [
        ("robot_id", "other"),
        ("robot_profile_hash", "other"),
        ("backend_instance_id", "other"),
        ("time_domain_id", "other"),
        ("clock_epoch", 1),
    ],
)
def test_each_generation_field_mismatch_is_stale(field: str, value) -> None:
    with pytest.raises(ContractError) as caught:
        check_generation(context(**{field: value}), context())
    assert caught.value.reason == "STALE_CONTEXT"
    assert field in str(caught.value)


def test_lease_duration_wall_and_seed_bounds() -> None:
    assert validate_lease_duration(100_000_000) == 100_000_000
    assert validate_lease_duration(10_000_000_000) == 10_000_000_000
    for value in (True, 99_999_999, 10_000_000_001, 1.5):
        with pytest.raises(ContractError) as caught:
            validate_lease_duration(value)
        assert caught.value.reason == "INVALID_LEASE_DURATION"
    assert validate_wall(0) == 0
    assert validate_wall(INT64_MAX) == INT64_MAX
    for value in (True, -1, INT64_MAX + 1, 1.5):
        with pytest.raises(ContractError) as caught:
            validate_wall(value)
        assert caught.value.reason == "INVALID_WALL_TIME"
    assert validate_seed(0) == 0
    for value in (True, -1, INT64_MAX + 1):
        with pytest.raises(ContractError) as caught:
            validate_seed(value)
        assert caught.value.reason == "INVALID_SEED"
