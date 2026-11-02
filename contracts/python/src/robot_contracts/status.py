"""Pure validation and trajectory reconstruction for command-status snapshots.

Runtime and the offline auditor both call these functions so identity, array
order, time arithmetic, interpolation and the reference/measured invariant
cannot drift between live validation and recorded-data checks.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

from .models import CommandStatus, ContractError, RunContext, Target

COMMAND_PHASES = ("ACCEPTED", "STARTED", "COMPLETED", "CANCELED", "FAILED")
TERMINAL_PHASES = frozenset({"COMPLETED", "CANCELED", "FAILED"})
EXECUTION_CLOCKS = ("steady", "robot")
INTERPOLATION_VERSIONS = ("linear_v1",)
_POSITION_FIELDS = (
    "initial_positions",
    "reference_positions",
    "measured_positions",
    "tracking_error",
)


def _integer(value: object, location: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ContractError("INVALID_COMMAND_STATUS", f"{location} must be an integer")
    if value < 0:
        raise ContractError("INVALID_COMMAND_STATUS", f"{location} must not be negative")
    return value


def _positions(value: object, location: str, length: int) -> tuple[float, ...]:
    try:
        result = tuple(value)  # type: ignore[arg-type]
    except TypeError as error:
        raise ContractError("INVALID_COMMAND_STATUS", f"{location} must be an array") from error
    if len(result) != length:
        raise ContractError(
            "INVALID_COMMAND_STATUS",
            f"{location} has {len(result)} values for {length} joints",
        )
    for index, item in enumerate(result):
        if isinstance(item, bool) or not isinstance(item, int | float):
            raise ContractError("INVALID_COMMAND_STATUS", f"{location}[{index}] must be a number")
        if not math.isfinite(float(item)):
            raise ContractError("INVALID_COMMAND_STATUS", f"{location}[{index}] must be finite")
    return tuple(float(item) for item in result)


def _positions_match(left: Sequence[float], right: Sequence[float]) -> bool:
    return len(left) == len(right) and all(
        math.isclose(a, b, rel_tol=1e-9, abs_tol=1e-12) for a, b in zip(left, right, strict=True)
    )


def validate_command_status(
    value: CommandStatus,
    *,
    expected_context: RunContext | None = None,
    expected_command_id: str | None = None,
    expected_joint_names: Sequence[str] | None = None,
) -> None:
    """Reject a malformed snapshot before it is logged, recorded or trusted."""
    if not isinstance(value, CommandStatus):
        raise ContractError("INVALID_COMMAND_STATUS", "value is not a CommandStatus")
    if not isinstance(value.context, RunContext):
        raise ContractError("INVALID_COMMAND_STATUS", "context must be a RunContext")
    if not isinstance(value.command_id, str) or not value.command_id:
        raise ContractError("INVALID_COMMAND_STATUS", "command_id must be non-empty")
    if isinstance(value.command_seq, bool) or not isinstance(value.command_seq, int):
        raise ContractError("INVALID_COMMAND_STATUS", "command_seq must be an integer")
    if value.command_seq < 1:
        raise ContractError("INVALID_COMMAND_STATUS", "command_seq must be positive")
    if isinstance(value.observation_id, bool) or not isinstance(value.observation_id, int):
        raise ContractError("INVALID_COMMAND_STATUS", "observation_id must be an integer")
    if value.observation_id < 1:
        raise ContractError("INVALID_COMMAND_STATUS", "observation_id must be positive")
    if value.phase not in COMMAND_PHASES:
        raise ContractError("INVALID_COMMAND_STATUS", f"unknown phase: {value.phase!r}")
    if not isinstance(value.reason, str):
        raise ContractError("INVALID_COMMAND_STATUS", "reason must be a string")
    if value.execution_clock not in EXECUTION_CLOCKS:
        raise ContractError(
            "INVALID_COMMAND_STATUS", f"unknown execution_clock: {value.execution_clock!r}"
        )
    if value.interpolation_version not in INTERPOLATION_VERSIONS:
        raise ContractError(
            "INVALID_COMMAND_STATUS",
            f"unknown interpolation_version: {value.interpolation_version!r}",
        )

    accepted = _integer(value.accepted_at_ns, "accepted_at_ns")
    started = _integer(value.started_at_ns, "started_at_ns")
    sample = _integer(value.sample_time_ns, "sample_time_ns")
    _integer(value.elapsed_ns, "elapsed_ns")
    if accepted > started:
        raise ContractError("INVALID_COMMAND_STATUS", "accepted_at_ns is after started_at_ns")
    if started > sample:
        raise ContractError("INVALID_COMMAND_STATUS", "started_at_ns is after sample_time_ns")

    names = tuple(value.joint_names)
    if not names or any(not isinstance(name, str) or not name for name in names):
        raise ContractError("INVALID_COMMAND_STATUS", "joint_names must be non-empty strings")
    if len(set(names)) != len(names):
        raise ContractError("INVALID_COMMAND_STATUS", "joint_names contain duplicates")
    positions = {
        name: _positions(getattr(value, name), name, len(names)) for name in _POSITION_FIELDS
    }
    for reference, measured, error in zip(
        positions["reference_positions"],
        positions["measured_positions"],
        positions["tracking_error"],
        strict=True,
    ):
        if not math.isclose(reference - measured, error, rel_tol=1e-9, abs_tol=1e-12):
            raise ContractError(
                "INVALID_COMMAND_STATUS",
                "tracking_error must equal reference_positions - measured_positions",
            )
    if isinstance(value.progress, bool) or not isinstance(value.progress, int | float):
        raise ContractError("INVALID_COMMAND_STATUS", "progress must be a number")
    if not math.isfinite(float(value.progress)) or not 0.0 <= float(value.progress) <= 1.0:
        raise ContractError("INVALID_COMMAND_STATUS", "progress must be finite and in [0, 1]")

    if expected_context is not None and value.context != expected_context:
        raise ContractError("STALE_CONTEXT", "command status context differs")
    if expected_command_id is not None and value.command_id != expected_command_id:
        raise ContractError("INVALID_COMMAND_STATUS", "command_id differs")
    if expected_joint_names is not None and names != tuple(expected_joint_names):
        raise ContractError("INVALID_COMMAND_STATUS", "joint_names/order differ")


def interpolate_reference(
    initial: Sequence[float],
    targets: Sequence[Target],
    elapsed_ns: int,
    interpolation_version: str,
) -> tuple[float, ...]:
    """Rebuild the reference the backend should have produced at elapsed_ns."""
    if interpolation_version != "linear_v1":
        raise ContractError(
            "INVALID_COMMAND_STATUS", f"unknown interpolation_version: {interpolation_version!r}"
        )
    if not targets:
        raise ContractError("INVALID_COMMAND_STATUS", "command has no trajectory targets")
    elapsed = max(0, elapsed_ns)
    start_time, start = 0, tuple(initial)
    for target in targets:
        if elapsed < target.time_ns:
            fraction = (elapsed - start_time) / (target.time_ns - start_time)
            return tuple(
                left + fraction * (right - left)
                for left, right in zip(start, target.positions, strict=True)
            )
        start_time, start = target.time_ns, target.positions
    return tuple(targets[-1].positions)


def validate_status_against_trajectory(
    value: CommandStatus,
    targets: Sequence[Target],
    *,
    expected_context: RunContext | None = None,
    expected_command_id: str | None = None,
    expected_command_seq: int | None = None,
    expected_observation_id: int | None = None,
    expected_joint_names: Sequence[str] | None = None,
) -> None:
    """Validate one sample's identity, reconstructed reference and progress."""
    validate_command_status(
        value,
        expected_context=expected_context,
        expected_command_id=expected_command_id,
        expected_joint_names=expected_joint_names,
    )
    if expected_command_seq is not None and value.command_seq != expected_command_seq:
        raise ContractError("INVALID_COMMAND_STATUS", "command_seq differs from command")
    if expected_observation_id is not None and value.observation_id != expected_observation_id:
        raise ContractError("INVALID_COMMAND_STATUS", "observation_id differs from command")
    expected_reference = interpolate_reference(
        value.initial_positions,
        targets,
        value.elapsed_ns,
        value.interpolation_version,
    )
    if not _positions_match(value.reference_positions, expected_reference):
        raise ContractError(
            "INVALID_COMMAND_STATUS", "reference_positions differ from accepted trajectory"
        )
    horizon = targets[-1].time_ns
    if value.phase == "COMPLETED" and value.elapsed_ns < horizon:
        raise ContractError(
            "INVALID_COMMAND_STATUS", "COMPLETED elapsed is shorter than trajectory horizon"
        )
    expected_progress = (
        1.0
        if value.phase == "COMPLETED" or horizon <= 0
        else min(1.0, max(0.0, value.elapsed_ns / horizon))
    )
    if not math.isclose(float(value.progress), expected_progress, rel_tol=1e-9, abs_tol=1e-12):
        raise ContractError("INVALID_COMMAND_STATUS", "progress differs from elapsed/phase")


def validate_status_transition(previous: CommandStatus | None, current: CommandStatus) -> None:
    """Keep one command's metadata fixed and its sampled execution monotonic.

    Callers validate the individual sample and trajectory first. Only the last
    immutable snapshot is retained; a terminal snapshot is permanently frozen.
    """
    if previous is None:
        return
    for field in (
        "context",
        "command_id",
        "command_seq",
        "observation_id",
        "joint_names",
        "initial_positions",
        "accepted_at_ns",
        "started_at_ns",
        "execution_clock",
        "interpolation_version",
    ):
        if getattr(previous, field) != getattr(current, field):
            raise ContractError("INVALID_COMMAND_STATUS", f"command metadata changed: {field}")
    if previous.phase in TERMINAL_PHASES:
        if current != previous:
            raise ContractError("INVALID_COMMAND_STATUS", "terminal snapshot changed")
        return
    if previous.phase == "STARTED" and current.phase == "ACCEPTED":
        raise ContractError("INVALID_COMMAND_STATUS", "execution phase went backwards")
    for field in ("sample_time_ns", "elapsed_ns", "progress"):
        if getattr(current, field) < getattr(previous, field):
            raise ContractError("INVALID_COMMAND_STATUS", f"{field} went backwards")


__all__ = [
    "COMMAND_PHASES",
    "EXECUTION_CLOCKS",
    "INTERPOLATION_VERSIONS",
    "TERMINAL_PHASES",
    "CommandStatus",
    "interpolate_reference",
    "validate_command_status",
    "validate_status_against_trajectory",
    "validate_status_transition",
]
