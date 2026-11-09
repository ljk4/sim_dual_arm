"""Pure control identity, command envelope and shared generation checks.

This module deliberately uses only the Python standard library.  It does not
import MuJoCo, SciPy, ROS, a scene, a clock or any backend.
"""

from __future__ import annotations

from numbers import Integral
from typing import Annotated, ClassVar

from pydantic import Field

from .models import ContractError, RunContext
from .validation import NonEmptyStr, PositiveInt, StrictContractModel

__all__ = [
    "GENERATION_FIELDS",
    "INT64_MAX",
    "LEASE_CONTEXT_FIELDS",
    "MAX_ID_LENGTH",
    "MAX_LEASE_DURATION_NS",
    "MIN_LEASE_DURATION_NS",
    "UINT64_MAX",
    "ControlCommand",
    "check_generation",
    "check_lease_context",
    "validate_context",
    "validate_lease_duration",
    "validate_owner",
    "validate_seed",
    "validate_wall",
]

INT64_MAX = 2**63 - 1
UINT64_MAX = 2**64 - 1
MAX_ID_LENGTH = 128
MIN_LEASE_DURATION_NS = 100_000_000
MAX_LEASE_DURATION_NS = 10_000_000_000
GENERATION_FIELDS = (
    "robot_id",
    "robot_profile_hash",
    "backend_instance_id",
    "time_domain_id",
    "clock_epoch",
)
LEASE_CONTEXT_FIELDS = GENERATION_FIELDS + ("episode_id", "control_epoch")


def _invalid(reason: str, detail: str) -> ContractError:
    return ContractError(reason, detail)


def _integer(value: object, location: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise _invalid("INVALID_CONTROL_IDENTITY", f"{location}: must be an integer")
    result = int(value)
    if result < minimum or result > maximum:
        raise _invalid("INVALID_CONTROL_IDENTITY", f"{location}: must be in {minimum}..{maximum}")
    return result


def _text(value: object, location: str, *, allow_empty: bool = False) -> str:
    if not isinstance(value, str):
        raise _invalid("INVALID_CONTROL_IDENTITY", f"{location}: must be a string")
    if not value and not allow_empty:
        raise _invalid("INVALID_CONTROL_IDENTITY", f"{location}: must not be empty")
    if len(value) > MAX_ID_LENGTH:
        raise _invalid("INVALID_CONTROL_IDENTITY", f"{location}: must be at most {MAX_ID_LENGTH}")
    return value


def validate_context(context: object) -> RunContext:
    """Reject a malformed RunContext before it reaches control state."""
    if not isinstance(context, RunContext):
        raise _invalid("INVALID_CONTEXT", "context must be a RunContext")
    return context


def check_generation(context: object, expected: object) -> None:
    """Compare exactly the five fields that define one robot generation."""
    if not isinstance(context, RunContext) or not isinstance(expected, RunContext):
        raise _invalid("STALE_CONTEXT", "RunContext")
    for name in GENERATION_FIELDS:
        if getattr(context, name) != getattr(expected, name):
            raise _invalid("STALE_CONTEXT", name)


def check_lease_context(context: object, expected: object) -> None:
    """Require the shared generation plus the episode/control epoch to match."""
    check_generation(context, expected)
    for name in ("episode_id", "control_epoch"):
        if getattr(context, name) != getattr(expected, name):
            raise _invalid("STALE_CONTEXT", name)


def validate_owner(owner: object) -> str:
    return _text(owner, "owner")


def validate_lease_duration(duration_ns: object) -> int:
    if isinstance(duration_ns, bool) or not isinstance(duration_ns, Integral):
        raise _invalid("INVALID_LEASE_DURATION", "duration_ns must be an integer")
    result = int(duration_ns)
    if not MIN_LEASE_DURATION_NS <= result <= MAX_LEASE_DURATION_NS:
        raise _invalid(
            "INVALID_LEASE_DURATION",
            f"duration_ns must be in {MIN_LEASE_DURATION_NS}..{MAX_LEASE_DURATION_NS}",
        )
    return result


def validate_wall(value: object) -> int:
    """Validate one explicit monotonic wall time in integer nanoseconds."""
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise _invalid("INVALID_WALL_TIME", "wall_time_ns must be an integer")
    result = int(value)
    if result < 0 or result > INT64_MAX:
        raise _invalid("INVALID_WALL_TIME", "wall_time_ns must be in 0..int64")
    return result


def validate_seed(seed: object) -> int:
    if isinstance(seed, bool) or not isinstance(seed, Integral):
        raise _invalid("INVALID_SEED", "seed must be an integer")
    result = int(seed)
    if result < 0 or result > INT64_MAX:
        raise _invalid("INVALID_SEED", "seed must be a non-negative int64")
    return result


class ControlCommand(StrictContractModel):
    """Transport-independent identity of one overall command or stream."""

    _error_reason: ClassVar[str] = "INVALID_CONTROL_IDENTITY"

    context: RunContext
    lease_id: Annotated[NonEmptyStr, Field(max_length=MAX_ID_LENGTH)]
    command_id: Annotated[NonEmptyStr, Field(max_length=MAX_ID_LENGTH)]
    command_seq: Annotated[PositiveInt, Field(le=UINT64_MAX)]
    observation_id: Annotated[PositiveInt, Field(le=UINT64_MAX)]
    accept_before_ns: Annotated[int, Field(strict=True, ge=0, le=INT64_MAX)]

    @property
    def generation(self) -> tuple[str, int]:
        return self.context.generation
