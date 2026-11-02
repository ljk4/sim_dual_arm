"""Pure control identity, command envelope and shared generation checks.

This module deliberately uses only the Python standard library.  It does not
import MuJoCo, SciPy, ROS, a scene, a clock or any backend.  Mock and native
control adapters compare the same five generation fields through
:func:`check_generation`; a full lease context additionally requires the
episode and control epoch to match.
"""

from __future__ import annotations

from dataclasses import dataclass
from numbers import Integral

from .models import ContractError, RunContext

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
        raise _invalid(
            "INVALID_CONTROL_IDENTITY",
            f"{location}: must be in {minimum}..{maximum}",
        )
    return result


def _text(value: object, location: str, *, allow_empty: bool = False) -> str:
    if not isinstance(value, str):
        raise _invalid("INVALID_CONTROL_IDENTITY", f"{location}: must be a string")
    if not value and not allow_empty:
        raise _invalid("INVALID_CONTROL_IDENTITY", f"{location}: must not be empty")
    if len(value) > MAX_ID_LENGTH:
        raise _invalid(
            "INVALID_CONTROL_IDENTITY",
            f"{location}: must be at most {MAX_ID_LENGTH} characters",
        )
    return value


def validate_context(context: object) -> RunContext:
    """Reject a malformed RunContext before it reaches control state."""
    if not isinstance(context, RunContext):
        raise _invalid("INVALID_CONTEXT", "context must be a RunContext")
    _text(context.robot_id, "context.robot_id")
    _text(context.robot_profile_hash, "context.robot_profile_hash")
    _text(context.backend_instance_id, "context.backend_instance_id")
    _text(context.time_domain_id, "context.time_domain_id")
    _text(context.episode_id, "context.episode_id", allow_empty=True)
    _integer(context.clock_epoch, "context.clock_epoch", 0, INT64_MAX)
    _integer(context.control_epoch, "context.control_epoch", 0, INT64_MAX)
    return context


def check_generation(context: object, expected: object) -> None:
    """Compare exactly the five fields that define one robot generation.

    Kept semantically identical to the former ``BackendEngine._check_generation``:
    a mismatch raises ``STALE_CONTEXT`` with the differing field name.
    """
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


@dataclass(frozen=True, slots=True)
class ControlCommand:
    """Transport-independent identity of one overall command or stream.

    ``command_seq`` identifies the whole command, not an individual chunk.  A
    continuous stream keeps the same context/lease/id/seq while every successor
    packet carries its own ``observation_id``.
    """

    context: RunContext
    lease_id: str
    command_id: str
    command_seq: int
    observation_id: int
    accept_before_ns: int

    def __post_init__(self) -> None:
        validate_context(self.context)
        lease_id = _text(self.lease_id, "lease_id")
        command_id = _text(self.command_id, "command_id")
        sequence = _integer(self.command_seq, "command_seq", 1, UINT64_MAX)
        observation = _integer(self.observation_id, "observation_id", 1, UINT64_MAX)
        accept_before = _integer(self.accept_before_ns, "accept_before_ns", 0, INT64_MAX)
        object.__setattr__(self, "lease_id", lease_id)
        object.__setattr__(self, "command_id", command_id)
        object.__setattr__(self, "command_seq", sequence)
        object.__setattr__(self, "observation_id", observation)
        object.__setattr__(self, "accept_before_ns", accept_before)

    @property
    def generation(self) -> tuple[str, int]:
        return self.context.generation
