"""The explicit concrete union of strict chunks and bounded-follower chunks.

Both values are accepted by the same ``NativeMotion`` / ``StreamSchedule``
physical loop, but they remain distinct immutable contract types.  There is no
inheritance and no duck-typed stand-in: every decision below uses
``isinstance`` against the two real classes.
"""

from __future__ import annotations

from robot_contracts.follower import PreparedFollowerPlan
from robot_contracts.models import ContractError

from .chunk_plan import PreparedChunk

__all__ = [
    "PreparedStreamChunk",
    "STREAM_CHUNK_KINDS",
    "chunk_kind",
    "known_chunk_kind",
]

type PreparedStreamChunk = PreparedChunk | PreparedFollowerPlan
STREAM_CHUNK_KINDS = frozenset({"CHUNK", "FOLLOWER"})


def chunk_kind(prepared: object) -> str:
    """Return ``CHUNK`` or ``FOLLOWER`` for one concrete prepared stream chunk."""
    if isinstance(prepared, PreparedChunk):
        return "CHUNK"
    if isinstance(prepared, PreparedFollowerPlan):
        return "FOLLOWER"
    raise ContractError(
        "INVALID_PREPARED_CHUNK",
        "prepared stream chunk must be a PreparedChunk or PreparedFollowerPlan",
    )


def known_chunk_kind(prepared: object) -> str | None:
    """Concrete kind, or ``None`` for legacy schedule fixtures.

    The scheduling helpers historically accepted attribute-compatible test
    doubles.  New production paths keep using the strict :func:`chunk_kind`;
    this non-strict probe only preserves those existing scheduling interfaces.
    """
    if isinstance(prepared, PreparedChunk):
        return "CHUNK"
    if isinstance(prepared, PreparedFollowerPlan):
        return "FOLLOWER"
    return None
