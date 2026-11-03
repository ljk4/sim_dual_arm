"""Absolute-time temporal ensemble for ACT prediction chunks.

This module is deliberately NumPy-only: it has no ROS, Torch or runner
dependency.  Each prediction chunk is indexed by the simulation time of the
observation that produced it.  Future point ``i`` of an observation at time
``t`` is scheduled at ``t + (i + 1) * 50 ms``; a historical point ``j`` from
observation ``s`` contributes only when both absolute times are equal, i.e.
``s + (j + 1) * 50 ms == t + (i + 1) * 50 ms``.

Weights follow LeRobot's fixed ACT temporal ensemble, but use absolute
observation time instead of assuming that every query advances one action step:
``w = exp(coeff * (t - s) / P)``.  A positive coefficient therefore keeps older
predictions for longer, while ``coeff == 0`` gives every contributor equal
weight.  Contributions are normalized independently for every future point, so
a chunk's tail can legitimately use only the newest prediction.

Only complete, finite model chunks that already passed admission are accepted.
Times must advance strictly on the 50 ms grid; nothing is rounded, interpolated
or extrapolated.  :meth:`TemporalEnsemble.preview` computes a result without
changing history, and :meth:`TemporalEnsemble.commit` persists it, which lets the
provider save evidence and finish its logging before a prediction becomes part
of the session history.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

ACTION_DIM = 16
CHUNK_SIZE = 60
PERIOD_NS = 50_000_000
MAX_HISTORY_AGE_NS = 3_000_000_000
MAX_HISTORY = 60


def validate_coefficient(value: object) -> float:
    """Return a finite coefficient in [0, 1], rejecting bools explicitly."""
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ValueError("temporal ensemble coefficient must be a finite number in [0, 1]")
    coefficient = float(value)
    if not math.isfinite(coefficient) or not 0.0 <= coefficient <= 1.0:
        raise ValueError("temporal ensemble coefficient must be a finite number in [0, 1]")
    return coefficient


@dataclass(frozen=True)
class _Entry:
    observation_time_ns: int
    observation_id: int
    actions: np.ndarray


@dataclass(frozen=True)
class EnsembleResult:
    """One fused chunk plus the evidence needed by the prediction artifact."""

    targets: np.ndarray
    counts: np.ndarray
    history_observation_ids: np.ndarray
    history_observation_time_ns: np.ndarray


class TemporalEnsemble:
    """Bounded per-session history of admitted ACT chunks indexed by sim time."""

    def __init__(self, coefficient: float) -> None:
        self._coefficient = validate_coefficient(coefficient)
        self._entries: list[_Entry] = []

    @property
    def coefficient(self) -> float:
        return self._coefficient

    @property
    def history_length(self) -> int:
        """Number of retained observations, including the newest one."""
        return len(self._entries)

    def reset(self) -> None:
        self._entries.clear()

    def preview(
        self,
        observation_time_ns: int,
        observation_id: int,
        actions: object,
    ) -> EnsembleResult:
        """Fuse ``actions`` with retained history without mutating the history."""
        block = _block(actions)
        time_ns = self._time(observation_time_ns)
        observation_id = _observation_id(observation_id)
        self._check_advance(time_ns)

        total = block.copy()
        minimum = block.copy()
        maximum = block.copy()
        weight_sum = np.ones(CHUNK_SIZE, dtype=np.float64)
        counts = np.ones(CHUNK_SIZE, dtype=np.int64)
        history_ids: list[int] = []
        history_times: list[int] = []

        for entry in self._entries:
            age_ns = time_ns - entry.observation_time_ns
            if age_ns <= 0 or age_ns % PERIOD_NS:
                raise ValueError("temporal ensemble history is not on the advancing 50 ms grid")
            if age_ns >= MAX_HISTORY_AGE_NS:
                # A chunk older than its horizon cannot cover any new future point.
                continue
            offset = age_ns // PERIOD_NS
            weight = math.exp(self._coefficient * offset)
            overlap = CHUNK_SIZE - offset
            total[:overlap] += weight * entry.actions[offset:]
            minimum[:overlap] = np.minimum(minimum[:overlap], entry.actions[offset:])
            maximum[:overlap] = np.maximum(maximum[:overlap], entry.actions[offset:])
            weight_sum[:overlap] += weight
            counts[:overlap] += 1
            history_ids.append(entry.observation_id)
            history_times.append(entry.observation_time_ns)

        targets = total / weight_sum[:, None]
        if not np.isfinite(targets).all():
            raise ValueError("temporal ensemble produced non-finite targets")
        # A convex average cannot leave its contributors' range. Floating-point
        # division can nevertheless turn 0.08 into 0.08000000000000002. Enforce
        # the mathematical range without widening or projecting robot limits.
        targets = np.clip(targets, minimum, maximum)
        return EnsembleResult(
            targets=targets,
            counts=counts,
            history_observation_ids=np.asarray(history_ids, dtype=np.int64),
            history_observation_time_ns=np.asarray(history_times, dtype=np.int64),
        )

    def commit(
        self,
        observation_time_ns: int,
        observation_id: int,
        actions: object,
    ) -> EnsembleResult:
        """Persist one admitted prediction and return its fused chunk."""
        block = _block(actions)
        time_ns = self._time(observation_time_ns)
        observation_id = _observation_id(observation_id)
        self._check_advance(time_ns)
        result = self.preview(time_ns, observation_id, block)

        self._entries.append(_Entry(time_ns, observation_id, block))
        cutoff = time_ns - MAX_HISTORY_AGE_NS
        self._entries = [entry for entry in self._entries if entry.observation_time_ns > cutoff]
        if len(self._entries) > MAX_HISTORY:
            self._entries = self._entries[-MAX_HISTORY:]
        return result

    # ------------------------------------------------------------- validation
    def _check_advance(self, time_ns: int) -> None:
        if not self._entries:
            return
        previous = self._entries[-1].observation_time_ns
        if time_ns <= previous:
            raise ValueError(f"observation time {time_ns} does not advance past {previous}")
        if (time_ns - previous) % PERIOD_NS:
            raise ValueError(
                "observation time is not on the same 50 ms grid as the session history"
            )

    @staticmethod
    def _time(value: object) -> int:
        if isinstance(value, bool) or not isinstance(value, int):
            raise ValueError("observation time must be an integer number of nanoseconds")
        return value


def _observation_id(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError("observation id must be a non-negative integer")
    return value


def _block(actions: object) -> np.ndarray:
    try:
        block = np.asarray(actions, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError("temporal ensemble actions must be a numeric 60x16 chunk") from error
    if block.shape != (CHUNK_SIZE, ACTION_DIM):
        raise ValueError(
            f"temporal ensemble actions have shape {list(block.shape)}, "
            f"expected [{CHUNK_SIZE}, {ACTION_DIM}]"
        )
    if not np.isfinite(block).all():
        raise ValueError("temporal ensemble actions contain non-finite values")
    return block


__all__ = [
    "ACTION_DIM",
    "CHUNK_SIZE",
    "MAX_HISTORY",
    "MAX_HISTORY_AGE_NS",
    "PERIOD_NS",
    "EnsembleResult",
    "TemporalEnsemble",
    "validate_coefficient",
]
