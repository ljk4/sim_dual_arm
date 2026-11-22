"""Absolute-time temporal ensemble for ACT prediction chunks."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

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
    """Bounded per-session history of admitted ACT chunks indexed by backend time."""

    def __init__(
        self,
        coefficient: float,
        *,
        chunk_size: int = CHUNK_SIZE,
        action_dim: int | None = None,
        period_ns: int = PERIOD_NS,
    ) -> None:
        self._coefficient = validate_coefficient(coefficient)
        self._chunk_size = int(chunk_size)
        self._action_dim = None if action_dim is None else int(action_dim)
        self._period_ns = int(period_ns)
        self._max_history_age_ns = self._chunk_size * self._period_ns
        self._max_history = self._chunk_size
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
        block = _block(actions, chunk_size=self._chunk_size, action_dim=self._action_dim)
        time_ns = self._time(observation_time_ns)
        observation_id = _observation_id(observation_id)
        self._check_advance(time_ns)

        total = block.copy()
        minimum = block.copy()
        maximum = block.copy()
        weight_sum = np.ones(self._chunk_size, dtype=np.float64)
        counts = np.ones(self._chunk_size, dtype=np.int64)
        history_ids: list[int] = []
        history_times: list[int] = []

        for entry in self._entries:
            age_ns = time_ns - entry.observation_time_ns
            if age_ns <= 0 or age_ns % self._period_ns:
                raise ValueError("temporal ensemble history is not on the advancing 50 ms grid")
            if age_ns >= self._max_history_age_ns:
                continue
            offset = age_ns // self._period_ns
            weight = math.exp(self._coefficient * offset)
            overlap = self._chunk_size - offset
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
        block = _block(actions, chunk_size=self._chunk_size, action_dim=self._action_dim)
        time_ns = self._time(observation_time_ns)
        observation_id = _observation_id(observation_id)
        self._check_advance(time_ns)
        result = self.preview(time_ns, observation_id, block)

        self._entries.append(_Entry(time_ns, observation_id, block))
        cutoff = time_ns - self._max_history_age_ns
        self._entries = [entry for entry in self._entries if entry.observation_time_ns > cutoff]
        if len(self._entries) > self._max_history:
            self._entries = self._entries[-self._max_history:]
        return result

    def _check_advance(self, time_ns: int) -> None:
        if not self._entries:
            return
        previous = self._entries[-1].observation_time_ns
        if time_ns <= previous:
            raise ValueError(f"observation time {time_ns} does not advance past {previous}")
        if (time_ns - previous) % self._period_ns:
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


def _block(
    actions: object, *, chunk_size: int = CHUNK_SIZE, action_dim: int | None = None
) -> np.ndarray:
    expected_shape = (
        f"[{chunk_size}, {action_dim}]" if action_dim is not None else f"[{chunk_size}, D]"
    )
    try:
        block = np.asarray(actions, dtype=np.float64)
    except (TypeError, ValueError) as error:
        raise ValueError(
            f"temporal ensemble actions must be a numeric {expected_shape} chunk"
        ) from error
    if action_dim is None:
        if block.ndim != 2 or block.shape[0] != chunk_size or block.shape[1] < 1:
            raise ValueError(
                f"temporal ensemble actions have shape {list(block.shape)}, "
                f"expected [{chunk_size}, >=1]"
            )
    elif block.shape != (chunk_size, action_dim):
        raise ValueError(
            f"temporal ensemble actions have shape {list(block.shape)}, "
            f"expected [{chunk_size}, {action_dim}]"
        )
    if not np.isfinite(block).all():
        raise ValueError("temporal ensemble actions contain non-finite values")
    return block


__all__ = [
    "CHUNK_SIZE",
    "MAX_HISTORY",
    "MAX_HISTORY_AGE_NS",
    "PERIOD_NS",
    "EnsembleResult",
    "TemporalEnsemble",
    "validate_coefficient",
]
