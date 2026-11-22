"""Per-axis bounded target admission with original-output evidence."""

from __future__ import annotations

import hashlib
import os
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class Admission:
    positions: np.ndarray
    correction: np.ndarray
    accepted: bool
    reason: str


def admit_action_chunk(
    raw, names, lower, upper, corrections: Mapping[str, float]
) -> Admission:
    """Project named axes to the profile bounds and reject every overrun."""
    original = np.asarray(raw, dtype=np.float64)
    admitted = original.copy()
    reason = ""
    for index, name in enumerate(names):
        quota = float(corrections.get(name, 0.0))
        values = original[:, index]
        if quota > 0.0:
            admitted[:, index] = np.clip(values, lower[index], upper[index])
            correction = float(np.max(np.abs(admitted[:, index] - values)))
            if correction > quota and not reason:
                reason = f"{name}: admission correction {correction:.6g} exceeds {quota:.6g}"
        elif (np.any(values < lower[index]) or np.any(values > upper[index])) and not reason:
            reason = f"{name}: prediction outside hard limits"
    return Admission(admitted, admitted - original, not reason, reason)


def save_prediction(
    directory: Path,
    identity: str,
    raw,
    decision: Admission,
    *,
    bytes_used: int,
    max_bytes: int,
    execution_targets=None,
    ensemble_counts=None,
    history_observation_ids=None,
    history_observation_time_ns=None,
    fsync: bool = False,
) -> tuple[Path, int]:
    """Atomically save raw/admitted arrays; never put tensors in ordinary logs."""
    extras = {
        "execution_targets": execution_targets,
        "ensemble_counts": ensemble_counts,
        "history_observation_ids": history_observation_ids,
        "history_observation_time_ns": history_observation_time_ns,
    }
    saved_extras = {name: np.asarray(value) for name, value in extras.items() if value is not None}
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / (hashlib.sha256(identity.encode()).hexdigest() + ".npz")
    temporary = path.with_suffix(".tmp")
    if path.exists():
        raise ValueError("prediction artifact identity already exists")
    try:
        with temporary.open("xb") as stream:
            np.savez_compressed(
                stream,
                raw_act=np.asarray(raw),
                admitted=decision.positions,
                correction=decision.correction,
                accepted=decision.accepted,
                reason=decision.reason,
                **saved_extras,
            )
            stream.flush()
            if fsync:
                os.fsync(stream.fileno())
        size = temporary.stat().st_size
        if bytes_used + size > max_bytes:
            raise ValueError("prediction artifact byte budget exhausted")
        temporary.replace(path)
        return path, size
    finally:
        temporary.unlink(missing_ok=True)
