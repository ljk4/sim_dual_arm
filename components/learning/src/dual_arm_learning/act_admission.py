"""User-approved gripper-only target admission with original-output evidence."""

from __future__ import annotations

import hashlib
import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class Admission:
    positions: np.ndarray
    correction: np.ndarray
    accepted: bool
    reason: str


def admit_grippers(raw, names, lower, upper, max_correction_m: float) -> Admission:
    """Keep arm targets strict; clip only named total gripper apertures."""
    original = np.asarray(raw, dtype=np.float64)
    admitted = original.copy()
    reason = ""
    for index, name in enumerate(names):
        values = original[:, index]
        if name in {"left_gripper_width", "right_gripper_width"}:
            admitted[:, index] = np.clip(values, lower[index], upper[index])
            if np.max(np.abs(admitted[:, index] - values)) > max_correction_m:
                reason = f"{name}: admission correction exceeds {max_correction_m} m"
        elif np.any(values < lower[index]) or np.any(values > upper[index]):
            reason = f"{name}: arm prediction outside hard limits"
    return Admission(admitted, admitted - original, not reason, reason)


def save_prediction(
    directory: Path, identity: str, raw, decision: Admission, *, bytes_used: int, max_bytes: int
) -> tuple[Path, int]:
    """Atomically save raw/admitted arrays; never put tensors in ordinary logs."""
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
            )
            stream.flush()
            os.fsync(stream.fileno())
        size = temporary.stat().st_size
        if bytes_used + size > max_bytes:
            raise ValueError("prediction artifact byte budget exhausted")
        temporary.replace(path)
        return path, size
    finally:
        temporary.unlink(missing_ok=True)
