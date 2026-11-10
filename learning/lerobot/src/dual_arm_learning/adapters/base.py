"""Typed model-adapter boundary consumed by the chunked policy pipeline."""

from __future__ import annotations

from typing import Protocol

import numpy as np


class PolicyModelAdapter(Protocol):
    """One model family: raw chunks in physical SI, no session or wire state."""

    policy_family: str
    chunk_size: int
    action_dim: int

    def reset(self) -> None:
        """Discard any private model-side history for a fresh episode."""

    def predict_raw_chunk(
        self, rgb_images: dict[str, np.ndarray], joint_positions: np.ndarray
    ) -> np.ndarray:
        """Return a raw ``(chunk_size, action_dim)`` physical-SI action chunk."""
        ...


__all__ = ["PolicyModelAdapter"]
