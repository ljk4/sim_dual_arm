"""Lean LeRobot ACT model adapter: raw chunks only, no session or wire state."""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import numpy as np

from ..serving.types import PolicyError
from .base import PolicyModelAdapter


def _vram_peak_mib() -> float | None:
    try:
        import torch

        if not torch.cuda.is_available() or not torch.cuda.is_initialized():
            return None
        torch.cuda.synchronize()
        return round(torch.cuda.max_memory_allocated() / (1024 * 1024), 3)
    except Exception:  # noqa: BLE001 - diagnostics must never change inference
        return None


class ActModelAdapter(PolicyModelAdapter):
    """Production ACT adapter over the ``predict_action_chunk`` path."""

    policy_family = "act"

    def __init__(
        self,
        handle: Any,
        *,
        device: str,
        chunk_size: int,
        action_dim: int,
        camera_names: tuple[str, ...],
        camera_width: int,
        camera_height: int,
        initial_positions: tuple[float, ...],
        load_seconds: float = 0.0,
    ) -> None:
        self.handle = handle
        self.device = device
        self.chunk_size = chunk_size
        self.action_dim = action_dim
        self.camera_names = camera_names
        self.camera_width, self.camera_height = camera_width, camera_height
        state = np.asarray(initial_positions, dtype=np.float32)
        if state.shape != (action_dim,) or not np.isfinite(state).all():
            raise PolicyError(
                "PROVIDER_MISMATCH",
                f"profile initial state is not {action_dim} finite joints",
            )
        self.initial_state = state
        self.load_seconds = float(load_seconds)
        self.warmup_seconds = 0.0
        self.warmup_shape: tuple[int, ...] | None = None
        self.predictions = 0
        self._log: Any = None

    def reset(self) -> None:
        """ACT chunks have no model-side history; a fresh call is already reset."""

    def predict_raw_chunk(
        self, rgb_images: dict[str, np.ndarray], joint_positions: np.ndarray
    ) -> np.ndarray:
        from ..training.bundle import predict_action_chunk

        observation = {
            "observation.state": np.asarray(joint_positions, dtype=np.float32),
            "images": rgb_images,
        }
        import torch

        with torch.inference_mode():
            result = predict_action_chunk(self.handle, observation)
        self.predictions += 1
        return result

    def warm_up(self) -> None:
        """Prove one real chunk has the configured shape before declaring ready."""
        started = time.monotonic()
        zeros = {
            camera: np.zeros((3, self.camera_height, self.camera_width), dtype=np.float32)
            for camera in self.camera_names
        }
        output = np.asarray(self.predict_raw_chunk(zeros, self.initial_state))
        expected = (self.chunk_size, self.action_dim)
        if tuple(output.shape) != expected:
            raise PolicyError(
                "PROVIDER_MISMATCH",
                f"warm-up output shape {list(output.shape)} != {list(expected)}",
            )
        if not np.isfinite(output).all():
            raise PolicyError("PROVIDER_MISMATCH", "warm-up output contains non-finite values")
        self.predictions = 0
        self.warmup_shape = tuple(int(item) for item in output.shape)
        self.warmup_seconds = time.monotonic() - started

    def summary(self) -> dict[str, Any]:
        return {
            "policy": self.policy_family,
            "device": self.device,
            "chunk_size": self.chunk_size,
            "action_dim": self.action_dim,
            "cameras": list(self.camera_names),
            "load_seconds": round(self.load_seconds, 6),
            "warmup_seconds": round(self.warmup_seconds, 6),
            "warmup_shape": list(self.warmup_shape or ()),
            "vram_peak_mib": _vram_peak_mib(),
        }

    def attach_logger(self, log: Any) -> None:
        self._log = log
        scalar = {
            key: value
            for key, value in self.summary().items()
            if isinstance(value, bool | int | float | str) or value is None
        }
        log.emit("act.model.ready", fields=scalar, critical=True)


def load_act_adapter(config: Any, *, device: str | None = None) -> ActModelAdapter:
    """Strictly load the model package, warm the model, then return the adapter."""
    device = config.device if device is None else device
    try:
        import torch
    except ImportError as error:  # pragma: no cover - production image has Torch
        raise PolicyError("PROVIDER_MISMATCH", "Torch is required for the ACT adapter") from error
    if device.startswith("cuda"):
        if not torch.cuda.is_available():
            raise PolicyError("PROVIDER_MISMATCH", f"requested {device} but CUDA is unavailable")
        torch.cuda.reset_peak_memory_stats()
    started = time.monotonic()
    from ..training.bundle import load_model

    handle = load_model(config.model_path, device=device)
    profile = json.loads(Path(config.robot_profile_path).read_text(encoding="utf-8"))
    adapter = ActModelAdapter(
        handle,
        device=device,
        chunk_size=config.chunk_size,
        action_dim=len(config.action_joint_names),
        camera_names=config.camera_names,
        camera_width=config.camera_width,
        camera_height=config.camera_height,
        initial_positions=tuple(joint["initial"] for joint in profile["joints"]),
        load_seconds=time.monotonic() - started,
    )
    adapter.warm_up()
    return adapter


__all__ = ["ActModelAdapter", "load_act_adapter"]
