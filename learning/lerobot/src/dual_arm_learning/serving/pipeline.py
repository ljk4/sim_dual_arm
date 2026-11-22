"""Chunked policy pipeline: decode, infer, admit, fuse and persist evidence."""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from ..adapters.base import PolicyModelAdapter
from .chunk_admission import admit_action_chunk, save_prediction
from .config import PolicyServiceConfig
from .temporal_ensemble import EnsembleResult, TemporalEnsemble, validate_coefficient
from .types import PolicyError, ProviderCameraSample


def decode_rgb(payload: bytes, width: int, height: int) -> np.ndarray:
    """Decode one raw HWC RGB payload to a CHW float32 image in [0,1]."""
    expected = width * height * 3
    if len(payload) != expected:
        raise PolicyError(
            "INVALID_OBSERVATION", f"RGB payload has {len(payload)} bytes, expected {expected}"
        )
    hwc = np.frombuffer(payload, dtype=np.uint8).reshape(height, width, 3)
    return hwc.transpose(2, 0, 1).astype(np.float32) / np.float32(255.0)


@dataclass(slots=True)
class PredictionBudget:
    """Shared byte budget for every prediction artifact of one service."""

    used: int = 0


@dataclass(frozen=True, slots=True)
class PipelineResult:
    """One admitted execution chunk plus the evidence that produced it."""

    targets: np.ndarray
    raw: np.ndarray
    admitted: np.ndarray
    correction: np.ndarray
    inference_duration_ns: int
    observation_id: int
    observation_time_ns: int
    ensemble: EnsembleResult | None = None
    artifact: Path | None = None


class ChunkedPolicyPipeline:
    """Session-scoped chunk pipeline over one stateless model adapter."""

    def __init__(
        self,
        adapter: PolicyModelAdapter,
        config: PolicyServiceConfig,
        *,
        prediction_directory: Path | None = None,
        prediction_max_bytes: int = 268435456,
        budget: PredictionBudget | None = None,
    ) -> None:
        if adapter.action_dim != len(config.action_joint_names):
            raise ValueError("adapter action_dim differs from the configured action joints")
        if (config.max_prediction_corrections or config.temporal_ensemble_coeff is not None) and (
            prediction_directory is None
        ):
            raise ValueError("prediction admission requires prediction artifacts")
        self.adapter, self.config = adapter, config
        self.prediction_directory = prediction_directory
        self.prediction_max_bytes = prediction_max_bytes
        self.budget = budget or PredictionBudget()
        self._ensemble = (
            None
            if config.temporal_ensemble_coeff is None
            else TemporalEnsemble(
                validate_coefficient(config.temporal_ensemble_coeff),
                chunk_size=config.chunk_size,
                action_dim=len(config.action_joint_names),
                period_ns=config.action_period_ns,
            )
        )

    def reset(self) -> None:
        self.adapter.reset()
        if self._ensemble is not None:
            self._ensemble.reset()

    def warm_up(self) -> None:
        """Warm up the adapter on the active RPC worker thread without recording state."""
        warm_up = getattr(self.adapter, "warm_up", None)
        if callable(warm_up):
            previous = getattr(self.adapter, "predictions", None)
            warm_up()
            if isinstance(previous, int) and previous > 0:
                self.adapter.predictions = previous

    def _prepare_inputs(
        self,
        cameras: tuple[ProviderCameraSample, ...],
        joint_positions: tuple[float, ...],
    ) -> tuple[dict[str, np.ndarray], np.ndarray]:
        config = self.config
        if tuple(camera.camera_name for camera in cameras) != config.camera_names:
            raise PolicyError("INVALID_OBSERVATION", "camera names/order differ")
        images = {
            camera.camera_name: decode_rgb(
                camera.payload, config.camera_width, config.camera_height
            )
            for camera in cameras
        }
        state = np.asarray(joint_positions, dtype=np.float32)
        if state.shape != (len(config.action_joint_names),):
            raise PolicyError("INVALID_OBSERVATION", "state must cover every action joint")
        return images, state

    def _run_adapter(
        self, images: dict[str, np.ndarray], state: np.ndarray
    ) -> tuple[np.ndarray, int]:
        started = time.monotonic_ns()
        try:
            output = np.asarray(self.adapter.predict_raw_chunk(images, state))
        except PolicyError:
            raise
        except Exception as error:  # noqa: BLE001 - model failure is explicit
            raise PolicyError("PROVIDER_ERROR", f"model failed: {error}") from error
        inference_duration_ns = time.monotonic_ns() - started
        expected = (self.adapter.chunk_size, len(self.config.action_joint_names))
        if tuple(output.shape) != expected:
            raise PolicyError(
                "INVALID_PREDICTION",
                f"model returned {list(output.shape)}, expected {list(expected)}",
            )
        if not np.isfinite(output).all():
            raise PolicyError("INVALID_PREDICTION", "model returned non-finite actions")
        return output, inference_duration_ns

    def _admit(
        self, output: np.ndarray, observation_time_ns: int, observation_id: int
    ) -> tuple[Any, Any, bool]:
        config = self.config
        evidence_enabled = bool(
            config.max_prediction_corrections or self._ensemble is not None
        )
        decision = admit_action_chunk(
            output,
            config.action_joint_names,
            config.joint_lower_bounds,
            config.joint_upper_bounds,
            config.max_prediction_corrections,
        )
        ensemble_result = None
        if decision.accepted and self._ensemble is not None:
            try:
                ensemble_result = self._ensemble.preview(
                    observation_time_ns, observation_id, decision.positions
                )
            except ValueError as error:
                raise PolicyError("INVALID_OBSERVATION", f"temporal ensemble: {error}") from error
        return decision, ensemble_result, evidence_enabled

    def _save_artifact(
        self,
        identity: str,
        output: np.ndarray,
        decision: Any,
        ensemble_result: Any,
        evidence_enabled: bool,
    ) -> Any:
        if not (evidence_enabled and self.prediction_directory is not None):
            return None
        extras = {}
        if ensemble_result is not None:
            extras = {
                "execution_targets": ensemble_result.targets,
                "ensemble_counts": ensemble_result.counts,
                "history_observation_ids": ensemble_result.history_observation_ids,
                "history_observation_time_ns": ensemble_result.history_observation_time_ns,
            }
        artifact, size = save_prediction(
            self.prediction_directory,
            identity,
            output,
            decision,
            bytes_used=self.budget.used,
            max_bytes=self.prediction_max_bytes,
            **extras,
        )
        self.budget.used += size
        return artifact

    def predict(
        self,
        *,
        identity: str,
        observation_id: int,
        observation_time_ns: int,
        cameras: tuple[ProviderCameraSample, ...],
        joint_positions: tuple[float, ...],
    ) -> PipelineResult:
        """Run one raw chunk through admission, temporal fusion and evidence."""
        images, state = self._prepare_inputs(cameras, joint_positions)
        output, inference_duration_ns = self._run_adapter(images, state)
        decision, ensemble_result, evidence_enabled = self._admit(
            output, observation_time_ns, observation_id
        )
        artifact = self._save_artifact(
            identity, output, decision, ensemble_result, evidence_enabled
        )
        if not decision.accepted:
            raise PolicyError("INVALID_PREDICTION", decision.reason)
        targets = decision.positions if ensemble_result is None else ensemble_result.targets
        return PipelineResult(
            targets=targets,
            raw=np.asarray(output, dtype=np.float64),
            admitted=decision.positions,
            correction=decision.correction,
            inference_duration_ns=inference_duration_ns,
            observation_id=observation_id,
            observation_time_ns=observation_time_ns,
            ensemble=ensemble_result,
            artifact=artifact,
        )

    def commit(self, result: PipelineResult) -> None:
        """Store the admitted chunk in history once every later check passed."""
        if self._ensemble is not None and result.ensemble is not None:
            try:
                self._ensemble.commit(
                    result.observation_time_ns, result.observation_id, result.admitted
                )
            except ValueError as error:
                raise PolicyError(
                    "INVALID_OBSERVATION", f"temporal ensemble commit: {error}"
                ) from error


__all__ = ["ChunkedPolicyPipeline", "PipelineResult", "PredictionBudget", "decode_rgb"]
