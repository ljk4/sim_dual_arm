"""Named ACT adapter over the accepted PolicyProvider boundary.

Production inference is always the 035 ``load_bundle`` + ``predict_action_chunk``
path: no constant, replay, teacher or rescue fallback exists.  Tests may inject a
small typed model object at the same boundary; that object is never selected by
name, environment variable or dynamic import.
"""

from __future__ import annotations

import hashlib
import json
import re
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol

import numpy as np

from .act_contract import ActContract
from .act_temporal import EnsembleResult, TemporalEnsemble, validate_coefficient
from .policy_config import PolicyServiceConfig
from .policy_types import (
    PolicyError,
    ProviderCameraSample,
    ProviderContext,
    ProviderInfo,
    ProviderObservation,
    ProviderPrediction,
    ProviderTarget,
)

_SAFE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$")


def _canonical(value: dict[str, Any]) -> str:
    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()


def decode_rgb(payload: bytes, width: int, height: int) -> np.ndarray:
    """Decode one raw HWC RGB payload to a CHW float32 image in [0,1]."""
    expected = width * height * 3
    if len(payload) != expected:
        raise PolicyError(
            "INVALID_OBSERVATION", f"RGB payload has {len(payload)} bytes, expected {expected}"
        )
    hwc = np.frombuffer(payload, dtype=np.uint8).reshape(height, width, 3)
    return hwc.transpose(2, 0, 1).astype(np.float32) / np.float32(255.0)


class ActModel(Protocol):
    """Small typed boundary used by production and by component tests."""

    def predict(self, images: dict[str, np.ndarray], state: np.ndarray) -> np.ndarray:
        """Return a physical-SI (60,16) chunk for CHW images and 16-state."""
        ...


@dataclass
class _Session:
    context: ProviderContext
    lock: threading.Lock = field(default_factory=threading.Lock)
    last_observation_id: int = 0
    predictions: int = 0
    ensemble: TemporalEnsemble | None = None


def _vram_peak_mib() -> float | None:
    try:
        import torch

        if not torch.cuda.is_available() or not torch.cuda.is_initialized():
            return None
        torch.cuda.synchronize()
        return round(torch.cuda.max_memory_allocated() / (1024 * 1024), 3)
    except Exception:  # noqa: BLE001 - diagnostics must never change inference
        return None


def _as_numpy(value: object) -> np.ndarray:
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()  # type: ignore[union-attr]
    return np.asarray(value)


class BundleActModel:
    """Production adapter: 035 load_bundle/predict_action_chunk only."""

    def __init__(self, handle: Any, device: str = "cpu") -> None:
        self.handle = handle
        self.device = device
        self.predictions = 0

    def predict(self, images: dict[str, np.ndarray], state: np.ndarray) -> np.ndarray:
        # predict_action_chunk accepts HWC float images and applies the training
        # preprocessor/postprocessor exactly once, returning physical SI actions.
        from .bundle import predict_action_chunk

        hwc = {
            name: np.ascontiguousarray(image.transpose(1, 2, 0)) for name, image in images.items()
        }
        observation = {
            "observation.state": np.asarray(state, dtype=np.float32),
            "images": hwc,
        }
        import torch

        with torch.inference_mode():
            result = predict_action_chunk(self.handle, observation)
        self.predictions += 1
        return result


class ActProvider:
    """One model, bounded sessions, serialized GPU calls and no hidden queue."""

    def __init__(
        self,
        config: PolicyServiceConfig,
        contract: ActContract,
        model: ActModel,
        *,
        load_seconds: float = 0.0,
        gripper_max_correction_m: float = 0.0,
        temporal_ensemble_coeff: float | None = None,
        prediction_directory: Path | None = None,
        prediction_max_bytes: int = 268435456,
    ) -> None:
        self.config = config
        self.contract = contract
        self.model = model
        self.load_seconds = float(load_seconds)
        self.warmup_seconds = 0.0
        self.warmup_shape: tuple[int, ...] | None = None
        self.gripper_max_correction_m = gripper_max_correction_m
        self.temporal_ensemble_coeff = (
            None
            if temporal_ensemble_coeff is None
            else validate_coefficient(temporal_ensemble_coeff)
        )
        self.prediction_directory = prediction_directory
        self.prediction_max_bytes = prediction_max_bytes
        self._prediction_bytes = 0
        if not 0.0 <= gripper_max_correction_m <= 0.03:
            raise ValueError("gripper admission must be in [0, 0.03] m")
        if (
            gripper_max_correction_m or self.temporal_ensemble_coeff is not None
        ) and prediction_directory is None:
            raise ValueError("gripper admission/temporal ensembling requires prediction artifacts")
        self._log: Any = None
        self._ready = False
        self._sessions: dict[str, _Session] = {}
        self._sessions_lock = threading.Lock()
        self._inference_lock = threading.Lock()
        self._predictions = 0
        self._warm_start()
        self._ready = True

    # ------------------------------------------------------------------ ready
    def _warm_start(self) -> None:
        started = time.monotonic()
        zeros = {
            camera: np.zeros(
                (3, self.contract.camera_height, self.contract.camera_width),
                dtype=np.float32,
            )
            for camera in self.contract.camera_names
        }
        profile_joints = self.contract.profile.get("joints", [])
        initial = [joint.get("initial") for joint in profile_joints]
        if len(initial) != self.contract.state_dim or any(
            isinstance(value, bool) or not isinstance(value, int | float) for value in initial
        ):
            raise PolicyError("PROVIDER_MISMATCH", "profile initial state is not 16 finite joints")
        state = np.asarray(initial, dtype=np.float32)
        if not np.isfinite(state).all():
            raise PolicyError(
                "PROVIDER_MISMATCH", "profile initial state contains non-finite values"
            )
        output = _as_numpy(self.model.predict(zeros, state))
        expected = (self.contract.chunk_size, self.contract.state_dim)
        if tuple(output.shape) != expected:
            raise PolicyError(
                "PROVIDER_MISMATCH",
                f"warm-up output shape {list(output.shape)} != {list(expected)}",
            )
        if not np.isfinite(output).all():
            raise PolicyError("PROVIDER_MISMATCH", "warm-up output contains non-finite values")
        self.warmup_shape = tuple(int(item) for item in output.shape)
        self.warmup_seconds = time.monotonic() - started

    @property
    def ready(self) -> bool:
        return self._ready

    def info(self) -> ProviderInfo:
        return ProviderInfo(
            protocol_major=self.config.protocol_major,
            model_bundle_digest=self.config.model_bundle_digest,
            processor_digest=self.config.processor_digest,
            observation_schema_hash=self.config.observation_schema_hash,
            action_schema_hash=self.config.action_schema_hash,
            robot_profile_hash=self.config.robot_profile_hash,
            state_joint_names=self.config.joint_names,
            action_joint_names=self.config.action_joint_names,
            required_camera_names=self.config.camera_names,
            action_period_ns=self.config.action_period_ns,
            stateful=self.config.stateful,
            ready=self._ready,
            command_space=self.config.command_space,
            chunk_size=self.config.chunk_size,
            test_only=self.config.test_only,
        )

    def summary(self) -> dict[str, Any]:
        result = self.contract.summary()
        result.update(
            {
                "gripper_max_correction_m": self.gripper_max_correction_m,
                "policy": "act",
                "device": self.config_device,
                "load_seconds": round(self.load_seconds, 6),
                "warmup_seconds": round(self.warmup_seconds, 6),
                "warmup_shape": list(self.warmup_shape or ()),
                "ready": self._ready,
                "temporal_ensemble": self.temporal_ensemble_coeff is not None,
                "temporal_ensemble_coeff": self.temporal_ensemble_coeff,
                "predict_api": "predict_action_chunk",
                "vram_peak_mib": _vram_peak_mib(),
                "predictions": self._predictions,
            }
        )
        return result

    @property
    def config_device(self) -> str:
        return str(getattr(self.model, "device", "unknown"))

    def attach_logger(self, log: Any) -> None:
        self._log = log
        summary = self.summary()
        scalar = {
            key: value
            for key, value in summary.items()
            if isinstance(value, bool | int | float | str) or value is None
        }
        self._emit("act.model.ready", **scalar)

    # -------------------------------------------------------------------- log
    def _emit(self, event: str, **fields: Any) -> None:
        if self._log is None:
            return
        self._log.emit(event, fields=fields, critical=True)

    @staticmethod
    def _context(context: ProviderContext) -> Any:
        from robot_run_logging import EventContext

        return EventContext(
            episode_id=context.episode_id if _SAFE_ID.fullmatch(context.episode_id) else None,
            clock_epoch=context.clock_epoch,
        )

    # ------------------------------------------------------------------- open
    def open(
        self,
        session_id: str,
        context: ProviderContext,
        open_request_id: str,
        robot_profile_json: bytes,
        observation_schema_json: bytes,
        action_schema_json: bytes,
    ) -> None:
        del open_request_id
        if context.robot_profile_hash != self.config.robot_profile_hash:
            raise PolicyError("CONTEXT_MISMATCH", "ACT context robot profile differs")
        try:
            profile = json.loads(robot_profile_json)
            observation = json.loads(observation_schema_json)
            action = json.loads(action_schema_json)
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise PolicyError("SCHEMA_MISMATCH", f"ACT session JSON is invalid: {error}") from error
        if not all(isinstance(item, dict) for item in (profile, observation, action)):
            raise PolicyError("SCHEMA_MISMATCH", "ACT session documents must be objects")
        if _canonical(profile) != self.config.robot_profile_hash:
            raise PolicyError("SCHEMA_MISMATCH", "ACT robot profile digest differs")
        if _canonical(observation) != self.config.observation_schema_hash:
            raise PolicyError("SCHEMA_MISMATCH", "ACT observation schema digest differs")
        if _canonical(action) != self.config.action_schema_hash:
            raise PolicyError("SCHEMA_MISMATCH", "ACT action schema digest differs")
        joints = profile.get("joints")
        if (
            not isinstance(joints, list)
            or tuple(item.get("name") for item in joints if isinstance(item, dict))
            != self.config.joint_names
        ):
            raise PolicyError("SCHEMA_MISMATCH", "ACT profile joint order differs")
        cameras = profile.get("cameras")
        if not isinstance(cameras, list):
            raise PolicyError("SCHEMA_MISMATCH", "ACT profile has no cameras")
        expected_calibrations = {
            camera["name"]: camera["calibration_digest"]
            for camera in self.contract.profile["cameras"]
        }
        actual_calibrations = {}
        for camera in cameras:
            if not isinstance(camera, dict):
                raise PolicyError("SCHEMA_MISMATCH", "ACT camera entry must be an object")
            name = camera.get("name")
            if name not in self.config.camera_names:
                raise PolicyError("SCHEMA_MISMATCH", "ACT camera name/order differs")
            if (camera.get("width"), camera.get("height")) != (
                self.config.camera_width,
                self.config.camera_height,
            ):
                raise PolicyError("SCHEMA_MISMATCH", f"ACT camera {name} dimensions differ")
            actual_calibrations[name] = camera.get("calibration_digest")
        if actual_calibrations != expected_calibrations:
            raise PolicyError("SCHEMA_MISMATCH", "ACT camera calibration differs from the profile")
        with self._sessions_lock:
            if session_id in self._sessions:
                raise PolicyError("INVALID_REQUEST", "ACT session already exists")
            ensemble = (
                TemporalEnsemble(self.temporal_ensemble_coeff)
                if self.temporal_ensemble_coeff is not None
                else None
            )
            self._sessions[session_id] = _Session(context=context, ensemble=ensemble)
        self._emit(
            "act.session.opened",
            episode_id=context.episode_id if _SAFE_ID.fullmatch(context.episode_id) else "",
            clock_epoch=context.clock_epoch,
            control_epoch=context.control_epoch,
            sessions=len(self._sessions),
        )

    # ---------------------------------------------------------------- predict
    def predict(
        self,
        session_id: str,
        request_id: str,
        context: ProviderContext,
        observation: ProviderObservation,
    ) -> ProviderPrediction:
        with self._sessions_lock:
            session = self._sessions.get(session_id)
        if session is None:
            raise PolicyError("UNKNOWN_SESSION", "ACT provider does not know this session")
        with session.lock:
            if context != session.context:
                raise PolicyError("STALE_CONTEXT", "ACT prediction context differs")
            if observation.observation_id <= session.last_observation_id:
                raise PolicyError(
                    "INVALID_OBSERVATION",
                    "ACT observations must advance monotonically in one session",
                )
            images = self._decode(observation.cameras)
            state = np.asarray(observation.joints.positions, dtype=np.float32)
            if state.shape != (len(self.config.action_joint_names),):
                raise PolicyError("INVALID_OBSERVATION", "ACT state must be 16-wide")
            started = time.monotonic_ns()
            with self._inference_lock:
                model_started = time.monotonic_ns()
                try:
                    output = _as_numpy(self.model.predict(images, state))
                except PolicyError:
                    raise
                except Exception as error:  # noqa: BLE001 - model failure is explicit
                    self._emit(
                        "act.predict.rejected",
                        reason="MODEL_ERROR",
                        error_type=type(error).__name__,
                        observation_id=observation.observation_id,
                    )
                    raise PolicyError("PROVIDER_ERROR", f"ACT model failed: {error}") from error
                model_elapsed = time.monotonic_ns() - model_started
            if output.shape != (self.config.chunk_size, len(self.config.action_joint_names)):
                self._emit(
                    "act.predict.rejected",
                    reason="INVALID_PREDICTION",
                    detail=f"shape={list(output.shape)}",
                    observation_id=observation.observation_id,
                )
                raise PolicyError(
                    "INVALID_PREDICTION",
                    f"ACT model returned {list(output.shape)}, expected (60,16)",
                )
            if not np.isfinite(output).all():
                self._emit(
                    "act.predict.rejected",
                    reason="INVALID_PREDICTION",
                    detail="non-finite",
                    observation_id=observation.observation_id,
                )
                raise PolicyError("INVALID_PREDICTION", "ACT model returned non-finite actions")
            ensemble_enabled = self.temporal_ensemble_coeff is not None
            ensemble_result = None
            decision = None
            if self.gripper_max_correction_m or ensemble_enabled:
                from .act_admission import admit_grippers, save_prediction

                decision = admit_grippers(
                    output,
                    self.config.action_joint_names,
                    self.config.joint_lower_bounds,
                    self.config.joint_upper_bounds,
                    self.gripper_max_correction_m,
                )
                if decision.accepted and ensemble_enabled:
                    ensemble_result = self._preview_ensemble(
                        session, observation, decision.positions
                    )
                extras = {}
                if ensemble_result is not None:
                    extras = {
                        "execution_targets": ensemble_result.targets,
                        "ensemble_counts": ensemble_result.counts,
                        "history_observation_ids": ensemble_result.history_observation_ids,
                        "history_observation_time_ns": (
                            ensemble_result.history_observation_time_ns
                        ),
                    }
                artifact, size = save_prediction(
                    self.prediction_directory,
                    session_id + ":" + request_id,
                    output,
                    decision,
                    bytes_used=self._prediction_bytes,
                    max_bytes=self.prediction_max_bytes,
                    **extras,
                )
                self._prediction_bytes += size
                self._emit(
                    "act.admission.finished",
                    request_id=request_id,
                    observation_id=observation.observation_id,
                    accepted=decision.accepted,
                    artifact=str(artifact),
                    corrected_scalars=int(np.count_nonzero(decision.correction)),
                    max_correction_m=float(np.max(np.abs(decision.correction))),
                    temporal_ensemble=ensemble_enabled,
                    temporal_ensemble_coeff=self.temporal_ensemble_coeff,
                    ensemble_history_sources=(
                        0
                        if ensemble_result is None
                        else int(ensemble_result.history_observation_ids.size)
                    ),
                )
                if not decision.accepted:
                    raise PolicyError("INVALID_PREDICTION", decision.reason)
                output = decision.positions if ensemble_result is None else ensemble_result.targets
            period = self.config.action_period_ns
            targets = tuple(
                ProviderTarget(
                    (index + 1) * period,
                    tuple(float(value) for value in output[index]),
                )
                for index in range(self.config.chunk_size)
            )
            elapsed = time.monotonic_ns() - started
            if elapsed > self.config.predict_deadline_s * 1_000_000_000:
                self._emit(
                    "act.predict.rejected",
                    reason="DEADLINE_EXCEEDED",
                    observation_id=observation.observation_id,
                    latency_ns=elapsed,
                )
                raise PolicyError(
                    "DEADLINE_EXCEEDED", "ACT model exceeded the configured prediction deadline"
                )
            session.last_observation_id = observation.observation_id
            session.predictions += 1
            self._predictions += 1
            self._emit(
                "act.predict.finished",
                episode_id=context.episode_id if _SAFE_ID.fullmatch(context.episode_id) else "",
                observation_id=observation.observation_id,
                request_id=request_id if _SAFE_ID.fullmatch(request_id) else "",
                chunk_size=self.config.chunk_size,
                latency_ns=elapsed,
                model_latency_ns=model_elapsed,
                vram_peak_mib=_vram_peak_mib(),
                predictions=session.predictions,
                temporal_ensemble=ensemble_enabled,
                temporal_ensemble_coeff=self.temporal_ensemble_coeff,
                ensemble_max_sources=(
                    0 if ensemble_result is None else int(ensemble_result.counts.max())
                ),
                ensemble_history_sources=(
                    0
                    if ensemble_result is None
                    else int(ensemble_result.history_observation_ids.size)
                ),
            )
            if ensemble_result is not None:
                # History is committed only after artifact, logs and deadline
                # succeeded; a rejected call can never poison later predictions.
                try:
                    session.ensemble.commit(
                        observation.reference_robot_time_ns,
                        observation.observation_id,
                        decision.positions,
                    )
                except ValueError as error:
                    raise PolicyError(
                        "INVALID_OBSERVATION", f"temporal ensemble commit: {error}"
                    ) from error
            return ProviderPrediction(
                joint_names=self.config.action_joint_names,
                targets=targets,
                model_bundle_digest=self.config.model_bundle_digest,
                inference_duration_ns=model_elapsed,
                context=context,
                session_id=session_id,
                request_id=request_id,
                observation_id=observation.observation_id,
            )

    def _preview_ensemble(
        self,
        session: _Session,
        observation: ProviderObservation,
        admitted: np.ndarray,
    ) -> EnsembleResult:
        """Validate absolute observation time and fuse without committing."""
        if session.ensemble is None:  # pragma: no cover - constructor invariant
            raise PolicyError("PROVIDER_ERROR", "ACT temporal ensemble state is missing")
        try:
            return session.ensemble.preview(
                observation.reference_robot_time_ns,
                observation.observation_id,
                admitted,
            )
        except ValueError as error:
            self._emit(
                "act.predict.rejected",
                reason="INVALID_OBSERVATION",
                detail="temporal_ensemble",
                observation_id=observation.observation_id,
            )
            raise PolicyError("INVALID_OBSERVATION", f"temporal ensemble: {error}") from error

    def _decode(self, cameras: tuple[ProviderCameraSample, ...]) -> dict[str, np.ndarray]:
        if tuple(camera.camera_name for camera in cameras) != self.config.camera_names:
            raise PolicyError("INVALID_OBSERVATION", "ACT camera names/order differ")
        images: dict[str, np.ndarray] = {}
        for camera in cameras:
            if (camera.width, camera.height) != (
                self.config.camera_width,
                self.config.camera_height,
            ):
                raise PolicyError(
                    "INVALID_OBSERVATION", f"ACT camera {camera.camera_name} dimensions differ"
                )
            images[camera.camera_name] = decode_rgb(
                camera.payload, self.config.camera_width, self.config.camera_height
            )
        return images

    # ------------------------------------------------------------------ reset
    def reset(self, session_id: str, reset_id: str, context: ProviderContext) -> None:
        del reset_id
        with self._sessions_lock:
            session = self._sessions.get(session_id)
        if session is None:
            raise PolicyError("UNKNOWN_SESSION", "ACT reset session is unknown")
        with session.lock:
            previous_predictions = session.predictions
            session.context = context
            session.last_observation_id = 0
            session.predictions = 0
            if session.ensemble is not None:
                session.ensemble.reset()
        self._emit(
            "act.session.reset",
            episode_id=context.episode_id if _SAFE_ID.fullmatch(context.episode_id) else "",
            clock_epoch=context.clock_epoch,
            control_epoch=context.control_epoch,
            previous_predictions=previous_predictions,
        )

    # ------------------------------------------------------------------ close
    def close(self, session_id: str) -> None:
        with self._sessions_lock:
            session = self._sessions.pop(session_id, None)
        if session is not None:
            self._emit(
                "act.session.closed",
                episode_id=(
                    session.context.episode_id
                    if _SAFE_ID.fullmatch(session.context.episode_id)
                    else ""
                ),
                predictions=session.predictions,
                sessions=len(self._sessions),
            )


def load_act_provider(
    *,
    config: PolicyServiceConfig,
    contract: ActContract,
    device: str,
    gripper_max_correction_m: float = 0.0,
    temporal_ensemble_coeff: float | None = None,
    prediction_directory: Path | None = None,
    prediction_max_bytes: int = 268435456,
) -> ActProvider:
    """Strictly load the 035 bundle, warm the model, then announce readiness."""
    started = time.monotonic()
    try:
        import torch
    except ImportError as error:  # pragma: no cover - production image has Torch
        raise PolicyError("PROVIDER_MISMATCH", "Torch is required for the ACT provider") from error
    if device.startswith("cuda"):
        if not torch.cuda.is_available():
            raise PolicyError("PROVIDER_MISMATCH", f"requested {device} but CUDA is unavailable")
        torch.cuda.reset_peak_memory_stats()
    from .bundle import load_bundle

    handle = load_bundle(contract.bundle_root, device=device)
    model = BundleActModel(handle, device=device)
    provider = ActProvider(
        config,
        contract,
        model,
        load_seconds=time.monotonic() - started,
        gripper_max_correction_m=gripper_max_correction_m,
        temporal_ensemble_coeff=temporal_ensemble_coeff,
        prediction_directory=prediction_directory,
        prediction_max_bytes=prediction_max_bytes,
    )
    return provider


__all__ = [
    "ActModel",
    "ActProvider",
    "BundleActModel",
    "decode_rgb",
    "load_act_provider",
]
