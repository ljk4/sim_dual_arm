"""Deterministic test-only Policy adapter with optional recorded-label replay."""

from __future__ import annotations

import json
import math
import os
import time
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
import yaml
from dual_arm_learning.adapters.base import PolicyModelAdapter
from dual_arm_learning.serving.config import PolicyServiceConfig, parse_policy_config
from dual_arm_learning.serving.types import (
    ProviderObservation,
    ProviderPrediction,
    ProviderTarget,
    RunContext,
)

_FAULTS = {
    "none",
    "disconnect",
    "wrong_joint_order",
    "wrong_model_digest",
    "short_chunk",
    "non_uniform_time",
    "nan_position",
    "late",
    "wrong_observation",
    "wrong_session",
    "wrong_context",
    "wrong_epoch",
}


@dataclass(frozen=True, slots=True)
class FakeSettings:
    amplitude_rad: float = 0.01
    amplitude_m: float = 0.001
    latency_s: float = 0.01
    fault: str = "none"
    stop_after_predictions: int = 0
    replay_labels: str = ""

    def __post_init__(self) -> None:
        if self.fault not in _FAULTS:
            raise ValueError(f"unsupported fake fault {self.fault!r}")
        if not 0.0 <= self.latency_s <= 30.0:
            raise ValueError("latency_s must be within [0, 30]")
        if self.stop_after_predictions < 0:
            raise ValueError("stop_after_predictions must be non-negative")


def load_fake_config(path: str | Path) -> tuple[PolicyServiceConfig, FakeSettings]:
    document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("fake policy config must be an object")
    unknown = set(document) - {"service", "fake"}
    if unknown:
        raise ValueError(f"unknown fake policy field(s): {sorted(unknown)}")
    service = parse_policy_config(document["service"])
    raw = document.get("fake", {})
    if not isinstance(raw, dict):
        raise ValueError("fake must be an object")
    allowed = {
        "amplitude_rad",
        "amplitude_m",
        "latency_s",
        "fault",
        "stop_after_predictions",
        "replay_labels",
    }
    unknown = set(raw) - allowed
    if unknown:
        raise ValueError(f"unknown fake field(s): {sorted(unknown)}")
    return service, FakeSettings(
        amplitude_rad=float(raw.get("amplitude_rad", 0.01)),
        amplitude_m=float(raw.get("amplitude_m", 0.001)),
        latency_s=float(raw.get("latency_s", 0.01)),
        fault=str(raw.get("fault", "none")),
        stop_after_predictions=int(raw.get("stop_after_predictions", 0)),
        replay_labels=str(raw.get("replay_labels", "")),
    )


class FakeProvider(PolicyModelAdapter):
    """One deterministic test-only PolicyModelAdapter with optional fault injection."""

    policy_family = "act"

    def __init__(
        self,
        config: PolicyServiceConfig,
        settings: FakeSettings | None = None,
        *,
        fault: str | None = None,
    ) -> None:
        self.config = config
        self.chunk_size = config.chunk_size
        self.action_dim = len(config.action_joint_names)
        self.settings = settings or FakeSettings()
        if fault is not None:
            self.settings = FakeSettings(
                self.settings.amplitude_rad,
                self.settings.amplitude_m,
                self.settings.latency_s,
                fault,
                self.settings.stop_after_predictions,
                self.settings.replay_labels,
            )
        self._sessions: set[str] = set()
        self.predictions = 0
        self.closed: list[str] = []
        self.replay_labels = self._load_replay(self.settings.replay_labels, self.action_dim)
        self._replay_starts: dict[str, int] = {}
        self._current_session_id: str = ""
        self._current_observation: ProviderObservation | None = None

    @staticmethod
    def _load_replay(value: str, action_dim: int = 16) -> dict[int, tuple[float, ...]]:
        if not value:
            return {}
        if value.startswith("env:"):
            value = os.environ.get(value[4:], "")
        if not value:
            raise ValueError("replay_labels resolved to an empty path")
        document = json.loads(Path(value).read_text(encoding="utf-8"))
        rows = document.get("labels") if isinstance(document, dict) else None
        if not isinstance(rows, list) or not rows:
            raise ValueError("replay labels export has no labels")
        result: dict[int, tuple[float, ...]] = {}
        for row in rows:
            if not isinstance(row, dict):
                continue
            label_time = int(row["label_backend_time_ns"])
            positions = tuple(float(item) for item in row["applied_positions"])
            if len(positions) != action_dim:
                raise ValueError(f"replay labels are not {action_dim}-wide")
            result[label_time] = positions
        if not result:
            raise ValueError("replay labels export has no usable rows")
        return result

    def open_session(self, session_id: str) -> None:
        self._sessions.add(session_id)

    def set_current_observation(
        self,
        session_id: str,
        request_id: str,
        context: RunContext,
        observation: ProviderObservation,
    ) -> None:
        del request_id, context
        self._current_session_id = session_id
        self._current_observation = observation

    def reset(self) -> None:
        self._replay_starts.pop(self._current_session_id, None)

    def close(self, session_id: str) -> None:
        self.closed.append(session_id)
        self._sessions.discard(session_id)
        self._replay_starts.pop(session_id, None)

    def predict_raw_chunk(
        self, rgb_images: dict[str, np.ndarray], joint_positions: np.ndarray
    ) -> np.ndarray:
        del rgb_images
        if self.settings.fault == "disconnect":
            raise ConnectionError("fake policy connection dropped")
        if self.settings.fault == "late":
            time.sleep(max(self.settings.latency_s, 0.5))
        elif self.settings.latency_s:
            time.sleep(self.settings.latency_s)
        self.predictions += 1
        if (
            self.settings.stop_after_predictions
            and self.predictions > self.settings.stop_after_predictions
        ):
            raise ConnectionError("fake provider reached its configured prediction limit")
        chunk = self.config.chunk_size
        period = self.config.action_period_ns
        base = [float(v) for v in joint_positions]
        if self.replay_labels and self._current_observation is not None:
            ordered = sorted(self.replay_labels)
            obs_time = self._current_observation.reference_robot_time_ns
            first_observation = self._replay_starts.setdefault(
                self._current_session_id, obs_time
            )
            source_observation = ordered[0] - period
            elapsed = obs_time - first_observation
            rows: list[tuple[float, ...]] = []
            for index in range(chunk):
                target_time = source_observation + elapsed + (index + 1) * period
                chosen = ordered[0]
                for label_time in ordered:
                    if label_time <= target_time:
                        chosen = label_time
                    else:
                        break
                rows.append(self.replay_labels[chosen])
            return np.asarray(rows, dtype=np.float32)

        result = np.empty((chunk, self.action_dim), dtype=np.float64)
        for index in range(chunk):
            for axis, value in enumerate(base):
                amplitude = (
                    self.settings.amplitude_m
                    if self.config.joint_names[axis].endswith("gripper_width")
                    else self.settings.amplitude_rad
                )
                sign = 1.0 if axis % 2 == 0 else -1.0
                offset = amplitude * sign * math.sin(2.0 * math.pi * (index + 1) / chunk)
                number = min(
                    max(value + offset, self.config.joint_lower_bounds[axis]),
                    self.config.joint_upper_bounds[axis],
                )
                if self.settings.fault == "nan_position" and index == 0 and axis == 0:
                    number = math.nan
                result[index, axis] = number
        if self.settings.fault == "short_chunk":
            return result[:-1].astype(np.float32)
        return result.astype(np.float32)

    def transform_prediction(
        self,
        session_id: str,
        request_id: str,
        context: RunContext,
        observation: ProviderObservation,
        prediction: ProviderPrediction,
    ) -> ProviderPrediction:
        del request_id
        fault = self.settings.fault
        if fault == "wrong_joint_order":
            names = prediction.joint_names
            return replace(prediction, joint_names=(names[1], names[0], *names[2:]))
        if fault == "wrong_model_digest":
            return replace(prediction, model_bundle_digest="0" * 64)
        if fault == "non_uniform_time":
            targets = list(prediction.targets)
            targets[10] = ProviderTarget(
                targets[10].time_from_observation_ns + self.config.action_period_ns // 2,
                targets[10].positions,
            )
            return replace(prediction, targets=tuple(targets))
        if fault == "wrong_observation":
            return replace(prediction, observation_id=observation.observation_id + 1)
        if fault == "wrong_session":
            return replace(prediction, session_id=session_id + "-old")
        if fault in {"wrong_context", "wrong_epoch"}:
            bad_context = RunContext(
                robot_id=context.robot_id,
                robot_profile_hash=context.robot_profile_hash,
                backend_instance_id=context.backend_instance_id,
                time_domain_id=context.time_domain_id,
                clock_epoch=max(0, context.clock_epoch - 1),
                episode_id=context.episode_id,
                control_epoch=context.control_epoch,
            )
            return replace(prediction, context=bad_context)
        return prediction


__all__ = ["FakeProvider", "FakeSettings", "load_fake_config"]
