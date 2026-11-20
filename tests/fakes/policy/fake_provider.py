"""Deterministic test-only Policy provider with optional recorded-label replay."""

from __future__ import annotations

import json
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path

import yaml
from dual_arm_learning.policy_config import PolicyServiceConfig, parse_policy_config
from dual_arm_learning.policy_types import (
    PolicyError,
    ProviderContext,
    ProviderInfo,
    ProviderObservation,
    ProviderPrediction,
    ProviderTarget,
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
    document = yaml.safe_load(Path(path).read_text())
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


class FakeProvider:
    """One deterministic provider; every output is explicitly test_only."""

    def __init__(
        self,
        config: PolicyServiceConfig,
        settings: FakeSettings | None = None,
        *,
        fault: str | None = None,
    ) -> None:
        self.config = config
        self.settings = settings or FakeSettings()
        if fault is not None:
            self.settings = FakeSettings(
                self.settings.amplitude_rad,
                self.settings.amplitude_m,
                self.settings.latency_s,
                fault,
                self.settings.stop_after_predictions,
            )
        self._sessions: set[str] = set()
        self.predictions = 0
        self.closed: list[str] = []
        self.replay_labels = self._load_replay(self.settings.replay_labels)
        self._replay_starts: dict[str, int] = {}

    @staticmethod
    def _load_replay(value: str) -> dict[int, tuple[float, ...]]:
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
            label_time = int(row["label_sim_time_ns"])
            positions = tuple(float(value) for value in row["applied_positions"])
            if len(positions) != 16:
                raise ValueError("replay labels are not 16-wide")
            result[label_time] = positions
        if not result:
            raise ValueError("replay labels export has no usable rows")
        return result

    def info(self) -> ProviderInfo:
        config = self.config
        return ProviderInfo(
            model_bundle_digest=config.model_bundle_digest,
            processor_digest=config.processor_digest,
            observation_schema_hash=config.observation_schema_hash,
            action_schema_hash=config.action_schema_hash,
            robot_profile_hash=config.robot_profile_hash,
            state_joint_names=config.joint_names,
            action_joint_names=config.action_joint_names,
            required_camera_names=config.camera_names,
            action_period_ns=config.action_period_ns,
            stateful=config.stateful,
            ready=True,
            command_space=config.command_space,
            chunk_size=config.chunk_size,
            test_only=config.test_only,
        )

    def open(
        self,
        session_id: str,
        context: ProviderContext,
        open_request_id: str,
        robot_profile_json: bytes,
        observation_schema_json: bytes,
        action_schema_json: bytes,
    ) -> None:
        del (
            context,
            open_request_id,
            robot_profile_json,
            observation_schema_json,
            action_schema_json,
        )
        self._sessions.add(session_id)

    def predict(
        self,
        session_id: str,
        request_id: str,
        context: ProviderContext,
        observation: ProviderObservation,
    ) -> ProviderPrediction:
        del request_id
        if self.settings.fault == "disconnect":
            raise ConnectionError("fake policy connection dropped")
        if self.settings.fault == "late":
            time.sleep(max(self.settings.latency_s, 0.5))
        elif self.settings.latency_s:
            time.sleep(self.settings.latency_s)
        if session_id not in self._sessions:
            raise PolicyError("UNKNOWN_SESSION", "fake provider does not know this session")
        self.predictions += 1
        if (
            self.settings.stop_after_predictions
            and self.predictions > self.settings.stop_after_predictions
        ):
            raise ConnectionError("fake provider reached its configured prediction limit")
        names = tuple(observation.joints.joint_names)
        if self.settings.fault == "wrong_joint_order":
            names = (names[1], names[0], *names[2:])
        chunk = self.config.chunk_size
        period = self.config.action_period_ns
        base = observation.joints.positions
        targets: list[ProviderTarget] = []
        if self.replay_labels:
            ordered = sorted(self.replay_labels)
            first_observation = self._replay_starts.setdefault(
                session_id, observation.reference_robot_time_ns
            )
            source_observation = ordered[0] - period
            elapsed = observation.reference_robot_time_ns - first_observation
            for index in range(chunk):
                target_time = source_observation + elapsed + (index + 1) * period
                chosen = ordered[0]
                for label_time in ordered:
                    if label_time <= target_time:
                        chosen = label_time
                    else:
                        break
                targets.append(
                    ProviderTarget((index + 1) * period, self.replay_labels[chosen])
                )
            return self._prediction(
                session_id, context, observation, names, targets
            )
        for index in range(chunk):
            positions: list[float] = []
            for axis, value in enumerate(base):
                amplitude = (
                    self.settings.amplitude_m
                    if self.config.joint_names[axis].endswith(
                        "gripper_width"
                    )
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
                positions.append(number)
            relative = (index + 1) * period
            if self.settings.fault == "non_uniform_time" and index == 10:
                relative += period // 2
            targets.append(ProviderTarget(relative, tuple(positions)))
        if self.settings.fault == "short_chunk":
            targets = targets[:-1]
        return self._prediction(session_id, context, observation, names, targets)

    def _prediction(
        self,
        session_id: str,
        context: ProviderContext,
        observation: ProviderObservation,
        names: tuple[str, ...],
        targets: list[ProviderTarget],
    ) -> ProviderPrediction:
        digest = self.config.model_bundle_digest
        if self.settings.fault == "wrong_model_digest":
            digest = "0" * 64
        observation_id = observation.observation_id
        if self.settings.fault == "wrong_observation":
            observation_id += 1
        returned_context = context
        if self.settings.fault in {"wrong_context", "wrong_epoch"}:
            returned_context = ProviderContext(
                context.robot_id,
                context.robot_profile_hash,
                context.backend_instance_id,
                context.time_domain_id,
                max(0, context.clock_epoch - 1),
                context.episode_id,
                context.control_epoch,
            )
        return ProviderPrediction(
            joint_names=names,
            targets=tuple(targets),
            model_bundle_digest=digest,
            inference_duration_ns=round(self.settings.latency_s * 1_000_000_000),
            context=returned_context,
            session_id=session_id + "-old" if self.settings.fault == "wrong_session" else "",
            request_id="",
            observation_id=observation_id,
        )

    def reset(self, session_id: str, reset_id: str, context: ProviderContext) -> None:
        del reset_id, context
        if session_id not in self._sessions:
            raise PolicyError("UNKNOWN_SESSION", "fake reset session")
        self._replay_starts.pop(session_id, None)

    def close(self, session_id: str) -> None:
        self.closed.append(session_id)
        self._sessions.discard(session_id)
        self._replay_starts.pop(session_id, None)


__all__ = ["FakeProvider", "FakeSettings", "load_fake_config"]
