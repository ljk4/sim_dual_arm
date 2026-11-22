"""Typed serving value objects shared by the Policy service, pipeline and test doubles."""

from __future__ import annotations

from dataclasses import dataclass

from robot_contracts.models import RunContext


class PolicyError(ValueError):
    """One service rejection with a stable reason used in logs and gRPC status."""

    def __init__(self, reason: str, detail: str = "") -> None:
        self.reason = reason
        super().__init__(f"{reason}: {detail}" if detail else reason)


@dataclass(frozen=True, slots=True)
class ProviderInfo:
    model_bundle_digest: str
    processor_digest: str
    observation_schema_hash: str
    action_schema_hash: str
    robot_profile_hash: str
    state_joint_names: tuple[str, ...]
    action_joint_names: tuple[str, ...]
    required_camera_names: tuple[str, ...]
    action_period_ns: int
    ready: bool
    command_space: str
    chunk_size: int
    test_only: bool

    @classmethod
    def from_config(cls, config: object) -> ProviderInfo:
        """Project one PolicyServiceConfig onto the wire identity."""
        return cls(
            model_bundle_digest=config.model_bundle_digest,  # type: ignore[attr-defined]
            processor_digest=config.processor_digest,  # type: ignore[attr-defined]
            observation_schema_hash=config.observation_schema_hash,  # type: ignore[attr-defined]
            action_schema_hash=config.action_schema_hash,  # type: ignore[attr-defined]
            robot_profile_hash=config.robot_profile_hash,  # type: ignore[attr-defined]
            state_joint_names=config.joint_names,  # type: ignore[attr-defined]
            action_joint_names=config.action_joint_names,  # type: ignore[attr-defined]
            required_camera_names=config.camera_names,  # type: ignore[attr-defined]
            action_period_ns=config.action_period_ns,  # type: ignore[attr-defined]
            ready=True,
            command_space=config.command_space,  # type: ignore[attr-defined]
            chunk_size=config.chunk_size,  # type: ignore[attr-defined]
            test_only=config.test_only,  # type: ignore[attr-defined]
        )


@dataclass(frozen=True, slots=True)
class ProviderJointSample:
    state_seq: int
    captured_at_robot_time_ns: int
    joint_names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class ProviderCameraSample:
    camera_name: str
    frame_seq: int
    captured_at_robot_time_ns: int
    frame_id: str
    calibration_digest: str
    width: int
    height: int
    encoding: int
    payload: bytes


@dataclass(frozen=True, slots=True)
class ProviderObservation:
    observation_id: int
    reference_robot_time_ns: int
    joints: ProviderJointSample
    cameras: tuple[ProviderCameraSample, ...]
    task_id: str
    task_text: str


@dataclass(frozen=True, slots=True)
class ProviderTarget:
    time_from_observation_ns: int
    positions: tuple[float, ...]


@dataclass(frozen=True, slots=True)
class ProviderPrediction:
    """One complete prediction; identity fields are checked against the request."""

    joint_names: tuple[str, ...]
    targets: tuple[ProviderTarget, ...]
    model_bundle_digest: str
    inference_duration_ns: int
    context: RunContext | None = None
    session_id: str = ""
    request_id: str = ""
    observation_id: int = 0


__all__ = [
    "PolicyError",
    "ProviderCameraSample",
    "ProviderInfo",
    "ProviderJointSample",
    "ProviderObservation",
    "ProviderPrediction",
    "ProviderTarget",
    "RunContext",
]
