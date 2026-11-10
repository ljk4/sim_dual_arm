"""Typed provider boundary shared by the service and test doubles.

These values contain no protobuf or ROS types, so a provider can be implemented
by a fake in ``tests/fakes`` or by a future ACT adapter without importing the
wire layer.  The service always rebuilds the canonical wire identity itself.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


class PolicyError(ValueError):
    """One service rejection with a stable reason used in logs and gRPC status."""

    def __init__(self, reason: str, detail: str = "") -> None:
        self.reason = reason
        super().__init__(f"{reason}: {detail}" if detail else reason)


@dataclass(frozen=True, slots=True)
class ProviderContext:
    robot_id: str
    robot_profile_hash: str
    backend_instance_id: str
    time_domain_id: str
    clock_epoch: int
    episode_id: str
    control_epoch: int


@dataclass(frozen=True, slots=True)
class ProviderInfo:
    protocol_major: int
    model_bundle_digest: str
    processor_digest: str
    observation_schema_hash: str
    action_schema_hash: str
    robot_profile_hash: str
    state_joint_names: tuple[str, ...]
    action_joint_names: tuple[str, ...]
    required_camera_names: tuple[str, ...]
    action_period_ns: int
    stateful: bool
    ready: bool
    command_space: str
    chunk_size: int
    test_only: bool

    @classmethod
    def from_config(cls, config: object) -> ProviderInfo:
        """Project one PolicyServiceConfig onto the wire identity."""
        return cls(
            protocol_major=config.protocol_major,  # type: ignore[attr-defined]
            model_bundle_digest=config.model_bundle_digest,  # type: ignore[attr-defined]
            processor_digest=config.processor_digest,  # type: ignore[attr-defined]
            observation_schema_hash=config.observation_schema_hash,  # type: ignore[attr-defined]
            action_schema_hash=config.action_schema_hash,  # type: ignore[attr-defined]
            robot_profile_hash=config.robot_profile_hash,  # type: ignore[attr-defined]
            state_joint_names=config.joint_names,  # type: ignore[attr-defined]
            action_joint_names=config.action_joint_names,  # type: ignore[attr-defined]
            required_camera_names=config.camera_names,  # type: ignore[attr-defined]
            action_period_ns=config.action_period_ns,  # type: ignore[attr-defined]
            stateful=config.stateful,  # type: ignore[attr-defined]
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
    """One complete prediction; identity fields are checked against the request.

    The optional identity fields are normally empty.  They exist so a fault
    injection double can return a wrong session/request/observation/epoch and
    prove the service rejects it instead of forwarding an old context.
    """

    joint_names: tuple[str, ...]
    targets: tuple[ProviderTarget, ...]
    model_bundle_digest: str
    inference_duration_ns: int
    context: ProviderContext | None = None
    session_id: str = ""
    request_id: str = ""
    observation_id: int = 0


class PolicyProvider(Protocol):
    """The only typed extension point; the service owns sessions and wire rules."""

    def info(self) -> ProviderInfo: ...

    def open(
        self,
        session_id: str,
        context: ProviderContext,
        open_request_id: str,
        robot_profile_json: bytes,
        observation_schema_json: bytes,
        action_schema_json: bytes,
    ) -> None: ...

    def predict(
        self,
        session_id: str,
        request_id: str,
        context: ProviderContext,
        observation: ProviderObservation,
    ) -> ProviderPrediction: ...

    def reset(
        self,
        session_id: str,
        reset_id: str,
        context: ProviderContext,
    ) -> None: ...

    def close(self, session_id: str) -> None: ...


__all__ = [
    "PolicyError",
    "PolicyProvider",
    "ProviderCameraSample",
    "ProviderContext",
    "ProviderInfo",
    "ProviderJointSample",
    "ProviderObservation",
    "ProviderPrediction",
    "ProviderTarget",
]
