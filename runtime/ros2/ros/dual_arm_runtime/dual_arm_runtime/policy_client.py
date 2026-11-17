"""Policy RPC client and strict wire-to-domain validation.

This module never imports ROS message classes.  The runtime converts one exact
capture into :class:`PolicyObservation`, then this client checks every returned
identity, digest, joint/camera order and 60x16 finite target before anything can
be submitted as a bounded-follower stream request.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from typing import Any

from robot_contracts.models import ContractError, RobotProfile, RunContext
from robot_run_logging import Scalar

from dual_arm_runtime.run_logging import EventSink, emit_best_effort
from dual_arm_runtime.runtime_config import PolicyConfig

IMAGE_ENCODING_RAW_RGB8 = 1


class PolicyRpcError(RuntimeError):
    def __init__(self, reason: str, detail: str = "") -> None:
        self.reason = reason
        super().__init__(f"{reason}: {detail}" if detail else reason)


@dataclass(frozen=True, slots=True)
class PolicyCameraSample:
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
class PolicyObservation:
    observation_id: int
    reference_robot_time_ns: int
    state_seq: int
    joint_names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    cameras: tuple[PolicyCameraSample, ...]
    task_id: str
    task_text: str


@dataclass(frozen=True, slots=True)
class PolicyPrediction:
    session_id: str
    request_id: str
    context: RunContext
    observation_id: int
    model_bundle_digest: str
    joint_names: tuple[str, ...]
    times_from_observation_ns: tuple[int, ...]
    positions: tuple[tuple[float, ...], ...]
    inference_duration_ns: int

    @property
    def target_count(self) -> int:
        return len(self.times_from_observation_ns)


def _finite(value: object, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise ContractError("POLICY_SCHEMA_MISMATCH", f"{location} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise ContractError("POLICY_SCHEMA_MISMATCH", f"{location} must be finite")
    return result


def _context_from_message(message: object) -> RunContext:
    return RunContext(
        robot_id=str(message.robot_id),  # type: ignore[attr-defined]
        robot_profile_hash=str(message.robot_profile_hash),  # type: ignore[attr-defined]
        backend_instance_id=str(message.backend_instance_id),  # type: ignore[attr-defined]
        time_domain_id=str(message.time_domain_id),  # type: ignore[attr-defined]
        clock_epoch=int(message.clock_epoch),  # type: ignore[attr-defined]
        episode_id=str(message.episode_id),  # type: ignore[attr-defined]
        control_epoch=int(message.control_epoch),  # type: ignore[attr-defined]
    )


def _context_message(context: RunContext) -> object:
    from dual_arm_policy.v1 import policy_pb2

    return policy_pb2.RunContext(
        robot_id=context.robot_id,
        robot_profile_hash=context.robot_profile_hash,
        backend_instance_id=context.backend_instance_id,
        time_domain_id=context.time_domain_id,
        clock_epoch=context.clock_epoch,
        episode_id=context.episode_id,
        control_epoch=context.control_epoch,
    )


def _canonical_json(document: dict[str, Any]) -> bytes:
    return json.dumps(document, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def observation_from_exact(
    exact: object,
    profile: RobotProfile,
    policy: PolicyConfig,
) -> PolicyObservation:
    """Copy one exact capture; never synthesise a newer observation ID."""
    frames = exact.frames  # type: ignore[attr-defined]
    if tuple(frames) != profile.camera_names:
        raise ContractError("POLICY_OBSERVATION_MISMATCH", "capture camera order differs")
    state_message = exact.state.payload  # type: ignore[attr-defined]
    names = tuple(state_message.joints.name)
    if names != profile.joint_names:
        raise ContractError("POLICY_OBSERVATION_MISMATCH", "capture joint order differs")
    positions = tuple(
        _finite(value, f"state.positions[{index}]")
        for index, value in enumerate(state_message.joints.position)
    )
    velocities = tuple(
        _finite(value, f"state.velocities[{index}]")
        for index, value in enumerate(state_message.joints.velocity)
    )
    if len(positions) != len(profile.joints):
        raise ContractError("POLICY_OBSERVATION_MISMATCH", "capture positions are not 16-wide")
    if velocities and len(velocities) != len(profile.joints):
        raise ContractError("POLICY_OBSERVATION_MISMATCH", "capture velocities are not 16-wide")
    cameras: list[PolicyCameraSample] = []
    for camera in profile.cameras:
        frame = frames[camera.name].payload
        image = frame.image
        if (image.width, image.height) != (camera.width, camera.height):
            raise ContractError("POLICY_OBSERVATION_MISMATCH", f"{camera.name}: dimensions")
        if image.encoding != "rgb8" or image.step != camera.width * 3:
            raise ContractError("POLICY_OBSERVATION_MISMATCH", f"{camera.name}: RGB encoding/step")
        payload = bytes(image.data)
        if len(payload) != camera.width * camera.height * 3:
            raise ContractError("POLICY_OBSERVATION_MISMATCH", f"{camera.name}: RGB payload")
        if image.header.frame_id != camera.frame_id:
            raise ContractError("POLICY_OBSERVATION_MISMATCH", f"{camera.name}: frame_id")
        if frame.calibration_digest != camera.calibration_digest:
            raise ContractError("POLICY_OBSERVATION_MISMATCH", f"{camera.name}: calibration digest")
        cameras.append(
            PolicyCameraSample(
                camera.name,
                int(frames[camera.name].sequence),
                exact.stamp_ns,  # type: ignore[attr-defined]
                camera.frame_id,
                camera.calibration_digest,
                camera.width,
                camera.height,
                IMAGE_ENCODING_RAW_RGB8,
                payload,
            )
        )
    if not 0 <= exact.stamp_ns <= 2**63 - 1:  # type: ignore[attr-defined]
        raise ContractError("POLICY_OBSERVATION_MISMATCH", "capture reference time")
    return PolicyObservation(
        int(exact.observation_id),  # type: ignore[attr-defined]
        int(exact.stamp_ns),  # type: ignore[attr-defined]
        int(exact.state.sequence),  # type: ignore[attr-defined]
        names,
        positions,
        velocities,
        tuple(cameras),
        policy.task_id,
        policy.task_text,
    )


class PolicyClient:
    """A bounded synchronous gRPC client with one session per episode."""

    def __init__(
        self,
        policy: PolicyConfig,
        profile: RobotProfile,
        profile_document: dict[str, Any],
        observation_document: dict[str, Any],
        action_document: dict[str, Any],
        *,
        log: EventSink | None = None,
    ) -> None:
        if not isinstance(policy, PolicyConfig):
            raise ContractError("POLICY_CONFIGURATION", "policy settings are missing")
        self.policy = policy
        self.profile = profile
        self._profile_json = _canonical_json(profile_document)
        self._observation_json = _canonical_json(observation_document)
        self._action_json = _canonical_json(action_document)
        self._log = log
        self._channel: Any | None = None
        self._stub: Any | None = None
        self.session_id = ""
        # The service owns model identity; the runtime adopts it from the
        # handshake and never reads a bundle or model file itself.
        self.model_bundle_digest = ""
        self.processor_digest = ""
        self.info_checked = False
        self.requests = 0

    # ------------------------------------------------------------- transport
    def connect(self) -> None:
        try:
            import grpc
            from dual_arm_policy.v1 import policy_pb2_grpc
        except ImportError as error:  # pragma: no cover - deployment dependency
            raise ContractError("POLICY_SDK_MISSING", str(error)) from error
        self._grpc = grpc
        options = [
            ("grpc.max_send_message_length", self.policy.max_message_bytes),
            ("grpc.max_receive_message_length", self.policy.max_message_bytes),
        ]
        self._channel = grpc.insecure_channel(self.policy.endpoint, options=options)
        try:
            grpc.channel_ready_future(self._channel).result(timeout=self.policy.connect_timeout_s)
        except Exception as error:
            self._emit(
                "policy.connect.failed",
                level="ERROR",
                reason="POLICY_CONNECT_FAILED",
                fields={
                    "endpoint": self.policy.endpoint,
                    "deadline_s": self.policy.connect_timeout_s,
                },
                exception=error,
            )
            raise ContractError(
                "POLICY_CONNECT_FAILED", f"{self.policy.endpoint}: {error}"
            ) from error
        self._stub = policy_pb2_grpc.PolicyServiceStub(self._channel)
        info = self._rpc(
            "GetPolicyInfo",
            self._info_request(),
            self.policy.connect_timeout_s,
        )
        self._validate_info(info)
        self.info_checked = True
        self._emit(
            "policy.info",
            reason="POLICY_INFO_VERIFIED",
            fields={
                "endpoint": self.policy.endpoint,
                "model_bundle_digest": self.model_bundle_digest,
                "processor_digest": self.processor_digest,
                "test_only": bool(info.test_only),
                "chunk_size": int(info.chunk_size),
                "action_period_ns": int(info.action_period_ns),
            },
        )

    def _info_request(self) -> object:
        from dual_arm_policy.v1 import policy_pb2

        return policy_pb2.GetPolicyInfoRequest()

    def _validate_info(self, info: object) -> None:
        model_digest = str(info.model_bundle_digest)  # type: ignore[attr-defined]
        processor_digest = str(info.processor_digest)  # type: ignore[attr-defined]
        if not model_digest or not processor_digest:
            raise ContractError("POLICY_INFO_MISMATCH", "info has no model identity")
        if self.model_bundle_digest and model_digest != self.model_bundle_digest:
            raise ContractError("POLICY_INFO_MISMATCH", "info.model_bundle_digest differs")
        if self.processor_digest and processor_digest != self.processor_digest:
            raise ContractError("POLICY_INFO_MISMATCH", "info.processor_digest differs")
        self.model_bundle_digest = model_digest
        self.processor_digest = processor_digest
        expected = (
            ("observation_schema_hash", self.policy.observation_schema_hash),
            ("action_schema_hash", self.policy.action_schema_hash),
            ("robot_profile_hash", self.policy.robot_profile_hash),
            ("action_period_ns", self.policy.action_period_ns),
            ("chunk_size", self.policy.chunk_size),
            ("command_space", "joint_position"),
        )
        for name, value in expected:
            if getattr(info, name) != value:
                raise ContractError("POLICY_INFO_MISMATCH", f"info.{name} differs")
        if tuple(info.state_joint_names) != self.profile.joint_names:
            raise ContractError("POLICY_INFO_MISMATCH", "state joint order differs")
        if tuple(info.action_joint_names) != self.profile.joint_names:
            raise ContractError("POLICY_INFO_MISMATCH", "action joint order differs")
        if tuple(info.required_camera_names) != self.profile.camera_names:
            raise ContractError("POLICY_INFO_MISMATCH", "camera order differs")
        if not bool(info.ready):
            raise ContractError("POLICY_NOT_READY", "service is not ready")
        test_only = bool(info.test_only)
        if test_only != self.policy.expected_test_only:
            raise ContractError("POLICY_TEST_ONLY_MISMATCH", "test_only differs from config")
        if test_only and not self.policy.allow_test_policy:
            raise ContractError(
                "TEST_POLICY_REJECTED",
                "runtime config does not allow a test-only policy",
            )
        if int(info.protocol_major) != 1:
            raise ContractError("POLICY_INFO_MISMATCH", "unsupported protocol major")

    def _rpc(self, name: str, request: object, timeout_s: float) -> object:
        if self._stub is None:
            raise ContractError("POLICY_NOT_CONNECTED", "connect() has not completed")
        started = time.monotonic_ns()
        try:
            response = getattr(self._stub, name)(request, timeout=timeout_s)
        except Exception as error:  # grpc.RpcError and transport failures
            elapsed = time.monotonic_ns() - started
            code = ""
            try:
                code = str(error.code().name)  # type: ignore[attr-defined]
            except Exception:
                code = type(error).__name__
            self._emit(
                "policy.rpc.failed",
                level="ERROR",
                reason=f"POLICY_RPC_{code}",
                fields={
                    "rpc": name,
                    "code": code,
                    "elapsed_ns": elapsed,
                    "deadline_s": timeout_s,
                },
                exception=error,
            )
            raise ContractError(f"POLICY_RPC_{code}", f"{name}: {error}") from error
        elapsed = time.monotonic_ns() - started
        self._emit(
            "policy.rpc.completed",
            reason="POLICY_RPC_COMPLETED",
            fields={"rpc": name, "elapsed_ns": elapsed, "deadline_s": timeout_s},
        )
        return response

    def _emit(
        self,
        event: str,
        *,
        level: str = "INFO",
        reason: str | None = None,
        fields: dict[str, Scalar] | None = None,
        exception: BaseException | None = None,
    ) -> None:
        error = emit_best_effort(
            self._log,
            event,
            level=level,
            reason=reason,
            fields=fields,
            exception=exception,
            critical=True,
        )
        if error is not None:
            raise ContractError("LOGGING_ERROR", error)

    # -------------------------------------------------------------- session
    def open(self, context: RunContext, open_request_id: str) -> str:
        from dual_arm_policy.v1 import policy_pb2

        request = policy_pb2.OpenSessionRequest(
            context=_context_message(context),
            model_bundle_digest=self.model_bundle_digest,
            observation_schema_hash=self.policy.observation_schema_hash,
            action_schema_hash=self.policy.action_schema_hash,
            robot_profile_json=self._profile_json,
            observation_schema_json=self._observation_json,
            action_schema_json=self._action_json,
            open_request_id=open_request_id,
        )
        reply = self._rpc("OpenSession", request, self.policy.open_deadline_s)
        if _context_from_message(reply.context) != context:
            raise ContractError("POLICY_CONTEXT_MISMATCH", "open reply context differs")
        self._validate_info(reply.policy)
        session_id = str(reply.session_id)
        if not session_id:
            raise ContractError("POLICY_SESSION_ERROR", "open reply has no session id")
        self.session_id = session_id
        self._emit(
            "policy.session.opened",
            reason="POLICY_SESSION_OPENED",
            fields={"session_id": session_id, "open_request_id": open_request_id},
        )
        return session_id

    def reset(self, context: RunContext, reset_id: str) -> RunContext:
        from dual_arm_policy.v1 import policy_pb2

        if not self.session_id:
            raise ContractError("POLICY_SESSION_ERROR", "no session is open")
        request = policy_pb2.ResetSessionRequest(
            session_id=self.session_id,
            reset_id=reset_id,
            new_context=_context_message(context),
        )
        reply = self._rpc("ResetSession", request, self.policy.open_deadline_s)
        if str(reply.session_id) != self.session_id or str(reply.reset_id) != reset_id:
            raise ContractError("POLICY_SESSION_ERROR", "reset reply identity differs")
        new_context = _context_from_message(reply.context)
        if new_context != context:
            raise ContractError("POLICY_CONTEXT_MISMATCH", "reset reply context differs")
        if not bool(reply.ready):
            raise ContractError("POLICY_NOT_READY", "reset did not make the session ready")
        return new_context

    def predict(
        self,
        context: RunContext,
        session_id: str,
        request_id: str,
        observation: PolicyObservation,
        *,
        deadline_s: float | None = None,
    ) -> PolicyPrediction:
        from dual_arm_policy.v1 import policy_pb2

        message = policy_pb2.Observation(
            observation_id=observation.observation_id,
            reference_robot_time_ns=observation.reference_robot_time_ns,
            joints=policy_pb2.JointSample(
                state_seq=observation.state_seq,
                captured_at_robot_time_ns=observation.reference_robot_time_ns,
                joint_names=list(observation.joint_names),
                positions=list(observation.positions),
                velocities=list(observation.velocities),
            ),
            task_id=observation.task_id,
            task_text=observation.task_text,
        )
        for camera in observation.cameras:
            message.cameras.append(
                policy_pb2.CameraSample(
                    camera_name=camera.camera_name,
                    frame_seq=camera.frame_seq,
                    captured_at_robot_time_ns=camera.captured_at_robot_time_ns,
                    frame_id=camera.frame_id,
                    calibration_digest=camera.calibration_digest,
                    width=camera.width,
                    height=camera.height,
                    encoding=camera.encoding,
                    payload=camera.payload,
                )
            )
        request = policy_pb2.PredictRequest(
            session_id=session_id,
            request_id=request_id,
            context=_context_message(context),
            observation=message,
        )
        timeout = self.policy.predict_deadline_s if deadline_s is None else deadline_s
        self.requests += 1
        self._emit(
            "policy.request",
            reason="POLICY_REQUEST",
            fields={
                "request_id": request_id,
                "observation_id": observation.observation_id,
                "state_seq": observation.state_seq,
                "camera_count": len(observation.cameras),
                "deadline_s": timeout,
            },
        )
        reply = self._rpc("Predict", request, timeout)
        return self._validate_prediction(context, session_id, request_id, observation, reply)

    def _validate_prediction(
        self,
        context: RunContext,
        session_id: str,
        request_id: str,
        observation: PolicyObservation,
        reply: object,
    ) -> PolicyPrediction:
        if str(reply.session_id) != session_id:  # type: ignore[attr-defined]
            raise ContractError("POLICY_SESSION_ERROR", "reply session differs")
        if str(reply.request_id) != request_id:  # type: ignore[attr-defined]
            raise ContractError("POLICY_REQUEST_MISMATCH", "reply request id differs")
        if _context_from_message(reply.context) != context:  # type: ignore[attr-defined]
            raise ContractError("POLICY_CONTEXT_MISMATCH", "reply context differs")
        if int(reply.observation_id) != observation.observation_id:  # type: ignore[attr-defined]
            raise ContractError("POLICY_OBSERVATION_MISMATCH", "reply observation id differs")
        if str(reply.model_bundle_digest) != self.model_bundle_digest:  # type: ignore[attr-defined]
            raise ContractError("POLICY_MODEL_MISMATCH", "reply model digest differs")
        action = reply.action  # type: ignore[attr-defined]
        if tuple(action.joint_names) != self.profile.joint_names:
            raise ContractError("POLICY_ACTION_MISMATCH", "reply joint order differs")
        targets = tuple(action.targets)
        if len(targets) != self.policy.chunk_size:
            raise ContractError(
                "POLICY_ACTION_MISMATCH",
                f"reply has {len(targets)} targets, expected {self.policy.chunk_size}",
            )
        times: list[int] = []
        positions: list[tuple[float, ...]] = []
        for index, target in enumerate(targets):
            expected = (index + 1) * self.policy.action_period_ns
            if int(target.time_from_observation_ns) != expected:
                raise ContractError(
                    "POLICY_ACTION_MISMATCH",
                    f"target[{index}] time is not on the uniform observation-relative grid",
                )
            values = tuple(
                _finite(value, f"target[{index}].positions[{axis}]")
                for axis, value in enumerate(target.positions)
            )
            if len(values) != len(self.profile.joints):
                raise ContractError("POLICY_ACTION_MISMATCH", f"target[{index}] is not 16-wide")
            for axis, (value, joint) in enumerate(zip(values, self.profile.joints, strict=True)):
                if not joint.minimum <= value <= joint.maximum:
                    raise ContractError(
                        "POLICY_ACTION_MISMATCH",
                        f"target[{index}].positions[{axis}] outside the robot joint range",
                    )
            times.append(int(target.time_from_observation_ns))
            positions.append(values)
        duration = int(reply.inference_duration_ns)  # type: ignore[attr-defined]
        if duration < 0:
            raise ContractError("POLICY_ACTION_MISMATCH", "negative inference duration")
        self._emit(
            "policy.prediction",
            reason="POLICY_PREDICTION_VALID",
            fields={
                "request_id": request_id,
                "observation_id": observation.observation_id,
                "targets": len(targets),
                "first_time_ns": times[0],
                "last_time_ns": times[-1],
                "inference_duration_ns": duration,
            },
        )
        return PolicyPrediction(
            session_id,
            request_id,
            context,
            observation.observation_id,
            self.model_bundle_digest,
            tuple(action.joint_names),
            tuple(times),
            tuple(positions),
            duration,
        )

    def close(self) -> None:
        from dual_arm_policy.v1 import policy_pb2

        if not self.session_id or self._stub is None:
            return
        request = policy_pb2.CloseSessionRequest(session_id=self.session_id)
        try:
            reply = self._rpc("CloseSession", request, self.policy.close_deadline_s)
        except ContractError as error:
            self._emit(
                "policy.session.close_failed",
                level="ERROR",
                reason="POLICY_CLOSE_FAILED",
                fields={"session_id": self.session_id, "failure": str(error)},
            )
            raise
        if str(reply.session_id) != self.session_id or not bool(reply.closed):  # type: ignore[attr-defined]
            raise ContractError("POLICY_SESSION_ERROR", "close reply differs")
        self._emit(
            "policy.session.closed",
            reason="POLICY_SESSION_CLOSED",
            fields={"session_id": self.session_id},
        )
        self.session_id = ""

    def shutdown(self) -> None:
        channel, self._channel = self._channel, None
        self._stub = None
        if channel is not None:
            channel.close()


__all__ = [
    "PolicyCameraSample",
    "PolicyClient",
    "PolicyObservation",
    "PolicyPrediction",
    "PolicyRpcError",
    "observation_from_exact",
]
