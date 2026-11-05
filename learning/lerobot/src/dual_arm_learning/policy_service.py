"""Bounded Policy RPC service over one typed provider boundary.

The service owns wire identity, session/request idempotence, observation and
prediction schema checks, deadlines and bounded caches.  It has no Torch, ROS,
MuJoCo or runtime import; a fake or ACT provider is injected from outside.
"""

from __future__ import annotations

import hashlib
import json
import math
import threading
import time
import uuid
from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Any

from google.protobuf.message import Message

from .policy_config import PolicyServiceConfig
from .policy_types import (
    PolicyError,
    PolicyProvider,
    ProviderCameraSample,
    ProviderContext,
    ProviderInfo,
    ProviderJointSample,
    ProviderObservation,
    ProviderPrediction,
)


def _bad(reason: str, detail: str = "") -> PolicyError:
    return PolicyError(reason, detail)


def _canonical_bytes(value: dict[str, Any]) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def semantic_hash(value: dict[str, Any]) -> str:
    return hashlib.sha256(_canonical_bytes(value)).hexdigest()


def _payload_digest(message: Message) -> str:
    return hashlib.sha256(message.SerializeToString(deterministic=True)).hexdigest()


def _finite(value: object, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise _bad("SCHEMA_MISMATCH", f"{location} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise _bad("SCHEMA_MISMATCH", f"{location} must be finite")
    return result


def _context_value(message: object) -> ProviderContext:
    context = ProviderContext(
        robot_id=str(message.robot_id),  # type: ignore[attr-defined]
        robot_profile_hash=str(message.robot_profile_hash),  # type: ignore[attr-defined]
        backend_instance_id=str(message.backend_instance_id),  # type: ignore[attr-defined]
        time_domain_id=str(message.time_domain_id),  # type: ignore[attr-defined]
        clock_epoch=int(message.clock_epoch),  # type: ignore[attr-defined]
        episode_id=str(message.episode_id),  # type: ignore[attr-defined]
        control_epoch=int(message.control_epoch),  # type: ignore[attr-defined]
    )
    for name in ("robot_id", "robot_profile_hash", "backend_instance_id", "time_domain_id"):
        if not getattr(context, name):
            raise _bad("INVALID_CONTEXT", f"context.{name} must not be empty")
    if not context.episode_id:
        raise _bad("INVALID_CONTEXT", "context.episode_id must not be empty")
    if context.control_epoch < 1:
        raise _bad("INVALID_CONTEXT", "context.control_epoch must be positive")
    return context


def _same_context(left: ProviderContext, right: ProviderContext) -> bool:
    return left == right


@dataclass(slots=True)
class _Session:
    session_id: str
    context: ProviderContext
    open_digest: str
    camera_calibrations: dict[str, str]
    lock: threading.Lock = field(default_factory=threading.Lock)
    request_cache: OrderedDict[str, tuple[str, object]] = field(default_factory=OrderedDict)
    reset_cache: OrderedDict[str, tuple[str, object]] = field(default_factory=OrderedDict)
    observation_digests: dict[int, str] = field(default_factory=dict)
    max_observation_id: int = 0


class PolicyService:
    """The actual service layer, independent of the gRPC server object."""

    def __init__(self, config: PolicyServiceConfig, provider: PolicyProvider) -> None:
        if not isinstance(config, PolicyServiceConfig):
            raise _bad("SCHEMA_MISMATCH", "config must be PolicyServiceConfig")
        self.config = config
        self.provider = provider
        try:
            info = provider.info()
        except PolicyError:
            raise
        except Exception as error:  # noqa: BLE001 - provider failure is explicit
            raise _bad("PROVIDER_ERROR", f"provider info failed: {error}") from error
        self.info = self._validate_info(info)
        self._sessions: dict[str, _Session] = {}
        self._open_cache: OrderedDict[str, tuple[str, str]] = OrderedDict()
        self._lock = threading.Lock()

    # ------------------------------------------------------------------ info
    def _validate_info(self, info: ProviderInfo) -> ProviderInfo:
        cfg = self.config
        if not isinstance(info, ProviderInfo):
            raise _bad("PROVIDER_MISMATCH", "provider.info must return ProviderInfo")
        expected = (
            ("protocol_major", cfg.protocol_major),
            ("model_bundle_digest", cfg.model_bundle_digest),
            ("processor_digest", cfg.processor_digest),
            ("observation_schema_hash", cfg.observation_schema_hash),
            ("action_schema_hash", cfg.action_schema_hash),
            ("robot_profile_hash", cfg.robot_profile_hash),
            ("state_joint_names", cfg.joint_names),
            ("action_joint_names", cfg.action_joint_names),
            ("required_camera_names", cfg.camera_names),
            ("action_period_ns", cfg.action_period_ns),
            ("stateful", cfg.stateful),
            ("command_space", cfg.command_space),
            ("chunk_size", cfg.chunk_size),
            ("test_only", cfg.test_only),
        )
        for name, value in expected:
            if getattr(info, name) != value:
                raise _bad("PROVIDER_MISMATCH", f"provider {name} differs from config")
        if info.ready is not True:
            raise _bad("NOT_READY", "provider is not ready")
        return info

    def info_reply(self, requested_model_digest: str = "") -> object:
        if requested_model_digest and requested_model_digest != self.info.model_bundle_digest:
            raise _bad("MODEL_DIGEST_MISMATCH", "requested model bundle is not loaded")
        return self._policy_info_message()

    def _policy_info_message(self) -> object:
        from dual_arm_policy.v1 import policy_pb2

        info = self.info
        return policy_pb2.PolicyInfo(
            protocol_major=info.protocol_major,
            model_bundle_digest=info.model_bundle_digest,
            processor_digest=info.processor_digest,
            observation_schema_hash=info.observation_schema_hash,
            action_schema_hash=info.action_schema_hash,
            robot_profile_hash=info.robot_profile_hash,
            state_joint_names=list(info.state_joint_names),
            action_joint_names=list(info.action_joint_names),
            required_camera_names=list(info.required_camera_names),
            action_period_ns=info.action_period_ns,
            stateful=info.stateful,
            ready=info.ready,
            command_space=info.command_space,
            chunk_size=info.chunk_size,
            test_only=info.test_only,
        )

    # ------------------------------------------------------------------ open
    def open_session(self, request: object, deadline_ns: int | None = None) -> object:
        del deadline_ns
        open_request_id = str(request.open_request_id)  # type: ignore[attr-defined]
        if not open_request_id:
            raise _bad("INVALID_REQUEST", "open_request_id must not be empty")
        context = _context_value(request.context)  # type: ignore[attr-defined]
        if context.robot_profile_hash != self.info.robot_profile_hash:
            raise _bad("CONTEXT_MISMATCH", "context robot profile differs")
        if request.model_bundle_digest != self.info.model_bundle_digest:  # type: ignore[attr-defined]
            raise _bad("MODEL_DIGEST_MISMATCH", "open model bundle differs")
        if request.observation_schema_hash != self.info.observation_schema_hash:  # type: ignore[attr-defined]
            raise _bad("SCHEMA_MISMATCH", "open observation schema differs")
        if request.action_schema_hash != self.info.action_schema_hash:  # type: ignore[attr-defined]
            raise _bad("SCHEMA_MISMATCH", "open action schema differs")
        calibrations = self._validate_documents(
            bytes(request.robot_profile_json),  # type: ignore[attr-defined]
            bytes(request.observation_schema_json),  # type: ignore[attr-defined]
            bytes(request.action_schema_json),  # type: ignore[attr-defined]
        )
        digest = _payload_digest(request)  # type: ignore[arg-type]
        with self._lock:
            cached = self._open_cache.get(open_request_id)
            if cached is not None:
                previous_digest, session_id = cached
                if previous_digest != digest:
                    raise _bad(
                        "REQUEST_ID_CONFLICT",
                        "open_request_id was already used with different content",
                    )
                session = self._sessions.get(session_id)
                if session is None:
                    raise _bad("UNKNOWN_SESSION", "cached open session no longer exists")
                return self._open_reply(session)
            if len(self._sessions) >= self.config.max_sessions:
                raise _bad("RESOURCE_EXHAUSTED", "session capacity is full")
            if len(self._open_cache) >= self.config.max_open_requests:
                raise _bad("RESOURCE_EXHAUSTED", "open request cache is full")
            session_id = f"policy-{uuid.uuid4()}"
            try:
                self.provider.open(
                    session_id,
                    context,
                    open_request_id,
                    bytes(request.robot_profile_json),  # type: ignore[attr-defined]
                    bytes(request.observation_schema_json),  # type: ignore[attr-defined]
                    bytes(request.action_schema_json),  # type: ignore[attr-defined]
                )
            except PolicyError:
                raise
            except Exception as error:  # noqa: BLE001
                raise _bad("PROVIDER_ERROR", f"provider open failed: {error}") from error
            session = _Session(session_id, context, digest, calibrations)
            self._sessions[session_id] = session
            self._open_cache[open_request_id] = (digest, session_id)
            return self._open_reply(session)

    def _open_reply(self, session: _Session) -> object:
        from dual_arm_policy.v1 import policy_pb2

        return policy_pb2.OpenSessionReply(
            session_id=session.session_id,
            context=self._context_message(session.context),
            policy=self._policy_info_message(),
        )

    @staticmethod
    def _context_message(context: ProviderContext) -> object:
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

    def _validate_documents(
        self, profile_json: bytes, observation_json: bytes, action_json: bytes
    ) -> dict[str, str]:
        try:
            profile = json.loads(profile_json)
            observation = json.loads(observation_json)
            action = json.loads(action_json)
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise _bad("SCHEMA_MISMATCH", f"session JSON is invalid: {error}") from error
        if not all(isinstance(item, dict) for item in (profile, observation, action)):
            raise _bad("SCHEMA_MISMATCH", "session documents must be JSON objects")
        try:
            profile_hash = semantic_hash(profile)  # type: ignore[arg-type]
            observation_hash = semantic_hash(observation)  # type: ignore[arg-type]
            action_hash = semantic_hash(action)  # type: ignore[arg-type]
        except (TypeError, ValueError) as error:
            raise _bad("SCHEMA_MISMATCH", f"session JSON is not canonical: {error}") from error
        if profile_hash != self.info.robot_profile_hash:
            raise _bad("SCHEMA_MISMATCH", "robot_profile_json digest differs")
        if observation_hash != self.info.observation_schema_hash:
            raise _bad("SCHEMA_MISMATCH", "observation_schema_json digest differs")
        if action_hash != self.info.action_schema_hash:
            raise _bad("SCHEMA_MISMATCH", "action_schema_json digest differs")
        joints = profile.get("joints")
        if (
            not isinstance(joints, list)
            or tuple(item.get("name") for item in joints if isinstance(item, dict))
            != self.config.joint_names
        ):
            raise _bad("SCHEMA_MISMATCH", "robot profile joint order differs")
        cameras = profile.get("cameras")
        if not isinstance(cameras, list):
            raise _bad("SCHEMA_MISMATCH", "robot profile has no cameras")
        calibrations: dict[str, str] = {}
        camera_names: list[str] = []
        for item in cameras:
            if not isinstance(item, dict):
                raise _bad("SCHEMA_MISMATCH", "profile camera entry must be an object")
            name = item.get("name")
            if not isinstance(name, str):
                raise _bad("SCHEMA_MISMATCH", "profile camera name must be a string")
            if (item.get("width"), item.get("height")) != (
                self.config.camera_width,
                self.config.camera_height,
            ):
                raise _bad("SCHEMA_MISMATCH", f"profile camera {name} dimensions differ")
            digest = item.get("calibration_digest")
            if not isinstance(digest, str) or not digest:
                raise _bad("SCHEMA_MISMATCH", f"profile camera {name} lacks a calibration digest")
            camera_names.append(name)
            calibrations[name] = digest
        if tuple(camera_names) != self.config.camera_names:
            raise _bad("SCHEMA_MISMATCH", "robot profile camera order differs")
        if tuple(observation.get("joint_names", ())) != self.config.joint_names:
            raise _bad("SCHEMA_MISMATCH", "observation schema joint order differs")
        if tuple(observation.get("camera_names", ())) != self.config.camera_names:
            raise _bad("SCHEMA_MISMATCH", "observation schema camera order differs")
        if tuple(action.get("joint_names", ())) != self.config.action_joint_names:
            raise _bad("SCHEMA_MISMATCH", "action schema joint order differs")
        if action.get("command_space") != self.config.command_space:
            raise _bad("SCHEMA_MISMATCH", "action schema command space differs")
        return calibrations

    # ------------------------------------------------------------- predict
    def predict(self, request: object, deadline_ns: int | None = None) -> object:
        from dual_arm_policy.v1 import policy_pb2

        session = self._session(str(request.session_id))  # type: ignore[attr-defined]
        request_id = str(request.request_id)  # type: ignore[attr-defined]
        if not request_id:
            raise _bad("INVALID_REQUEST", "request_id must not be empty")
        context = _context_value(request.context)  # type: ignore[attr-defined]
        with session.lock:
            # Old-context responses are never reused or executed after reset.
            if not _same_context(context, session.context):
                raise _bad("STALE_CONTEXT", "predict context differs from the open session")
            observation = self._validate_observation(session, request.observation)  # type: ignore[attr-defined]
            digest = _payload_digest(request)  # type: ignore[arg-type]
            cached = session.request_cache.get(request_id)
            if cached is not None:
                previous_digest, reply = cached
                if previous_digest != digest:
                    raise _bad(
                        "REQUEST_ID_CONFLICT",
                        "request_id was already used with different content",
                    )
                return reply
            if len(session.request_cache) >= self.config.max_request_cache:
                session.request_cache.popitem(last=False)
            started = time.monotonic_ns()
            try:
                prediction = self.provider.predict(
                    session.session_id, request_id, context, observation
                )
            except PolicyError:
                raise
            except Exception as error:  # noqa: BLE001
                raise _bad("PROVIDER_ERROR", f"provider predict failed: {error}") from error
            elapsed = time.monotonic_ns() - started
            if deadline_ns is not None and elapsed > deadline_ns:
                raise _bad("DEADLINE_EXCEEDED", "provider prediction exceeded its deadline")
            target_count, inference_duration = self._validate_prediction(
                prediction, observation, context, session.session_id, request_id
            )
            reply = policy_pb2.PredictReply(
                session_id=session.session_id,
                request_id=request_id,
                context=self._context_message(session.context),
                observation_id=observation.observation_id,
                model_bundle_digest=self.info.model_bundle_digest,
                action=policy_pb2.ActionChunk(
                    joint_names=list(self.info.action_joint_names),
                    targets=[
                        policy_pb2.JointTarget(
                            time_from_observation_ns=target.time_from_observation_ns,
                            positions=list(target.positions),
                        )
                        for target in prediction.targets
                    ],
                ),
                inference_duration_ns=inference_duration,
            )
            assert len(reply.action.targets) == target_count
            session.request_cache[request_id] = (digest, reply)
            return reply

    def _session(self, session_id: str) -> _Session:
        if not session_id:
            raise _bad("INVALID_REQUEST", "session_id must not be empty")
        with self._lock:
            session = self._sessions.get(session_id)
        if session is None:
            raise _bad("UNKNOWN_SESSION", f"unknown session {session_id}")
        return session

    def _validate_observation(self, session: _Session, observation: object) -> ProviderObservation:
        cfg = self.config
        if int(observation.observation_id) < 1:  # type: ignore[attr-defined]
            raise _bad("INVALID_OBSERVATION", "observation_id must be positive")
        reference = int(observation.reference_robot_time_ns)  # type: ignore[attr-defined]
        if reference < 0:
            raise _bad("INVALID_OBSERVATION", "reference robot time must be non-negative")
        if observation.task_id != cfg.task_id or observation.task_text != cfg.task_text:  # type: ignore[attr-defined]
            raise _bad("INVALID_OBSERVATION", "task identity/text differs from the fixed task")
        joints = observation.joints  # type: ignore[attr-defined]
        names = tuple(joints.joint_names)
        positions = tuple(
            _finite(value, f"joints.positions[{index}]")
            for index, value in enumerate(joints.positions)
        )
        velocities = tuple(
            _finite(value, f"joints.velocities[{index}]")
            for index, value in enumerate(joints.velocities)
        )
        if names != cfg.joint_names:
            raise _bad("INVALID_OBSERVATION", "joint names/order differ")
        if len(positions) != len(cfg.joint_names):
            raise _bad("INVALID_OBSERVATION", "joint positions do not cover all 16 joints")
        if velocities and len(velocities) != len(cfg.joint_names):
            raise _bad("INVALID_OBSERVATION", "joint velocities are not 16-wide")
        if int(joints.state_seq) < 1:  # type: ignore[attr-defined]
            raise _bad("INVALID_OBSERVATION", "state_seq must be positive")
        if int(joints.captured_at_robot_time_ns) != reference:  # type: ignore[attr-defined]
            raise _bad("INVALID_OBSERVATION", "joint capture time differs from the reference")
        cameras = tuple(observation.cameras)  # type: ignore[attr-defined]
        if tuple(camera.camera_name for camera in cameras) != cfg.camera_names:
            raise _bad("INVALID_OBSERVATION", "camera names/order differ")
        typed: list[ProviderCameraSample] = []
        total_payload = 0
        for camera in cameras:
            if int(camera.frame_seq) < 1:
                raise _bad("INVALID_OBSERVATION", f"{camera.camera_name}: frame_seq")
            if int(camera.captured_at_robot_time_ns) != reference:
                raise _bad("INVALID_OBSERVATION", f"{camera.camera_name}: capture time")
            if (int(camera.width), int(camera.height)) != (cfg.camera_width, cfg.camera_height):
                raise _bad("INVALID_OBSERVATION", f"{camera.camera_name}: dimensions")
            if int(camera.encoding) != 1:
                raise _bad("INVALID_OBSERVATION", f"{camera.camera_name}: encoding is not RAW_RGB8")
            payload = bytes(camera.payload)
            if len(payload) != cfg.camera_width * cfg.camera_height * 3:
                raise _bad("INVALID_OBSERVATION", f"{camera.camera_name}: RGB payload size")
            if not camera.frame_id:
                raise _bad("INVALID_OBSERVATION", f"{camera.camera_name}: frame_id is empty")
            if camera.calibration_digest != session.camera_calibrations.get(camera.camera_name):
                raise _bad("INVALID_OBSERVATION", f"{camera.camera_name}: calibration digest")
            total_payload += len(payload)
            typed.append(
                ProviderCameraSample(
                    str(camera.camera_name),
                    int(camera.frame_seq),
                    int(camera.captured_at_robot_time_ns),
                    str(camera.frame_id),
                    str(camera.calibration_digest),
                    int(camera.width),
                    int(camera.height),
                    int(camera.encoding),
                    payload,
                )
            )
        if total_payload > cfg.max_message_bytes:
            raise _bad("RESOURCE_EXHAUSTED", "observation payload exceeds the configured bound")
        return ProviderObservation(
            int(observation.observation_id),
            reference,
            ProviderJointSample(
                int(joints.state_seq),
                int(joints.captured_at_robot_time_ns),
                names,
                positions,
                velocities,
            ),
            tuple(typed),
            str(observation.task_id),
            str(observation.task_text),
        )

    def _validate_prediction(
        self,
        prediction: ProviderPrediction,
        observation: ProviderObservation,
        observation_context: ProviderContext,
        session_id: str,
        request_id: str,
    ) -> tuple[int, int]:
        cfg = self.config
        if not isinstance(prediction, ProviderPrediction):
            raise _bad("INVALID_PREDICTION", "provider did not return ProviderPrediction")
        # A provider cannot smuggle an old/wrong context into a canonical reply.
        if prediction.context is not None and prediction.context != observation_context:
            raise _bad("STALE_CONTEXT", "provider prediction context differs")
        if prediction.observation_id and prediction.observation_id != observation.observation_id:
            raise _bad("INVALID_PREDICTION", "provider observation identity differs")
        if prediction.session_id and prediction.session_id != session_id:
            raise _bad("INVALID_PREDICTION", "provider session identity differs")
        if prediction.request_id and prediction.request_id != request_id:
            raise _bad("INVALID_PREDICTION", "provider request identity differs")
        if tuple(prediction.joint_names) != cfg.action_joint_names:
            raise _bad("INVALID_PREDICTION", "prediction joint names/order differ")
        if prediction.model_bundle_digest != self.info.model_bundle_digest:
            raise _bad("INVALID_PREDICTION", "prediction model digest differs")
        targets = tuple(prediction.targets)
        if len(targets) != cfg.chunk_size:
            raise _bad(
                "INVALID_PREDICTION",
                f"prediction has {len(targets)} targets, expected {cfg.chunk_size}",
            )
        previous = 0
        for index, target in enumerate(targets):
            expected = (index + 1) * cfg.action_period_ns
            if target.time_from_observation_ns != expected:
                raise _bad(
                    "INVALID_PREDICTION",
                    f"target[{index}] time is not the uniform {cfg.action_period_ns}ns grid",
                )
            if target.time_from_observation_ns <= previous:
                raise _bad("INVALID_PREDICTION", "target times are not increasing")
            previous = target.time_from_observation_ns
            positions = tuple(target.positions)
            if len(positions) != len(cfg.action_joint_names):
                raise _bad("INVALID_PREDICTION", f"target[{index}] is not 16-wide")
            for axis, value in enumerate(positions):
                number = _finite(value, f"target[{index}].positions[{axis}]")
                low = cfg.joint_lower_bounds[axis]
                high = cfg.joint_upper_bounds[axis]
                if not low <= number <= high:
                    raise _bad(
                        "INVALID_PREDICTION",
                        f"target[{index}].positions[{axis}] outside the robot joint range",
                    )
            if observation.reference_robot_time_ns + target.time_from_observation_ns > 2**63 - 1:
                raise _bad("INVALID_PREDICTION", "target absolute time exceeds int64")
        duration = prediction.inference_duration_ns
        if isinstance(duration, bool) or not isinstance(duration, int) or duration < 0:
            raise _bad("INVALID_PREDICTION", "inference_duration_ns must be non-negative")
        return len(targets), duration

    # --------------------------------------------------------------- reset
    def reset_session(self, request: object) -> object:
        from dual_arm_policy.v1 import policy_pb2

        session = self._session(str(request.session_id))  # type: ignore[attr-defined]
        reset_id = str(request.reset_id)  # type: ignore[attr-defined]
        if not reset_id:
            raise _bad("INVALID_REQUEST", "reset_id must not be empty")
        context = _context_value(request.new_context)  # type: ignore[attr-defined]
        with session.lock:
            if len(session.reset_cache) >= self.config.max_request_cache:
                session.reset_cache.popitem(last=False)
            cached = session.reset_cache.get(reset_id)
            if cached is not None:
                digest, reply = cached
                if digest != _payload_digest(request):  # type: ignore[arg-type]
                    raise _bad("REQUEST_ID_CONFLICT", "reset_id content differs")
                return reply
            if context.robot_profile_hash != self.info.robot_profile_hash:
                raise _bad("STALE_CONTEXT", "reset robot profile differs")
            if context.clock_epoch < session.context.clock_epoch:
                raise _bad("STALE_CONTEXT", "reset clock_epoch moved backwards")
            try:
                self.provider.reset(session.session_id, reset_id, context)
            except PolicyError:
                raise
            except Exception as error:  # noqa: BLE001
                raise _bad("PROVIDER_ERROR", f"provider reset failed: {error}") from error
            session.context = context
            session.request_cache.clear()
            session.observation_digests.clear()
            session.max_observation_id = 0
            reply = policy_pb2.ResetSessionReply(
                session_id=session.session_id,
                reset_id=reset_id,
                context=self._context_message(context),
                ready=True,
            )
            session.reset_cache[reset_id] = (_payload_digest(request), reply)  # type: ignore[arg-type]
            return reply

    # --------------------------------------------------------------- close
    def close_session(self, request: object) -> object:
        from dual_arm_policy.v1 import policy_pb2

        session_id = str(request.session_id)  # type: ignore[attr-defined]
        if not session_id:
            raise _bad("INVALID_REQUEST", "session_id must not be empty")
        with self._lock:
            session = self._sessions.pop(session_id, None)
        if session is not None:
            with session.lock:
                try:
                    self.provider.close(session.session_id)
                except PolicyError:
                    raise
                except Exception as error:  # noqa: BLE001
                    raise _bad("PROVIDER_ERROR", f"provider close failed: {error}") from error
        return policy_pb2.CloseSessionReply(session_id=session_id, closed=True)


__all__ = ["PolicyService", "semantic_hash"]
