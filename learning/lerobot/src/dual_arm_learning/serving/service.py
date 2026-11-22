"""Bounded Policy RPC service over one typed PolicyModelAdapter boundary."""

from __future__ import annotations

import hashlib
import json
import math
import threading
import time
import uuid
from collections import OrderedDict
from dataclasses import dataclass, field
from pathlib import Path

from google.protobuf.message import Message
from robot_contracts.io import canonical_sha256 as semantic_hash
from robot_contracts.models import ContractError, RobotProfile, RunContext

from ..adapters.base import PolicyModelAdapter
from .config import PolicyServiceConfig
from .pipeline import ChunkedPolicyPipeline, PipelineResult, PredictionBudget
from .types import (
    PolicyError,
    ProviderCameraSample,
    ProviderInfo,
    ProviderJointSample,
    ProviderObservation,
    ProviderPrediction,
    ProviderTarget,
)


def _bad(reason: str, detail: str = "") -> PolicyError:
    return PolicyError(reason, detail)


def _payload_digest(message: Message) -> str:
    return hashlib.sha256(message.SerializeToString(deterministic=True)).hexdigest()


def _predict_fingerprint(request: object) -> str:
    observation = request.observation  # type: ignore[attr-defined]
    payload = {
        "session_id": str(request.session_id),  # type: ignore[attr-defined]
        "request_id": str(request.request_id),  # type: ignore[attr-defined]
        "observation_id": int(observation.observation_id),
        "reference_robot_time_ns": int(observation.reference_robot_time_ns),
        "state_seq": int(observation.joints.state_seq),
        "frames": [
            (str(camera.camera_name), int(camera.frame_seq), int(camera.captured_at_robot_time_ns))
            for camera in observation.cameras
        ],
    }
    return semantic_hash(payload)


def _finite(value: object, location: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise _bad("SCHEMA_MISMATCH", f"{location} must be a number")
    result = float(value)
    if not math.isfinite(result):
        raise _bad("SCHEMA_MISMATCH", f"{location} must be finite")
    return result


def _context_value(message: object) -> RunContext:
    robot_id = str(message.robot_id)  # type: ignore[attr-defined]
    robot_profile_hash = str(message.robot_profile_hash)  # type: ignore[attr-defined]
    backend_instance_id = str(message.backend_instance_id)  # type: ignore[attr-defined]
    time_domain_id = str(message.time_domain_id)  # type: ignore[attr-defined]
    clock_epoch = int(message.clock_epoch)  # type: ignore[attr-defined]
    episode_id = str(message.episode_id)  # type: ignore[attr-defined]
    control_epoch = int(message.control_epoch)  # type: ignore[attr-defined]
    for name, value in (
        ("robot_id", robot_id),
        ("robot_profile_hash", robot_profile_hash),
        ("backend_instance_id", backend_instance_id),
        ("time_domain_id", time_domain_id),
    ):
        if not value:
            raise _bad("INVALID_CONTEXT", f"context.{name} must not be empty")
    if not episode_id:
        raise _bad("INVALID_CONTEXT", "context.episode_id must not be empty")
    if control_epoch < 1:
        raise _bad("INVALID_CONTEXT", "context.control_epoch must be positive")
    try:
        return RunContext(
            robot_id=robot_id,
            robot_profile_hash=robot_profile_hash,
            backend_instance_id=backend_instance_id,
            time_domain_id=time_domain_id,
            clock_epoch=clock_epoch,
            episode_id=episode_id,
            control_epoch=control_epoch,
        )
    except ContractError as error:
        raise _bad("INVALID_CONTEXT", str(error)) from error


@dataclass(slots=True)
class _Session:
    session_id: str
    context: RunContext
    open_digest: str
    camera_calibrations: dict[str, str]
    pipeline: ChunkedPolicyPipeline
    lock: threading.Lock = field(default_factory=threading.Lock)
    request_cache: OrderedDict[str, tuple[str, object]] = field(default_factory=OrderedDict)
    reset_cache: OrderedDict[str, tuple[str, object]] = field(default_factory=OrderedDict)
    last_observation_id: int = 0
    predictions: int = 0


class PolicyService:
    """The actual service layer, independent of the gRPC server object."""

    def __init__(
        self,
        config: PolicyServiceConfig,
        adapter: PolicyModelAdapter,
        *,
        prediction_directory: Path | None = None,
        prediction_max_bytes: int = 268435456,
    ) -> None:
        if not isinstance(config, PolicyServiceConfig):
            raise _bad("SCHEMA_MISMATCH", "config must be PolicyServiceConfig")
        self.config = config
        self.adapter = adapter
        self.prediction_directory = prediction_directory
        self.prediction_max_bytes = prediction_max_bytes
        self._budget = PredictionBudget()
        self.info = self._validate_info(ProviderInfo.from_config(config))
        self._sessions: dict[str, _Session] = {}
        self._open_cache: OrderedDict[str, tuple[str, str]] = OrderedDict()
        self._lock = threading.Lock()

    def _validate_info(self, info: ProviderInfo) -> ProviderInfo:
        if not hasattr(self.adapter, "predict_raw_chunk"):
            raise _bad("PROVIDER_MISMATCH", "adapter must implement PolicyModelAdapter")
        if (
            getattr(self.adapter, "policy_family", None) != "act"
            or getattr(self.adapter, "chunk_size", None) != self.config.chunk_size
            or getattr(self.adapter, "action_dim", None) != len(self.config.action_joint_names)
        ):
            raise _bad("PROVIDER_MISMATCH", "adapter family/dimensions differ from config")
        return info

    def info_reply(self, requested_model_digest: str = "") -> object:
        if requested_model_digest and requested_model_digest != self.info.model_bundle_digest:
            raise _bad("MODEL_DIGEST_MISMATCH", "requested model bundle is not loaded")
        return self._policy_info_message()

    def _policy_info_message(self) -> object:
        from dual_arm_policy import policy_pb2

        fields = {
            name: getattr(self.info, name) for name in self.info.__dataclass_fields__
        }
        for name in ("state_joint_names", "action_joint_names", "required_camera_names"):
            fields[name] = list(fields[name])
        return policy_pb2.PolicyInfo(**fields)

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
                    raise _bad("REQUEST_ID_CONFLICT", "open_request_id conflicts with content")
                session = self._sessions.get(session_id)
                if session is None:
                    raise _bad("UNKNOWN_SESSION", "cached open session no longer exists")
                return self._open_reply(session)
            if len(self._sessions) >= self.config.max_sessions:
                raise _bad("RESOURCE_EXHAUSTED", "session capacity is full")
            if len(self._open_cache) >= self.config.max_open_requests:
                raise _bad("RESOURCE_EXHAUSTED", "open request cache is full")
            session_id = f"policy-{uuid.uuid4()}"
            open_hook = getattr(self.adapter, "open_session", None)
            if callable(open_hook):
                open_hook(session_id)
            pipeline = ChunkedPolicyPipeline(
                self.adapter,
                self.config,
                prediction_directory=self.prediction_directory,
                prediction_max_bytes=self.prediction_max_bytes,
                budget=self._budget,
            )
            session = _Session(session_id, context, digest, calibrations, pipeline=pipeline)
            self._sessions[session_id] = session
            self._open_cache[open_request_id] = (digest, session_id)
        try:
            pipeline.warm_up()
        except BaseException:
            with self._lock:
                self._sessions.pop(session_id, None)
                self._open_cache.pop(open_request_id, None)
            raise
        return self._open_reply(session)

    def _open_reply(self, session: _Session) -> object:
        from dual_arm_policy import policy_pb2

        return policy_pb2.OpenSessionReply(
            session_id=session.session_id,
            context=self._context_message(session.context),
            policy=self._policy_info_message(),
        )

    @staticmethod
    def _context_message(context: RunContext) -> object:
        from dual_arm_policy import policy_pb2

        return policy_pb2.RunContext(**context.model_dump(mode="python"))

    def _validate_documents(
        self, profile_json: bytes, observation_json: bytes, action_json: bytes
    ) -> dict[str, str]:
        cfg = self.config
        try:
            profile = json.loads(profile_json)
            observation = json.loads(observation_json)
            action = json.loads(action_json)
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise _bad("SCHEMA_MISMATCH", f"session JSON is invalid: {error}") from error
        if not all(isinstance(item, dict) for item in (profile, observation, action)):
            raise _bad("SCHEMA_MISMATCH", "session documents must be JSON objects")
        try:
            profile_hash = semantic_hash(profile)
            observation_hash = semantic_hash(observation)
            action_hash = semantic_hash(action)
        except (TypeError, ValueError) as error:
            raise _bad("SCHEMA_MISMATCH", f"session JSON is not canonical: {error}") from error
        if profile_hash != self.info.robot_profile_hash:
            raise _bad("SCHEMA_MISMATCH", "robot_profile_json digest differs")
        if observation_hash != self.info.observation_schema_hash:
            raise _bad("SCHEMA_MISMATCH", "observation_schema_json digest differs")
        if action_hash != self.info.action_schema_hash:
            raise _bad("SCHEMA_MISMATCH", "action_schema_json digest differs")
        try:
            robot = RobotProfile.validate_dict(profile)
        except ContractError as error:
            raise _bad("SCHEMA_MISMATCH", f"robot profile is invalid: {error}") from error
        if robot.joint_names != cfg.joint_names:
            raise _bad("SCHEMA_MISMATCH", "robot profile joint order differs")
        if robot.camera_names != cfg.camera_names:
            raise _bad("SCHEMA_MISMATCH", "robot profile camera order differs")
        for camera in robot.cameras:
            if (camera.width, camera.height) != (cfg.camera_width, cfg.camera_height):
                raise _bad("SCHEMA_MISMATCH", f"profile camera {camera.name} dimensions differ")
            if not camera.calibration_digest:
                raise _bad("SCHEMA_MISMATCH", f"profile camera {camera.name} lacks a digest")
        if tuple(observation.get("joint_names", ())) != cfg.joint_names:
            raise _bad("SCHEMA_MISMATCH", "observation schema joint order differs")
        if tuple(observation.get("camera_names", ())) != cfg.camera_names:
            raise _bad("SCHEMA_MISMATCH", "observation schema camera order differs")
        if tuple(action.get("joint_names", ())) != cfg.action_joint_names:
            raise _bad("SCHEMA_MISMATCH", "action schema joint order differs")
        if action.get("command_space") != cfg.command_space:
            raise _bad("SCHEMA_MISMATCH", "action schema command space differs")
        return {camera.name: camera.calibration_digest for camera in robot.cameras}

    def _run_pipeline(
        self,
        session: _Session,
        request_id: str,
        context: RunContext,
        observation: ProviderObservation,
        started: int,
        deadline_ns: int | None,
    ) -> tuple[ProviderPrediction, PipelineResult]:
        if observation.observation_id <= session.last_observation_id:
            raise _bad("INVALID_OBSERVATION", "observations must advance monotonically")
        set_obs = getattr(self.adapter, "set_current_observation", None)
        if callable(set_obs):
            set_obs(session.session_id, request_id, context, observation)
        try:
            pipeline_result = session.pipeline.predict(
                identity=session.session_id + ":" + request_id,
                observation_id=observation.observation_id,
                observation_time_ns=observation.reference_robot_time_ns,
                cameras=observation.cameras,
                joint_positions=observation.joints.positions,
            )
        except PolicyError:
            raise
        except Exception as error:  # noqa: BLE001
            raise _bad("PROVIDER_ERROR", f"pipeline predict failed: {error}") from error
        limit_ns = int(self.config.predict_deadline_s * 1_000_000_000)
        limit_ns = limit_ns if deadline_ns is None else min(limit_ns, deadline_ns)
        if time.monotonic_ns() - started > limit_ns:
            raise _bad("DEADLINE_EXCEEDED", "prediction exceeded its deadline")
        prediction = self._adapter_prediction(
            pipeline_result, session.session_id, request_id, context, observation
        )
        transform = getattr(self.adapter, "transform_prediction", None)
        if callable(transform):
            prediction = transform(
                session.session_id, request_id, context, observation, prediction
            )
        return prediction, pipeline_result

    def _reply(
        self,
        session: _Session,
        request_id: str,
        observation: ProviderObservation,
        prediction: ProviderPrediction,
        inference_duration: int,
    ) -> object:
        from dual_arm_policy import policy_pb2

        return policy_pb2.PredictReply(
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

    def predict(self, request: object, deadline_ns: int | None = None) -> object:
        session = self._session(str(request.session_id))  # type: ignore[attr-defined]
        request_id = str(request.request_id)  # type: ignore[attr-defined]
        if not request_id:
            raise _bad("INVALID_REQUEST", "request_id must not be empty")
        context = _context_value(request.context)  # type: ignore[attr-defined]
        with session.lock:
            if context != session.context:
                raise _bad("STALE_CONTEXT", "predict context differs from the open session")
            observation = self._validate_observation(session, request.observation)  # type: ignore[attr-defined]
            digest = _predict_fingerprint(request)
            cached = session.request_cache.get(request_id)
            if cached is not None:
                previous_digest, reply = cached
                if previous_digest != digest:
                    raise _bad("REQUEST_ID_CONFLICT", "request_id conflicts with content")
                return reply
            if len(session.request_cache) >= self.config.max_request_cache:
                session.request_cache.popitem(last=False)
            started = time.monotonic_ns()
            prediction, pipeline_result = self._run_pipeline(
                session, request_id, context, observation, started, deadline_ns
            )
            target_count, inference_duration = self._validate_prediction(
                prediction, observation, context, session.session_id, request_id
            )
            session.pipeline.commit(pipeline_result)
            session.last_observation_id = observation.observation_id
            session.predictions += 1
            reply = self._reply(session, request_id, observation, prediction, inference_duration)
            assert len(reply.action.targets) == target_count
            session.request_cache[request_id] = (digest, reply)
            return reply

    def _adapter_prediction(
        self,
        result: PipelineResult,
        session_id: str,
        request_id: str,
        context: RunContext,
        observation: ProviderObservation,
    ) -> ProviderPrediction:
        period = self.config.action_period_ns
        targets = tuple(
            ProviderTarget((index + 1) * period, tuple(map(float, result.targets[index])))
            for index in range(self.config.chunk_size)
        )
        return ProviderPrediction(
            self.config.action_joint_names,
            targets,
            self.config.model_bundle_digest,
            result.inference_duration_ns,
            context,
            session_id,
            request_id,
            observation.observation_id,
        )

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
            raise _bad("INVALID_OBSERVATION", "reference time must be non-negative")
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
            raise _bad("INVALID_OBSERVATION", "positions do not cover configured joints")
        if velocities and len(velocities) != len(cfg.joint_names):
            raise _bad("INVALID_OBSERVATION", "velocities do not match configured joint count")
        if int(joints.state_seq) < 1:  # type: ignore[attr-defined]
            raise _bad("INVALID_OBSERVATION", "state_seq must be positive")
        if int(joints.captured_at_robot_time_ns) != reference:  # type: ignore[attr-defined]
            raise _bad("INVALID_OBSERVATION", "joint capture time differs")
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
                    str(camera.camera_name), int(camera.frame_seq),
                    int(camera.captured_at_robot_time_ns), str(camera.frame_id),
                    str(camera.calibration_digest), int(camera.width), int(camera.height),
                    int(camera.encoding), payload,
                )
            )
        if total_payload > cfg.max_message_bytes:
            raise _bad("RESOURCE_EXHAUSTED", "payload exceeds the configured bound")
        return ProviderObservation(
            int(observation.observation_id),
            reference,
            ProviderJointSample(
                int(joints.state_seq), int(joints.captured_at_robot_time_ns),
                names, positions, velocities,
            ),
            tuple(typed),
            str(observation.task_id),
            str(observation.task_text),
        )

    def _validate_prediction(
        self,
        prediction: ProviderPrediction,
        observation: ProviderObservation,
        observation_context: RunContext,
        session_id: str,
        request_id: str,
    ) -> tuple[int, int]:
        cfg = self.config
        if not isinstance(prediction, ProviderPrediction):
            raise _bad("INVALID_PREDICTION", "provider did not return ProviderPrediction")
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
        for index, target in enumerate(targets):
            if target.time_from_observation_ns != (index + 1) * cfg.action_period_ns:
                raise _bad("INVALID_PREDICTION", f"target[{index}] is off the 50 ms grid")
            positions = tuple(target.positions)
            if len(positions) != len(cfg.action_joint_names):
                raise _bad(
                    "INVALID_PREDICTION",
                    f"target[{index}] is not {len(cfg.action_joint_names)}-wide",
                )
            for axis, value in enumerate(positions):
                number = _finite(value, f"target[{index}].positions[{axis}]")
                if not cfg.joint_lower_bounds[axis] <= number <= cfg.joint_upper_bounds[axis]:
                    raise _bad("INVALID_PREDICTION", f"target[{index}] axis {axis} out of range")
            if observation.reference_robot_time_ns + target.time_from_observation_ns > 2**63 - 1:
                raise _bad("INVALID_PREDICTION", "target absolute time exceeds int64")
        duration = prediction.inference_duration_ns
        if isinstance(duration, bool) or not isinstance(duration, int) or duration < 0:
            raise _bad("INVALID_PREDICTION", "inference_duration_ns must be non-negative")
        return len(targets), duration

    def reset_session(self, request: object) -> object:
        from dual_arm_policy import policy_pb2

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
            session.pipeline.reset()
            session.context = context
            session.last_observation_id = 0
            session.predictions = 0
            session.request_cache.clear()
            reply = policy_pb2.ResetSessionReply(
                session_id=session.session_id,
                reset_id=reset_id,
                context=self._context_message(context),
                ready=True,
            )
            session.reset_cache[reset_id] = (_payload_digest(request), reply)  # type: ignore[arg-type]
            return reply

    def close_session(self, request: object) -> object:
        from dual_arm_policy import policy_pb2

        session_id = str(request.session_id)  # type: ignore[attr-defined]
        if not session_id:
            raise _bad("INVALID_REQUEST", "session_id must not be empty")
        with self._lock:
            session = self._sessions.pop(session_id, None)
        if session is not None:
            close_hook = getattr(self.adapter, "close", None)
            if callable(close_hook):
                with session.lock:
                    close_hook(session.session_id)
        return policy_pb2.CloseSessionReply(session_id=session_id, closed=True)


__all__ = ["PolicyService", "semantic_hash"]
