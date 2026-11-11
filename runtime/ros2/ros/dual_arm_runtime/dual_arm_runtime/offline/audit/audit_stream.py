"""Streaming MCAP handlers shared by the episode and follower audits."""

from __future__ import annotations

import hashlib
import math
from collections import Counter

from robot_contracts.execution_feedback import ControlEvent, ExecutionFeedback
from robot_contracts.models import ContractError, RunContext
from robot_contracts.profiles import normalize_cameras
from robot_contracts.stream_request import REQUEST_MOTION, StreamRequest
from robot_contracts.trajectory.follower_reference import FollowerReference

from dual_arm_runtime.offline.audit.audit_common import (
    _MAX_COMMANDS,
    _MAX_EVENTS,
    _MAX_FEEDBACK_PER_COMMAND,
    _MAX_METADATA_ENTRIES,
    _TERMINAL,
    record_error,
    validate_camera_frame,
)


class StreamAudit:
    """Collect bounded sequence and identity facts from one recorded stream."""

    reference_topic_name = "follower_references"
    required_capabilities = frozenset(
        {
            "exact_capture",
            "simulation_reset",
            "lease",
            "stop",
            "command_query",
            "control_events",
            "follower_reference",
            "bounded_target_follower_v1",
        }
    )

    def __init__(self, episode_id: str, configuration: dict) -> None:
        self.episode_id = episode_id
        self.configuration = configuration
        recipe = configuration.get("recipe")
        profile = configuration.get("profile")
        if not isinstance(recipe, dict) or not isinstance(profile, dict):
            raise record_error("runtime configuration snapshot is incomplete")
        self.root = str(recipe.get("robot_namespace", "")).rstrip("/")
        if not self.root:
            raise record_error("runtime configuration has no robot_namespace")
        self.reference_topic = f"{self.root}/{self.reference_topic_name}"
        self.expected_context = configuration.get("run_context")
        if not isinstance(self.expected_context, dict):
            raise record_error("runtime recording has no run_context")
        self.profile_document = profile
        try:
            self.cameras = normalize_cameras(profile, "<recorded profile>")
        except ContractError as error:
            raise record_error(f"invalid recorded profile cameras: {error}") from error
        self._profile_object = None
        self.camera_names = tuple(camera.name for camera in self.cameras)
        self.camera_specs = {camera.name: camera for camera in self.cameras}
        self.joint_names = tuple(
            str(item["name"]) for item in profile.get("joints", ()) if isinstance(item, dict)
        )
        if not self.joint_names:
            raise record_error("recorded profile has no joints")
        observation = recipe.get("observation")
        if not isinstance(observation, dict):
            observation = configuration.get("observation", {})
        if not isinstance(observation, dict):
            raise record_error("recorded observation snapshot is incomplete")
        _positive_int(observation.get("max_age_ns"), "max_age_ns")
        _positive_number(observation.get("wall_arrival_timeout_s"), "wall_arrival_timeout_s")
        motions = recipe.get("motions")
        if not isinstance(motions, list):
            raise record_error("runtime recipe has no motion list")
        self.motion_count = len(motions)
        control = recipe.get("control_constraints")
        if not isinstance(control, dict):
            raise record_error("runtime recipe has no control constraints")
        limits = control.get("max_accelerations")
        if not isinstance(limits, list) or len(limits) != len(self.joint_names):
            raise record_error("control max_accelerations do not match the profile")
        self.max_accelerations = tuple(float(value) for value in limits)
        if not all(math.isfinite(value) and value >= 0 for value in self.max_accelerations):
            raise record_error("control max_accelerations must be finite and non-negative")
        self.counts: Counter = Counter()
        self.skipped = 0
        self.states: dict[int, int] = {}
        self.frames: dict[tuple[str, int], int] = {}
        self.capture_digests: dict[tuple[str, int], bytes] = {}
        self.indices: list[tuple[int, int, tuple[str, ...], tuple[int, ...], int]] = []
        self.observation_ids: list[int] = []
        self.commands: dict[str, StreamRequest] = {}
        self.command_order: list[str] = []
        self.chunk_requests: dict[tuple[str, int], StreamRequest] = {}
        self.references: dict[str, dict[int, FollowerReference]] = {}
        self.reference_duplicates = 0
        self.events: dict[str, list[ControlEvent]] = {}
        self.event_sequences: list[int] = []
        self.feedback: dict[str, list[ExecutionFeedback]] = {}
        self.terminals: dict[str, ExecutionFeedback] = {}
        self.info_seen = 0
        self.motion_min: list[float] | None = None
        self.motion_max: list[float] | None = None
        self.topics: set[str] = set()

    def capture_payload(self, topic: str, message: object, payload: bytes) -> None:
        """Retain only hashes to detect conflicting copies of one exact capture."""
        if not self.context(message):
            return
        sequence = int(message.state_seq if topic.endswith("/state") else message.frame_seq)
        key = (topic, sequence)
        digest = hashlib.sha256(payload).digest()
        previous = self.capture_digests.get(key)
        if previous is not None and previous != digest:
            raise record_error("same capture identity has conflicting payloads")
        if previous is None and len(self.capture_digests) >= _MAX_METADATA_ENTRIES:
            raise record_error("capture digest metadata exceeds the audit bound")
        self.capture_digests[key] = digest

    def context_value(self, parsed: RunContext) -> bool:
        """Validate one already decoded immutable context."""
        if parsed.episode_id != self.episode_id:
            self.skipped += 1
            return False
        for key, value in self.expected_context.items():
            if getattr(parsed, key, None) != value:
                raise record_error(f"mixed context field: {key}")
        return True

    def context(self, message: object) -> bool:
        """Validate a message whose IDL places context at the top level."""
        return self.context_value(message.context)  # type: ignore[attr-defined]

    def info(self, message: object) -> None:
        if not self.context(message):
            return
        self.info_seen += 1
        if message.backend_kind != "mujoco":
            raise record_error("recorded backend kind is not mujoco")
        if bool(message.test_only):
            raise record_error("diagnostic recording is marked test_only")
        if message.context.robot_profile_hash != self.configuration["profile_hash"]:
            raise record_error("recorded backend profile differs")
        if tuple(message.joint_names) != self.joint_names:
            raise record_error("recorded backend joint order differs")
        if tuple(message.camera_names) != self.camera_names:
            raise record_error("recorded backend camera order differs")
        missing = self.required_capabilities - set(message.capabilities)
        if missing:
            raise record_error("recorded backend lacks required capabilities")

    def state(self, message: object) -> None:
        if not self.context(message):
            return
        if tuple(message.joint_names) != self.joint_names:
            raise record_error("recorded state joint order differs")
        positions = tuple(float(value) for value in message.positions)
        if len(positions) != len(self.joint_names) or not all(
            math.isfinite(value) for value in positions
        ):
            raise record_error("recorded state positions are invalid")
        velocities = tuple(float(value) for value in message.velocities)
        if len(velocities) != len(self.joint_names) or not all(
            math.isfinite(value) for value in velocities
        ):
            raise record_error("recorded state velocities are invalid")
        stamp = message.stamp_ns
        if stamp < 0 or message.joints_stamp_ns != stamp:
            raise record_error("recorded state stamp is invalid")
        sequence = int(message.state_seq)
        if sequence < 1:
            raise record_error("recorded state sequence is invalid")
        if sequence in self.states and self.states[sequence] != stamp:
            raise record_error("same state sequence has a different stamp")
        if sequence not in self.states and len(self.states) >= _MAX_METADATA_ENTRIES:
            raise record_error("state metadata exceeds the audit bound")
        self.states[sequence] = stamp
        if self.motion_min is None:
            self.motion_min = list(positions)
            self.motion_max = list(positions)
        else:
            assert self.motion_max is not None
            for index, value in enumerate(positions):
                self.motion_min[index] = min(self.motion_min[index], value)
                self.motion_max[index] = max(self.motion_max[index], value)

    def frame(self, topic: str, message: object) -> None:
        if not self.context(message):
            return
        camera_name = topic.split("/")[-2]
        camera = self.camera_specs.get(camera_name)
        if camera is None:
            raise record_error(f"unknown camera topic {camera_name}")
        validate_camera_frame(message, camera)
        stamp = message.stamp_ns
        sequence = int(message.frame_seq)
        if sequence < 1:
            raise record_error(f"invalid frame sequence for {camera_name}")
        key = (camera_name, sequence)
        if key in self.frames and self.frames[key] != stamp:
            raise record_error(f"same camera frame sequence has a different stamp: {camera_name}")
        if key not in self.frames and len(self.frames) >= _MAX_METADATA_ENTRIES:
            raise record_error("frame metadata exceeds the audit bound")
        self.frames[key] = stamp

    def observation(self, message: object) -> None:
        if not self.context(message):
            return
        names = tuple(message.camera_names)
        sequences = tuple(int(value) for value in message.frame_seqs)
        if names != self.camera_names or len(sequences) != len(names):
            raise record_error("observation camera names/order differ")
        if len(self.indices) >= _MAX_METADATA_ENTRIES:
            raise record_error("observation metadata exceeds the audit bound")
        self.observation_ids.append(int(message.observation_id))
        self.indices.append(
            (
                int(message.observation_id),
                int(message.state_seq),
                names,
                sequences,
                message.reference_time_ns,
            )
        )

    def command(self, message: object) -> None:
        request = message
        if not self.context_value(request.trajectory.context):
            return
        if request.kind == REQUEST_MOTION:
            raise record_error("MOTION command is not part of a follower recording")
        anchor = request.anchor
        if anchor is None:
            raise record_error("follower command has no stream anchor")
        command_id = request.trajectory.command_id
        identity = (command_id, anchor.predecessor_seq)
        previous = self.chunk_requests.get(identity)
        if previous is not None:
            if previous != request:
                raise record_error(f"same command/chunk ID has different content: {identity}")
            return
        if len(self.chunk_requests) >= _MAX_COMMANDS:
            raise record_error("chunk request metadata exceeds the audit bound")
        existing = self.commands.get(command_id)
        if existing is not None:
            if not (
                existing.trajectory.context == request.trajectory.context
                and existing.trajectory.lease_id == request.trajectory.lease_id
                and existing.trajectory.command_seq == request.trajectory.command_seq
                and existing.trajectory.joint_names == request.trajectory.joint_names
            ):
                raise record_error(f"same command ID has different identity: {command_id}")
        else:
            self.commands[command_id] = request
            self.command_order.append(command_id)
        self.chunk_requests[identity] = request

    def event(self, message: object) -> None:
        if not self.context(message):
            return
        event = message
        command_id = event.command_id
        if command_id not in self.commands:
            raise record_error(f"control event references an unknown command: {command_id}")
        request = self.commands[command_id]
        if event.is_chunk:
            request = self.chunk_requests.get((command_id, event.chunk_seq - 1))
            if request is None:
                raise record_error("chunk control event references an unknown request")
        if (
            event.command_seq != request.trajectory.command_seq
            or event.observation_id != request.trajectory.observation_id
        ):
            raise record_error("control event command identity differs")
        if len(self.event_sequences) >= _MAX_EVENTS:
            raise record_error("control event metadata exceeds the audit bound")
        self.event_sequences.append(event.event_seq)
        self.events.setdefault(command_id, []).append(event)

    def feedback_message(self, message: object) -> None:
        if not self.context(message):
            return
        execution = message
        request = self.commands.get(execution.command_id)
        if request is None:
            raise record_error(
                f"execution feedback references an unknown command: {execution.command_id}"
            )
        trajectory = request.trajectory
        if (
            execution.command_seq != trajectory.command_seq
            or execution.observation_id != trajectory.observation_id
            or execution.joint_names != trajectory.joint_names
        ):
            raise record_error("execution feedback identity differs")
        if execution.context.robot_profile_hash != self.configuration["profile_hash"]:
            raise record_error("execution feedback profile differs")
        history = self.feedback.setdefault(execution.command_id, [])
        if len(history) >= _MAX_FEEDBACK_PER_COMMAND:
            raise record_error("execution feedback history exceeds the audit bound")
        history.append(execution)
        if execution.phase in _TERMINAL:
            previous = self.terminals.get(execution.command_id)
            if previous is not None and previous != execution:
                raise record_error(
                    f"terminal execution feedback changed after freeze: {execution.command_id}"
                )
            self.terminals[execution.command_id] = execution
        elif execution.command_id in self.terminals:
            raise record_error("execution feedback continued after a terminal freeze")


def _positive_int(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise record_error(f"{name} must be a positive integer")
    return value


def _positive_number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise record_error(f"{name} must be a positive number")
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise record_error(f"{name} must be a positive number")
    return number


__all__ = ["StreamAudit"]
