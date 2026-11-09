"""Streaming MCAP audit for a continuous bounded-follower policy episode.

The bounded reader keeps only sequence/stamp maps, per-command event and
feedback summaries and per-camera references.  Camera payloads are validated
and discarded, never collected into an episode-sized image cache.  This module
is the only recording auditor: the legacy strict-motion audit was deleted with
the single-shot control path.
"""

from __future__ import annotations

import hashlib
import math
from collections import Counter
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from robot_contracts.execution_feedback import ControlEvent, ExecutionFeedback
from robot_contracts.models import ContractError, RunContext
from robot_contracts.profiles import normalize_cameras
from robot_contracts.stream_request import REQUEST_MOTION, StreamRequest
from robot_contracts.trajectory import (
    FollowerReference,
    FollowerValidationLimits,
)
from robot_contracts.trajectory import (
    validate as validate_follower_reference,
)
from robot_ros_support.conversions import context_from_message, time_ns
from robot_ros_support.follower_conversions import follower_reference_from_message
from robot_ros_support.stream_conversions import (
    control_event_from_message,
    execution_feedback_from_message,
    stream_command_from_message,
)
from rosidl_runtime_py.utilities import get_message

from dual_arm_runtime.policy_audit_rules import require_completed_chunks

_TERMINAL = frozenset({"COMPLETED", "CANCELED", "FAILED"})
_MAX_FEEDBACK_PER_COMMAND = 4096
_MAX_METADATA_ENTRIES = 1_000_000
_MAX_COMMANDS = 4096
_MAX_EVENTS = 100_000
_TERMINAL_EVENT_PHASES = frozenset({"COMPLETED", "CANCELED", "FAILED"})


def _validate_camera_frame(message: object, camera: object) -> None:
    """Validate one recorded camera frame against its declared profile entry."""
    name = camera.name  # type: ignore[attr-defined]
    image = message.image  # type: ignore[attr-defined]
    if (image.width, image.height) != (camera.width, camera.height):  # type: ignore[attr-defined]
        raise _record_error(f"Camera dimensions differ: {name}")
    if image.encoding != "rgb8" or len(image.data) != image.width * image.height * 3:
        raise _record_error(f"Invalid RGB frame: {name}")
    if image.step != image.width * 3:
        raise _record_error(f"Camera RGB step differs: {name}")
    if image.header.frame_id != camera.frame_id:  # type: ignore[attr-defined]
        raise _record_error(f"Camera frame_id differs: {name}")
    camera_info = message.camera_info  # type: ignore[attr-defined]
    if (camera_info.width, camera_info.height) != (camera.width, camera.height):  # type: ignore[attr-defined]
        raise _record_error(f"CameraInfo dimensions differ: {name}")
    if camera_info.header.frame_id != camera.frame_id:  # type: ignore[attr-defined]
        raise _record_error(f"CameraInfo frame_id differs: {name}")
    if message.calibration_digest != camera.calibration_digest:  # type: ignore[attr-defined]
        raise _record_error(f"Camera calibration differs: {name}")


def _record_error(message: str) -> ContractError:
    return ContractError("RECORDING_ERROR", message)


def _close(left: float, right: float, tolerance: float) -> bool:
    return math.isclose(left, right, rel_tol=1e-9, abs_tol=tolerance)


def _vectors_close(left: tuple[float, ...], right: tuple[float, ...], tolerance: float) -> bool:
    return len(left) == len(right) and all(
        _close(a, b, tolerance) for a, b in zip(left, right, strict=True)
    )


class _Audit:
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
            raise _record_error("runtime configuration snapshot is incomplete")
        self.root = str(recipe.get("robot_namespace", "")).rstrip("/")
        if not self.root:
            raise _record_error("runtime configuration has no robot_namespace")
        self.reference_topic = f"{self.root}/{self.reference_topic_name}"
        self.expected_context = configuration.get("run_context")
        if not isinstance(self.expected_context, dict):
            raise _record_error("runtime recording has no run_context")
        self.profile_document = profile
        try:
            self.cameras = normalize_cameras(profile, "<recorded profile>")
        except ContractError as error:
            raise _record_error(f"invalid recorded profile cameras: {error}") from error
        self._profile_object = None
        self.camera_names = tuple(camera.name for camera in self.cameras)
        self.camera_specs = {camera.name: camera for camera in self.cameras}
        self.joint_names = tuple(
            str(item["name"]) for item in profile.get("joints", ()) if isinstance(item, dict)
        )
        if not self.joint_names:
            raise _record_error("recorded profile has no joints")
        observation = recipe.get("observation")
        if not isinstance(observation, dict):
            observation = configuration.get("observation", {})
        if not isinstance(observation, dict):
            raise _record_error("recorded observation snapshot is incomplete")
        _positive_int(observation.get("max_age_ns"), "max_age_ns")
        _positive_number(observation.get("wall_arrival_timeout_s"), "wall_arrival_timeout_s")
        motions = recipe.get("motions")
        if not isinstance(motions, list):
            raise _record_error("runtime recipe has no motion list")
        self.motion_count = len(motions)
        control = recipe.get("control_constraints")
        if not isinstance(control, dict):
            raise _record_error("runtime recipe has no control constraints")
        limits = control.get("max_accelerations")
        if not isinstance(limits, list) or len(limits) != len(self.joint_names):
            raise _record_error("control max_accelerations do not match the profile")
        self.max_accelerations = tuple(float(value) for value in limits)
        if not all(math.isfinite(value) and value >= 0 for value in self.max_accelerations):
            raise _record_error("control max_accelerations must be finite and non-negative")
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

    # ---- Common context -----------------------------------------------------------

    def capture_payload(self, topic: str, message: object, payload: bytes) -> None:
        """Retain only hashes to detect conflicting copies of one exact capture."""
        if not self.context(message):
            return
        sequence = int(message.state_seq if topic.endswith("/state") else message.frame_seq)
        key = (topic, sequence)
        digest = hashlib.sha256(payload).digest()
        previous = self.capture_digests.get(key)
        if previous is not None and previous != digest:
            raise _record_error("same capture identity has conflicting payloads")
        if previous is None and len(self.capture_digests) >= _MAX_METADATA_ENTRIES:
            raise _record_error("capture digest metadata exceeds the audit bound")
        self.capture_digests[key] = digest

    def context_value(self, parsed: RunContext) -> bool:
        """Validate one already decoded immutable context."""
        if parsed.episode_id != self.episode_id:
            self.skipped += 1
            return False
        for key, value in self.expected_context.items():
            if getattr(parsed, key, None) != value:
                raise _record_error(f"mixed context field: {key}")
        return True

    def context(self, message: object) -> bool:
        """Validate a message whose IDL places context at the top level."""
        return self.context_value(context_from_message(message.context))  # type: ignore[attr-defined]

    # ---- Stream handlers ----------------------------------------------------------

    def info(self, message: object) -> None:
        if not self.context(message):
            return
        self.info_seen += 1
        if message.backend_kind != "mujoco":
            raise _record_error("recorded backend kind is not mujoco")
        if bool(message.test_only):
            raise _record_error("diagnostic recording is marked test_only")
        if message.context.robot_profile_hash != self.configuration["profile_hash"]:
            raise _record_error("recorded backend profile differs")
        if tuple(message.joint_names) != self.joint_names:
            raise _record_error("recorded backend joint order differs")
        if tuple(message.camera_names) != self.camera_names:
            raise _record_error("recorded backend camera order differs")
        missing = self.required_capabilities - set(message.capabilities)
        if missing:
            raise _record_error("recorded backend lacks required capabilities")

    def state(self, message: object) -> None:
        if not self.context(message):
            return
        name = self.root + "/state"
        if tuple(message.joints.name) != self.joint_names:
            raise _record_error("recorded state joint order differs")
        positions = tuple(float(value) for value in message.joints.position)
        if len(positions) != len(self.joint_names) or not all(
            math.isfinite(value) for value in positions
        ):
            raise _record_error("recorded state positions are invalid")
        velocities = tuple(float(value) for value in message.joints.velocity)
        if len(velocities) != len(self.joint_names) or not all(
            math.isfinite(value) for value in velocities
        ):
            raise _record_error("recorded state velocities are invalid")
        stamp = time_ns(message.stamp)
        if stamp < 0 or time_ns(message.joints.header.stamp) != stamp:
            raise _record_error("recorded state stamp is invalid")
        sequence = int(message.state_seq)
        if sequence < 1:
            raise _record_error("recorded state sequence is invalid")
        if sequence in self.states and self.states[sequence] != stamp:
            raise _record_error("same state sequence has a different stamp")
        if sequence not in self.states and len(self.states) >= _MAX_METADATA_ENTRIES:
            raise _record_error("state metadata exceeds the audit bound")
        self.states[sequence] = stamp
        if self.motion_min is None:
            self.motion_min = list(positions)
            self.motion_max = list(positions)
        else:
            assert self.motion_max is not None
            for index, value in enumerate(positions):
                self.motion_min[index] = min(self.motion_min[index], value)
                self.motion_max[index] = max(self.motion_max[index], value)
        del name

    def frame(self, topic: str, message: object) -> None:
        if not self.context(message):
            return
        camera_name = topic.split("/")[-2]
        camera = self.camera_specs.get(camera_name)
        if camera is None:
            raise _record_error(f"unknown camera topic {camera_name}")
        _validate_camera_frame(message, camera)
        stamp = time_ns(message.image.header.stamp)
        sequence = int(message.frame_seq)
        if sequence < 1:
            raise _record_error(f"invalid frame sequence for {camera_name}")
        key = (camera_name, sequence)
        if key in self.frames and self.frames[key] != stamp:
            raise _record_error(f"same camera frame sequence has a different stamp: {camera_name}")
        if key not in self.frames and len(self.frames) >= _MAX_METADATA_ENTRIES:
            raise _record_error("frame metadata exceeds the audit bound")
        self.frames[key] = stamp

    def observation(self, message: object) -> None:
        if not self.context(message):
            return
        names = tuple(message.camera_names)
        sequences = tuple(int(value) for value in message.frame_seqs)
        if names != self.camera_names or len(sequences) != len(names):
            raise _record_error("observation camera names/order differ")
        if len(self.indices) >= _MAX_METADATA_ENTRIES:
            raise _record_error("observation metadata exceeds the audit bound")
        self.observation_ids.append(int(message.observation_id))
        self.indices.append(
            (
                int(message.observation_id),
                int(message.state_seq),
                names,
                sequences,
                time_ns(message.reference_time),
            )
        )

    def command(self, message: object) -> None:
        request = stream_command_from_message(message)
        if not self.context_value(request.trajectory.context):
            return
        if request.kind == REQUEST_MOTION:
            raise _record_error("MOTION command is not part of a follower recording")
        anchor = request.anchor
        if anchor is None:
            raise _record_error("follower command has no stream anchor")
        command_id = request.trajectory.command_id
        identity = (command_id, anchor.predecessor_seq)
        previous = self.chunk_requests.get(identity)
        if previous is not None:
            if previous != request:
                raise _record_error(f"same command/chunk ID has different content: {identity}")
            return
        if len(self.chunk_requests) >= _MAX_COMMANDS:
            raise _record_error("chunk request metadata exceeds the audit bound")
        existing = self.commands.get(command_id)
        if existing is not None:
            if not (
                existing.trajectory.context == request.trajectory.context
                and existing.trajectory.lease_id == request.trajectory.lease_id
                and existing.trajectory.command_seq == request.trajectory.command_seq
                and existing.trajectory.joint_names == request.trajectory.joint_names
            ):
                raise _record_error(f"same command ID has different identity: {command_id}")
        else:
            self.commands[command_id] = request
            self.command_order.append(command_id)
        self.chunk_requests[identity] = request

    def event(self, message: object) -> None:
        if not self.context(message):
            return
        event = control_event_from_message(message)
        command_id = event.command_id
        if command_id not in self.commands:
            raise _record_error(f"control event references an unknown command: {command_id}")
        request = self.commands[command_id]
        if event.is_chunk:
            request = self.chunk_requests.get((command_id, event.chunk_seq - 1))
            if request is None:
                raise _record_error("chunk control event references an unknown request")
        if (
            event.command_seq != request.trajectory.command_seq
            or event.observation_id != request.trajectory.observation_id
        ):
            raise _record_error("control event command identity differs")
        if len(self.event_sequences) >= _MAX_EVENTS:
            raise _record_error("control event metadata exceeds the audit bound")
        self.event_sequences.append(event.event_seq)
        self.events.setdefault(command_id, []).append(event)

    def feedback_message(self, message: object) -> None:
        if not self.context(message):
            return
        execution = execution_feedback_from_message(message)
        request = self.commands.get(execution.command_id)
        if request is None:
            raise _record_error(
                f"execution feedback references an unknown command: {execution.command_id}"
            )
        trajectory = request.trajectory
        if (
            execution.command_seq != trajectory.command_seq
            or execution.observation_id != trajectory.observation_id
            or execution.joint_names != trajectory.joint_names
        ):
            raise _record_error("execution feedback identity differs")
        if execution.context.robot_profile_hash != self.configuration["profile_hash"]:
            raise _record_error("execution feedback profile differs")
        history = self.feedback.setdefault(execution.command_id, [])
        if len(history) >= _MAX_FEEDBACK_PER_COMMAND:
            raise _record_error("execution feedback history exceeds the audit bound")
        history.append(execution)
        if execution.phase in _TERMINAL:
            previous = self.terminals.get(execution.command_id)
            if previous is not None and previous != execution:
                raise _record_error(
                    f"terminal execution feedback changed after freeze: {execution.command_id}"
                )
            self.terminals[execution.command_id] = execution
        elif execution.command_id in self.terminals:
            raise _record_error("execution feedback continued after a terminal freeze")

    # ---- Final checks -------------------------------------------------------------

    def _profile(self):
        from robot_contracts.models import JointSpec, RobotProfile

        if self._profile_object is not None:
            return self._profile_object
        document = self.profile_document
        joints = tuple(
            JointSpec(
                item["name"],
                item["unit"],
                float(item["min"]),
                float(item["max"]),
                float(item["max_velocity"]),
                float(item["initial"]),
            )
            for item in document["joints"]
        )
        self._profile_object = RobotProfile(
            robot_id=document["robot_id"],
            profile_id=document["profile_id"],
            digest=self.configuration["profile_hash"],
            test_only=bool(document.get("test_only", False)),
            joints=joints,
            cameras=self.cameras,
        )
        return self._profile_object

    def finish(self) -> dict:
        if not self.info_seen:
            raise _record_error("no BackendInfo was recorded for the episode")
        previous = 0
        for observation_id in self.observation_ids:
            if observation_id != previous + 1:
                raise _record_error("observation IDs are not contiguous from one")
            previous = observation_id
        observation_set = set(self.observation_ids)
        for observation_id, state_seq, names, sequences, reference_time_ns in self.indices:
            del observation_id
            state_stamp = self.states.get(state_seq)
            if state_stamp is None:
                raise _record_error("observation references a missing state")
            if reference_time_ns != state_stamp:
                raise _record_error("observation reference time differs from its exact capture")
            for camera, sequence in zip(names, sequences, strict=True):
                frame_stamp = self.frames.get((camera, sequence))
                if frame_stamp is None:
                    raise _record_error("observation references a missing frame")
                if frame_stamp != state_stamp:
                    raise _record_error("exact capture stamps differ")
                if sequence != state_seq:
                    raise _record_error("exact capture state/frame sequences differ")
        previous_sequence = 0
        for command_id in self.command_order:
            request = self.commands[command_id]
            command = request.trajectory
            if command.command_seq <= previous_sequence:
                raise _record_error("stream command sequences are not increasing")
            previous_sequence = command.command_seq
            if command.observation_id not in observation_set:
                raise _record_error("stream command references a missing observation")
            if command.joint_names != self.joint_names:
                raise _record_error("stream command joint order differs")
            if command.context.robot_profile_hash != self.configuration["profile_hash"]:
                raise _record_error("stream command profile differs")
        for request in self.chunk_requests.values():
            trajectory = request.trajectory
            if trajectory.observation_id not in observation_set:
                raise _record_error("chunk request references a missing observation")
            if trajectory.joint_names != self.joint_names:
                raise _record_error("chunk request joint order differs")
        for command_id, by_key in self.references.items():
            for key, record in by_key.items():
                if record.request.trajectory.observation_id not in observation_set:
                    raise _record_error(
                        f"follower reference {command_id}/{key} references a missing observation"
                    )
        self._check_event_sequences()
        missing_terminals = [
            command_id for command_id in self.references if command_id not in self.terminals
        ]
        if missing_terminals:
            raise _record_error(
                "accepted command has no terminal feedback: " + missing_terminals[0]
            )
        self._check_events()
        self._check_feedback_references()
        completed = sum(
            1 for execution in self.terminals.values() if execution.phase == "COMPLETED"
        )
        motion = 0.0
        if self.motion_min is not None and self.motion_max is not None:
            motion = max(
                high - low for low, high in zip(self.motion_min, self.motion_max, strict=True)
            )
        return {
            "message_counts": dict(self.counts),
            "observations": len(self.indices),
            "commands": len(self.commands),
            "frames": len(self.frames),
            "commands_without_terminal": len(self.commands) - len(self.terminals),
            "terminal_events": sum(
                1
                for history in self.events.values()
                for event in history
                if not getattr(event, "is_chunk", False)
                and getattr(event, "phase", "") in _TERMINAL_EVENT_PHASES
            ),
            "follower_references": sum(len(value) for value in self.references.values()),
            "reference_duplicates": self.reference_duplicates,
            "feedback_commands": len(self.terminals),
            "completed_commands": completed,
            "configured_motions": self.motion_count,
            "max_joint_motion": motion,
            "skipped_context_messages": self.skipped,
        }

    def _check_event_sequences(self) -> None:
        sequences = self.event_sequences
        for left, right in zip(sequences, sequences[1:], strict=False):
            if right != left + 1:
                raise _record_error("control event sequence gap or duplicate")
        if sequences and sequences[0] < 1:
            raise _record_error("control event sequence must start positive")

    @staticmethod
    def _overall_history(history: list[ControlEvent]) -> list[ControlEvent]:
        return sorted(
            (event for event in history if not event.is_chunk),
            key=lambda item: item.event_seq,
        )

    def _check_overall_events(
        self, command_id: str, reference: FollowerReference, history: list[ControlEvent]
    ) -> None:
        overall = self._overall_history(history)
        if not overall:
            raise _record_error(f"accepted reference lacks overall control events: {command_id}")
        phases = [event.phase for event in overall]
        if phases[0] != "ACCEPTED" or phases.count("ACCEPTED") != 1:
            raise _record_error(f"ACCEPTED is not unique and first: {command_id}")
        terminal_position = next(
            (index for index, phase in enumerate(phases) if phase in _TERMINAL_EVENT_PHASES),
            None,
        )
        if terminal_position is None:
            raise _record_error(f"accepted reference lacks overall terminal event: {command_id}")
        if terminal_position != len(phases) - 1:
            raise _record_error(f"overall events continue after terminal: {command_id}")
        if "STARTED" in phases and phases.index("STARTED") != 1:
            raise _record_error(f"overall STARTED is out of order: {command_id}")
        if overall[0].sim_time_ns != reference.accepted_sim_time_ns:
            raise _record_error("overall ACCEPTED time differs from the accepted reference")
        terminal = self.terminals[command_id]
        terminal_event = overall[-1]
        if terminal_event.phase != terminal.phase:
            raise _record_error("overall terminal event phase differs from feedback")
        if terminal.has_started:
            if "STARTED" not in phases:
                raise _record_error("feedback started without an overall STARTED event")
            if overall[1].sim_time_ns != terminal.started_sim_time_ns:
                raise _record_error("overall STARTED time differs from feedback")
        elif "STARTED" in phases:
            raise _record_error("overall STARTED event without started feedback")
        if terminal_event.sim_time_ns != terminal.finished_sim_time_ns:
            raise _record_error("overall terminal event time differs from feedback")

    def _check_chunk_events(
        self, command_id: str, by_key: dict[int, FollowerReference], history: list[object]
    ) -> None:
        chunk_events = [event for event in history if getattr(event, "is_chunk", False)]
        keys = sorted(by_key)
        if keys != list(range(len(keys))):
            raise _record_error("chunk reference sequence has a gap")
        for key in keys:
            chunk_seq = key + 1
            events = sorted(
                (event for event in chunk_events if event.chunk_seq == chunk_seq),
                key=lambda item: item.event_seq,
            )
            if not events:
                raise _record_error(
                    f"chunk reference lacks control events: {command_id}/{chunk_seq}"
                )
            phases = [event.phase for event in events]
            if phases[0] != "ACCEPTED" or phases.count("ACCEPTED") != 1:
                raise _record_error(f"chunk ACCEPTED is not unique and first: {chunk_seq}")
            if phases[-1] not in {"RETIRED", "CANCELED", "REJECTED"}:
                raise _record_error(f"chunk has no terminal chunk event: {chunk_seq}")
            if len(phases) > 2 and phases[1] != "STARTED":
                raise _record_error(f"chunk STARTED is out of order: {chunk_seq}")
            if len(phases) > 3:
                raise _record_error(f"chunk events continue after terminal: {chunk_seq}")
            if events[0].sim_time_ns != by_key[key].accepted_sim_time_ns:
                raise _record_error("chunk ACCEPTED time differs from its reference")
            if self.terminals[command_id].phase == "COMPLETED" and phases[-1] != "RETIRED":
                raise _record_error("completed stream has an unretired chunk")

    def _check_events(self) -> None:
        for command_id, by_key in self.references.items():
            history = self.events.get(command_id, [])
            if not history:
                raise _record_error(f"accepted reference lacks control events: {command_id}")
            first = min(by_key.values(), key=lambda record: record.accepted_sim_time_ns)
            self._check_overall_events(command_id, first, history)
            self._check_chunk_events(command_id, by_key, history)
        for command_id in self.commands:
            if command_id in self.references:
                continue
            phases = {event.phase for event in self.events.get(command_id, ())}
            if phases & {"ACCEPTED", "STARTED", "COMPLETED", "RETIRED"}:
                raise _record_error("control event claims acceptance without a recorded reference")

    def _check_feedback_references(self) -> None:
        for command_id, history in self.feedback.items():
            by_key = self.references.get(command_id)
            if not by_key:
                continue
            for execution in history:
                if execution.reference_kind not in {"TRAJECTORY", "CHUNK"}:
                    # BRAKE/HOLD are real applied references, but they must never
                    # be reconstructed from the original predicted plan.
                    continue
                if not execution.has_started:
                    raise _record_error("trajectory feedback has no actual started time")
                if execution.chunk_seq < 1:
                    raise _record_error("chunk feedback has no chunk sequence")
                record = by_key.get(execution.chunk_seq - 1)
                if record is None:
                    raise _record_error("chunk feedback lacks an accepted reference")
                actual_started = None
                try:
                    positions, velocities, accelerations, origin = self.reference_sample(
                        record, execution, actual_started
                    )
                except ContractError as error:
                    raise _record_error(f"reference reconstruction failed: {error}") from error
                if not _vectors_close(positions, execution.reference_positions, 1e-12):
                    raise _record_error("feedback reference positions differ from the plan")
                if not _vectors_close(velocities, execution.reference_velocities, 1e-12):
                    raise _record_error("feedback reference velocities differ from the plan")
                if not _vectors_close(
                    accelerations, execution.reference_accelerations, 1e-9
                ):
                    raise _record_error("feedback reference accelerations differ from the plan")
                expected_elapsed = execution.reference_application_sim_time_ns - origin
                if execution.reference_elapsed_ns != expected_elapsed:
                    raise _record_error("feedback elapsed is not the actual application time")


def _positive_int(value: object, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise _record_error(f"{name} must be a positive integer")
    return value


def _positive_number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise _record_error(f"{name} must be a positive number")
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise _record_error(f"{name} must be a positive number")
    return number


def _run_audit(audit: _Audit, directory: Path) -> dict:
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=str(directory), storage_id="mcap"),
            rosbag2_py.ConverterOptions("", ""),
        )
    except Exception as error:
        raise _record_error(f"cannot open recording: {error}") from error
    types = {topic.name: get_message(topic.type) for topic in reader.get_all_topics_and_types()}
    audit.topics = set(types)
    required = {
        f"{audit.root}/info",
        f"{audit.root}/state",
        f"{audit.root}/control_events",
        f"{audit.root}/execution_feedback",
        audit.reference_topic,
        "/runtime/commands",
        "/runtime/observation_index",
    } | {f"{audit.root}/cameras/{name}/frame" for name in audit.camera_names}
    missing = required - audit.topics
    if missing:
        raise _record_error("recording is missing topics: " + ", ".join(sorted(missing)))
    camera_topics = {f"{audit.root}/cameras/{name}/frame" for name in audit.camera_names}
    while reader.has_next():
        topic, payload, _bag_time_ns = reader.read_next()
        message = deserialize_message(payload, types[topic])
        audit.counts[topic] += 1
        try:
            if topic == f"{audit.root}/info":
                audit.info(message)
            elif topic == f"{audit.root}/state":
                audit.capture_payload(topic, message, payload)
                audit.state(message)
            elif topic in camera_topics:
                audit.capture_payload(topic, message, payload)
                audit.frame(topic, message)
            elif topic == "/runtime/observation_index":
                audit.observation(message)
            elif topic == "/runtime/commands":
                audit.command(message)
            elif topic == audit.reference_topic:
                audit.reference(message)
            elif topic == f"{audit.root}/control_events":
                audit.event(message)
            elif topic == f"{audit.root}/execution_feedback":
                audit.feedback_message(message)
        except ContractError:
            raise
        except Exception as error:
            raise _record_error(f"{topic}: {type(error).__name__}: {error}") from error
    return audit.finish()


def _close(left: float, right: float, tolerance: float) -> bool:
    return math.isclose(left, right, rel_tol=1e-9, abs_tol=tolerance)


def _vectors_close(left: tuple[float, ...], right: tuple[float, ...], tolerance: float) -> bool:
    return len(left) == len(right) and all(
        _close(a, b, tolerance) for a, b in zip(left, right, strict=True)
    )


class _FollowerAudit(_Audit):
    def __init__(self, episode_id: str, configuration: dict) -> None:
        super().__init__(episode_id, configuration)
        recipe = configuration.get("recipe")
        if not isinstance(recipe, dict):
            raise _record_error("follower recipe is missing")
        action_source = str(recipe.get("action_source", "policy"))
        section = "teacher" if action_source == "teacher" else "policy"
        policy = recipe.get(section)
        follower = recipe.get("follower")
        if not isinstance(policy, dict) or not isinstance(follower, dict):
            raise _record_error(
                f"{section}/follower configuration is missing from the recording"
            )
        self.action_source = action_source
        self.termination_mode = str(policy.get("termination_mode", "finite_chunks"))
        if self.termination_mode not in {"finite_chunks", "task"}:
            raise _record_error("unknown stream termination mode")
        control = recipe.get("control_constraints")
        if not isinstance(control, dict):
            raise _record_error("control constraints are missing from the recording")
        accelerations = control.get("max_accelerations")
        deviations = follower.get("max_target_deviations")
        if not isinstance(accelerations, list) or not isinstance(deviations, list):
            raise _record_error("follower limits are incomplete")
        if len(accelerations) != len(self.joint_names) or len(deviations) != len(self.joint_names):
            raise _record_error("follower limits do not match the profile joints")
        self.policy = policy
        self.follower = follower
        self.limits = FollowerValidationLimits(
            joint_names=tuple(self.joint_names),
            max_accelerations=tuple(float(value) for value in accelerations),
            max_target_deviations=tuple(float(value) for value in deviations),
            target_count=int(follower["target_count"]),
            max_terminal_tail_duration_ns=int(follower["max_terminal_tail_duration_ns"]),
            duration_grid_ns=int(follower["duration_grid_ns"]),
        )
        self.applied_chunks: set[tuple[str, int]] = set()
        self.final_references = 0

    def reference(self, message: object) -> None:
        record = follower_reference_from_message(message)
        if not self.context_value(record.request.trajectory.context):
            return
        command_id = record.request.trajectory.command_id
        request = self.commands.get(command_id)
        if request is None:
            raise _record_error(f"follower reference precedes its command: {command_id}")
        if request.kind == REQUEST_MOTION:
            raise _record_error("MOTION command cannot carry a follower reference")
        anchor = record.request.anchor
        if anchor is None:
            raise _record_error("follower reference has no anchor")
        key = anchor.predecessor_seq
        if self.chunk_requests.get((command_id, key)) != record.request:
            raise _record_error(
                f"follower reference differs from its raw chunk request: {command_id}/{key}"
            )
        try:
            validate_follower_reference(record, self._profile(), self.limits)
        except ContractError as error:
            raise _record_error(f"follower reference validation failed: {error}") from error
        by_key = self.references.setdefault(command_id, {})
        existing = by_key.get(key)
        if existing is not None:
            if existing.reference_digest != record.reference_digest:
                raise _record_error(
                    f"same follower chunk has conflicting references: {command_id}/{key}"
                )
            self.reference_duplicates += 1
            return
        by_key[key] = record
        if record.is_final:
            self.final_references += 1

    def reference_sample(
        self,
        record: object,
        execution: object,
        actual_started_sim_ns: int | None,
    ) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...], int]:
        del actual_started_sim_ns
        if not isinstance(record, FollowerReference):
            raise _record_error("follower sample record has the wrong type")
        if execution.reference_kind != "CHUNK":  # type: ignore[attr-defined]
            raise _record_error("follower applied sample is not a chunk reference")
        application = execution.reference_application_sim_time_ns  # type: ignore[attr-defined]
        try:
            raw = record.sample_at(application)
            applied = record.applied_sample_at(application, self._profile())
        except ContractError as error:
            raise _record_error(f"applied follower sample failed: {error}") from error
        corrections = tuple(
            sampled - actual
            for sampled, actual in zip(
                applied.sampled_positions, applied.applied_positions, strict=True
            )
        )
        if not _vectors_close(corrections, applied.corrections, 1e-12):
            raise _record_error("applied follower correction differs")
        if not _vectors_close(raw.positions, applied.sampled_positions, 1e-12):
            raise _record_error("raw coefficient sample differs from the applied record")
        if not _vectors_close(raw.velocities, applied.sampled_velocities, 1e-12):
            raise _record_error("raw coefficient velocity differs")
        if not _vectors_close(raw.accelerations, applied.sampled_accelerations, 1e-9):
            raise _record_error("raw coefficient acceleration differs")
        self.applied_chunks.add((record.command_id, record.chunk_seq))
        return (
            applied.applied_positions,
            applied.sampled_velocities,
            applied.sampled_accelerations,
            record.start_sim_time_ns,
        )

    def finish(self) -> dict:
        result = super().finish()
        policy = self.policy
        configured_chunks = int(policy["max_chunks"])
        chunk_size = int(policy["chunk_size"])
        period_ns = int(policy["action_period_ns"])
        raw_points = 0
        for command_id, by_key in self.references.items():
            keys = sorted(by_key)
            if keys != list(range(len(keys))):
                raise _record_error("follower chunk sequence has a gap")
            for index, key in enumerate(keys):
                record = by_key[key]
                if record.chunk_seq != key + 1:
                    raise _record_error("follower chunk sequence differs from its raw anchor")
                targets = record.request.trajectory.targets
                if len(targets) != chunk_size:
                    raise _record_error(
                        f"raw follower source is not {chunk_size} points: {command_id}/{key}"
                    )
                for target_index, target in enumerate(targets):
                    if target.time_ns != (target_index + 1) * period_ns:
                        raise _record_error("raw follower source is not on the 50 ms grid")
                    if len(target.positions) != len(self.joint_names):
                        raise _record_error("raw follower source is not 16-wide")
                    if not all(math.isfinite(value) for value in target.positions):
                        raise _record_error("raw follower source has a non-finite position")
                if record.window.source_targets != targets:
                    raise _record_error("raw follower window does not retain every source point")
                expected_final = index == len(keys) - 1
                if record.is_final and not expected_final:
                    raise _record_error("a follower chunk other than the last is final")
                if expected_final and not record.is_final:
                    # A legal abort can leave a non-final last accepted block.
                    if algorithm_completed(result):
                        raise _record_error("completed follower stream lacks a final block")
                raw_points += len(targets)
        policy_chunks = sum(len(value) for value in self.references.values())
        require_completed_chunks(
            completed=algorithm_completed(result),
            policy_chunks=policy_chunks,
            configured_chunks=configured_chunks,
            strict=self.termination_mode == "finite_chunks",
        )
        if (
            algorithm_completed(result)
            and self.termination_mode == "finite_chunks"
            and not self.final_references
        ):
            raise _record_error("completed follower references contain no final block")
        applied = 0
        for command_id, by_key in self.references.items():
            for key in by_key:
                if (command_id, key + 1) in self.applied_chunks:
                    applied += 1
        if (
            self.termination_mode == "finite_chunks"
            and algorithm_completed(result)
            and applied != sum(len(value) for value in self.references.values())
        ):
            raise _record_error("completed follower stream has an unapplied accepted reference")
        result.update(
            {
                "stream_format": "v1",
                "policy_format": "continuous-bounded-follower",
                "configured_chunks": configured_chunks,
                "policy_chunks": policy_chunks,
                "raw_prediction_points": raw_points,
                "applied_chunks": applied,
                "final_chunks": self.final_references,
                "chunk_size": chunk_size,
                "action_period_ns": period_ns,
                "configured_target_count": self.limits.target_count,
                "action_source": self.action_source,
                "termination_mode": self.termination_mode,
                "test_only": bool(policy.get("expected_test_only", False)),
            }
        )
        return result


def algorithm_completed(result: dict) -> bool:
    """A completed command is the only case that must explain every reference."""
    return bool(result.get("completed_commands")) and not bool(
        result.get("commands_without_terminal")
    )


def audit_follower_recording(
    directory: Path,
    episode_id: str,
    configuration: dict,
) -> dict:
    """Validate one MCAP follower recording and return compact policy counters."""
    return _run_audit(_FollowerAudit(episode_id, configuration), directory)


__all__ = ["audit_follower_recording"]
