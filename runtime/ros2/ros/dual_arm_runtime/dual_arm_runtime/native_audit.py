"""Streaming audit of one native v3 MCAP recording.

The reader keeps only bounded metadata: sequence/stamp maps, per-command event
and feedback summaries and per-camera references.  Camera payloads are validated
and discarded, never collected into an episode-sized image cache.
"""

from __future__ import annotations

import hashlib
import math
from collections import Counter
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from robot_contracts.models import ContractError, RunContext
from robot_contracts.native import (
    NATIVE_MOTION,
    AcceptedReference,
    NativeControlEvent,
    NativeExecution,
    NativeRequest,
    sample_accepted_reference,
    validate_accepted_reference,
)
from robot_contracts.profiles import normalize_cameras
from robot_ros_support.conversions import context_from_message, time_ns
from robot_ros_support.native_conversions import (
    accepted_reference_from_message,
    native_command_from_message,
    native_control_event_from_message,
    native_execution_from_message,
)
from rosidl_runtime_py.utilities import get_message

_TERMINAL = frozenset({"COMPLETED", "CANCELED", "FAILED"})
_MAX_FEEDBACK_PER_COMMAND = 4096
_MAX_METADATA_ENTRIES = 1_000_000
_MAX_COMMANDS = 4096
_MAX_EVENTS = 100_000
_TERMINAL_EVENT_PHASES = frozenset({"COMPLETED", "CANCELED", "FAILED"})


def _record_error(message: str) -> ContractError:
    return ContractError("RECORDING_ERROR", message)


def _close(left: float, right: float, tolerance: float) -> bool:
    return math.isclose(left, right, rel_tol=1e-9, abs_tol=tolerance)


def _vectors_close(left: tuple[float, ...], right: tuple[float, ...], tolerance: float) -> bool:
    return len(left) == len(right) and all(
        _close(a, b, tolerance) for a, b in zip(left, right, strict=True)
    )


class _Audit:
    reference_topic_name = "accepted_references"
    required_capabilities = frozenset(
        {
            "native_motion",
            "exact_capture",
            "simulation_reset",
            "lease",
            "stop",
            "command_query",
            "native_events",
            "accepted_reference",
        }
    )

    def __init__(self, episode_id: str, configuration: dict) -> None:
        self.episode_id = episode_id
        self.configuration = configuration
        recipe = configuration.get("recipe")
        profile = configuration.get("profile")
        if not isinstance(recipe, dict) or not isinstance(profile, dict):
            raise _record_error("native configuration snapshot is incomplete")
        self.root = str(recipe.get("robot_namespace", "")).rstrip("/")
        if not self.root:
            raise _record_error("native configuration has no robot_namespace")
        self.reference_topic = f"{self.root}/{self.reference_topic_name}"
        self.expected_context = configuration.get("run_context")
        if not isinstance(self.expected_context, dict):
            raise _record_error("native recording has no run_context")
        self.profile_document = profile
        try:
            self.cameras = normalize_cameras(profile, "<recorded native profile>")
        except ContractError as error:
            raise _record_error(f"invalid recorded profile cameras: {error}") from error
        self._profile_object = None
        self.camera_names = tuple(camera.name for camera in self.cameras)
        self.camera_specs = {camera.name: camera for camera in self.cameras}
        self.joint_names = tuple(
            str(item["name"]) for item in profile.get("joints", ()) if isinstance(item, dict)
        )
        if not self.joint_names:
            raise _record_error("native profile has no joints")
        observation = recipe.get("observation")
        if not isinstance(observation, dict):
            observation = configuration.get("observation", {})
        if not isinstance(observation, dict):
            raise _record_error("native observation snapshot is incomplete")
        _positive_int(observation.get("max_age_ns"), "max_age_ns")
        _positive_number(observation.get("wall_arrival_timeout_s"), "wall_arrival_timeout_s")
        motions = recipe.get("motions")
        if not isinstance(motions, list):
            raise _record_error("native recipe has no motion list")
        self.motion_count = len(motions)
        control = recipe.get("control_constraints")
        if not isinstance(control, dict):
            raise _record_error("native recipe has no control constraints")
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
        self.commands: dict[str, NativeRequest] = {}
        self.command_order: list[str] = []
        self.chunk_requests: dict[tuple[str, int], NativeRequest] = {}
        self.references: dict[str, dict[int, AcceptedReference]] = {}
        self.reference_duplicates = 0
        self.events: dict[str, list[NativeControlEvent]] = {}
        self.event_sequences: list[int] = []
        self.feedback: dict[str, list[NativeExecution]] = {}
        self.terminals: dict[str, NativeExecution] = {}
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
                raise _record_error(f"mixed native context field: {key}")
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
            raise _record_error("native diagnostic recording is marked test_only")
        if message.context.robot_profile_hash != self.configuration["profile_hash"]:
            raise _record_error("recorded backend profile differs")
        if tuple(message.joint_names) != self.joint_names:
            raise _record_error("recorded backend joint order differs")
        if tuple(message.camera_names) != self.camera_names:
            raise _record_error("recorded backend camera order differs")
        missing = self.required_capabilities - set(message.capabilities)
        if missing:
            raise _record_error("recorded backend lacks native capabilities")

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
            raise _record_error("native state metadata exceeds the audit bound")
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
        from dual_arm_runtime.audit import _validate_camera_frame

        camera_name = topic.split("/")[-2]
        camera = self.camera_specs.get(camera_name)
        if camera is None:
            raise _record_error(f"unknown camera topic {camera_name}")
        _validate_camera_frame(message, camera, 2)
        stamp = time_ns(message.image.header.stamp)
        sequence = int(message.frame_seq)
        if sequence < 1:
            raise _record_error(f"invalid frame sequence for {camera_name}")
        key = (camera_name, sequence)
        if key in self.frames and self.frames[key] != stamp:
            raise _record_error(f"same camera frame sequence has a different stamp: {camera_name}")
        if key not in self.frames and len(self.frames) >= _MAX_METADATA_ENTRIES:
            raise _record_error("native frame metadata exceeds the audit bound")
        self.frames[key] = stamp

    def observation(self, message: object) -> None:
        if not self.context(message):
            return
        names = tuple(message.camera_names)
        sequences = tuple(int(value) for value in message.frame_seqs)
        if names != self.camera_names or len(sequences) != len(names):
            raise _record_error("observation camera names/order differ")
        if len(self.indices) >= _MAX_METADATA_ENTRIES:
            raise _record_error("native observation metadata exceeds the audit bound")
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
        request = native_command_from_message(message)
        if not self.context_value(request.trajectory.context):
            return
        command_id = request.trajectory.command_id
        key = 0 if request.kind == NATIVE_MOTION else request.anchor.predecessor_seq
        identity = (command_id, key)
        previous = self.chunk_requests.get(identity)
        if previous is not None:
            if previous != request:
                raise _record_error(f"same command/chunk ID has different content: {identity}")
            return
        if len(self.chunk_requests) >= _MAX_COMMANDS:
            raise _record_error("native chunk request metadata exceeds the audit bound")
        existing = self.commands.get(command_id)
        if existing is not None:
            if not (
                existing.kind != NATIVE_MOTION
                and request.kind != NATIVE_MOTION
                and existing.trajectory.context == request.trajectory.context
                and existing.trajectory.lease_id == request.trajectory.lease_id
                and existing.trajectory.command_seq == request.trajectory.command_seq
                and existing.trajectory.joint_names == request.trajectory.joint_names
            ):
                raise _record_error(f"same command ID has different identity: {command_id}")
        else:
            self.commands[command_id] = request
            self.command_order.append(command_id)
        self.chunk_requests[identity] = request

    def reference(self, message: object) -> None:
        record = accepted_reference_from_message(message)
        if not self.context_value(record.request.trajectory.context):
            return
        command_id = record.request.trajectory.command_id
        request = self.commands.get(command_id)
        if request is None:
            raise _record_error(f"accepted reference precedes its command: {command_id}")
        if request.kind == NATIVE_MOTION:
            if record.request != request:
                raise _record_error(f"accepted reference request differs: {command_id}")
            key = 0
        else:
            current = record.request
            if (
                current.kind == NATIVE_MOTION
                or current.trajectory.context != request.trajectory.context
                or current.trajectory.lease_id != request.trajectory.lease_id
                or current.trajectory.command_seq != request.trajectory.command_seq
            ):
                raise _record_error(f"accepted chunk request identity differs: {command_id}")
            assert current.anchor is not None
            key = current.anchor.predecessor_seq
        if self.chunk_requests.get((command_id, key)) != record.request:
            raise _record_error(
                f"accepted reference differs from its raw chunk request: {command_id}/{key}"
            )
        validate_accepted_reference(record, self._profile(), self.max_accelerations)
        by_key = self.references.setdefault(command_id, {})
        existing = by_key.get(key)
        if existing is not None:
            if existing.reference_digest != record.reference_digest:
                raise _record_error(
                    f"same command chunk has conflicting accepted references: {command_id}"
                )
            self.reference_duplicates += 1
            return
        by_key[key] = record

    def event(self, message: object) -> None:
        if not self.context(message):
            return
        event = native_control_event_from_message(message)
        command_id = event.command_id
        if command_id not in self.commands:
            raise _record_error(f"native event references an unknown command: {command_id}")
        request = self.commands[command_id]
        if event.is_chunk:
            request = self.chunk_requests.get((command_id, event.chunk_seq - 1))
            if request is None:
                raise _record_error("native chunk event references an unknown request")
        if (
            event.command_seq != request.trajectory.command_seq
            or event.observation_id != request.trajectory.observation_id
        ):
            raise _record_error("native event command identity differs")
        if len(self.event_sequences) >= _MAX_EVENTS:
            raise _record_error("native event metadata exceeds the audit bound")
        self.event_sequences.append(event.event_seq)
        self.events.setdefault(command_id, []).append(event)

    def feedback_message(self, message: object) -> None:
        if not self.context(message):
            return
        execution = native_execution_from_message(message)
        request = self.commands.get(execution.command_id)
        if request is None:
            raise _record_error(
                f"native feedback references an unknown command: {execution.command_id}"
            )
        trajectory = request.trajectory
        if (
            execution.command_seq != trajectory.command_seq
            or execution.observation_id != trajectory.observation_id
            or execution.joint_names != trajectory.joint_names
        ):
            raise _record_error("native feedback identity differs")
        if execution.context.robot_profile_hash != self.configuration["profile_hash"]:
            raise _record_error("native feedback profile differs")
        history = self.feedback.setdefault(execution.command_id, [])
        if len(history) >= _MAX_FEEDBACK_PER_COMMAND:
            raise _record_error("native feedback history exceeds the audit bound")
        history.append(execution)
        if execution.phase in _TERMINAL:
            previous = self.terminals.get(execution.command_id)
            if previous is not None and previous != execution:
                raise _record_error(
                    f"terminal native feedback changed after freeze: {execution.command_id}"
                )
            self.terminals[execution.command_id] = execution
        elif execution.command_id in self.terminals:
            raise _record_error("native feedback continued after a terminal freeze")

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
            raise _record_error("no BackendInfo was recorded for the native episode")
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
                raise _record_error("native command sequences are not increasing")
            previous_sequence = command.command_seq
            if command.observation_id not in observation_set:
                raise _record_error("native command references a missing observation")
            if command.joint_names != self.joint_names:
                raise _record_error("native command joint order differs")
            if command.context.robot_profile_hash != self.configuration["profile_hash"]:
                raise _record_error("native command profile differs")
        for request in self.chunk_requests.values():
            trajectory = request.trajectory
            if trajectory.observation_id not in observation_set:
                raise _record_error("native chunk request references a missing observation")
            if trajectory.joint_names != self.joint_names:
                raise _record_error("native chunk request joint order differs")
        for command_id, by_key in self.references.items():
            for key, record in by_key.items():
                if record.request.trajectory.observation_id not in observation_set:
                    raise _record_error(
                        f"native reference {command_id}/{key} references a missing observation"
                    )
        self._check_event_sequences()
        missing_terminals = [
            command_id for command_id in self.references if command_id not in self.terminals
        ]
        if missing_terminals:
            raise _record_error(
                "accepted native command has no terminal feedback: " + missing_terminals[0]
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
            "accepted_references": sum(len(value) for value in self.references.values()),
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
                raise _record_error("native event sequence gap or duplicate")
        if sequences and sequences[0] < 1:
            raise _record_error("native event sequence must start positive")

    @staticmethod
    def _overall_history(history: list[NativeControlEvent]) -> list[NativeControlEvent]:
        return sorted(
            (event for event in history if not event.is_chunk),
            key=lambda item: item.event_seq,
        )

    def _check_overall_events(
        self, command_id: str, reference: AcceptedReference, history: list[NativeControlEvent]
    ) -> None:
        overall = self._overall_history(history)
        if not overall:
            raise _record_error(f"accepted reference lacks overall native events: {command_id}")
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
        self, command_id: str, by_key: dict[int, AcceptedReference], history: list[object]
    ) -> None:
        chunk_events = [event for event in history if getattr(event, "is_chunk", False)]
        keys = sorted(by_key)
        if keys != list(range(len(keys))):
            raise _record_error("native chunk reference sequence has a gap")
        for key in keys:
            chunk_seq = key + 1
            events = sorted(
                (event for event in chunk_events if event.chunk_seq == chunk_seq),
                key=lambda item: item.event_seq,
            )
            if not events:
                raise _record_error(
                    f"chunk reference lacks native events: {command_id}/{chunk_seq}"
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
                raise _record_error(f"accepted reference lacks native events: {command_id}")
            request = self.commands[command_id]
            first = min(by_key.values(), key=lambda record: record.accepted_sim_time_ns)
            self._check_overall_events(command_id, first, history)
            if request.kind == NATIVE_MOTION:
                if len(by_key) != 1:
                    raise _record_error("MOTION command has multiple accepted references")
                continue
            self._check_chunk_events(command_id, by_key, history)
        for command_id in self.commands:
            if command_id in self.references:
                continue
            phases = {event.phase for event in self.events.get(command_id, ())}
            if phases & {"ACCEPTED", "STARTED", "COMPLETED", "RETIRED"}:
                raise _record_error("native event claims acceptance without a recorded reference")

    def reference_sample(
        self,
        record: object,
        execution: NativeExecution,
        actual_started_sim_ns: int | None,
    ) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...], int]:
        """Return the actual applied reference for one feedback record."""
        sample = sample_accepted_reference(
            record,  # type: ignore[arg-type]
            execution.reference_application_sim_time_ns,
            actual_started_sim_ns=actual_started_sim_ns,
        )
        if execution.reference_kind == "TRAJECTORY":
            origin = execution.started_sim_time_ns
        else:
            anchor = record.request.anchor  # type: ignore[attr-defined]
            assert anchor is not None
            origin = anchor.start_sim_time_ns
        assert origin is not None
        return sample.positions, sample.velocities, sample.accelerations, origin

    def _check_feedback_references(self) -> None:
        for command_id, history in self.feedback.items():
            by_key = self.references.get(command_id)
            if not by_key:
                continue
            request = self.commands[command_id]
            for execution in history:
                if execution.reference_kind not in {"TRAJECTORY", "CHUNK"}:
                    # BRAKE/HOLD are real applied references, but they must never
                    # be reconstructed from the original predicted plan.
                    continue
                if not execution.has_started:
                    raise _record_error("trajectory feedback has no actual started time")
                if request.kind == NATIVE_MOTION:
                    record = by_key.get(0)
                    if record is None:
                        raise _record_error("motion feedback lacks an accepted reference")
                    actual_started = execution.started_sim_time_ns
                else:
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
        raise _record_error(f"cannot open native recording: {error}") from error
    types = {topic.name: get_message(topic.type) for topic in reader.get_all_topics_and_types()}
    audit.topics = set(types)
    required = {
        f"{audit.root}/info",
        f"{audit.root}/state",
        f"{audit.root}/native_events",
        f"{audit.root}/native_feedback",
        audit.reference_topic,
        "/runtime/native/commands",
        "/runtime/observation_index",
    } | {f"{audit.root}/cameras/{name}/frame" for name in audit.camera_names}
    missing = required - audit.topics
    if missing:
        raise _record_error("native recording is missing topics: " + ", ".join(sorted(missing)))
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
            elif topic == "/runtime/native/commands":
                audit.command(message)
            elif topic == audit.reference_topic:
                audit.reference(message)
            elif topic == f"{audit.root}/native_events":
                audit.event(message)
            elif topic == f"{audit.root}/native_feedback":
                audit.feedback_message(message)
            elif (
                audit.reference_topic_name == "follower_references"
                and topic == f"{audit.root}/accepted_references"
            ):
                raise _record_error(
                    "v4 follower recording must not mix strict AcceptedReference messages"
                )
        except ContractError:
            raise
        except Exception as error:
            raise _record_error(f"{topic}: {type(error).__name__}: {error}") from error
    return audit.finish()


def audit_native_recording(
    directory: Path,
    episode_id: str,
    configuration: dict,
) -> dict:
    """Validate one native v3 recording directory and return compact counters."""
    return _run_audit(_Audit(episode_id, configuration), directory)
