"""Streaming MCAP audit for a continuous bounded-follower policy episode.

The bounded reader keeps only sequence/stamp maps, per-command event and
feedback summaries and per-camera references.  Camera payloads are validated
and discarded, never collected into an episode-sized image cache.  This module
is the only recording auditor: the legacy strict-motion audit was deleted with
the single-shot control path.
"""

from __future__ import annotations

from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from robot_contracts.execution_feedback import ControlEvent
from robot_contracts.models import ContractError
from robot_contracts.trajectory import FollowerReference
from rosidl_runtime_py.utilities import get_message

from dual_arm_runtime.audit_common import (
    _TERMINAL_EVENT_PHASES,
    record_error,
    vectors_close,
)
from dual_arm_runtime.audit_stream import StreamAudit


class _Audit(StreamAudit):
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

    def _check_observations(self) -> set[int]:
        previous = 0
        for observation_id in self.observation_ids:
            if observation_id != previous + 1:
                raise record_error("observation IDs are not contiguous from one")
            previous = observation_id
        observation_set = set(self.observation_ids)
        for _observation_id, state_seq, names, sequences, reference_time_ns in self.indices:
            state_stamp = self.states.get(state_seq)
            if state_stamp is None:
                raise record_error("observation references a missing state")
            if reference_time_ns != state_stamp:
                raise record_error("observation reference time differs from its exact capture")
            for camera, sequence in zip(names, sequences, strict=True):
                frame_stamp = self.frames.get((camera, sequence))
                if frame_stamp is None:
                    raise record_error("observation references a missing frame")
                if frame_stamp != state_stamp:
                    raise record_error("exact capture stamps differ")
                if sequence != state_seq:
                    raise record_error("exact capture state/frame sequences differ")
        return observation_set

    def _check_commands(self, observation_set: set[int]) -> None:
        previous_sequence = 0
        for command_id in self.command_order:
            request = self.commands[command_id]
            command = request.trajectory
            if command.command_seq <= previous_sequence:
                raise record_error("stream command sequences are not increasing")
            previous_sequence = command.command_seq
            if command.observation_id not in observation_set:
                raise record_error("stream command references a missing observation")
            if command.joint_names != self.joint_names:
                raise record_error("stream command joint order differs")
            if command.context.robot_profile_hash != self.configuration["profile_hash"]:
                raise record_error("stream command profile differs")
        for request in self.chunk_requests.values():
            trajectory = request.trajectory
            if trajectory.observation_id not in observation_set:
                raise record_error("chunk request references a missing observation")
            if trajectory.joint_names != self.joint_names:
                raise record_error("chunk request joint order differs")
        for command_id, by_key in self.references.items():
            for key, record in by_key.items():
                if record.request.trajectory.observation_id not in observation_set:
                    raise record_error(
                        f"follower reference {command_id}/{key} references a missing observation"
                    )

    def _result(self) -> dict:
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

    def finish(self) -> dict:
        if not self.info_seen:
            raise record_error("no BackendInfo was recorded for the episode")
        observation_set = self._check_observations()
        self._check_commands(observation_set)
        self._check_event_sequences()
        missing_terminals = [
            command_id for command_id in self.references if command_id not in self.terminals
        ]
        if missing_terminals:
            raise record_error(
                "accepted command has no terminal feedback: " + missing_terminals[0]
            )
        self._check_events()
        self._check_feedback_references()
        return self._result()

    def _check_event_sequences(self) -> None:
        sequences = self.event_sequences
        for left, right in zip(sequences, sequences[1:], strict=False):
            if right != left + 1:
                raise record_error("control event sequence gap or duplicate")
        if sequences and sequences[0] < 1:
            raise record_error("control event sequence must start positive")

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
            raise record_error(f"accepted reference lacks overall control events: {command_id}")
        phases = [event.phase for event in overall]
        if phases[0] != "ACCEPTED" or phases.count("ACCEPTED") != 1:
            raise record_error(f"ACCEPTED is not unique and first: {command_id}")
        terminal_position = next(
            (index for index, phase in enumerate(phases) if phase in _TERMINAL_EVENT_PHASES),
            None,
        )
        if terminal_position is None:
            raise record_error(f"accepted reference lacks overall terminal event: {command_id}")
        if terminal_position != len(phases) - 1:
            raise record_error(f"overall events continue after terminal: {command_id}")
        if "STARTED" in phases and phases.index("STARTED") != 1:
            raise record_error(f"overall STARTED is out of order: {command_id}")
        if overall[0].sim_time_ns != reference.accepted_sim_time_ns:
            raise record_error("overall ACCEPTED time differs from the accepted reference")
        terminal = self.terminals[command_id]
        terminal_event = overall[-1]
        if terminal_event.phase != terminal.phase:
            raise record_error("overall terminal event phase differs from feedback")
        if terminal.has_started:
            if "STARTED" not in phases:
                raise record_error("feedback started without an overall STARTED event")
            if overall[1].sim_time_ns != terminal.started_sim_time_ns:
                raise record_error("overall STARTED time differs from feedback")
        elif "STARTED" in phases:
            raise record_error("overall STARTED event without started feedback")
        if terminal_event.sim_time_ns != terminal.finished_sim_time_ns:
            raise record_error("overall terminal event time differs from feedback")

    def _check_chunk_events(
        self, command_id: str, by_key: dict[int, FollowerReference], history: list[object]
    ) -> None:
        chunk_events = [event for event in history if getattr(event, "is_chunk", False)]
        keys = sorted(by_key)
        if keys != list(range(len(keys))):
            raise record_error("chunk reference sequence has a gap")
        for key in keys:
            chunk_seq = key + 1
            events = sorted(
                (event for event in chunk_events if event.chunk_seq == chunk_seq),
                key=lambda item: item.event_seq,
            )
            if not events:
                raise record_error(
                    f"chunk reference lacks control events: {command_id}/{chunk_seq}"
                )
            phases = [event.phase for event in events]
            if phases[0] != "ACCEPTED" or phases.count("ACCEPTED") != 1:
                raise record_error(f"chunk ACCEPTED is not unique and first: {chunk_seq}")
            if phases[-1] not in {"RETIRED", "CANCELED", "REJECTED"}:
                raise record_error(f"chunk has no terminal chunk event: {chunk_seq}")
            if len(phases) > 2 and phases[1] != "STARTED":
                raise record_error(f"chunk STARTED is out of order: {chunk_seq}")
            if len(phases) > 3:
                raise record_error(f"chunk events continue after terminal: {chunk_seq}")
            if events[0].sim_time_ns != by_key[key].accepted_sim_time_ns:
                raise record_error("chunk ACCEPTED time differs from its reference")
            if self.terminals[command_id].phase == "COMPLETED" and phases[-1] != "RETIRED":
                raise record_error("completed stream has an unretired chunk")

    def _check_events(self) -> None:
        for command_id, by_key in self.references.items():
            history = self.events.get(command_id, [])
            if not history:
                raise record_error(f"accepted reference lacks control events: {command_id}")
            first = min(by_key.values(), key=lambda record: record.accepted_sim_time_ns)
            self._check_overall_events(command_id, first, history)
            self._check_chunk_events(command_id, by_key, history)
        for command_id in self.commands:
            if command_id in self.references:
                continue
            phases = {event.phase for event in self.events.get(command_id, ())}
            if phases & {"ACCEPTED", "STARTED", "COMPLETED", "RETIRED"}:
                raise record_error("control event claims acceptance without a recorded reference")

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
                    raise record_error("trajectory feedback has no actual started time")
                if execution.chunk_seq < 1:
                    raise record_error("chunk feedback has no chunk sequence")
                record = by_key.get(execution.chunk_seq - 1)
                if record is None:
                    raise record_error("chunk feedback lacks an accepted reference")
                try:
                    positions, velocities, accelerations, origin = self.reference_sample(
                        record, execution, None
                    )
                except ContractError as error:
                    raise record_error(f"reference reconstruction failed: {error}") from error
                if not vectors_close(positions, execution.reference_positions, 1e-12):
                    raise record_error("feedback reference positions differ from the plan")
                if not vectors_close(velocities, execution.reference_velocities, 1e-12):
                    raise record_error("feedback reference velocities differ from the plan")
                if not vectors_close(
                    accelerations, execution.reference_accelerations, 1e-9
                ):
                    raise record_error("feedback reference accelerations differ from the plan")
                expected_elapsed = execution.reference_application_sim_time_ns - origin
                if execution.reference_elapsed_ns != expected_elapsed:
                    raise record_error("feedback elapsed is not the actual application time")


def _run_audit(audit: _Audit, directory: Path) -> dict:
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=str(directory), storage_id="mcap"),
            rosbag2_py.ConverterOptions("", ""),
        )
    except Exception as error:
        raise record_error(f"cannot open recording: {error}") from error
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
        raise record_error("recording is missing topics: " + ", ".join(sorted(missing)))
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
            raise record_error(f"{topic}: {type(error).__name__}: {error}") from error
    return audit.finish()


def audit_follower_recording(
    directory: Path,
    episode_id: str,
    configuration: dict,
) -> dict:
    """Validate one MCAP follower recording and return compact policy counters."""
    from dual_arm_runtime.follower_audit import _FollowerAudit

    return _run_audit(_FollowerAudit(episode_id, configuration), directory)


__all__ = ["audit_follower_recording"]
