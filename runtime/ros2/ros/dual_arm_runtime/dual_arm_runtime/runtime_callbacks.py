"""ROS subscription callbacks and stream-identity validation for RuntimeNode."""

from __future__ import annotations

import hashlib
import time

from robot_contracts.models import ContractError
from robot_ros_support.conversions import context_from_message, time_ns
from robot_ros_support.follower_conversions import follower_reference_from_message
from robot_ros_support.stream_conversions import (
    control_event_from_message,
    execution_feedback_from_message,
)

from dual_arm_runtime.runtime_identity import execution_identity

_TERMINAL = frozenset({"COMPLETED", "CANCELED", "FAILED"})
_MAX_CONTROL_EVENTS = 4096


class RuntimeCallbacks:
    """Callbacks keep RuntimeNode itself focused on the public IO surface."""

    def __init__(self, node: object) -> None:
        self.node = node

    def _context(self):
        return self.node._episode.context

    def on_info(self, message: object) -> None:
        node = self.node
        context = self._context()
        incoming = context_from_message(message.context)
        if context is not None and incoming.generation != context.generation:
            node._episode.report_control_fault("STALE_CONTEXT: backend restarted")
        with node._condition:
            node._info = message
            node._condition.notify_all()
        if context is not None and incoming == context:
            node._episode.record(node.info_topic, message, 0)

    def on_state(self, message: object) -> None:
        node = self.node
        context = self._context()
        if context is None or context_from_message(message.context) != context:
            node._rejected += 1
            return
        names = list(message.joints.name)
        if names != list(node.config.profile.joint_names):
            node._episode.report_control_fault("STATE_JOINTS_DIFFER")
            return
        if len(message.joints.position) != len(names):
            node._episode.report_control_fault("STATE_POSITION_COUNT_DIFFERS")
            return
        stamp_ns = time_ns(message.stamp)
        if time_ns(message.joints.header.stamp) != stamp_ns:
            node._episode.report_control_fault("STATE_STAMP_MISMATCH")
            return
        node._episode.record(f"{node._root}/state", message, stamp_ns)
        accepted = node._buffer.add(
            "state",
            context,
            int(message.state_seq),
            stamp_ns,
            message,
            wall_arrival_ns=time.monotonic_ns(),
            fingerprint=(stamp_ns, tuple(message.joints.position), tuple(message.joints.velocity)),
        )
        if not accepted and node._buffer.conflict_reason:
            node._episode.report_control_fault(
                f"OBSERVATION_CONFLICT: {node._buffer.conflict_reason}"
            )
        node.notify()

    def validate_frame(self, camera_name: str, message: object) -> None:
        node = self.node
        camera = node._cameras[camera_name]
        image = message.image
        info = message.camera_info
        stamp_ns = time_ns(image.header.stamp)
        if (image.width, image.height) != (camera.width, camera.height):
            raise ContractError("SCHEMA_MISMATCH", f"{camera_name}: image dimensions")
        if image.encoding != "rgb8" or image.step != camera.width * 3:
            raise ContractError("SCHEMA_MISMATCH", f"{camera_name}: RGB encoding/step")
        if len(image.data) != camera.width * camera.height * 3:
            raise ContractError("SCHEMA_MISMATCH", f"{camera_name}: RGB payload size")
        if image.header.frame_id != camera.frame_id:
            raise ContractError("SCHEMA_MISMATCH", f"{camera_name}: image frame_id")
        if time_ns(info.header.stamp) != stamp_ns or info.header.frame_id != camera.frame_id:
            raise ContractError("SCHEMA_MISMATCH", f"{camera_name}: CameraInfo header")
        if (info.width, info.height) != (camera.width, camera.height):
            raise ContractError("SCHEMA_MISMATCH", f"{camera_name}: CameraInfo dimensions")
        if message.calibration_digest != camera.calibration_digest:
            raise ContractError("SCHEMA_MISMATCH", f"{camera_name}: calibration digest")

    def on_frame(self, camera_name: str, message: object) -> None:
        node = self.node
        context = self._context()
        if context is None or context_from_message(message.context) != context:
            node._rejected += 1
            return
        try:
            self.validate_frame(camera_name, message)
        except ContractError as error:
            node._episode.report_control_fault(str(error))
            return
        stamp_ns = time_ns(message.image.header.stamp)
        node._episode.record(node._camera_topics[camera_name], message, stamp_ns)
        digest = hashlib.sha256(bytes(message.image.data)).digest()
        accepted = node._buffer.add(
            camera_name,
            context,
            int(message.frame_seq),
            stamp_ns,
            message,
            wall_arrival_ns=time.monotonic_ns(),
            fingerprint=(stamp_ns, digest),
        )
        if not accepted and node._buffer.conflict_reason:
            node._episode.report_control_fault(
                f"OBSERVATION_CONFLICT: {node._buffer.conflict_reason}"
            )
        node.notify()

    def on_event(self, message: object) -> None:
        node = self.node
        context = self._context()
        if context is None or context_from_message(message.context) != context:
            node._rejected += 1
            return
        try:
            event = control_event_from_message(message)
            if event.is_chunk:
                request = node._expected_chunks.get((event.command_id, event.chunk_seq))
            else:
                request = node._expected_requests.get(event.command_id)
            if request is None:
                raise ContractError(
                    "INVALID_CONTROL_EVENT", f"event for unknown command/chunk {event.command_id}"
                )
            trajectory = request.trajectory
            if (
                event.command_seq != trajectory.command_seq
                or event.observation_id != trajectory.observation_id
            ):
                raise ContractError("INVALID_CONTROL_EVENT", "event command identity differs")
        except ContractError as error:
            node._episode.report_control_fault(str(error))
            return
        signature = (
            event.context,
            event.event_seq,
            event.command_id,
            event.is_chunk,
            event.chunk_seq,
            event.phase,
            event.reason,
            event.backend_time_ns,
        )
        with node._condition:
            previous = node._events.get(event.event_seq)
            if previous == signature:
                node._event_duplicates += 1
                return
            if previous is not None:
                node._episode.report_control_fault(
                    f"CONTROL_EVENT_CONFLICT: sequence {event.event_seq}"
                )
                return
            if len(node._events) >= _MAX_CONTROL_EVENTS:
                node._episode.report_control_fault("CONTROL_EVENT_CACHE_FULL")
                return
            node._events[event.event_seq] = signature
        node._episode.record(f"{node._root}/control_events", message, event.backend_time_ns)
        with node._condition:
            key = (event.command_id, event.chunk_seq)
            if event.is_chunk:
                if event.phase == "ACCEPTED":
                    node._accepted_chunk_events.add(key)
                if event.phase in {"RETIRED", "CANCELED", "REJECTED"}:
                    node._terminal_chunk_events.add(key)
            elif event.phase in _TERMINAL:
                node._overall_terminal_events.add(event.command_id)
            node._condition.notify_all()

    def on_feedback(self, message: object) -> None:
        node = self.node
        context = self._context()
        if context is None or context_from_message(message.context) != context:
            node._rejected += 1
            return
        try:
            execution = execution_feedback_from_message(message)
            request = node._expected_requests.get(execution.command_id)
            if (
                execution.chunk_seq > 0
                and (execution.command_id, execution.chunk_seq) not in node._expected_chunks
            ):
                raise ContractError(
                    "INVALID_EXECUTION_FEEDBACK", "feedback names an unknown applied chunk"
                )
            if request is None:
                raise ContractError(
                    "INVALID_EXECUTION_FEEDBACK",
                    f"feedback for unknown command/chunk {execution.command_id}",
                )
            trajectory = request.trajectory
            if (
                execution.command_seq != trajectory.command_seq
                or execution.observation_id != trajectory.observation_id
                or execution.joint_names != trajectory.joint_names
            ):
                raise ContractError("INVALID_EXECUTION_FEEDBACK", "feedback identity differs")
        except ContractError as error:
            node._episode.report_control_fault(str(error))
            return
        try:
            node.record_execution(execution)
        except ContractError as error:
            node._episode.report_control_fault(str(error))

    def on_follower_reference(self, message: object) -> None:
        node = self.node
        context = self._context()
        try:
            reference = follower_reference_from_message(message)
            if context is None or reference.context != context:
                node._rejected += 1
                return
            if node._follower_limits is None:
                raise ContractError("FOLLOWER_DISABLED", "no follower limits configured")
            request = node._expected_chunks.get((reference.command_id, reference.chunk_seq))
            if request is None or reference.request != request:
                raise ContractError(
                    "REFERENCE_REQUEST_MISMATCH", "follower reference request differs"
                )
            from robot_trajectory.follower_reference import validate as validate_follower_reference

            validate_follower_reference(reference, node.config.profile, node._follower_limits)
        except Exception as error:
            if isinstance(error, ContractError) and error.reason == "EPISODE_STOPPED":
                return
            node._episode.report_control_fault(
                str(error)
                if isinstance(error, ContractError)
                else f"INVALID_FOLLOWER_REFERENCE: {error}"
            )
            return
        node.record_follower_reference(reference)

    def on_task_state(self, message: object) -> None:
        node = self.node
        context = self._context()
        if context is None or context_from_message(message.context) != context:
            node._rejected += 1
            return
        try:
            node.record_task_state(message)
        except ContractError as error:
            node._episode.report_control_fault(str(error))


__all__ = ["RuntimeCallbacks", "execution_identity"]
