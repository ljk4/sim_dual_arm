"""Explicit reader for pre-bounded-gripper ExecutionFeedback recordings.

``robot_api_msgs`` 0.1.0 declared fixed ``GripperFeedback[2]`` and
``FingerFeedback[2]`` arrays.  Version 0.2.0 introduced bounded
``[<=8]`` arrays, which CDR serializes with a length prefix, so the current
generated type cannot decode the old bytes.  This module accepts a recording
only when its complete stored definition matches the known legacy layout, then
parses the legacy CDR directly and validates every decoded sample with the
current shared DTO.  It never imports the old SDK and never writes to the
recording.

Run inside the runtime container (ROS sourced) with::

    python -m dual_arm_runtime.offline.legacy_feedback <recording-dir> \
        [--topic /robot/execution_feedback] [--limit 1]
"""

from __future__ import annotations

import argparse
import hashlib
import json
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from robot_contracts.execution_feedback import (
    ExecutionFeedback,
    FingerFeedback,
    GripperFeedback,
)
from robot_contracts.models import ContractError, RunContext

from dual_arm_runtime.offline.feedback_definition import (
    EXECUTION_FEEDBACK_TYPE,
    is_legacy_feedback_definition,
)

__all__ = [
    "EXECUTION_FEEDBACK_TYPE",
    "LegacyFeedbackError",
    "decode_legacy_execution_feedback",
    "read_legacy_feedback",
]


class LegacyFeedbackError(ValueError):
    """The recording is not the documented legacy feedback layout."""


@dataclass
class _Cdr:
    """Minimal little-endian CDR reader for the legacy message layout."""

    data: bytes
    origin: int = 4
    offset: int = 4

    def __post_init__(self) -> None:
        if len(self.data) < 4:
            raise LegacyFeedbackError("CDR payload is shorter than its encapsulation")
        if self.data[:2] != b"\x00\x01":
            raise LegacyFeedbackError("only little-endian CDR is supported")

    def _align(self, alignment: int) -> None:
        self.offset += (-(self.offset - self.origin)) % alignment

    def _unpack(self, code: str, size: int, alignment: int) -> Any:
        self._align(alignment)
        if self.offset + size > len(self.data):
            raise LegacyFeedbackError("CDR payload ended before the message")
        value = struct.unpack_from(code, self.data, self.offset)[0]
        self.offset += size
        return value

    def u8(self) -> int:
        return self._unpack("<B", 1, 1)

    def u32(self) -> int:
        return self._unpack("<I", 4, 4)

    def u64(self) -> int:
        return self._unpack("<Q", 8, 8)

    def i64(self) -> int:
        return self._unpack("<q", 8, 8)

    def f64(self) -> float:
        return self._unpack("<d", 8, 8)

    def string(self) -> str:
        size = self.u32()
        if size < 1 or self.offset + size > len(self.data):
            raise LegacyFeedbackError("CDR string length is invalid")
        raw = self.data[self.offset:self.offset + size]
        self.offset += size
        if raw[-1] != 0:
            raise LegacyFeedbackError("CDR string is not NUL-terminated")
        try:
            return raw[:-1].decode("utf-8")
        except UnicodeDecodeError as error:
            raise LegacyFeedbackError("CDR string is not UTF-8") from error

    def sequence_count(self) -> int:
        return self.u32()


def _read_run_context(reader: _Cdr) -> RunContext:
    return RunContext(
        robot_id=reader.string(),
        robot_profile_hash=reader.string(),
        backend_instance_id=reader.string(),
        time_domain_id=reader.string(),
        clock_epoch=reader.u64(),
        episode_id=reader.string(),
        control_epoch=reader.u64(),
    )


def _read_finger(reader: _Cdr) -> FingerFeedback:
    return FingerFeedback(
        side=reader.string(),
        finger_index=reader.u8(),
        body_name=reader.string(),
        joint_speed_m_s=reader.f64(),
        object_names=tuple(
            reader.string() for _ in range(reader.sequence_count())
        ),
        normal_forces=tuple(
            reader.f64() for _ in range(reader.sequence_count())
        ),
        contact_counts=tuple(
            reader.u32() for _ in range(reader.sequence_count())
        ),
    )


def _read_gripper(reader: _Cdr) -> GripperFeedback:
    side = reader.string()
    mode = reader.string()
    position_ok = bool(reader.u8())
    contact_ok = bool(reader.u8())
    contact_object = reader.string()
    ambiguous = bool(reader.u8())
    target_position_m = reader.f64()
    initial_reference_m = reader.f64()
    final_reference_m = reader.f64()
    measured_position_m = reader.f64()
    measured_velocity_m_s = reader.f64()
    target_error_m = reader.f64()
    # The legacy IDL stored exactly two inline FingerFeedback records.
    fingers = tuple(_read_finger(reader) for _ in range(2))
    return GripperFeedback(
        side=side,
        mode=mode,
        position_ok=position_ok,
        contact_ok=contact_ok,
        contact_object=contact_object or None,
        ambiguous=ambiguous,
        target_position_m=target_position_m,
        initial_reference_m=initial_reference_m,
        final_reference_m=final_reference_m,
        measured_position_m=measured_position_m,
        measured_velocity_m_s=measured_velocity_m_s,
        target_error_m=target_error_m,
        fingers=fingers,
    )


def decode_legacy_execution_feedback(payload: bytes) -> ExecutionFeedback:
    """Decode one legacy ExecutionFeedback payload and validate it as a DTO."""
    reader = _Cdr(payload)
    try:
        context = _read_run_context(reader)
        command_id = reader.string()
        command_seq = reader.u64()
        observation_id = reader.u64()
        phase = reader.string()
        reason = reader.string()
        stop_reason = reader.string()
        motion_seq = reader.u64()
        stream_id = reader.string()
        chunk_seq = reader.u64()
        accepted_sim_time_ns = reader.i64()
        accepted_wall_time_ns = reader.i64()
        has_started = bool(reader.u8())
        started_sim_time_ns = reader.i64()
        started_wall_time_ns = reader.i64()
        has_finished = bool(reader.u8())
        finished_sim_time_ns = reader.i64()
        finished_wall_time_ns = reader.i64()
        motion_phase = reader.string()
        reference_application_sim_time_ns = reader.i64()
        measurement_sim_time_ns = reader.i64()
        reference_elapsed_ns = reader.i64()
        stable_for_ns = reader.i64()
        joint_names = tuple(
            reader.string() for _ in range(reader.sequence_count())
        )
        reference_positions = tuple(
            reader.f64() for _ in range(reader.sequence_count())
        )
        reference_velocities = tuple(
            reader.f64() for _ in range(reader.sequence_count())
        )
        reference_accelerations = tuple(
            reader.f64() for _ in range(reader.sequence_count())
        )
        measured_positions = tuple(
            reader.f64() for _ in range(reader.sequence_count())
        )
        measured_velocities = tuple(
            reader.f64() for _ in range(reader.sequence_count())
        )
        position_errors = tuple(
            reader.f64() for _ in range(reader.sequence_count())
        )
        velocity_errors = tuple(
            reader.f64() for _ in range(reader.sequence_count())
        )
        reference_kind = reader.string()
        tracking_quality = reader.string()
        tracking_violation_ns = reader.i64()
        # The legacy IDL stored exactly two inline GripperFeedback records.
        grippers = tuple(_read_gripper(reader) for _ in range(2))
        if reader.offset != len(payload):
            raise LegacyFeedbackError(
                f"legacy payload has {len(payload) - reader.offset} trailing bytes"
            )
        return ExecutionFeedback(
            context=context,
            command_id=command_id,
            command_seq=command_seq,
            observation_id=observation_id,
            phase=phase,
            reason=reason,
            stop_reason=stop_reason,
            motion_seq=None if motion_seq == 0 else motion_seq,
            stream_id=stream_id,
            chunk_seq=chunk_seq,
            accepted_sim_time_ns=accepted_sim_time_ns,
            accepted_wall_time_ns=accepted_wall_time_ns,
            has_started=has_started,
            started_sim_time_ns=started_sim_time_ns if has_started else None,
            started_wall_time_ns=started_wall_time_ns if has_started else None,
            has_finished=has_finished,
            finished_sim_time_ns=finished_sim_time_ns if has_finished else None,
            finished_wall_time_ns=finished_wall_time_ns if has_finished else None,
            motion_phase=motion_phase,
            reference_application_sim_time_ns=reference_application_sim_time_ns,
            measurement_sim_time_ns=measurement_sim_time_ns,
            reference_elapsed_ns=reference_elapsed_ns,
            stable_for_ns=stable_for_ns,
            joint_names=joint_names,
            reference_positions=reference_positions,
            reference_velocities=reference_velocities,
            reference_accelerations=reference_accelerations,
            measured_positions=measured_positions,
            measured_velocities=measured_velocities,
            position_errors=position_errors,
            velocity_errors=velocity_errors,
            reference_kind=reference_kind,
            tracking_quality=tracking_quality,
            tracking_violation_ns=tracking_violation_ns,
            grippers=grippers,
        )
    except ContractError as error:
        raise LegacyFeedbackError(f"decoded sample is not a valid contract DTO: {error}") from error


def _verify_legacy_definition(definitions: dict[str, str], type_name: str) -> str:
    text = definitions.get(type_name)
    if text is None:
        raise LegacyFeedbackError(f"bag has no stored definition for {type_name}")
    if not is_legacy_feedback_definition(text):
        raise LegacyFeedbackError(
            "bag definition is not the complete fixed two-gripper/two-finger legacy layout"
        )
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def _storage_uri(path: Path) -> str:
    # rosbag2 storage options accept the directory that holds metadata.yaml.
    return str(path if path.is_dir() else path.parent)


def read_legacy_feedback(
    path: Path, *, topic: str | None = None, limit: int = 1
) -> dict[str, Any]:
    """Read and validate up to ``limit`` legacy feedback samples."""
    if limit < 1:
        raise LegacyFeedbackError("limit must be positive")
    import rosbag2_py

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=_storage_uri(path), storage_id="mcap"),
            rosbag2_py.ConverterOptions("", ""),
        )
    except Exception as error:
        raise LegacyFeedbackError(f"cannot open recording: {error}") from error
    try:
        types = {item.name: item.type for item in reader.get_all_topics_and_types()}
        if topic is None:
            candidates = [
                name for name, type_name in types.items()
                if type_name == EXECUTION_FEEDBACK_TYPE
            ]
            if len(candidates) != 1:
                raise LegacyFeedbackError(
                    "recording must contain exactly one ExecutionFeedback topic; "
                    f"found {candidates}"
                )
            topic = candidates[0]
        elif types.get(topic) != EXECUTION_FEEDBACK_TYPE:
            raise LegacyFeedbackError(
                f"topic {topic!r} is not {EXECUTION_FEEDBACK_TYPE}"
            )
        definitions = {
            item.topic_type: item.encoded_message_definition
            for item in reader.get_all_message_definitions()
        }
        digest = _verify_legacy_definition(definitions, EXECUTION_FEEDBACK_TYPE)
        reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
        samples: list[dict[str, Any]] = []
        while len(samples) < limit and reader.has_next():
            _topic, payload, timestamp_ns = reader.read_next()
            feedback = decode_legacy_execution_feedback(payload)
            samples.append(
                {
                    "timestamp_ns": int(timestamp_ns),
                    "command_id": feedback.command_id,
                    "observation_id": feedback.observation_id,
                    "phase": feedback.phase,
                    "motion_phase": feedback.motion_phase,
                    "joint_count": len(feedback.joint_names),
                    "grippers": [
                        {
                            "side": gripper.side,
                            "mode": gripper.mode,
                            "contact_ok": gripper.contact_ok,
                            "contact_object": gripper.contact_object,
                            "measured_position_m": gripper.measured_position_m,
                            "target_error_m": gripper.target_error_m,
                            "fingers": [
                                {
                                    "finger_index": finger.finger_index,
                                    "body_name": finger.body_name,
                                    "object_names": list(finger.object_names),
                                }
                                for finger in gripper.fingers
                            ],
                        }
                        for gripper in feedback.grippers
                    ],
                }
            )
        if not samples:
            raise LegacyFeedbackError(f"recording has no messages on {topic}")
        return {
            "bag": str(path),
            "topic": topic,
            "legacy_definition_sha256": digest,
            "validated_against": "robot_contracts.ExecutionFeedback",
            "samples": samples,
        }
    finally:
        reader.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path, help="recording directory or .mcap file")
    parser.add_argument("--topic", help="ExecutionFeedback topic (default: unique match)")
    parser.add_argument("--limit", type=int, default=1)
    parser.add_argument("--output", type=Path, help="write JSON here instead of stdout")
    arguments = parser.parse_args(argv)
    try:
        document = read_legacy_feedback(
            arguments.recording, topic=arguments.topic, limit=arguments.limit
        )
    except LegacyFeedbackError as error:
        print(f"legacy feedback read failed: {error}", file=sys.stderr)
        return 2
    text = json.dumps(document, indent=2, allow_nan=False) + "\n"
    if arguments.output is None:
        sys.stdout.write(text)
    else:
        arguments.output.write_text(text, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
