"""Legacy fixed-array feedback CDR decoding, without the old SDK."""

from __future__ import annotations

import struct

import pytest
from dual_arm_runtime.offline.feedback_definition import (
    EXECUTION_FEEDBACK_TYPE,
    LEGACY_FEEDBACK_DEFINITION,
)
from dual_arm_runtime.offline.legacy_feedback import (
    LegacyFeedbackError,
    _verify_legacy_definition,
    decode_legacy_execution_feedback,
)


class _Writer:
    """Little-endian CDR writer mirroring the documented legacy layout."""

    def __init__(self) -> None:
        self.data = bytearray(b"\x00\x01\x00\x00")

    def _align(self, alignment: int) -> None:
        self.data.extend(b"\x00" * (-(len(self.data) - 4) % alignment))

    def u8(self, value: int) -> None:
        self.data.append(value)

    def u32(self, value: int) -> None:
        self._align(4)
        self.data.extend(struct.pack("<I", value))

    def u64(self, value: int) -> None:
        self._align(8)
        self.data.extend(struct.pack("<Q", value))

    def i64(self, value: int) -> None:
        self._align(8)
        self.data.extend(struct.pack("<q", value))

    def f64(self, value: float) -> None:
        self._align(8)
        self.data.extend(struct.pack("<d", value))

    def string(self, value: str) -> None:
        raw = value.encode("utf-8") + b"\x00"
        self.u32(len(raw))
        self.data.extend(raw)

    def sequence(self, values, write) -> None:
        self.u32(len(values))
        for value in values:
            write(value)


def _write_context(writer: _Writer) -> None:
    writer.string("panda-stack")
    writer.string("profile-digest")
    writer.string("backend-instance")
    writer.string("mujoco-sim")
    writer.u64(0)
    writer.string("episode-1")
    writer.u64(1)


def _write_finger(writer: _Writer, side: str, index: int) -> None:
    writer.string(side)
    writer.u8(index)
    writer.string(f"{side}_finger{index}")
    writer.f64(0.0)
    writer.sequence(("source",), writer.string)
    writer.sequence((1.5,), writer.f64)
    writer.sequence((2,), writer.u32)


def _write_gripper(writer: _Writer, side: str) -> None:
    writer.string(side)
    writer.string("CONTACT")
    writer.u8(0)
    writer.u8(1)
    writer.string("source")
    writer.u8(0)
    for value in (0.0, 0.08, 0.0, 0.03985, 0.0, -0.03985):
        writer.f64(value)
    _write_finger(writer, side, 1)
    _write_finger(writer, side, 2)


def legacy_payload() -> bytes:
    writer = _Writer()
    _write_context(writer)
    writer.string("command-1")
    writer.u64(1)
    writer.u64(2)
    writer.string("STARTED")
    writer.string("")
    writer.string("")
    writer.u64(1)
    writer.string("stream-1")
    writer.u64(0)
    writer.i64(0)
    writer.i64(10)
    writer.u8(1)
    writer.i64(20)
    writer.i64(20)
    writer.u8(0)
    writer.i64(0)
    writer.i64(0)
    writer.string("RUNNING")
    for value in (30, 32, 10, 0):
        writer.i64(value)
    writer.sequence(("j1", "j2"), writer.string)
    for values in ((0.2, -0.2), (0.1, -0.1), (0.0, 0.0), (0.1, -0.2),
                   (0.05, -0.1), (0.1, 0.0), (0.05, 0.0)):
        writer.sequence(values, writer.f64)
    writer.string("TRAJECTORY")
    writer.string("FOLLOWING")
    writer.i64(0)
    _write_gripper(writer, "left")
    _write_gripper(writer, "right")
    return bytes(writer.data)


def test_synthetic_legacy_payload_decodes_and_validates():
    feedback = decode_legacy_execution_feedback(legacy_payload())
    assert feedback.command_id == "command-1"
    assert feedback.context.episode_id == "episode-1"
    assert tuple(gripper.side for gripper in feedback.grippers) == ("left", "right")
    right = feedback.grippers[1]
    assert right.mode == "CONTACT"
    assert right.contact_object == "source"
    assert tuple(finger.finger_index for finger in right.fingers) == (1, 2)
    assert right.fingers[0].object_names == ("source",)
    assert right.fingers[0].normal_forces == (1.5,)


def test_trailing_bytes_and_bad_modes_are_rejected():
    payload = legacy_payload()
    with pytest.raises(LegacyFeedbackError, match="trailing bytes"):
        decode_legacy_execution_feedback(payload + b"\x00\x00\x00\x00")
    broken = bytearray(payload)
    # Break the first finger index from 1 to 0 so the DTO rejects it.
    position = broken.find(b"left\x00\x01")
    assert position >= 0
    broken[position + len(b"left\x00")] = 0
    with pytest.raises(LegacyFeedbackError, match="not a valid contract DTO"):
        decode_legacy_execution_feedback(bytes(broken))


def test_complete_legacy_definition_is_accepted_with_comments_and_separators():
    recorded = LEGACY_FEEDBACK_DEFINITION.replace(
        "MSG:", "=" * 80 + "\nMSG:"
    ).replace(
        "robot_api_msgs/RunContext context",
        "# Frozen overall execution feedback.\nrobot_api_msgs/RunContext context",
    )
    digest = _verify_legacy_definition({EXECUTION_FEEDBACK_TYPE: recorded}, EXECUTION_FEEDBACK_TYPE)
    assert len(digest) == 64


def test_definition_with_only_the_two_array_markers_is_rejected():
    recorded = (
        "robot_api_msgs/GripperFeedback[2] grippers\n"
        "robot_api_msgs/FingerFeedback[2] fingers\n"
    )
    with pytest.raises(LegacyFeedbackError, match="complete fixed"):
        _verify_legacy_definition({EXECUTION_FEEDBACK_TYPE: recorded}, EXECUTION_FEEDBACK_TYPE)


@pytest.mark.parametrize(
    ("original", "changed"),
    [
        ("uint64 observation_id", "int64 observation_id"),
        (
            "uint64 command_seq\nuint64 observation_id",
            "uint64 observation_id\nuint64 command_seq",
        ),
        ("robot_api_msgs/GripperFeedback[2] grippers",
         "robot_api_msgs/GripperFeedback[<=8] grippers"),
        ("robot_api_msgs/FingerFeedback[2] fingers",
         "robot_api_msgs/FingerFeedback[<=8] fingers"),
    ],
)
def test_one_changed_field_or_bound_invalidates_the_legacy_layout(original, changed):
    recorded = LEGACY_FEEDBACK_DEFINITION.replace(original, changed, 1)
    with pytest.raises(LegacyFeedbackError, match="complete fixed"):
        _verify_legacy_definition({EXECUTION_FEEDBACK_TYPE: recorded}, EXECUTION_FEEDBACK_TYPE)
