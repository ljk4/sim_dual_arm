"""Ordinary export and audit identify the feedback layout before decoding."""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
from dual_arm_runtime.offline import export_episode, mcap_reader
from dual_arm_runtime.offline.feedback_definition import (
    EXECUTION_FEEDBACK_TYPE,
    LEGACY_FEEDBACK_DEFINITION,
    require_supported_feedback_definition,
)
from dual_arm_runtime.offline.follower_definition import (
    FOLLOWER_REFERENCE_TYPE,
    require_current_follower_reference_definition,
)
from robot_contracts.models import ContractError

_TYPES = {"/robot/execution_feedback": EXECUTION_FEEDBACK_TYPE}
FOLLOWER_INSTALLED = "uint64 curve_duration_ns\n"


class _DefinitionReader:
    def __init__(self, definition: str) -> None:
        self._definition = definition

    def get_all_message_definitions(self):
        return [
            SimpleNamespace(
                topic_type=EXECUTION_FEEDBACK_TYPE,
                encoded_message_definition=self._definition,
            )
        ]


class _Rosbag2:
    def __init__(self, current: str) -> None:
        self._current = current

    class _Source:
        def __init__(self, current: str) -> None:
            self._current = current

        def get_full_text(self, type_name: str):
            assert type_name == EXECUTION_FEEDBACK_TYPE
            return SimpleNamespace(encoded_message_definition=self._current)

    def LocalMessageDefinitionSource(self):  # noqa: N802 - mirrors the ROS API
        return self._Source(self._current)


def test_legacy_definition_is_identified_before_deserialization():
    assert require_supported_feedback_definition(
        _DefinitionReader(LEGACY_FEEDBACK_DEFINITION),
        _TYPES,
        _Rosbag2("robot_api_msgs/GripperFeedback[<=8] grippers"),
    ) is True


def test_identical_definition_is_accepted():
    definition = "robot_api_msgs/GripperFeedback[<=8] grippers"
    assert require_supported_feedback_definition(
        _DefinitionReader(definition), _TYPES, _Rosbag2(definition)
    ) is False


def test_definition_with_only_the_legacy_array_markers_is_not_legacy():
    with pytest.raises(ContractError, match="neither the installed layout"):
        require_supported_feedback_definition(
            _DefinitionReader(
                "robot_api_msgs/GripperFeedback[2] grippers\n"
                "robot_api_msgs/FingerFeedback[2] fingers\n"
            ),
            _TYPES,
            _Rosbag2("robot_api_msgs/GripperFeedback[<=8] grippers"),
        )


class _ExportReader:
    def __init__(
        self, definition: str, messages=(), follower_definition: str = FOLLOWER_INSTALLED
    ) -> None:
        self._definition = definition
        self._messages = list(messages)
        self._follower_definition = follower_definition
        self.closed = False

    def open(self, *_args) -> None:
        pass

    def get_all_topics_and_types(self):
        return [
            SimpleNamespace(name="/robot/execution_feedback", type=EXECUTION_FEEDBACK_TYPE),
            SimpleNamespace(name="/runtime/observation_index",
                            type="sim_api_msgs/msg/ObservationIndex"),
            SimpleNamespace(name="/robot/state", type="robot_api_msgs/msg/RobotState"),
            SimpleNamespace(name="/robot/follower_references",
                            type="robot_api_msgs/msg/FollowerReference"),
            SimpleNamespace(name="/robot/control_events", type="robot_api_msgs/msg/ControlEvent"),
            SimpleNamespace(name="/runtime/commands", type="robot_api_msgs/msg/StreamCommand"),
        ]

    def get_all_message_definitions(self):
        return [
            SimpleNamespace(
                topic_type=EXECUTION_FEEDBACK_TYPE,
                encoded_message_definition=self._definition,
            ),
            SimpleNamespace(
                topic_type=FOLLOWER_REFERENCE_TYPE,
                encoded_message_definition=self._follower_definition,
            ),
        ]

    def has_next(self) -> bool:
        return bool(self._messages)

    def read_next(self):
        return self._messages.pop(0)

    def close(self) -> None:
        self.closed = True


def _install_ros_modules(monkeypatch, reader, installed: str):
    decoded = []

    def deserialize(payload, message_type):
        decoded.append((payload, message_type))
        return SimpleNamespace(state_seq=1, camera_names=("global",))

    rosbag2_py = SimpleNamespace(
        SequentialReader=lambda: reader,
        StorageOptions=lambda **kwargs: kwargs,
        ConverterOptions=lambda *args: args,
        LocalMessageDefinitionSource=lambda: SimpleNamespace(
            get_full_text=lambda name: SimpleNamespace(
                encoded_message_definition=(
                    FOLLOWER_INSTALLED
                    if name == FOLLOWER_REFERENCE_TYPE
                    else installed
                )
            )
        ),
    )
    monkeypatch.setitem(sys.modules, "rosbag2_py", rosbag2_py)
    monkeypatch.setitem(sys.modules, "rclpy", SimpleNamespace())
    monkeypatch.setitem(
        sys.modules, "rclpy.serialization",
        SimpleNamespace(deserialize_message=deserialize),
    )
    monkeypatch.setitem(sys.modules, "rosidl_runtime_py", SimpleNamespace())
    monkeypatch.setitem(
        sys.modules, "rosidl_runtime_py.utilities",
        SimpleNamespace(get_message=lambda type_name: type_name),
    )
    return decoded


def test_export_decodes_legacy_feedback_without_the_current_type(monkeypatch):
    from dual_arm_runtime.offline import legacy_feedback

    reader = _ExportReader(
        LEGACY_FEEDBACK_DEFINITION,
        [
            ("/runtime/observation_index", b"index", 0),
            ("/robot/execution_feedback", b"legacy-payload", 0),
        ],
    )
    decoded = _install_ros_modules(
        monkeypatch, reader, "robot_api_msgs/GripperFeedback[<=8] grippers"
    )
    legacy_payloads = []

    def decode(payload):
        legacy_payloads.append(payload)
        return SimpleNamespace(reference_kind="TRAJECTORY")

    monkeypatch.setattr(legacy_feedback, "decode_legacy_execution_feedback", decode)
    metadata = export_episode._read_metadata(Path("/recording"), "/robot")
    assert metadata["camera_names"] == ("global",)
    assert legacy_payloads == [b"legacy-payload"]
    assert [payload for payload, _type in decoded] == [b"index"]
    assert reader.closed


class _AuditReader(_ExportReader):
    def get_all_topics_and_types(self):
        return [
            SimpleNamespace(name="/robot/info", type="robot_api_msgs/msg/BackendInfo"),
            SimpleNamespace(name="/robot/state", type="robot_api_msgs/msg/RobotState"),
            SimpleNamespace(name="/robot/control_events", type="robot_api_msgs/msg/ControlEvent"),
            SimpleNamespace(
                name="/robot/execution_feedback", type=EXECUTION_FEEDBACK_TYPE
            ),
            SimpleNamespace(
                name="/robot/follower_references", type=FOLLOWER_REFERENCE_TYPE
            ),
            SimpleNamespace(name="/runtime/commands", type="robot_api_msgs/msg/StreamCommand"),
            SimpleNamespace(
                name="/runtime/observation_index",
                type="sim_api_msgs/msg/ObservationIndex",
            ),
            SimpleNamespace(
                name="/robot/cameras/global/frame",
                type="robot_api_msgs/msg/CameraFrame",
            ),
        ]


class _FakeAudit:
    root = "/robot"
    reference_topic = "/robot/follower_references"
    camera_names = ("global",)
    required_capabilities = frozenset(
        {"execution_feedback.gripper_set.v1", "bounded_target_follower_v1"}
    )

    def __init__(self) -> None:
        from collections import Counter

        self.counts = Counter()
        self.feedback = []

    def context_value(self, _context) -> bool:
        return True

    def capture_payload(self, *_args) -> None:
        pass

    def state(self, _message) -> None:
        pass

    def frame(self, _topic, _message) -> None:
        pass

    def observation(self, _message) -> None:
        pass

    def command(self, _message) -> None:
        pass

    def reference(self, _message) -> None:
        pass

    def event(self, _message) -> None:
        pass

    def info(self, _message) -> None:
        pass

    def feedback_message(self, message) -> None:
        self.feedback.append(message)

    def finish(self) -> dict:
        return {"ok": True}


def test_audit_decodes_legacy_feedback_and_accepts_the_recorded_layout(monkeypatch):
    from dual_arm_runtime.offline import legacy_feedback

    reader = _AuditReader(
        LEGACY_FEEDBACK_DEFINITION,
        [("/robot/execution_feedback", b"legacy-payload", 0)],
    )
    decoded = _install_ros_modules(
        monkeypatch, reader, "robot_api_msgs/GripperFeedback[<=8] grippers"
    )
    execution = SimpleNamespace(context="ctx", reference_kind="TRAJECTORY")

    def decode(payload):
        assert payload == b"legacy-payload"
        return execution

    monkeypatch.setattr(legacy_feedback, "decode_legacy_execution_feedback", decode)
    audit = _FakeAudit()
    assert mcap_reader._run_audit(audit, Path("/recording")) == {"ok": True}
    assert audit.required_capabilities == frozenset({"bounded_target_follower_v1"})
    assert audit.feedback == [execution]
    assert decoded == []
    assert reader.closed


def test_export_reads_current_layout(monkeypatch):
    current = "robot_api_msgs/GripperFeedback[<=8] grippers"
    reader = _ExportReader(current, [("/runtime/observation_index", b"index", 0)])
    decoded = _install_ros_modules(monkeypatch, reader, current)
    metadata = export_episode._read_metadata(Path("/recording"), "/robot")
    assert metadata["camera_names"] == ("global",)
    assert len(decoded) == 1
    assert reader.closed


FOLLOWER_CURRENT = (
    "uint64 motion_seq\n"
    "uint64 requested_window_ns\n"
    "uint64 curve_duration_ns\n"
    "uint64 duration_grid_ns\n"
)
FOLLOWER_LEGACY = FOLLOWER_CURRENT.replace(
    "curve_duration_ns", "ruckig_duration_ns"
)


class _FollowerDefinitionReader:
    def __init__(self, definition: str | None) -> None:
        self._definition = definition

    def get_all_message_definitions(self):
        if self._definition is None:
            return []
        return [
            SimpleNamespace(
                topic_type=FOLLOWER_REFERENCE_TYPE,
                encoded_message_definition=self._definition,
            )
        ]


class _FollowerRosbag:
    def __init__(self, installed: str) -> None:
        self._installed = installed

    class _Source:
        def __init__(self, installed: str) -> None:
            self._installed = installed

        def get_full_text(self, type_name: str):
            assert type_name == FOLLOWER_REFERENCE_TYPE
            return SimpleNamespace(encoded_message_definition=self._installed)

    def LocalMessageDefinitionSource(self):  # noqa: N802 - mirrors the ROS API
        return self._Source(self._installed)


def test_follower_current_definition_needs_no_legacy_digest():
    assert require_current_follower_reference_definition(
        _FollowerDefinitionReader(FOLLOWER_CURRENT),
        {"/robot/follower_references": FOLLOWER_REFERENCE_TYPE},
        _FollowerRosbag(FOLLOWER_CURRENT),
    ) is False


def test_follower_field_rename_is_the_known_legacy_layout():
    assert require_current_follower_reference_definition(
        _FollowerDefinitionReader(FOLLOWER_LEGACY),
        {"/robot/follower_references": FOLLOWER_REFERENCE_TYPE},
        _FollowerRosbag(FOLLOWER_CURRENT),
    ) is True


@pytest.mark.parametrize(
    "recorded",
    (
        FOLLOWER_CURRENT.replace("uint64 curve_duration_ns", "int64 curve_duration_ns"),
        FOLLOWER_LEGACY + "uint64 extra_ns\n",
        FOLLOWER_LEGACY.replace("uint64 duration_grid_ns", "int64 duration_grid_ns"),
    ),
)
def test_follower_other_layout_changes_are_rejected(recorded):
    with pytest.raises(ContractError, match="neither the installed layout"):
        require_current_follower_reference_definition(
            _FollowerDefinitionReader(recorded),
            {"/robot/follower_references": FOLLOWER_REFERENCE_TYPE},
            _FollowerRosbag(FOLLOWER_CURRENT),
        )


def test_follower_missing_stored_definition_is_rejected():
    with pytest.raises(ContractError, match="no stored definition"):
        require_current_follower_reference_definition(
            _FollowerDefinitionReader(None),
            {"/robot/follower_references": FOLLOWER_REFERENCE_TYPE},
            _FollowerRosbag(FOLLOWER_CURRENT),
        )
