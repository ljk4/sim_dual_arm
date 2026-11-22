"""Export and audit decode the installed MCAP layouts."""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
from dual_arm_runtime.offline.export import export_episode, mcap_reader
from dual_arm_runtime.offline.export.follower_definition import (
    FOLLOWER_REFERENCE_TYPE,
    require_current_follower_reference_definition,
)
from robot_contracts.models import ContractError

FOLLOWER_INSTALLED = "uint64 curve_duration_ns\n"


class _ExportReader:
    def __init__(
        self, messages=(), follower_definition: str = FOLLOWER_INSTALLED
    ) -> None:
        self._messages = list(messages)
        self._follower_definition = follower_definition
        self.closed = False

    def open(self, *_args) -> None:
        pass

    def get_all_topics_and_types(self):
        return [
            SimpleNamespace(
                name="/runtime/observation_index",
                type="sim_api_msgs/msg/ObservationIndex",
            ),
            SimpleNamespace(name="/robot/state", type="robot_api_msgs/msg/RobotState"),
            SimpleNamespace(
                name="/robot/follower_references",
                type="robot_api_msgs/msg/FollowerReference",
            ),
            SimpleNamespace(
                name="/robot/control_events",
                type="robot_api_msgs/msg/ControlEvent",
            ),
            SimpleNamespace(
                name="/runtime/commands", type="robot_api_msgs/msg/StreamCommand"
            ),
        ]

    def get_all_message_definitions(self):
        return [
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


def _install_ros_modules(monkeypatch, reader):
    decoded = []

    def deserialize(payload, message_type):
        decoded.append((payload, message_type))
        return SimpleNamespace(
            state_seq=1,
            camera_names=("global",),
            context=SimpleNamespace(
                robot_id="robot",
                robot_profile_hash="profile",
                backend_instance_id="backend",
                time_domain_id="sim",
                clock_epoch=0,
                episode_id="episode",
                control_epoch=1,
            ),
        )

    rosbag2_py = SimpleNamespace(
        SequentialReader=lambda: reader,
        StorageOptions=lambda **kwargs: kwargs,
        ConverterOptions=lambda *args: args,
        LocalMessageDefinitionSource=lambda: SimpleNamespace(
            get_full_text=lambda name: SimpleNamespace(
                encoded_message_definition=FOLLOWER_INSTALLED
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


class _AuditReader(_ExportReader):
    def get_all_topics_and_types(self):
        return [
            SimpleNamespace(name="/robot/info", type="robot_api_msgs/msg/BackendInfo"),
            SimpleNamespace(name="/robot/state", type="robot_api_msgs/msg/RobotState"),
            SimpleNamespace(
                name="/robot/control_events", type="robot_api_msgs/msg/ControlEvent"
            ),
            SimpleNamespace(
                name="/robot/execution_feedback",
                type="robot_api_msgs/msg/ExecutionFeedback",
            ),
            SimpleNamespace(
                name="/robot/follower_references", type=FOLLOWER_REFERENCE_TYPE
            ),
            SimpleNamespace(
                name="/runtime/commands", type="robot_api_msgs/msg/StreamCommand"
            ),
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
        {"execution_feedback.gripper_set", "bounded_target_follower"}
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


def test_audit_decodes_installed_feedback(monkeypatch):
    reader = _AuditReader(
        [("/robot/execution_feedback", b"payload", 0),]
    )
    decoded = _install_ros_modules(monkeypatch, reader)
    execution = SimpleNamespace(context="ctx", reference_kind="TRAJECTORY")
    monkeypatch.setattr(
        mcap_reader, "decode_message", lambda *args, **kwargs: execution
    )
    audit = _FakeAudit()
    assert mcap_reader._run_audit(audit, Path("/recording")) == {"ok": True}
    assert audit.feedback == [execution]
    assert [payload for payload, _type in decoded] == [b"payload"]
    assert reader.closed


def test_export_reads_installed_layout(monkeypatch):
    reader = _ExportReader([("/runtime/observation_index", b"index", 0)])
    decoded = _install_ros_modules(monkeypatch, reader)
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
FOLLOWER_RENAMED = FOLLOWER_CURRENT.replace(
    "curve_duration_ns", "other_duration_ns"
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


def test_follower_current_definition_is_accepted():
    assert require_current_follower_reference_definition(
        _FollowerDefinitionReader(FOLLOWER_CURRENT),
        {"/robot/follower_references": FOLLOWER_REFERENCE_TYPE},
        _FollowerRosbag(FOLLOWER_CURRENT),
    ) is None


@pytest.mark.parametrize(
    "recorded",
    (
        FOLLOWER_CURRENT.replace("uint64 curve_duration_ns", "int64 curve_duration_ns"),
        FOLLOWER_RENAMED,
        FOLLOWER_RENAMED + "uint64 extra_ns\n",
        FOLLOWER_CURRENT.replace("uint64 duration_grid_ns", "int64 duration_grid_ns"),
    ),
)
def test_follower_other_layout_changes_are_rejected(recorded):
    with pytest.raises(ContractError, match="differs from the installed layout"):
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
