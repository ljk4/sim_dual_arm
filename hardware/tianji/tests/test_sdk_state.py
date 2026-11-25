import io
import json
import threading

import pytest

from robot_backend_hardware.tianji.apex_sdk import ApexClient
from robot_backend_hardware.tianji.apex_sdk.errors import TimeoutError
from robot_backend_hardware.tianji.apex_sdk.realtime import RealtimeChannel


def test_joint_target_uses_tcp_json_and_preserves_degrees():
    messages = []

    class Socket:
        def sendall(self, data):
            messages.append(data)

    client = ApexClient.__new__(ApexClient)
    client._seq = 0
    client._lock = threading.Lock()
    client._dof = 7
    client._sock = Socket()
    client._file = io.StringIO('{"ok": true}\n')
    target = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0]
    client.update_tracking_joints("arm_left", target)

    assert messages[0].endswith(b"\n")
    assert json.loads(messages[0]) == {
        "id": 1, "cmd": "update_tracking_target", "arm": "arm_left",
        "jointPositionTarget": target,
    }


def channel_with_reversed_ids():
    channel = RealtimeChannel("controller.invalid")
    channel._by_arm = {
        0: {"arm": "arm_right", "jointState": {"positions": [20.0] * 7}},
        1: {"arm": "arm_left", "jointState": {"positions": [10.0] * 7}},
    }
    return channel


def test_explicit_arm_name_takes_priority_over_udp_index():
    state = channel_with_reversed_ids().build_state(["arm_left", "arm_right"], arm="arm_left")
    assert len(state["arms"]) == 1
    assert state["arms"][0]["arm"] == "arm_left"
    assert state["arms"][0]["jointState"]["positions"] == [10.0] * 7


def test_unknown_arm_does_not_fall_back_to_another_arm():
    assert channel_with_reversed_ids().build_state(["arm_left", "arm_right"], arm="missing")["arms"] == []


@pytest.mark.parametrize("positions", [[], [0.0] * 3])
def test_missing_joint_feedback_is_rejected(positions):
    client = ApexClient.__new__(ApexClient)
    client._dof = 7
    client.get_state = lambda **kwargs: {"arms": [{"jointState": {"positions": positions}}]}
    with pytest.raises(TimeoutError):
        client.get_joint_positions("arm_left")


def test_udp_parser_preserves_measurement_identity_and_sequence():
    packet = {"type": "status-push", "seq": 25, "timestamp_us": 12345,
              "armId": 1, "arm": "arm_left", "jointState": {"positions": [10.0] * 7}}
    raw, state = RealtimeChannel._parse_json(json.dumps(packet).encode())
    assert raw["arm"] == "arm_left"
    assert state.sequence == 25
    assert state.timestamp_us == 12345
    assert state.q == [10.0] * 7
