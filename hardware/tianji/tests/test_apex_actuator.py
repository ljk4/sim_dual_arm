import time

import numpy as np

from robot_backend_hardware.tianji.arms import ApexRobotActuator
from robot_backend_hardware.tianji.types import CmdKey, CommandSpace, MotionMode, MotionModeState, StateKey


class FakeRealtime:
    def __init__(self) -> None:
        self.states = {
            0: {
                "jointState": {
                    "positions": [1.0] * 7,
                    "torques": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
                },
                "currentPose": {"x": 0.1, "y": 0.2, "z": 0.3, "rx": 0, "ry": 0, "rz": 0},
            },
            1: {
                "jointState": {
                    "positions": [2.0] * 7,
                    "torques": [1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7],
                },
                "currentPose": {"x": 0.4, "y": 0.5, "z": 0.6, "rx": 0, "ry": 0, "rz": 0},
            },
        }

    def latest_arm_raw(self, arm_id):
        return self.states.get(arm_id)


class FakeApexClient:
    def __init__(self) -> None:
        self.realtime = FakeRealtime()
        self.calls = []

    def trajectory_stop(self, arm):
        self.calls.append(("trajectory_stop", arm))

    def set_online_tracking(self, arm, enable, mode, reference_frame=1):
        self.calls.append(("set_online_tracking", arm, enable, mode, reference_frame))

    def wait_online_tracking_settled(self, arm, timeout=8.0):
        self.calls.append(("wait_online_tracking_settled", arm, timeout))
        return True

    def update_tracking_joints(self, arm, joints):
        self.calls.append(("update_tracking_joints", arm, list(joints)))

    def update_tracking_pose(self, arm, **pose):
        self.calls.append(("update_tracking_pose", arm, pose))

    def move_j(self, arm, joints, vel=0.3, arm_id=0):
        self.calls.append(("move_j", arm, list(joints), vel, arm_id))

    def move_l(self, arm, pose, vel=0.3, arm_id=0):
        self.calls.append(("move_l", arm, pose, vel, arm_id))

    def wait_motion_done(self, *args, **kwargs):
        self.calls.append(("wait_motion_done",))
        return True

    def start_realtime(self, arm, freq_hz=120):
        self.calls.append(("start_realtime", arm, freq_hz))


def _actuator():
    actuator = ApexRobotActuator()
    actuator._client = FakeApexClient()
    return actuator


def _armed(actuator, command_space):
    """把 actuator 置成「已 arm」状态。

    命令路径现在会在发现跟踪未启用时自行 arm (驱动器故障被踢出跟踪态后的恢复
    路径)。这些用例只关心 key 路由, 所以先跳过启用流程, 免得断言被 arm 产生的
    SDK 调用干扰。
    """
    space = "joint" if command_space == CommandSpace.JOINT else "cartesian"
    actuator._motion_state = MotionModeState(MotionMode.ONLINE_TRACKING, command_space)
    actuator._tracking_enabled = True
    actuator._tracking_mode = space
    actuator._last_armed_space = space
    actuator._tracking_active = {"arm_left": True, "arm_right": True}


def test_poll_state_reads_sdk_realtime_cache() -> None:
    actuator = _actuator()

    state = actuator.poll_state()

    np.testing.assert_array_equal(state[StateKey.LEFTARM_STATE_JOINT_POS], [1.0] * 7)
    np.testing.assert_array_equal(state[StateKey.RIGHTARM_STATE_JOINT_POS], [2.0] * 7)
    np.testing.assert_allclose(
        state[StateKey.LEFTARM_STATE_CART_POS],
        [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],
    )
    np.testing.assert_allclose(
        state[StateKey.LEFTARM_STATE_JOINT_TORQUE],
        [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
    )
    np.testing.assert_allclose(
        state[StateKey.RIGHTARM_STATE_JOINT_TORQUE],
        [1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7],
    )


def test_poll_state_requests_realtime_restart_when_udp_is_stale() -> None:
    actuator = _actuator()
    stale = time.monotonic() - 1.0
    actuator._rt_last_update = {0: stale, 1: stale}

    state = actuator.poll_state()

    assert StateKey.LEFTARM_STATE_JOINT_POS not in state
    assert StateKey.RIGHTARM_STATE_JOINT_POS not in state
    assert actuator._client.calls == [
        ("start_realtime", "arm_left", 120),
        ("start_realtime", "arm_right", 120),
    ]
    assert actuator._rt_recovery_active


def test_poll_state_requests_realtime_restart_when_arm_cache_is_empty() -> None:
    actuator = _actuator()
    actuator._client.realtime.states.pop(1)

    state = actuator.poll_state()

    assert StateKey.LEFTARM_STATE_JOINT_POS in state
    assert StateKey.RIGHTARM_STATE_JOINT_POS not in state
    assert actuator._client.calls == [
        ("start_realtime", "arm_right", 120),
    ]


def test_poll_state_recreates_missing_realtime_channel() -> None:
    actuator = _actuator()
    actuator._client.realtime = None
    attached = []
    actuator._attach_rt_listener = lambda: attached.append(True)

    assert actuator.poll_state() == {}
    assert actuator._client.calls == [
        ("start_realtime", "arm_left", 120),
        ("start_realtime", "arm_right", 120),
    ]
    assert attached == [True]


def test_realtime_recovery_reconnects_when_restart_does_not_restore_udp() -> None:
    actuator = _actuator()
    stale = time.monotonic() - 1.0
    actuator._rt_last_update = {0: stale, 1: stale}
    events = []
    actuator._rt_recovery_attempts = 1
    actuator._disconnect = lambda: events.append("disconnect")
    actuator._try_reconnect = lambda: events.append("reconnect")

    actuator.poll_state()

    assert events == ["disconnect", "reconnect"]


def test_fresh_udp_resets_recovery_state() -> None:
    actuator = _actuator()
    now = time.monotonic()
    actuator._rt_last_update = {0: now, 1: now}
    actuator._rt_recovery_active = True
    actuator._rt_recovery_attempts = 2

    state = actuator.poll_state()

    assert StateKey.LEFTARM_STATE_JOINT_POS in state
    assert StateKey.RIGHTARM_STATE_JOINT_POS in state
    assert not actuator._rt_recovery_active
    assert actuator._rt_recovery_attempts == 0


def test_one_joint_tracking_mode_switch_configures_both_arms() -> None:
    actuator = _actuator()
    actuator._sync_joint_tracking_targets = lambda **_kwargs: None

    actuator.set_motion_mode(MotionMode.ONLINE_TRACKING, CommandSpace.JOINT)

    assert actuator.get_motion_mode() == MotionModeState(
        MotionMode.ONLINE_TRACKING,
        CommandSpace.JOINT,
    )
    assert actuator._client.calls == [
        ("trajectory_stop", "arm_left"),
        ("trajectory_stop", "arm_right"),
        ("set_online_tracking", "arm_left", True, "joint", 1),
        ("update_tracking_joints", "arm_left", [1.0] * 7),
        ("set_online_tracking", "arm_right", True, "joint", 1),
        ("update_tracking_joints", "arm_right", [2.0] * 7),
    ]


def test_joint_tracking_routes_existing_joint_command_keys() -> None:
    actuator = _actuator()
    _armed(actuator, CommandSpace.JOINT)

    actuator.send_cmd({
        CmdKey.LEFTARM_CMD_JOINT_POS: np.arange(7, dtype=np.float64),
        CmdKey.RIGHTARM_CMD_JOINT_POS: np.arange(10, 17, dtype=np.float64),
    })

    assert actuator._client.calls == [
        ("update_tracking_joints", "arm_left", list(range(7))),
        ("update_tracking_joints", "arm_right", list(range(10, 17))),
    ]


def test_cartesian_tracking_uses_public_sdk_api() -> None:
    actuator = _actuator()
    _armed(actuator, CommandSpace.CART)

    actuator.send_cmd({
        CmdKey.LEFTARM_CMD_CART_POS: np.array([0.1, 0.2, 0.3, 0, 0, 0, 1], dtype=np.float64),
    })

    name, arm, pose = actuator._client.calls[0]
    assert name == "update_tracking_pose"
    assert arm == "arm_left"
    assert pose["x"] == 0.1
    assert pose["reference_frame"] == 1


def test_move_to_pose_routes_joint_targets() -> None:
    actuator = _actuator()

    actuator.move_to_pose(joint_positions={"left": [0.0] * 7}, vel=0.2)

    assert actuator._client.calls == [
        ("move_j", "arm_left", [0.0] * 7, 0.2, 0),
        ("wait_motion_done",),
    ]


def test_move_to_pose_routes_cartesian_targets() -> None:
    actuator = _actuator()

    actuator.move_to_pose(
        cart_positions={"right": [0.4, 0.5, 0.6, 0, 0, 0, 1]},
        vel=0.25,
        block=False,
    )

    assert len(actuator._client.calls) == 1
    name, arm, pose, vel, arm_id = actuator._client.calls[0]
    assert name == "move_l"
    assert arm == "arm_right"
    assert arm_id == 1
    assert vel == 0.25
    assert pose["x"] == 0.4


def test_move_to_pose_rejects_same_arm_dual_target() -> None:
    actuator = _actuator()

    try:
        actuator.move_to_pose(
            joint_positions={"left": [0.0] * 7},
            cart_positions={"left": [0.1, 0.2, 0.3, 0, 0, 0, 1]},
        )
    except ValueError as error:
        assert "left" in str(error)
    else:
        raise AssertionError("expected ValueError for conflicting targets")
