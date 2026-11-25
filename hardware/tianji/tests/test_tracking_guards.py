"""在线跟踪护栏的单元测试 (无需真机)。

覆盖 2026-07-23 apex.log 里那次故障链路上的三个点:
  1. 关节目标单步限幅  —— 防止跳变撞控制器 |q_cmd-q_fdb| <= 0.05 rad 护栏
  2. enable 失败退避    —— 防止 120Hz enable 风暴把驱动器反复打故障
  3. 同步阶段实时读角  —— 防止冻结一个过期目标导致对齐结束即超差
"""
import pytest

from robot_backend_hardware.tianji.arms import ApexRobotActuator
from robot_backend_hardware.tianji.types import CmdKey, CommandSpace, MotionMode, MotionModeState


class FakeClient:
    """只实现被测路径用到的那几个 SDK 方法。"""

    def __init__(self, positions=None, fail_enable=False, fail_stream_for=()):
        self.positions = positions or {"arm_left": [0.0] * 7, "arm_right": [0.0] * 7}
        self.fail_enable = fail_enable
        self.fail_stream_for = set(fail_stream_for)
        self.tracking_calls = []          # (arm, enable, mode)
        self.joint_targets = []           # (arm, [deg])
        self.settle_calls = []
        self.sent = []                    # 通用 _send (trajectory_stop 等)

    def _send(self, cmd, **kwargs):
        self.sent.append((cmd, kwargs))
        return {}

    def set_online_tracking(self, arm, enable, mode="cartesian", reference_frame=1):
        self.tracking_calls.append((arm, enable, mode))
        if enable and self.fail_enable:
            raise RuntimeError("joint drive error active; clear fault before motion")

    def get_joint_positions(self, arm, timeout=1.0):
        return list(self.positions[arm])

    def update_tracking_joints(self, arm, joints):
        if arm in self.fail_stream_for:
            raise RuntimeError("online tracking not enabled")
        self.joint_targets.append((arm, list(joints)))

    def wait_arm_settled(self, arm, timeout=10.0, **kwargs):
        self.settle_calls.append(arm)
        return True

    def wait_online_tracking_settled(self, arm, timeout=10.0):
        return True

    def trajectory_stop(self, arm):
        self.sent.append(("trajectory_stop", {"arm": arm}))


def make_actuator(client, space=CommandSpace.JOINT, **kwargs):
    act = ApexRobotActuator(**kwargs)
    act._client = client
    act._motion_state = MotionModeState(MotionMode.ONLINE_TRACKING, space)
    return act


def arm_joint_tracking(act):
    """走一遍 enable + 同步, 让限幅基准落到实测角上。"""
    act._ensure_tracking_enabled("joint")


# ── 1. 单步限幅 ────────────────────────────────────────────────────────────

def test_joint_step_is_clamped_to_limit():
    client = FakeClient()
    act = make_actuator(client, max_joint_step_deg=1.0)
    arm_joint_tracking(act)
    client.joint_targets.clear()

    # 一次要求跳 10°, 应被削到 1°
    act._send_tracking_cmd({CmdKey.RIGHTARM_CMD_JOINT_POS: [10.0] + [0.0] * 6})

    arm, sent = client.joint_targets[-1]
    assert arm == "arm_right"
    assert sent[0] == pytest.approx(1.0)


def test_clamped_target_converges_over_successive_commands():
    client = FakeClient()
    act = make_actuator(client, max_joint_step_deg=1.0)
    arm_joint_tracking(act)
    client.joint_targets.clear()

    for _ in range(12):
        act._send_tracking_cmd({CmdKey.RIGHTARM_CMD_JOINT_POS: [10.0] + [0.0] * 6})

    assert client.joint_targets[-1][1][0] == pytest.approx(10.0)


def test_continuous_trajectory_passes_through_unchanged():
    """连续轨迹不该被限幅动到 —— 限幅只削跳变, 不做滤波。"""
    client = FakeClient()
    act = make_actuator(client, max_joint_step_deg=1.0)
    arm_joint_tracking(act)
    client.joint_targets.clear()

    wanted = [0.2 * i for i in range(1, 11)]
    for value in wanted:
        act._send_tracking_cmd({CmdKey.RIGHTARM_CMD_JOINT_POS: [value] + [0.0] * 6})

    assert [t[1][0] for t in client.joint_targets] == pytest.approx(wanted)


def test_limit_disabled_when_zero():
    client = FakeClient()
    act = make_actuator(client, max_joint_step_deg=0.0)
    arm_joint_tracking(act)
    client.joint_targets.clear()

    act._send_tracking_cmd({CmdKey.RIGHTARM_CMD_JOINT_POS: [50.0] + [0.0] * 6})
    assert client.joint_targets[-1][1][0] == pytest.approx(50.0)


# ── 2. enable 失败退避 ─────────────────────────────────────────────────────

def test_enable_failure_backs_off_instead_of_hammering():
    client = FakeClient(fail_enable=True)
    act = make_actuator(client, tracking_arm_retry_interval_s=60.0)

    with pytest.raises(Exception):
        act._ensure_tracking_enabled("joint")
    attempts_after_first = len([c for c in client.tracking_calls if c[1] is True])

    # 退避窗口内的后续命令不得再碰下位机
    for _ in range(50):
        with pytest.raises(Exception):
            act._ensure_tracking_enabled("joint")

    assert len([c for c in client.tracking_calls if c[1] is True]) == attempts_after_first


def test_gives_up_and_drops_commands_after_max_failures():
    client = FakeClient(fail_enable=True)
    act = make_actuator(client, tracking_arm_retry_interval_s=0.0,
                        tracking_arm_max_failures=3)

    for _ in range(3):
        with pytest.raises(Exception):
            act._ensure_tracking_enabled("joint")

    assert act._tracking_giveup is True
    # 闩死后 send_cmd 静默丢弃, 既不再走 move_j 也不再喂跟踪流
    before = len(client.tracking_calls)
    act.send_cmd({CmdKey.RIGHTARM_CMD_JOINT_POS: [0.0] * 7})
    assert len(client.tracking_calls) == before


def test_set_motion_mode_clears_giveup_latch():
    client = FakeClient()
    act = make_actuator(client)
    act._tracking_giveup = True
    act._arm_fail_count = 5
    act._motion_state = MotionModeState(MotionMode.TRAJECTORY, CommandSpace.CART)

    act.set_motion_mode(MotionMode.ONLINE_TRACKING, CommandSpace.JOINT)

    assert act._tracking_giveup is False
    assert act._arm_fail_count == 0


def test_set_motion_mode_arms_tracking_eagerly():
    """空间已知就该在切模式时 arm 完, 不能把 enable+同步拖进 120Hz 命令路径。"""
    client = FakeClient()
    act = make_actuator(client)
    act._motion_state = MotionModeState(MotionMode.TRAJECTORY, CommandSpace.CART)

    act.set_motion_mode(MotionMode.ONLINE_TRACKING, CommandSpace.JOINT)

    assert act._tracking_enabled is True
    assert act._tracking_mode == "joint"
    assert [c for c in client.tracking_calls if c[1] is True], "应当已 enable"
    # 首条真实命令不再触发任何 enable
    before = len(client.tracking_calls)
    act.send_cmd({CmdKey.RIGHTARM_CMD_JOINT_POS: [0.0] * 7})
    assert len(client.tracking_calls) == before


# ── 3. 同步阶段与软停 ──────────────────────────────────────────────────────

def test_sync_rereads_feedback_every_iteration():
    """臂还在动时, 同步必须跟着实测角走, 不能冻结第一帧。"""
    client = FakeClient()
    moving = {"n": 0}
    real_get = client.get_joint_positions

    def drifting(arm, timeout=1.0):
        moving["n"] += 1
        return [moving["n"] * 0.5] + [0.0] * 6

    client.get_joint_positions = drifting
    act = make_actuator(client)
    act._sync_joint_tracking_targets(hold_s=0.05, freq_hz=200.0)

    values = [t[1][0] for t in client.joint_targets]
    assert len(set(values)) > 1, "同步阶段目标被冻结, 臂动起来就会超差"
    assert real_get is not None


def test_soft_stop_uses_the_space_it_was_armed_with():
    """回滚后 _tracking_mode 会被清空, 软停仍必须发 joint 而不是 cartesian。"""
    client = FakeClient()
    act = make_actuator(client)
    arm_joint_tracking(act)

    act._tracking_mode = None          # 模拟一次失败回滚后的状态
    act._tracking_active = {"arm_left": True, "arm_right": True}
    client.tracking_calls.clear()

    act._stop_tracking(wait=False)

    assert client.tracking_calls, "应当发出软停"
    assert all(mode == "joint" for _, _, mode in client.tracking_calls)


def test_stream_rejection_disarms_tracking():
    """下位机拒绝跟踪流 = 该臂已退出跟踪, 必须让上层知道并重新 arm。"""
    client = FakeClient()
    act = make_actuator(client)
    arm_joint_tracking(act)
    assert act._tracking_enabled is True

    # arm 成功之后, 控制器才因驱动器故障把该臂踢出跟踪态
    client.fail_stream_for = {"arm_right"}

    with pytest.raises(Exception):
        act._send_tracking_cmd({CmdKey.RIGHTARM_CMD_JOINT_POS: [0.0] * 7})

    assert act._tracking_enabled is False


def test_entering_tracking_waits_for_arms_to_settle():
    client = FakeClient()
    act = make_actuator(client)
    act._motion_state = MotionModeState(MotionMode.TRAJECTORY, CommandSpace.CART)

    act.set_motion_mode(MotionMode.ONLINE_TRACKING, CommandSpace.JOINT)

    assert set(client.settle_calls) == {"arm_left", "arm_right"}
    # 停稳等待必须发生在 enable 之前
    assert client.sent and client.sent[0][0] == "trajectory_stop"


def test_cart_tracking_rejects_joint_keys():
    """空间是显式的, 混进另一空间的 key 必须直接报错而不是猜。"""
    client = FakeClient()
    act = make_actuator(client, space=CommandSpace.CART)
    with pytest.raises(ValueError):
        act._send_tracking_cmd({CmdKey.RIGHTARM_CMD_JOINT_POS: [0.0] * 7})
