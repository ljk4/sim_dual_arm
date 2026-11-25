"""APEX dual-arm device driver with tracking startup and fault protection.

Joint targets and feedback use degrees. The embedded SDK owns the TCP
command connection and the sole UDP state receiver.
"""

import logging
import math
import socket
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any, Optional

import numpy as np

from robot_backend_hardware.common.device import Actuator
from robot_backend_hardware.tianji.types import (
    CmdKey,
    CommandSpace,
    MotionMode,
    MotionModeState,
    StateKey,
)

log = logging.getLogger(__name__)

# Apex robotState 枚举 (与 UDP status-push 一致)
ROBOT_STATE_ESTOP = 6
ROBOT_STATE_FAULT = 7


def _euler_xyz_to_quat(rx: float, ry: float, rz: float) -> tuple:
    """Convert fixed-axis XYZ (RPY) Euler angles (radians) to quaternion (qx,qy,qz,qw).

    This matches tf2 setRPY(rx, ry, rz) convention:
    R = Rz(rz) * Ry(ry) * Rx(rx)  (extrinsic/fixed-axis XYZ)
    """
    cx, sx = np.cos(rx / 2), np.sin(rx / 2)
    cy, sy = np.cos(ry / 2), np.sin(ry / 2)
    cz, sz = np.cos(rz / 2), np.sin(rz / 2)
    qw = cx * cy * cz + sx * sy * sz
    qx = sx * cy * cz - cx * sy * sz
    qy = cx * sy * cz + sx * cy * sz
    qz = cx * cy * sz - sx * sy * cz
    return float(qx), float(qy), float(qz), float(qw)


def _quat_to_euler_xyz(qx: float, qy: float, qz: float, qw: float) -> tuple:
    """Convert quaternion (qx,qy,qz,qw) to fixed-axis XYZ (RPY) Euler angles (radians).

    Inverse of _euler_xyz_to_quat. Matches tf2 getRPY() convention.
    """
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    rx = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = np.clip(sinp, -1.0, 1.0)
    ry = np.arcsin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    rz = np.arctan2(siny_cosp, cosy_cosp)

    return float(rx), float(ry), float(rz)


def _sdk_space(command_space: CommandSpace) -> str:
    """CommandSpace → apex_sdk 的 mode 字符串。"""
    return "joint" if command_space == CommandSpace.JOINT else "cartesian"


class ApexRobotActuator(Actuator):
    """Actuator implementation for the Apex robot controller."""

    supports_pose_motion = True

    def __init__(
        self,
        name: str = "apex_robot",
        host: str = "",
        left_arm: str = "arm_left",
        right_arm: str = "arm_right",
        enable_on_start: bool = False,
        command_vel: float = 0.3,
        connect_timeout_s: float = 10.0,
        auto_clear_fault: bool = True,
        max_joint_step_deg: float = 1.0,
        tracking_arm_retry_interval_s: float = 1.0,
        tracking_arm_max_failures: int = 3,
        tracking_settle_timeout_s: float = 3.0,
        poll_hz: float = 120.0,
        command_dump_dir: str | None = None,
    ) -> None:
        super().__init__(name)
        self._host = host
        self._left_arm = left_arm
        self._right_arm = right_arm
        self._enable_on_start = enable_on_start
        self._command_vel = command_vel
        self._connect_timeout_s = connect_timeout_s
        self._auto_clear_fault = auto_clear_fault
        # 单条命令允许的最大关节增量 (度)。控制器 CSP 有 |q_cmd-q_fdb| <= 0.05 rad
        # (≈2.87°) 的硬护栏, 超过就拒绝该步并很快锁存跟随误差故障 (0xFF73)。
        # 上游任何一次跳变 (chunk 边界 / 命令队列丢弃了中间点 / 模型野值) 都会踩到,
        # 所以在最靠近下位机的地方按步限幅, 把跳变摊成多个周期。
        self._max_joint_step_deg = float(max_joint_step_deg)
        self._tracking_arm_retry_interval_s = float(tracking_arm_retry_interval_s)
        self._tracking_arm_max_failures = int(tracking_arm_max_failures)
        self._tracking_settle_timeout_s = float(tracking_settle_timeout_s)

        self._client = None
        self._ApexClient = None
        self._apex_error_types: tuple[type[BaseException], ...] = ()
        self._apex_conn_error_types: tuple[type[BaseException], ...] = (OSError,)
        self._consecutive_errors = 0
        self._max_consecutive_errors = 5
        self._reconnect_interval_s = 3.0
        self._last_reconnect_attempt = 0.0

        self._motion_state = MotionModeState(MotionMode.TRAJECTORY, CommandSpace.CART)
        self._tracking_mode = None          # "joint" | "cartesian" | None(未定)
        self._tracking_enabled = False      # 是否已 enable 在线跟踪
        self._tracking_active: dict[str, bool] = {}  # arm_name -> active
        # 最近一次实际 enable 用的空间。_tracking_mode 在软停/失败时会被清成 None,
        # 但软停请求必须带上**当初 enable 的那个空间**, 否则控制器收到的是
        # mode="cartesian" 的软停而实际跑的是 joint 跟踪。
        self._last_armed_space: Optional[str] = None
        # 上一次成功下发的关节目标 (arm_name -> [deg]), 用于按步限幅
        self._last_joint_cmd: dict[str, list[float]] = {}
        self._joint_clamp_warned_at = 0.0
        # 在线跟踪 arm(启用) 失败后的退避状态: 没有它, 命令流水线会以 cmd_hz
        # (120Hz) 无限重试 enable, 每次都让控制器重新初始化 CSP 并再次触发驱动器
        # 故障 —— 实测 2.5s 内把 J6 反复打故障 44 次, 最终锁死需人工复位。
        self._arm_fail_count = 0
        self._last_arm_attempt = 0.0
        # 连续启用失败到达阈值后闩死, 只有重新 set_motion_mode 才解除
        self._tracking_giveup = False
        self._giveup_warned_at = 0.0

        self._cmd_log_counter = 0
        self._cmd_log_interval = 10  # print every N commands
        self._poll_hz = poll_hz
        self._rt_last_update: dict[int, float] = {}  # arm_id -> monotonic timestamp
        self._rt_stale_timeout_s = 3.0 / self._poll_hz * 10  # ~10 UDP cycles ≈ 0.25s
        self._rt_recovery_interval_s = 1.0
        self._last_rt_recovery_attempt = 0.0
        self._rt_recovery_attempts = 0
        self._rt_recovery_active = False
        self._rt_lock = threading.Lock()
        self._fault_arms: dict[str, dict] = {}  # arm_name -> 最近一次故障快照
        self._prefix_warned: set[tuple] = set()
        # 串行化 send_cmd 与模式切换; _mode_switching 期间在途命令直接丢弃,
        # 否则软停等待窗口内的残留命令会把在线跟踪重新喂活。
        self._io_lock = threading.RLock()
        self._mode_switching = False

        # ── 指令记录器: 环形缓冲 + 故障时 dump ──
        # 记录最近 2000 条下发指令 (含 tracking targets, move_j, set_online_tracking)
        # 配置 command_dump_dir 时, 故障记录写入该目录
        self._cmd_ring: deque = deque(maxlen=2000)
        self._cmd_ring_lock = threading.Lock()
        self._cmd_dump_dir = Path(command_dump_dir) if command_dump_dir else None
        self._last_dump_time = 0.0  # 防止短时间内反复 dump

    # ── 指令记录器 ─────────────────────────────────────────────────────────────

    def _record_cmd(self, cmd_type: str, arm: str, data: list[float],
                    q_fdb: Optional[list[float]] = None) -> None:
        """记录一条下发指令到环形缓冲。"""
        entry = {
            "ts": time.time(),
            "mono": time.monotonic(),
            "type": cmd_type,
            "arm": arm,
            "data": list(data) if data else [],
            "q_fdb": list(q_fdb) if q_fdb else [],
        }
        with self._cmd_ring_lock:
            self._cmd_ring.append(entry)

    def _dump_cmd_ring(self, reason: str = "fault") -> Optional[str]:
        """把环形缓冲 dump 到 CSV 文件。返回文件路径。"""
        if self._cmd_dump_dir is None:
            return None
        now = time.time()
        # 5 秒内不重复 dump
        if now - self._last_dump_time < 5.0:
            return None
        self._last_dump_time = now

        with self._cmd_ring_lock:
            if not self._cmd_ring:
                return None
            entries = list(self._cmd_ring)

        ts_str = time.strftime("%Y%m%d_%H%M%S", time.localtime(now))
        filename = f"apex_cmd_dump_{ts_str}_{reason}.csv"
        filepath = self._cmd_dump_dir / filename

        try:
            with open(filepath, "w") as f:
                f.write("timestamp,monotonic,type,arm,"
                        "j0,j1,j2,j3,j4,j5,j6,"
                        "fdb_j0,fdb_j1,fdb_j2,fdb_j3,fdb_j4,fdb_j5,fdb_j6\n")
                for e in entries:
                    ts = f"{e['ts']:.6f}"
                    mono = f"{e['mono']:.6f}"
                    data_cols = ",".join(f"{v:.4f}" for v in e["data"][:7]) if e["data"] else ",,,,,,"
                    fdb_cols = ",".join(f"{v:.4f}" for v in e["q_fdb"][:7]) if e["q_fdb"] else ",,,,,,"
                    f.write(f"{ts},{mono},{e['type']},{e['arm']},"
                            f"{data_cols},{fdb_cols}\n")
            log.warning("指令缓冲已 dump 到 %s (%d 条记录, 原因: %s)",
                        filepath, len(entries), reason)
            return str(filepath)
        except Exception as exc:
            log.error("dump 指令缓冲失败: %s", exc)
            return None

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def start(self) -> None:
        from robot_backend_hardware.tianji.apex_sdk import ApexClient, ApexError
        from robot_backend_hardware.tianji.apex_sdk.errors import ConnectionError as ApexConnectionError

        self._check_udp_port_free()

        self._apex_error_types = (ApexError,)
        self._apex_conn_error_types = (OSError, ApexConnectionError)
        self._ApexClient = ApexClient

        try:
            self._client = ApexClient(self._host, timeout=self._connect_timeout_s)
            log.info("ApexRobotActuator [%s] connected to %s", self.name, self._host)
        except Exception as e:
            self._client = None
            log.warning("ApexRobotActuator [%s]: initial connection failed, will retry: %s",
                        self.name, e)
            return

        if self._enable_on_start:
            for arm in self._configured_arms:
                self._client.enable(arm)
                log.info("Apex arm enabled: %s", arm)

        self._attach_rt_listener()

    def stop(self) -> None:
        if self._client is None:
            return
        self._mode_switching = True
        try:
            with self._io_lock:
                self._stop_tracking(wait=True)
                self._motion_state = MotionModeState(
                    MotionMode.TRAJECTORY, CommandSpace.CART
                )
                try:
                    self._client.close()
                except OSError:
                    pass
                finally:
                    self._client = None
                    log.info("ApexRobotActuator [%s] stopped", self.name)
        finally:
            self._mode_switching = False

    def is_healthy(self) -> bool:
        if self._client is None:
            return False
        with self._rt_lock:
            if self._fault_arms:
                return False
            if not self._rt_last_update:
                return True  # no UDP data expected yet (just connected)
            now = time.monotonic()
            for arm_id, last_t in self._rt_last_update.items():
                if now - last_t > self._rt_stale_timeout_s:
                    return False
        return True

    # ── State ─────────────────────────────────────────────────────────────────

    def poll_state(self) -> dict[str, np.ndarray]:
        if self._client is None:
            self._try_reconnect()
            return {}

        realtime = getattr(self._client, "realtime", None)
        if realtime is None:
            self._recover_stale_realtime(set(self._configured_arms))
            return {}

        state: dict[str, np.ndarray] = {}
        arm_configs = [(0, self._left_arm, "leftarm"), (1, self._right_arm, "rightarm")]
        stale_arms: set[str] = set()
        fresh_arms: set[str] = set()

        now = time.monotonic()
        with self._rt_lock:
            timestamps = dict(self._rt_last_update)

        for arm_id, arm_name, default_prefix in arm_configs:
            if not arm_name:
                continue
            msg = realtime.latest_arm_raw(arm_id)
            if msg is None:
                stale_arms.add(arm_name)
                continue
            reported_arm = msg.get("arm")
            effective_arm = (
                reported_arm
                if reported_arm in self._configured_arms
                else arm_name
            )
            last_t = timestamps.get(arm_id)
            if last_t is not None and now - last_t > self._rt_stale_timeout_s:
                log.debug("Stale UDP data for arm %d (%.2fs old), skipping",
                          arm_id, now - last_t)
                stale_arms.add(effective_arm)
                continue

            prefix = self._resolve_prefix(msg, arm_id, arm_name, default_prefix)
            if prefix is None:
                continue

            # 故障归属按报文自带的臂名判定, 不用索引 —— armId 与 left/right 的
            # 对应关系并不保证 (本机 armId 0 实为 arm_right), 标错会导致 clear_faults
            # 去 disable/enable 另一条臂, 故障永远清不掉。
            self._update_fault_state(self._fault_arm_name(msg, arm_name), msg)

            js = msg.get("jointState", {})
            positions = js.get("positions", [])
            if positions:
                state[f"{prefix}_state_joint_pos"] = np.array(positions, dtype=np.float64)
                fresh_arms.add(effective_arm)
            else:
                stale_arms.add(effective_arm)
            torques = js.get("torques", [])
            if torques:
                state[f"{prefix}_state_joint_torque"] = np.array(
                    torques, dtype=np.float64
                )

            pose = msg.get("currentPose", {})
            if pose:
                x = pose.get("x", 0.0)
                y = pose.get("y", 0.0)
                z = pose.get("z", 0.0)
                rx = np.radians(pose.get("rx", 0.0))
                ry = np.radians(pose.get("ry", 0.0))
                rz = np.radians(pose.get("rz", 0.0))
                qx, qy, qz, qw = _euler_xyz_to_quat(rx, ry, rz)
                state[f"{prefix}_state_cart_pos"] = np.array(
                    [x, y, z, qx, qy, qz, qw], dtype=np.float64
                )


        if stale_arms:
            self._recover_stale_realtime(stale_arms)
        elif self._rt_recovery_active and set(self._configured_arms) <= fresh_arms:
            log.info("Apex UDP 状态流已恢复")
            self._rt_recovery_active = False
            self._rt_recovery_attempts = 0

        return state

    def _recover_stale_realtime(self, stale_arms: set[str]) -> None:
        """Restart a stale UDP status stream, then reconnect if it stays stale."""
        now = time.monotonic()
        if now - self._last_rt_recovery_attempt < self._rt_recovery_interval_s:
            return
        if not self._io_lock.acquire(blocking=False):
            return

        try:
            if self._client is None:
                return
            self._last_rt_recovery_attempt = now
            self._rt_recovery_attempts += 1
            self._rt_recovery_active = True
            arms = sorted(stale_arms)

            if self._rt_recovery_attempts == 1:
                log.warning(
                    "Apex UDP 状态流过期 (%s), 正在请求控制器恢复 120Hz 推送",
                    ", ".join(arms),
                )
                try:
                    had_realtime = getattr(self._client, "realtime", None) is not None
                    for arm in arms:
                        self._client.start_realtime(
                            arm,
                            freq_hz=max(1, int(round(self._poll_hz))),
                        )
                    if not had_realtime:
                        self._attach_rt_listener()
                    return
                except Exception as exc:
                    log.warning("恢复 Apex UDP 推送失败，将重连控制器: %s", exc)

            else:
                log.warning(
                    "Apex UDP 状态流仍未恢复 (%s), 正在重连控制器",
                    ", ".join(arms),
                )

            self._disconnect()
            self._try_reconnect()
        finally:
            self._io_lock.release()

    # ── Commands ──────────────────────────────────────────────────────────────

    def send_cmd(self, cmd: dict[str, np.ndarray]) -> None:
        # 模式切换/软停进行中: 丢弃在途命令。若继续下发, TRACKING 会被重新喂活,
        # TRAJECTORY 则会用过期目标触发一次 move_j 大幅运动。
        if self._client is None or self._mode_switching:
            return

        with self._io_lock:
            if self._client is None or self._mode_switching:
                return

            if self._tracking_giveup:
                # 跟踪已闩死: 静默丢弃, 但每秒提醒一次, 免得看起来像卡住
                now = time.monotonic()
                if now - self._giveup_warned_at > 1.0:
                    self._giveup_warned_at = now
                    log.warning("在线跟踪已停用 (驱动器故障未复位), 正在丢弃命令")
                return

            self._cmd_log_counter += 1
            if self._cmd_log_counter % self._cmd_log_interval == 1:
                self._log_cmd(cmd)

            if self._motion_state.mode == MotionMode.ONLINE_TRACKING:
                self._send_tracking_cmd(cmd)
            else:
                self._send_trajectory_cmd(cmd)

    def _log_cmd(self, cmd: dict[str, np.ndarray]) -> None:
        """Log command at reduced frequency for debugging."""
        import time as _time
        mode_name = (
            f"TRACKING/{self._motion_state.command_space.name}"
            if self._motion_state.mode == MotionMode.ONLINE_TRACKING
            else "TRAJ"
        )
        latency_str = ""
        ts_ns = cmd.get("_timestamp_ns")
        if ts_ns:
            latency_ms = (_time.time() * 1e9 - ts_ns) / 1e6
            latency_str = f" latency={latency_ms:.1f}ms"
        parts = []
        for key, val in cmd.items():
            if key.startswith("_"):
                continue
            arr = np.asarray(val, dtype=np.float64).reshape(-1)
            fmt = ", ".join(f"{v:.3f}" for v in arr[:7])
            parts.append(f"  {key}: [{fmt}]")
        log.info("[CMD #%d %s%s]\n%s", self._cmd_log_counter, mode_name, latency_str,
                 "\n".join(parts))

    def _send_trajectory_cmd(self, cmd: dict[str, np.ndarray]) -> None:
        """Execute commands as point-to-point planned motions."""
        if CmdKey.LEFTARM_CMD_JOINT_POS in cmd:
            self._client.move_j(
                self._left_arm,
                self._as_float_list(cmd[CmdKey.LEFTARM_CMD_JOINT_POS]),
                vel=self._command_vel, arm_id=0,
            )
        if CmdKey.RIGHTARM_CMD_JOINT_POS in cmd:
            self._client.move_j(
                self._right_arm,
                self._as_float_list(cmd[CmdKey.RIGHTARM_CMD_JOINT_POS]),
                vel=self._command_vel, arm_id=1,
            )
        if CmdKey.LEFTARM_CMD_CART_POS in cmd:
            self._client.move_l(
                self._left_arm,
                self._array_to_pose(cmd[CmdKey.LEFTARM_CMD_CART_POS]),
                vel=self._command_vel, arm_id=0,
            )
        if CmdKey.RIGHTARM_CMD_CART_POS in cmd:
            self._client.move_l(
                self._right_arm,
                self._array_to_pose(cmd[CmdKey.RIGHTARM_CMD_CART_POS]),
                vel=self._command_vel, arm_id=1,
            )

        unsupported = {
            CmdKey.LEFTARM_GRIPPER_CMD_POS,
            CmdKey.RIGHTARM_GRIPPER_CMD_POS,
        } & set(cmd)
        if unsupported:
            log.warning("ApexRobotActuator ignored unsupported keys: %s", sorted(unsupported))

    def _send_tracking_cmd(self, cmd: dict[str, np.ndarray]) -> None:
        """Stream targets in the configured command space."""
        space = self._motion_state.command_space

        if space == CommandSpace.JOINT:
            invalid = {
                CmdKey.LEFTARM_CMD_CART_POS,
                CmdKey.RIGHTARM_CMD_CART_POS,
            } & set(cmd)
            if invalid:
                raise ValueError(
                    f"joint tracking rejects Cartesian command keys: {sorted(invalid)}"
                )
            # 正常情况下 set_motion_mode 已经 arm 过了; 这里只在控制器把我们踢出
            # 跟踪态 (驱动器故障后 PLANNER PROTECT 强退) 时重新 arm, 且带退避。
            self._ensure_tracking_enabled("joint")
            targets = (
                (self._left_arm, cmd.get(CmdKey.LEFTARM_CMD_JOINT_POS), "left"),
                (self._right_arm, cmd.get(CmdKey.RIGHTARM_CMD_JOINT_POS), "right"),
            )
            for arm, target, side in targets:
                if target is None or not arm:
                    continue
                limited = self._limit_joint_step(arm, self._as_float_list(target))
                try:
                    self._client.update_tracking_joints(arm, limited)
                except Exception as e:
                    # 跟踪流被下位机拒绝 = 该臂已经不在跟踪态。继续闷头发下去只会
                    # 让上层以为动作在执行, 所以撤销 enable 标志走重新 arm 流程。
                    log.warning("update_tracking_joints (%s) failed: %s", side, e)
                    self._tracking_enabled = False
                    self._last_joint_cmd.pop(arm, None)
                    raise
                self._last_joint_cmd[arm] = limited
                # 记录下发指令 (每 10 条附带 q_fdb 快照, 减少 UDP 读取开销)
                fdb = None
                if self._cmd_log_counter % 10 == 0:
                    fdb = self._read_rt_joint_positions(arm)
                self._record_cmd("tracking", arm, limited, q_fdb=fdb)
            return

        invalid = {
            CmdKey.LEFTARM_CMD_JOINT_POS,
            CmdKey.RIGHTARM_CMD_JOINT_POS,
        } & set(cmd)
        if invalid:
            raise ValueError(
                f"Cartesian tracking rejects joint command keys: {sorted(invalid)}"
            )
        self._ensure_tracking_enabled("cartesian")
        mapping = (
            (self._left_arm, CmdKey.LEFTARM_TRACKING_TARGET, CmdKey.LEFTARM_CMD_CART_POS, "left"),
            (self._right_arm, CmdKey.RIGHTARM_TRACKING_TARGET, CmdKey.RIGHTARM_CMD_CART_POS, "right"),
        )
        for arm, tracking_key, cart_key, side in mapping:
            if not arm:
                continue
            target = cmd.get(tracking_key)
            if target is None:
                target = cmd.get(cart_key)
            if target is None:
                continue
            pose = self._array_to_pose(target)
            try:
                self._client.update_tracking_pose(
                    arm,
                    x=pose["x"], y=pose["y"], z=pose["z"],
                    rx=pose["rx"], ry=pose["ry"], rz=pose["rz"],
                    reference_frame=1,
                )
            except Exception as e:
                log.warning("update_tracking_pose (%s) failed: %s", side, e)
                self._tracking_enabled = False
                raise

    def _limit_joint_step(self, arm: str, target: list[float]) -> list[float]:
        """把单条命令的关节增量限制在 _max_joint_step_deg 以内。

        基准是**上一条实际下发的目标**而不是实测角: 在线跟踪是开环喂目标, 用实测角
        做基准会因伺服滞后不断把命令往回拽, 变成低通滤波器。用上一条下发值只削掉
        跳变, 连续轨迹原样通过。
        """
        limit = self._max_joint_step_deg
        if limit <= 0:
            return target
        previous = self._last_joint_cmd.get(arm)
        if previous is None or len(previous) != len(target):
            return target

        limited = list(target)
        max_excess = 0.0
        for i, (prev, want) in enumerate(zip(previous, target)):
            delta = want - prev
            if abs(delta) > limit:
                max_excess = max(max_excess, abs(delta))
                limited[i] = prev + math.copysign(limit, delta)
        if max_excess:
            now = time.monotonic()
            if now - self._joint_clamp_warned_at > 1.0:
                self._joint_clamp_warned_at = now
                log.warning(
                    "%s 关节目标单步跳变 %.2f° 超过限幅 %.2f°, 已按步削平; "
                    "上游轨迹过快或 chunk 边界不连续",
                    arm, max_excess, limit,
                )
        return limited


    # ── Motion mode control ───────────────────────────────────────────────────

    def set_motion_mode(
        self,
        mode: MotionMode,
        command_space: CommandSpace = CommandSpace.CART,
    ) -> None:
        """Configure trajectory or online tracking execution.

        command_space 在这里就已知, 所以在线跟踪是**在模式切换时一次性 arm 完**的。
        不要退回"等首条命令再惰性 enable": 那会把 enable + 0.5s 同步塞进 cmd_hz
        的命令路径里阻塞, 期间上游积压的目标被流水线丢到只剩最后一条, 恢复下发时
        就是一次大跳变 —— 2026-07-23 锁存 J6 跟随误差故障正是这么来的。
        """
        target = MotionModeState(MotionMode(mode), CommandSpace(command_space))
        if target.command_space == CommandSpace.UNSPECIFIED:
            target = MotionModeState(target.mode, CommandSpace.CART)
        if target == self._motion_state and target.mode != MotionMode.ONLINE_TRACKING:
            return

        self._mode_switching = True
        try:
            with self._io_lock:
                client = self._require_client()
                if self._tracking_active:
                    self._stop_tracking(wait=True)

                previous_state = self._motion_state
                self._motion_state = target
                if target.mode != MotionMode.ONLINE_TRACKING:
                    log.info("Switched to TRAJECTORY mode (软停完成)")
                    return

                try:
                    # 带故障锁存进跟踪必然立刻再次报错, 先清障 + 确认使能。
                    self._prepare_for_motion()
                    for arm in self._configured_arms:
                        client.trajectory_stop(arm)
                    # trajectory_stop 只是**请求**停止, 臂还要减速几百 ms。不等停稳
                    # 就 enable 跟踪, 同步阶段读到的实测角会立刻过期, 对齐结束时
                    # q_cmd 已落后 q_fdb 好几度 → 撞 CSP 护栏 → 锁存跟随误差故障。
                    self._wait_arms_settled()

                    self._arm_fail_count = 0
                    self._last_arm_attempt = 0.0
                    self._tracking_giveup = False
                    self._last_joint_cmd.clear()
                    self._ensure_tracking_enabled(_sdk_space(target.command_space))
                except Exception:
                    self._stop_tracking(wait=False)
                    self._motion_state = previous_state
                    raise

                log.info(
                    "Switched motion configuration to %s/%s",
                    target.mode.name, target.command_space.name,
                )
        finally:
            self._mode_switching = False

    def get_motion_mode(self) -> MotionModeState:
        return self._motion_state

    def _wait_arms_settled(self) -> None:
        """等两条臂真正停下来 (关节速度归零) 后再继续。"""
        client = self._client
        if client is None or self._tracking_settle_timeout_s <= 0:
            return
        waiter = getattr(client, "wait_arm_settled", None)
        if waiter is None:
            time.sleep(min(self._tracking_settle_timeout_s, 0.5))
            return
        for arm in self._configured_arms:
            try:
                if not waiter(arm, timeout=self._tracking_settle_timeout_s):
                    log.warning("臂 %s 等待停稳超时 (%.1fs), 仍继续进入在线跟踪",
                                arm, self._tracking_settle_timeout_s)
            except Exception as e:
                log.warning("等待 %s 停稳失败: %s", arm, e)

    def _stop_tracking(self, wait: bool) -> None:
        """软停在线跟踪并等待停稳。

        发 enable=False 后必须等待停稳, 否则指令流中断会触发驱动器保护并锁存
        故障 (0xE001)。状态位在**发出软停之前**就清掉, 这样即便有并发命令挤进来
        也不会走 _send_tracking_cmd 把跟踪重新喂活。
        """
        client = self._client
        if client is None:
            self._tracking_active.clear()
            self._tracking_enabled = False
            self._tracking_mode = None
            self._last_joint_cmd.clear()
            return

        active_arms = [arm for arm, active in self._tracking_active.items() if active]
        # 用**当初 enable 的空间**软停。_tracking_mode 在上一次失败回滚时已被清成
        # None, 拿它兜底会给控制器发 mode="cartesian" 的软停而实际跑的是 joint。
        prev_space = self._tracking_mode or self._last_armed_space or "cartesian"
        self._tracking_active.clear()
        self._tracking_enabled = False
        self._tracking_mode = None
        self._last_joint_cmd.clear()
        if not active_arms:
            return

        for arm in active_arms:
            try:
                client.set_online_tracking(arm, enable=False, mode=prev_space)
            except Exception as e:
                log.warning("Failed to stop tracking for %s: %s", arm, e)
        if not wait:
            return
        # 等待每条臂软停完成 (enable=False 且 softStopping=False)
        for arm in active_arms:
            try:
                if not client.wait_online_tracking_settled(arm, timeout=8.0):
                    log.warning("臂 %s 在线跟踪软停超时", arm)
            except Exception as e:
                log.warning("等待 %s 软停失败: %s", arm, e)

    def _ensure_tracking_enabled(self, space: str) -> None:
        """启用在线跟踪, space = 'joint' | 'cartesian'。

        正常路径由 set_motion_mode 调用一次。命令路径只在控制器把我们踢出跟踪态
        后才会走到这里, 此时必须退避: 命令流水线以 cmd_hz 调用 send_cmd, 直接重试
        就是 120Hz 的 enable 风暴, 每次都让控制器重新初始化 CSP 并重新踩故障。
        """
        if self._tracking_enabled and self._tracking_mode == space:
            return

        now = time.monotonic()
        if self._arm_fail_count and now - self._last_arm_attempt < self._tracking_arm_retry_interval_s:
            # 退避窗口内不碰下位机, 直接让本条命令失败
            raise RuntimeError(
                f"在线跟踪启用失败 {self._arm_fail_count} 次, 退避中 "
                f"({self._tracking_arm_retry_interval_s:.1f}s)"
            )
        self._last_arm_attempt = now

        client = self._require_client()
        # 若之前以别的空间启用过, 先软停并等待停稳再切换空间
        if self._tracking_enabled and self._tracking_mode != space:
            self._stop_tracking(wait=True)

        try:
            self._attempt_enable_tracking(client, space)
        except Exception as error:
            # 同步失败就不能标记为已启用: 否则下一条命令会跳过同步直接流目标,
            # 首帧相对实际位置阶跃 → 跟随误差 → 驱动器锁存故障。回滚后重试。
            self._arm_fail_count += 1
            log.warning("在线跟踪启用失败 (第 %d/%d 次): %s",
                        self._arm_fail_count, self._tracking_arm_max_failures, error)
            self._stop_tracking(wait=False)
            if self._arm_fail_count >= self._tracking_arm_max_failures:
                # 反复启不起来说明是硬件级故障, 继续重试只会把驱动器打得更死。
                # 闩死跟踪流: 后续命令直接丢弃, 直到上层重新 set_motion_mode。
                # 注意不能退回 TRAJECTORY —— 那会把 120Hz 的流当成 move_j 灌下去。
                self._tracking_giveup = True
                log.error(
                    "在线跟踪连续 %d 次启用失败, 已停止喂送跟踪流; "
                    "请复位驱动器故障后重新 set_motion_mode 进入跟踪",
                    self._arm_fail_count,
                )
            raise

        self._arm_fail_count = 0
        self._tracking_enabled = True
        log.info("Online tracking enabled (%s)", space)

    def _attempt_enable_tracking(self, client, space: str) -> None:
        """带内部重试的 tracking 启用流程。

        CSP 对齐完成后首帧可能因 q_cmd/q_fdb 偏差 > 0.05 rad 触发
        0xFF73 故障。本方法最多重试 3 次, 每次失败后:
        1. 停止 tracking
        2. 等待故障自动恢复 (EtherCAT Recovery)
        3. 清障 + 再次 move_j 刷新 q_cmd
        4. 重新 enable tracking
        """
        MAX_INTERNAL_RETRIES = 3
        last_error = None

        for attempt in range(MAX_INTERNAL_RETRIES):
            if attempt > 0:
                log.info("_attempt_enable_tracking: 第 %d 次重试...", attempt + 1)
                # 先完全停止上一次的 tracking 状态
                self._stop_tracking(wait=False)
                # 等待 EtherCAT 驱动器故障恢复 (Recovery 通常需 1-3 秒)
                time.sleep(2.0)
                # 清障
                try:
                    self._prepare_for_motion()
                except Exception as e:
                    log.warning("重试清障失败: %s", e)
                    last_error = e
                    continue
                # 再次刷新 q_cmd
                self._refresh_qcmd_before_tracking(client)

            if attempt == 0:
                # 首次尝试: 完整准备流程
                self._prepare_for_motion()
                self._refresh_qcmd_before_tracking(client)

            try:
                if space == "joint":
                    # ── 预读 q_fdb + enable + 零延迟首帧 ──
                    # 在 enable 之前就缓存好各臂 q_fdb, enable 后瞬间发出,
                    # 消除 "enable → 读取 → 发送" 的延迟 (原本 25-50ms)。
                    # arm_right J6 对这个延迟极其敏感。
                    pre_targets: dict[str, list[float]] = {}
                    for arm in self._configured_arms:
                        rt_pos = self._read_rt_joint_positions(arm)
                        if rt_pos is not None:
                            pre_targets[arm] = rt_pos
                        else:
                            pre_targets[arm] = client.get_joint_positions(
                                arm, timeout=2.0)
                    log.info("enable 前预读 q_fdb: %s",
                             {a: [round(v, 2) for v in p[:7]]
                              for a, p in pre_targets.items()})

                    # 逐臂 enable, enable 后立即发预缓存的 target
                    for arm in self._configured_arms:
                        self._record_cmd("enable_tracking", arm, pre_targets.get(arm, []))
                        client.set_online_tracking(
                            arm, enable=True, mode=space, reference_frame=1)
                        # enable 返回后立即发送! 不做任何其他 IO
                        if arm in pre_targets:
                            client.update_tracking_joints(
                                arm, pre_targets[arm])
                            self._record_cmd("first_target", arm, pre_targets[arm])
                        self._tracking_active[arm] = True

                    # 现在两臂都 enable 且都收到了至少一帧 target,
                    # 继续高频喂实测角确保 CSP 对齐稳定
                    self._sync_joint_tracking_targets(hold_s=1.5)
                else:
                    for arm in self._configured_arms:
                        client.set_online_tracking(
                            arm, enable=True, mode=space, reference_frame=1)
                        self._tracking_active[arm] = True
                self._tracking_mode = space
                self._last_armed_space = space
                # 成功!
                return
            except Exception as e:
                last_error = e
                log.warning(
                    "_attempt_enable_tracking 第 %d/%d 次失败: %s",
                    attempt + 1, MAX_INTERNAL_RETRIES, e)
                # 标记 tracking 未启用
                self._tracking_active.clear()
                self._tracking_mode = None

        # 所有重试都失败
        raise RuntimeError(
            f"在线跟踪内部重试 {MAX_INTERNAL_RETRIES} 次均失败: {last_error}"
        )

    def _read_rt_joint_positions(self, arm: str) -> Optional[list[float]]:
        """从 UDP 实时通道读取指定臂的实测关节角 (EtherCAT q_fdb)。

        与 client.get_joint_positions() 不同, 这里读的是驱动器的实际反馈位置,
        而非规划器/控制器的输出位置。CSP 步长检查 (0.05 rad) 比较的是
        q_cmd vs q_fdb, 因此跟踪同步阶段必须喂 q_fdb 而不是规划器报告的位置。
        """
        raw = self._arm_raw_by_name(arm)
        if raw is None:
            return None
        js = raw.get("jointState", {})
        positions = js.get("positions")
        if positions and len(positions) > 0:
            return list(positions)
        return None

    def _sync_joint_tracking_targets(self, hold_s: float = 1.5, freq_hz: float = 100.0) -> None:
        """关节 tracking 启用后, 立即用当前实测关节角持续喂目标, 确保平滑启动。

        关键行为: set_online_tracking(enable=True) 之后必须立即开始发送 tracking
        目标, 不能等待! 控制器的 CSP 对齐过程需要收到 target 作为参考——如果在
        对齐期间不发任何 target, 对齐结束后 q_cmd 会落到某个旧值, 与 q_fdb
        偏差超 0.05 rad 就触发 0xFF73 故障。

        正确做法 (参照 SDK sync_online_tracking_joints):
        1. enable 后立即以高频 (100Hz) 持续发送当前 q_fdb 作为 target
        2. 持续 hold_s 秒确保控制器内部状态完全稳定
        3. 期间周期性刷新实测角, 跟踪臂的微小漂移

        优先使用 UDP 实时通道的 q_fdb (真实伺服反馈)。
        """
        client = self._require_client()
        interval = 1.0 / max(freq_hz, 1.0)

        # ── 立即开始喂实测角, 不等待! ──
        # 控制器 CSP 对齐需要 target 作为参考。对齐期间发送的 q_fdb 确保:
        # 对齐结束时 q_cmd ≈ target ≈ q_fdb → 偏差 ≈ 0 → 不触发故障。
        started = time.monotonic()
        targets: dict[str, list[float]] = {}
        refresh_interval = 0.3  # 每 0.3s 重新读一次实测角
        last_refresh = 0.0

        while time.monotonic() - started < hold_s:
            elapsed = time.monotonic() - started
            # 每 refresh_interval 刷新一次实测角, 跟踪臂的微小漂移
            should_refresh = (elapsed - last_refresh) >= refresh_interval or not targets
            for arm in self._configured_arms:
                if should_refresh:
                    rt_pos = self._read_rt_joint_positions(arm)
                    if rt_pos is not None:
                        targets[arm] = rt_pos
                    else:
                        targets[arm] = client.get_joint_positions(arm, timeout=2.0)
                if arm in targets:
                    client.update_tracking_joints(arm, targets[arm])
                    self._record_cmd("sync", arm, targets[arm])
            if should_refresh:
                last_refresh = elapsed
            time.sleep(interval)

        # 限幅基准 = 同步结束时的实测角, 之后第一条真实命令的跳变才会被削平
        self._last_joint_cmd = dict(targets)
        log.info("关节 tracking 已同步到当前实测关节角 (持续 %.1fs @%.0fHz)", hold_s, freq_hz)

    # ── Fault handling ────────────────────────────────────────────────────────

    def _attach_rt_listener(self) -> None:
        """在 SDK 自带的 UDP 通道上挂监听, 只记录到达时间用于判 stale。

        注意: 不要另开 socket 抢 UDP 9101 并把 client._rt_channel 置空 ——
        那会让 SDK 的 get_state / get_joint_positions / wait_motion_done 全部失效。
        """
        realtime = getattr(self._client, "realtime", None)
        if realtime is None:
            log.warning("Apex SDK 未启动 UDP 通道, 状态读取不可用")
            return
        realtime.add_listener(self._on_rt_frame)

    def _on_rt_frame(self, raw: dict) -> None:
        with self._rt_lock:
            self._rt_last_update[int(raw.get("armId", 0))] = time.monotonic()

    def _fault_arm_name(self, msg: dict, fallback: str) -> str:
        """从 UDP 报文解析该帧属于哪条臂; 报文没带 'arm' 字段时退回索引推断。"""
        reported = msg.get("arm")
        if isinstance(reported, str) and reported in self._configured_arms:
            return reported
        return fallback

    def _arm_raw_by_name(self, arm_name: str) -> Optional[dict]:
        """按臂名取最新 UDP 报文, 优先匹配报文自带的 'arm' 字段。"""
        realtime = getattr(self._client, "realtime", None)
        if realtime is None:
            return None
        for arm_id in (0, 1):
            raw = realtime.latest_arm_raw(arm_id)
            if raw is not None and raw.get("arm") == arm_name:
                return raw
        return self._arm_raw(arm_name)  # 报文无 'arm' 字段时退回索引

    def _arm_id_for(self, arm_name: str) -> int:
        """臂名 → UDP armId, 同样优先按报文自带的 'arm' 字段匹配。"""
        realtime = getattr(self._client, "realtime", None)
        if realtime is not None:
            for arm_id in (0, 1):
                raw = realtime.latest_arm_raw(arm_id)
                if raw is not None and raw.get("arm") == arm_name:
                    return arm_id
        return 0 if arm_name == self._left_arm else 1

    def _wait_fresh_fault_state(self, arm: str, timeout: float = 2.5) -> Optional[dict]:
        """等该臂一帧**新**的 UDP 状态后重新判定故障; 返回 None 表示已无故障。

        不能只依赖轮询线程刷新的 _fault_arms: disable 期间控制器可能暂停状态推送,
        poll_state 会因 stale 跳过该臂, 标志位就一直粘在故障态 —— 于是「其实已清除」
        被误判成「清不掉」。
        """
        arm_id = self._arm_id_for(arm)
        with self._rt_lock:
            baseline = self._rt_last_update.get(arm_id)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._rt_lock:
                stamp = self._rt_last_update.get(arm_id)
            if stamp is not None and stamp != baseline:
                raw = self._arm_raw_by_name(arm)
                if raw is not None:
                    self._update_fault_state(arm, raw)
                    return self.faulted_arms().get(arm)
            time.sleep(0.05)
        log.warning("等待 %s 新状态帧超时 (%.1fs), 无法确认故障是否已清除", arm, timeout)
        return self.faulted_arms().get(arm)

    @staticmethod
    def _fault_summary(faulted: dict) -> str:
        return "; ".join(
            f"{arm}(robotState={info.get('robot_state')}, estop={info.get('estop')}, "
            f"jointErrors={info.get('errors') or '无'})"
            for arm, info in sorted(faulted.items())
        )

    def _update_fault_state(self, arm_name: str, msg: dict) -> None:
        """根据 UDP status-push 维护每条臂的故障锁存状态。"""
        robot_state = int(msg.get("robotState", 0) or 0)
        estop = bool(msg.get("estopLatched")) or robot_state == ROBOT_STATE_ESTOP
        faulted = bool(msg.get("faultLatched")) or robot_state == ROBOT_STATE_FAULT or estop
        errors = [(i, e) for i, e in
                  enumerate(msg.get("jointState", {}).get("errorCodes", []) or []) if e]
        snapshot = {"robot_state": robot_state, "estop": estop, "errors": errors}

        with self._rt_lock:
            was = arm_name in self._fault_arms
            if faulted:
                self._fault_arms[arm_name] = snapshot
            else:
                self._fault_arms.pop(arm_name, None)

        if faulted and not was:
            log.error(
                "Apex 臂 %s 进入故障态: robotState=%d estopLatched=%s jointErrors=%s",
                arm_name, robot_state, estop, errors or "无",
            )
            # 故障瞬间 dump 指令缓冲, 便于复盘
            self._dump_cmd_ring(reason=f"{arm_name}_J{errors[0][0]+1 if errors else 'X'}")
        elif was and not faulted:
            log.info("Apex 臂 %s 故障已解除", arm_name)

    def faulted_arms(self) -> dict[str, dict]:
        with self._rt_lock:
            return dict(self._fault_arms)

    def clear_faults(self, retries: int = 3) -> bool:
        """用 disable→enable 循环清除驱动器锁存故障。

        急停锁存 (estopLatched) 属于硬件级, 软件清不掉, 必须物理复位急停按钮。
        """
        client = self._require_client()
        faulted = self.faulted_arms()
        if not faulted:
            return True

        ok = True
        for arm, info in sorted(faulted.items()):
            if info.get("estop"):
                log.error("臂 %s 急停锁存, 软件无法清除, 请先物理复位急停按钮", arm)
                ok = False
                continue
            if not self._clear_arm_fault(client, arm, retries):
                ok = False
        return ok

    def _clear_arm_fault(self, client, arm: str, retries: int) -> bool:
        last_error = None
        for attempt in range(1, retries + 1):
            log.warning("清除 %s 驱动器故障 (第 %d/%d 次): disable → enable",
                        arm, attempt, retries)
            try:
                client.disable(arm)
            except Exception as e:
                log.debug("disable(%s) 异常(可忽略): %s", arm, e)
            time.sleep(1.0)
            try:
                client.enable(arm)
            except Exception as e:
                last_error = e
                log.warning("enable(%s) 失败: %s", arm, e)
            # 主动等一帧新状态再判定, 不依赖轮询线程的标志位
            info = self._wait_fresh_fault_state(arm)
            if info is None:
                log.info("臂 %s 故障已清除", arm)
                return True
            log.warning("臂 %s 仍处故障: robotState=%s estop=%s jointErrors=%s",
                        arm, info.get("robot_state"), info.get("estop"),
                        info.get("errors") or "无")
            if info.get("estop"):
                log.error("臂 %s 为急停锁存, 软件无法清除, 不再重试", arm)
                return False
        log.error("臂 %s 故障清除失败%s; 多为硬件级(碰撞/限位/超程), 需人工处理",
                  arm, f" (最后一次 enable 报错: {last_error})" if last_error else "")
        return False

    def _prepare_for_motion(self) -> None:
        """进入在线跟踪前: 清除锁存故障 + 确认两臂已使能。"""
        faulted = self.faulted_arms()
        if faulted:
            if not self._auto_clear_fault:
                raise RuntimeError(
                    f"Apex 驱动器故障锁存: {self._fault_summary(faulted)}; "
                    f"auto_clear_fault=false, 请先人工清除"
                )
            if not self.clear_faults():
                remaining = self.faulted_arms() or faulted
                raise RuntimeError(
                    f"Apex 驱动器故障未能清除: {self._fault_summary(remaining)}。"
                    "estop=True 需物理复位急停按钮; 否则多为碰撞/限位/超程等硬件级"
                    "故障, 请在示教器上确认并复位后重试。"
                )

        client = self._require_client()
        for arm in self._configured_arms:
            raw = self._arm_raw_by_name(arm)
            # robotState 0=未连接 1=空闲 → 尚未使能; 已使能时不重复 enable, 避免抱闸抖动
            if raw is not None and int(raw.get("robotState", 0) or 0) in (0, 1):
                try:
                    client.enable(arm)
                    log.info("Apex arm enabled: %s", arm)
                except Exception as e:
                    log.warning("enable(%s) 失败: %s", arm, e)

    def _refresh_qcmd_before_tracking(self, client) -> None:
        """在进入 online tracking 前, 逐臂串行发原地 move_j 刷新控制器 q_cmd。

        Apex 控制器的 planner 是全局互锁的——任一臂在执行 trajectory 时,
        另一臂的 trajectory_exec 会被拒绝 (planner busy)。因此必须串行:
        对每条臂单独 move_j → wait_motion_done, 完成后再处理下一条。

        这确保每条臂的 planner q_cmd 都被刷新到当前 q_fdb, 避免进入
        CSP 跟踪时首帧超过 0.05 rad 限制触发 0xFF73 故障。
        """
        # 先确保所有臂已 settled (使能 + 速度归零)
        for arm in self._configured_arms:
            try:
                client.wait_arm_settled(arm, timeout=3.0, vel_eps_deg=1.0)
            except Exception as e:
                log.warning("_refresh_qcmd: wait_arm_settled(%s) 异常: %s", arm, e)

        # ── 逐臂串行刷新 (两轮) ──
        for round_idx in range(2):
            round_label = f"round{round_idx + 1}"
            for arm_idx, arm in enumerate(self._configured_arms):
                try:
                    cur_pos = client.get_joint_positions(arm, timeout=2.0)
                    if not cur_pos or len(cur_pos) == 0:
                        log.warning("_refresh_qcmd %s: %s 读取关节角为空", round_label, arm)
                        continue
                    client.move_j(arm, cur_pos, vel=0.05,
                                  max_velocity=10.0, max_acceleration=10.0,
                                  arm_id=arm_idx)
                    self._record_cmd("move_j_refresh", arm, cur_pos)
                    # 等待 trajectory 完成 + 关节速度稳定归零
                    # wait_arm_settled 需要连续 5 次采样均 < 0.3°/s, 比
                    # wait_motion_done (3次 < 0.5°/s) 更严格, 防止大惯量
                    # 关节 (J3) 减速末端被误判为"已停"。
                    client.wait_arm_settled(arm, timeout=5.0, vel_eps_deg=0.3)
                    log.info("_refresh_qcmd %s: %s (arm_id=%d) 完成", round_label, arm, arm_idx)
                except Exception as e:
                    log.warning("_refresh_qcmd %s: %s move_j/wait fail: %s", round_label, arm, e)

            if round_idx == 0:
                # 第一轮后等伺服收敛, 大惯量关节可能还有残余运动
                time.sleep(0.5)

        time.sleep(0.3)
        # ── 诊断: 打印各轴 q_fdb, 重点关注 arm_right J6 ──
        for arm_idx, arm in enumerate(self._configured_arms):
            try:
                rt_pos = self._read_rt_joint_positions(arm)
                tcp_pos = client.get_joint_positions(arm, timeout=2.0)
                if rt_pos and tcp_pos:
                    diffs = [abs(r - t) for r, t in zip(rt_pos, tcp_pos)]
                    max_diff = max(diffs)
                    max_j = diffs.index(max_diff)
                    log.info("_refresh_qcmd diag %s: max|rt-tcp|=%.4f° @J%d "
                             "rt_J6=%.3f tcp_J6=%.3f",
                             arm, max_diff, max_j + 1,
                             rt_pos[5] if len(rt_pos) > 5 else 0,
                             tcp_pos[5] if len(tcp_pos) > 5 else 0)
            except Exception as e:
                log.warning("_refresh_qcmd diag %s fail: %s", arm, e)
        log.info("_refresh_qcmd: 两轮串行刷新完成, 双臂 q_cmd 已对齐 q_fdb")

    def _arm_raw(self, arm_name: str) -> Optional[dict]:
        realtime = getattr(self._client, "realtime", None)
        if realtime is None:
            return None
        arm_id = 0 if arm_name == self._left_arm else 1
        return realtime.latest_arm_raw(arm_id)

    # ── Scripted motion ───────────────────────────────────────────────────────

    def move_to_pose(
        self,
        joint_positions: dict[str, list[float]] | None = None,
        cart_positions: dict[str, Any] | None = None,
        vel: float = 0.3,
        block: bool = True,
    ) -> None:
        """Move arms to target joint angles or Cartesian end-effector poses.

        Temporarily exits ONLINE_TRACKING if active, executes the motion,
        then restores the previous mode if needed. Each arm accepts either a
        joint target or a Cartesian target, but not both at once.

        Args:
            joint_positions: {"left": [7 floats], "right": [7 floats]} angles in degrees.
            cart_positions: {"left": [x,y,z,qx,qy,qz,qw], "right": [...]}.
            vel: velocity ratio 0.0-1.0.
            block: if True, wait until motion completes.
        """
        joint_positions = joint_positions or {}
        cart_positions = cart_positions or {}
        conflict = set(joint_positions) & set(cart_positions)
        if conflict:
            raise ValueError(
                f"move_to_pose received both joint and Cartesian targets for: "
                f"{sorted(conflict)}"
            )

        # 整个 homing 期间视为模式切换: 丢弃在途命令, 避免 move_j 与残留
        # tracking 目标互相打架。
        self._mode_switching = True
        try:
            with self._io_lock:
                client = self._require_client()

                previous_state = self._motion_state
                was_tracking = previous_state.mode == MotionMode.ONLINE_TRACKING
                if was_tracking:
                    # 等待软停完成再执行 move_j, 避免 tracking 与轨迹指令冲突触发故障
                    self._stop_tracking(wait=True)
                    self._motion_state = MotionModeState(
                        MotionMode.TRAJECTORY, previous_state.command_space
                    )

                # 带故障锁存时 move_j 会被驱动器直接拒绝, 先清障 + 确认使能
                self._prepare_for_motion()

                arms = {"left": (self._left_arm, 0), "right": (self._right_arm, 1)}
                for side, (arm, arm_id) in arms.items():
                    if arm and side in joint_positions:
                        client.move_j(arm, joint_positions[side], vel=vel, arm_id=arm_id)
                    if arm and side in cart_positions:
                        client.move_l(
                            arm, self._array_to_pose(cart_positions[side]),
                            vel=vel, arm_id=arm_id,
                        )

                if block:
                    if not client.wait_motion_done():
                        log.warning("move_to_pose: 等待运动完成超时, 机械臂可能未到位")

                if was_tracking:
                    # move_j 刚结束时臂仍在减速, 必须等停稳再 enable, 否则同步阶段
                    # 采到的实测角立刻过期 → CSP 对齐结束即超差锁存故障。
                    self._wait_arms_settled()
                    self._arm_fail_count = 0
                    self._last_arm_attempt = 0.0
                    self._tracking_giveup = False
                    self._last_joint_cmd.clear()
                    self._motion_state = previous_state
                    self._ensure_tracking_enabled(
                        _sdk_space(previous_state.command_space)
                    )
        finally:
            self._mode_switching = False

    def get_features(self) -> dict[str, dict[str, Any]]:
        return {
            StateKey.LEFTARM_STATE_JOINT_POS: {"dtype": "float64", "shape": [7]},
            StateKey.LEFTARM_STATE_CART_POS: {"dtype": "float64", "shape": [7]},
            StateKey.LEFTARM_STATE_JOINT_TORQUE: {"dtype": "float64", "shape": [7]},
            StateKey.RIGHTARM_STATE_JOINT_POS: {"dtype": "float64", "shape": [7]},
            StateKey.RIGHTARM_STATE_CART_POS: {"dtype": "float64", "shape": [7]},
            StateKey.RIGHTARM_STATE_JOINT_TORQUE: {"dtype": "float64", "shape": [7]},
            CmdKey.LEFTARM_CMD_CART_POS: {"dtype": "float64", "shape": [7]},
            CmdKey.LEFTARM_CMD_JOINT_POS: {"dtype": "float64", "shape": [7]},
            CmdKey.RIGHTARM_CMD_CART_POS: {"dtype": "float64", "shape": [7]},
            CmdKey.RIGHTARM_CMD_JOINT_POS: {"dtype": "float64", "shape": [7]},
            CmdKey.LEFTARM_TRACKING_TARGET: {"dtype": "float64", "shape": [7]},
            CmdKey.RIGHTARM_TRACKING_TARGET: {"dtype": "float64", "shape": [7]},
        }

    # ── Private helpers ──────────────────────────────────────────────────────

    @property
    def _configured_arms(self) -> list[str]:
        return [arm for arm in (self._left_arm, self._right_arm) if arm]


    def _check_udp_port_free(self) -> None:
        """确认没有别的进程已占用状态推送端口 UDP 9101。

        SDK 用 SO_REUSEADDR 绑定该端口, 两个进程 (典型: 本 server 与 ROS2 的
        apex_arm_controller) 能同时绑上同一端口, 内核只把每个数据报投递给其中
        一个 socket —— 状态推送被瓜分, 表现为随机丢状态、跟随异常。这里用一次
        **不带 SO_REUSEADDR** 的探测绑定把冲突检出并直接拒绝启动。
        """
        from robot_backend_hardware.tianji.apex_sdk.protocol import UDP_PORT

        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.bind(("0.0.0.0", UDP_PORT))
        except OSError as e:
            raise RuntimeError(
                f"UDP {UDP_PORT} 已被占用 ({e}); Apex 状态推送端口不能与其他进程共享。"
                "请关闭使用该控制器的其他驱动进程后重试。"
            ) from e
        finally:
            probe.close()


    def _require_client(self):
        if self._client is None:
            raise RuntimeError("ApexRobotActuator is not connected")
        return self._client

    def _disconnect(self) -> None:
        if self._client is not None:
            try:
                self._client.close()
            except OSError:
                pass
            self._client = None

    def _try_reconnect(self) -> None:
        now = time.monotonic()
        if now - self._last_reconnect_attempt < self._reconnect_interval_s:
            return
        self._last_reconnect_attempt = now
        try:
            self._client = self._ApexClient(self._host, timeout=self._connect_timeout_s)
            log.info("ApexRobotActuator [%s] reconnected to %s", self.name, self._host)
            with self._rt_lock:
                self._rt_last_update.clear()
                self._fault_arms.clear()
            if self._enable_on_start:
                for arm in self._configured_arms:
                    self._client.enable(arm)
            self._attach_rt_listener()
        except Exception as e:
            self._client = None
            log.warning("ApexRobotActuator reconnect failed: %s", e)

    def _arm_name_from_index(self, idx: int) -> str:
        if idx == 0:
            return self._left_arm
        if idx == 1:
            return self._right_arm
        return ""

    def _resolve_prefix(
        self, msg: dict, arm_id: int, arm_name: str, default_prefix: str
    ) -> Optional[str]:
        """Decide leftarm/rightarm prefix from the UDP frame's own arm name.

        Trusts the reported "arm" field over the armId index convention so a
        swapped controller arm ordering cannot silently mislabel feedback.
        """
        reported = msg.get("arm", "")
        if not reported:
            return default_prefix
        if reported == self._left_arm:
            resolved = "leftarm"
        elif reported == self._right_arm:
            resolved = "rightarm"
        else:
            if ("unknown", arm_id, reported) not in self._prefix_warned:
                self._prefix_warned.add(("unknown", arm_id, reported))
                log.warning(
                    "poll_state: armId=%d reported unknown arm name %r "
                    "(configured left=%r right=%r); dropping frame",
                    arm_id, reported, self._left_arm, self._right_arm,
                )
            return None
        if resolved != default_prefix:
            if ("mismatch", arm_id, reported) not in self._prefix_warned:
                self._prefix_warned.add(("mismatch", arm_id, reported))
                log.warning(
                    "poll_state: armId=%d name %r maps to %s but index convention "
                    "expected %s; using name",
                    arm_id, reported, resolved, default_prefix,
                )
        return resolved

    def _prefix_for_arm(self, arm_name: str, idx: int) -> Optional[str]:
        if arm_name == self._left_arm or idx == 0:
            return "leftarm"
        if arm_name == self._right_arm or idx == 1:
            return "rightarm"
        return None

    @staticmethod
    def _pose_to_array(pose: dict) -> Optional[np.ndarray]:
        """Convert Apex pose dict {x,y,z,rx,ry,rz} (degrees) to [x,y,z,qx,qy,qz,qw]."""
        if not pose:
            return None
        x = pose.get("x", 0.0)
        y = pose.get("y", 0.0)
        z = pose.get("z", 0.0)
        rx = np.radians(pose.get("rx", 0.0))
        ry = np.radians(pose.get("ry", 0.0))
        rz = np.radians(pose.get("rz", 0.0))
        qx, qy, qz, qw = _euler_xyz_to_quat(rx, ry, rz)
        return np.array([x, y, z, qx, qy, qz, qw], dtype=np.float64)

    @staticmethod
    def _array_to_pose(arr) -> dict:
        """Convert [x,y,z,qx,qy,qz,qw] to Apex pose dict {x,y,z,rx,ry,rz} (degrees)."""
        values = np.asarray(arr, dtype=np.float64).reshape(-1)
        if values.size != 7:
            raise ValueError(f"Cartesian command expects shape [7], got {values.shape}")
        x, y, z, qx, qy, qz, qw = values
        rx, ry, rz = _quat_to_euler_xyz(qx, qy, qz, qw)
        return {
            "x": float(x), "y": float(y), "z": float(z),
            "rx": float(np.degrees(rx)),
            "ry": float(np.degrees(ry)),
            "rz": float(np.degrees(rz)),
        }

    @staticmethod
    def _as_float_list(arr) -> list[float]:
        return [float(x) for x in np.asarray(arr, dtype=np.float64).reshape(-1)]
