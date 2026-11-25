"""Apex SDK TCP 命令通道客户端"""

import json
import logging
import socket
import threading
import time
from typing import List, Optional, Callable

from .errors import ConnectionError, CommandError, TimeoutError
from .protocol import TCP_PORT, UDP_PORT, RT_DEFAULT_FREQ_HZ
from .realtime import RealtimeChannel
from .types import (
    ArmState, JointState, CartesianPose,
    TrajectoryProgress, TrajectorySegment,
)
from .utils import clamp_vel_scale, validate_joint_target

logger = logging.getLogger(__name__)


class ApexClient:
    """Apex 机械臂 TCP 命令通道客户端"""

    def __init__(self, host: str, port: int = TCP_PORT, timeout: float = 10.0):
        self._host = host
        self._port = port
        self._timeout = timeout
        self._sock: Optional[socket.socket] = None
        self._file = None
        self._seq = 0
        self._lock = threading.Lock()
        self._arms: List[str] = []
        self._dof: int = 7
        self._version: int = 0
        self._sub_thread: Optional[threading.Thread] = None
        self._sub_active = False
        self._rt_channel: Optional[RealtimeChannel] = None

        self._connect()

    # ------------------------------------------------------------------
    # 连接管理
    # ------------------------------------------------------------------

    def _connect(self):
        """建立 TCP 连接并发送 connect 命令"""
        try:
            self._sock = socket.create_connection(
                (self._host, self._port), timeout=self._timeout
            )
            self._file = self._sock.makefile("r")
        except OSError as e:
            raise ConnectionError(f"无法连接到 {self._host}:{self._port}: {e}")

        # 先绑定 UDP 9101，再 connect（避免服务端开始推送时客户端尚未监听）
        client_ip = self._sock.getsockname()[0]
        self._start_rt_channel(RT_DEFAULT_FREQ_HZ)

        resp = self._send("connect", udp_ip=client_ip)
        self._arms = resp.get("arms", [])
        self._dof = resp.get("dof", 7)
        self._version = resp.get("version", 1)

        # 等待首帧 UDP，后续 get_state 不再走 TCP
        if self._rt_channel is not None:
            if not self._rt_channel.wait_ready(timeout=2.0, min_arms=1):
                logger.warning("connect 后尚未收到 UDP 状态推送，get_state 可能短暂超时")

    def _start_rt_channel(self, freq_hz: int = RT_DEFAULT_FREQ_HZ):
        """启动本地 UDP 接收（须在 connect 之前调用，监听 :9101）"""
        if self._rt_channel is not None:
            self._rt_channel.stop()
        arm_id = 0
        self._rt_channel = RealtimeChannel(
            host=self._host, port=UDP_PORT, arm_id=arm_id, freq_hz=freq_hz
        )
        self._rt_channel.start()

    @property
    def arms(self) -> List[str]:
        """可用臂列表"""
        return self._arms

    @property
    def dof(self) -> int:
        """关节自由度"""
        return self._dof

    @property
    def realtime(self) -> Optional[RealtimeChannel]:
        """UDP 状态通道（connect 后自动启动）"""
        return self._rt_channel

    def close(self):
        """关闭连接"""
        self._sub_active = False
        if self._sub_thread is not None:
            self._sub_thread.join(timeout=2.0)
            self._sub_thread = None
        if self._sock is not None:
            logger.info("主动关闭 TCP 连接 %s:%s", self._host, self._port)
        if self._rt_channel is not None:
            self._rt_channel.stop()
            self._rt_channel = None
        if self._sock is not None:
            self._sock.close()
            self._sock = None
            self._file = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    # ------------------------------------------------------------------
    # 底层通信
    # ------------------------------------------------------------------

    def _next_id(self) -> int:
        self._seq += 1
        return self._seq

    def _send(self, cmd: str, **kwargs) -> dict:
        """发送命令并等待响应"""
        if self._sock is None:
            raise ConnectionError("连接已断开")

        msg = {"id": self._next_id(), "cmd": cmd, **kwargs}
        payload = json.dumps(msg, ensure_ascii=False) + "\n"

        with self._lock:
            try:
                self._sock.sendall(payload.encode())
            except OSError as e:
                logger.warning(
                    "TCP 发送失败 %s:%s: %s (errno=%s)",
                    self._host, self._port, e, getattr(e, "errno", None),
                )
                raise ConnectionError(f"发送失败: {e}") from e
            line = self._file.readline()

        if not line:
            logger.warning(
                "TCP 连接断开 %s:%s: reason=peer_close (对端关闭连接)",
                self._host, self._port,
            )
            raise ConnectionError("连接已断开")

        resp = json.loads(line)
        if not resp.get("ok", False):
            raise CommandError(
                resp.get("msg", "unknown error"),
                code=resp.get("err", 0),
            )
        return resp

    # ------------------------------------------------------------------
    # 会话命令
    # ------------------------------------------------------------------

    def heartbeat(self):
        """心跳保活"""
        self._send("heartbeat")

    def acquire_control(self, arm: str):
        """获取控制权"""
        self._send("acquire_control", arm=arm)

    def release_control(self, arm: str):
        """释放控制权"""
        self._send("release_control", arm=arm)

    # ------------------------------------------------------------------
    # 使能与模式
    # ------------------------------------------------------------------

    def enable(self, arm: str):
        """使能机械臂"""
        self._send("enable", arm=arm)

    def disable(self, arm: str):
        """下使能机械臂"""
        self._send("disable", arm=arm)

    def emergency_stop(self):
        """急停（所有臂）"""
        self._send("emergency_stop")

    def set_robot_mode(self, arm: str, mode: int):
        """设置控制模式 (0=位置, 1=力矩)"""
        self._send("set_robot_mode", arm=arm, mode=mode)

    def get_robot_mode(self, arm: str) -> dict:
        """查询控制模式"""
        resp = self._send("get_robot_mode", arm=arm)
        return resp.get("data", {})

    def set_vel_scale(self, arm: str, scale: float):
        """设置全局速度比例 (0.0-1.0)"""
        self._send("set_vel_scale", arm=arm, scale=clamp_vel_scale(scale))

    def get_vel_scale(self, arm: str) -> dict:
        """查询速度比例"""
        resp = self._send("get_vel_scale", arm=arm)
        return resp.get("data", {})

    # ------------------------------------------------------------------
    # 示教
    # ------------------------------------------------------------------

    def teaching_start(self, arm: str, teaching_mode: int = 0,
                       joint_axis: int = 0, direction: int = 1,
                       speed_ratio: float = 0.3, is_step: bool = False,
                       step_value: float = 0.0, **kwargs):
        """开始示教"""
        params = {
            "arm": arm,
            "teachingMode": teaching_mode,
            "jointAxis": joint_axis,
            "direction": direction,
            "speedRatio": speed_ratio,
            "isStep": is_step,
            "stepValue": step_value,
        }
        params.update(kwargs)
        self._send("teaching_start", **params)

    def teaching_stop(self, arm: str):
        """停止示教"""
        self._send("teaching_stop", arm=arm)

    # ------------------------------------------------------------------
    # 轨迹运动
    # ------------------------------------------------------------------

    def trajectory_exec(self, arm: str, segments: List[dict]):
        """执行轨迹运动（原始 segments 字典列表）"""
        self._send("trajectory_exec", arm=arm, segments=segments)

    def move_j(self, arm: str, target_joints: List[float],
               vel: float = 0.3, max_velocity: float = 60.0,
               max_acceleration: float = 40.0, blend_radius: float = 0.0,
               arm_id: int = 0):
        """关节空间运动 (MoveJ)"""
        validate_joint_target(target_joints, self._dof)
        seg = {
            "armId": arm_id,
            "motionType": 0,
            "jointPositionTarget": list(target_joints),
            "velocityRatio": clamp_vel_scale(vel),
            "maxVelocity": max_velocity,
            "maxAcceleration": max_acceleration,
            "blendRadius": blend_radius,
        }
        self._send("trajectory_exec", arm=arm, segments=[seg])

    def move_l(self, arm: str, pose: dict, vel: float = 0.3,
               max_velocity: float = 0.5, max_acceleration: float = 1.0,
               blend_radius: float = 0.0, arm_id: int = 0):
        """笛卡尔直线运动 (MoveL)

        pose: {"x", "y", "z", "rx", "ry", "rz"}
        """
        seg = {
            "armId": arm_id,
            "motionType": 1,
            "cartesianTarget": pose,
            "velocityRatio": clamp_vel_scale(vel),
            "maxVelocity": max_velocity,
            "maxAcceleration": max_acceleration,
            "blendRadius": blend_radius,
        }
        self._send("trajectory_exec", arm=arm, segments=[seg])

    def move_c(self, arm: str, target_pose: dict, aux_pose: dict,
               vel: float = 0.3, arm_id: int = 0):
        """圆弧运动 (MoveC)"""
        seg = {
            "armId": arm_id,
            "motionType": 2,
            "cartesianTarget": target_pose,
            "auxiliaryPose": aux_pose,
            "velocityRatio": clamp_vel_scale(vel),
            "maxVelocity": 0.5,
            "maxAcceleration": 1.0,
            "blendRadius": 0.0,
        }
        self._send("trajectory_exec", arm=arm, segments=[seg])

    def trajectory_pause(self, arm: str):
        """暂停轨迹"""
        self._send("trajectory_pause", arm=arm)

    def trajectory_resume(self, arm: str):
        """恢复轨迹"""
        self._send("trajectory_resume", arm=arm)

    def trajectory_stop(self, arm: str):
        """停止轨迹"""
        self._send("trajectory_stop", arm=arm)

    # ------------------------------------------------------------------
    # 状态查询（全部基于 UDP status-push，不再走 TCP get_state）
    # ------------------------------------------------------------------

    def get_state(self, arm: str = "", timeout: float = 1.0) -> dict:
        """查询状态（UDP 缓存组装，兼容原 get_state 返回结构）

        Args:
            arm: 非空时只返回该臂；空则返回已收到的全部臂
            timeout: 若缓存尚无数据，最长等待秒数
        """
        if self._rt_channel is None:
            raise ConnectionError("UDP 状态通道未启动")

        min_arms = 1
        if not arm and self._arms:
            min_arms = len(self._arms)

        if not self._rt_channel.wait_ready(timeout=timeout, min_arms=1):
            raise TimeoutError("UDP 状态尚未就绪")

        # 多臂时尽量等齐；超时则返回当前已有缓存
        if min_arms > 1:
            self._rt_channel.wait_ready(timeout=min(timeout, 0.2), min_arms=min_arms)

        return self._rt_channel.build_state(self._arms, arm=arm)

    def wait_motion_done(self, timeout: float = 30.0, poll_interval: float = 0.05,
                         vel_eps_deg: float = 0.5) -> bool:
        """等待所有臂运动完成（基于 UDP 关节速度判定）

        UDP 推送不含 trajectoryProgress，故以关节速度接近零并连续稳定为判据。
        """
        start = time.time()
        time.sleep(0.2)
        stable_count = 0

        while time.time() - start < timeout:
            try:
                resp = self.get_state(timeout=0.5)
            except (TimeoutError, ConnectionError):
                time.sleep(poll_interval)
                continue

            arms_data = resp.get("arms", [])
            if not arms_data:
                time.sleep(poll_interval)
                continue

            all_stopped = True
            for a in arms_data:
                vels = a.get("jointState", {}).get("velocities", [])
                if vels and max(abs(v) for v in vels) > vel_eps_deg:
                    all_stopped = False
                    break
                # robotState=4 表示轨迹执行中（若服务端 UDP 将来填充）
                if int(a.get("robotState", 0)) == 4:
                    all_stopped = False
                    break

            if all_stopped:
                stable_count += 1
                if stable_count >= 3:
                    return True
            else:
                stable_count = 0

            time.sleep(poll_interval)

        return False

    def get_joint_positions(self, arm: str, timeout: float = 1.0) -> List[float]:
        """从 UDP 状态读取指定臂当前关节角（度）"""
        state = self.get_state(arm=arm, timeout=timeout)
        arms_data = state.get("arms", [])
        if not arms_data:
            raise TimeoutError(f"UDP 无臂 {arm} 的状态")
        positions = list(arms_data[0].get("jointState", {}).get("positions", []))
        if len(positions) != self._dof:
            raise TimeoutError(f"UDP 无臂 {arm} 的完整关节位置")
        return positions

    # ------------------------------------------------------------------
    # 状态订阅（UDP）
    # ------------------------------------------------------------------

    def subscribe_state(self, callback: Callable[[dict], None], freq: int = 10):
        """订阅状态推送（基于本地 UDP 缓存，按 freq 回调）

        callback 参数为与 get_state 相同结构的字典（含 arms 列表）。
        """
        freq = max(1, min(int(freq), 100))
        self._sub_active = False
        if self._sub_thread is not None:
            self._sub_thread.join(timeout=2.0)
            self._sub_thread = None

        self._sub_active = True
        self._sub_thread = threading.Thread(
            target=self._state_push_loop, args=(callback, freq), daemon=True
        )
        self._sub_thread.start()

    def unsubscribe_state(self):
        """取消状态订阅"""
        self._sub_active = False
        if self._sub_thread is not None:
            self._sub_thread.join(timeout=2.0)
            self._sub_thread = None

    def _state_push_loop(self, callback: Callable[[dict], None], freq: int):
        """按频率从 UDP 缓存组装状态并回调"""
        interval = 1.0 / freq
        while self._sub_active:
            t0 = time.time()
            try:
                if self._rt_channel is not None and self._rt_channel.arm_ids():
                    callback(self._rt_channel.build_state(self._arms))
            except Exception:
                logger.exception("subscribe_state callback failed")
            sleep = interval - (time.time() - t0)
            if sleep > 0:
                time.sleep(sleep)

    # ------------------------------------------------------------------
    # 实时控制
    # ------------------------------------------------------------------

    def start_realtime(self, arm: str, freq_hz: int = RT_DEFAULT_FREQ_HZ, client_ip: str = "") -> RealtimeChannel:
        """调整 UDP 推送参数（connect 时已自动激活，一般无需再调）

        可用于更改频率或指定 client_ip；未指定 IP 时使用 TCP 连接源地址。
        本地 UDP 监听在 connect 时已启动，状态 API 依赖该通道，勿随意 stop。
        """
        kwargs = {"arm": arm, "freq": freq_hz}
        if client_ip:
            kwargs["udp_ip"] = client_ip
        self._send("start_realtime", **kwargs)

        if self._rt_channel is None:
            self._start_rt_channel(freq_hz)
        return self._rt_channel

    def stop_realtime(self, arm: str = ""):
        """通知服务端停止对该臂的 UDP 推送

        注意：本地 UDP 监听保持运行，以保证 get_state 等仍可读缓存；
        全部断开请调用 close()。
        """
        if arm:
            self._send("stop_realtime", arm=arm)
        elif self._arms:
            self._send("stop_realtime", arm=self._arms[0])
    # ------------------------------------------------------------------
    # 运动学计算
    # ------------------------------------------------------------------

    def forward_kinematics(self, arm: str, joint_positions: List[float]) -> dict:
        """正向运动学：关节角度 → 末端位姿

        Args:
            arm: 目标臂名称
            joint_positions: 关节角度列表 (度)

        Returns:
            {"pose": {"x", "y", "z", "rx", "ry", "rz"}}
            位置单位 m，姿态单位 度
        """
        resp = self._send("forward_kinematics", arm=arm, joint_positions=list(joint_positions))
        return resp.get("data", {})

    def inverse_kinematics(self, arm: str, pose: dict,
                           seed: Optional[List[float]] = None) -> dict:
        """逆向运动学：末端位姿 → 关节角度

        Args:
            arm: 目标臂名称
            pose: 目标位姿 {"x", "y", "z", "rx", "ry", "rz"}
                  位置单位 m，姿态单位 度
            seed: 初始关节角 (度)，不传则使用当前关节位置

        Returns:
            {"joint_positions": [...], "solver_return_code": 0}
            关节角度单位 度
        """
        kwargs = {"arm": arm, "pose": pose}
        if seed is not None:
            kwargs["seed"] = list(seed)
        resp = self._send("inverse_kinematics", **kwargs)
        return resp.get("data", {})

    # ------------------------------------------------------------------
    # 在线跟踪 (Online Tracking)
    # ------------------------------------------------------------------

    def set_online_tracking(self, arm: str, enable: bool,
                            mode: str = "cartesian",
                            reference_frame: int = 1) -> dict:
        """开启/关闭在线跟踪

        Args:
            arm: 臂名称
            enable: True=开启, False=软停后关闭（需继续等待停稳）
            mode: "cartesian" 笛卡尔位姿跟踪；"joint" 关节空间稀疏目标流式平滑
            reference_frame: 笛卡尔参考系（World=1），关节模式可忽略
        """
        mode_norm = mode.strip().lower()
        if mode_norm in ("joint", "jointspace", "joint_space"):
            mode_payload = "joint"
        else:
            mode_payload = "cartesian"
        return self._send(
            "set_online_tracking",
            arm=arm,
            enable=enable,
            mode=mode_payload,
            referenceFrame=reference_frame,
        )

    def get_online_tracking_state(self, arm: str) -> dict:
        """查询在线跟踪状态

        Returns:
            {"enable": bool, "mode": 0|1, "softStopping": bool}
            mode: 0=Cartesian, 1=JointSpace
        """
        resp = self._send("get_online_tracking_state", arm=arm)
        return resp.get("data", {})

    def update_tracking_pose(self, arm: str, x: float, y: float, z: float,
                             rx: float = 0.0, ry: float = 0.0, rz: float = 0.0,
                             reference_frame: int = 1) -> dict:
        """更新笛卡尔在线跟踪目标（位置 m，姿态 deg）"""
        return self._send(
            "update_tracking_target",
            arm=arm,
            x=x, y=y, z=z, rx=rx, ry=ry, rz=rz,
            referenceFrame=reference_frame,
        )

    def update_tracking_joints(self, arm: str, joint_positions: List[float]) -> dict:
        """更新关节空间在线跟踪目标（度）

        需先 set_online_tracking(..., mode="joint")。稀疏更新即可（如 10–100Hz）。
        """
        validate_joint_target(joint_positions, self._dof)
        return self._send(
            "update_tracking_target",
            arm=arm,
            jointPositionTarget=list(joint_positions),
        )

    def wait_online_tracking_settled(self, arm: str, timeout: float = 10.0,
                                     poll_interval: float = 0.05) -> bool:
        """等待在线跟踪软停完成（enable=False 且 softStopping=False）"""
        start = time.time()
        while time.time() - start < timeout:
            try:
                st = self.get_online_tracking_state(arm)
            except CommandError:
                time.sleep(poll_interval)
                continue
            if not st.get("enable", False) and not st.get("softStopping", False):
                return True
            time.sleep(poll_interval)
        return False

    def wait_arm_settled(self, arm: str, timeout: float = 10.0,
                         vel_eps_deg: float = 1.0,
                         require_enabled: bool = True) -> bool:
        """等待臂使能且关节速度接近零（基于 UDP）

        robotState: 2=已使能；轨迹中(4)视为未稳定。
        """
        start = time.time()
        stable = 0
        while time.time() - start < timeout:
            try:
                st = self.get_state(arm=arm, timeout=0.5)
            except (TimeoutError, ConnectionError):
                time.sleep(0.05)
                continue
            arms = st.get("arms", [])
            if not arms:
                time.sleep(0.05)
                continue
            a = arms[0]
            rs = int(a.get("robotState", -1))
            if require_enabled and rs not in (2, 3, 4, 5):
                stable = 0
                time.sleep(0.05)
                continue
            if rs == 4:
                stable = 0
                time.sleep(0.05)
                continue
            vels = a.get("jointState", {}).get("velocities", [])
            moving = bool(vels) and max(abs(v) for v in vels) > vel_eps_deg
            if moving:
                stable = 0
            else:
                stable += 1
                if stable >= 5:
                    return True
            time.sleep(0.05)
        return False

    def sync_online_tracking_joints(self, arm: str, hold_s: float = 1.0,
                                    freq_hz: float = 50.0) -> List[float]:
        """关节空间 OT 开启后，用当前反馈关节做同步保持再发运动目标

        必须在 set_online_tracking(..., mode=\"joint\") 之后调用：
        重新读取 UDP 关节角，并按 hold_s 持续 update_tracking_joints，
        避免首帧相对控制器 init 有阶跃导致跟随误差掉电。
        """
        q = self.get_joint_positions(arm, timeout=2.0)
        validate_joint_target(q, self._dof)
        dt = 1.0 / max(freq_hz, 1.0)
        t0 = time.time()
        # 开环保持 0.3s 后刷新一次反馈，消除 enable/OT 过程中的微小漂移
        refreshed = False
        while time.time() - t0 < hold_s:
            self.update_tracking_joints(arm, q)
            if not refreshed and (time.time() - t0) >= 0.3:
                try:
                    q = self.get_joint_positions(arm, timeout=0.3)
                    validate_joint_target(q, self._dof)
                    refreshed = True
                except (TimeoutError, ValueError, CommandError):
                    pass
            time.sleep(dt)
        return list(q)
