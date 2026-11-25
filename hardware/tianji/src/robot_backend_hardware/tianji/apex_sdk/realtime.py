"""Apex SDK UDP 实时通道（JSON status-push）"""

import json
import socket
import threading
import time
from typing import Callable, Dict, Iterator, List, Optional, Sequence

from .protocol import UDP_PORT, RT_DEFAULT_FREQ_HZ
from .types import RtState
from .errors import RealtimeError


class RealtimeChannel:
    """UDP 实时状态接收通道（JSON 格式）

    服务端 connect / start_realtime 后按频率向客户端 :9101 单播 status-push。
    本通道绑定本地 UDP 端口，缓存每臂最新一帧，供 get_state 等状态 API 使用。
    """

    def __init__(self, host: str, port: int = UDP_PORT, arm_id: int = 0,
                 mode: int = 0, freq_hz: int = RT_DEFAULT_FREQ_HZ):
        self._host = host
        self._port = port
        self._arm_id = arm_id
        self._freq_hz = freq_hz
        self._running = False
        self._sock: Optional[socket.socket] = None
        self._latest_state: Optional[RtState] = None
        self._by_arm: Dict[int, dict] = {}
        self._latest_raw: Optional[dict] = None
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._recv_thread: Optional[threading.Thread] = None
        self._callbacks: List[Callable[[dict], None]] = []

    def start(self):
        """启动实时通道（监听 UDP 9101 接收数据）"""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            self._sock.bind(("0.0.0.0", self._port))
        except OSError as e:
            raise RealtimeError(
                f"无法绑定 UDP 端口 {self._port}，请确认客户端正在监听: {e}"
            ) from e
        self._sock.settimeout(0.1)
        self._running = True

        self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._recv_thread.start()

    def stop(self):
        """停止实时通道"""
        self._running = False
        with self._cv:
            self._cv.notify_all()
        if self._recv_thread is not None:
            self._recv_thread.join(timeout=1.0)
            self._recv_thread = None
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def latest_state(self) -> Optional[RtState]:
        """获取最新状态帧（非阻塞，优先当前 arm_id）"""
        with self._lock:
            preferred = self._by_arm.get(self._arm_id)
            if preferred is not None:
                return self._raw_to_rt_state(preferred)
            return self._latest_state

    def latest_arm_raw(self, arm_id: Optional[int] = None) -> Optional[dict]:
        """获取指定臂最新原始 status-push 字典"""
        with self._lock:
            if arm_id is None:
                return dict(self._latest_raw) if self._latest_raw else None
            raw = self._by_arm.get(arm_id)
            return dict(raw) if raw else None

    def arm_ids(self) -> List[int]:
        with self._lock:
            return sorted(self._by_arm.keys())

    def wait_ready(self, timeout: float = 2.0, min_arms: int = 1) -> bool:
        """等待至少收到 min_arms 臂的状态"""
        deadline = time.time() + max(0.0, timeout)
        with self._cv:
            while len(self._by_arm) < min_arms:
                remaining = deadline - time.time()
                if remaining <= 0:
                    return False
                self._cv.wait(timeout=remaining)
            return True

    def build_state(self, arm_names: Optional[Sequence[str]] = None,
                    arm: str = "") -> dict:
        """组装与历史 get_state 兼容的状态字典（来自 UDP 缓存）"""
        with self._lock:
            by_arm = {k: dict(v) for k, v in self._by_arm.items()}
            latest = dict(self._latest_raw) if self._latest_raw else {}

        names = list(arm_names or [])
        arms_out: List[dict] = []

        for idx in sorted(by_arm):
            raw = self._normalize_arm_json(by_arm[idx], idx, names)
            if not arm or raw.get("arm") == arm:
                arms_out.append(raw)

        seq = latest.get("seq", 0)
        ts = latest.get("timestamp_us", 0)
        return {
            "ok": True,
            "seq": seq,
            "timestamp_us": ts,
            "arms": arms_out,
            # UDP status-push 不含 trajectoryProgress；保留空数组以兼容旧解析逻辑
            "trajectoryProgress": [],
            "source": "udp",
        }

    def add_listener(self, callback: Callable[[dict], None]) -> None:
        with self._lock:
            self._callbacks.append(callback)

    def remove_listener(self, callback: Callable[[dict], None]) -> None:
        with self._lock:
            self._callbacks = [c for c in self._callbacks if c is not callback]

    def recv_state(self, timeout: float = 0.1) -> Optional[RtState]:
        """阻塞接收一帧状态（带超时）；优先读缓存"""
        if self.wait_ready(timeout=timeout, min_arms=1):
            return self.latest_state
        return None

    def __iter__(self) -> Iterator[RtState]:
        """迭代器模式，持续接收状态帧"""
        while self._running:
            state = self.recv_state(timeout=0.05)
            if state is not None:
                yield state

    def _recv_loop(self):
        """后台接收线程，持续更新 latest_state / per-arm 缓存"""
        while self._running:
            try:
                if self._sock is None:
                    break
                self._sock.settimeout(0.05)
                data, _ = self._sock.recvfrom(65536)
                raw, rt = self._parse_json(data)
                if raw is None:
                    continue
                callbacks: List[Callable[[dict], None]] = []
                with self._cv:
                    arm_id = int(raw.get("armId", 0))
                    self._by_arm[arm_id] = raw
                    self._latest_raw = raw
                    self._latest_state = rt
                    callbacks = list(self._callbacks)
                    self._cv.notify_all()
                for cb in callbacks:
                    try:
                        cb(raw)
                    except Exception:
                        pass
            except socket.timeout:
                continue
            except OSError:
                break

    @staticmethod
    def _normalize_arm_json(raw: dict, arm_idx: int,
                            arm_names: Sequence[str]) -> dict:
        out = dict(raw)
        out.pop("type", None)
        out.pop("udp_ip", None)
        out.pop("freq_hz", None)
        out.pop("seq", None)
        out.pop("timestamp_us", None)
        out["armId"] = arm_idx
        if "arm" not in out and arm_idx < len(arm_names):
            out["arm"] = arm_names[arm_idx]
        return out

    @classmethod
    def _raw_to_rt_state(cls, msg: dict) -> RtState:
        js = msg.get("jointState", {})
        pose = msg.get("currentPose", {})
        pose_tool = msg.get("currentPoseTool", {})
        positions = list(js.get("positions", []) or [])
        return RtState(
            sequence=msg.get("seq", 0),
            timestamp_us=msg.get("timestamp_us", 0),
            arm_id=msg.get("armId", 0),
            dof=len(positions),
            robot_state=msg.get("robotState", 0),
            control_mode=0,
            error_code=0,
            q=positions,
            dq=list(js.get("velocities", []) or []),
            tau=list(js.get("torques", []) or []),
            tcp_pose=[
                pose.get("x", 0), pose.get("y", 0), pose.get("z", 0),
                pose.get("rx", 0), pose.get("ry", 0), pose.get("rz", 0),
            ],
            tcp_pose_tool=[
                pose_tool.get("x", 0), pose_tool.get("y", 0), pose_tool.get("z", 0),
                pose_tool.get("rx", 0), pose_tool.get("ry", 0), pose_tool.get("rz", 0),
            ],
            external_wrench=[0, 0, 0, 0, 0, 0],
        )

    @classmethod
    def _parse_json(cls, data: bytes):
        """解析 JSON 状态帧 -> (raw_dict, RtState)"""
        try:
            msg = json.loads(data.decode("utf-8").strip())
        except (json.JSONDecodeError, UnicodeDecodeError):
            return None, None

        if msg.get("type") != "status-push":
            return None, None

        return msg, cls._raw_to_rt_state(msg)
