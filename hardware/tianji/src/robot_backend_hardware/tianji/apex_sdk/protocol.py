"""Apex SDK 协议常量与帧解析/序列化"""

import struct
from typing import Tuple

# TCP 命令通道
TCP_PORT = 9100

# UDP 实时通道
UDP_PORT = 9101
RT_DEFAULT_FREQ_HZ = 120

# 帧魔数
RT_STATE_MAGIC = 0xAE01
RT_COMMAND_MAGIC = 0xAE02

# 协议版本
PROTOCOL_VERSION = 1

# 自由度
DOF = 7

# 安全超时
RT_TIMEOUT_MS = 100

# struct 格式（小端）
# RtStatePacket: magic(H) version(H) seq(I) ts(Q) arm_id(B) dof(B) robot_state(B) control_mode(B) error_code(I) q[7](7d) dq[7](7d) tau[7](7d) tcp_pose[6](6d) wrench[6](6d)
RT_STATE_FMT = "<HHIQBBBBI7d7d7d6d6d"
RT_STATE_SIZE = struct.calcsize(RT_STATE_FMT)  # 288 bytes

# RtCommandPacket: magic(H) version(H) seq(I) ts(Q) arm_id(B) dof(B) mode(B) reserved(B) target_q[7](7d) target_dq[7](7d) target_tau[7](7d)
RT_COMMAND_FMT = "<HHIQBBBB7d7d7d"
RT_COMMAND_SIZE = struct.calcsize(RT_COMMAND_FMT)  # 188 bytes

# 控制模式
MODE_POSITION = 0
MODE_VELOCITY = 1
MODE_TORQUE = 2

# 机器人状态枚举
ROBOT_STATE_DISCONNECTED = 0
ROBOT_STATE_IDLE = 1
ROBOT_STATE_ENABLED = 2
ROBOT_STATE_TEACHING = 3
ROBOT_STATE_TRAJECTORY = 4
ROBOT_STATE_PAUSED = 5
ROBOT_STATE_ESTOP = 6
ROBOT_STATE_FAULT = 7


def pack_rt_command(sequence: int, timestamp_us: int, arm_id: int, mode: int,
                    target_q: Tuple[float, ...],
                    target_dq: Tuple[float, ...],
                    target_tau: Tuple[float, ...]) -> bytes:
    """序列化 RtCommandPacket"""
    return struct.pack(
        RT_COMMAND_FMT,
        RT_COMMAND_MAGIC, PROTOCOL_VERSION, sequence, timestamp_us,
        arm_id, DOF, mode, 0,
        *target_q, *target_dq, *target_tau
    )


def unpack_rt_state(data: bytes) -> dict:
    """解析 RtStatePacket，返回字段字典"""
    values = struct.unpack(RT_STATE_FMT, data)
    idx = 0
    result = {
        "magic": values[idx],
        "version": values[idx + 1],
        "sequence": values[idx + 2],
        "timestamp_us": values[idx + 3],
        "arm_id": values[idx + 4],
        "dof": values[idx + 5],
        "robot_state": values[idx + 6],
        "control_mode": values[idx + 7],
        "error_code": values[idx + 8],
    }
    idx = 9
    result["q"] = list(values[idx:idx + 7])
    idx += 7
    result["dq"] = list(values[idx:idx + 7])
    idx += 7
    result["tau"] = list(values[idx:idx + 7])
    idx += 7
    result["tcp_pose"] = list(values[idx:idx + 6])
    idx += 6
    result["external_wrench"] = list(values[idx:idx + 6])
    return result
