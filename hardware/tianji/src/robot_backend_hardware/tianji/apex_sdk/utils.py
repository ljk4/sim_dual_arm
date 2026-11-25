"""Apex SDK 工具函数"""

import time
from typing import List


def timestamp_us() -> int:
    """获取当前微秒时间戳"""
    return int(time.time() * 1_000_000)


def clamp(value: float, min_val: float, max_val: float) -> float:
    """限幅"""
    return max(min_val, min(max_val, value))


def clamp_vel_scale(scale: float) -> float:
    """限制速度比例到 [0.0, 1.0]"""
    return clamp(scale, 0.0, 1.0)


def validate_joint_target(joints: List[float], dof: int = 7) -> List[float]:
    """验证关节目标维度"""
    if len(joints) != dof:
        raise ValueError(f"关节目标维度错误: 期望 {dof}, 实际 {len(joints)}")
    return list(joints)
