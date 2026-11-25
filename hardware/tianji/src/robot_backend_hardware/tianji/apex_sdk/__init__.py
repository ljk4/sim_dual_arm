"""Apex SDK - 机械臂控制客户端"""

from .client import ApexClient
from .realtime import RealtimeChannel
from .types import (
    RtState, RtCommand, JointState, CartesianPose,
    ArmState, TrajectoryProgress, TrajectorySegment,
)
from .errors import ApexError, ConnectionError, CommandError, TimeoutError, RealtimeError
from .protocol import (
    TCP_PORT, UDP_PORT,
    MODE_POSITION, MODE_VELOCITY, MODE_TORQUE,
    ROBOT_STATE_DISCONNECTED, ROBOT_STATE_IDLE, ROBOT_STATE_ENABLED,
    ROBOT_STATE_TEACHING, ROBOT_STATE_TRAJECTORY, ROBOT_STATE_PAUSED,
    ROBOT_STATE_ESTOP, ROBOT_STATE_FAULT,
)

__version__ = "0.1.0"

__all__ = [
    "ApexClient",
    "RealtimeChannel",
    "RtState",
    "RtCommand",
    "JointState",
    "CartesianPose",
    "ArmState",
    "TrajectoryProgress",
    "TrajectorySegment",
    "ApexError",
    "ConnectionError",
    "CommandError",
    "TimeoutError",
    "RealtimeError",
    "TCP_PORT",
    "UDP_PORT",
    "MODE_POSITION",
    "MODE_VELOCITY",
    "MODE_TORQUE",
]
