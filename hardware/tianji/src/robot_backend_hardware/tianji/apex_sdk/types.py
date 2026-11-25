"""Apex SDK 数据结构定义"""

from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class RtState:
    """UDP 实时状态帧"""
    sequence: int = 0
    timestamp_us: int = 0
    arm_id: int = 0
    dof: int = 7
    robot_state: int = 0
    control_mode: int = 0
    error_code: int = 0
    q: List[float] = field(default_factory=lambda: [0.0] * 7)
    dq: List[float] = field(default_factory=lambda: [0.0] * 7)
    tau: List[float] = field(default_factory=lambda: [0.0] * 7)
    tcp_pose: List[float] = field(default_factory=lambda: [0.0] * 6)
    tcp_pose_tool: List[float] = field(default_factory=lambda: [0.0] * 6)
    external_wrench: List[float] = field(default_factory=lambda: [0.0] * 6)


@dataclass
class RtCommand:
    """UDP 实时指令帧"""
    arm_id: int = 0
    mode: int = 0
    target_q: List[float] = field(default_factory=lambda: [0.0] * 7)
    target_dq: List[float] = field(default_factory=lambda: [0.0] * 7)
    target_tau: List[float] = field(default_factory=lambda: [0.0] * 7)


@dataclass
class JointState:
    """关节状态"""
    positions: List[float] = field(default_factory=lambda: [0.0] * 7)
    velocities: List[float] = field(default_factory=lambda: [0.0] * 7)
    torques: List[float] = field(default_factory=lambda: [0.0] * 7)


@dataclass
class CartesianPose:
    """笛卡尔位姿"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0


@dataclass
class ArmState:
    """单臂状态"""
    arm_id: int = 0
    robot_state: int = 0
    joint_state: Optional[JointState] = None
    current_pose: Optional[CartesianPose] = None
    current_pose_tool: Optional[CartesianPose] = None


@dataclass
class TrajectoryProgress:
    """轨迹进度"""
    arm_id: int = 0
    is_finished: bool = False
    total_segments: int = 0
    current_segment: int = 0
    progress: int = 0  # 0–100 整数百分比


@dataclass
class TrajectorySegment:
    """轨迹段定义"""
    arm_id: int = 0
    motion_type: int = 0
    velocity_ratio: float = 0.3
    max_velocity: float = 60.0
    max_acceleration: float = 40.0
    blend_radius: float = 0.0
    joint_position_target: Optional[List[float]] = None
    cartesian_target: Optional[CartesianPose] = None
    auxiliary_pose: Optional[CartesianPose] = None

    def to_dict(self) -> dict:
        """转换为协议 JSON 格式"""
        d = {
            "armId": self.arm_id,
            "motionType": self.motion_type,
            "velocityRatio": self.velocity_ratio,
            "maxVelocity": self.max_velocity,
            "maxAcceleration": self.max_acceleration,
            "blendRadius": self.blend_radius,
        }
        if self.joint_position_target is not None:
            d["jointPositionTarget"] = self.joint_position_target
        if self.cartesian_target is not None:
            ct = self.cartesian_target
            d["cartesianTarget"] = {
                "x": ct.x, "y": ct.y, "z": ct.z,
                "rx": ct.rx, "ry": ct.ry, "rz": ct.rz,
            }
        if self.auxiliary_pose is not None:
            ap = self.auxiliary_pose
            d["auxiliaryPose"] = {
                "x": ap.x, "y": ap.y, "z": ap.z,
                "rx": ap.rx, "ry": ap.ry, "rz": ap.rz,
            }
        return d
