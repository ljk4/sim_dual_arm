"""Named, bounded QoS profiles shared by public ROS adapters."""

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

INFO_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
SENSOR_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
EVENT_QOS = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)

# Task recording: bounded reliable history, shared explicitly by both endpoints.
RELIABLE_SENSOR_QOS = QoSProfile(depth=32, reliability=ReliabilityPolicy.RELIABLE)
