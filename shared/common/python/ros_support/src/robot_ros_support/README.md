# robot_ros_support

ROS 2 消息与契约 DTO 的双向转换与通信配置：

- **`conversions.py`**：机器人状态（`RobotState`）、相机帧（`CameraFrame`）、观测索引与运行上下文的双向转换。
- **`stream_conversions.py`**：流式控制命令（`StreamCommand`）、块状态（`StreamChunkStatus`）、执行反馈（`ExecutionFeedback`）与控制事件的双向转换。
- **`follower_conversions.py`**：受限跟随参考（`FollowerReference`）、锚点（`ReferenceAnchor`）与样条分段（`ReferenceSegment`）的双向转换。
- **`qos.py`**：相机流、关节状态流、控制反馈流与服务调用的显式 QoS 配置。

所有转换函数均执行严格字段映射，不静默忽略未知字段或填充隐式默认值。

目录导航：[上级目录](../README.md)
