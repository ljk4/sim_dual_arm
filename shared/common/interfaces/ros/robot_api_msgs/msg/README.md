# ROS 2 消息定义（msg）

按职责划分为三组：

- **观测与上下文**：`RobotState`、`CameraFrame`、`ObservationIndex`、`RunContext`、`BackendInfo`、`RuntimeStatus`。
- **流式控制与反馈**：`StreamCommand`、`StreamChunkStatus`、`ExecutionFeedback`、`ControlEvent`、`GripperFeedback`、`FingerFeedback`。
  `execution_feedback.gripper_set` 表示后端提供具名夹爪和手指反馈。每条反馈包含 0～8 个夹爪，每个夹爪包含 1～8 个手指，`side` 是唯一非空标识。Marvin 的双侧双指规则由方案实现检查。
- **受限跟随参考**：`FollowerReference`、`ReferenceAnchor`、`ReferenceSegment`。

目录导航：[上级目录](../README.md)
