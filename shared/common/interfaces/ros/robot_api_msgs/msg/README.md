# ROS 2 消息定义（msg）

按职责划分为三组：

- **观测与上下文**：`RobotState`、`CameraFrame`、`ObservationIndex`、`RunContext`、`BackendInfo`、`RuntimeStatus`。
- **流式控制与反馈**：`StreamCommand`、`StreamChunkStatus`、`ExecutionFeedback`、`ControlEvent`、`GripperFeedback`、`FingerFeedback`。
  `ExecutionFeedback.grippers` 与 `GripperFeedback.fingers` 是有界命名集合（分别为 0～8 和 1～8），`side` 是唯一非空标识，不再限定为 left/right 或固定顺序；Marvin 的双侧双指规则由方案实现检查。该容器变化对应能力标识 `execution_feedback.gripper_set.v1`。
- **受限跟随参考**：`FollowerReference`、`ReferenceAnchor`、`ReferenceSegment`。

目录导航：[上级目录](../README.md)
