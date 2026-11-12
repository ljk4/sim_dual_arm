# robot_api_msgs

双臂机器人公共 ROS 2 接口包：

- **[`msg/`](msg/README.md)**：机器人状态、相机帧、观测索引、流式命令、跟随参考、执行反馈消息。
- **[`srv/`](srv/README.md)**：控制租约、Episode 生命周期、流式命令查询、受限跟随提交服务。

[`CMakeLists.txt`](CMakeLists.txt) 是接口编译与安装的唯一入口。修改接口定义后，须在 `runtime` 容器内重新运行 `runtime/ros2/scripts/run.sh build-ros-sdk`。

## 版本与能力

当前包版本 **0.3.0**（[`package.xml`](package.xml)），能力标识 **`execution_feedback.gripper_set.v1`**：

- `ExecutionFeedback.grippers` 为 `GripperFeedback[<=8]`，`GripperFeedback.fingers` 为 `FingerFeedback[<=8]`；夹爪允许 0～8 个，每个夹爪允许 1～8 个手指。Panda 的“恰好左右双指”规则在 `simulation` 实现侧，不在公共消息中。
- `FollowerReference.curve_duration_ns` 是生产者完整曲线的总时长（`uint64`）。0.3.0 只把该字段从 `ruckig_duration_ns` 改名，CDR 类型、顺序和边界未变；字段关系与校验以 [`robot_contracts`](../../../python/contracts/src/robot_contracts/trajectory/follower_reference.py) 为准。
- 版本演进与能力协商细节见 [阶段 07 交接](../../../../../docs/architecture-tasks/shared-scenarios/deliveries/phase-07.md) 与 [阶段 08 交接](../../../../../docs/architecture-tasks/shared-scenarios/deliveries/phase-08.md)。

历史录制中的 0.1.0 固定数组反馈布局由 runtime 的专用 legacy 解码读取；旧轨迹字段按已知改名识别并使用 v1 摘要载荷校验后重建为当前记录。两者都不把旧 payload 交给当前生成类型，完整历史 episode 的导出与审计见 [阶段 12 交接](../../../../../docs/architecture-tasks/shared-scenarios/deliveries/phase-12.md)。

目录导航：[上级目录](../README.md)
