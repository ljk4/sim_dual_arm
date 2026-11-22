# robot_api_msgs

双臂机器人公共 ROS 2 接口包：

- **[`msg/`](msg/README.md)**：机器人状态、相机帧、观测索引、流式命令、跟随参考、执行反馈消息。
- **[`srv/`](srv/README.md)**：控制租约、Episode 生命周期、流式命令查询、受限跟随提交服务。

[`CMakeLists.txt`](CMakeLists.txt) 是接口编译与安装的唯一入口。修改接口定义后，须在 `runtime` 容器内重新运行 `scripts/build_sdk.sh`。

## 后端能力

后端在 `BackendInfo` 中声明 `bounded_target_follower`、`execution_feedback.gripper_set` 等能力及标称执行网格周期 `control_grid_ns`，并在时间戳字段中统一使用 `*_backend_time_ns`；Runtime 启动时核对所需能力与执行网格。

- `ExecutionFeedback.grippers` 为 `GripperFeedback[<=8]`，`GripperFeedback.fingers` 为 `FingerFeedback[<=8]`；夹爪允许 0～8 个，每个夹爪允许 1～8 个手指。Marvin 的“恰好左右双指”规则在 `simulation` 实现侧，不在公共消息中。
- `SubmitFollower` 接收原始窗口及 Runtime 生成的 `ReferenceSegment` 系数。后端校验后广播已接受的 `FollowerReference`。字段关系与校验见 [`robot_contracts`](../../../python/contracts/src/robot_contracts/trajectory/follower_reference.py)。

离线读取时[核对定义](../../../../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/export/follower_definition.py)，按记录的 CDR 布局解码并校验生效参考的摘要。使用方式见[离线导出与审计](../../../../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/README.md)。

目录导航：[上级目录](../README.md)
