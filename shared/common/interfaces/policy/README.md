# Policy 协议定义

[`v1/policy.proto`](v1/policy.proto) 是 gRPC Policy 推理服务的唯一协议源码。

- **SDK 生成**：由 `runtime` 容器内的 `runtime/ros2/scripts/run.sh build-policy-sdk`（调用 [`shared/common/tools/scripts/generate_policy_sdk.py`](../../tools/scripts/generate_policy_sdk.py)）生成 Python Protobuf/gRPC 代码至 `build/generated/policy-sdk/`。
- **使用边界**：严禁手动修改生成的 SDK 文件；ROS 2 机器人接口位于 [`interfaces/ros/robot_api_msgs/`](../ros/robot_api_msgs/README.md)，与 Policy SDK 相互独立。

详细字段语义见 [v1 说明](v1/README.md)。

目录导航：[上级目录](../../README.md)
