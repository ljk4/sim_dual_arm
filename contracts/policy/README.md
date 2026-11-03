# Policy 协议

`v1/policy.proto` 是唯一 Policy 协议源码。生成的 Python Protobuf/gRPC SDK 由
`tools/contracts/generate_policy_sdk.py` 输出到构建目录。ROS 接口另由
`contracts/ros/robot_api_msgs/` 构建；两者不能混称为同一 SDK。
不手工修改生成文件。
字段语义和兼容规则见 [v1 说明](v1/README.md)。

目录导航：[上级目录](../README.md)。
