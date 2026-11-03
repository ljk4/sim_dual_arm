# 契约生成

`generate_policy_sdk.py` 从 `contracts/policy/v1/policy.proto` 生成可安装的 Python
Protobuf/gRPC SDK 到构建目录，同时记录源码摘要。ROS 消息包另由 ROS 构建工具生成。
生成文件不提交、不手工修改；协议变更后重新生成并跑兼容检查。

目录导航：[上级目录](../README.md)。
