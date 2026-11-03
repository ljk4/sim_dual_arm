# 公共契约

协议、ROS IDL、schema 和轻量 Python 类型的唯一维护位置。

- `policy/v1/policy.proto`：Policy RPC 与动作块语义。
- `ros/robot_api_msgs/`：ROS 2 action/msg/srv 包。
- `schemas/v1`～`v4`：profile、观测、动作、配置和 episode manifest。
- `python/src/robot_contracts/`：无 ROS/ML 依赖的类型、校验和语义哈希。

生成 SDK/ROS 文件写到 `.build/`，不在组件里复制；规则见
[项目规范](../docs/standards.md)。

目录导航：[上级目录](../README.md)。
