# 公共契约（Contracts）

本目录是跨端协议、ROS 2 IDL、JSON Schema 与轻量 Python 契约模型的唯一数据源（SSOT）。

## 目录结构

- **[`policy/v1/policy.proto`](policy/v1/README.md)**：gRPC Policy 推理服务与动作块（Action Chunk）协议定义。
- **[`ros/`](ros/README.md)**：通用 `robot_api_msgs` 与仿真专用 `sim_api_msgs` 消息/服务接口包。
- **[`schemas/v1/`](schemas/v1/README.md)**：机器人 Profile、观测、动作、运行配置、日志事件与 Episode Manifest 的 JSON Schema 定义。
- **[`python/src/robot_contracts/`](python/src/robot_contracts/README.md)**：无 ROS/MuJoCo/PyTorch 依赖的 Pydantic v2 契约模型、语义哈希与轨迹数据（`trajectory/`）；算法位于 `libraries/trajectory`。

生成的 SDK 与构建产物输出至 `.build/`，不在各运行端复制接口源码。详细约定见[项目规范](../docs/standards.md)。

目录导航：[上级目录](../README.md)
