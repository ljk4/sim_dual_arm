# 跨端共享 Python 包（Python）

本目录存放具有明确调用边界、保持现有 `pyproject + src` 结构和导入名的独立 Python 包：

- **[`contracts`](contracts/README.md)**：`robot_contracts` 契约模型、语义哈希与轨迹数据结构。
- **[`ros_support/`](ros_support/README.md)**：ROS 2 `robot_api_msgs` 与 Python 契约 DTO 之间的显式双向转换及 QoS 配置。
- **[`trajectory/`](trajectory/README.md)**：样条求值、连续限位、受限跟随规划验证和参考重建，依赖 `robot_contracts`，保持 ROS-free。
- **[`run_logging/`](run_logging/README.md)**：有界结构化事件队列、JSONL 日志轮转与健康检查。

共享库不导入具体业务端（`simulation`、`runtime`、`learning`）代码，也不持有全局运行生命周期。

目录导航：[上级目录](../README.md)
