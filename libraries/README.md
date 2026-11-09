# 共享库

有明确调用方的轻量实现，避免把同一语义复制到组件：

- `ros_support/`：ROS 消息与公共 DTO 的显式转换、QoS。
- `run_logging/`：结构化事件、有界队列和 JSONL 轮转。

库不导入具体组件，不持有运行生命周期；依赖规则见
[项目规范](../docs/standards.md)。

目录导航：[上级目录](../README.md)。
