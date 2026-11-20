# robot_contracts

跨端共享的单层 Pydantic v2 契约模型、语义校验与数据结构：

- **`models.py`、`validation.py`**：基础契约模型（`StrictContractModel`）、严格类型校验、单位检查与通用摘要工具。
- **`profiles.py`、`episodes.py`、`action_chunks.py`**：机器人 Profile、相机标定描述、Episode Manifest、动作块（`ActionWindow`）与语义哈希（`semantic_hash`）。
- **`binding.py`**：ROS 之外可用的方案绑定模型（`ScenarioBinding`）。`load_binding` 依据显式源根和产物根解析路径，校验公共档案、引用图和阶段条件，返回 `ResolvedBinding`。
- **`control.py`、`stream_request.py`、`execution_feedback.py`、`task_state.py`**：控制命令、流式请求（`StreamRequest`）、实测执行反馈（`ExecutionFeedback`）与任务状态（`TaskState`）契约 DTO。
- **[`trajectory/`](trajectory/README.md)**：不可变轨迹数据、结构校验和参考摘要。轨迹求值、跟随规划及数值验证见 `shared/common/python/trajectory`。

运行端调用 `robot_trajectory` 生成轨迹，执行端使用同一数学库校验收到的参考段。

目录导航：[上级目录](../README.md)
