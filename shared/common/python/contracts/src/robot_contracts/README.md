# robot_contracts

跨端共享的单层 Pydantic v2 契约模型、语义校验与数据结构：

- **`models.py`、`validation.py`**：基础契约模型（`StrictContractModel`）、严格类型校验、单位检查与通用摘要工具。
- **`profiles.py`、`episodes.py`、`action_chunks.py`**：机器人 Profile、相机标定描述、Episode Manifest、动作块（`ActionWindow`）与语义哈希（`semantic_hash`）。
- **`binding.py`**：轻量、严格、ROS-free 的方案 binding v1 模型（`ScenarioBinding`）、显式源根/产物根下的路径解析（`load_binding` 返回 `ResolvedBinding`）、显式引用图与阶段条件。只校验结构与公共档案，不解析端私有 recipe 语义、不加载模型权重、不导入端实现。
- **`control.py`、`stream_request.py`、`execution_feedback.py`、`task_state.py`**：控制命令、流式请求（`StreamRequest`）、实测执行反馈（`ExecutionFeedback`）与任务状态（`TaskState`）契约 DTO。
- **[`trajectory/`](trajectory/README.md)**：不可变轨迹数据、版本常量、结构校验和参考摘要。求值、跟随规划及数值验证见 `shared/common/python/trajectory`；本包不依赖该算法库。

仿真侧的 Ruckig 在线求解器调用保留在 `simulation/mujoco/` 中，不进入本包。

目录导航：[上级目录](../README.md)
