# robot_contracts

跨端共享的单层 Pydantic v2 契约模型、语义校验与纯标准库轨迹数学实现：

- **`models.py`、`validation.py`**：基础契约模型（`StrictContractModel`）、严格类型校验、单位检查与通用摘要工具。
- **`profiles.py`、`episodes.py`、`action_chunks.py`**：机器人 Profile、相机标定描述、Episode Manifest、动作块（`ActionWindow`）与语义哈希（`semantic_hash`）。
- **`control.py`、`stream_request.py`、`execution_feedback.py`、`task_state.py`**：控制命令、流式请求（`StreamRequest`）、实测执行反馈（`ExecutionFeedback`）与任务状态（`TaskState`）契约 DTO。
- **`trajectory/`**：纯标准库轨迹数学包（延迟导入导出）：
  - `spline.py`、`_math/spline_math.py`：不可变分段三次样条计划（`ReferencePlan`）、采样、连续极值解析计算与约束验证。
  - `follower.py`、`_math/follower_math.py`：受限跟随计划（`PreparedFollowerPlan`）、端速估计与约束、连续偏差计算。
  - `follower_reference.py`：受限跟随参考记录（`FollowerReference`）、采样验证与摘要计算（`compute_follower_reference_digest`）。

仿真侧的 Ruckig 在线求解器调用保留在 `simulation/mujoco/` 中，不进入本包。

目录导航：[上级目录](../README.md)
