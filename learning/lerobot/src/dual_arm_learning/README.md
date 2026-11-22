# dual_arm_learning

无 ROS 依赖（ROS-free）的 ACT 数据集转换、训练与策略推理服务包：

- **`adapters/`**（策略适配器）：
  - `base.py`、`lerobot_act.py`：模型适配器抽象基类与 LeRobot ACT 推理适配器（直接消费 `(C, H, W)` 图像与向量状态，基于 `RobotProfile` 维度动态适配）。
- **`serving/`**（策略推理流水线与 gRPC 服务）：
  - `pipeline.py`、`chunk_admission.py`、`temporal_ensemble.py`：分块推理流水线（`ChunkedPolicyPipeline`）、夹爪开度准入与预测边界投影、按绝对时刻对齐的历史预测指数加权融合。
  - `config.py`、`types.py`、`service.py`、`server.py`、`cli.py`：单层 Pydantic 服务配置与契约模型（复用 `robot_contracts.models.RunContext`）、会话管理、gRPC 服务端及命令行入口。
- **`dataset/`**（数据集构建与起点筛选）：
  - `dataset.py`、`common.py`、`export.py`、`manifest.py`：数据转换主入口、共享哈希/路径原语、单局导出校验、LeRobot 分片写入与自包含 Manifest 组装。
  - `sample_filter.py`、`sample_filter_rules.py`、`sample_filter_mask.py`：full 训练时基于 `datasets/lerobot/` 自包含筛选元数据在内存中抽稀冗余静止锚点、Pydantic 规则校验与采样掩码摘要验证。
- **`training/`**（模型训练与单文件模型包）：
  - `training.py`、`manifest.py`、`engine.py`、`support.py`：训练准入校验、CUDA AMP 混合精度训练循环、PyAV 视频解码、张量批处理 DataLoader 与验证评估。
  - `bundle.py`：把策略权重、归一化张量与推理元数据写入唯一 `model.safetensors`，并直接从该文件重建策略与处理器。
- **包根模块**：`cli.py`（`convert`/`train` CLI 入口）、`config.py`（配方 Pydantic 模型）、`scenario.py`（方案绑定解析与四分区路径展开）、`console_progress.py`（终端进度与控制台摘要）。

目录导航：[上级目录](../README.md)
