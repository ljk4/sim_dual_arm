# dual_arm_learning

无 ROS 依赖（ROS-free）的 ACT 数据集转换、训练与策略推理服务包：

- **策略推理流水线与 gRPC 服务**：
  - `adapters/base.py`、`adapters/lerobot_act.py`：策略适配器协议与 LeRobot ACT 纯模型推理封装。
  - `pipeline.py`、`chunk_admission.py`、`temporal_ensemble.py`：分块推理流水线（`ChunkedPolicyPipeline`）、夹爪开度准入修正与按绝对仿真时刻对齐的历史预测指数加权融合。
  - `policy_config.py`、`policy_types.py`、`policy_service.py`、`policy_server.py`、`policy_service_cli.py`：单层 Pydantic 服务配置与契约模型、会话管理、gRPC 服务端及命令行入口。
- **数据集构建与起点筛选**：
  - `dataset.py`、`dataset_common.py`、`dataset_export.py`、`dataset_manifest.py`：数据转换主入口、共享哈希/路径原语、单局导出校验、LeRobot 分片写入与 Manifest 组装。
  - `sample_filter.py`、`sample_filter_rules.py`、`sample_filter_mask.py`：训练起点静止冗余筛选、Pydantic 规则校验与采样掩码摘要验证。
- **模型训练、检查点与发布包**：
  - `training.py`、`training_manifest.py`、`training_engine.py`、`training_support.py`：训练准入校验、预算上下文、CUDA AMP 混合精度训练循环、PyAV 视频解码、张量批处理 DataLoader 与验证评估。
  - `checkpoints.py`、`bundle.py`、`budget.py`：检查点保存/恢复、safetensors 模型包导出与追加式训练预算台账。
- **CLI 与通用配置**：`cli.py`、`config.py`、`config_yaml.py`。

目录导航：[上级目录](../README.md)
