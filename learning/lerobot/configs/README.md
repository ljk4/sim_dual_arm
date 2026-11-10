# 学习端配置说明（Configs）

- **`container.env`**：声明 `learning` 容器内源码路径、只读缓存路径、离线模式（禁止访问 Hugging Face Hub 或下载权重）与 GPU 环境变量，由 Compose `env_file` 自动加载。
- **`act_dataset.yaml` / `act_dataset.container.yaml`**：将导出的合格 Episode 转换为 45/5 LeRobot 数据集的配置。
- **`act_training.yaml` / `act_training.container.yaml`**：ACT 训练超参数配置，`training` 段显式管理 `batch_size`、`num_workers`、`pin_memory`、`prefetch_factor`、`use_amp`、`amp_dtype`（默认 `bfloat16`）与验证间隔。
- **`act_sample_filter.yaml`**：训练集静止冗余采样起点筛选规则。
- **`policy_temporal.yaml`**：正式 ACT Policy 服务配置模板（启用按绝对仿真时间对齐的预测融合 `model.temporal_ensemble_coeff: 0.01`）。通过 `prepare-policy` 绑定真实模型包后生成可执行的 `policy.yaml`。
- **`fake_policy.yaml`**：仅供测试替身（Fake Policy Provider）使用的配置模板，严禁用于正式评估。

目录导航：[上级目录](../README.md)
