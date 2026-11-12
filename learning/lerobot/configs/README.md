# 学习端配置说明（Configs）

本目录保留学习端私有环境与测试替身；真实 Panda 学习配方已迁至方案目录。

- **`container.env`**：声明 `learning` 容器内源码路径、只读缓存路径、离线模式（禁止访问 Hugging Face Hub 或下载权重）与 GPU 环境变量，由 Compose `env_file` 自动加载。
- **`fake_policy.yaml`**：仅供测试替身（Fake Policy Provider）使用的配置模板，测试归属，严禁用于正式评估。
- **[`shared/scenarios/panda_stack/configs/learning/`](../../../shared/scenarios/panda_stack/configs/learning/)**：真实 Panda 数据集转换、训练、采样过滤与 Policy 服务配方。

学习端命令仍支持直接 `--config`/`--template`；脚本默认路径已指向方案配方（`policy_temporal.yaml`、`act_dataset.container.yaml`、`act_training.container.yaml`）。`--scenario` 接入留阶段 04。

目录导航：[上级目录](../README.md)
