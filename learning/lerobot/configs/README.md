# 学习端配置说明（Configs）

本目录保留学习端私有环境与测试替身；真实 Panda 学习配方已迁至方案目录。

- **`container.env`**：声明 `learning` 容器内源码与缓存路径、离线模式（禁止访问 Hugging Face Hub 或下载权重）与 GPU 环境变量，由 Compose `env_file` 自动加载。
- **`fake_policy.yaml`**：仅供测试替身（Fake Policy Provider）使用的配置模板，测试归属，严禁用于正式评估。
- **[`shared/scenarios/panda_stack/configs/learning/`](../../../shared/scenarios/panda_stack/configs/learning/)**：真实 Panda 数据集转换、训练、采样过滤与 Policy 服务配方。

方案入口使用 `--scenario <id>` 选择配方，直接模式使用 `--config`/`--template`，两者互斥。默认配方为 `policy_temporal.yaml`、`act_dataset.yaml` 和 `act_training.yaml`；数据与训练配方各维护一份，路径按项目根解析，不另设容器副本。

目录导航：[上级目录](../README.md)
