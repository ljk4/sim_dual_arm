# 学习端配置说明（Configs）

本目录保存学习端容器环境。业务配方位于各方案目录，策略测试替身位于 [tests/fakes/policy](../../../tests/fakes/policy/)。

- **`container.env`**：声明 `learning` 容器内源码与缓存路径、离线模式（禁止访问 Hugging Face Hub 或下载权重）与 GPU 环境变量，由 Compose `env_file` 自动加载。
- **[`shared/scenarios/marvin_sim/stack/configs/learning/`](../../../shared/scenarios/marvin_sim/stack/configs/learning/)**：Marvin 堆叠的数据集转换、训练、采样过滤与 Policy 服务配方。
- **[`shared/scenarios/marvin_sim/handover/configs/learning/`](../../../shared/scenarios/marvin_sim/handover/configs/learning/)**：Marvin 交接的数据集转换、训练、采样过滤与 Policy 服务配方。

方案入口使用 `--robot <robot> --task <task>` 选择配方。直接模式使用 `--config`。修改转换、训练和服务参数时，编辑选定的配方。方案模式按机器人和任务归档产物，直接模式使用配方中的数据、模型和日志路径。

目录导航：[上级目录](../README.md)
