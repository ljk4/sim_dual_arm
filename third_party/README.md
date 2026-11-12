# 第三方源码说明（Third Party）

本目录管理固定版本的外部开源代码库：

- **`src/lerobot-act-v0.6.1/`**：学习端容器只读挂载使用的固定版本 LeRobot 源码（上游版本与摘要记录于 [`learning/lerobot/docker/learning-source.json`](../learning/lerobot/docker/learning-source.json)）。`src/` 目录由根 `.gitignore` 忽略，保持上游文件与许可证原样不作修改。

学习端的 `PROJECT_SOURCE_PATHS` 直接使用该目录下的 `src/`。新检出仓库或清理本地源码后，需要按 `learning-source.json` 的 `revision` 恢复对应上游源码至声明的 `source` 路径；构建容器镜像不会下载这份源码。训练所需的本地骨干权重见[训练步骤](../README.md#32-转换数据集与训练-act-模型)。

## 其他外部资产位置

- **Franka Emika Panda 仿真模型**：直接维护于 [`simulation/mujoco/assets/franka_emika_panda/`](../simulation/mujoco/assets/README.md)，随仿真目录只读挂载至容器 `/app/assets/`。
- **固定 Ruckig Wheel 安装包**：维护于 [`simulation/mujoco/docker/wheels/`](../simulation/mujoco/docker/wheels/README.md)，包含二进制 wheel、`provenance.json` 来源摘要与上游 MIT 许可证。

目录导航：[上级目录](../README.md)
