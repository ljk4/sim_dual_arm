# 工具入口

薄入口，业务实现留在 `components/`、`libraries/` 和 `contracts/`：

- `assets/`：固定 Panda 资产获取与校验。
- `build/`：ROS/仿真/开发镜像和工作空间构建。
- `contracts/`：Policy SDK 生成。
- `run/`：mock/原生采集、Policy、学习、任务和检查入口。

工具不复制生命周期或算法；参数从 `configs/` 读取，产物写 `.build/` 或 `var/`。

目录导航：[上级目录](../README.md)。
