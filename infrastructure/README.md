# 公共基础设施（Infrastructure）

本目录维护跨运行端复用的容器环境脚本、SDK 生成器与架构边界检查工具，不包含机器人业务逻辑、仿真场景参数或训练算法。

## 目录结构

- **[`docker/`](docker/README.md)**：跨端共享的 Fast DDS 配置（`fastdds.xml`）与唯一可 `source` 的容器环境初始化脚本（`setup.sh`）。
- **[`scripts/`](scripts/README.md)**：容器内 ROS 2 SDK / Policy SDK 构建脚本、工作空间摘要校验器、统一 `pytest` 入口与 Dockerfile/Compose 边界合规检查器。

各端的 `Dockerfile`、`docker-compose.yml`、`configs/container.env` 与 `scripts/run.sh` 均维护在各自实现目录（`simulation/mujoco/`、`runtime/ros2/`、`learning/lerobot/`）下。

目录导航：[上级目录](../README.md)
