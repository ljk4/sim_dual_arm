# Learning 容器构建与运行指南

- **`Dockerfile`**：基于 `huggingface/lerobot-gpu:latest` 构建，直接复用基础镜像中的 PyTorch、CUDA 与 FFmpeg 环境，清空默认 `ENTRYPOINT` 并以 `CMD ["sleep", "infinity"]` 空闲启动。
- **`docker-compose.yml`**：定义 `learning` 服务（默认镜像 `sim-dual-arm/learning:current`），启用 NVIDIA GPU，仅接入 `learning` 网络。项目、共享包与固定版本的 LeRobot 源码只读挂载，`build/` 和 `var/` 可写；Policy SDK 位于 `/build/generated/policy-sdk`，数据与缓存位于 `/data/learning/`，不加载 ROS 环境或挂载 ROS SDK。
- **`learning-source.json`**：记录挂载的 LeRobot 上游提交版本与归档摘要。

## 构建与常用命令

首次启动前，在仓库根目录按[环境准备](../../../README.md#1-环境准备)创建输出目录，并按[第三方源码说明](../../../third_party/README.md)准备 LeRobot 源码。

```bash
cd learning/lerobot/docker
docker compose build
docker compose up -d
docker compose exec learning bash

# 在容器内执行命令：
cd /workspace
learning/lerobot/scripts/run.sh --help
learning/lerobot/scripts/run.sh test -q tests/unit
```

启动 `policy` 服务前，须先在 `runtime` 容器中执行 `runtime/ros2/scripts/run.sh build-policy-sdk` 生成 `/build/generated/policy-sdk`。

目录导航：[上级目录](../README.md)
