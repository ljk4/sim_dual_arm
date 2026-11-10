# Learning 容器构建与运行指南

- **`Dockerfile`**：基于 `huggingface/lerobot-gpu:latest` 构建，直接复用基础镜像中的 PyTorch、CUDA 与 FFmpeg 环境，清空默认 `ENTRYPOINT` 并以 `CMD ["sleep", "infinity"]` 空闲启动。
- **`docker-compose.yml`**：定义 `learning` 服务（默认镜像 `sim-dual-arm/learning:current`），启用 NVIDIA GPU，仅接入 `learning` 网络，只读挂载固定版本的 LeRobot v0.6.1 源码（`/third_party`）、预置权重缓存（`/opt/learning-cache`）与 Policy SDK（`/opt/policy-sdk`），不挂载任何 ROS 目录。
- **`learning-source.json`**：记录挂载的 LeRobot 上游提交版本与归档摘要。

## 构建与常用命令

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

> 启动 `policy` 服务前，须先在 `runtime` 容器中执行 `runtime/ros2/scripts/run.sh build-policy-sdk` 生成 `/opt/policy-sdk`。

目录导航：[上级目录](../README.md)
