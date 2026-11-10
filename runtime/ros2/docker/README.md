# Runtime 容器构建与编排

- **`Dockerfile`**：基于 `ros:jazzy-ros-base-noble` 构建运行端环境，安装 `colcon`、MCAP、Pydantic、JSON Schema 及 `requirements-dev.txt` 中的依赖。不 `COPY` 业务源码或启动脚本，清空 `ENTRYPOINT` 并以 `CMD ["sleep", "infinity"]` 空闲启动。
- **`docker-compose.yml`**：定义 `runtime` 服务（默认镜像 `sim-dual-arm/runtime:current`），加载 `../configs/container.env`，同时接入 `ros` 与 `learning` 网络（无需 GPU）。

## 常用操作

```bash
cd runtime/ros2/docker
docker compose build
docker compose up -d
docker compose exec runtime bash

# 在容器内构建 SDK 或运行测试：
runtime/ros2/scripts/run.sh build-ros-sdk
runtime/ros2/scripts/run.sh build-policy-sdk
runtime/ros2/scripts/run.sh test -q tests/unit
```

目录导航：[上级目录](../README.md)
