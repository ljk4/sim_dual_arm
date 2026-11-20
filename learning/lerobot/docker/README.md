# Learning 容器构建与使用

- **基础镜像**：基于 `huggingface/lerobot-gpu:latest`，复用 PyTorch、CUDA 与 FFmpeg 环境（需要 NVIDIA GPU）。
- **空闲启动**：容器以 `CMD ["sleep", "infinity"]` 空闲运行，通过容器内 `scripts/*.sh` 执行任务。

## 常用操作

首次使用前，请先按[根 README](../../../README.md) 准备目录与 LeRobot 源码，然后使用 `./env.sh` 管理容器：

```bash
# 宿主机：构建并启动 learning 容器，然后进入终端
./env.sh up learning
./env.sh enter learning
```

> 启动 `scripts/server_policy.sh` 前，须先在 `runtime` 容器运行过 `scripts/build_sdk.sh` 生成 `/build/generated/policy-sdk`。

运行测试见[测试执行方式](../../../tests/README.md#执行方式)。
