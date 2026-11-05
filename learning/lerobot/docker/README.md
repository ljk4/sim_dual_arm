# Learning 镜像

`Dockerfile` 直接继承 Hugging Face 官方 `huggingface/lerobot-gpu:latest`，
不安装第二套 Python、Torch 或 CUDA。基础镜像已包含 FFmpeg、LeRobot、GPU 训练依赖和测试工具；
本端没有额外依赖需要 `COPY` 或 `pip install`。镜像只清空继承的入口并空闲启动，
路径、离线设置和 GPU 能力放在 `../configs/container.env`。

项目运行时仍从只读挂载加载固定的 LeRobot v0.6.1 源码、项目源码、数据和模型；
来源记录在 `learning-source.json`。官方 `latest` 会变化，每次重建都需核对实际
镜像摘要、Python/Torch/LeRobot 版本，并重跑数据转换、训练与模型加载检查。
这次构建的官方镜像已包含 LeRobot 0.6.2、Torch 2.15.0+cu132 和 TorchCodec
0.17.0+cpu；它们超出固定 v0.6.1 源码声明的部分依赖范围。已有合成数据转换通过，
但不能据此代替真实训练和推理验收。

容器离线，训练不
下载 backbone 或写 Hub/W&B。`docker-compose.yml` 用 `build` 指定本 Dockerfile，
默认镜像 `sim-dual-arm/learning:current`，只加入 learning 网络，不挂 ROS SDK。

标准构建与运行：

```bash
cd learning/lerobot/docker
docker compose build
docker compose up -d
docker compose exec learning bash
# 容器内：
learning/lerobot/scripts/run.sh --help
learning/lerobot/scripts/run.sh test -q tests/unit
```

当前通用容器配方是 `act_dataset.container.yaml` + `act_training.container.yaml`
（`/data`、`/opt/learning-cache`）。新实验使用独立配置、输出目录和经过核对的预算。
Policy SDK 由 runtime 的 `run.sh build-policy-sdk` 生成到
`.build/generated/policy-sdk`，只读挂到 `/opt/policy-sdk`。业务脚本在真正需要时
检查当前 proto 摘要和模块来源；公共 shell 入口不预先强制生成 SDK。

目录导航：[上级目录](../README.md)。
