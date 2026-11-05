# 构建工具

- `test.sh`：运行/学习端共用的容器内 pytest 入口，固定关闭自动插件收集，保留真实退出码。
- `build_ros_sdk.sh`：容器内用 `dev_workspace.py`/colcon 生成 ROS SDK 到
  `/build/ros-dev`（宿主 `.build/ros-dev`），由 runtime `run.sh build-ros-sdk` 调用。
- `build_policy_sdk.sh`：容器内用 `policy_sdk.py` 生成 Policy SDK 到
  `/build/generated/policy-sdk`，由 runtime `run.sh build-policy-sdk` 调用。
- `dev_workspace.py`：生成/校验 ROS 契约工作空间，检查 host home 不被重置。
- `policy_sdk.py`：生成/校验挂载 SDK，拒绝无归属输出、过期摘要和镜像模块回退。
- `ros_workspace.py`：colcon 的 Python 入口。
- `generate_policy_sdk.py`：从 `contracts/policy/v1/policy.proto` 生成 Protobuf/gRPC SDK。
- `check_dockerfiles.py`：检查每端唯一 Dockerfile/Compose/run.sh、无旧入口包装，
  拒绝业务 COPY、内嵌程序、非 idle Compose，并保留 GPU/网络/SDK 边界检查。

SDK 与工作空间构建输出写到 `.build/`，镜像写入 Docker 存储，不修改源码树。宿主
在各端 `docker/` 目录用 `docker compose build/up -d/exec` 管理容器，进入后显式运行
各端 `run.sh`；不在宿主 venv 中运行项目 Python，也不自动生成 SDK。

目录导航：[上级目录](../README.md)。
