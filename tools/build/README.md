# 构建工具

- `ros_workspace.py`：生成契约、构建/安装 ROS 工作空间。
- `sim.sh`：构建 `mujoco-dev` 仿真依赖 target。
- `stage2.sh`：构建 ROS base/dev/Policy 依赖镜像，生成宿主 ROS 和 Policy SDK。
- `dev.sh`、`dev_workspace.py`：开发镜像/依赖工作空间。
- `policy_sdk.sh`、`policy_sdk.py`：用容器生成、复用和校验挂载的 Policy SDK，
  拒绝无归属输出、过期摘要和镜像模块回退。
- `image_metadata.py`：仿真依赖版本、Ruckig wheel 来源记录与安装校验。
- `check_dockerfiles.py`：检查业务 COPY、内嵌程序和 shell 控制流等常见违规。

ROS/SDK 构建输出写到 `.build/`，镜像写入 Docker 存储，不修改源码树；
依赖由相应 Dockerfile、requirements 与锁文件声明。

`DEV_DOCKER` 选择 Docker 命令；`DEV_ROS_BASE_IMAGE`、`DEV_IMAGE` 选择依赖镜像，
`DEV_BUILD_ROOT`、`POLICY_SDK_ROOT` 选择生成输出，`POLICY_BUILD_IMAGE` 可单独选择
SDK 生成容器。`PROTOCOL_IMAGE`、`POLICY_IMAGE` 可覆盖构建标签，便于隔离验收。
默认路径见脚本；项目级运行顺序见根 README。IDL/proto 改动需停止相关服务并重建
对应 SDK，Python/配置改动只需重启进程。

目录导航：[上级目录](../README.md)。
