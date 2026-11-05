# Runtime 镜像与组合

- `Dockerfile`：唯一运行依赖镜像，直接基于 `ros:jazzy-ros-base-noble`，包含
  colcon、MCAP、jsonschema 和本目录 `requirements-dev.txt`；Fast DDS 配置从公共
  文件只读挂载，不 COPY 业务或 `run.sh`。仅保留依赖 `PATH` 环境变量；
  `ENTRYPOINT` 清空，`CMD` 为 `sleep infinity`。
- `docker-compose.yml`：单个空闲 `runtime` 服务，build 本目录 Dockerfile，镜像默认
  `sim-dual-arm/runtime:current`；加入共享 ROS 网络与 learning 网络。
- `../configs/container.env`：容器内源码、ROS 与路径设置。
- `../scripts/run.sh`：进入容器后手动执行的功能脚本，提供
  `build-ros-sdk`、`build-policy-sdk`、`prepare`、`collect`、`policy`、`control`、
  `audit`、`export`、`campaign`、`evaluate`、`test`。

源码、配置、SDK、数据与模型通过只读/可写挂载提供。在本目录执行
`docker compose build`、`docker compose up -d`、`docker compose exec runtime bash`。
ROS/Policy SDK 由容器内 `run.sh build-*-sdk` 调用
`infrastructure/scripts/build_*_sdk.sh` 生成。

目录导航：[上级目录](../README.md)。
