# 公共 Docker 文件

跨组件复用的容器环境文件，不放具体实现的业务源码、场景配置或模型：

- `fastdds.xml`：源码挂载中的 Fast DDS 配置；仿真与运行镜像都不复制该文件。
- `setup.sh`：唯一可 source 的公共 helper。它拒绝宿主执行，只设置 PATH、
  PYTHONPATH 和 ROS；不预检尚未生成的 SDK，也不启动业务。三端 `run.sh`、
  SDK 构建和共享测试脚本都复用它。仿真将它单独只读挂到
  `/config/setup.sh`，由 `PROJECT_SETUP` 选择，使用 `/app` 项目根。

组件专属 Dockerfile、锁文件、来源信息和 `docker-compose.yml` 在各自实现目录：
`simulation/mujoco/docker/`、`learning/lerobot/docker/`、
`runtime/ros2/docker/`。每端直接使用自己的 Compose 文件。

目录导航：[上级目录](../README.md)。
