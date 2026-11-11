# 构建、SDK 与边界检查脚本

本目录提供容器内 SDK 构建、测试执行与容器配置合规检查工具：

- **`build_ros_sdk.sh` / `dev_workspace.py` / `ros_workspace.py`**：在 `runtime` 容器内使用 `colcon` 编译 `contracts/ros/robot_api_msgs` 与 `contracts/ros/sim_api_msgs`至 `/build/ros-dev`（宿主机 `.build/ros-dev`），并逐包校验生成模块、生成源码 SHA-256 摘要供 `runtime` 与 `simulation` 启动前校验。
- **`build_policy_sdk.sh` / `policy_sdk.py` / `generate_policy_sdk.py`**：从 `contracts/policy/v1/policy.proto` 生成 Python Protobuf/gRPC SDK 至 `/build/generated/policy-sdk`，并校验模块归属、文件完整性与 `.proto` 摘要，防止回退到过期的安装模块。
- **`test.sh`**：三端共用的容器内 `pytest` 入口，固定设置 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`，防止 ROS `launch_testing` 插件提前导入无关测试模块。
- **`check_dockerfiles.py`**：静态检查三端 `Dockerfile`、`docker-compose.yml` 与 `scripts/run.sh` 是否满足单入口、空闲启动、无业务源码 `COPY`、无内嵌脚本及网络/GPU/SDK 挂载隔离规范。

所有构建产物均输出至 `.build/`，不污染源码树。

目录导航：[上级目录](../README.md)
