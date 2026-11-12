# 构建、SDK 与边界检查脚本

本目录提供容器内 SDK 构建、测试执行与容器配置合规检查工具：

- **`build_ros_sdk.sh` / `dev_workspace.py` / `sdk_packages.py` / `sdk_marker.py` / `ros_workspace.py`**：在 `runtime` 容器内使用 `colcon` 构建 ROS 契约 SDK。`sdk_packages.py` 负责 binding/显式包选择、依赖闭包与摘要，`sdk_marker.py` 负责完成标记与模块来源校验，`dev_workspace.py` 保留工作区所有权、加锁、构建和 CLI。无 `--scenario`/`--package` 时只构建核心 `robot_api_msgs` 至 `/build/ros-core`；`--scenario <id>` 由 binding 选择核心、可选公共包和方案扩展包，构建至 `/build/ros-dev`。完成标记记录实际包集、各包源码摘要与生成模块来源，校验时拒绝包集不符、缺包、重复模块和错误来源。`--scenario` 选择依赖 `robot_contracts.binding`，因此需在已安装共享 Python 包的容器内执行。
- **`build_policy_sdk.sh` / `policy_sdk.py` / `generate_policy_sdk.py`**：从 `shared/common/interfaces/policy/v1/policy.proto` 生成 Python Protobuf/gRPC SDK 至 `/build/generated/policy-sdk`，并校验模块归属、文件完整性与 `.proto` 摘要，防止回退到过期的安装模块。
- **`test.sh`**：三端共用的容器内 `pytest` 入口，固定设置 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`，防止 ROS `launch_testing` 插件提前导入无关测试模块。
- **`check_dockerfiles.py`**：静态检查三端 `Dockerfile`、`docker-compose.yml`、`docker/deployment.yaml` 与 `scripts/run.sh` 是否满足单入口、空闲启动、无业务源码 `COPY`、无内嵌脚本及只读源码/产物隔离规范。GPU、源挂载位置、网络、必需挂载与 `run.sh` 命令由各端 `docker/deployment.yaml` 声明，公共检查器只验证共同规则与声明一致性。

所有构建产物均输出至 `build/`，不污染源码树。

目录导航：[上级目录](../README.md)
