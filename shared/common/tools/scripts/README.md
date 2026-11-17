# 构建、SDK 与边界检查脚本

本目录提供容器内 SDK 构建、测试执行与容器配置合规检查工具：

- **`build_ros_sdk.sh` / `dev_workspace.py` / `sdk_packages.py` / `sdk_marker.py` / `ros_workspace.py`**：在 `runtime` 容器内使用 `colcon` 构建 ROS 契约 SDK。默认一次编译 `shared/common/interfaces/ros/*` 与全部 `shared/scenarios/*/*/extensions/ros/*` 接口包至 `/build/ros-dev`，构建一次即可服务全部方案；`sdk_packages.py` 负责全量/binding/显式包选择、依赖闭包与摘要，`sdk_marker.py` 负责完成标记与模块来源校验，`dev_workspace.py` 负责加锁、构建和 CLI。`colcon` 的 `build`、`log` 与 `COLCON_HOME` 位于 `/tmp` 临时目录并在构建结束后删除，输出根只保留 `install/` 与 `contracts.json`。`--scenario <robot>/<task>`（或 `--robot/--task`）或 `--package` 可将 `check` 限定为所需包集，只要这些包都存在于已编译集合并与源码摘要一致即通过；`--scenario` 选择依赖 `robot_contracts.binding`，因此需在已安装共享 Python 包的容器内执行。
- **`build_policy_sdk.sh` / `policy_sdk.py` / `generate_policy_sdk.py`**：从 `shared/common/interfaces/policy/v1/policy.proto` 生成 Python Protobuf/gRPC SDK 至 `/build/generated/policy-sdk`，并校验模块归属、文件完整性与 `.proto` 摘要，防止回退到过期的安装模块。
- **`test.sh`**：三端共用的容器内 `pytest` 入口，固定设置 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`，防止 ROS `launch_testing` 插件提前导入无关测试模块。
- **`check_dockerfiles.py`**：静态检查三端 `Dockerfile`、`docker-compose.yml`、`docker/deployment.yaml` 与 `scripts/test.sh` 等声明入口是否满足空闲启动、无业务源码 `COPY`、无内嵌脚本及只读源码/产物隔离规范。GPU、源挂载位置、网络、必需挂载与入口脚本由各端 `docker/deployment.yaml` 声明。

所有构建产物均输出至 `build/`，不污染源码树。

目录导航：[上级目录](../README.md)
