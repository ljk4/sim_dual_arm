# 公共容器环境配置

提供跨 `simulation`、`runtime` 与 `learning` 容器复用的运行时基础配置：

- **`fastdds.xml`**：ROS 2 Fast DDS 通信配置文件，以只读方式挂载至 `simulation` 与 `runtime` 容器（不打包进镜像）。
- **`setup.sh`**：全仓唯一可 `source` 的容器环境初始化脚本。负责校验容器运行边界（拒绝在宿主机直接执行）并配置 `PATH`、`PYTHONPATH` 与 ROS 2 环境；各端 `scripts/run.sh`、SDK 构建脚本与 `test.sh` 均复用该脚本。

目录导航：[上级目录](../README.md)
