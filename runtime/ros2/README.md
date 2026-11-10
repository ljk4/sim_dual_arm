# ROS 2 运行端（Runtime ROS 2）

ROS 2 运行端连接仿真/真机后端（Robot API）与策略服务端（Policy API），负责观测同步、控制租约与流式动作块调度、MCAP 录制、标签导出与离线评估。

## 目录结构

- **[`ros/dual_arm_runtime/`](ros/dual_arm_runtime/README.md)**：`dual_arm_runtime` 核心 Python 包。
- **[`configs/`](configs/README.md)**：容器环境配置（`container.env`）、示教采集模板（`teacher_collect.yaml`）、策略运行模板（`policy_rollout.yaml`）与实验台账配置（`experiments/`）。
- **[`docker/`](docker/README.md)**：运行端 `Dockerfile`、`docker-compose.yml` 与开发测试依赖清单。
- **[`scripts/`](scripts/README.md)**：容器内统一命令入口 `run.sh` 以及配置准备、采集台账、标签导出与评估脚本。

## 架构边界

- 运行端不导入后端物理引擎（MuJoCo）代码，也不包含深度学习框架（PyTorch/LeRobot）依赖。
- 所有业务命令须在 `runtime` 容器内通过 `runtime/ros2/scripts/run.sh <command>` 执行（涵盖 `build-ros-sdk`、`build-policy-sdk`、`prepare`、`collect`、`policy`、`interactive`、`export`、`campaign`、`evaluate`、`test`）。

目录导航：[上级目录](../README.md)
