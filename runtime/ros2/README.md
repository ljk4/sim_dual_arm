# ROS 2 运行端（Runtime ROS 2）

ROS 2 运行端连接仿真/真机后端（Robot API）与策略服务端（Policy API），负责观测同步、控制租约与流式动作块调度、MCAP 录制、标签导出与离线评估。

本端持有会话、录制、导出、审计与采集 worker。公共消息和 Policy 契约在 [`shared/common/interfaces/`](../../shared/common/interfaces/policy/README.md)；当前 Panda 堆叠任务的运行配方、Teacher 与 campaign 选择在 [`shared/scenarios/panda_stack/`](../../shared/scenarios/panda_stack/README.md)。

## 目录结构

- **[`ros/dual_arm_runtime/`](ros/dual_arm_runtime/README.md)**：`dual_arm_runtime` 核心 Python 包。
- **[`configs/`](configs/README.md)**：容器环境配置（`container.env`）、示教采集模板（`teacher_collect.yaml`）、策略运行模板（`policy_rollout.yaml`）与实验台账配置（`experiments/`）。
- **[`docker/`](docker/README.md)**：运行端 `Dockerfile`、`docker-compose.yml` 与开发测试依赖清单。
- **[`scripts/`](scripts/README.md)**：单路录制入口 `record.sh`、宿主机并行编排入口 `record_parallel.sh`、其他命令入口 `run.sh`；Python 业务逻辑集中在 `dual_arm_runtime/offline/`。

## 架构边界

- 运行端不导入后端物理引擎（MuJoCo）代码，也不包含深度学习框架（PyTorch/LeRobot）依赖。
- 示教录制在 `runtime` 容器内通过 `runtime/ros2/scripts/record.sh --count N` 执行，达到本次新增合格数量或连续失败上限后结束。
- 单路 pilot 试采合格后，正式采集使用宿主机的 `runtime/ros2/scripts/record_parallel.sh --target-total 50 --workers auto` 补足累计 50 条合格 formal 数据；入口编排独立后台仿真实例，禁用可视化，统一使用原采集台账。
- 其他业务命令通过 `runtime/ros2/scripts/run.sh <command>` 执行（涵盖 `build-ros-sdk`、`build-policy-sdk`、`prepare`、`policy`、`interactive`、`export`、`campaign`、`evaluate`、`test`）。

目录导航：[上级目录](../README.md)
