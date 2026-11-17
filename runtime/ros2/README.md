# ROS 2 运行端（Runtime ROS 2）

ROS 2 运行端负责连接仿真/真机后端（Robot API）与策略服务端（Policy API），完成观测同步、控制调度、示教录制、策略推理与批量测试。

## 常用入口

- **宿主机并行采集**：仓库根目录 [`./record_parallel.sh`](../../record_parallel.sh)（自动管理多个无界面仿真 worker）。
- **容器内脚本**（位于 [`scripts/`](scripts/README.md)）：
  - `scripts/build_sdk.sh`：编译 ROS 2 接口包并生成 Policy SDK。
  - `scripts/record.sh`：单路示教录制；`scripts/record.sh status --robot <robot> --task <task>` 查看采集与筛选状态。
  - `scripts/policy.sh`：单局非交互式策略推理。
  - `scripts/batch_policy.sh`：无人工干预批量仿真推理测试。
  - `scripts/interactive.sh`：交互式单局策略推理。
  - `scripts/test.sh`：运行容器内 `pytest` 测试。

所有运行入口直接通过 `--robot <robot> --task <task>`（或显式 `--config <path>`）在内存中解析 `<shared>/scenarios/<robot>/<task>/binding.yaml`，支持 `--set dot.path=value` 覆盖参数，不生成准备目录。运行端只通过 gRPC 连接 Policy Service，不读取模型 Bundle。

MCAP 录制使用 `zstd_fast` 原生无损块压缩；消息字节、主题和时间戳保持完整，离线读取与审计入口相同。

示教与推理仿真后端在 simulation 容器启动，入口见 [simulation 脚本说明](../../simulation/mujoco/scripts/README.md)。

## 目录结构

- [`ros/dual_arm_runtime/`](ros/dual_arm_runtime/README.md)：`dual_arm_runtime` 核心 Python 包。
- [`configs/`](configs/README.md)：运行端环境变量 `container.env`。
- [`docker/`](docker/README.md)：`Dockerfile`、`docker-compose.yml` 与依赖清单。
- [`scripts/`](scripts/README.md)：容器内单职责入口脚本。
