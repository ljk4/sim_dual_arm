# ROS 2 运行端（Runtime ROS 2）

ROS 2 运行端负责连接仿真/真机后端（Robot API）与策略服务端（Policy API），完成观测同步、控制调度、示教录制与策略推理客户端。

## 常用入口

- **宿主机并行采集**：仓库根目录 [`./record_parallel.sh`](../../record_parallel.sh)（自动管理多个无界面仿真 worker）。
- **容器内脚本**（`scripts/`）：
  - `scripts/build_sdk.sh`：编译 ROS 2 接口包并生成 Policy SDK。
  - `scripts/record.sh`：单路示教录制；`scripts/record.sh status --robot <robot> --backend <backend> --task <task>` 查看采集与筛选状态。
  - `scripts/client_policy.sh`：单局非交互式策略推理（gRPC 客户端）。
  - `scripts/client_interactive.sh`：交互式单局策略推理客户端。
- **宿主机并行推理**：仓库根目录 [`./inference_parallel.sh`](../../inference_parallel.sh)（自动管理多个无界面仿真 worker 与批量统计报告）。
- **宿主机录制转视频**：仓库根目录 [`./recording_to_video.sh`](../../recording_to_video.sh)（将单局或批量 MCAP 录制合成为多视角可视化 MP4，默认保存在同目录下）。
  - `scripts/test.sh`：运行容器内 `pytest` 测试。

使用 `--robot <robot> --backend <backend> --task <task>` 从 `<shared>/scenarios/<robot>/<backend>/<task>/binding.yaml` 在内存中选择配方。直接配置模式的路径参数见各命令帮助。策略客户端使用 `--policy-endpoint HOST:PORT` 连接指定的 gRPC 服务，其他业务参数在 Runtime 配方中修改。

MCAP 录制使用 `zstd_fast` 原生无损块压缩；消息字节、主题和时间戳保持完整，离线读取与审计入口相同。

示教与推理仿真后端在 `mujoco` 容器启动，用法见 [脚本入口与使用指南](../../docs/features/脚本入口与使用指南.md)。

## 目录结构

- `ros/dual_arm_runtime/`：`dual_arm_runtime` 核心 Python 包。
- `configs/`：运行端环境变量 `container.env`。
- `docker/`：`Dockerfile` 与 `docker-compose.yml`。
- `scripts/`：容器内单职责入口脚本。
