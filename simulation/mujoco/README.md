# MuJoCo 仿真后端

实现双 Panda 物理步进、三相机渲染、受限动作执行、实测反馈与程序示教，通过
ROS 2 Robot API 与运行端通信。

| 目录 | 内容 |
| --- | --- |
| `src/robot_backend_sim/` | 物理、控制、渲染与 ROS 后端实现 |
| `assets/franka_emika_panda/` | 随项目挂载的 Panda 模型与网格 |
| `configs/` | 场景、控制、任务与检查配置 |
| `docker/` | 依赖镜像、安装锁、固定 wheel 与 simulation 服务 |
| `scripts/` | 容器内 `run.sh` 与 backend 配置准备 |

首次使用从 [Docker 构建与运行说明](docker/README.md) 开始：

```bash
# 宿主：
cd simulation/mujoco/docker
docker compose build
docker compose up -d
docker compose exec simulation bash
# simulation 容器内：
scripts/run.sh environment --run-id env-001
```

`run.sh` 提供 `environment`、`scene`、`kinematics`、`prepare`、`backend`。
backend 需要 runtime 生成的 ROS SDK；仿真检查可以独立运行。
模型随本目录挂到 `/app/assets/`，报告与运行数据写到 `/data`；
完整采集与任务记录由运行端负责。命令参数见[脚本说明](scripts/README.md)。

目录导航：[上级目录](../README.md)。
