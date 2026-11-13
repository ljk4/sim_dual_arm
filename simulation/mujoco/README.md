# MuJoCo 物理仿真后端

实现双 Panda 与 Marvin 双臂的物理步进、三路无窗口 EGL 相机渲染、Ruckig 有界跟随动作执行、实测反馈发布与程序示教（Teacher），通过 ROS 2 Robot API 与运行端通信。

本端持有 MuJoCo 物理、渲染、任务插件与执行映射。公共消息定义在 [`shared/common/interfaces/ros/`](../../shared/common/interfaces/ros/README.md)；方案档案、场景配方、Teacher 扩展与实现选择分别在 [`shared/scenarios/panda_stack/`](../../shared/scenarios/panda_stack/README.md)（Panda 堆叠）与 [`shared/scenarios/marvin_handover/`](../../shared/scenarios/marvin_handover/README.md)（Marvin 交接），准备入口按 binding 选择配方，不在本端写方案名分支。

## 目录结构

| 目录 | 职责内容 |
| --- | --- |
| [`src/robot_backend_sim/`](src/robot_backend_sim/README.md) | 物理引擎封装、运动控制、渲染线程、任务插件与 ROS 2 节点实现 |
| [`assets/`](assets/README.md) | 本端持有的机器人模型与网格：`franka_emika_panda/`（Menagerie）和 `marvin/`（原始 URDF/STL） |
| [`configs/`](configs/README.md) | 仿真端私有环境变量 `container.env` 与说明；数值配方位于 `shared/scenarios/<id>/configs/simulation/` |
| [`docker/`](docker/README.md) | 仿真容器 `Dockerfile`、`docker-compose.yml`、`requirements.lock` 与固定 Ruckig wheel |
| [`scripts/`](scripts/README.md) | 容器内统一命令入口 `run.sh`；`backend`/`viewer` 直接按 `--scenario` 或 `--config` 加载配方 |

## 快速启动

```bash
# 宿主机：构建并进入容器
cd simulation/mujoco/docker
docker compose build
docker compose up -d
docker compose exec simulation bash

# 容器内：执行环境与渲染检查
scripts/run.sh environment --run-id env-001
```

详细构建、挂载与运行说明见 [`docker/README.md`](docker/README.md) 与 [`scripts/README.md`](scripts/README.md)。

目录导航：[上级目录](../README.md)
