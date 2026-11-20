# MuJoCo 物理仿真后端

实现 Marvin 双臂（`marvin_sim/stack` 与 `marvin_sim/handover`）的物理仿真、EGL 离屏相机渲染、Ruckig 轨迹跟随、状态发布与程序示教（Teacher），通过 ROS 2 Robot API 与运行端通信。

## 目录结构

| 目录 | 说明 |
| :--- | :--- |
| [`src/robot_backend_sim/`](src/robot_backend_sim/README.md) | 物理引擎封装、运动控制、渲染线程、任务插件与 ROS 2 节点 |
| [`assets/`](assets/README.md) | 机器人模型与网格资源（`marvin/`） |
| [`configs/`](configs/README.md) | 仿真端环境变量 `container.env` |
| [`docker/`](docker/README.md) | `Dockerfile`、`docker-compose.yml` 与依赖清单 |
| [`scripts/`](scripts/README.md) | 容器内单职责脚本：`backend.sh`、`viewer.sh`、`test.sh` |

## 快速使用

```bash
# 宿主机：启动并进入仿真容器
./env.sh up simulation
./env.sh enter simulation

# 宿主机：打开实时只读 3D 视图（需先启动 backend）
./env.sh viewer --robot marvin_sim --task stack --role teacher
```

详细操作见 [`docker/README.md`](docker/README.md) 与 [`scripts/README.md`](scripts/README.md)。
