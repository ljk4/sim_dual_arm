# MuJoCo 物理仿真后端

实现 Marvin 双臂（`marvin_sim/stack` 与 `marvin_sim/handover`）的物理仿真、EGL 离屏相机渲染、轨迹校验与跟随、状态发布和程序示教（Teacher）。运行端通过 ROS 2 Robot API 提交分段轨迹；仿真端校验后执行，并发布实际生效参考和反馈。

## 目录结构

| 目录 | 说明 |
| :--- | :--- |
| `src/robot_backend_sim/` | 物理引擎封装、运动控制、渲染线程、任务插件与 ROS 2 节点 |
| `assets/` | 机器人模型与网格资源（[`marvin/`](assets/marvin/README.md)） |
| `configs/` | 仿真端环境变量 `container.env` |
| `docker/` | `Dockerfile` 与 `docker-compose.yml` |
| `scripts/` | 容器内单职责脚本：`backend.sh`（及薄封装 `teacher_backend.sh`、`inference_backend.sh`）、`viewer.sh`、`test.sh` |

## 快速使用

```bash
# 宿主机：启动并进入仿真容器
./env.sh up simulation
./env.sh enter simulation

# 宿主机：打开实时只读 3D 视图（需先启动 backend）
./env.sh viewer --robot marvin_sim --task stack
```

更多用法见 [脚本入口与使用指南](../../docs/features/脚本入口与使用指南.md)。
