# MuJoCo 物理仿真后端

实现双臂机器人在 MuJoCo 中的物理仿真（如 `marvin/mujoco/stack` 与 `marvin/mujoco/handover`）、EGL 离屏相机渲染、轨迹校验与跟随、状态发布和程序示教（Teacher）。运行端通过 ROS 2 Robot API 提交分段轨迹；仿真端校验后执行，并发布实际生效参考和反馈。URDF 与网格等机器人本体资产统一从 [`shared/robots/`](../../shared/robots/marvin/README.md) 加载。

## 目录结构

| 目录 | 说明 |
| :--- | :--- |
| `src/robot_backend_sim/` | 物理引擎封装、运动控制、渲染线程、`common/` 通用机制与 `<robot>/<task>` 任务插件 |
| `configs/` | 仿真端环境变量 `container.env` |
| `docker/` | `Dockerfile` 与 `docker-compose.yml` |
| `scripts/` | 容器内单职责脚本：`backend.sh`（及薄封装 `teacher_backend.sh`、`inference_backend.sh`）、`viewer.sh`、`test.sh` |

## 快速使用

```bash
# 宿主机：启动并进入 MuJoCo 容器
./env.sh up mujoco
./env.sh mujoco

# 宿主机：打开实时只读 3D 视图（需先启动 backend）
./env.sh viewer --robot marvin --backend mujoco --task stack
```

更多用法见 [脚本入口与使用指南](../../docs/features/脚本入口与使用指南.md)。
