# MuJoCo 配置说明

| 目录/文件 | 用途说明 |
| --- | --- |
| [`backend/`](backend/README.md) | 后端启动配方模板（组合场景、控制参数、Profile 与任务插件） |
| [`scene/`](scene/README.md) | 双 Panda 物理场景几何、学习相机视场变体与堆叠任务阈值 |
| [`checks/`](checks/README.md) | 三相机静态可见性门限与关键位姿运动学净空检查参数 |
| `container.env` | `simulation` 容器内运行路径、ROS 2 域 ID 与 NVIDIA EGL 环境变量 |

- 跨端共享的控制限值、受限跟随与流式超时配置集中维护在 [`profiles/control/`](../../../profiles/control/README.md)。
- 历史回归对照专用的后端配置夹具存放于 [`tests/fixtures/simulation/`](../../../tests/fixtures/simulation/README.md)。

目录导航：[上级目录](../README.md)
