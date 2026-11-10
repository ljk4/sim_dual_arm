# 仿真端命令入口（Scripts）

`run.sh` 是 `simulation` 容器内的唯一命令分发脚本，自动加载 `/config/setup.sh` 并调用对应的 Python 模块：

| 子命令 | 功能说明 | 默认输出路径 |
| --- | --- | --- |
| `environment` | 检查 MuJoCo 依赖、单 Panda 物理步进、DAQP 与 NVIDIA EGL 离屏渲染 | `/data/reports/sim-check/<run-id>/` |
| `scene` | 检查双 Panda 场景在 67 个布局下的三路相机静态可见性 | `/data/reports/sim-check/<run-id>/` |
| `kinematics` | 检查堆叠任务关键位姿 IK 可达性与路径碰撞净空 | `/data/reports/sim-check/<run-id>/` |
| `prepare` | 从后端模板、随机种子与 `run-id` 生成 `backend.yaml` 与 `run_manifest.json`（`prepare_backend.py`） | `--output` 指定的全新目录 |
| `backend` | 校验 `/opt/ros-dev` SDK 后启动 MuJoCo ROS 2 物理仿真后端 | `/data/runs/backend/<run-id>/` |
| `viewer` | 订阅 `/robot/visualization/mujoco_state` 快照，通过宿主机 X11 打开只读实时 GUI | 实时窗口显示 |
| `test` | 运行固定禁用自动插件收集的 `pytest` 测试套件 | 终端输出 |

- 所有报告目录与 `prepare` 输出目录均禁止覆盖已存在路径。
- 在宿主机直接运行 `run.sh` 会报错退出（返回码 2），请先进入 `simulation` 容器执行。

目录导航：[上级目录](../README.md)
