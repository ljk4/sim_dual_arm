# 仿真脚本

`run.sh` 是进入 simulation 容器后手动执行的功能脚本。
它复用公共 `infrastructure/docker/setup.sh` 加载 ROS 与源码路径，然后调用真实
Python CLI，参数原样传递。

| 命令 | 功能 | 主要输出 |
| --- | --- | --- |
| `environment` | 依赖、单 Panda 物理、DAQP 与 NVIDIA EGL 检查 | `/data/reports/sim-check/<run-id>/` |
| `scene` | 双 Panda 场景三相机可见性检查 | 同上 |
| `kinematics` | 关键位姿与运动学净空检查 | 同上 |
| `prepare` | 从模板、seed、run-id 生成 backend 配置 | 显式 `--output` 下的 `backend.yaml` 和 `run_manifest.json` |
| `backend` | 运行 MuJoCo ROS 后端，启动前检查 ROS SDK | `/data/runs/backend/<run-id>/` |

在宿主进入 `simulation/mujoco/docker` 并运行 `docker compose exec simulation bash`，再执行：

```bash
scripts/run.sh --help
scripts/run.sh scene --help
```

`prepare_backend.py` 负责配置准备；Panda 模型直接位于 `../assets/franka_emika_panda/`。
场景、控制与物理参数由 `configs/` 提供。backend 必须显式指定 `--config` 和
`--run-id`；scene/kinematics 的默认 YAML 在 `run.sh` 中选择，可用 CLI 参数覆盖。

模型通过仿真项目的 `/app:ro` 挂载进入容器。报告和准备输出使用独立目录，
已存在即拒绝覆盖。
ROS SDK 由 runtime 构建并挂到 `/opt/ros-dev`，仅 backend 启动时检查；
仿真构建、检查与容器空闲启动不要求预先生成 SDK。

构建、挂载和完整启动顺序见 [Docker 说明](../docker/README.md)。

目录导航：[上级目录](../README.md)。
