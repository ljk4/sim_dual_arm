# 仿真端命令入口（Scripts）

`run.sh` 是 `simulation` 容器内的唯一命令分发脚本，自动加载 `shared/common/tools/docker/setup.sh` 并调用对应的 Python 模块：

| 子命令 | 功能说明 | 默认输出路径 |
| --- | --- | --- |
| `environment` | 检查 MuJoCo 依赖、物理步进、DAQP 与 NVIDIA EGL 离屏渲染 | `/data/_system/diagnostics/environment/<run-id>/` |
| `scene` | 检查 `panda_stack` 双 Panda 场景在 67 个布局下的三路相机静态可见性 | `/data/panda_stack/diagnostics/scene/<run-id>/` |
| `marvin` | 编译并检查 `marvin_handover` 的 Marvin 场景：关节绑定、碰撞、夹爪开度与三路渲染 | `/data/marvin_handover/diagnostics/marvin/<run-id>/` |
| `kinematics` | 检查 `panda_stack` 堆叠任务关键位姿 IK 可达性与路径碰撞净空 | `/data/panda_stack/diagnostics/kinematics/<run-id>/` |
| `handover` | 生成并检查 `marvin_handover` 的抓取、空中交接与入箱候选路径 | `/data/marvin_handover/diagnostics/handover/<run-id>/` |
| `backend` | 按 `--scenario` 或 `--config` 加载配方，校验 `/opt/ros-dev` SDK 后启动 MuJoCo ROS 2 物理仿真后端 | `/data/<scenario-id>/logs/backend/<run-id>/`（direct 模式 `_direct`） |
| `viewer` | 订阅 `/robot/visualization/mujoco_state` 快照，通过宿主机 X11 打开只读实时 GUI；支持 `--scenario` | 实时窗口显示 |
| `test` | 运行固定禁用自动插件收集的 `pytest` 测试套件 | 终端输出 |

`backend` / `viewer` 必须且只能指定 `--scenario <id>` 或 `--config <recipe>` 之一：

- 方案模式额外接受 `--role teacher|policy`、`--shared-root`、`--bundle`（仅 policy）与 `--seed`；
- 后端在内存中解析并展开配方，把本次实际生效的快照写到 `<run-root>/<run-id>/backend.yaml`，日志写到同目录 `log/`；
- 所有入口支持重复 `--set dot.path=value`，只覆盖配方中已存在的键；
- 不生成 `/data/prepared/` 等中间目录。

```bash
# 直接以方案启动示教后端
scripts/run.sh backend --scenario panda_stack --role teacher \
  --seed 0 --run-id teacher-backend

# 直接以方案启动推理后端
scripts/run.sh backend --scenario panda_stack --role policy \
  --bundle /data/panda_stack/models/bundles/stack-act \
  --seed 20000 --run-id policy-backend --set run.realtime_speed=0.4

# 直接配置模式
scripts/run.sh backend \
  --config /shared/scenarios/panda_stack/configs/simulation/backend/backend_stack_teacher.yaml

# 只读 viewer
scripts/run.sh viewer --scenario panda_stack --role teacher
```

- 所有报告目录与后端运行目录均禁止覆盖已存在路径。
- 在宿主机直接运行 `run.sh` 会报错退出（返回码 2），请先进入 `simulation` 容器执行。

目录导航：[上级目录](../README.md)
