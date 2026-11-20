# 仿真端命令入口（Scripts）

本目录提供 `simulation` 容器内的单职责入口脚本，均自动加载 `shared/common/tools/docker/setup.sh`：

| 脚本 | 功能说明 | 默认输出路径 |
| --- | --- | --- |
| `backend.sh` | 按 `--robot/--task` 或 `--config` 加载配方，校验 `/opt/ros-dev` SDK 后启动 MuJoCo ROS 2 物理仿真后端 | `/data/<robot>/<task>/logs/backend/<run-id>/`（direct 模式 `_direct`） |
| `teacher_backend.sh` | 显式指定 `--robot <id> --task <id>`，启动 Teacher 后端；终端只显示关键参数，详细参数写入 `<run-root>/<run-id>-startup-parameters.yaml` | `/data/<robot>/<task>/logs/backend/<run-id>/` |
| `inference_backend.sh` | 显式指定 `--robot <id> --task <id>`，启动推理后端（仿真端不加载模型权重，无需 Bundle）；详细参数写入 `<run-root>/<run-id>-startup-parameters.yaml` | `/data/<robot>/<task>/logs/backend/<run-id>/` |
| `viewer.sh` | 订阅 `/robot/visualization/mujoco_state` 快照，通过宿主机 X11 打开只读实时 GUI；支持 `--robot/--task` | 实时窗口显示 |
| `test.sh` | 运行固定禁用自动插件收集的 `pytest` 测试套件 | 终端输出 |

`backend.sh` / `viewer.sh` 必须且只能指定 `--robot <id> --task <id>` 或 `--config <recipe>` 之一：

- `--robot/--task` 模式额外接受 `--role teacher|policy`、`--shared-root` 与 `--seed`；
- 后端在内存中解析并展开配方，把本次实际生效的快照写到 `<run-root>/<run-id>/backend.yaml`，日志写到同目录 `log/`；
- 所有入口支持重复 `--set dot.path=value`，只覆盖配方中已存在的键；
- 不生成 `/data/prepared/` 等中间目录。

```bash
# 按机器人/任务启动示教后端
scripts/backend.sh --robot marvin_sim --task stack --role teacher \
  --seed 0 --run-id teacher-backend

# 按机器人/任务启动推理后端
scripts/backend.sh --robot marvin_sim --task stack --role policy \
  --seed 20000 --run-id policy-backend --set run.realtime_speed=0.4

# 直接配置模式
scripts/backend.sh \
  --config /shared/scenarios/marvin_sim/stack/configs/simulation/backend/backend_stack_teacher.yaml

# 只读 viewer（宿主机用 ./env.sh viewer --robot marvin_sim --task stack --role teacher）
scripts/viewer.sh --robot marvin_sim --task stack --role teacher
```

- 所有后端运行目录均禁止覆盖已存在路径。
- 在宿主机直接运行这些脚本会报错退出（返回码 2），请先进入 `simulation` 容器执行。

目录导航：[上级目录](../README.md)
