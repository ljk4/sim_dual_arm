# 仿真端命令入口（Scripts）

本目录提供 `simulation` 容器内的单职责入口脚本，均自动加载 `shared/common/tools/docker/setup.sh`：

| 脚本 | 功能说明 | 默认输出路径 |
| --- | --- | --- |
| `backend.sh` | 按 `--robot/--task` 或 `--config` 加载配方，校验 `/opt/ros-dev` SDK 后启动 MuJoCo ROS 2 物理仿真后端，就绪后写入 `ready.json` | `/data/<robot>/<task>/logs/backend/<run-id>/`（direct 模式 `_direct`） |
| `teacher_backend.sh` | 显式透传 `--role teacher --seed 0 --run-id teacher-backend` 给 `backend.sh` 启动 Teacher 后端 | `/data/<robot>/<task>/logs/backend/<run-id>/` |
| `inference_backend.sh` | 显式透传 `--role policy --seed 20000 --run-id policy-backend` 给 `backend.sh` 启动推理后端（仿真端不加载模型权重） | `/data/<robot>/<task>/logs/backend/<run-id>/` |
| `viewer.sh` | 订阅 `/robot/visualization/mujoco_state` 快照，通过宿主机 X11 打开只读实时 GUI；支持 `--robot/--task` | 实时窗口显示 |
| `test.sh` | 运行固定禁用自动插件收集的 `pytest` 测试套件 | 终端输出 |

`backend.sh` / `viewer.sh` 必须且只能指定 `--robot <id> --task <id>` 或 `--config <recipe>` 之一：

- `--robot/--task` 模式接受 `--role teacher|policy` 与 `--shared-root`。
- 后端在内存中解析并展开配方，把本次实际生效的快照写到 `<run-root>/<run-id>/backend.yaml`，日志写到同目录 `log/`，节点就绪时写入 `<run-root>/<run-id>/ready.json`。
- 后端支持 `--seed`、`--realtime-speed`、`--namespace`（覆盖 ROS 命名空间）与 `--no-visualization`（禁用可视化快照话题），亦可使用 `--run-root` 与 `--run-id` 指定输出目录。其他参数在对应的后端配方中修改。

```bash
# 按机器人/任务启动示教后端
scripts/backend.sh --robot marvin_sim --task stack --role teacher \
  --seed 0 --run-id teacher-backend --realtime-speed 0.4

# 按机器人/任务启动推理后端
scripts/backend.sh --robot marvin_sim --task stack --role policy \
  --seed 20000 --run-id policy-backend --realtime-speed 0.4

# 直接配置模式
scripts/backend.sh \
  --config /shared/scenarios/marvin_sim/stack/configs/simulation/backend/backend_stack_teacher.yaml

# 只读 viewer（宿主机用 ./env.sh viewer --robot marvin_sim --task stack --role teacher）
scripts/viewer.sh --robot marvin_sim --task stack --role teacher
```

- 所有后端运行目录均禁止覆盖已存在路径。
- 在宿主机直接运行这些脚本会报错退出（返回码 2），请先进入 `simulation` 容器执行。

目录导航：[上级目录](../README.md)
