# marvin/mujoco/handover

Marvin 固定根双臂方案：左手抓取物块、空中交接给右手、再放入开口箱。本目录保存方案选择、任务级机器人档案以及仿真/运行/学习配方；三端实现仍由 `simulation/`、`runtime/`、`learning/` 提供。

当前复位使用固定初始布局，`seed` 标识运行而不随机化物块或箱位。训练/验证划分与多次推理评估衡量该固定布局下的表现。

## 已有配置与档案

| 路径 | 内容 |
| --- | --- |
| `binding.yaml` | 三端方案、档案、配方与 Teacher 扩展包选择 |
| `configs/simulation/scene/marvin_handover.yaml` | 机器人装配选择、任务环境、执行器/复位数值和相机安装 |
| `configs/simulation/backend/backend_marvin_teacher.yaml` | 任务插件与 Teacher 后端（`task_type: handover`、`task_mode: teacher`），启用分割质量计数 |
| `configs/simulation/backend/backend_marvin_policy.yaml` | Policy 推理后端（`task_mode: inference`），运行同一物理评价 |
| `configs/simulation/scene/marvin_handover_task.yaml` | 阶段门禁、终态评价、180 s 执行预算、Teacher 时钟与容量 |
| `configs/simulation/scene/marvin_handover_task_inference.yaml` | 完整任务推理评价：Teacher 关闭、180 s 仿真 / 420 s 墙钟、同一门禁与成功判定 |
| `configs/simulation/scene/handover_paths.yaml` | 任务目标、抓持点、接近/退避位、协调抬升、执行时长与诊断阈值 |
| `configs/runtime/teacher_collect.yaml` | runtime Teacher 采集配方（180 s 任务预算、能力标识） |
| `configs/runtime/policy_eval.yaml` | runtime Policy 执行配方（180 s 任务预算、1 s 停止余量、420 s 墙钟） |
| `configs/learning/act_dataset.yaml` | 正式数据转换，50 条合格局，随机抽取 10% 作为验证集 |
| `configs/learning/act_training.yaml` | ACT full 5000 步、训练日志与模型路径 |
| `configs/learning/policy.yaml` | ACT Policy 服务；首轮关闭时间融合，为 `left_joint7` 与两夹爪配置逐轴修正额度 |
| `configs/runtime/handover_campaign.yaml` | 正式采集预算（50 条 / 最多 100 次尝试）、种子、台账、阶段质量规则、导出标签上限与独立路径 |
| `../../../../robots/marvin/extensions/ros/marvin_teacher_api_msgs/` | Marvin 共享 60×16 Teacher IDL、能力标识与包构建文件 |
| `../../../../robots/marvin/profiles/control/marvin_control.yaml` | 16 维控制轴、跟踪容差、停止与接触参数 |
| `profiles/control/marvin_stream.yaml` | 连续块流的提前量、观测年龄与超时 |
| `../../../../robots/marvin/profiles/control/marvin_follower.yaml` | Teacher 有界跟随窗口、参考偏差预算与终端尾长 |
| `../../../../robots/marvin/profiles/control/marvin_follower_inference.yaml` | ACT 推理窗口与逐轴参考偏差上限，独立于关节硬限位 |
| `profiles/robot/profile.json` | Robot Profile，摘要根据场景声明计算 |
| `../../../../robots/marvin/profiles/observations/marvin_rgb_joint.json` | 三路 RGB + 16 维机器人状态 |
| `../../../../robots/marvin/profiles/actions/marvin_joint_position.json` | 16 维关节位置动作 |

Marvin 原始 URDF/STL 位于 [`shared/robots/marvin/`](../../../../robots/marvin/README.md)，与 `marvin/mujoco/stack` 共用其 `profiles/` 与扩展包。

## 选择方式与公开契约

`configs/simulation/scene/marvin_handover.yaml` 通过 `robot.assembly: marvin` 与 `task.environment: handover` 注册仿真后端。接口包为 `robot_api_msgs`、`sim_api_msgs` 与 `marvin_teacher_api_msgs`（能力标识 `marvin_teacher_api_msgs.teacher_chunk`），由 `runtime` 容器执行一次 `scripts/build_sdk.sh` 统一构建至 `output/build/`：

- 控制轴顺序：左臂 7 关节、左夹爪总开度、右臂 7 关节、右夹爪总开度（臂关节 `rad`，夹爪总开度 `0.0014 .. 0.0814 m`）。
- 关节名映射：`Arm_L{1..7}_Joint` → `left_joint{1..7}`，`Arm_R{1..7}_Joint` → `right_joint{1..7}`。

## 操作入口

使用 `--robot marvin --backend mujoco --task handover` 选择本方案，或用 `--config` 指定配方：

```bash
# mujoco 容器：启动 Teacher 后端
scripts/backend.sh --robot marvin --backend mujoco --task handover --role teacher \
  --seed 0 --run-id teacher-backend

# runtime 容器：试采与查看状态
scripts/record.sh --robot marvin --backend mujoco --task handover \
  --purpose pilot --count 1 --max-consecutive-failures 1
scripts/record.sh status --robot marvin --backend mujoco --task handover

# 宿主机项目根目录：并行补足 50 条正式数据
./record_parallel.sh --robot marvin --backend mujoco --task handover \
  --purpose formal --target-total 50 --workers auto
```

正式采集达到 campaign 目标时，Runtime 将有序合格局写入 `output/marvin/mujoco/handover/datasets/raw/formal-select.json`。

```bash
# lerobot 容器：转换正式数据（45/5），训练 5000 步
scripts/convert.sh --robot marvin --backend mujoco --task handover
scripts/train.sh --robot marvin --backend mujoco --task handover --mode full
```

训练将 `model.safetensors` 与 `training_args.yaml` 写入 `output/marvin/mujoco/handover/models/act_<时间戳>/`。

```bash
# lerobot 容器：从 models/ 选择最新模型，可用 --model 指定目录
scripts/server_policy.sh --robot marvin --backend mujoco --task handover

# mujoco 容器：推理后端（不使用 Teacher）
scripts/backend.sh --robot marvin --backend mujoco --task handover --role policy \
  --seed 5100 --run-id policy-backend

# runtime 容器：交互式一局或非交互式单局
scripts/client_interactive.sh --robot marvin --backend mujoco --task handover --seed 5100
scripts/client_policy.sh --robot marvin --backend mujoco --task handover \
  --reset-seed 5100 --run-id policy-run

# 宿主机根目录：批量推理
./inference_parallel.sh --robot marvin --backend mujoco --task handover \
  --count 10 --seed-start 5100
```

目录导航：[公共层与方案绑定](../../../../README.md) · [根 README](../../../../../README.md)
