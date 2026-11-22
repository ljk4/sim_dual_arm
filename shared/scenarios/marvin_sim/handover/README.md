# marvin_sim/handover

Marvin 固定根双臂方案：左手抓取物块、空中交接给右手、再放入开口箱。本目录保存方案选择、机器人/观测/动作档案、仿真/运行/学习配方与方案专用 Teacher 接口；三端实现仍由 `simulation/`、`runtime/`、`learning/` 提供。

当前复位使用固定初始布局，`seed` 标识运行而不随机化物块或箱位。训练/验证划分与多次推理评估衡量该固定布局下的表现；不同种子不代表不同初始布局。

## 已有配置与档案

| 路径 | 内容 |
| --- | --- |
| `binding.yaml` | 三端方案、档案、配方与 Teacher 扩展包选择 |
| `configs/simulation/scene/marvin_handover.yaml` | 机器人装配选择、任务环境、执行器/复位数值和相机安装 |
| `configs/simulation/backend/backend_marvin_teacher.yaml` | 任务插件与 Teacher 后端（`task_type: handover`、`task_mode: teacher`），启用分割质量计数 |
| `configs/simulation/backend/backend_marvin_policy.yaml` | Policy 推理后端（`task_mode: inference`），只运行同一物理评价 |
| `configs/simulation/scene/marvin_handover_task.yaml` | 阶段门禁、终态评价、180 s 执行预算、Teacher 时钟与容量 |
| `configs/simulation/scene/marvin_handover_task_inference.yaml` | 完整任务推理评价：Teacher 关闭、180 s 仿真 / 420 s 墙钟、同一门禁与成功判定 |
| `configs/simulation/scene/handover_paths.yaml` | 任务目标、抓持点、接近/退避位、协调抬升、执行时长与诊断阈值 |
| `configs/runtime/teacher_collect.yaml` | runtime Teacher 采集配方（180 s 任务预算、能力标识） |
| `configs/runtime/policy_eval.yaml` | runtime Policy 执行配方（180 s 任务预算、1 s 停止余量、420 s 墙钟） |
| `configs/learning/act_dataset.yaml` | 正式数据转换，50 条合格局，随机抽取 10% 作为验证集 |
| `configs/learning/act_training.yaml` | ACT full 5000 步、训练日志与模型路径 |
| `configs/learning/policy.yaml` | ACT Policy 服务；首轮关闭时间融合，按实测为 `left_joint7` 与两夹爪配置逐轴修正额度 |
| `configs/runtime/handover_campaign.yaml` | 正式采集预算（50 条 / 最多 100 次尝试）、种子、台账、阶段质量规则、导出标签上限与独立路径 |
| `../extensions/ros/marvin_teacher_api_msgs/` | `marvin_sim` 共享 60×16 Teacher IDL、能力标识与包构建文件 |
| `../profiles/control/marvin_control.yaml` | 16 维控制轴、实测跟踪容差、停止与接触参数 |
| `profiles/control/marvin_stream.yaml` | 连续块流的提前量、观测年龄与超时 |
| `../profiles/control/marvin_follower.yaml` | Teacher 有界跟随窗口、参考偏差预算与终端尾长 |
| `../profiles/control/marvin_follower_inference.yaml` | ACT 推理窗口与逐轴参考偏差上限，独立于关节硬限位 |
| `profiles/robot/` | Robot Profile（`profile.json`）。Profile 摘要根据场景声明计算。 |
| `../profiles/observations/marvin_rgb_joint.json` | 三路 RGB + 16 维机器人状态 |
| `../profiles/actions/marvin_joint_position.json` | 16 维关节位置动作 |

Marvin 的原始 URDF/STL 位于仿真端 [`assets/marvin/`](../../../../simulation/mujoco/assets/marvin/README.md)。堆叠与交接方案共用 `../profiles/` 下的公共控制、跟随、观测与动作档案。

## 选择方式

`configs/simulation/scene/marvin_handover.yaml` 中两个字段分别选择能力和任务：

```yaml
robot:
  assembly: marvin
task:
  environment: handover
```

仿真端根据 `robot.assembly` 定位模型，根据 `task.environment` 选择任务环境。binding 的 `recipes.dataset`、`recipes.training`、`recipes.policy_service`、`recipes.runtime_policy` 与 `recipes.simulation_policy` 指向本方案正式配方。

本方案接口包为 `robot_api_msgs`、`sim_api_msgs` 与 `marvin_teacher_api_msgs`（能力标识 `marvin_teacher_api_msgs.teacher_chunk`），已包含在全量 SDK 构建中；一次 `scripts/build_sdk.sh` 即服务全部方案：

```bash
# runtime 容器
scripts/build_sdk.sh
```

## 公开契约

- 顺序：左臂 7 关节、左夹爪总开度、右臂 7 关节、右夹爪总开度。
- 臂关节单位 `rad`，夹爪总开度单位 `m`；总开度为指尖最小孔径。mesh 指面沿指长有渐变，不表示整面平行间距。
- 关节限位来自源 URDF；夹爪总开度范围由源码网格和编译模型测量，开端 `0.0814 m`、闭端 `0.0014 m`，档案/检查容差 `5e-4 m`。
- 原始关节名到公开名称的映射：`Arm_L{1..7}_Joint` → `left_joint{1..7}`，`Arm_R{1..7}_Joint` → `right_joint{1..7}`。
- 夹爪执行器控制量为两指闭合量（`0 .. 0.08 m`）；总开度与闭合量之和为开端测量值。名义 TCP 位于机器人适配指定的手部站点。
- 手指联动来自源 URDF mimic，MuJoCo 导入为每侧一条 joint equality；编译模型 `neq=2`，没有重复的手工约束。
- 同臂碰撞排除只有 `Arm_*5_Link`/`Arm_*7_Link`；左右臂、同臂其余配对和手指碰撞保持有效。底座网格仅渲染，适用边界见资产说明。

## 操作入口

使用 `--robot marvin_sim --task handover` 从本目录的 binding 选择配方，或用 `--config` 指定配方。

### 采集

使用条数、用途和仿真倍速等专用参数。其他采集参数在相应配方中修改。

```bash
# simulation 容器：直接启动 Teacher 后端
scripts/backend.sh --robot marvin_sim --task handover --role teacher \
  --seed 0 --run-id teacher-backend

# runtime 容器：使用已启动的 Teacher 后端试采
scripts/record.sh --robot marvin_sim --task handover \
  --purpose pilot --count 1 --max-consecutive-failures 1

# 宿主机项目根目录：并行补足 50 条正式数据
./record_parallel.sh --robot marvin_sim --task handover \
  --purpose formal --target-total 50 --workers auto

# runtime 容器：查看采集与筛选状态
scripts/record.sh status --robot marvin_sim --task handover
```

正式采集达到 campaign 目标时，Runtime 将有序合格局写入 `var/marvin_sim/handover/datasets/raw/formal-select.json`。质量规则按本任务阶段声明：左手抓取看左腕/全局，空中交接看物块与左右腕，运输/放置看右腕/全局与箱口；`left_grasp`、`dual_hold`、`left_release`、`right_release` 为必需阶段，缺失即导出不合格。

### 数据转换与训练

```bash
# learning 容器：转换正式数据（45/5），训练 5000 步
scripts/convert.sh --robot marvin_sim --task handover
scripts/train.sh --robot marvin_sim --task handover --mode full
```

Learning 转换器按 `act_dataset.yaml` 的 `validation_percent` 从 50 条合格局随机抽取 10% 作为验证集。训练将 `model.safetensors` 和 `training_args.yaml` 写入 `/data/marvin_sim/handover/models/act_<时间戳>/`。

### 推理

```bash
# learning 容器：从 models/ 选择最新模型，可用 --model 指定目录
scripts/server_policy.sh --robot marvin_sim --task handover

# simulation 容器：推理后端（不使用 Teacher）
scripts/backend.sh --robot marvin_sim --task handover --role policy \
  --seed 5100 --run-id policy-backend

# runtime 容器：交互式一局或非交互式单局
scripts/client_interactive.sh --robot marvin_sim --task handover \
  --seed 5100 --run-id policy-interactive
scripts/client_policy.sh --robot marvin_sim --task handover \
  --reset-seed 5100 --run-id policy-run

# 宿主机根目录：学习端策略服务在线时运行批量推理
./inference_parallel.sh --robot marvin_sim --task handover \
  --count 10 --seed-start 5100
```

本轮 evaluation 种子为 5100–5219，累计 120 次。失败、拒绝和取消同样占用次数。任务使用 180 s 仿真预算、1 s 停止余量和 420 s 墙钟预算，每批最多 10 次。验收要求至少一次交接与入箱成功，并从录制核对左手抬升、空中共持、左手释放后右手持稳及入箱完成。

目录导航：[公共层与方案绑定](../../../README.md) · [根 README](../../../../README.md)
