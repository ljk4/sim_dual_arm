# marvin_handover

Marvin 固定根双臂交接方案。P02A～P06 已交付：模型/标准控制、真实 Teacher 全程交接、录制与 6 条合格诊断数据（4/2 划分）、128 step 新模型与 bundle 的 Predict/后端执行/收尾，以及最终证据、职责与命令核对。Teacher 物理任务连续 5 次成功；短训练策略未完成交接，推理专用轨迹保真容差及模型效果限制见[P05 交付](../../../docs/marvin-handover/deliveries/phase-05.md)，最终验收与使用说明见[P06 交付](../../../docs/marvin-handover/deliveries/phase-06.md)，数据见[P04 交付](../../../docs/marvin-handover/deliveries/phase-04.md)。

## 已有配置与档案

| 路径 | 内容 |
| --- | --- |
| `binding.yaml` | 方案选择：仿真/mujoco/handover、runtime/ros2、Teacher 能力、三份配方与扩展包 |
| `configs/simulation/scene/marvin_handover.yaml` | 机器人装配选择、任务环境、执行器/复位数值和相机安装 |
| `configs/simulation/backend/backend_marvin_control.yaml` | 无任务插件的 direct backend 配方；`task`/`task_mode` 省略 |
| `configs/simulation/backend/backend_marvin_teacher.yaml` | 任务插件与 Teacher 后端（`task_type: handover`、`task_mode: teacher`），启用分割质量计数 |
| `configs/simulation/backend/backend_marvin_policy.yaml` | Policy 推理后端（`task_mode: inference`），只运行同一物理评价 |
| `configs/simulation/scene/marvin_handover_task.yaml` | 阶段门禁、终态评价、180 s 执行预算、Teacher 时钟与容量 |
| `configs/simulation/scene/marvin_handover_task_inference.yaml` | 短时推理评价：Teacher 关闭、10 s 仿真 / 60 s 墙钟、同一门禁与成功判定 |
| `configs/simulation/planning/handover_paths.yaml` | P02B 任务目标、抓持点、接近/退避位、协调抬升、执行时长与诊断阈值 |
| `configs/simulation/checks/handover_right_grasp_miss_task.yaml` | 右手未接稳负向诊断：右闭爪命令宽于物块，其他参数相同 |
| `configs/runtime/teacher_collect.yaml` | runtime Teacher 采集配方（180 s 任务预算、能力标识） |
| `configs/runtime/policy_eval.yaml` | runtime Policy 执行配方（10 s 任务预算、1 s 停止余量、60 s 墙钟） |
| `configs/learning/act_dataset.yaml` | 诊断数据转换：6/4/2、validation `[4,5]`、`diagnostic: true` |
| `configs/learning/act_training.yaml` | ACT smoke 128 step、独立 900 s 预算台账与 checkpoint 路径 |
| `configs/learning/policy.yaml` | ACT Policy 服务；首轮关闭时间融合，按实测限制夹爪修正 |
| `configs/runtime/experiments/handover_campaign.yaml` | P04 诊断采集预算、种子、台账、阶段质量规则、导出标签上限与独立路径 |
| `extensions/ros/marvin_handover_api_msgs/` | 60×16 交接 Teacher IDL、能力标识与包构建文件 |
| `profiles/control/marvin_control_v1.yaml` | 16 维控制轴、实测跟踪容差、停止与接触参数 |
| `profiles/control/marvin_stream_v1.yaml` | 连续块流的提前量、观测年龄与超时 |
| `profiles/control/marvin_follower_v1.yaml` | Teacher 有界跟随窗口、参考偏差预算与终端尾长 |
| `profiles/control/marvin_follower_inference_v1.yaml` | 诊断 ACT 推理专用跟随预算，按本模型实测目标偏离逐轴设置 |
| `profiles/robots/marvin_handover_v1/` | Robot Profile v2、三路相机标定文件；Profile 摘要由本方案场景声明计算 |
| `profiles/observations/marvin_rgb_joint_v1.json` | 三路 RGB + 16 维机器人状态 |
| `profiles/actions/marvin_joint_position_v1.json` | 16 维关节位置动作 |

Marvin 的原始 URDF/STL 归 simulation 的 [`assets/marvin/`](../../../simulation/mujoco/assets/marvin/README.md)；本方案不保存第二份运行时资产。

## 选择方式

`configs/simulation/scene/marvin_handover.yaml` 中两个字段分别选择能力和任务：

```yaml
robot:
  assembly: marvin
task:
  environment: handover
```

装配入口按这两个字段组合机器人装配与任务环境；`task.environment=handover` 不用于推断机器人型号。`robot.assembly` 由 simulation 的资产定位解析为本端固定版本模型，recipe 不写模型文件路径。P05 已将 dataset/training/policy_service、runtime_policy 与 simulation_policy 引用加入 binding；`models.bundle` 指向本任务真实诊断 bundle。

本方案 SDK 包集为 `robot_api_msgs`、`sim_api_msgs` 与 `marvin_handover_api_msgs`。从 Panda 或其他包集切换前，先结束占用 `/opt/ros-dev` 的进程，按本方案重建并核对包集标记，再启动后端。

## 公开契约

- 顺序：左臂 7 关节、左夹爪总开度、右臂 7 关节、右夹爪总开度。
- 臂关节单位 `rad`，夹爪总开度单位 `m`；总开度为指尖最小孔径。mesh 指面沿指长有渐变，不表示整面平行间距。
- 关节限位来自源 URDF；夹爪总开度范围由源码网格和编译模型测量，开端 `0.0814 m`、闭端 `0.0014 m`，档案/检查容差 `5e-4 m`。
- 原始关节名到公开名称的映射：`Arm_L{1..7}_Joint` → `left_joint{1..7}`，`Arm_R{1..7}_Joint` → `right_joint{1..7}`。
- 夹爪执行器控制量为两指闭合量（`0 .. 0.08 m`）；总开度与闭合量之和为开端测量值。名义 TCP 位于机器人适配指定的手部站点。
- 手指联动来自源 URDF mimic，MuJoCo 导入为每侧一条 joint equality；编译模型 `neq=2`，没有重复的手工约束。
- 同臂碰撞排除只有 `Arm_*5_Link`/`Arm_*7_Link`；左右臂、同臂其余配对和手指碰撞保持有效。底座网格仅渲染，适用边界见资产说明。

## 离线检查

在 `simulation` 容器内：

```bash
scripts/run.sh marvin --run-id marvin-p01a
```

报告写入容器 `/data/reports/sim-check/<run-id>/report.json`，渲染图像写入同一运行目录的 `artifacts/renders/`。检查内容包括：宿主机/容器资产路径解析、编译模型形状与命名绑定、源 URDF 质量/主惯量对照、equality 枚举、TCP/指面/开度测量、静置步进、14 个臂关节低幅方向、左右隔离、夹爪三态开度与联动、桌面/物块/开口箱碰撞、同臂/跨臂/桌/箱/手指有效碰撞探针、开口射线和三路渲染。

## P02/P03 接入点

- P02A 已把 `MarvinBinding` 的具名地址接入标准控制：公开顺序左臂、左总开度、右臂、右总开度；`MarvinIO` 负责 qpos/dof/actuator、TCP/指面、开度与闭合量映射。
- P02B 在 simulation 增加 `tasks/handover/` 目标/路径模块、`tasks/handover/cohold.py` 实测抓持基准共持步骤和 `physics/contact_snapshot.py` 一致接触快照；`scripts/run.sh handover` 生成路径报告，五类物理诊断复用这些模块和标准控制。
- P02B 已调整物块长度、箱位/尺寸和接触参数，并测量所持物块包络、双手腕间隙和放置净空；数值与依据见[交付记录](../../../docs/marvin-handover/deliveries/phase-02b.md)。
- `backend_marvin_control.yaml` 通过 direct `prepare --template` 启动；P02B 的 SDK 只需 `robot_api_msgs` 与 `sim_api_msgs`。
- P03 的 `backend_marvin_teacher.yaml` 显式选择 `task_type: handover`、`task_mode: teacher`；任务插件生成不可变目标与门禁事实，Teacher 服务按 `marvin_handover_api_msgs.teacher_chunk.v1` 注册。
- 控制、stream、follower 配方已按真实编译模型和实际执行建立；P02B 已验证真实持物、双臂 IK、路径净空及连续交接入箱的实际执行。
- 当前箱位、物块初态、相机安装、交接区和抓持区由场景与任务/运动学配方声明。
- `READY_POSE` 仅是离线渲染用的诊断位姿，不是任务抓取位姿。

P03 准备与运行（容器内）：

```bash
# runtime 容器
runtime/ros2/scripts/run.sh build-ros-sdk --scenario marvin_handover --shared-root /shared
runtime/ros2/scripts/run.sh prepare --scenario marvin_handover --role teacher \
  --seed 0 --run-id teacher-1 --output /data/prepared/teacher-1

# simulation 容器
scripts/run.sh prepare --scenario marvin_handover --role teacher \
  --seed 0 --run-id sim-teacher-1 --output /data/prepared/sim-teacher-1
scripts/run.sh backend --config /data/prepared/sim-teacher-1/backend.yaml --run-id backend-teacher-1

# runtime 容器：一次完整示教
python3 -m dual_arm_runtime.runtime_node --project-root /workspace \
  --config /data/prepared/teacher-1/runtime.yaml \
  --output /data/raw/teacher-1 --run-id teacher-1 --episode-id teacher-1 --once
```

完整结果、失败入口和预算见[P03 交付记录](../../../docs/marvin-handover/deliveries/phase-03.md)。

P04 采集与筛选（runtime 容器；后端由本方案 simulation 配方启动）：

```bash
runtime/ros2/scripts/record.sh --scenario marvin_handover --purpose pilot \
  --count 1 --max-consecutive-failures 1 --backend-log /data/runs/backend/<run>/log
runtime/ros2/scripts/record.sh --scenario marvin_handover --purpose formal \
  --count 6 --max-consecutive-failures 3 --backend-log /data/runs/backend/<run>/log
runtime/ros2/scripts/run.sh campaign --scenario marvin_handover status
runtime/ros2/scripts/run.sh campaign --scenario marvin_handover select \
  --out /data/campaigns/marvin_handover_v1/SELECTED.json
```

质量规则按本任务阶段声明：左手抓取看左腕/全局，空中交接看物块与左右腕，运输/放置看右腕/全局与箱口；`left_grasp`、`dual_hold`、`left_release`、`right_release` 为必需阶段，缺失即导出不合格。最终数据、路径与 4/2 划分见[P04 交付记录](../../../docs/marvin-handover/deliveries/phase-04.md)。

P05 学习与执行冒烟的复现入口（路径基准同 P04；`/data` 与仓库 `var/` 为同一目录）。现有数据、模型和三次 evaluation 已完成，P06 已复用且不重复执行以下转换/训练/评价命令；重新实验须使用新的输出目录及单独预算。

```bash
# learning 容器：4/2 转换、128 step 短训练、bundle、诊断 Policy 准入
learning/lerobot/scripts/run.sh convert --scenario marvin_handover
learning/lerobot/scripts/run.sh train --scenario marvin_handover --mode smoke
learning/lerobot/scripts/run.sh bundle --scenario marvin_handover --mode smoke \
  --run /data/learning/marvin_handover_diag/runs/<actual-run> --checkpoint final.pt \
  --output /data/learning/marvin_handover_diag/bundles/<new-bundle>
learning/lerobot/scripts/run.sh prepare-policy --scenario marvin_handover \
  --bundle /data/learning/marvin_handover_diag/bundles/<new-bundle> --no-require-formal \
  --run-id <policy-run> --output /data/prepared/<policy-run>/policy

# runtime 容器：首次使用或 proto 变更后生成 Policy SDK
runtime/ros2/scripts/run.sh build-policy-sdk

# learning 容器：启动 Policy 服务（诊断 bundle 使用生成的 policy.yaml）
learning/lerobot/scripts/run.sh policy \
  --config /data/prepared/<policy-run>/policy/policy.yaml \
  --log-directory /data/logs/act-policy/<policy-run> --run-id <policy-run>

# simulation 容器：推理后端（不使用 Teacher）
scripts/run.sh prepare --scenario marvin_handover --role policy \
  --bundle /data/learning/marvin_handover_diag/bundles/<new-bundle> \
  --seed 5100 --run-id sim-policy --output /data/prepared/sim-policy
scripts/run.sh backend --config /data/prepared/sim-policy/backend.yaml --run-id backend-policy

# runtime 容器：绑定同一 bundle，再执行一次 evaluation 尝试
runtime/ros2/scripts/run.sh prepare --scenario marvin_handover --role policy \
  --policy-binding /data/prepared/<policy-run>/policy/policy_binding.json \
  --bundle /data/learning/marvin_handover_diag/bundles/<new-bundle> \
  --seed 5100 --run-id <policy-run> --output /data/prepared/<policy-run>/runtime
runtime/ros2/scripts/run.sh campaign \
  --config shared/scenarios/marvin_handover/configs/runtime/experiments/handover_campaign.yaml \
  run --purpose evaluation --mode act --seed 5100 --run-id <attempt> \
  --runtime-config /data/prepared/<policy-run>/runtime/runtime.yaml -- \
  python3 -m dual_arm_runtime.runtime_node --project-root /workspace --once \
  --config /data/prepared/<policy-run>/runtime/runtime.yaml \
  --run-id {run_id} --episode-id {episode_id} \
  --output {attempt_dir}/stack/{run_id}/raw \
  --runs-output {attempt_dir}/stack/{run_id}/runs
```

evaluation 种子从 5100–5119 分配，失败/拒绝/取消同样占用最多三次尝试；当前预算已用尽 3/3，不重用示例 seed 5100 或改用 test 预算。task 模式使用 10 s 评价预算、1 s 停止余量和 60 s 墙钟。第三次在 4 个接受窗口后因跟随偏差拒绝停止，其中前三个正常 HANDOFF；任务未完成，详细身份、执行反馈及推理专用容差见[P05 交付记录](../../../docs/marvin-handover/deliveries/phase-05.md)。诊断 bundle 用于链路验证，后续效果训练另行安排。

物理诊断入口：

| 诊断 | 仓库入口 |
| --- | --- |
| 左手独立抓取与持稳 | [test_marvin_left_hold_ros.py](../../../tests/integration/test_marvin_left_hold_ros.py) |
| 右手独立抓取与持稳 | [test_marvin_right_hold_ros.py](../../../tests/integration/test_marvin_right_hold_ros.py) |
| 共持全过程与抬升 | [test_marvin_dual_hold_ros.py](../../../tests/integration/test_marvin_dual_hold_ros.py) |
| 带物受控停止 | [test_marvin_stop_hold_ros.py](../../../tests/integration/test_marvin_stop_hold_ros.py) |
| 连续空中交接、退手与入箱释放 | [test_marvin_transfer_ros.py](../../../tests/integration/test_marvin_transfer_ros.py) |

目录导航：[P03 交付记录](../../../docs/marvin-handover/deliveries/phase-03.md) · [P05 交付记录](../../../docs/marvin-handover/deliveries/phase-05.md) · [P06 交付记录](../../../docs/marvin-handover/deliveries/phase-06.md) · [任务书](../../../docs/marvin-handover/README.md) · [架构边界](../../../docs/marvin-handover/architecture.md)
