# marvin_handover

Marvin 固定根双臂方案：左手抓取物块、空中交接给右手、再放入开口箱。本目录保存方案选择、机器人/观测/动作档案、仿真/运行/学习配方与方案专用 Teacher 接口；三端实现仍由 `simulation/`、`runtime/`、`learning/` 提供。

## 已有配置与档案

| 路径 | 内容 |
| --- | --- |
| `binding.yaml` | 方案选择：仿真/mujoco/handover、runtime/ros2、Teacher 能力、三份配方与扩展包；`models.bundle` 指向本任务真实诊断 Bundle |
| `configs/simulation/scene/marvin_handover.yaml` | 机器人装配选择、任务环境、执行器/复位数值和相机安装 |
| `configs/simulation/backend/backend_marvin_control.yaml` | 无任务插件的 direct backend 配方；`task`/`task_mode` 省略 |
| `configs/simulation/backend/backend_marvin_teacher.yaml` | 任务插件与 Teacher 后端（`task_type: handover`、`task_mode: teacher`），启用分割质量计数 |
| `configs/simulation/backend/backend_marvin_teacher_right_miss.yaml` | 右手未接稳的负向诊断后端配方 |
| `configs/simulation/backend/backend_marvin_policy.yaml` | Policy 推理后端（`task_mode: inference`），只运行同一物理评价 |
| `configs/simulation/scene/marvin_handover_task.yaml` | 阶段门禁、终态评价、180 s 执行预算、Teacher 时钟与容量 |
| `configs/simulation/scene/marvin_handover_task_inference.yaml` | 短时推理评价：Teacher 关闭、10 s 仿真 / 60 s 墙钟、同一门禁与成功判定 |
| `configs/simulation/planning/handover_paths.yaml` | 任务目标、抓持点、接近/退避位、协调抬升、执行时长与诊断阈值 |
| `configs/simulation/checks/handover_right_grasp_miss_task.yaml` | 右手未接稳负向诊断：右闭爪命令宽于物块，其他参数相同 |
| `configs/runtime/teacher_collect.yaml` | runtime Teacher 采集配方（180 s 任务预算、能力标识） |
| `configs/runtime/policy_eval.yaml` | runtime Policy 执行配方（10 s 任务预算、1 s 停止余量、60 s 墙钟） |
| `configs/learning/act_dataset.yaml` | 诊断数据转换：6 条、4/2 划分、validation `[4,5]`、`diagnostic: true` |
| `configs/learning/act_training.yaml` | ACT smoke 128 step、独立 900 s 预算台账与 checkpoint 路径 |
| `configs/learning/policy.yaml` | ACT Policy 服务；首轮关闭时间融合，按实测限制夹爪修正 |
| `configs/runtime/experiments/handover_campaign.yaml` | 诊断采集预算、种子、台账、阶段质量规则、导出标签上限与独立路径 |
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

装配入口按这两个字段组合机器人装配与任务环境；`task.environment=handover` 不用于推断机器人型号。`robot.assembly` 由 simulation 的资产定位解析为本端固定版本模型，recipe 不写模型文件路径。binding 的 `recipes.dataset`、`recipes.training`、`recipes.policy_service`、`recipes.runtime_policy` 与 `recipes.simulation_policy` 指向本方案的诊断配方，`models.bundle` 指向真实诊断 Bundle。

本方案接口包为 `robot_api_msgs`、`sim_api_msgs` 与 `marvin_handover_api_msgs`，已包含在全量 SDK 构建中；一次 `build-ros-sdk` 即服务全部方案，切换方案无需重建：

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

## 离线检查

在 `simulation` 容器内：

```bash
python3 -m robot_backend_sim.tools.check_marvin_scene \
  --scene-config /shared/scenarios/marvin_handover/configs/simulation/scene/marvin_handover.yaml \
  --profile /shared/scenarios/marvin_handover/profiles/robots/marvin_handover_v1/profile.json \
  --schemas /shared/common/interfaces/schemas \
  --report-root /data/marvin_handover/diagnostics/marvin \
  --run-id marvin-p01a
```

报告写入容器 `/data/marvin_handover/diagnostics/marvin/<run-id>/report.json`，渲染图像写入同一运行目录的 `artifacts/renders/`。检查内容包括：宿主机/容器资产路径解析、编译模型形状与命名绑定、源 URDF 质量/主惯量对照、equality 枚举、TCP/指面/开度测量、静置步进、14 个臂关节低幅方向、左右隔离、夹爪三态开度与联动、桌面/物块/开口箱碰撞、同臂/跨臂/桌/箱/手指有效碰撞探针、开口射线和三路渲染。

## 操作入口

所有入口都直接以 `--scenario marvin_handover` 读取本目录 `binding.yaml`；不生成 `/data/prepared/` 中间目录，`--scenario` 与 `--config` 互斥，重复 `--set dot.path=value` 可覆盖配方中已存在的键。

### 采集

```bash
# simulation 容器：直接启动 Teacher 后端
scripts/backend.sh --scenario marvin_handover --role teacher \
  --seed 0 --run-id teacher-backend

# runtime 容器：试采与正式采集（默认后端日志为 /data/marvin_handover/logs/backend/teacher-backend/log）
scripts/record.sh --scenario marvin_handover \
  --purpose pilot --count 1 --max-consecutive-failures 1
scripts/record.sh --scenario marvin_handover \
  --purpose formal --count 6 --max-consecutive-failures 3

# runtime 容器：查看采集与筛选状态
scripts/record.sh status --scenario marvin_handover
```

正式采集达到 campaign 目标时也会自动原子写出 `var/marvin_handover/datasets/formal-select.json`。质量规则按本任务阶段声明：左手抓取看左腕/全局，空中交接看物块与左右腕，运输/放置看右腕/全局与箱口；`left_grasp`、`dual_hold`、`left_release`、`right_release` 为必需阶段，缺失即导出不合格。

### 数据转换与训练

```bash
# learning 容器：4/2 转换、128 step 短训练、bundle
scripts/convert.sh --scenario marvin_handover
scripts/train.sh --scenario marvin_handover --mode smoke
scripts/bundle.sh --scenario marvin_handover --mode smoke \
  --run /data/marvin_handover/models/runs/<actual-run> --checkpoint final.pt \
  --output /data/marvin_handover/models/bundles/<new-bundle>
```

诊断配方 `act_dataset.yaml` 使用 `diagnostic: true`，训练与转换只用于链路验证。

### 推理

```bash
# learning 容器：直接启动 Policy 服务（默认使用 binding 的诊断 Bundle）
scripts/policy_backend.sh --scenario marvin_handover \
  --bundle /data/marvin_handover/models/bundles/<bundle>

# simulation 容器：推理后端（不使用 Teacher）
scripts/backend.sh --scenario marvin_handover --role policy \
  --bundle /data/marvin_handover/models/bundles/<bundle> \
  --seed 5100 --run-id policy-backend

# runtime 容器：交互式一局或非交互式单局
scripts/interactive.sh --scenario marvin_handover \
  --bundle /data/marvin_handover/models/bundles/<bundle> \
  --seed 5100 --run-id policy-interactive
scripts/policy.sh --scenario marvin_handover \
  --bundle /data/marvin_handover/models/bundles/<bundle> \
  --reset-seed 5100 --run-id policy-run
```

evaluation 种子从 5100–5119 分配，失败/拒绝/取消同样占用最多三次尝试。task 模式使用 10 s 评价预算、1 s 停止余量和 60 s 墙钟。短训练诊断模型未完成交接，推理专用轨迹容差与模型效果限制属于模型能力边界，不代表链路失败；后续效果训练另行安排。

## 物理诊断入口

| 诊断 | 仓库入口 |
| --- | --- |
| 左手独立抓取与持稳 | [test_marvin_left_hold_ros.py](../../../tests/integration/test_marvin_left_hold_ros.py) |
| 右手独立抓取与持稳 | [test_marvin_right_hold_ros.py](../../../tests/integration/test_marvin_right_hold_ros.py) |
| 共持全过程与抬升 | [test_marvin_dual_hold_ros.py](../../../tests/integration/test_marvin_dual_hold_ros.py) |
| 带物受控停止 | [test_marvin_stop_hold_ros.py](../../../tests/integration/test_marvin_stop_hold_ros.py) |
| 连续空中交接、退手与入箱释放 | [test_marvin_transfer_ros.py](../../../tests/integration/test_marvin_transfer_ros.py) |

`READY_POSE` 仅是离线渲染用的诊断位姿，不是任务抓取位姿。当前箱位、物块初态、相机安装、交接区和抓持区由场景与任务/运动学配方声明。

目录导航：[公共层与方案绑定](../../README.md) · [根 README](../../../README.md)
