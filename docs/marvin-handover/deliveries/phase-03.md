# P03 交付：交接任务、Teacher 与完整示教

日期：2026-11-13。**P03 已完成，停在 P04 正式采集之前**。工作树、历史成功/失败证据保留；未暂存、提交或推送。P00/P01/P01A/P02A/P02B 交付记录未覆盖。

## 验收结果

Marvin 交接任务通过真实 runtime Teacher → 标准动作流 → `SubmitFollower` → MuJoCo 机器人 → 任务门禁/物理评价的完整链执行。物块仅由接触运动改变；每次尝试由 runtime 显式复位后重新准备。

| 证据 | 结果 |
| --- | --- |
| 连续 5 次复位完整成功 [successes.json](../../../var/reports/p03/successes.json) | 05–09 均为 `SUCCEEDED`；每次 56.1 s 仿真跨度、57 个执行窗口、61.0–61.4 s 墙钟 |
| 首次完整成功 [p03-teacher-05](../../../var/raw/p03-teacher-05/p03-teacher-05/task_result.json) | 8 个实际顶点全部在箱内，箱底支撑 2.943 N，双手释放，稳定 1.0 s |
| 右手未接稳 [p03-miss-01](../../../var/raw/p03-miss-01/p03-miss-01/task_result.json) | `GATE_TIMEOUT: dual_hold/right_close`；左手保持闭合夹持、右手 0 接触、左释放未发生 |
| 操作者中断 [p03-interrupt-01](../../../var/raw/p03-interrupt-01/p03-interrupt-01/task_result.json) | `interrupted/SIGINT`；停止与释放完成，任务 `CONTROL_FAILED: RUNTIME_SHUTDOWN` |
| 接稳/释放顺序图像 | [right_catch/global](../../../var/reports/p03/images/right_catch/global.png) → [left_release/right_wrist](../../../var/reports/p03/images/left_release/right_wrist.png) → [stable/global](../../../var/reports/p03/images/stable/global.png) |

五次成功的关键实测（仿真时间，单位 s）：

| run | task start | right catch | left release | left retreat | task complete | final block center (m) |
| --- | --- | --- | --- | --- | --- | --- |
| 05 | 1.292 | 37.872 | 41.228 | 46.034 | 57.404 | `(0.640261, -0.269738, 0.821892)` |
| 06 | 1.148 | 37.722 | 41.078 | 45.884 | 57.256 | `(0.640250, -0.269748, 0.821892)` |
| 07 | 1.160 | 37.772 | 41.128 | 45.934 | 57.306 | `(0.640259, -0.269739, 0.821892)` |
| 08 | 1.156 | 37.772 | 41.128 | 45.934 | 57.304 | `(0.640261, -0.269738, 0.821892)` |
| 09 | 1.020 | 37.622 | 40.978 | 45.784 | 57.154 | `(0.640261, -0.269738, 0.821892)` |

每次成功都满足同一组独立条件：八顶点箱内、箱底支撑 ≥ 0.5 N、双手无物块接触、线/角速度 ≤ 0.02 m/s / 0.2 rad/s、位置/姿态漂移 ≤ 5 mm / 0.05 rad，并连续 1.0 s。右手接稳到左手释放间隔 **3.356 s**，左手释放后右手继续持物；运输、放置和释放只发生在左臂退出之后。

## 阶段、门禁与指标

`TaskState.phase` 仍是 `IDLE / PREPARED / RUNNING / SUCCEEDED / FAILED`。行为阶段写入 `metrics.behavior_phase`，Teacher 阶段码固定为：

| 码 | 阶段 | 门禁事实 |
| --- | --- | --- |
| 0 | `left_approach` | 左臂到达预抓取位，TCP/关节误差与速度在配方内 |
| 1 | `left_grasp` | 左两指同时接触同一物块、接触力、指速与姿态持续成立 |
| 2 | `left_lift` | 左手持续持物，物块离桌且高度 ≥ 0.10 m |
| 3 | `handover_approach` | 左手持物到交接区；右臂张开接近第二抓持区；全程无外部支撑 |
| 4 | `dual_hold` | 右手两指接触同一物块，接稳时长与相对位姿稳定 |
| 5 | `left_release` | 左手无接触、开度 ≥ 0.075 m、指速 ≤ 0.005 m/s；右手持续持物 |
| 6 | `left_retreat` | 左臂到达避让位且无接触；右手保持 |
| 7 | `right_transport` | 右手持物到箱口上方；无外部支撑 |
| 8 | `right_place` | 右手持物，物块实际顶点已进入箱体 |
| 9 | `right_release` | 双手均无接触，物块由箱底支撑 |
| 10 | `stable` | 独立物理评价全部成立并连续稳定 |

门禁参数（`configs/simulation/scene/marvin_handover_task.yaml`）：接触力 ≥ 0.5 N、指速 ≤ 0.02 m/s、相对位姿 5 mm / 0.05 rad、门禁连续 0.5 s、单步等待上限 8.0 s、离桌高度 0.10 m、释放开度 0.075 m、失持容忍 0.3 s、禁止接触力 0.5 N、箱底容差 1 mm、最终稳定 1.0 s。接触力、持物与相对位姿容差复用 P02B `diagnostics`，没有另设一套数值。

`metrics` 至少包含：行为阶段/步骤/门禁状态/失败原因、阶段进入与等待、左右接触数与最小力、指速、开度、离桌高度、桌/箱/外部支撑力、交接窗口与违规、右手接稳与左手实际释放时间、左退开完成、八个实际顶点、完整包含、箱底支撑、双手释放、线/角速度、位置/姿态漂移、稳定时长、机器人-机器人/机器人-箱体接触力、阶段事件序列。真值只进入 TaskState，不进入 Policy 观测。

## Teacher IDL 与执行契约

- 新方案包 `shared/scenarios/marvin_handover/extensions/ros/marvin_handover_api_msgs`：`TeacherCapability.msg`（`marvin_handover_api_msgs.teacher_chunk.v1`）、`GetTeacherChunk.srv`、`package.xml`、`CMakeLists.txt`。
- 保持 60×16、50 ms、显式 `robot_profile_hash`、`task_epoch`、`capture_seq`、观测/源时间、`plan_digest`/`chunk_digest`。阶段码使用本任务 0–10 语义。
- 请求新增显式 `handoff_time`：Teacher 从该调度交接时刻开始新阶段运动，使目标与 follower 将继承的参考状态连续；没有该字段的 stack 调用保持原行为。
- 两端按 `<package>.teacher_chunk.v<N>` 能力标识解析生成类型并校验 `TeacherCapability.ID`；通用 `teacher_client`、采集流程和任务客户端不导入具体扩展包。simulation 的任务插件声明 `teacher_capability`，通用 ROS 节点据此注册服务。
- Teacher 只采样当前已通过门禁的步骤并保持其末端姿态；未来动作块不能提前打开左爪。`on_capture` 把该捕获的门禁事实冻结，过期 epoch/capture 被拒绝。推理模式使用同一评价器、准备 evaluation-only 计划，不输出 Teacher 目标。

## 执行预算修复

- runtime 全局任务仿真上限从 **30 s** 调整为有界 **300 s**，块数上限 512 → 2048；`max_chunks × follower.target_count × action_period_ns ≥ task_sim_budget_ns` 与墙钟 `≥ 仿真预算 + 20 s` 在配置加载时校验。runtime 期望块缓存与上述上限一致。
- 本方案首轮：`task_sim_budget_ns = 180 s`、`max_chunks = 256`（20 点 × 50 ms = 1 s 窗口，覆盖 256 s）、`total_wall_deadline_s = 420 s`、`task_stop_margin_s = 1 s`。实测 56.1 s 仿真、57 块，预算余量充足。
- simulation 任务配方 `success.max_task_sim_duration_s = 180 s`；runtime prepare 与 collection 选择在准备阶段交叉校验运行时预算、块长与周期同仿真任务 Judge 一致，不一致直接失败。
- 租约、跟踪、停止和轨迹门槛未放宽。

## 环境、SDK、命令与产物

simulation 容器 `/app`，runtime 容器 `/workspace`，ROS 2 Jazzy，MuJoCo 3.17.0。方案场景 `/shared/scenarios/marvin_handover`，产物 `/data`。

```bash
# runtime 容器；确认无进程占用旧 SDK
runtime/ros2/scripts/run.sh build-ros-sdk --scenario marvin_handover --shared-root /shared
# built 3 packages: robot_api_msgs, sim_api_msgs, marvin_handover_api_msgs
# package_set_sha256 = d92f4ee04ae9c75cdf1c5f15c86b1254fb64a9bff1c41269d546e55223a1cf4c
# Panda 回归后已按本方案重建同一包集；P04 直接复用该活动 SDK。

# simulation 容器
scripts/run.sh prepare --scenario marvin_handover --role teacher --seed 0 \
  --run-id p03-sim-teacher --output /data/prepared/p03-sim-teacher
scripts/run.sh backend --config /data/prepared/p03-sim-teacher/backend.yaml \
  --run-id p03-backend-05

# runtime 容器
runtime/ros2/scripts/run.sh prepare --scenario marvin_handover --role teacher --seed 0 \
  --run-id p03-runtime-teacher --output /data/prepared/p03-runtime-teacher
python3 -m dual_arm_runtime.runtime_node --project-root /workspace \
  --config /data/prepared/p03-runtime-teacher/runtime.yaml \
  --output /data/raw/p03-teacher-05 --run-id p03-teacher-05 --episode-id p03-teacher-05 --once

# 失败入口：右手闭合命令保持 0.075 m（宽于 60 mm 物块），其他目标与链路相同
scripts/run.sh prepare --template \
  /shared/scenarios/marvin_handover/configs/simulation/backend/backend_marvin_teacher_right_miss.yaml \
  --seed 0 --run-id p03-miss-teacher --output /data/prepared/p03-miss-teacher
# 运行端仍使用同一 runtime teacher 配置
# 中断：运行中向 runtime 进程发送 SIGINT
timeout --signal=INT --kill-after=40 20 python3 -m dual_arm_runtime.runtime_node ...
```

产物：`var/prepared/p03-sim-teacher/`、`var/prepared/p03-runtime-teacher/`、`var/prepared/p03-miss-teacher/`；成功/失败/中断原始录制与 `task_result.json` 位于 `var/raw/p03-teacher-05..09/`、`var/raw/p03-miss-01/`、`var/raw/p03-interrupt-01/`；backend 日志位于 `var/runs/backend/p03-backend-05/`、`p03-backend-miss/`；汇总、日志与抽帧位于 `var/reports/p03/`。collection binding 阶段校验与 runtime 选择预检见 `var/reports/p03/collection-validate.log`、`var/reports/p03/collection-selection.log`。

## 失败与停止证据

- **右手未接稳**：右爪以 0.075 m 闭合后 `right_finger_contacts = 0`、`right_holding = false`；左爪 0.0545 m、两指接触、物块离桌 119.6 mm。`dual_hold/right_close` 等待 8.50 s 后 `GATE_TIMEOUT`，`right_catch_sim_time_ns = 0`、`left_release_sim_time_ns = 0`，左退开未开始。证明右手稳定接住同一悬空物块之前左手不释放。超时后任务失败，Teacher 停止，runtime 记录 `TASK_FAILED`。
- **操作者中断**：19.4 s 时 SIGINT，机器人处于 `left_handover`（左持物离桌 149.4 mm，右未接）。runtime 记录 `SIGINT` → `control.stop_completed` → `control.release_completed`，backend 依次记录 `HANDOFF`/`PENDING`/`STOPPING RUNTIME_SHUTDOWN`/`CANCELED`；任务 `FAILED: CONTROL_FAILED: RUNTIME_SHUTDOWN`，录制完整关闭，不记成功。
- 针对性状态机/物理测试：位置门禁超时、左手释放后右手失持 `HOLD_LOST`、物块高于箱口不判成功、右手未接稳时 Teacher 块保持左爪闭合、evaluation-only 计划使用同一物理成功条件，均在 `tests/unit/test_handover_task.py`。

## 任务、机器人与 ROS 职责修复

- simulation 新增 handover 配置/观测/评价/Teacher 插件（`tasks/handover/`），复用 [plan.py](../../../simulation/mujoco/src/robot_backend_sim/tasks/handover/plan.py) 的 P02B 目标链与路径净空、P02B 实测接触容差、具名机器人映射和 owner 一致快照；`cohold.py` 未改，双 grasps 回归仍通过。
- 规划改为只读快照：`layout_from_config` 从配置与冻结 qpos 生成布局，物块实际落座位置作为目标中心（2 mm 一致性容差），规划线程不读活动 `MjData`。
- 目标生成按 follower 的加速度受限形状采样，并按调度交接时刻对齐阶段起点；解决了长动作在 20 点窗口下超参考偏差预算的问题。路径、抓持点、箱位和接触参数未改。
- simulation Teacher 服务注册改为插件声明能力 + 通用按能力解析；runtime `teacher_service_type` 同样按能力包解析，删除了对 `panda_stack_api_msgs` 的硬编码导入。
- runtime 任务标识改由配方 `teacher.task_id` 提供（Panda 配方显式 `panda-stack`，本方案 `marvin-handover`），通用自动启动不再按动作来源写死 Panda 名称。
- 通用 `TargetSource.predict` 增加可选 `handoff_time_ns`；Policy 源忽略，Teacher 源透传。无新增任务行为分支。
- scenario 保存选择与参数：binding、三份配方（simulation_teacher/runtime_teacher/campaign）、IDL 与最小 binding；业务实现留在所属端。

## 回归

| 验证 | 结果 | 日志 |
| --- | --- | --- |
| 全量单元（排除宿主缺少 grpc/终端的 learning 模块） | **1052 passed, 7 skipped**，50.6 s | `var/reports/p03/regression-unit.log` |
| Panda API：Teacher 扩展、任务准备、backend ROS、可选接口 | **10 passed, 1 skipped**，6.9 s（Panda SDK `3b4a536b…`） | `var/reports/p03/regression-panda-api.log` |
| P02B 连续空中交接入箱 | **1 passed**，127.66 s | `var/reports/p03/regression-p02b-transfer.log` |
| P02B 左/右/共持/带物停止 | **4 passed**，314.70 s | `var/reports/p03/regression-p02b-holds.log` |
| Marvin 新 Teacher 能力解析、handoff 字段与旧能力拒绝 | **2 passed**，0.4 s | `var/reports/p03/regression-marvin-extension.log` |

全量单元测试的 7 个 skip 来自宿主缺少 `mujoco`/`mink` 的可选模块（与 P00 已知缺口一致）；`test_learning_console_progress` 依赖真实终端，不属于本次改动范围。Panda 采集选择、导出质量与标签测试未改，保持原状。

## 未完成与下一阶段

- 正式采集、导出、质量审计、6 条合格数据与 4/2 划分属于 P04；campaign 已填写 pilot/formal 预算、种子和路径，`quality` 规则尚未按真实渲染测量填写，`required_paths` 只列已存在文件。
- `export_quality.py` 仍固定 stack 阶段集合，P04 按本任务阶段/事件区间修正；本轮未导出正式数据集。
- 训练、新模型 bundle 与策略冒烟属于 P05；binding 未声明 model、dataset/training/policy 配方。
- P03/P04 合计的同模型同配方连续 5 次复位成功已在本轮取得（05–09），若后续改门禁、模型、场景或执行参数，需重新取得。
- 底座仍为仅渲染网格；本任务路径未进入未建模底座区域。

目录导航：[任务书入口](../README.md) · [P02B 交付](phase-02b.md) · [P04](../phase-04-runtime-data.md)
