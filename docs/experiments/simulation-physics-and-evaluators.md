# 仿真接触物理、夹持跟踪与任务评价器

本文总结 MuJoCo 仿真环境（`simulation/physics`）与任务评价器（`tasks/stack`、`tasks/handover` 及离线审计）在刚体接触摩擦、双指夹持状态判定、几何投影与多阶段交接证明上的核心修复经验。

---

## 1. 刚体接触摩擦与求解器稳定性

### 1.1 自由物体开启 `condim=4` 扭转摩擦

- **故障现象**：抓取点偏心时，搬运中的源方块可能绕双指的夹持轴旋转。可用 `source_tilt` 确认翻转发生在哪个阶段。
- **物理根因**：MuJoCo 中 geom 的默认接触维度为 `condim=3`（仅包含 1 个法向约束与 2 个切向滑动摩擦，**绕接触法向的扭转摩擦力矩恒为 0**）。即使配置了 `object_friction = [1.0, 0.02, 0.0001]`，在 `condim=3` 下第 2 项扭转摩擦系数 `0.02` 会被完全忽略；当策略抓取点相对方块重心有轻微偏心时，重力矩使方块在双指间无阻力旋转。
- **配置**：Marvin Stack 场景为自由物体设置 `object_contact_dim: 4`，并使用 `cone: elliptic` 与 `impratio: 10.0`。

### 1.2 检查轻质量方块的静止支撑

- **实测现象**：在 40 g 轻质量方块堆叠（box-on-box 面接触）场景下，若开启 MuJoCo 的 `noslip_iterations = 3` 后处理求解器，静态叠放在蓝块上的红块会每隔约 `0.2 s` 出现单步 `0.29–0.37 rad/s` 的角速度数值脉冲微颤，反复打断评价器要求的 `2.0 s` 连续静止计时。
- **排查方法**：若叠放后连续稳定计时反复归零，检查源块和目标块的角速度与接触力，并核对求解器设置。

---

## 2. 接触活动性过滤与双指夹持跟踪判定

### 2.1 仅聚合 `efc_address >= 0` 的有效接触（B04）

- **问题根因**：MuJoCo 在接觸检测 margin 内但尚存在正间隙（未真正产生约束力）的几何对也会生成 `mjContact` 记录，其 `efc_address == -1`（inactive）。旧版接触快照将所有距离在阈值内的记录计入接触，导致手指靠近物块但未触碰时即被误判为已接触。
- **修复标准**：`physics/contact_snapshot.py` 与任务评价器只聚合 **`efc_address >= 0`** 的活动约束；处于 margin 内正间隙且 `efc_address == -1` 的非活动接触一律不计入手指接触数、持物证据或外部支撑力。

### 2.2 零速度容差统一与有效夹持状态豁免

- **统一零速度分类容差**：在 `gripper_contact.py` 与 `motion_controller.py` 中统一复用 `REFERENCE_VELOCITY_ATOL_M_S = 1e-9`。避免闭合参考速度因浮点尾数呈现极小正值（如 `+5.34e-14 m/s`）时既不满足闭合 `<= 0` 又不满足张开 `> 1e-9`，从而掉入分类空隙触发 `TRACKING_TIMEOUT`。
- **按物理夹持事实维持接触约束豁免**：
  - 当夹爪夹持约 40 mm 宽的物块时，控制参考开度通常维持在 `0–20 mm` 以提供夹持预载，实测开度则停在 `~40 mm`。若后续预测块的参考开度在 `12 mm -> 22 mm` 之间微调（参考速度短暂为正，但参考开度仍小于实测开度），物理上双指仍紧紧夹持物块。
  - `gripper_contact.py` 按完整物理事实识别有效夹持：**参考开度小于实测开度、位置误差超过容差、双指同时接触同一目标物体、双指法向力均达到接触门槛（如 `>= 0.5 N`）且手指实测速度低于静止阈值**。只要上述夹持证据成立，参考速度短暂变正不再取消 `CONSTRAINED` 跟踪豁免；仅当单指碰撞、法向力不足（如 `< 0.5 N`）或丢失接触且持续超差满 `2.0 s` 时，才触发真实的 `TRACKING_TIMEOUT`。

---

## 3. 堆叠任务（`marvin_sim/stack`）评价器关键修复

### 3.1 目标局部坐标系 XY 支撑投影（B02）

- **问题根因**：`tasks/stack/evaluator.py::_support_xy_error` 在计算源方块中心相对目标方块顶面的水平越界量（XY over-edge）时，误用了 `R @ delta` 而非转置矩阵，导致当目标方块存在绕 Z 轴旋转（Yaw $\neq 0^\circ$）时水平投影计算错误（例如目标块旋转 $10^\circ$、局部偏移 `(0.027, 0.027)` 时，真实越界为 `-0.001 m` 在界内，旧公式算得 `+0.0066 m` 误判出界）。
- **修复标准**：使用 **`R.T @ delta`** 将世界系位置差严格投影到目标方块的局部坐标系，再与半尺寸及 `support_xy_margin_m` 比较；同时保持源块与目标块的独立倾角（upright）约束不变。

---

## 4. 交接任务（`marvin_handover`）评价器关键修复

### 4.1 箱底支撑按真实 `box_bottom` Geom 判定（B05）

- **问题根因**：旧实现按整个目标箱体 body 汇总接触力，导致物块卡在箱体侧壁或斜靠箱沿时也能产生足够的合外力并通过“入箱支撑”条件。
- **修复标准**：`build_contact_snapshot` 新增按具体 geom 统计的 `SupportGeomContact`，根据 `geom1/geom2` 顺序与接触法向计算支撑体对物块的世界系向上（+Z）支撑力。最终入箱条件仅消费 **`box_bottom`** geom 的向上支撑力；箱体侧壁与桌面接触仍计入 `external_support_force()` 用于拦截交接期间的违规借力，但绝不能冒充箱底支撑。

### 4.2 空中交接纯物理事件序状态机（B03）

无论运行 Teacher 还是 Policy（`steps=()`），`marvin_sim/handover/evaluator.py` 均通过独立于名义步骤的 `_SequenceFact` 状态机验证真实空中交接顺序（短去抖 `gates.stable_duration_s = 0.08 s`），严禁“直接单手把物块放入箱底即判成功”：

1. **左手空中独持（`left_airborne_stable`）**：左手双指有效接触持物、右手未持物、物块离桌达到 `lift_height_m` 且无任何外部支撑，连续满足 `gates.stable_duration_s`（`0.08 s`）。
2. **双手空中共持（`dual_airborne_stable`）**：在完成左独持证明后，左右手双指同时有效持物且无任何外部支撑，连续满足 `0.08 s`。
3. **左手释放且右手空中独持（`left_release_observed` / `right_airborne_stable`）**：在完成空中共持证明后，左手双指零接触且右手双指单独在空中有效持物、无外部支撑，连续满足 `0.08 s`，随后才允许放入目标箱底并累计最终 `1.0 s` 稳定成功窗口。

### 4.3 双持成立后左手正常张开期的持物职责移交（B09）

- **问题根因**：在空中双持稳定证明（`dual_airborne_stable`）已成立后，左手开始正常张开释放，在达到无接触释放终点前存在短暂单指擦碰过渡。若在左手张开前仍要求左指保持双指夹持力，会导致左手正常张开超过 `hold_loss_tolerance_s` 被误判为 `HOLD_LOST`。
- **修复标准**：在**空中双持稳定证明成立后**，只要右手已建立有效持物（`right_holding`），左手即可自由张开释放；同时继续严格检查右手在此期间不得失持以及左手最终必须完成零接触释放。

---

## 5. 离线汇总与结构化日志契约（B01 / B07）

- **稳定支撑时长（B01）**：Stack 要求连续支撑 `2.0 s`，Handover 要求 `1.0 s`。在线评价器将判定写入 `task_result.json`，离线评估核对结果、配置摘要和时间关系。
- **日志事件上下文隔离（B07）**：`ros/node_services.py` 在记录 `backend.task.teacher_chunk` 事件时，严格使用 `EventContext(episode_id=context.episode_id, request_id=chunk.request_id)`，禁止将 `request_id` 混写入 `episode_id` 字段。

---

目录导航：[实践索引](README.md) · [全链路闭环排查](closed-loop-triage.md) · [数据与模型诊断](data-and-model-diagnostics.md) · [推理轨迹执行](inference-trajectory-execution.md)
