# 双臂 ACT 采集—训练—推理—仿真全链路逐级排查与闭环修复经验

日期：2026-11-16

## 1. 背景与核心结论

针对此前双任务 240 次批量推理几乎无成功（`panda_stack` 历史汇总 2/120、`marvin_handover` 0/120）的问题，本轮采用**自底向上、逐环排除易于排除项**的工程排查方法，依次验证：**采集数据与物理回放 → 模型离线预测与数据集对齐 → 推理执行侧轨迹跟随与阶段推进 → 仿真环境接触摩擦与任务评价器**。

核心结论：

1. **采集数据与基础轨迹可执行性正常**：随机抽查 6 局正式数据（Panda 3 局、Marvin 3 局），Parquet 标签与原始导出逐值一致（float32 误差为 0），无 NaN/Inf 或硬限位越界。修复交接评价器左爪正常张开释放误判（B09）后，6 局数据无论以原三次样条 Reference 还是 20 Hz 位置标签在真实 MuJoCo 物理中回放均 **6/6（100%）成功**；将数据集真值送入生产 `prepare_follower_plan` + `MotionRuntime`（Oracle Follower）同样 **100% 成功**。
2. **模型已学到正确任务轨迹，无需推倒重训**：使用当前部署模型（Panda `stack-act-refined2`、Marvin `handover-act-refined-final`）在抽样帧上离线对比 60 步预测与真值，Panda 活动臂 MAE 为 `0.0132 rad`（静止基线 `0.1620 rad`），Marvin 活动臂 MAE 为 `0.0105 rad`（静止基线 `0.2281 rad`）。但数据集中存在 32%–47% 的阶段末门控等待静止帧，使单帧模型在阶段切换处呈现“近期点保持等待、中远期点推进动作”的分布特性。
3. **真正导致闭环大面积失败的是推理执行侧与仿真/评价器的四处拦截与失配**：
   - **推理执行窗口与过渡重定时不完整**：短窗口（`target_count=5`，0.25 s）落在单帧模型的前段等待区导致原地死锁超时；切到长窗口（`target_count=20`，1.0 s）跨过等待区后，因 P3-C 过渡重定时仅覆盖 2 个夹爪轴且仅按单轴极限包络触发，模型跨预测的正常闭环修正直接触发臂轴或夹爪轴 `FOLLOWER_DEVIATION_EXCEEDED` / `FOLLOWER_HORIZON_EXCEEDED` 中途停机；同时 `marvin_follower_inference_v1.yaml` 中仅放宽了左臂偏差预算而将右臂遗留在 `0.05 rad`。
   - **臂轴非最终块截断引发交接点速度过冲**：当模型末点超出单窗口可减速停止范围时，Ruckig 规划出 `>1.0 s` 曲线并在 `1.0 s` 处截断，留下高达 `0.7–0.8 rad/s` 的未减速速度传入下一块（臂轴以 `1.0 rad/s²` 刹停需 `0.32 rad` 距离），造成机械臂在交接点过冲碰撞。
   - **单帧模型在放置/交接等待区的尾部动作死锁**：物块入箱后右臂静止等待，模型在 `0..39` 点预测保持闭合、在 `40..59` 点预测张爪释放；固定取 `20..39` 点使夹爪永远停在闭合态无法释放。
   - **仿真接触维度与评价器参考位姿锁死**：`panda_stack` 场景中自由方块未开启 `condim=4`（默认 `condim=3` 无绕夹持轴的扭转摩擦力矩），轻微偏心抓取即导致红块在两指间旋转 90°；`marvin_handover` 评价器在物块尚处桌面（14.2 s 初触瞬间）即锁死左手空中稳定参考位姿 `_sequence_references["left"]`，拔离桌面时的正常微调使后续 16 秒空中稳定持物一直无法通过相对位姿检查。
4. **修复效果**：在不重训模型、不重采数据、保持原有成功阈值与安全守卫的前提下，完成上述针对性修复后进行 **10 次正式三端闭环测试（Panda 5 次 + Marvin 5 次）**，取得 **8/10（80%）`TASK_SUCCEEDED`**（其中 Panda **5/5 = 100%** 成功，Marvin **3/5 = 60%** 首次实现完整闭环成功）。

---

## 2. 问题排查思路

面对“采集—训练—推理—仿真”长链条闭环失败，若直接在末端调参或设计复杂的跨端重规划协议，极易在错误假设下空转。本轮遵循以下四步收敛原则：

```text
[Step 1: 采集数据抽查与物理回放]
   ├── 抽查 Parquet/MP4/PNG 数值一致性、时间戳连续性、静止帧占比
   └── 脱离模型与规划器，用录制 Reference / 20Hz 标签 / Oracle Follower 在真实 MuJoCo 回放
         │ (若回放失败 → 先修评价器或基础物理；回放 100% 成功 ↓)
[Step 2: 模型离线输出对比]
   ├── 检查预处理器/反归一化统计量与关节/相机顺序
   └── 在抽样训练/验证帧上对比模型 60 点预测与真值（区分近期 1..5、中期 21..40、远期 41..60）
         │ (若预测乱飞 → 查训练管线；预测与真值吻合 ↓)
[Step 3: 推理执行侧隔离与闭环追踪]
   ├── 追踪模型输出经过 ActionWindow 选窗 → transition_retiming → Ruckig 规划 → MotionRuntime 跟随的全过程
   └── 定位为何模型给出了正确趋势，执行器却报 FOLLOWER_DEVIATION_EXCEEDED / TASK_TIMEOUT / 过冲碰撞
         │ (解除执行器自杀拦截与死锁后，观察物理终态 ↓)
[Step 4: 仿真物理环境与评价器精确对齐]
   ├── 检查抓取/搬运/叠放中的物理异常（如方块在指间无阻力翻转、堆叠后角速度微颤）
   └── 检查评价器状态机在真实物理接触过渡期的边界条件
```

---

## 3. 排查过程、方法与实测结果

### 3.1 第一步：采集数据抽查与真实物理回放

- **排查方法**：
  1. 编写 `var/_system/diagnostics/triage_data_audit.py`，以固定种子抽样 6 局正式数据（Panda：`10027/10007/10045`；Marvin：`4157/4155/4158`，共 4190 帧），逐列比对 Parquet 中 `action`、`observation.state`、`raw_action_chunk` 与原始导出 JSON，并核对 MP4 视频帧与原 PNG 的时间戳及 PSNR。
  2. 编写 `var/_system/diagnostics/data_first_replay.py`，包含三种回放模式：
     - `reference`：直接采样录制 MCAP 中的三次样条 `FollowerReference` 送入 `RobotIO` 与 `mj_step`；
     - `labels`：对训练使用的 20 Hz `applied_positions` 做线性插值与匹配速度前馈；
     - `oracle_follower`：将数据集未来 60 点真值送入生产 `prepare_follower_plan`（Ruckig）与 `MotionRuntime`。
- **排查结果**：
  - 数据完整性：Parquet 与导出 JSON 按 float32 逐值完全相等（最大差 0），20 Hz 网格严格连续，action lead 恒为 50 ms，无 NaN/Inf 或越界值。
  - 静止帧与边界特征：Panda 近静止帧占比 `31.6%–35.9%`，Marvin 占比 `41.5%–46.7%`，集中在各阶段末尾等待物理门控稳定及窗口耗尽；Marvin `left_joint7` 示教最小值 `-1.5707996 rad` 紧贴 `-1.5708 rad` 下限。
  - 回放发现并修复 **B09**：Marvin 3 局回放在 `33.15 s`（空中双持已稳定成立、左手正在正常张开释放的斜坡段）被 `evaluator.py::_holder_required` 误判为左手 `HOLD_LOST`。修复空步骤 Policy 路径在稳定双持成立后将持物职责移交右手后，**Panda 3/3、Marvin 3/3 在 `reference` 与 `labels` 模式下全部 100% `SUCCEEDED`**，`oracle_follower` 模式同样 **100% `SUCCEEDED`**。

### 3.2 第二步：模型离线输出与输入归一化核查

- **排查方法**：
  1. 编写 `var/_system/diagnostics/data_first_input_normalization.py`，校验 Bundle 内 `safetensors` 归一化参数（mean/std）、关节顺序、相机名称及在线观测张量构造路径。
  2. 编写 `var/_system/diagnostics/collection_model_probe.py`，在抽样 6 局的运动段、闭爪段、开爪段和阶段等待段共 56 个查询点运行真实模型推理。
- **排查结果**：
  - 预处理器与反归一化参数 float32 误差为 0，输入通道与训练完全一致。
  - 模型离线预测整体准确（Panda 活动臂 MAE `0.0132 rad`，Marvin 活动臂 MAE `0.0105 rad`）。
  - 发现关键分布特征：在阶段切换等待帧（aliased wait），模型预测的前 5–10 个点（`0.05–0.50 s`）变化极小（继续保持等待姿态），第 20–40 个点（`1.0–2.0 s`）才展开下一阶段运动；而在放置完成等待释放时，张爪动作出现在第 40–59 个点（`2.0–3.0 s`）。

### 3.3 第三步：推理执行侧故障定位与修复

- **排查方法**：
  1. 对比 `target_count=5` 与 `target_count=20` 的真实三端闭环运行日志、MCAP `/runtime/commands`、`follower.chunk_retimed` 事件及 `/robot/execution_feedback`。
  2. 在 `simulation` 容器内用 `prepare_follower_plan` 重放所有被拒绝的真实命令（P3-D 6 个代表样本及新闭环拒绝样本）。
- **定位到的四个执行侧根因与修复**：
  1. **窗口长度与阶段停顿死锁**：`target_count=5`（0.25 s）仅取模型预测的前段等待点，导致 Panda 和 Marvin 在阶段切换处无限原地踏步直至 `TASK_TIMEOUT`。将 `panda_follower_v1.yaml` 与 `marvin_follower_inference_v1.yaml` 统一为 `target_count: 20`（1.0 s），直接跨过前 20 点等待区。
  2. **`marvin_follower_inference_v1.yaml` 左右臂偏差预算不对称**：通过 `git log` 查明该配置最初仅根据第 1 帧观测（仅左臂运动、右臂静止）测量生成，左臂设为 `[0.30, 1.30, 0.52, 0.98, 0.55, 0.44, 1.51]`，右臂却停留在 `[0.05, 0.05, 0.05, 0.05, 0.10, 0.05, 0.05]`。当第 14 秒后右臂开始大幅运动时，立刻因 `0.051 rad > 0.05 rad` 被拒绝或被过度裁剪失步。将其修正为左右臂对称的合理预算 `[0.50, 1.30, 0.52, 0.98, 0.80, 0.50, 1.51, 0.02]`。
  3. **扩展 `transition_retiming.py` 至全 16 轴并引入可刹停边界 `_stop_bounds` 与直线偏离触发**：
     - 原 `PolicyStreamRunner` 仅对 2 个夹爪轴调用 `retime_action_window`，且仅在单轴全速冲刺包络 `excess > budget` 时触发，漏掉了臂轴跳变及因 Ruckig 固定窗口匀化使实际曲线偏差略超预算（如 `0.010021 m > 0.01 m`）的情形。
     - 臂轴最大速度 `0.8 rad/s`、最大加速度 `1.0 rad/s²`，从满速刹停需 `0.8 s`（`0.32 rad`）。若末点超出单窗口内可减速到零的区间 `_stop_bounds`，Ruckig 会规划出 `>1.0 s` 曲线并在 `1.0 s` 处强行截断，在交接点留下 `-0.7 rad/s` 的高速导致下一块过冲 `0.26 rad` 撞击对侧手臂（`FORBIDDEN_CONTACT`）。
     - 修复：`PolicyStreamRunner` 对全部 16 轴启用 `retime_action_window`；当 `excess > budget` 或 `line_excess > 0.90 * budget` 时触发整形，并对超界的臂轴末点用 `_stop_bounds` 约束，确保单块内可平稳减速。
  4. **阶段末等待死锁推进（Wait-State Unsticking）与交接共持同步**：
     - 当整机 16 轴在继承 `anchor` 与当前 `selected` 窗口内均处于静止等待态（`all_stationary`），而同一预测块的连续尾部 `tail`（`40..59`）已明确给出张爪释放或阶段启动目标时，将 `tail[-1]` 目标引入当前窗口，解决入箱后右夹爪卡在 `0.0497 m` 不张开的死锁。
     - 当夹爪进入闭合区间（`end < 0.068 m`）且尾部继续关紧时直接采用 `tail[-1]` 完成闭合；当持物夹爪准备张开而对侧夹爪尚在闭合途中（`anchor > 0.056 m`）时保持闭合多等一个窗口，确保空中共持满 `0.5 s`。

### 3.4 第四步：仿真物理环境与评价器故障定位与修复

- **排查方法**：
  1. 检查 `marvin-symbudget` 录包中物块已在空中稳定持握 16 秒并成功交接放入箱底、但 `left_airborne_stable` 仍为 `False` 的原因，逐帧打印 `block_height_above_initial_m`、`external_support_force_n` 与 `_sequence_references`。
  2. 检查 `panda-k20` 录包中红块在 `13.5–20.0 s` 抓起搬运期间 `source_tilt` 从 `0.004 rad` 缓慢增大至 `1.57 rad`（翻转 90°）的原因，对比 `panda/scene.py` 与 `marvin/scene.py` 编译后的 `MjModel.geom_condim` 与 `geom_friction`。
- **定位到的两个根因与修复**：
  1. **Marvin 评价器过早锁死桌面接触参考位姿（B10）**：
     - `simulation/mujoco/src/robot_backend_sim/tasks/handover/evaluator.py::_update_sequence` 原先只要 `left_holding` 为真（`14.22 s` 手指刚在桌面上接触物块，`external_support_force = 4.07 N`，`height_above_initial = -0.0001 m`），就调用 `_remember_sequence_reference` 将桌面初触位姿永久锁死在 `_sequence_references["left"]`。随后物块拔离桌面上升至 `0.12 m` 过程中在指间自然微调了数毫米（超出 `0.006 m` 阈值），导致此后 16 秒空中稳定持物永远无法通过 `_sequence_side_stable`。
     - 修复：仅当物块真正进入空中独持候选状态（`not externally_supported and block_height_above_initial_m >= lift_height_m`）或已完成该阶段证明时才记录并保持 `_LEFT` 参考位姿；`_RIGHT` 同理仅在进入空中共持候选状态时记录。41 项交接单测与接触证据测试全部通过。
  2. **Panda 仿真场景缺失 `condim=4` 扭转摩擦**：
     - `marvin_handover` 为自由物块显式设置了 `condim = 4`、`object_friction = [1.0, 0.02, 0.0001]`、`cone = elliptic`、`impratio = 10.0`，使双指夹持具备绕闭合轴的抗扭转摩擦力矩；而 `panda/scene.py` 未设置 `condim`，默认 `condim = 3` 下 MuJoCo 完全忽略 `object_friction[1]`（扭转阻力矩为 0），策略抓取稍偏离重心即在空中旋转 90°。
     - 同时实测发现：若在 Panda 40 g 轻方块堆叠上开启 `noslip_iterations = 3`，会导致静态堆叠在蓝块上的红块每隔 ~0.2 s 出现单步 `0.29–0.37 rad/s` 的求解器角速度脉冲微颤，反复打断 2.0 s 稳定计时。
     - 修复：在 `panda/scene.py` 与 `panda_stack_learning.yaml` 中为自由物体启用 `condim = 4`、`object_friction = [1.0, 0.02, 0.0001]`、`cone = elliptic`、`impratio = 10.0`，不启用 `noslip_iterations`。红块搬运倾角从 `1.57 rad` 降至 **`0.00096 rad`（0.055°）**，且堆叠后静止无微颤。

---

## 4. 问题表现及可能原因对照表

| 阶段/类别 | 故障现象与日志表现 | 真实根因 | 已排除的干扰假说 | 定位与修复方法 |
| --- | --- | --- | --- | --- |
| **策略准入** (`learning`) | Marvin 开局第 3 块即报 `POLICY_RPC_DATA_LOSS` / `left_joint7: arm prediction outside hard limits` | 示教 `left_joint7` 贴近 `-1.5708 rad` 下限（仅差 `4.25e-7 rad`），线性头在未选尾部（索引 46–50）过冲 `0.005–0.008 rad` | 网络传输丢包、模型权重损坏 | P3-B 按轴有界投影 `max_prediction_corrections`（`left_joint7: 0.01 rad`） |
| **窗口调度** (`runtime`) | `target_count=5` 下机器人停在 pregrasp 或阶段起点一动不动，直至 `TASK_TIMEOUT` | 训练数据含 32%–47% 阶段末门控等待静止帧；单帧模型前 5–10 点预测保持不动、20–40 点才运动，0.25 s 重规划陷入等待死循环 | 视觉骨干崩溃、时间融合符号写反 | 执行窗口设为 `target_count: 20`（1.0 s），并在 `all_stationary` 时利用连续尾部目标破除等待死锁 |
| **跟随规划** (`runtime` / `simulation`) | 中途报错 `FOLLOWER_DEVIATION_EXCEEDED`（如 `right_joint2` 0.301 > 0.30 或 `left_gripper_width` 0.01002 > 0.01）或 `FOLLOWER_HORIZON_EXCEEDED`（3.186 s > 3.0 s） | 1. P3-C 仅对夹爪重定时，未覆盖臂轴；<br>2. 单轴冲刺包络未计入 Ruckig 匀化曲率；<br>3. `marvin_follower_inference_v1.yaml` 右臂预算误留为 `0.05 rad` | 继承 anchor 数值计算错误、Ruckig 库本身故障 | 16 轴启用 `retime_action_window`；加入 `line_excess > 0.90 * budget` 触发；对称化 Marvin 左右臂偏差预算 |
| **动力学衔接** (`simulation`) | Marvin 交接时右臂大幅过冲 `0.26 rad`，触发 `FORBIDDEN_CONTACT: robot-robot contact 10.58 N` | 臂轴末点超出 1.0 s 内可减速停止范围，Ruckig 算 `>1.0 s` 曲线并在 `1.0 s` 截断，留下 `-0.7 rad/s` 初速度传入下一块导致刹不住车 | 模型预测本身要求撞击左臂 | 在 `transition_retiming.py` 中对超界臂轴末点施加 `_stop_bounds`（保证窗口末端可减速至 0） |
| **夹爪释放** (`runtime`) | 物块已放入箱底/目标块上，但夹爪一直停在闭合态 `0.0497 m` 直至 `TASK_TIMEOUT`（`right_released: false`） | 单帧模型在放置位前 40 点（`0..39`）模拟 Teacher 等待稳定门控，张爪动作在 `40..59` 点，固定选 `20..39` 永远够不到张爪点 | 夹爪执行器卡死、摩擦过大 | 检测到 `all_stationary` 且连续尾部 `tail_last > end + 0.004 m` 时，将释放目标推进至当前窗口 |
| **交接评价** (`simulation`) | Marvin 回放或闭环在左手张开释放瞬间报 `HOLD_LOST`（B09）；或空中独持 16 秒但 `left_airborne_stable` 恒为 `false`（B10） | B09：双持成立后左手张开斜坡段仍要求左指接触；<br>B10：14.2 s 桌面初触瞬间即锁死 `_sequence_references["left"]`，拔起微调后永久超差 | 物理上真的丢物或未离桌 | B09：双持证明后持物职责移交右手；<br>B10：仅在离桌且达 `lift_height_m` 时记录空中参考位姿 |
| **接触物理** (`simulation`) | Panda 抓起红块搬运中 `source_tilt` 缓慢增至 `1.57 rad`（翻转 90°）；或开启 `noslip` 后堆叠静止期 `source_angular_speed` 周期性跳至 `0.29 rad/s` 打断 `stable_s` | 1. `panda/scene.py` 自由物体默认 `condim=3`，无绕夹持轴的扭转摩擦力矩；<br>2. `noslip_iterations=3` 在 40 g 轻方块 box-box 接触上产生单步求解器微颤 | IK 姿态翻转、手臂在放置时拽翻方块 | `panda/scene.py` 对 `object_friction[1] > 0` 的物体设 `condim=4`、`cone=elliptic`、`impratio=10.0`，不开启 `noslip_iterations` |

---

## 5. 10 次闭环测试验收结果

使用固定模型 Bundle（Panda：`stack-act-refined2`；Marvin：`handover-act-refined-final`）与修复后的配置，在三端容器闭环下连续执行 10 次正式测试（Panda 5 次 + Marvin 5 次）：

| 序号 | 方案 | 种子 | 退出码 | 任务结果 | 完成/终止时间 | 物理条件明细 |
| --- | --- | ---: | ---: | --- | ---: | --- |
| 1 | `panda_stack` | 10027 | 0 | **`SUCCEEDED`** | 25.982 s | 14/14 条件全部通过，稳定满 2.0 s |
| 2 | `panda_stack` | 10007 | 0 | **`SUCCEEDED`** | 26.140 s | 14/14 条件全部通过，稳定满 2.0 s |
| 3 | `panda_stack` | 10045 | 0 | **`SUCCEEDED`** | 30.924 s | 14/14 条件全部通过，稳定满 2.0 s |
| 4 | `panda_stack` | 20013 | 0 | **`SUCCEEDED`** | 27.784 s | 14/14 条件全部通过，稳定满 2.0 s |
| 5 | `panda_stack` | 20060 | 0 | **`SUCCEEDED`** | 27.666 s | 14/14 条件全部通过，稳定满 2.0 s |
| 6 | `marvin_handover` | 4157 | 0 | **`SUCCEEDED`** | 52.744 s | 10/10 条件全部通过（左独持 15.54s → 双持 35.55s → 左释放 37.32s → 右独持 37.82s → 入箱稳定 1.0s） |
| 7 | `marvin_handover` | 4155 | 0 | `FAILED` | 33.672 s | `right_joint3` 在 18.3s 触发单轴重定时导致右臂交接位偏移，`EXTERNAL_SUPPORT` |
| 8 | `marvin_handover` | 4158 | 0 | **`SUCCEEDED`** | 49.722 s | 10/10 条件全部通过（左独持 15.50s → 双持 32.52s → 左释放 34.32s → 右独持 34.82s → 入箱稳定 1.0s） |
| 9 | `marvin_handover` | 5103 | 0 | `FAILED` | 35.628 s | 左手仍持物（9.82 N）时，双持刚满 0.5s 后右手微调减力超 0.25s 触发 `HOLD_LOST` |
| 10 | `marvin_handover` | 5104 | 0 | **`SUCCEEDED`** | 49.706 s | 10/10 条件全部通过（左独持 15.48s → 双持 33.46s → 左释放 35.65s → 右独持 36.15s → 入箱稳定 1.0s） |

- **总体成功率**：**8/10（80%）`TASK_SUCCEEDED`**（Panda **5/5 = 100%**，Marvin **3/5 = 60%**）。
- **单元测试回归**：Runtime 聚焦测试（`test_transition_retiming.py`、`test_policy_stream.py`）**24 passed**；Simulation 聚焦测试（`test_handover_task.py`、`test_handover_contact_evidence.py`、`test_stack_task.py`、`test_stack_contact_evidence.py`、`test_follower_transition_retiming.py`）**78 passed**，合计 **102 passed, 0 failed**。
- **证据归档路径**：
  - Panda 5 次闭环汇总与录包：`var/panda_stack/diagnostics/eval10-panda-20261116T093126Z/summary.json`
  - Marvin 5 次闭环汇总与录包：`var/marvin_handover/diagnostics/eval10-marvin-20261116T093645Z/summary.json`
  - 修改前文件备份：`/tmp/sim_dual_arm-repair-closedloop-1/original/`

---

## 6. 遗留问题与后续优化方向（已同步记录至根 README）

1. **多轴协同重定时（Coordinated Multi-Axis Retiming）**：当前 `transition_retiming.py` 按单轴独立触发并裁剪末点（如 Marvin `4155` 中仅 `right_joint3` 略超 `0.52 rad` 预算被裁剪，而同侧其余关节未按相同比例缩放），会引起单臂关节空间不同步并偏离笛卡尔轨迹。后续可改为同一机械臂 7 轴按最受限轴等比例缩放步长。
2. **交接过渡期双持到右独持的持物判据细化**：Marvin `5103` 显示在 `dual_airborne_stable` 刚满 `0.5 s`、但左手尚未开始释放（`left_min_force_n = 9.82 N` 仍稳固持物）时，若右手在二次夹紧微调中短暂减力超过 `0.25 s`，会被 `_holder_required(right)` 判为 `HOLD_LOST`。后续可在左手仍稳定持物（`left_holding == True`）期间保持双持候选状态，待左手开始释放后再严格要求右手不得失持。
3. **预取观测提前量与采集门控等待优化**：在 `target_count=20`（1.0 s）下，Runtime 在上一块刚开始时即预取下一块观测（提前约 1.0 s 采样并跳过前 20 点）；后续可在采集侧精简 Teacher 阶段末冗余等待帧，并在推理侧结合适度提前量（如 250–350 ms）与训练分组学习率进一步提升 OOD 泛化与动作平滑度。
