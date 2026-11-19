# 全链路闭环排查方法与验收总结

本文总结双臂 ACT“采集—训练—推理—仿真”长链条闭环问题的标准化排查方法、跨模块故障根因对照表以及双任务闭环验收结果。

---

## 1. 自底向上四步排查法

面对长链路闭环失败（如机械臂原地停滞、中途被规划器拒绝、抓取翻转或成功放置后未判成功），严禁在末端盲目调参或在未经证实的假设下设计复杂跨端协议。必须按**从底层到高层、从确定性回放到在线闭环**的顺序逐环排除：

```text
[Step 1: 采集数据抽查与真实物理回放]
   ├── 核对 Parquet/MP4/PNG 数值一致性、时间戳连续性、静止帧占比与硬限位余量
   └── 脱离模型，用录制 Reference / 20Hz 标签 / Oracle Follower 在真实 MuJoCo 中回放
         │ (若回放失败 → 先修评价器或基础物理；回放 100% 成功 ↓)
[Step 2: 模型离线输出与归一化核查]
   ├── 校验 Bundle 内 safetensors 归一化统计量（mean/std）、关节顺序与相机通道映射
   └── 在抽样训练/验证帧上对比 60 点预测与真值（分段考察近期 1..10、中期 20..40、远期 40..59）
         │ (若预测异常 → 查训练管线；预测趋势与真值吻合 ↓)
[Step 3: 推理执行侧隔离与闭环追踪]
   ├── 逐层比对同一观测时间戳下的动作流：
   │   raw_act → admitted → execution_targets → FollowerReference → ExecutionFeedback
   └── 定位 ActionWindow 选窗、transition_retiming 整形、Ruckig 规划与 MotionRuntime 跟踪中的拦截点
         │ (解除执行器误拒、速度过冲与等待死锁后，观察物理终态 ↓)
[Step 4: 仿真物理环境与评价器边界对齐]
   ├── 检查抓取/搬运/叠放中的接触物理（如自由物体 condim 扭转摩擦、轻物体求解器微颤）
   └── 检查任务评价器状态机在接触建立、拔离桌面、双持交接与释放过渡期的边界条件
```

### 排查铁律

1. **先定位首发物理事件**：按时间戳查找后端与运行端日志中的首个拒绝或停止原因（例如后端 `TRACKING_TIMEOUT` 停止后，运行端请求下一块常表现为 `STREAM_ANCHOR_UNAVAILABLE`，切勿把后果当根因）。
2. **区分模型能力与执行截断**：当机械臂看似不动或停在半途时，必须同时对比完整 60 点预测块与实际截取窗口（`ActionWindow`），不能仅凭视频表现断定模型未学会动作。
3. **用 Oracle Follower 隔离规划器怀疑**：将数据集未来 60 点真值直接送入生产 `prepare_follower_plan`（Ruckig）与 `MotionRuntime`。若 Oracle Follower 100% 成功，即可排除“规划器本身无法执行任务轨迹”的假说。

---

## 2. 全链路故障现象与根因速查表

| 所属领域 | 故障现象与日志表现 | 真实根因 | 已排除的干扰假说 | 核心修复方法 |
| --- | --- | --- | --- | --- |
| **策略准入** (`learning`) | Marvin 开局第 3 块报 `POLICY_RPC_DATA_LOSS` / `left_joint7: arm prediction outside hard limits` | 示教 `left_joint7` 紧贴 `-1.5708 rad` 下限（仅差 `4.25e-7 rad`），线性头在未执行尾部（索引 46–50）轻微外推过冲 `0.005–0.008 rad` | 网络传输丢包、模型权重损坏 | 方案配置启用按轴有界输出准入 `max_prediction_corrections`（`left_joint7: 0.01 rad`），整块严格校验 |
| **时间融合** (`learning`) | 历史合法夹爪开度加权平均后报超出 `0.08 m` 上限；或抓取阶段夹爪关闭明显滞后 | 1. IEEE 754 浮点加权和产生 `0.08000000000000002 m` 尾数溢出；<br>2. 时间融合保留旧的张开预测，延迟了当前帧要求的快速闭合 | 模型预测开度越界、夹爪控制反向 | 1. 融合结果钳制在当次参与样本的凸包（`[min, max]`）内；<br>2. 按绝对目标仿真时间戳对齐，按场景权衡融合系数 |
| **窗口调度** (`runtime`) | 短窗口（2 点或 5 点）下机械臂原地微动或停在阶段起点直至 `TASK_TIMEOUT` | 1. 极短窗口受零初速加速约束推进不足；<br>2. 数据集含 32%–47% 阶段末门控等待静止帧，单帧模型前 5–10 点预测保持等待、20–40 点才展开动作 | 模型只会输出常数、视觉骨干失效 | 执行窗口扩至 `target_count: 20`（1.0 s）跨过等待区；在整机静止且尾部（`40..59`）有明确动作时推进尾部目标 |
| **跟随规划** (`runtime` / `simulation`) | 中途报 `FOLLOWER_DEVIATION_EXCEEDED` 或 `FOLLOWER_HORIZON_EXCEEDED`（Ruckig 曲线 `3.186 s > 3.0 s`） | 1. 过渡重定时仅覆盖夹爪而未覆盖臂轴；<br>2. 单轴冲刺包络未计入 Ruckig 匀化曲率；<br>3. `marvin_follower_inference_v1.yaml` 右臂预算误留为首帧静止测得的 `0.05 rad` | 继承 anchor 数值计算错误、Ruckig 库缺陷 | 全 16 轴启用仅限 Policy 的 `retime_action_window`；加入 `line_excess > 0.90 * budget` 触发；对称化 Marvin 左右臂偏差预算 |
| **动力学衔接** (`runtime` / `simulation`) | Marvin 交接时右臂大幅过冲 `0.26 rad`，触发 `FORBIDDEN_CONTACT: robot-robot contact 10.58 N` | 臂轴末点超出 1.0 s 内可减速到零的范围，Ruckig 规划出 `>1.0 s` 曲线并在 `1.0 s` 截断，留下 `-0.7 rad/s` 初速度传入下一块导致无法刹停 | 模型预测轨迹本身要求撞击对侧手臂 | 在 `transition_retiming.py` 中对超界臂轴末点施加 `_stop_bounds`（确保单窗口末端可平稳减速至零） |
| **导出与制动** (`simulation`) | 夹持或跨块切换时报 `INVALID_RUCKIG_EXPORT`（含飞秒级或亚纳秒级制动前缀） | C1 连续边界处 `1e-17`～`1e-10 m/s` 的浮点速度超限使 Ruckig 生成极短制动段，被旧导出器一刀切拒绝 | 速度上限配置错误 | 纳秒量化为 `0 ns` 的极小前缀安全折叠；`> 0 ns` 的真实制动段正式导出多项式系数并将主轨迹时间后移 |
| **夹爪释放** (`runtime`) | 物块已入箱或叠放到位，但夹爪一直停在闭合态 `0.0497 m` 直至 `TASK_TIMEOUT` | 单帧模型在前 40 点（`0..39`）模拟 Teacher 等待稳定门控，张爪动作在 `40..59` 点，固定截取 `20..39` 永远够不到释放点 | 夹爪执行器卡死、接触摩擦过大 | 检测到 16 轴静止（`all_stationary`）且连续尾部 `tail_last > end + 0.004 m` 时，将释放目标引入当前窗口 |
| **夹持跟踪** (`simulation`) | 稳定夹持方块期间报 `TRACKING_TIMEOUT` | 1. 闭合参考速度 `+5.3e-14 m/s` 落在零速度容差空隙；<br>2. 持物期间参考开度短暂微增（仍小于实测物块宽度且双指保持法向力），被旧逻辑当成释放过渡取消接触豁免 | 物理上发生滑脱或伺服失效 | 1. 统一零速度容差 `1e-9 m/s`；<br>2. 只要参考小于实测开度且满足双指同物体有效法向力与低指速，即維持有效夹持约束豁免 |
| **接触物理** (`simulation`) | Panda 抓起红块搬运中缓慢旋转 90°（`source_tilt -> 1.57 rad`）；或开启 `noslip` 后堆叠静止期角速度周期性跳变 | 1. `panda/scene.py` 自由物体默认 `condim=3`，无绕夹持轴的扭转摩擦力矩；<br>2. `noslip_iterations=3` 在 40 g 轻方块堆叠接触上产生单步求解器微颤 | IK 姿态翻转、放置时手部碰撞拽翻方块 | 自由物体启用 `condim=4`、`cone=elliptic`、`impratio=10.0`（搬运倾角降至 `0.00096 rad`），不启用 `noslip_iterations` |
| **评价与汇总** (`simulation` / `runtime`) | 1. Marvin 左手正常张开时误报 `HOLD_LOST`（B09）；<br>2. 空中独持 16 s 仍判 `left_airborne_stable=false`（B10）；<br>3. 离线汇总将 1.0 s 合法交接判为失败（B01） | 1. 双持成立后左手张开斜坡段仍被要求左指接触；<br>2. 桌面初触瞬间即锁死左手空中参考位姿，拔起微调后永久超差；<br>3. 离线汇总硬编码 `stable_elapsed_s >= 2.0` | 物理上真的丢物、空中晃动超标 | 1. 双持证明后持物职责移交右手；<br>2. 仅在离桌且达高度门槛时记录参考位姿；<br>3. 离线汇总直接消费 sidecar 冻结的成功事实 |

---

## 3. 闭环验收结果与后续优化方向

### 3.1 10 次正式三端闭环验收

在不重训模型、不重采数据、保持原有成功阈值与安全守卫的前提下，使用固定模型 Bundle（Panda：`stack-act-refined2`；Marvin：`handover-act-refined-final`）连续执行 10 次三端容器闭环测试，取得 **8/10（80%）`TASK_SUCCEEDED`**：

- **`panda_stack`（5/5 = 100% 成功）**：种子 `10027`（25.98 s）、`10007`（26.14 s）、`10045`（30.92 s）、`20013`（27.78 s）、`20060`（27.67 s），14/14 项物理条件全部通过并稳定支撑满 2.0 s。
- **`marvin_handover`（3/5 = 60% 成功）**：种子 `4157`（52.74 s）、`4158`（49.72 s）、`5104`（49.71 s），10/10 项条件全部通过（完整经历左独持 → 空中共持 → 左释放 → 右独持 → 入箱底板稳定满 1.0 s）；种子 `4155` 因单轴独立重定时引起交接位偏移失败，种子 `5103` 因左手未释放前右手二次微调短暂减力超 0.25 s 触发 `HOLD_LOST`。

### 3.2 后续迭代优化方向与最新代码修复

1. **同臂多轴协同重定时（已实现待端到端重验）**：由 `RobotProfile.coordinated_axis_groups` 提供机器人侧别轴组拓扑，公共层 `transition_retiming.py` 清除场景启发式魔数并按同侧手臂最受限轴比例等比例协同缩放所有运动关节位移。
2. **交接过渡期持物判据细化（已实现待端到端重验）**：`marvin_sim/handover/evaluator.py` 在左手仍稳固持物且右爪处于闭合接触微调期时豁免右手短暂减力误判，并在左爪完全脱离接触后刷新右爪稳定性相对位姿参考。
3. **Teacher 相位时钟对齐、静止帧过滤与预热/异步推理（已实现待重采重训验收）**：对齐 `panda_sim/stack` Teacher 阶段起点到 `handoff_time_ns` 并拆分采集/推理 `follower` 配置（`10` / `20`）；修正 `sample_filter.py` 仅保护真实 `phase` 边界；训练启用 `lr_backbone = 1e-5`；推理启用 `OpenSession` / `3.0 s` settle 真实观测预热与后台异步推理及可配置 `prefetch_lead_ns`。

---

目录导航：[实践索引](README.md) · [数据与模型诊断](data-and-model-diagnostics.md) · [推理轨迹执行](inference-trajectory-execution.md) · [仿真物理与评价器](simulation-physics-and-evaluators.md)
