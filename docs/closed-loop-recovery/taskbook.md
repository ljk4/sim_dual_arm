# 闭环成功率回退修复与工程规范收敛任务书

分支：`fix/closed-loop-and-ownership`（阶段一：开发阶段，遵循 [`AGENTS.md`](../../AGENTS.md) §1.1）  
进度追踪：[progress.md](progress.md)

---

## 1. 问题排查结论与回退根因

`6ba0f73` 提交在不重采数据、不重训模型的前提下取得了 8/10（80%）闭环成功率（Panda 5/5、Marvin 3/5）。随后的重构与参数变更（`87e0ed5`～`ac6c32c`）使旧数据集与旧模型作废。对新采数据与新模型（`panda_sim/stack`: `act_20261117_082737`）的实测盘点显示：19 次有效策略闭环推理仅成功 2 次（10.5%），`marvin_sim/handover` 仅完成 50 条采集尚未转换与训练。

### 1.1 成功率回退三大直接根因

1. **`panda_sim/stack` Teacher 采集窗口配置耦合与阶段相位时钟未对齐 `handoff_time_ns`**：
   - `panda_sim/stack` 的 Teacher 采集与策略推理共用了同一个 `shared/scenarios/panda_sim/stack/profiles/control/panda_follower_v1.yaml`（`target_count: 20`，即 1.0 s 执行窗口），而 `marvin_sim/handover` 已将采集（`target_count: 10`）与推理（`target_count: 20`）拆分为两份独立配置。
   - `simulation/mujoco/src/robot_backend_sim/panda_sim/stack/__init__.py`、`teacher.py` 与 `gate.py` 未像 `marvin_sim/handover` 那样将阶段运动起点绑定到块交接时间 `handoff_time_ns`，而是从早 1.0 s 的观测门控通过时刻 `actual_start_ns` 开始采样 smoothstep 轨迹。
   - 结果：每阶段前 20 步（1.0 s 平滑加速段）在 `handoff_time_ns` 之前被 `ActionWindow` 丢弃，窗口首点发生大跨度阶跃；阶段末产生 34～39 帧（1.7～1.95 s）的 `WAITING_FOR_FEEDBACK` 死等平台；全部 50 条正式数据均在旧 30 s 预算下于第 580 帧（29.0 s）被截断。
2. **阶段末静止帧过滤未压缩目标动作序列且保护过宽**：
   - `learning/lerobot/src/dual_arm_learning/sample_filter.py` 仅在 DataLoader 采样器侧过滤查询锚点，不改写 Parquet 时间轴与 60 步未来目标 `action[t : t+60]`。
   - `_protections()` 将静止平台内部的 `gate_state` 跳变（`TRACKING -> WAITING_FOR_FEEDBACK`）误判为阶段边界并保护 $\pm 5$ 帧，叠加保守阈值（`min_frames: 15, edge_frames: 5, interior_stride: 5`），训练集仅剔除 22.63% 的查询帧（55% 静止帧仍保留在训练集中）。
3. **公共轨迹层 `transition_retiming.py` 混入场景魔数与跨段单轴启发式**：
   - `shared/common/python/trajectory/src/robot_trajectory/transition_retiming.py` 硬编码了 `0.068`、`0.060`、`0.056`、`other_closing`、`all_stationary` 等场景特定启发式与单轴独立裁剪逻辑。
   - 在新数据下引发手臂尚未下探到位时提前 1 s 关爪抓空、以及放置张爪后全臂 7 轴跳跃重定时导致震荡撞翻方块或无法退避（`right_tcp_clearance < 0.08 m`）。

### 1.2 `README.md` 第 5 节“待后续改进”逐项核查与归属划分

| 项目 | 核查定位与核心问题 | 代码归属划分 |
| :--- | :--- | :--- |
| **1. 多轴协同过渡重定时** | `transition_retiming.py` 逐轴独立裁剪，改变单臂 7 轴联合运动方向并改变笛卡尔轨迹；同时违规混入场景启发式魔数 | 协同轴组拓扑由机器人层 `RobotProfile`（`shared/common/python/contracts/.../models.py`）派生；纯数学等比例缩放算法归属 `shared/common/python/trajectory/.../transition_retiming.py`；运行端 `PolicyStreamRunner` 显式接入；彻底清除场景魔数 |
| **2. 空中交接持物移交边界细化** | `marvin_sim/handover/evaluator.py` 在 `dual` 刚满 `0.5 s` 但左手尚未释放时，右手短暂微调减力触发 `HOLD_LOST`，且左爪脱离瞬间右爪位姿参考未刷新 | 属于 `simulation/mujoco` 下机器人与任务绑定层：`simulation/mujoco/src/robot_backend_sim/marvin_sim/handover/evaluator.py` |
| **3. 视觉骨干与头部分组学习率** | `training_engine.py` 将 `policy.parameters()` 全部设为单一 `lr = 1e-4`，未向 `ACTConfig` / `AdamW` 拆分 `lr_backbone = 1e-5` | 通用机制归属 `learning/lerobot`（`config.py`、`training_support.py`、`training_engine.py`）；具体学习率数值配置归属 `shared/scenarios/<robot-id>/<task-id>/configs/learning/act_training.yaml` |
| **4. 冷启动预热与异步预取推理** | `OpenSession` / `startup_settle` 未预热完整推理链路；首块后立即在 0% 进度采样过早观测且同步阻塞等待 `Predict` RPC | 跨会话预热归属 `learning/lerobot`（`pipeline.py`、`policy_service.py`）；会话流调度/预热/异步预取归属 `runtime/ros2`（`runtime_config.py`、`target_source.py`、`chunk_stream.py`）；预热时长与开关参数归属各方案 `shared/scenarios/<robot-id>/<task-id>/configs/runtime/` |

---

## 2. 分阶段实施计划与验收标准

- **P01 — 更新 `AGENTS.md`**：补充 `### 2.4 改动/新增代码归属判断与破例确认准则`，明确“端无关 (`shared/`) vs 四端内”与“端内 `common/` vs `<robot-id>/` vs `<robot-id>/<task-id>/`”的两级判断流程，以及破坏性改动/偏离规范需提前请示用户许可的原则。
- **P02 — 多轴协同过渡重定时**：在 `RobotProfile` 派生 `coordinated_axis_groups`，在 `transition_retiming.py` 实现同臂多轴按最受限轴等比例协同缩放并彻底清除场景硬编码启发式，在 `PolicyStreamRunner._retime` 接入并在 `tests/unit/test_transition_retiming.py` 补充单元测试。
- **P03 — 修复 Panda Teacher 时钟对齐、配置拆分与静止帧过滤**：对齐 `panda_sim/stack` 阶段运动起点与 `handoff_time_ns`；拆分 `panda_follower_v1.yaml`（采集 `target_count: 10`）与 `panda_follower_inference_v1.yaml`（推理 `target_count: 20`）；修正 `sample_filter.py` 的 `_protections()` 与两端 `act_sample_filter.yaml` 阈值。
- **P04 — 启动预热增强与异步预取推理**：服务端 `OpenSession` 调用 `pipeline.warm_up()`；扩展 `startup_settle_sim_ns` 上限并设为 3.0 s，开启 `warmup_on_settle` 在 `StartTask` 前执行真实观测预热 `Predict`；在 `ContinuousStreamRunner` 实现 `async_inference` 后台异步推理与 `prefetch_lead_ns` 准时预取；补充单元测试。
- **P05 — 空中交接移交边界细化与视觉骨干分组学习率**：细化 `marvin_sim/handover/evaluator.py` 的 `_holder_required` 移交过渡守卫与左爪释放后的右爪稳定性参考刷新；为 `learning/lerobot` 增加 `training.lr_backbone` 支持并在两方案 `act_training.yaml` 配置 `1e-5`。
- **P06 — 归属清理、全量单测回归与文档同步**：修正 `hardware/README.md` 单层方案路径，清理残留孤立 `.pyc`，同步更新 `README.md` §5 与 `docs/experiments/closed-loop-triage.md`，通过容器运行相关单元测试并汇总结果（重采与重训待代码确认后执行）。
