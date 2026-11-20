# 闭环成功率回退修复与工程规范收敛任务书

分支：`fix/closed-loop-and-ownership`（阶段一：开发阶段，遵循 [`AGENTS.md`](../../../AGENTS.md) §1.1）

进度追踪：[progress.md](progress.md)

---

## 1. 问题排查结论与回退根因

`6ba0f73` 提交在不重采数据、不重训模型的前提下取得了 8/10（80%）闭环成功率（Panda 5/5、Marvin 3/5）。随后的重构与参数变更（`87e0ed5`～`ac6c32c`）使旧数据集与旧模型作废。对新采数据与新模型（`panda_sim/stack`: `act_20261117_082737`）的实测盘点显示：19 次有效策略闭环推理仅成功 2 次（10.5%），`marvin_sim/handover` 仅完成 50 条采集尚未转换与训练。

### 1.1 成功率回退三大直接根因

1. **`panda_sim/stack` Teacher 采集窗口配置耦合与阶段相位时钟未对齐 `handoff_time_ns`**：
   - `panda_sim/stack` 的 Teacher 采集与策略推理共用了同一个 `shared/scenarios/panda_sim/stack/profiles/control/panda_follower.yaml`（`target_count: 20`，即 1.0 s 执行窗口），而 `marvin_sim/handover` 已将采集（`target_count: 10`）与推理（`target_count: 20`）拆分为两份独立配置。
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
- **P03 — 修复 Panda Teacher 时钟对齐、配置拆分与静止帧过滤**：对齐 `panda_sim/stack` 阶段运动起点与 `handoff_time_ns`；拆分 `panda_follower.yaml`（采集 `target_count: 10`）与 `panda_follower_inference.yaml`（推理 `target_count: 20`）；修正 `sample_filter.py` 的 `_protections()` 与两端 `act_sample_filter.yaml` 阈值。
- **P04 — 启动预热增强与异步预取推理**：服务端 `OpenSession` 调用 `pipeline.warm_up()`；扩展 `startup_settle_sim_ns` 上限并设为 3.0 s，开启 `warmup_on_settle` 在 `StartTask` 前执行真实观测预热 `Predict`；在 `ContinuousStreamRunner` 实现 `async_inference` 后台异步推理与 `prefetch_lead_ns` 准时预取；补充单元测试。
- **P05 — 空中交接移交边界细化与视觉骨干分组学习率**：细化 `marvin_sim/handover/evaluator.py` 的 `_holder_required` 移交过渡守卫与左爪释放后的右爪稳定性参考刷新；为 `learning/lerobot` 增加 `training.lr_backbone` 支持并在两方案 `act_training.yaml` 配置。
- **P06 — 归属清理、全量单测回归与文档同步**：修正 `hardware/README.md` 单层方案路径，清理残留孤立 `.pyc`，同步更新 `README.md` §5 与 `docs/experiments/closed-loop-triage.md`，通过容器运行相关单元测试并汇总结果。

---

## 3. 第二轮实测问题排查与修复计划（Issues 1–8）

- **P07 — `panda_sim/stack` 推理失败与 `marvin_sim/handover` 训练/并行采集修复（Issues 5, 7, 8）**：
  - **Issue 8（`marvin_sim/handover` `FilterError: label episode_id disagrees with the attempt`）**：
    - 根因：`campaign.py` 对 4 位数种子（`4100..4199`）生成 `episode_id` 时使用 `f"{purpose}-{seed:05d}-{run_id}"`（即 `formal-04100-...`），而 `attempt_id` 未补零（`formal:4100:...`），导致 `sample_filter.py:381` 用 `attempt_id.replace(":", "-")` 校验时失败；同时 `sample_filter.py:139` 的 `_source_paths` 多写了前导 `logs/` 目录，导致所有 episode 都找不到运行端 `events.*.jsonl`、静默跳过 `phase_boundary_frames` 保护。
    - 修复：在 `sample_filter.py` 按 `attempt_dir.name` / `raw/manifest.json` 的 `episode_id` 与 `(purpose, seed, run_id)` 语义一致校验（直接兼容已采集的 50 条 Marvin 原始数据与 LeRobot 数据集），并修复 `_source_paths` 事件日志匹配路径。
  - **Issue 5（`panda_sim/stack` 推理未成功）**：
    - 根因：对 `act_20261118_060048` 离线与在线回放核查表明，虽然模型与训练流程正常，但在仅有 5,000 step 的训练预算下，将 `lr_backbone` 从 `1e-4` 骤降 10 倍至 `1e-5` 加上 `sample_filter.py:139` 丢失阶段边界保护，导致 ResNet18 视觉骨干欠拟合（`best_val_l1 = 0.0465`，远高于历史 `0.0132 rad`），即便是训练种子 `10000` 也出现 `+0.078 rad`（约 5.5 cm）的横向定位偏移并在 4 cm 方块旁抓空闭环振荡。
    - 修复：修复 `sample_filter.py` 阶段边界保护；保留 `learning/lerobot` 分组学习率机制并将两方案 `act_training.yaml` 默认 `lr_backbone` 恢复为 `0.0001`（匹配 5,000 step 下 `~0.013 rad` 收敛验证值，并在 `training_args.yaml` 记录 `lr_backbone`）。
  - **Issue 7（`marvin_sim/handover` 并行采集连续 3 次失败）**：
    - 根因：`pool-1794985076901478387` 中 4 路并发以 `realtime_speed: 1.0` 采集且并行运行 `export_episode.py`（每局压缩 2,760 张 PNG）时，物理任务与相机可见性其实全部成功（`TASK_SUCCEEDED`, `vis_qualified=True`），但后端 `queues.render_snapshots: 4`（20 Hz 下仅 200 ms 缓冲）在 transient 渲染阻塞 >200 ms 时丢弃 1~17 帧快照，打破了导出器严格零间隙 20 Hz 连续性检查（`trajectory_qualified=False`）。
    - 修复：在两方案后端配方将 `queues.render_snapshots` 由 `4` 提升至 `32`、`queues.render_outputs` 由 `8` 提升至 `32`（符合 `BackendConfig <= 64` 契约），提供 1.6 s 瞬态抗抖动缓冲。
- **P08 — 后端启动/停止就绪反馈与并行进度剩余时间（Issues 1, 2）**：
  - **Issue 1**：在 `backend_cli.py::_serve_node` 启动完成（`_emit_backend_start` 后）和关闭完成时输出明确且立即刷新的控制台就绪/停止状态反馈。
  - **Issue 2**：为 `record_pool.py` 与 `inference_worker.py` 的进度条补充 `HH:MM:SS` 格式的 `用时` 与估算 `剩余` 时间（`--:--:--` 到 `00:00:00`），并在 TTY 轮询等待期间定期刷新进度。
- **P09 — 全链路日志与 `run-id` 统一时间戳结尾（Issues 3, 4）**：
  - 统一 `record.sh`、`record_parallel.sh`、`convert.sh`、`train.sh`、`teacher_backend.sh`、`inference_backend.sh`、`client_policy.sh`、`client_interactive.sh`、`server_policy.sh`、`inference_parallel.sh` 等日志目录/文件/默认 `run-id` 使用可读本地时间戳 `_<YYYYMMDD_HHMMSS>` 结尾（不再使用 `time.time_ns()` 或 `-$$` 后缀）；当向 `teacher_backend.sh` / `inference_backend.sh` 传入 `--run-id <前缀>` 时自动追加 `_<YYYYMMDD_HHMMSS>`（同秒冲突追加 `_2`），确保可重复运行且不破坏 `default_backend_log` 自动发现。
- **P10 — 更新 `README.md` §4/§5 与全量回归验收（Issue 6）**：
  - 在 `README.md` §4 为 `marvin_sim/handover` 补充 `./env.sh viewer` 可视化命令；移除已全部定位并修复的 §5（`## 5. 已发现的待后续改进问题`）；运行三端容器回归单元测试并更新 `progress.md`。
