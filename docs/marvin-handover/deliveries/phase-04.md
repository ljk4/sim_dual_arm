# P04 交付：Runtime 录制、导出、质量与诊断数据

日期：2026-11-13。**P04 六条诊断数据与 4/2 划分已交付；中断台账汇总已由 P05 前置修复补齐，见 [P05 交付](phase-05.md)**。工作树、P03/P04 原始录制、失败与中断证据保留；未暂存、提交或推送。P00～P03 交付记录未覆盖。

## 验收结果

Marvin 交接方案经本方案 campaign 取得 1 条合格 pilot 和 6 条合格诊断示教，全部来自真实 runtime Teacher → 受限执行参考 → MuJoCo 机器人 → TaskState 物理终态。三路 RGB、实测机器人状态、TaskState、接受参考、执行反馈与运行事件按同一 episode 身份完整录制；导出生成 1120 条 20 Hz 标签并按 `SELECTED.json` 划分为 4 训练 / 2 验证。

| 证据 | 结果 |
| --- | --- |
| pilot 合格 | seed 3101，`SUCCEEDED`，56.1 s 仿真、1120 标签、质量合格 |
| 6 条诊断数据 | seed 4100–4105，全部 `SUCCEEDED`；每次约 57.3 s 仿真、1120 标签、质量合格 |
| 4/2 划分 | [SELECTED.json](../../../var/campaigns/marvin_handover_v1/SELECTED.json)；训练 4100–4103，验证 4104–4105，零基位置 `[4, 5]` |
| 边界与同绝对时刻审计 | [boundary_audit_absolute_time.json](../../../var/reports/p04/boundary_audit_absolute_time.json)；6 条均满足任务成功、质量合格、全部标签落在实际执行区间；raw 对照覆盖 6720/6720 标签 |
| 中断采集 | ledger `INTERRUPTED` / exit 130；runtime `SIGTERM` → 停止、释放、录制关闭、日志关闭；任务如实 `CONTROL_FAILED: RUNTIME_SHUTDOWN` |
| P03 预检 | 已有录制在新目录完成导出，11 个阶段区间与 1120 标签成立；P03 录制未开启质量事件，质量按显式缺失证据判不合格，未退化为全局评价 |
| Panda 回归 | 真实 Panda 导出仍 9 阶段、480 标签、质量合格；采集选择与标签/质量测试通过 |

每次成功的关键实测（仿真时间，单位 s）：

| seed | task start | right catch | left release | task complete | 箱底支撑 (N) | 最终物块中心 (m) |
| --- | --- | --- | --- | --- | --- | --- |
| 4100 | 1.226 | 37.822 | 41.178 | 57.354 | 2.943 | `(0.640247, -0.269751, 0.821892)` |
| 4101 | 1.214 | 37.822 | 41.178 | 57.356 | 2.943 | `(0.640242, -0.269750, 0.821892)` |
| 4102 | 1.188 | 37.772 | 41.128 | 57.304 | 2.943 | `(0.640247, -0.269751, 0.821892)` |
| 4103 | 1.204 | 37.822 | 41.178 | 57.354 | 2.943 | `(0.640247, -0.269751, 0.821892)` |
| 4104 | 1.196 | 37.772 | 41.128 | 57.306 | 2.943 | `(0.640259, -0.269739, 0.821892)` |
| 4105 | 1.218 | 37.822 | 41.178 | 57.354 | 2.943 | `(0.640247, -0.269751, 0.821892)` |

每次都是独立复位后的新任务：`task_epoch` 依次为 6、8、10、12、14、16，任务起点约 1.2 s，右手接稳到左手释放间隔 3.356 s，终态物块位于箱内并由箱底支撑。

## 录制内容与数据边界

每个 episode 的 MCAP 保存：

- 三路 `320×240` RGB：`/robot/cameras/{global,left_wrist,right_wrist}/frame`，与实测状态同一精确捕获。
- `/robot/state`：16 维实测关节/夹爪状态；`/robot/task_state`：完整 TaskState 与 `metrics_json`。
- `/runtime/commands`、`/robot/follower_references`、`/robot/control_events`、`/robot/execution_feedback`：Teacher 请求、实际接受参考、逐块执行事件与实测反馈。
- `/runtime/observation_index`：观测、状态与三路帧的对应关系。

导出目录保存 `labels.json`、`raw_predictions.json`（原始 60 点 Teacher 块）、`frame_index.json`、`quality.json`、`summary.json`、全部标签帧 RGB、阶段边界预览 PNG 和三路 GIF。TaskState、接触、物块位姿、箱底支撑与分割计数只进入 `quality.json`、`summary.json` 和边界审计，不进入策略观测；学习输入仍限于 profile 声明的三路 RGB 与 16 维实测状态。

动作标签是**观测后 50 ms 实际受限执行参考**的采样，标签行同时保存 `reference_start_sim_time_ns`、实际 `started_sim_time_ns` 与块结束时间。录制关闭前执行一次 MCAP 审计，确认观测、命令、参考、反馈与事件序列一致；`recording.complete = true` 只在审计通过后写入。

## 阶段区间、导出与质量修复

- `export_quality.py` 不再固定 stack 阶段集合。阶段顺序与必需阶段由 campaign `quality.phases` / `quality.required_phases` 提供，区间从本任务 `follower.chunk_accepted` 事件的实际阶段和精确观测时间生成。
- 无阶段区间、必需阶段缺失或阶段事件与观测时间不对应时，导出显式不合格；删除“无阶段时静默按全局可见性评价”的退路。Panda 配方显式声明原 9 阶段，行为不变。
- 标签数量上限由 campaign `export.max_labels` 提供（本方案 1400，覆盖 57.8 s 的 1155 个精确捕获）；不再把 30 s 的默认 700 套到本任务。
- `stack/*/export` 仍是采集与学习消费端的真实约定，未改路径布局；`collect_attempt`、`evaluate` 和 P05 转换使用同一约定。
- `select` 从配方生成划分说明，新增 `--out`：仅筛选成功后写入新清单，已有清单拒绝覆盖。本方案清单说明为“前 4 条训练、后 2 条验证（零基位置 4、5），诊断用途”，不再出现 50/45/5。
- campaign 停止宽限由 `launch.stop_grace_s` 提供（本方案 45 s），使中断时 runtime 能在有界时间内完成控制停止、释放、录制与日志关闭；中断事实写入 ledger 的 `INTERRUPTED`。
- 交接质量随阶段选择对象与相机：左手抓取看左腕/全局，空中交接看物块与左右腕，运输/放置看右腕/全局与箱口；不要求左腕始终看到桌面。

最终质量参数（`handover_campaign.yaml`）：

| 参数 | 值 |
| --- | --- |
| 阶段顺序 | `left_approach, left_grasp, left_lift, handover_approach, dual_hold, left_release, left_retreat, right_transport, right_place, right_release, stable` |
| 必需阶段 | `left_grasp, dual_hold, left_release, right_release` |
| 合格比例 / 最长遮挡 | 0.85 / 0.35 s |
| 全局 `block` 像素下限 | 128（全部阶段 required；实测最小 284） |
| 左腕 `block` 像素下限 | 256（`left_grasp`～`left_retreat` required；实测最小 1260） |
| 右腕 `block` 像素下限 | 256（`dual_hold`～`stable` required；实测最小 1682） |
| 全局 `box` 像素下限 | 128（`right_transport`～`stable` required；实测最小 456） |
| 记录项 | 左腕 `block`（`left_approach` 与运输后）、右腕 `block`（`handover_approach`）、右腕 `box`、全局 `table` |

上述门槛来自真实渲染测量：pilot 测量试采（seed 3100）的 [quality.json](../../../var/campaigns/marvin_handover_v1/attempts/pilot-03100-record-1794543323584080369/stack/record-1794543323584080369/export/quality.json) 记录逐个规则的最小像素、合格比例和最长遮挡；左腕在 `left_approach` 的比例为 0.773、右腕在 `handover_approach` 的比例为 0.517，因此这两段改为记录项，其余所需阶段实测比例均为 1.0。按最终规则重导出后该 episode 质量合格。6 条诊断数据的 required 规则实测比例均为 1.0、最小像素 284、最长遮挡 0 s。

## 预算、清单与划分

| 用途 | 预算 | 实际 | 说明 |
| --- | --- | --- | --- |
| pilot | 3 次，`pilot_act_reserved_attempts=0` | 2 次 | seed 3100 为渲染测量试采；seed 3101 为合格 pilot |
| formal（诊断） | 18 次，目标 6 | 6 次 | seed 4100–4105，全部合格 |
| 中断 | 占 pilot 预算 | 1 次 | seed 3102，`INTERRUPTED`；pilot 预算至此用尽 |

种子区间与 Panda 分开（pilot 3100–3102，formal 4100–4105），台账、目录与数据根均在本方案 `var/campaigns/marvin_handover_v1/`。原始字节 7.83 GB / 10.74 GB（10 GiB 上限），采集期间磁盘余量始终高于 5 GiB 门禁。P03 的五次复位成功未计入上述配额。

`SELECTED.json` 由 `campaign --scenario marvin_handover select --out ...` 生成，未手写成功台账、清单或 manifest。6 条均为 `diagnostic: true`；P05 dataset 配方继续显式设置该标记，不作为正式训练证据。

## 原始与导出路径

容器内 `/workspace/var` 与 `/data` 映射到仓库 `var/`。

| seed | 尝试目录 | 原始 episode | 导出 |
| --- | --- | --- | --- |
| pilot 3100 | `var/campaigns/marvin_handover_v1/attempts/pilot-03100-record-1794543323584080369` | `stack/record-1794543323584080369/raw/pilot-03100-...` | `stack/record-1794543323584080369/export` |
| pilot 3101 | `var/campaigns/marvin_handover_v1/attempts/pilot-03101-record-1794543506070566260` | `stack/record-1794543506070566260/raw/pilot-03101-...` | `stack/record-1794543506070566260/export` |
| formal 4100 | `var/campaigns/marvin_handover_v1/attempts/formal-04100-record-1794543618762556154` | `stack/record-1794543618762556154/raw/formal-04100-...` | `stack/record-1794543618762556154/export` |
| formal 4101 | `var/campaigns/marvin_handover_v1/attempts/formal-04101-record-1794543708253208317` | `stack/record-1794543708253208317/raw/formal-04101-...` | `stack/record-1794543708253208317/export` |
| formal 4102 | `var/campaigns/marvin_handover_v1/attempts/formal-04102-record-1794543797927051087` | `stack/record-1794543797927051087/raw/formal-04102-...` | `stack/record-1794543797927051087/export` |
| formal 4103 | `var/campaigns/marvin_handover_v1/attempts/formal-04103-record-1794543887740049704` | `stack/record-1794543887740049704/raw/formal-04103-...` | `stack/record-1794543887740049704/export` |
| formal 4104 | `var/campaigns/marvin_handover_v1/attempts/formal-04104-record-1794543977683774201` | `stack/record-1794543977683774201/raw/formal-04104-...` | `stack/record-1794543977683774201/export` |
| formal 4105 | `var/campaigns/marvin_handover_v1/attempts/formal-04105-record-1794544066827257368` | `stack/record-1794544066827257368/raw/formal-04105-...` | `stack/record-1794544066827257368/export` |

其他证据：

- P03 预检导出：`var/exports/p04-preflight-p03-05/`；摘要 `var/reports/p04/preflight-p03.json`。
- 测量试采按最终规则重导出：`var/exports/p04-pilot-3100-final-rules/`；摘要 `var/reports/p04/preflight-pilot3100.json`。
- Panda 真实导出回归：`var/exports/p04-panda-regression-pilot01000/`；摘要 `var/reports/p04/preflight-panda.json`。
- prepared backend：`var/prepared/p04-sim-teacher/`；后端配置启用 `render_quality`，日志 `var/runs/backend/p04-backend-pilot/log/events.*.jsonl`（9 次 reset、9 次 task start、35762 条质量事件）。
- collection 校验：`var/reports/p04/collection-validate.log`；campaign 状态与筛选输出：`var/reports/p04/campaign-status.txt`、`var/reports/p04/select-output.txt`。

实际采集命令（runtime 容器 `/workspace`，simulation 容器 `/app`）：

```bash
# simulation：准备并启动启用分割计数的 Teacher 后端
scripts/run.sh prepare --scenario marvin_handover --role teacher --seed 0 \
  --run-id p04-sim-teacher --output /data/prepared/p04-sim-teacher
scripts/run.sh backend --config /data/prepared/p04-sim-teacher/backend.yaml \
  --run-id p04-backend-pilot

# runtime：pilot 与诊断采集
runtime/ros2/scripts/record.sh --scenario marvin_handover --purpose pilot \
  --count 1 --max-consecutive-failures 1 \
  --backend-log /data/runs/backend/p04-backend-pilot/log
runtime/ros2/scripts/record.sh --scenario marvin_handover --purpose formal \
  --count 6 --max-consecutive-failures 3 \
  --backend-log /data/runs/backend/p04-backend-pilot/log

# runtime：台账与工具生成的 SELECTED 清单
runtime/ros2/scripts/run.sh campaign --scenario marvin_handover status
runtime/ros2/scripts/run.sh campaign --scenario marvin_handover select \
  --out /data/campaigns/marvin_handover_v1/SELECTED.json
```

## 标签与 handoff 时序审计

边界审计覆盖 `left_grasp`（左手闭合）、`dual_hold`（接稳）、`left_release`（左手释放）和 `right_release`（右手释放）。每条 episode 的边界帧、标签索引、参考应用时刻与图像路径见 [boundary_audit_absolute_time.json](../../../var/reports/p04/boundary_audit_absolute_time.json)。raw/applied 比较使用同一命令、同一块及同一来源观测，在 `label_time - raw_source_observation_time` 处取样；6720/6720 标签均有精确时间点，无插值、裁剪或外推。

| 审计项 | 结果 |
| --- | --- |
| 标签落在实际执行区间 | 6/6 episode 全部 1120 条标签满足 `reference_start ≤ actual_start ≤ label_time ≤ block_end` |
| 标签时刻 | 边界观测 + 50 ms；`reference_start` 与 `actual_start` 相等，说明块按调度交接时刻开始，标签取实际应用参考而非 Teacher 源时间 |
| Teacher 源滞后与对照范围 | 标签观测距原始块来源观测 0.25–2.10 s；标签对应 raw 块内偏移 0.30–2.15 s，全部处于原始块 0.05–3.00 s 的覆盖范围，时间残差为零 |
| 同绝对时刻差异 | 每条 1120 标签中，臂关节 456 条不同、夹爪 48 条不同，合计 504 条；六条全局最大臂差 0.032903 rad、夹爪差 0.002870 m（差异阈值分别为 1e-6 rad / 1e-6 m） |
| 交接段 | 每条 `handover_approach` 的 320 标签中，臂关节 152 条不同，最大差 0.032448 rad；夹爪无差异 |
| 四个关键边界 | 同绝对时刻 raw/applied 差仅为数值舍入量级，最大臂差 4.90e-10 rad、夹爪差 6.42e-11 m；标签来源仍由实际接受参考与应用时刻确定 |
| 执行跟踪 | 边界处实际应用参考与实测状态最大差 0.00004–0.0062（rad 或 m），与 follower 跟踪容差一致 |
| 观测连续性 | 标签观测周期全为 50 ms，无缺口；开头/结束拒绝为 `no_started_block` 29–30 条与关机 `brake_replaced` 4 条 |

原 [boundary_audit.json](../../../var/reports/p04/boundary_audit.json) 与脚本保留。其边界图像和执行区间检查仍可复用；其中 965/1120、交接段 1.049 rad 是不同绝对时刻的比较，不用于同一时刻 raw/applied 偏差判断。当前对照统计以新报告为准；实际执行标签、原始录制和 SELECTED 未改变。

边界图像（formal 4100，全部 6 条同结构）：

- 左手闭合：[left_wrist](../../../var/campaigns/marvin_handover_v1/attempts/formal-04100-record-1794543618762556154/stack/record-1794543618762556154/export/preview/left_grasp_left_wrist.png) · [global](../../../var/campaigns/marvin_handover_v1/attempts/formal-04100-record-1794543618762556154/stack/record-1794543618762556154/export/preview/left_grasp_global.png)
- 双手共持：[global](../../../var/campaigns/marvin_handover_v1/attempts/formal-04100-record-1794543618762556154/stack/record-1794543618762556154/export/preview/dual_hold_global.png) · [right_wrist](../../../var/campaigns/marvin_handover_v1/attempts/formal-04100-record-1794543618762556154/stack/record-1794543618762556154/export/preview/dual_hold_right_wrist.png)
- 左手释放：[global](../../../var/campaigns/marvin_handover_v1/attempts/formal-04100-record-1794543618762556154/stack/record-1794543618762556154/export/preview/left_release_global.png) · [right_wrist](../../../var/campaigns/marvin_handover_v1/attempts/formal-04100-record-1794543618762556154/stack/record-1794543618762556154/export/preview/left_release_right_wrist.png)
- 右手释放：[global](../../../var/campaigns/marvin_handover_v1/attempts/formal-04100-record-1794543618762556154/stack/record-1794543618762556154/export/preview/right_release_global.png) · [right_wrist](../../../var/campaigns/marvin_handover_v1/attempts/formal-04100-record-1794543618762556154/stack/record-1794543618762556154/export/preview/right_release_right_wrist.png)

## 中断采集

seed 3102 在第 19 s 左右由操作者中断。ledger 写入 `RESERVED` → `INTERRUPTED`（exit 130，`interrupted`）；runtime 依次记录 `run.stopping SIGTERM` → `control.stop_completed` → `control.release_completed` → `recording.completed`（`recording_complete: true`，6739/6739 条写入）→ `episode.finished EPISODE_INTERRUPTED` → `run.finalizing`；日志 `logging_closed: true`。episode manifest 记录 runtime `interrupted/SIGTERM`、任务 `failed / TASK_FAILED: CONTROL_FAILED: RUNTIME_SHUTDOWN`，不记成功。ledger 的 `recording_complete` 为 false 是因为 collector 随中断终止、未把 manifest 复制到尝试根目录；完整关闭证据在尝试内的 episode manifest 与 runtime 日志中。完整摘录见 [interrupt-evidence.json](../../../var/reports/p04/interrupt-evidence.json)。

中断前已有两个 pilot episode 连续成功，且 6 条 formal 的 `task_epoch` 递增、任务起点回到约 1.2 s、终态物块位置一致，场景复位后重录不继承上局任务状态。

## 回归

| 验证 | 结果 | 日志 |
| --- | --- | --- |
| runtime 全量单元（排除宿主缺少 `dual_arm_learning` 的终端进度测试） | **949 passed, 31 skipped** | `var/reports/p04/regression-unit.log` |
| Panda 采集选择、导出质量、标签、campaign | **109 passed** | `var/reports/p04/regression-panda-labels-quality.log` |
| Panda 真实 episode 导出 | 9 个 stack 阶段、480 标签、质量合格 | `var/exports/p04-panda-regression-pilot01000/` |
| Panda campaign 选择 | `campaign --scenario panda_stack status` 正常，预算与目录不变 | `var/reports/p04/` 命令输出 |
| Marvin 真实导出 | 11 个阶段、1120 标签、required 规则比例 1.0、遮挡 0 s | 6 条 formal 的 `quality.json` |

31 个 skip 仍是宿主缺少 `mujoco`/`mink` 等可选模块，与 P00 已知缺口一致。P03 门禁、模型、摩擦、容差、任务评价和连续 5 次复位未改动；P02B 物理诊断与 P03 完整示教不受本轮离线导出、质量规则和后端渲染计数配置影响，未重跑。

## 职责边界与修复

- 通用录制/导出不导入本任务 Teacher 类型、不求解交接位姿；Teacher 扩展仍由 `marvin_handover_api_msgs.teacher_chunk.v1` 解析。
- 阶段集合、必需阶段和质量规则属于本方案配方；通用工具只消费配置与录制事实。Panda 用同一工具与自己的阶段列表通过。
- campaign 的预算、种子、台账、磁盘门禁与筛选机制保持共用；本方案只改选择说明、输出清单和停止宽限，未复制同步、租约、动作发送或收尾流程。
- `TaskState`、分割计数和物块真值只用于门禁、阶段区间、质量审计与边界审计；导出观测保持三路 RGB 与实测状态。

## 未完成与下一阶段

- 中断 pilot 3102 已通过台账追加接口补齐真实终态：录制完整、runtime interrupted/SIGTERM、task failed、wall 42.384136 s、不合格。原身份、预算与历史行保留，六条 SELECTED 不变；证据见 [interrupt-backfill.json](../../../var/reports/p05/interrupt-backfill.json)。
- 训练、bundle、Policy 服务与推理冒烟属于 P05；binding 仍未声明 dataset/training/policy 配方和模型。
- P05 dataset 需要显式 `diagnostic: true`、expected 6/4/2、validation positions `[4, 5]`，并消费 [SELECTED.json](../../../var/campaigns/marvin_handover_v1/SELECTED.json) 与 `stack/*/export`。
- learning 现有静态样本筛选规则仍固定右臂七轴与 stack 阶段；本轮未配置 `data.sample_filter`，P05 首轮也不启用。
- 固定布局 6 条样本只验证软件链路与数据格式，不证明泛化；扩大规模需先给出具体失败原因和新预算。
- P03 录制未启用分割质量事件，预检只能验证阶段与标签；本任务质量门槛由 P04 的真实渲染测量确定。

目录导航：[任务书入口](../README.md) · [P03 交付](phase-03.md) · [P05](../phase-05-learning.md)
