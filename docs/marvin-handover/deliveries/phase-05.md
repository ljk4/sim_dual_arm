# P05 交付：诊断数据短训练、新 bundle 与 Policy 执行冒烟

日期：2026-11-13。**按用户最新明确的“跑通链路即可”目标，P05 诊断链路验收完成，可进入 P06。** 数据转换、128 step 训练、checkpoint、新 bundle、真实 Predict、后端执行和收尾均有产物。三次 evaluation 尝试全部计入台账；第三次使用扩大跟随偏差预算，接受 4 个窗口，并在第 5 次提交被拒绝后受控停止，未运行至 10 s 预算截止，任务未记成功。P04 中断台账已按真实录制产物补录终态。工作树、历史证据保留；未暂存、提交或推送。P00～P04 交付记录未覆盖。

验收口径更新：此前按任务书“不放宽轨迹约束”条款将原容差下执行列为未完成。用户随后明确本轮只验证链路，不追求短训练策略效果，因此当前诊断条件下的实测执行用于本轮验收；原容差下可用性不再阻塞 P06。该更新不改变历史尝试的失败结果、配置或预算，也不把扩大容差说成保持原约束。后续 5K 量级训练与策略成功率验证另行安排，本轮不加训。

## 验收结果

| 证据 | 实际结果 |
| --- | --- |
| 新模型加载 | Policy 服务加载 `p05-smoke-128` bundle：bundle digest `7ddcdb62…d70a83`、processor `85f627f5…9895faf2`、model sha `57a4a92e…52fb3c2`、checkpoint smoke/128、Marvin profile `bc815504…7bd0947` |
| Open / Predict | 6 次保存了副产物的 Predict 全部返回有限 `60×16` 原始动作；原始/准入副产物逐次保存，臂关节修正恒为 0，夹爪修正最大值 `0.003181 m` |
| 提交 / 接受 | 扩大轨迹保真预算的最后一次实验：同一逻辑 command 提交 5 个 chunk 请求，接受 4 个，拒绝 1 个 |
| 实测执行 | 前 3 个窗口有 21 条执行反馈/窗口，`STARTED`、`FOLLOWING`，最大位置误差 `0.00342`；第 4 窗口启动后在停止中 `CANCELED`；实测状态从复位位姿移动最大 `1.369 rad` |
| 任务结果 | 未完成：`task_result.outcome=failed`，`TASK_FAILED: CONTROL_FAILED: RUNTIME_SHUTDOWN`，任务态 `RUNNING → FAILED`；不记为交接成功 |
| 收尾 | 控制停止/释放成功、Policy `CloseSession` OK、`policy.closed exit_code=0`、录制完整关闭、后端 `closed=true`、runtime 日志关闭、进程不残留 |

## 诊断执行条件与轨迹保真限制

`max_target_deviations` 比较拟合参考与锚点至逐时刻预测目标的分段线性轨迹，是触发 `FOLLOWER_DEVIATION_EXCEEDED` 的实际准入门槛。它与关节硬限位、实测状态跟踪参考的误差是不同约束。

原配方的臂轴预算统一为 0.05 rad；inference 左臂改为 `[0.30, 1.30, 0.52, 0.98, 0.55, 0.44, 1.51] rad`，最大为原值 30.2 倍。右臂仅 `right_joint5` 从 0.05 提至 0.10 rad，其余右臂轴仍为 0.05。第三次后端/runtime 实际加载同一数组；`right_joint2 0.104619 > 0.05` 的拒绝与配置一致。

这些窗口证明新权重产生的动作确已驱动物理机器人，并正常收尾，满足更新后的诊断链路目标；它们不证明模型可在原容差下执行。Teacher 配方及硬限位保持原值，轨迹保真容差确已扩大。服务侧“臂修正为 0”只描述服务准入前后；0.00342 的实测跟踪误差只描述机器人跟踪后端参考，二者均未量化模型目标到后端拟合参考的差异，本轮不据此报告策略保真性能。

[执行事件复核](../../../var/reports/p05/execution_evidence_reconciled.json) 区分了 command 与 chunk 生命周期：前三个 chunk 分别在 3.35 / 4.35 / 5.35 s 正常 `RETIRED / HANDOFF`，第四个在 5.622 s `CANCELED`，第五个未接受。它们共享同一逻辑 command，不能把 command 的 2.37 s 起点作为每个 chunk 的起点，也不要求流式交接的每个 chunk 都以 command 级 `COMPLETED` 结束。

evaluation 预算已用尽（3/3）。P06 复用当前执行证据，不新增推理尝试、换用 test 预算或加训。短训练模型未完成整项交接任务、原容差下可用性尚未取得证据均如实保留，不作为本轮链路验收的阻塞项。

## 接口与统计离线复核

新增 [interface_audit.json](../../../var/reports/p05/interface_audit.json) 及只读复核脚本 [audit_interface.py](../../../var/reports/p05/audit_interface.py)，读取既有 dataset/bundle、processor 统计和六份预测，不调用模型、训练或仿真。宿主机仓库根目录执行：

```bash
python3 var/reports/p05/audit_interface.py
```

- dataset、bundle state/action、observation/action 档案的 16 轴顺序一致：左臂 7 轴、左总开度、右臂 7 轴、右总开度。转换与服务路径使用绝对位置/开度，未发现增量动作混用。
- processor 的 state/action mean/std 与训练统计摘要最大差 `5.96e-8`，预处理、ACT 预测、反归一化调用路径未发现统计错配或重复归一化。
- 以 `e3296b…npz` 为例，60 步内左/右臂各轴变化范围最大分别为 `0.000691 / 0.001777 rad`；相对训练 action mean/std 的标准分 RMS 为 `0.9275`。证据支持块内时间变化小，不支持直接断言输出训练均值，也未定位其训练原因。

本次复核没有发现需要修改执行机制或算法适配的接口缺陷。未开展逐帧图像核对或模型内部张量追踪，不据此宣称策略效果正常；这些检查不作为链路收口的附加条件。

## 数据转换

dataset 配方 [act_dataset.yaml](../../../shared/scenarios/marvin_handover/configs/learning/act_dataset.yaml) 显式 `diagnostic: true`、expected 6/4/2、`validation_positions: [4, 5]`，消费 P04 工具生成的 [SELECTED.json](../../../var/campaigns/marvin_handover_v1/SELECTED.json)（sha256 `f7f14f79…33af6d4`，未改写）与 `stack/*/export`。转换在 learning 容器 `/workspace` 执行：

```bash
learning/lerobot/scripts/run.sh convert --scenario marvin_handover
```

| 项 | 结果 |
| --- | --- |
| 输出 | `var/learning/marvin_handover_diag/dataset`，73 MB，manifest sha256 `0790c6c0…c3f0c1e` |
| episode / 帧 | 训练 4100–4103 / 4480 帧；验证 4104–4105 / 2240 帧；每条 1120 标签 |
| 输入特征 | 仅三路 `320×240` RGB 与 16 维实测状态；TaskState、接触、物块位姿与分割真值不进入策略输入 |
| 标签 | 观测后 50 ms 的实际受限执行参考（`applied_positions`），原始 60 点 Teacher 块只保留为审计字段 |
| 统计 | `processors/train_stats_summary.json` 为 `training_split_only`，action count 4480（仅训练集）；验证 episode 未参与拟合 |
| 契约 | 20 Hz、`60×16`、50 ms、三路相机、两侧夹爪均为总开度 |

首轮未配置 `data.sample_filter`，4 条训练 episode 全部保留。

## 有界训练与 checkpoint

training 配方 [act_training.yaml](../../../shared/scenarios/marvin_handover/configs/learning/act_training.yaml) 使用独立预算台账；smoke 128 steps，预留与总预算均为 900 s。设备实测 RTX 4050 6 GiB，batch 16、2 workers、bfloat16 AMP。执行：

```bash
learning/lerobot/scripts/run.sh train --scenario marvin_handover --mode smoke
```

| 项 | 结果 |
| --- | --- |
| run | `var/learning/marvin_handover_diag/runs/smoke-20261113T052745Z-353` |
| 步数 / 退出原因 | 128/128，`completed` |
| 耗时 / 预算 | 53.08 s；台账 RESERVED 900 s、FINISHED `completed`、`elapsed_seconds=53.07` |
| loss | 有限；step 120 `loss=0.5706`、`l1=0.2596`、`kl=0.0311`；最后进度行 `loss=0.53876` |
| VRAM | 峰值分配 1870 MiB / 保留 1982 MiB |
| checkpoint | `final.pt`（sha256 `b5a327f1…169627`）与 `latest.pt`，均为 step 128；未运行验证，`best_val_l1=null` |
| 报告 | [training-summary.json](../../../var/reports/p05/training-summary.json)、run 内 `training_report.json` |

训练配置与数据绑定保持原样；未加训、未进入 overfit/full。预算台账：[training-budget.jsonl](../../../var/learning/marvin_handover_diag/training-budget.jsonl)。

## Bundle 与非正式准入

从本次 `final.pt` 导出：

```bash
learning/lerobot/scripts/run.sh bundle --scenario marvin_handover --mode smoke \
  --run /data/learning/marvin_handover_diag/runs/smoke-20261113T052745Z-353 \
  --checkpoint final.pt \
  --output /data/learning/marvin_handover_diag/bundles/p05-smoke-128
```

bundle [p05-smoke-128](../../../var/learning/marvin_handover_diag/bundles/p05-smoke-128) 的 digest 为 `7ddcdb62…d70a83`，manifest sha256 `59952313…e16787ca`。内容为本次权重、Marvin profile、训练集统计、dataset/training 来源与 processors；`diagnostic=true`、`synthetic=false`、4 train / 2 validation、`state_dim=16`、`chunk_size=60`。导出时用训练集真实样本做了同一 `predict_action_chunk` 往返，形状 `60×16`、最大差 `0.0`。bundle 摘要：[bundle-summary.json](../../../var/reports/p05/bundle-summary.json)。

binding 已加入 dataset/training/policy_service/runtime_policy/simulation_policy 引用，并将 `models.bundle` 指向该真实 bundle（无 Panda 权重或占位摘要）。诊断服务通过现有入口准入：

```bash
learning/lerobot/scripts/run.sh prepare-policy --scenario marvin_handover \
  --bundle /data/learning/marvin_handover_diag/bundles/p05-smoke-128 \
  --no-require-formal --run-id p05-policy-2 --output /data/prepared/p05-policy-2/policy
```

生成的 `policy.yaml` 保持 `service.test_only: false`，仅 `model.require_formal: false`；正式准入的默认值未改。policy binding sha256 `6fa6af46…a386c12`。

## 首轮执行参数

policy 配方 [policy.yaml](../../../shared/scenarios/marvin_handover/configs/learning/policy.yaml) 首轮关闭时间融合（`temporal_ensemble_coeff: null`），并按本模型实测输出设置夹爪修正上限 `0.005 m`：训练帧与首次实际推理观测上，原始总开度最多超上界 `0.0032 m`；修正只作用于 `left/right_gripper_width`，臂关节修正为 0（[prediction-artifacts.json](../../../var/reports/p05/prediction-artifacts.json) 的 max_arm_correction 全为 0）。

inference 后端 [backend_marvin_policy.yaml](../../../shared/scenarios/marvin_handover/configs/simulation/backend/backend_marvin_policy.yaml) 使用 `task_mode: inference`；任务配方 [marvin_handover_task_inference.yaml](../../../shared/scenarios/marvin_handover/configs/simulation/scene/marvin_handover_task_inference.yaml) 关闭 Teacher，评价预算 10 s 仿真 / 60 s 墙钟。后端 `backend.task.prepare` 记录 `prepare_kind: evaluation_only`、`step_count=0`，未输出 Teacher 目标。

runtime 配方 [policy_eval.yaml](../../../shared/scenarios/marvin_handover/configs/runtime/policy_eval.yaml)：`task_sim_budget_ns=10 s`、`task_stop_margin_s=1.0`、`total_wall_deadline_s=60`、`startup_settle_sim_ns=2 s`、`max_chunks=16`，按 follower `target_count=20` 覆盖 16 s ≥ 10 s；任务配方 `max_task_sim_duration_s=10` 与 runtime 一致。prepare 生成 [runtime.yaml](../../../var/prepared/p05-policy-2/runtime-follower/runtime.yaml)（sha256 `3abbc554…c3d3a`）。

短训练模型的第一个 chunk 在复位位姿上偏离实测状态最多 `1.369 rad`，原跟随预算 `0.05 rad` 会拒绝。实验新增 [marvin_follower_inference_v1.yaml](../../../shared/scenarios/marvin_handover/profiles/control/marvin_follower_inference_v1.yaml)，左侧最高 1.51 rad、仅 right_joint5 为 0.10 rad、夹爪 0.01 m。硬限位、速度/加速度和制动可行性保留；轨迹保真门槛确已放宽，该配方仅用于本次诊断链路，不作为正式执行参数或原容差通过证据。

## 后端执行冒烟

服务在 learning 容器启动，simulation/runtime 使用同一 bundle。三次尝试全部使用本方案 campaign 的 `evaluation` 用途与 [handover_campaign.yaml](../../../shared/scenarios/marvin_handover/configs/runtime/experiments/handover_campaign.yaml)，seed 5100–5102 依次分配，失败/拒绝均计数（3/3），未使用 `test`，未占 pilot/formal。

| seed | attempt | runtime 配置 | 结果 | 关闭原因 |
| --- | --- | --- | --- | --- |
| 5100 | `evaluation:5100:p05-eval-1` | `p05-policy-1`（无夹爪修正） | failed | `INVALID_PREDICTION: target[0] axis 7 out of range`（左夹爪原始值超上界） |
| 5101 | `evaluation:5101:p05-eval-2` | `p05-policy-2`（夹爪修正 0.005） | failed | `FOLLOWER_DEVIATION_EXCEEDED: left_joint1 0.266789 > 0.05` |
| 5102 | `evaluation:5102:p05-eval-3` | `p05-policy-2/runtime-follower` | failed | 第 5 窗口 `right_joint2 0.104619 > 0.05`；前 4 窗口已接受并执行 |

三次 runtime 均 `recording.complete=true`、`runtime.outcome=failed`、`task.outcome=failed`，wall 分别 6.00 s、6.00 s、13.00 s；原始字节 29.3 / 30.1 / 79.7 MB。台账见 [attempts.jsonl](../../../var/campaigns/marvin_handover_v1/attempts.jsonl)，状态见 [campaign-status-after.txt](../../../var/reports/p05/campaign-status-after.txt)。

最后一次尝试的逐步证据（[execution_evidence.json](../../../var/reports/p05/execution_evidence.json)）：

| chunk | command | 观测 | 接受/执行 | 反馈 | 峰值位置误差 | 结果 |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | `STREAM_FIRST`，60×16 | 42 | 接受，2.37 s 开始 | 21 条，`STARTED`/`FOLLOWING` | 0.00325 | 执行 |
| 2 | `STREAM_NEXT`，60×16 | 43 | 接受 | 21 条，`STARTED`/`FOLLOWING` | 0.00342 | 执行 |
| 3 | `STREAM_NEXT`，60×16 | 67 | 接受 | 21 条，`STARTED`/`FOLLOWING` | 0.00180 | 执行 |
| 4 | `STREAM_NEXT`，60×16 | 86 | 接受，启动 | 3 条，`STARTED→STOPPING→CANCELED` | 0.00009 | 停止中取消 |
| 5 | `STREAM_NEXT`，60×16 | 107 | 未接受 | 0 | — | 跟随偏差拒绝 |

同一时段 `/robot/state` 112 条、`/runtime/commands` 5 条提交消息（同一逻辑 command 的不同 chunk）、`/robot/follower_references` 4 条、`/robot/execution_feedback` 92 条、`/runtime/observation_index` 112 条；每份请求轨迹 60 点 × 16、有限，首点 50 ms、末点 3.0 s。实测状态从复位位姿到停止移动的最大单轴增量 `1.369 rad`，与模型目标方向一致。Predict 的原始/准入 6 份 `60×16` 数组全部有限；原始与准入的差仅存在于两侧夹爪（≤`0.0032 m`）。

任务侧：`behavior_phase` 到 `released`，`task_start_sim_time_ns=2.072 s`、`sample_sim_time_ns=5.636 s`，无 `SUCCEEDED`；任务以 `TASK_FAILED: CONTROL_FAILED: RUNTIME_SHUTDOWN` 结束。Policy 服务收到 `CloseSession` 并返回 OK，最终 `policy.closed exit_code=0`；后端 `backend.close.end closed=true`；runtime `control.stop_completed`、`control.release_completed`、`recording.completed`、`run.finalizing` 均存在且日志关闭。服务与后端进程在证据采集后停止，不残留。

服务日志：[p05-policy-2](../../../var/logs/act-policy/p05-policy-2/)；后端日志：[p05-backend-policy-2](../../../var/runs/backend/p05-backend-policy-2/)；失败尝试目录：[evaluation-05100…](../../../var/campaigns/marvin_handover_v1/attempts/evaluation-05100-p05-eval-1/)、[evaluation-05101…](../../../var/campaigns/marvin_handover_v1/attempts/evaluation-05101-p05-eval-2/)、[evaluation-05102…](../../../var/campaigns/marvin_handover_v1/attempts/evaluation-05102-p05-eval-3/)。

## 中断台账补录

P04 的 pilot 3102 录制已完成关闭，但通用中断汇总在进程组仍在收尾时读取产物，留下 `recording_complete=false`、runtime/task 空字段与 `wall_s=0`。本次修复：

- `campaign_execution.py` 在中断停止时等待整个启动进程组完成有界收尾，并携带实际 wall 时长；终态分类在尝试根缺少复制件时读取 `stack/*/raw/*/{manifest,task_result}.json` 的唯一真实录制。
- `campaign.py` 新增 `backfill-terminal` 追加接口：只接受已有 `INTERRUPTED/FAILED` 的尝试，保留原 event、seed、purpose、mode、预算与历史行，只追加一份从真实产物重读的终态。

对 `pilot:3102` 执行后追加的终态为：`recording_complete=true`、`runtime=interrupted/SIGTERM`、`task=failed`、`wall_s=42.384136`、`qualified=false`；原 `INTERRUPTED` 行仍在，seed 3102、pilot 预算占用与三类 pilot 尝试计数不变。补录记录见 [interrupt-backfill.json](../../../var/reports/p05/interrupt-backfill.json)。未重采、未改 `SELECTED.json`、未新增采集预算。

## 配方与配置修复

- [task_config.py](../../../simulation/mujoco/src/robot_backend_sim/tasks/handover/task_config.py)：`teacher.max_plan_duration_s < success.max_task_sim_duration_s` 只在 Teacher 开启时校验。P03 的 180 s Teacher 配方不变；短时 inference 配方可在 Teacher 关闭时使用 10 s 评价预算。
- 新增 simulation policy 后端、inference 任务配方、runtime policy 配方、learning dataset/training/policy_service 配方，并加入 binding；模型由真实 bundle 提供。
- 学习端复用现有转换、预算、checkpoint、bundle 与 Policy 会话机制；ROS-free 边界未变，未按方案名增加任务分支。

## 回归

| 验证 | 结果 | 日志 |
| --- | --- | --- |
| runtime 容器全量单元 | 964 passed、30 skipped、3 failed | [regression-unit.log](../../../var/reports/p05/regression-unit.log) |
| learning 容器 learning/policy/Panda 相关 | 114 passed | [regression-learning-policy.log](../../../var/reports/p05/regression-learning-policy.log) |

3 个失败均为 `test_learning_console_progress.py` 在 runtime 容器缺 `dual_arm_learning` 的既有模块路径问题；同一测试在 learning 容器通过，与 P04 的已知缺口一致。覆盖包括：Marvin/ Panda binding 与 prepare、bundle 校验与 CLI、Policy 配置/契约/RPC、正式准入、campaign 选择与中断分类、handover 推理配方与预算一致性、learning 分配与训练清单。P04 的 SELECTED、六条导出、质量与 boundary audit 未改动；未重新采集。

Marvin 任务的 `task_config` 在 simulation 容器实际加载通过（inference 10 s 与 P03 180 s 两份配方），新 backend.yaml 在 simulation 容器实际启动；Panda 未使用本次新增的 inference 配方，原配置/bundle/正式准入路径由上述测试覆盖。

## 职责边界

- 数据转换、训练、bundle 与服务保持 ROS-free；未导入 MuJoCo/ROS/Teacher 类型。
- inference 评价只由任务插件执行同一物理评价，`prepare_kind=evaluation_only`，未输出 Teacher 动作。
- 执行参数按本模型实测输出设置在 inference 专用 follower profile；Teacher 的采集预算与门禁未改。
- campaign 的中断汇总与追加接口是通用机制，未包含方案名分支。

## 限制与下一阶段

- 四个接受窗口使用了推理专用保真容差，原容差下执行未得到验证；第 5 个预测窗口因 `right_joint2` 偏差被拒绝后受控停止。这不阻塞更新后的链路验收，也不构成策略可用的证据。
- inference follower 左臂保真容差最高为原值 30.2 倍，不能推广为正式执行参数。离线核对未发现轴顺序、绝对动作语义或处理器统计错配；“近均值输出”应更正为“短训练预测在 chunk 时间维上几乎平坦”，现有检查不足以认定模型直接输出训练均值。
- 6 条诊断数据只验证软件链路与数据格式，不证明泛化。
- P06 最终验收仍需：汇总各阶段实际产物与回归、补齐新方案 README 的可复制使用说明、更新总进度并集中列出限制。新模型执行反馈已取得，可在 P06 复用本记录。

目录导航：[任务书入口](../README.md) · [P04 交付](phase-04.md) · [P06](../phase-06-acceptance.md)
