# P05 短训练与新模型执行冒烟

前置：P04 的合格真实仿真数据和 4/2 划分可用。目标：从本任务数据生成一个新 ACT 模型，取得模型加载、Predict、真实后端执行反馈与资源收尾证据。

当前交付已取得真实训练、bundle、后端执行和收尾证据。2026-11-13 用户明确本轮“跑通链路即可，不必追求仿真推理成功”，因此按诊断链路验收 P05 已完成，见 [phase-05.md](deliveries/phase-05.md)。已有 inference 配方扩大了 `max_target_deviations`，该差异必须记录；本次验收不声称模型在原轨迹保真容差下可用，也不要求为此追加实验。

转换与训练只消费 profile 声明的 RGB、实测机器人状态，以及观测后 50 ms 的实际受限执行参考标签。TaskState、接触、物块位姿、分割及成功判定真值保留为审计元数据，不进入策略输入特征或替代动作标签。

先验证新档案、任务文本和数据/训练配方能否驱动现有 ACT 流程。learning 中可复用的数据/训练运行、预算、模型管理和会话机制留在本端；ACT 模型、处理器和格式归算法适配，任务样本规则归任务选择与配方。接入 Marvin 不按机器人或任务名称复制 ACT 流水线，也不因跨任务复用而将其搬入 `shared/common/`。

## 前置修复结果

- P04 的 6 条 SELECTED 数据、4/2 划分和实际执行标签已核验可用，不重新采集。[同绝对时刻审计](../../var/reports/p04/boundary_audit_absolute_time.json) 已覆盖全部 6720 标签。raw Teacher 对照须使用同一 command/chunk 和同一绝对标签时刻；按 `label_time - raw_source_observation_time` 取原始块内偏移，并验证在该块时间范围内，不能直接使用当前观测后的 50 ms 偏移。
- 中断 pilot 3102 已通过 `campaign backfill-terminal` 从真实产物追加终态：录制完整、runtime interrupted/SIGTERM、task failed、wall 42.384136 s、不合格。原 INTERRUPTED、attempt/seed、预算占用与历史行保留，SELECTED 不变。后续中断汇总继续要求有界收尾后读取真实 episode 产物。
- 任务配置现仅在 Teacher 开启时校验计划时长。独立 inference 配方显式禁用 Teacher、评价预算 10 s，实际 prepare 为 evaluation-only；P03 的 180 s 配方保持不变。

## 数据转换与学习边界

在本阶段创建真实 dataset/training/policy_service、runtime_policy 与 simulation_policy 配方，再将已存在的引用加入 binding。learning 阶段不需要模型；inference 阶段在新 bundle 生成后才校验，不能为提前通过校验填 Panda 模型。

1. 新 dataset 配方设 `diagnostic: true`，expected 总数/训练/验证为 6/4/2，validation_positions 为 `[4, 5]`。填本任务 task_text、Marvin profile、真实 select 与 export glob，输出到独立 learning 根。
2. 保持现有 16 维、三路 320×240、20 Hz、60 点/50 ms 契约。核对新 profile、action/observation 档案与实测标签顺序和单位；两侧夹爪均为总开度。
3. 转换使用实际采集和受限参考标签。归一化与图像处理器统计只使用训练 split；验证 episode 不参与统计拟合。保留 provenance、质量结果与 train/validation 清单。
4. 首轮不配置 `data.sample_filter`，保留全部合格训练锚点。现有静态筛选器硬编码右臂七轴和 stack 阶段，不能直接用于双臂交接；后续只有确有静态冗余影响时才改造它，本轮无需新增筛选框架。

## 有界训练与 checkpoint

- 使用 Marvin 独立 training 配方和预算台账，首轮 `smoke` 为 128 steps，预留与独立总训练预算均设为 900 秒；具体 batch/worker 按当前设备能实际运行的值设置。总预算到限即收尾，不自动加训或启动 full 模式。
- 按[第三方源码说明](../../third_party/README.md)和[当前训练流程](../../README.md)准备固定 LeRobot 源码、ACT 实现及离线 backbone 权重。缺少依赖在本端补齐，不为导入测试改变正式模型来源校验。
- smoke 训练结束保存真实 final/latest checkpoint、实际步数、有限 loss 与退出原因。若要记录验证 loss，显式启用独立验证并记录设置；首轮不以 loss 或任务成功率作为门槛。
- bundle 默认查找 `best.pt`，但未运行验证的 smoke 不保证产生 best。导出时显式选实际存在的 `final.pt` 或 `latest.pt`，保持训练配置与 checkpoint 的身份一致。
- 训练完成后保留其原始 YAML 和数据绑定，bundle 仍使用同一训练配置。若后续调整配方，则另存新配置/运行，不能修改旧训练配置后直接导出原 checkpoint。

## 新 bundle 与非正式服务准入

bundle 包含本次权重、Marvin profile、处理器/训练统计及数据/训练来源。不得替换为 Panda 权重、复用旧统计或手写 policy_binding。

当前 `prepare-policy` 已提供 `--no-require-formal`，服务配方也支持 `model.require_formal: false`。本任务诊断模型使用该现有入口；保留 `service.test_only: false` 和真实 bundle，不采用 Fake Policy。正式模式继续要求 full checkpoint、正式数据规模并拒绝 diagnostic 数据，禁止修改全局默认值来放行本次 smoke。

以下命令为实现完成后的入口模板，均在 learning 容器 `/workspace` 执行；将尖括号占位符替换为实际值，输出目录每次使用新名称：

```bash
learning/lerobot/scripts/run.sh convert --scenario marvin_handover
learning/lerobot/scripts/run.sh train --scenario marvin_handover --mode smoke

# TRAIN_RUN 指向上条命令实际生成的目录
TRAIN_RUN="/data/learning/marvin_handover_diag/runs/<actual-run>"
learning/lerobot/scripts/run.sh bundle --scenario marvin_handover --mode smoke \
  --run "$TRAIN_RUN" --checkpoint final.pt \
  --output "/data/learning/marvin_handover_diag/bundles/<new-bundle>"

learning/lerobot/scripts/run.sh prepare-policy --scenario marvin_handover \
  --bundle "/data/learning/marvin_handover_diag/bundles/<new-bundle>" \
  --no-require-formal --run-id "<policy-run>" \
  --output "/data/prepared/<policy-run>/policy"
```

simulation/runtime 的 policy prepare 引用同一 bundle，runtime 同时读取 learning 实际生成的 policy_binding。按现有准备入口验证 profile、模型、处理器、run 身份。诊断模式保留有限数值、关节硬限位、速度/加速度及制动校验；服务修正与 inference 专用轨迹保真容差分别记录。已扩大的保真容差仅作为本次链路冒烟条件，不提升为正式模型执行参数，后续也不继续扩大以追求通过。

## 后端执行冒烟与验收

启动新 Policy 服务、Marvin 推理后端和 runtime。推理后端配置 `task_mode: inference`，`handover` 只运行物理评价，不输出 Teacher 动作。runtime 明确使用现有 `termination_mode: task`，使 PrepareTask/StartTask 与物理评价实际启动。

首轮最多 3 次尝试，统一使用本方案 campaign 的 `evaluation` 用途（`evaluation_max_attempts=3`，从 5100–5119 区间分配未用种子）及其台账；失败、拒绝、取消均计入，不另从 `test` 用途再增加三次。默认 `task_sim_budget_ns=10000000000`（10 秒）、`task_stop_margin_s=1.0`、`total_wall_deadline_s=60`。60 秒从流对象建立起计时，包含流内 startup settle，启动子超时不在其外额外累加；服务进程启动与有界收尾另记录实际耗时。按实际 follower.target_count 配置足够 `max_chunks`，使 `max_chunks × target_count × 50000000 ≥ task_sim_budget_ns`；任务评价器的短时诊断配方与此预算一致，不改 P03 完整 Teacher 的配方。

task 模式的实际截止由 `task_sim_budget_ns` 与 stop margin 决定，不能只改 `total_sim_deadline_ns`。`finite_chunks` 当前不调用任务 prepare/start，本轮不采用它替代所需的评价生命周期。预算截止时任务未完成应如实记录，链路验收仍按下面的执行与收尾证据判断。

逐项报告真实结果：

| 证据 | 通过含义 |
| --- | --- |
| 新模型加载 | 服务实际加载本次 bundle 权重、处理器和 Marvin 档案，身份一致 |
| Open / Predict | 使用当前后端 RGB/实测状态，返回新模型产生的有限 60×16 动作 |
| 提交 / 接受 | 新预测经过现有限位与参考处理，真实后端接受至少一个执行窗口 |
| 实测执行 | 接受窗口与 command/chunk、实测 RobotState/执行反馈可对应，物理步进推进且无该窗口的控制失败；仅返回接受响应不足以通过 |
| 任务结果 | 如实保存成功、失败或未完成，不将链路冒烟记为交接任务成功 |
| 收尾 | 有界停止、控制终态、Policy Close、租约与录制/日志关闭；本次进程不残留 |

前四项与收尾必须取得真实证据。若新模型所有窗口都被安全拒绝，应保留拒绝原因，结果为“预测通过、执行冒烟未完成”；回到模型/契约/执行对应问题修复，不能关闭保护、用 Teacher 接管或把没有执行的结果标成完成。达到预算后留下明确未完成项，不自动延长训练。

短训练策略不以完成整项交接任务为门槛，也不要求在本阶段达到可用的策略效果。当前 128 step 足以验证训练和执行链是否相连；后续 5K 量级或更长训练另设数据、计算和评价预算，不在本轮自动加训。初次接入关闭额外时间融合，使用本模型测得的执行参数；如确需融合或夹爪修正，须记录其对原始预测与接受参考的影响，不能直接复制 Panda 的修正幅度与系数。

`deliveries/phase-05.md` 保存数据/训练/bundle 来源、实际 checkpoint、步数/耗时、prepared 配置、各项协议/控制证据、任务结果和失败收尾。Panda 的配置、bundle、Policy 协议与正式准入回归通过。

职责验收确认：本次机器人/任务差异优先由档案和配方表达；实际需要的算法边界修复集中在已有 ACT 适配及相应消费者，不给通用训练/服务会话添加任务名分支。具备同一输入输出能力的实现继续复用，保持 learning 的 ROS-free 边界。

目录导航：[任务书入口](README.md) · [前一阶段 P04](phase-04-runtime-data.md) · [下一阶段 P06（已交付）](deliveries/phase-06.md)
