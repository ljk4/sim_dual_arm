# P05 短训练与新模型执行冒烟

前置：P04 的合格真实仿真数据和 4/2 划分可用。目标：从本任务数据生成一个新 ACT 模型，取得模型加载、Predict、真实后端执行反馈与资源收尾证据。

先验证新档案、任务文本和数据/训练配方能否驱动现有 ACT 流程。learning 中可复用的数据/训练运行、预算、模型管理和会话机制留在本端；ACT 模型、处理器和格式归算法适配，任务样本规则归任务选择与配方。接入 Marvin 不按机器人或任务名称复制 ACT 流水线，也不因跨任务复用而将其搬入 `shared/common/`。

## 数据转换与学习边界

1. 新 dataset 配方设 `diagnostic: true`，expected 总数/训练/验证为 6/4/2，validation_positions 为 `[4, 5]`。填本任务 task_text、Marvin profile、真实 select 与 export glob，输出到独立 learning 根。
2. 保持现有 16 维、三路 320×240、20 Hz、60 点/50 ms 契约。核对新 profile、action/observation 档案与实测标签顺序和单位；两侧夹爪均为总开度。
3. 转换使用实际采集和受限参考标签。归一化与图像处理器统计只使用训练 split；验证 episode 不参与统计拟合。保留 provenance、质量结果与 train/validation 清单。
4. 首轮不配置 `data.sample_filter`，保留全部合格训练锚点。现有静态筛选器硬编码右臂七轴和 stack 阶段，不能直接用于双臂交接；后续只有确有静态冗余影响时才改造它，本轮无需新增筛选框架。

## 有界训练与 checkpoint

- 使用 Marvin 独立 training 配方和预算台账，首轮 `smoke` 为 128 steps，预留与独立总训练预算均设为 900 秒；具体 batch/worker 按当前设备能实际运行的值设置。总预算到限即收尾，不自动加训或启动 full 模式。
- 按[第三方源码说明](../../third_party/README.md)和[当前训练流程](../../README.md)准备固定 LeRobot 源码、ACT 实现及离线 backbone 权重。缺少依赖在本端补齐，不为导入测试改变正式模型来源校验。
- smoke 训练结束保存真实 final/latest checkpoint、实际步数、有限 loss 与退出原因。若要记录验证 loss，显式启用独立验证并记录设置；首轮不以 loss 或任务成功率作为门槛。
- bundle 默认查找 `best.pt`，但未运行验证的 smoke 不保证产生 best。导出时显式选实际存在的 `final.pt` 或 `latest.pt`，保持训练配置与 checkpoint 的身份一致。

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

simulation/runtime 的 policy prepare 引用同一 bundle，runtime 同时读取 learning 实际生成的 policy_binding。按现有准备入口验证 profile、模型、处理器、run 身份；诊断准入不放宽数值/轨迹/控制约束。

## 后端执行冒烟与验收

启动新 Policy 服务、Marvin 推理后端和 runtime。推理后端使用 `handover` 的评价模式，保留物理 TaskState，不启用 Teacher 动作生成。首轮至多 3 次尝试，每次设置有限 max_chunks、仿真总时限和墙钟时限，记录到本任务独立日志；初始预算建议不超过 10 秒仿真、60 秒运行墙钟时间，启动预算另列。

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

短训练策略不以完成整项交接任务为门槛。初次接入关闭额外时间融合，使用本模型测得的执行参数；如确需融合或夹爪修正，须记录其对原始预测与接受参考的影响，不能直接复制 Panda 的修正幅度与系数。

`deliveries/phase-05.md` 保存数据/训练/bundle 来源、实际 checkpoint、步数/耗时、prepared 配置、各项协议/控制证据、任务结果和失败收尾。Panda 的配置、bundle、Policy 协议与正式准入回归通过。

职责验收确认：本次机器人/任务差异优先由档案和配方表达；实际需要的算法边界修复集中在已有 ACT 适配及相应消费者，不给通用训练/服务会话添加任务名分支。具备同一输入输出能力的实现继续复用，保持 learning 的 ROS-free 边界。

目录导航：[任务书入口](README.md) · [前一阶段 P04](phase-04-runtime-data.md) · [下一阶段 P06](phase-06-acceptance.md)
