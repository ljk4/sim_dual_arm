# P06 收口状态：最终验收与交付

日期：2026-11-13。**P06 已完成。** 本轮按“链路跑通”口径验收，不要求 128 step 策略完成交接或达到成功率；未新增训练、采集或 Policy rollout，evaluation 保持 3/3。工作树与历史产物未暂存、提交或推送。

## 验收结论

| 项 | 结果 |
| --- | --- |
| Teacher 完整物理任务 | 同一最终模型/配方连续 5 次复位成功，5/5 `TASK_SUCCEEDED`；P04 另有 6 条同任务诊断成功 |
| 失败与停止 | 右手未接稳时左手保持持物；SIGINT 中断完成停止、释放与录制关闭 |
| 数据 | 6 条诊断示教、1120 标签/条、4/2 SELECTED；[绝对时刻边界审计](../../var/reports/p04/boundary_audit_absolute_time.json) 6720/6720 标签可比 |
| 新模型执行 | 新 bundle 实际加载、6 份有限 Predict、1 个逻辑 command 的 4 个窗口被接受并执行、完整收尾；任务仍 `failed` |
| 职责边界 | 跨端公共、端内机制、机器人/任务适配与任务实现分离；静态导入检查无反向依赖 |
| 回归 | 复用 P03～P05 既有结果；补验最后代码改动影响的 campaign 测试 12 passed，learning 容器终端进度测试 7 passed |
| Panda | 复用 P03/P04 已有 API、采集选择、导出质量与真实导出结果；无 Panda 路径代码改动 |

## 关键统计与限制

- P05 的 5 次提交属于同一逻辑 command：chunk 1–3 在 3.35/4.35/5.35 s `RETIRED / HANDOFF`，chunk 4 在停止中取消，chunk 5 因 `right_joint2 0.104619 > 0.05` 被拒绝。command 起点不能当作各 chunk 起点，正常流式交接也不要求 command 级 `COMPLETED`。
- 实际执行使用 inference 专用轨迹保真容差：左臂 `[0.30, 1.30, 0.52, 0.98, 0.55, 0.44, 1.51] rad`，最高为原值 30.2 倍；右臂仅 `right_joint5` 为 `0.10 rad`，其余仍 `0.05 rad`。硬限位、速度/加速度与制动约束未改。原容差下可用性未验证。
- 新模型身份：bundle `7ddcdb62…d70a83`、processor `85f627f5…9895faf2`、model sha `57a4a92e…52fb3c2`、checkpoint smoke/128、Marvin profile `bc815504…7bd0947`。
- 接口审计未发现轴顺序、绝对动作语义或处理器统计错配；短训练预测在 chunk 时间维变化很小，现有证据不足以认定模型直接输出训练均值。
- 6 条诊断数据来自固定布局，只验证软件链路与数据格式，不证明泛化；短训练策略未完成交接。
- evaluation 预算已用尽 3/3；5K 量级效果训练、正式准入与原容差可用性是独立后续计划，本轮不自动启动。

## 交付与下一步

实际验收记录、证据索引、职责修复摘要、有效命令、补验结果与集中限制见 [deliveries/phase-06.md](deliveries/phase-06.md)。任务书、架构说明与[方案 README](../../shared/scenarios/marvin_handover/README.md) 已同步。后续模型效果实验应使用新目录、新预算和明确记录的评价条件。

目录导航：[任务书入口](README.md) · [P05 交付](deliveries/phase-05.md) · [P06 交付](deliveries/phase-06.md)
