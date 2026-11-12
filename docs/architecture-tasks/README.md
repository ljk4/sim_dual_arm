# 架构任务书

本目录存放架构任务与阶段交接。规划、实现状态和验证结果分别以任务书、进度及交接为准。

## 公共层与方案绑定迁移

先读仓库 [RULES.md](../../RULES.md) 和 [总要求](shared-scenarios/README.md)，再执行本轮阶段。当前状态见 [progress.md](shared-scenarios/progress.md)。

| 阶段 | 文件 | 交付重点 |
| --- | --- | --- |
| 00 | [基线清点](shared-scenarios/phase-00-baseline.md) | 工作树、保护对象与归属 |
| 01 | [目录与路径](shared-scenarios/phase-01-layout.md) | 公共目录、档案与挂载 |
| 02 | [绑定解析](shared-scenarios/phase-02-binding.md) | 数据模型与只读工具 |
| 03 | [仿真与运行端](shared-scenarios/phase-03-runtime-simulation.md) | 配方、binding 与入口消费 |
| 04 | [学习端](shared-scenarios/phase-04-learning.md) | 数据、训练、模型与服务入口 |
| 05 | [采集与并行](shared-scenarios/phase-05-recording.md) | campaign 与隔离 worker |
| 06 | [SDK 与 Teacher](shared-scenarios/phase-06-sdk-teacher.md) | 按包集构建与专用扩展 |
| 07 | [末端反馈](shared-scenarios/phase-07-feedback.md) · [读取收尾](shared-scenarios/phase-07-reading.md) | 有界反馈与现有旧格式读取 |
| 08 | [轨迹来源](shared-scenarios/phase-08-trajectory.md) | 公共表示与求解器信息 |
| 09 | [部署检查](shared-scenarios/phase-09-deployment.md) | 实现要求与公共规则 |
| 10 | [方案模板](shared-scenarios/phase-10-template.md) | 换什么改什么与配置替换 |
| 11 | [代码精简](shared-scenarios/phase-11-simplification.md) | 重复解析、校验、摘要与包装清理 |
| 12 | [集成回归](shared-scenarios/phase-12-regression.md) | 有界真实闭环、并行收尾与历史读取 |
| 13 | [最终收口](shared-scenarios/phase-13-closure.md) | 活跃入口、操作文档与实际交付 |

每轮只读总要求、当前阶段、进度和必要交接，按需查看源码。执行后更新 progress 与对应短交接；详细日志放诊断目录。提示词由聊天提供，不另存文档。

目录导航：[文档索引](../README.md)
