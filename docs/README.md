# 文档索引

本目录维护当前任务、功能说明与实验记录。项目操作见[根 README](../README.md)，工程实施遵守仓库 [RULES.md](../RULES.md)。

## 专题与归档目录

| 目录 | 用途 | 维护规则 |
| --- | --- | --- |
| [features/](features/README.md) | 核心功能设计、跨模块协作流程与接口说明 | 一个功能一篇，与当前代码实现保持同步 |
| [experiments/](experiments/README.md) | 关键实验对比、问题定位与可复用的工程经验 | 经确认后收录，保留实验条件、证据路径与适用边界 |
| [history/](history/README.md) | 历史阶段演进与 DSH 关键结论索引 | 仅保留有决策价值的结论；早期逐阶段原文通过 Git 历史追溯 |
| [marvin-handover/](marvin-handover/README.md) | Marvin 双臂桌面抓取、空中交接与入箱的目标、职责边界及模型检查说明 | 区分已确认目标、初版设计、静态事实与实际运行结果 |

## 阅读建议

1. 首次使用或运行闭环：阅读根目录 [README.md](../README.md)。
2. 了解公共协议、方案绑定与四端实现的归属：阅读 [shared/README.md](../shared/README.md)。
3. 接入 Marvin 并开展交接任务：阅读[目标与边界](marvin-handover/README.md)及[模型检查说明](marvin-handover/model-notes.md)，再按阶段推进。
4. 参与开发或修改配置：阅读仓库 [RULES.md](../RULES.md) 及相关目录的 `README.md`。
5. 排查推理执行问题或查阅历史决策：参阅 [experiments/](experiments/README.md) 与 [history/](history/README.md)。
