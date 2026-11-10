# 文档索引

本目录维护项目的总体状态、工程规范与专题文档：

- **[项目状态](project-status.md)**：总体目标、技术基线、已完成里程碑、产物路径与下一步计划。
- **[项目规范](standards.md)**：目录组织、架构边界、接口契约、代码质量、配置、日志、容器与验收标准。

## 专题与归档目录

| 目录 | 用途 | 维护规则 |
| --- | --- | --- |
| [features/](features/README.md) | 核心功能设计、跨模块协作流程与接口说明 | 一个功能一篇，与当前代码实现保持同步 |
| [experiments/](experiments/README.md) | 关键实验对比、问题定位与可复用的工程经验 | 经确认后收录，保留实验条件、证据路径与适用边界 |
| [history/](history/README.md) | 历史阶段演进与 DSH 关键结论索引 | 仅保留有决策价值的结论；早期逐阶段原文通过 Git 历史追溯 |
| [refactoring-tasks/](refactoring-tasks/README.md) | 六阶段架构瘦身、解耦与性能优化重构任务书（已完成） | 归档已完成的架构重构方案与量化验收标准 |

## 阅读建议

1. 首次使用或运行闭环：阅读根目录 [README.md](../README.md)。
2. 了解当前进展与系统边界：阅读 [project-status.md](project-status.md)。
3. 参与开发或修改配置：阅读 [standards.md](standards.md) 及各子目录的 `README.md`。
4. 排查推理执行问题或查阅历史决策：参阅 [experiments/](experiments/README.md) 与 [history/](history/README.md)。
