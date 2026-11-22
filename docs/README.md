# 文档索引

本目录维护当前任务、功能说明与实验记录。项目操作见[根 README](../README.md)，工程实施与闭环开发流程遵守仓库 [AGENTS.md](../AGENTS.md)。

## 专题与归档目录

| 目录 | 用途 | 维护规则 |
| --- | --- | --- |
| [features/](features/README.md) | 核心功能设计、跨模块协作流程与接口说明 | 一个功能一篇，与当前代码实现保持同步 |
| [experiments/](experiments/README.md) | 关键实验对比、问题定位与可复用的工程经验 | 经确认后收录，保留实验条件、证据路径与适用边界 |
| [colab/](colab/train_act_colab.ipynb) | Google Colab 无 Docker 学习端环境配置脚本与训练 Notebook | 与 [learning/lerobot/README.md](../learning/lerobot/README.md) 环境约定保持同步 |

## 阅读建议

1. 首次使用或运行闭环：阅读根目录 [README.md](../README.md)。
2. 查阅全部脚本参数与示例：阅读 [脚本入口与使用指南](features/脚本入口与使用指南.md)。
3. 了解各端职责分工、通信协作契约、公共协议及待解耦边界：阅读 [跨端架构分工与解耦审查](features/跨端架构分工与解耦审查.md) 与 [shared/README.md](../shared/README.md)。
4. 参与开发或修改配置：阅读仓库 [AGENTS.md](../AGENTS.md) 及相关目录的 `README.md`。
5. 排查推理执行问题：参阅 [experiments/](experiments/README.md)。
