# 文档索引

本目录维护当前任务、功能说明与实验记录。项目操作见[根 README](../README.md)，工程实施与闭环开发流程遵守仓库 [AGENTS.md](../AGENTS.md)。

## 专题与归档目录

| 目录 | 用途 | 维护规则 |
| --- | --- | --- |
| [features/](features/README.md) | 核心功能设计、跨模块协作流程与扩展演进说明 | 一个主题一篇，与当前代码实现保持同步 |
| [experiments/](experiments/README.md) | 关键实验对比、问题定位与可复用的工程经验 | 经确认后收录，保留实验条件、证据路径与适用边界 |
| [colab/](colab/train_act_colab.ipynb) | Google Colab 无 Docker 学习端环境配置脚本、大内存数据转换与 ACT 训练 Notebook | 与 [learning/lerobot/README.md](../learning/lerobot/README.md) 环境约定保持同步 |

## 阅读建议

1. 首次使用或运行闭环：阅读根目录 [README.md](../README.md)。
2. 查阅全部脚本参数与示例：阅读 [脚本入口与使用指南](features/脚本入口与使用指南.md)。
3. 了解各端职责分工、通信协作契约、公共协议及待解耦边界：阅读 [跨端架构分工与解耦审查](features/跨端架构分工与解耦审查.md) 与 [shared/README.md](../shared/README.md)。
4. 扩展真机、新仿真器（Isaac Sim）、RL 或开展 Real2Sim/Sim2Real：阅读 [架构扩展与演进指南](features/架构扩展与演进指南.md)、[天机 Real2Sim 标定与实施方案](features/天机Real2Sim标定与实施方案.md) 与 [强化学习架构与 Real2Sim 协同设计](features/强化学习架构与Real2Sim协同设计.md)。
5. 参与开发或修改配置：阅读仓库 [AGENTS.md](../AGENTS.md) 及相关目录的 `README.md`。
6. 排查推理执行问题：参阅 [experiments/](experiments/README.md)。
