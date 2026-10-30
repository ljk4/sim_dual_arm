# 文档索引

当前设计材料：

| 文档 | 内容 |
| --- | --- |
| [阶段 1～2 可维护性审查](maintainability-review.md) | 已复现问题、目录与职责评估、阶段 2R 整改顺序；源码修复尚未执行 |
| [阶段 1～2 使用与验证](stage2-quickstart.md) | 实际构建/启动命令、MCAP 输出、测试与维护规则 |
| [逐步实现路线图](implementation-roadmap.md) | 推荐实现顺序、每阶段产物、完成标准和首批工作 |
| [目录划分与组织规范](repository-layout.md) | 仓库分类、配置层次、构建输出、数据产物与扩展规则 |
| [环境边界与接口设计](architecture-and-interfaces.md) | 四类环境职责、Robot/Policy/Runtime API、时钟与控制权 |
| [环境与依赖分析](docker-dependency-analysis.md) | 镜像依赖归属、候选技术栈和运行组合 |
| [仿真环境设计](simulation-design.md) | 无真机数据与推理闭环、场景、渲染与验收计划 |

唯一维护的协议与部署草案：

- [Policy API v1](../contracts/policy/v1/policy.proto)
- [仿真采集部署绑定示例](../deploy/examples/sim_collect.yaml)

历史材料：archive 中的早期 Humble 方案，以及 dependencies 中的 Python 3.10 解析快照。它们保留用于追溯，不能当作当前安装基线。docs/contracts 的旧文件路径仅保留迁移提示。
