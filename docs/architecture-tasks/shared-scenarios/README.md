# 公共层与方案绑定迁移：总要求

仓库：`/home/ljk/work/sim_dual_arm`。实际状态见 [progress.md](progress.md)。每轮只执行指定阶段，完成后停止；提示词在聊天中提供。

## 目标与所有权

公共代码放在 `shared/common/`；任务、机器人、后端和学习实现的选择关系放在 `shared/scenarios/<id>/`。在现有接口支持的能力范围内，增加方案通过新增目录和显式引用完成，公共代码不注册方案名称。

```text
shared/
├── common/
│   ├── interfaces/             # Policy、ROS、JSON Schema
│   ├── python/                 # contracts、trajectory、ros_support、run_logging
│   └── tools/                  # SDK、环境与通用检查
└── scenarios/
    ├── _template/              # 新方案说明与 binding.example.yaml
    └── panda_stack/
        ├── binding.yaml
        ├── profiles/
        ├── configs/            # simulation、runtime、learning 配方
        └── extensions/         # 方案专用接口
```

| 归属 | 职责 |
| --- | --- |
| simulation | 仿真、渲染、执行映射、求解器、任务插件、Teacher 与仿真评价 |
| hardware | 真实设备驱动与能力适配；当前没有可验证的真实驱动 |
| runtime | 会话、控制编排、录制、导出、审计与采集 worker |
| learning | 数据转换、训练、模型包、Policy 服务与算法适配 |
| shared/common | 跨端格式、必要校验、共享数学与通用工具 |
| shared/scenarios | 档案、配方、模型引用、实现选择与专用接口 |

各端的 Dockerfile、依赖锁、环境配置、入口脚本和资产归所属实现。`tests/`、`third_party/`、`.build/`、`var/` 保持原类别。新增算法或驱动在对应端实现；新增公共能力按实际需求演进协议。

## 实施规则

先读仓库 [RULES.md](../../../RULES.md)。它约束实现方式，也约束本任务的验证和交付：

- 为现有用法和本次明确要求实现功能；不添加未来后端注册中心、通用迁移框架、配置继承系统或只负责转发的包装层。
- 不新增仅供留痕的哈希。新增摘要必须替代实质上更贵的操作，并影响后续行为。已有协议明确使用的 Profile、模型、轨迹摘要，以及用于判断 SDK 是否需重建的摘要，按实际消费者维护；重复计算和无人消费的附加记录在代码精简阶段处理。
- 校验放在最能决定行为的边界。公共层解析绑定和公共契约，各端解析私有 recipe、判断自身实现支持范围，learning 验证真实模型；下游复用结果。
- 验证针对仍未解决的问题。改动公共序列化时检查受影响的消费者；文档改动检查链接和命令。一次通过后，只有新改动或新证据才扩大或重复运行。
- 保留现有直接配置入口。方案选择与同类 `--config`/`--template` 互斥；有效配置实际使用所选 recipe 与 bundle。配置路径以所属文件为基准。
- 公共生产包不反向导入四端；contracts、trajectory、binding 与 learning 保持 ROS-free，ROS 适配放在 ros_support。Panda 和 ACT 的规则留在实现侧。
- 档案与控制参数保留权威来源，binding 只选引用。checkpoint 作为模型引用，真实 `policy_binding.json` 由 learning 的 bundle 校验生成。
- 容器使用 `/shared:ro`；simulation 自身为 `/app`，runtime/learning 为 `/workspace`。源配置与产物隔离，不假设宿主存在 `/data`。
- 不兼容接口变化明确拒绝；历史读取只覆盖真实存在且需要保留的格式。不通过旧根路径软链接、转发包或隐藏默认值维持源码入口。
- 遵守 [代码规范](../../standards.md) 的职责与规模要求；不为凑行数机械拆文件，也不新增规模检查工具。

## 保护范围

以当前工作树为基线保留用户修改，不自动 commit、push、stash、reset、clean。旧 MCAP、数据集、模型、prepared、台账和训练划分保持原样，历史记录中的路径不批量替换。

控制限位、跟随预算、相机、接触与质量门禁、任务判据和训练参数不属于本次重构。第三方源码、许可证、Panda 资产和 Ruckig wheel 不改。

需要业务验证时先确认资源可用，使用独立诊断目录、种子和台账，只结束本轮进程。完整训练、正式补采和完整评价集不在本任务范围。

## 读取与交付

每轮读取 RULES、本文件、本阶段、progress 和上一阶段交接。随后只读相关源码、规范章节与明确需要的早期接口记录，不通读所有阶段或历史日志。无关缺口记入已知问题，继续可以独立完成的工作。

`progress.md` 是短索引；`deliveries/phase-NN.md` 记录当前实现、关键入口、验证命令与结果。注意事项集中在“已知问题”一节。日志可放 `.build/`，交接不堆叠完整日志、摘要清单、评分表或重复验收报告。`baseline-files.tsv` 仅按需查询。

完成表示本阶段要求已经交付；缺少必要验证时标为“待验证”。静态检查、类型往返、真实服务调用和物理任务结果按实际证据报告。已有通过结果可引用，不能把历史容器状态或旧测试结果写成当前实测。

## 阶段索引

| 阶段 | 任务书 |
| --- | --- |
| 00 | [基线清点](phase-00-baseline.md) |
| 01 | [目录与路径](phase-01-layout.md) |
| 02 | [绑定解析](phase-02-binding.md) |
| 03 | [仿真与运行端](phase-03-runtime-simulation.md) |
| 04 | [学习端](phase-04-learning.md) |
| 05 | [采集与并行](phase-05-recording.md) |
| 06 | [SDK 与 Teacher](phase-06-sdk-teacher.md) |
| 07 | [末端反馈](phase-07-feedback.md)；[历史读取收尾](phase-07-reading.md)已完成 |
| 08 | [轨迹表示与求解器来源](phase-08-trajectory.md) |
| 09 | [部署检查](phase-09-deployment.md) |
| 10 | [方案模板](phase-10-template.md) |
| 11 | [代码精简](phase-11-simplification.md) |
| 12 | [有界集成回归](phase-12-regression.md) |
| 13 | [文档与最终收口](phase-13-closure.md) |

当前执行顺序：08 → 09 → 10 → 11 → 12 → 13。已完成阶段不重做。

目录导航：[架构任务索引](../README.md) · [文档索引](../../README.md)
