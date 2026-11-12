# 阶段 03：方案配方与仿真、运行端接入

状态：待实施。本阶段遵守[总要求](README.md)，只完成本文件范围。

## 前置与读取范围

上一阶段 02 的必要验收通过；读取 `deliveries/phase-02.md` 的接口与限制。

必读总要求、本文件、已有 progress 和上阶段交接；源码按下面入口查找。除明确引用外，不要求阅读其他阶段全文。

## 目标与范围

将当前 Panda 的实例配方集中到方案目录，建立真实 `panda_stack/binding.yaml`，并让 simulation 和 runtime 的准备/执行入口实际消费绑定。学习端 CLI 与并行采集接入分别留阶段 04、05。

## 配方归属

| 当前来源 | 目标 |
| --- | --- |
| simulation/mujoco/configs/backend 中 Panda 配方 | panda_stack/configs/simulation/backend |
| simulation/mujoco/configs/scene、checks 中 Panda 配方 | panda_stack/configs/simulation/scene、checks |
| runtime/ros2/configs 的 teacher_collect、policy_rollout、stack_campaign | panda_stack/configs/runtime |
| learning/lerobot/configs 的真实 Panda dataset/training/policy 配方 | panda_stack/configs/learning |

`container.env`、依赖锁、Dockerfile 保留各端；fake/历史回归配置保留测试归属。每个基线/重复配方按调用证据迁移、合并或保留，不能按文件名批量删除。

本阶段移动学习配方时只修复引用和原入口默认路径，使既有直接配置方式继续可用；新的学习端 --scenario 行为留阶段 04。不得产生阶段间悬空路径。

## 实际入口

- simulation 的 prepare/backend、viewer、场景和运动学检查选择正确 recipe。
- runtime 的 prepare、interactive/policy 选择 teacher/policy recipe、控制约束和档案，继续消费真实 policy_binding.json。
- 复用阶段 02 的解析 API，不复制一套 binding 解析器。私有 YAML 完整语义由端模型校验。
- --scenario 与同类直接 --config/--template 互斥；run-id/seed/output 等允许覆盖必须记录。
- 保留直接配置模式；--help 不要求 SDK、服务和模型已经可用。

核实迁入公共目录的 `runtime_config.schema.json` 调用方：无调用则撤下失效定义；若确需保留文档格式，放在 runtime 私有目录，并与真实 RuntimeRecipe 保持一致。不能保留一份仍要求旧 `script_targets` 的公共重复定义，也不因此改动既有 Episode/日志公共格式。

## 参数与身份

- 方案绑定主要保存选择关系，具体数值仍维护在对应 recipe。不要在 binding 中再次复制 16 维限值、相机尺寸和训练参数。
- 纯移动阶段保持 Profile、observation、action 内容及语义摘要不变。路径变化导致的 recipe 文件摘要变化属于正常的新运行身份，必须解释。
- 任务阶段、接触门禁、质量规则和成功判定配置归方案；实现代码留在各端。
- 控制器通用默认值归实现，当前 Panda/实验数值归方案；不得借分类重新调参。
- `policy_binding.json` 继续由真实模型包校验生成，不能由 scenario binding 替代其内容真实性校验。
- 记录原始模型输出、准入目标、融合目标、参考轨迹与实测反馈的分层行为不变。

新增 prepared 记录 scenario_id、binding 摘要、实际输入和覆盖来源；旧 prepared 不改写。bundle 为空时 collection 可校验，inference 准备缺模型时报错。

Teacher 扩展尚未拆分：本阶段 binding 的 extension_ros 为空、common_ros 选择现有两个包；阶段 06 负责更新实际包选择。不得提前引用不存在的 panda_stack_api_msgs。

## 验收

- 真实 binding 中每个已声明源文件存在，递归依赖完整，无旧 profiles/contracts 路径。
- 从其他 cwd 准备出的有效业务参数相同；除路径、scenario 身份和新运行 ID 外保持基线参数。
- 选中替代测试 recipe 会改变实际准备输入，不只是日志打印；参数冲突有明确错误。
- 配置、准备/入口的相关测试通过；不要求本阶段完成真实闭环或修复 ACT 失败。
- 失效 Runtime Schema 已按调用证据处理，公共目录不再维护漂移的私有配置定义。
- 交接记录新 recipe 路径、参数约定和 manifest 字段，供学习与采集端接入。

## 交付与停止点

- 更新 `progress.md`，写 `deliveries/phase-03.md` 简短交接；基线清单只查询/补充相关行。
- 交接列出实际入口/接口、关键决定、命令与结果、未运行项及原因，状态必须真实。
- 完成本阶段后停止，不自动进入下一阶段。必要验证未完成时保留“待验证”，不得写通过。

目录导航：[总要求与阶段索引](README.md)
