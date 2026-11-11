# 交付其他模型的实现提示词

配套规范：[公共层与方案绑定迁移任务书](shared-scenarios-taskbook.md)。提示词中的任务书是唯一目标说明，实施记录描述实际完成状态。

建议保留同一工作树和实施记录逐阶段交付。下面同时提供完整实施、单阶段实施和独立验收三种提示词，选择一种复制即可；完整实施不要求每阶段再次向用户确认。

## 1. 完整实施提示词

```text
你在 /home/ljk/work/sim_dual_arm 仓库工作。请实际实施公共层与方案绑定迁移，而不是只给建议或再写一份计划。

先完整阅读：
1. docs/architecture-tasks/shared-scenarios-taskbook.md
2. docs/standards.md
3. docs/README.md，以及实际受影响目录的 README 和适用的 AGENTS.md
4. 如果已经存在，docs/architecture-tasks/shared-scenarios-implementation.md

目标是把 contracts、libraries、infrastructure、profiles 收拢到 shared/，分为 common 和 scenarios；交付真实 panda_stack 方案、_template 示例、轻量严格的 binding 解析与四端实际接入。目录结构、绑定字段、有限的接口整理、阶段验收以任务书为准。

重要判断：
- 公共层定义共同语义，不依赖四端具体实现；不追求适配一切未来能力。
- 方案目录保存选择关系、配置与必要专用接口；物理、驱动、任务、训练和运行业务代码留在四端。
- 换已接入能力的方案只需新增/修改绑定；新增后端或驱动仍需在所属端实现。
- 模型 checkpoint 是引用，不为每个 checkpoint 新建方案；模型真实性仍使用既有 bundle 校验。
- 合并目录不合并 Python 包职责，也不创建泛化 utils 或自动编排平台。

开始前用 git status 和实际源码清点当前状态。这里已有大量未提交与未跟踪改动，工作树而非 HEAD 是你的基线。保留已有修改、删除和未跟踪文件，不自动 commit、push、stash、reset、clean，不凭旧文档恢复文件。

严格按 S0→S1→S2→S3→S4→S5 顺序工作：
S0：逐文件归属与路径调用者清点，记录现有失败、资源状态和历史读取基线。
S1：纯迁移公共目录、档案、路径与挂载；保持协议载荷和物理参数不变，先验证再继续。
S2：实现 binding 模型/校验工具，迁移实例 recipes，实际接通 prepare、record/record_parallel、learning 与推理入口。
S3：按任务书整理可选 SDK、Panda Teacher 扩展、末端反馈容器、曲线表示与来源、部署声明。不得把物理接触规则一并泛化或删除严格校验。
S4：交付 _template 的操作矩阵和示例；用临时新增方案及合适的测试替身验证公共代码无需修改。
S5：执行最小有界集成回归、历史读取验证，更新活跃文档、删除旧路径入口并完成交付记录。

每阶段完成相关验证再进入下一阶段，不需要为每个可逆操作暂停征求确认。协议改动必须与纯移动分开；阶段失败先定位，不以修改测试断言或放宽业务门禁获得通过。

保护要求：
- 不修改或搬走 var/ 的旧 MCAP、模型、prepared、台账和训练/验证数据。
- 不重写旧摘要，不手改旧 prepared 路径，不删除失败记录，不挪用正式或评价预算。
- 不调整控制限位、跟随预算、相机、成功判据、ACT 参数和夹爪准入参数。
- 不修改第三方源码和许可证、Panda 模型、固定 Ruckig wheel。
- 不修复与此次迁移无关的推理失败，不运行完整训练或补采正式数据集。
- 清理只针对自己启动的资源。先检查已有容器/进程及挂载，不停掉其他正在使用的业务。
- 不自动创建真机驱动或宣称真机验证通过。

实现注意事项：
- 保持 robot_contracts、robot_trajectory、robot_ros_support、robot_run_logging 的包名，删除旧路径依赖，不做软链接或转发层。
- 容器统一使用 SHARED_ROOT=/shared 的只读公共挂载；setup、SDK 与 source paths 显式使用正确根，不能沿用错误 parents[n]。
- binding 只读校验不启动四端、不导入业务实现、不预留种子；相对路径不依赖 cwd；禁止隐式 Panda 默认值。
- --scenario 与同类 --config/--template 冲突时报错；允许的运行覆盖要进入有效配置和 manifest。
- 核心 SDK 不强制 sim 或 Panda 包；选中的扩展包必须参与 SDK 完整性、模块来源和摘要校验。
- 迁移后的普通控制不能要求安装 Panda Teacher；模型输入输出、时间与单位校验仍严格。
- README 变动不应使 SDK 过期，真实 IDL 或包集变动必须失效。
- 根 tests 和 third_party 可保留。当前 learner 专用 fake 不应冒充独立协议替身。
- 遍历 Python、shell、YAML、JSON、IDL、Proto、CMake、package.xml、Compose、README 里的路径，尤其检查并行 worker 和 campaign.required_paths。

验证按任务书第 11 节执行。优先复用相关测试，新测试集中在真实边界与故障路径。区分静态检查、单元、SDK、集成、物理任务成功；一次 Fake 或配置样例不是新后端证明。存在可用真实模型时做一次有界闭环，物理任务失败与软件链路结果分别报告，不使用 Teacher 或真值救场。

持续维护 docs/architecture-tasks/shared-scenarios-implementation.md，记录：
基线和保护对象、逐文件迁移表、已完成阶段、兼容性/协议变化、具体命令与结果、未运行项目及原因、最终新入口和剩余限制。

环境缺失时继续可独立完成的工作，给出尚未验证的具体项和可复现命令，不假称全部通过。遇到必须改变保护对象的真实冲突时说明具体证据并请求对应决策，其余工作继续推进。

最终回复只需列出：实际完成阶段、主要入口与文档链接、验证结果、未完成项。不自动提交或推送。
```

## 2. 按阶段交付提示词

适合希望每次只做一小步的实施方式。首次建议选择 S0，确认清点结果后再做 S1；后续可分别交付 S2、S3、S4 和 S5。不要在不同工作树间遗漏未提交改动。

复制下面提示词，并填写阶段范围。S0 是分析与记录；S1 及之后要求实际修改目标代码。

```text
请在 /home/ljk/work/sim_dual_arm 继续实施公共层与方案绑定迁移。

本轮授权阶段范围：[填写 S0 / S1 / S2 / S3 / S4 / S5，或相邻范围]。
严格只完成这个范围，不擅自进入后续阶段；范围内的工作、修复和验证应持续完成，不停在建议或计划。

完整阅读 docs/architecture-tasks/shared-scenarios-taskbook.md、现有 implementation 记录、docs/standards.md 和适用 AGENTS.md。任务书是目标，工作树源码是现状；不要把文档中的目标路径当成已经存在的功能。

从 git status 和现有工作树接续，不重复或撤销前一阶段成果。已有大量用户修改，不自动 commit/push/stash/reset/clean，不改旧数据、模型、台账、prepared、实验参数或第三方资产。

若前置阶段缺少必要交付，先核实并明确前置缺口；不得通过跳过前置校验强行进入本轮协议或行为修改。可继续当前范围内不依赖缺口的工作。

完成任务书中本阶段的具体交付与阶段验证。业务检查在相应容器进行；只清理本轮资源；没有环境就如实标为未验证，不能降低门禁。

更新 docs/architecture-tasks/shared-scenarios-implementation.md，记录本轮实际差异、阶段状态、命令和结果、未完成项。只在真实完成后标记阶段通过，不把阶段交付称为整个迁移完成。

最终给出本阶段交付文件、验证结果和下一阶段仍需处理的工作；不自动开始下一阶段。
```

## 3. 独立验收提示词

```text
请只读验收 /home/ljk/work/sim_dual_arm 的公共层与方案绑定迁移，不编辑源码，不自动提交、修复、启动业务或重建容器。

阅读 docs/architecture-tasks/shared-scenarios-taskbook.md 和 shared-scenarios-implementation.md，核对实际工作树与报告的完成阶段。已有大量与本轮无关的用户改动，不把所有 git diff 都归因于迁移；迁移边界不明时明确指出证据不足。

重点核对：
1. shared/common 与 scenarios 的归属是否真实，是否只是移动了目录。
2. binding 是否被各端及并行采集实际消费；从其他 cwd 是否仍能正确解析。
3. common 是否存在 Panda/stack/ACT/MuJoCo 的业务分支或反向导入。
4. 方案配置是否单一来源，模型绑定和运行摘要是否由真实内容产生。
5. 核心 SDK 是否能不依赖仿真/Panda 扩展，扩展包是否受来源与过期检查。
6. 固定 Teacher、末端反馈、曲线来源及部署规则是否按任务书处理，严格语义是否保留。
7. 旧数据、模型、台账、prepared 是否原样保留；历史读取与协议版本是否有证据。
8. 旧路径、脚本根目录、Compose、worker、campaign 和活跃文档是否遗漏。
9. 模板是否明确换任务、机器人、后端、算法、checkpoint、新能力时分别改什么。
10. 验证证据是否区分静态、单元、SDK、集成、物理成功与泛化。

可执行纯读取的路径、AST、配置和文档一致性检查；需要产生文件、占用 GPU、启动进程或写台账的验证只报告所需命令，不自行执行。

按影响程度列出可操作问题，给出文件位置、触发条件、影响及修复方向。对没有证据的结论使用“未验证”，不要推断通过。没有问题则明确已核对范围与剩余验证限制。
```

目录导航：[架构任务索引](README.md) · [任务书](shared-scenarios-taskbook.md)
