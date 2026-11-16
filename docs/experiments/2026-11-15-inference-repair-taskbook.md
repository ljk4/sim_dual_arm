# 双任务推理修复任务书

日期：2026-11-15。状态：P1/P2/P3-B 已验证，P3-C 已补修；P3-D 原 anchor 拒绝结论保留，STOPPING 实测行的决策解释已撤回；P3-E 有运动学反事实，P3-F 已冻结但R2暂缓实施；按用户最新优先级改走数据抽查→物理回放→模型拟合→真实推理对照（§14）。P3 总体及 P4–P7 尚未完成；没有将原型、阶段推进或草案视为完整任务成功。

仓库：`/home/ljk/work/sim_dual_arm`。最高实施规则：[RULES.md](../../RULES.md)。

## 1. 目标与依据

先使评价结果可信，再修复模型输出与执行链的适配、示教时间分布和训练选模问题，最后用固定模型、固定配置的闭环试验判断效果。清理无消费者的代码、哈希与产物，但不破坏仍有用途的协议、缓存和故障证据。

本任务书的直接依据：

- [独立失败归因与互证报告](../../var/_system/diagnostics/2026-11-15-inference-failure-analysis.md)
- [代码与产物审计，含 B01–B08 编号](../../var/_system/diagnostics/2026-11-15-code-and-artifact-audit.md)
- [原始输出、逐局配置与全部示教统计](../../var/_system/diagnostics/inference_failure_evidence.json)
- [240 次录制中的物理进展](../../var/_system/diagnostics/recorded_physical_progress.json)
- [5 次新增诊断闭环](../../var/_system/diagnostics/closed_loop_comparisons.json)

这些 `var/` 文件是本机证据，不随源码分发；不存在时应说明无法执行哪项历史复核，不能伪造结果或添加旧产物回退机制。

已知起点：原 Panda 历史汇总 2/120 成功，Marvin 0/120；各含多个模型和执行配置，不能作为单一模型成功率。新增诊断显示 Panda 改变融合/窗口可解除代表性停滞，但随后仍翻转或释放不足；Marvin 小幅边界投影解除第三次预测拒绝后，仍被夹爪 Follower 偏差拒绝。当前没有“一项参数即可闭环”的证据。

## 2. 实施约束与复原

1. **不执行 Git 写操作**：不 add、commit、reset、checkout、stash、clean、创建分支或 worktree；只读检查使用 `GIT_OPTIONAL_LOCKS=0`。开始时记录用户已有修改，禁止覆盖。
2. 改文件前按原相对路径保存在 `/tmp/sim_dual_arm-repair-<阶段>-<唯一运行标识>/original/`；新建文件单独列出。复原时只恢复本阶段确实改过的文件，不能覆盖用户后续修改。无需新增源码哈希或备份框架。
3. 每阶段形成可审查的正确状态；通过后保留修复供下一阶段使用，**不自动撤销已经通过的修复**。用于消融的临时配置和进程内补丁在试验后恢复。需要撤回时提供精确文件列表和恢复方法。
4. 第一阶段仅允许代码、相关测试及必要文档变更；不训练、不补采、不修改权重和现有正式数据，不运行新的批量推理。
5. 后续阶段若需要新训练或正式数据，先明确数据版本、步数、模型初始化、种子和评价计划；本任务书不授权静默增加训练/采集预算。
6. 保持原关节硬限位、碰撞/掉落守卫、跟随预算及任务成功阈值。Panda 的 upright 要求不能改为立方体任一面朝上；Marvin 必须证明真实空中交接顺序。
7. 通用层不得按任务名选择常量，不增 legacy 分支、纯转发类、无消费者摘要或第二份可编辑参数。跨端接口变更按 RULES §4 先给出具体字段、生产者/消费者和测试方案，再确认实施。
8. 运行日志和正式录制依既有 `logs/` 归属；离线复核、单测输出和诊断实验归 `diagnostics/`；临时测试视图与编译缓存放 `/tmp`。新增产物类型仍按 RULES §3.1 说明消费者与归属。
9. 不删除或改写原 240 次结果、数据集、Bundle 和本轮调查证据。历史事实、修复后的重新计算、新闭环实验分开报告。

## 3. 阶段顺序与验收

每阶段更新本任务书的状态和实际验证结论；缺少证据写“未验证”，不能记为通过。单测、录制重算和真实闭环属于三种不同证据。

| 阶段 | 目标与范围 | 验收结果 | 后续依赖 |
| --- | --- | --- | --- |
| P1 | 修复离线评分硬编码稳定时长 B01、Stack 坐标投影 B02 | 已完成：1 秒合法事实离线通过，负例仍拒绝；XY 旋转投影按局部系计算，原 240 局复核无新增成功 | 为 P2–P7 提供正确的基础判断 |
| P2 | 修复交接过程验证与接触证据 B03/B04/B05，修正 episode 日志关联 B07 | 已完成（2026-11-15）：真实 MjModel 接触采样贯通到 observation/评价；inactive gap 不计接触、箱壁/桌面不冒充箱底；Policy 与 Teacher 均要求实测左独持→空中共持→释放后右独持，直接入箱、缺段、乱序、桌面共持、未释放等反例被拒绝。详见文末「P2 实施结果」 | P1 后开展；在正式评价前必须完成 |
| P3 | 进行中，P3-F 离线冻结已完成 | P3-C 已补修；P3-D 原 anchor 拒绝证明保留、STOPPING 行的决策解释撤回；P3-E/P3-F 有双门运动学候选，R2-F1 已获确认并冻结（§13），生产尚未实施。R2暂缓，先执行§14的数据优先排查；P3总体未通过 |
| P4 | 修复 Teacher 的限位余量、阶段停顿与采集时序一致性 | 新示教诊断在原物理判据下通过；动作时间线、速度/加速度、边界余量有实测证据 | 依赖 P2；和 P3 使用独立试验编号 |
| P5 | 修复训练优化与选模指标 | 分组学习率生效；活动轴物理误差和阶段进度可用；固定变量的训练对照可复现 | 数据版本与 P3 执行配置冻结后再训练 |
| P6 | 简化环境、无用摘要、兼容分支、死代码、包装与产物保留 | 每个删除/保留项都有消费者依据；不破坏缓存、契约和重放；普通启动不隐式更新学习依赖 | 与算法实验分批变更；清理前保留仍需的证据 |
| P7 | 固定部署方案的最终闭环评估 | 分任务、模型、配置和布局报告实际成功/失败及置信边界；成功有完整物理证据 | P1–P6 中影响执行/评分的改动完成后 |

环境修复有独立边界：若 `latest` 或测试挂载问题妨碍当前验证，可以先单独处理 P6 的最小环境启动子项并验收；不得将其混写成 P1 评价器修复，也不得静默拉取新学习镜像。

### P1：评分时长与几何投影

**P1-A：B01。** 检查 `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/evaluation_metrics.py::_apply_task` 及调用链，删除把所有任务都要求为 2 秒稳定的逻辑。推荐最小方案：稳定窗口由已有任务评价器按其配置执行，离线汇总消费冻结的 `SUCCEEDED` 事实及现有完整性约束，不再自行维护另一套时长阈值。

- 不把 2.0 换成 1.0；不按 `task_type`、scenario 名分支；不从当前可编辑 YAML 反推旧录包当时的要求。
- 不新增 ROS 消息、schema 字段或快照副本；不为重复离线计算把任务配置读取扩散到通用层。
- 保留正式 Policy 来源、非 test-only、任务确实被评价、终态/条件一致、有效时间关系与配置/计划身份等现有检查。只保留 `result.outcome=succeeded` 字符串不算修复。
- 持续时间不足的正/负边界测试放在实际任务评价器；离线端验证完整成功事实，不把 `RUNNING + conditions_passed=True` 当成功。

**P1-B：B02。** 修正 `simulation/mujoco/src/robot_backend_sim/tasks/stack/evaluator.py::_support_xy_error` 的世界系到目标局部系投影，用 `R.T @ delta`，并保留现有支撑区域、margin 和独立倾角条件。

- 不把旋转物体当轴对齐物体；不替换为圆形半径判定；不改变源块/目标块尺寸、支撑 margin 或倾角阈值。
- 最小反例：目标 yaw=10°、目标局部偏移 `(0.027, 0.027, 0.05)`、半尺寸 0.03、margin 0.002；正确 XY over-edge 均为 -0.001 m。旧计算有一轴约 +0.006606 m。
- 用正负 yaw、零旋转及真实越界点检验坐标变换，不能只测试 helper 内部使用了“转置”这个实现细节。

**测试与历史复核：**

| 验证 | 必须保留的判断 |
| --- | --- |
| 合法 1 秒稳定的已完成交接事实 | 离线 task_success 为 true，无任务名特判 |
| Panda 原配置稳定窗口 | 在线评价在达到原窗口前保持 RUNNING，达到后才 SUCCEEDED |
| Marvin 原配置稳定窗口 | 在线评价在达到原窗口前保持 RUNNING，达到后才 SUCCEEDED |
| 只有成功字符串、缺少冻结物理事实 | false |
| RUNNING/FAILED、conditions_passed=false、teacher、test-only 等现有拒绝 | 仍拒绝，保持现有独立维度语义 |
| XY 旋转边界内/外，正负 yaw 和 identity | 几何结果正确；90° 翻转仍不能通过独立 upright 条件 |
| 两个原 Panda 成功与一个旋转项变化局 | 只读复核；不能用修正一个条件抹掉其余失败 |

优先扩展已有测试：`tests/unit/test_stack_evaluation.py`、`test_stack_task.py`、`test_handover_task.py`、`test_task_result_sidecar.py`。相关测试原有失败先记录再区分来源；不靠 importorskip、删测试或放宽断言获得“通过”。

历史代表局：

- `closure120-panda-b16-ep-0002`、`closure120-panda-b23-ep-0001`：原成功证据保持可解释。
- `closure120-panda-b14-ep-0003`：修正后 XY 项从失败变通过，但其余失败条件仍应保留，不能宣称任务成功。
- 原 Marvin 120 局均未完成任务；P1 不应把它们重标为成功。

P1 不实施 B03–B08，不改变接触计数、交接状态机、Follower/准入/融合、训练配置、容器配置或原始证据。直接执行提示词见[第一阶段提示词](2026-11-15-stage1-repair-prompt.md)。

### P2：接触证据与交接顺序

执行提示词见[第二阶段提示词](2026-11-15-stage2-repair-prompt.md)。P2 开始前重新确认 P1 的实际补丁；P1 新增的“最终放置后稳定 1 秒”测试需先建立有效交接前缀，原“空步骤计划只要入箱即可成功”测试改为拒绝负例，不能给生产代码增加绕过交接验证的开关。

- 任务内根据实测双指接触力、物块离桌/外部支撑和时间窗判定左手独持→双手共持→左手释放后右手独持；不依赖 Teacher 的名义阶段，不靠最终物块位姿反推。
- 使用源配置中已有门槛，异常抖动、重复同时间戳不得积累持稳时间；复位清空顺序状态。能在当前 `metrics_json` 内表达时不扩展 wire schema，不引入通用“工作流引擎”。
- 正例及负例覆盖缺少某一阶段、顺序错误、外部支撑、共持不足、丢失持物与跨 episode 状态污染；最终入箱但没交接必须拒绝。
- 明确 inactive 接触、零力接触及真实受力接触的各项语义；只修依赖该证据的判断，不批量改变所有碰撞守卫。
- 箱底支撑用真实 geom/法向区分底板与侧壁，并覆盖侧壁夹住、桌面支撑和真实落底正例。B07 以真实 episode 上下文修正，不能用字符串截断猜 episode。

### P3：模型输出与执行适配

P3 整体较难，先拆为 P3-A 离线定位、P3-B 输出准入和 P3-C 夹爪重新定时/阶段推进。P3-A 已完成，P3-B 已实施（§9）；P3-C 已有离线与部分闭环证据，Teacher 范围隔离与测试时间轴已按 §10.9 补修，补修提示词见 [P3-C 补修提示词](2026-11-15-stage3c-followup-prompt.md)。独立补修复核见 §10.10。P3-D 的原拒绝重放与 P3-E 运动学候选见 §11–§12；P3-F 已获确认并完成离线冻结（§13），R2实施暂缓，当前按§14先排除数据/模型/执行的容易排除项。**P3 仍未完成完整闭环，P3 总体仍未通过。** P3-B 改变此前臂关节不裁剪的部署约束；P3-C 则显式改变触发时的 Policy 执行目标，不能混淆两者。

- 先重放保存的真实输出及 `ActionWindow.selected_indices`，核实准备执行的点、后续尾部、融合输出、实际参考与拒绝轴。不能固定分析 `0..4` 或 `0..19`。
- 准入策略先比较小幅有界投影与仅对实际执行窗口准入等候选的契约影响，选择一个实现；不同时铺设多套生产分支。跨端含义发生改变时按 RULES 确认具体方案。
- 任何方案都保留最终执行硬限位和大幅异常拒绝；修正额度来自方案配置/证据，不硬编码到通用机器人实现。保留必要的原始与实际执行值，拒绝后不得悄悄复用旧动作。
- 处理夹爪动作过渡的可实现性、继承状态和跟随偏差；不能简单放大阈值来消掉新暴露的拒绝。
- 用固定 Bundle、匹配种子比较原配置和一个干预；Panda 至少包含已知停滞种子 20013、夹住后停滞样本对应种子及原成功种子。Marvin 固定布局的多个 seed 只算重复试验。
- 单一成功样本只证明一局，阶段推进只记阶段推进；新运行独立于原 240 次计数。

### P4：示教与时间分布

- 对 IK 设置有任务可行性证据的限位余量并验证碰撞、精度与计划成功，不能先假定一个固定 0.05 rad 对所有关节都可行。
- 统一正式采集执行节奏，处理阶段终点等待和动作 chunk 内的停顿。保留抓持/交接所需真实稳定窗口，不为了减少静止帧跳过物理门控。
- 若重新定时，重建一致的 20 Hz 观测/动作时间线，检查速度、加速度、action lead、图像对齐；不能删帧后仍冒用原时间戳。
- 单帧歧义可以由示教设计或真正需要的时序输入解决，先做最小对照；历史输入/模型结构变化另列契约范围。
- 如需新正式数据，先明确是否替换原 50 条、训练/验证拆分和来源；不得静默混合新旧策略。先验收诊断 Teacher 轨迹再形成正式集。

### P5：训练与选模

- 沿上游优化器参数分组配置骨干/其他参数学习率，验证覆盖所有可训练参数且没有重复；不把未知配置默默忽略。
- 移除恒等调度包装，第一组对照保持其他条件不变；如试衰减，作为另一组实验，不同时更换损失、学习率和数据。
- 输出物理单位的逐轴/活动组 MAE、静止组损失占比、阶段进度与闭环表现；组定义来自方案/档案，不在通用层硬编码右臂索引。
- 对低方差轴提出有单位依据的归一化/权重策略；不统一套用 `std>=0.01`，不凭验证 L1 较低断言闭环提升。
- 固定数据版本、随机种子、更新步数、初始化链和部署配置后做受控对照；训练与补采必须在明确范围内执行。

### P6：复杂度、环境与保留策略

- B06：删除无消费者的空源码摘要及仅为它存在的扫描；不要换成另一个“更全面的全仓哈希”系统。沿用已有生效配置来源。
- B08：固定可复现的基础来源，明确启动已有环境与构建环境的区别；普通启动复用本地镜像，不隐式拉取 latest。所有宿主机入口仍归 env.sh，不对单端 down --remove-orphans。
- 删除确定无生产消费者的旧 ObservationBuffer 和 mock 采集工具及其旧测试；拆除确认无用的纯转发函数，调用方同步更新。
- 移除生产链 legacy digest 兼容；仍需读取的历史取证采用边界明确的一次性离线工具，不能扩大为长期兼容框架。
- 任务过滤轴组/阶段从通用学习层移到方案；协议常量与机器人维度各有唯一来源。默认关闭的过滤功能先查消费者，不能直接判死代码。
- 保留有用的 Predict 缓存、Profile/Bundle/SDK/Follower 跨进程契约；是否精简按消费者和实测重复成本决定。
- 先列保留/候选删除路径、用途与大小，提供无删除的预览；大体积删除单独确认具体清单。保留失败取证、必要成功正例和唯一正式产物；不清空 var、不覆盖原报告，不引入另一套归档平台。

### P7：最终评估

- 预先冻结每任务的模型、配方、种子列表、布局分布与实际尝试数，禁止边试边换配置再报成同一个成功率。
- 用现有数据、既有产物类型记录；每个名义成功都复核完整持续窗口、真实支撑、释放/撤退或交接顺序。
- 单测通过、离线误差降低、越过首个阻断和闭环成功分别报告。runtime exit code=0、录制完整、最终位置接近均不能替代任务成功。
- 报告全部尝试和失败，不丢弃“工程错误”样本后只算好局；需要细分可另列，但保留总数。Marvin 布局未随机化时不宣称布局泛化。
- 试验次数和总体通过门槛在该阶段开始前明确；本任务书不把再次运行 120 次或“至少一局成功”当作默认质量门槛。

## 4. 每阶段交付格式

只维护一份阶段摘要；先写结果，再给证据：

1. 修复的问题、受影响入口及行为变化。
2. 实际修改文件与必要调用方；哪些假说被推翻或仍未验证。
3. 验证命令、环境、执行数量、通过/失败/跳过及失败来源。实际没跑的检查必须标明。
4. 若运行闭环，列模型/种子/生效配置、实际物理结果及首个失败原因，不只列退出码。
5. 原有修改保护情况、精确备份位置、新文件列表与恢复方法；容器与后台进程恢复情况。
6. 是否达到本阶段门槛及下一阶段所需输入。没有用户下一步指令时，不自动扩大阶段范围。

P1 的结果直接补充本文末尾；详细单测/离线结果沿用各场景 diagnostics；跨场景摘要归 `var/_system/diagnostics/`。不要再生成无人消费的总清单、全仓摘要或证明“跑过”的证据包。

## 5. 执行状态

| 阶段 | 状态 | 结论与证据 |
| --- | --- | --- |
| P1 | 已完成 | B01/B02 已修复并通过最小复现、相关回归与只读历史复核；未新增任务成功。详见文末「P1 实施结果」 |
| P2 | 已完成 | 已补修候选重抓参考生命周期及两处新增测试跳过；新增 8 项回归通过，simulation 104 passed、runtime 48 passed，详见 §7.8；完整闭环仍未验证 |
| P3 | 进行中，P3-F 离线冻结已完成 | P3-C 已补修；P3-D 原 anchor 拒绝证明保留、STOPPING 行的决策解释撤回；P3-E/P3-F 有双门运动学候选，R2-F1 已获确认并冻结（§13），生产尚未实施。R2暂缓，先执行§14的数据优先排查；P3总体未通过 |
| P4 | 待执行 | 先明确数据调整范围 |
| P5 | 待执行 | 先冻结数据/执行条件和训练范围 |
| P6 | 待执行 | 按独立子项实施，环境阻塞可先处理最小子项 |
| P7 | 待执行 | 待部署配置冻结 |

## 6. P1 实施结果（2026-11-15）

### 6.1 修复与行为变化

- **B01（离线稳定时长）**：`evaluation_metrics.py::_apply_task` 不再把 `stable_elapsed_s` 与全局硬编码 `2.0` 比较。离线汇总只消费冻结事实：`evaluated=true`、`test_only=false`、`action_source=policy`、结果/原因/状态均为成功、`conditions_passed=true`、`start>0`、`completed>=start`、`sample>=completed`、`stable_elapsed_s` 为正数、任务配置与计划摘要非空。任务评价器继续按各自配置执行稳定窗口（Panda 2.0 s、Marvin 1.0 s），sidecar 原样冻结该值；离线端不读取 YAML、不按任务名分支、不维护第二套时长阈值。
- **B02（Stack XY 投影）**：`_support_xy_error` 由 `R @ delta` 改为 `R.T @ delta`，世界偏移现在投影到目标局部系。目标/源块半尺寸、`support_xy_margin_m`、支撑边界与独立 `source_tilt`/`target_tilt` 条件均未改变。
- 未修改接触计数、交接状态机、准入/Follower/融合、训练、模型、数据、ROS/protobuf/schema 或容器配置。

### 6.2 验证命令与环境

容器使用既有 `sim-dual-arm-runtime-1`、`sim-dual-arm-simulation-1`（原镜像启动，未 build/pull，learning 未启动）。因 `tests/` 不在端容器挂载内，测试副本放入容器 `/tmp/p1view/tests`，`shared` 与源码通过符号链接解析到 `/shared`、`/app`，因此导入的是当前修改源码。执行入口为 `/shared/common/tools/scripts/test.sh -q <tests>`；命令内固定 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`。

| 环境 | 测试范围 | 加入回归后、修复前 | 修复后 |
| --- | --- | --- | --- |
| runtime | `test_stack_evaluation.py`、`test_task_result_sidecar.py` | 2 failed, 26 passed | 28 passed |
| simulation | `test_stack_task.py`、`test_handover_task.py` | 8 failed, 28 passed | 36 passed |

修复前基线（未加入新回归）：runtime 两文件 17 passed，simulation 两文件 24 passed；未发现既有失败。相关回归另行通过：runtime 5 个文件（加入 `test_offline_audit.py`、`test_episode_outcomes.py`、`test_stack_campaign.py`）48 passed；simulation 6 个文件（加入 `test_stack_task_execution.py`、`test_stack_teacher.py`、`test_task_config.py`、`test_handover_target_geometry.py`）66 passed。未使用 importorskip、未删除负例或放宽实现断言。

新增回归覆盖：合法 1.0 s、非 test-only 的 Policy 成功事实离线通过（`task_type` 分别为 handover/stack，证明通用逻辑不识别任务名）；`RUNNING` 状态、`conditions_passed=false`、缺失/零稳定窗口、`evaluated=false`、test-only、teacher 来源、失败结果仍被拒绝；Panda 在线评价在 1.999 s 保持 RUNNING、2.0 s 才 SUCCEEDED；Marvin 在线评价在 0.999 s 保持 RUNNING、1.0 s 才 SUCCEEDED；Yaw −10°/0°/10° 下目标局部偏移 `(0.027,0.027,0.05)` 的 XY over-edge 均为 −0.001 m，旋转边界内/外分别按 0.027999/0.028001 判断，90° 翻转仍被原倾角条件拒绝。

边界测试说明：条件是 over-edge `<= 0.0`；精确构造在边界上的点在浮点往返后可能落在 +1e-17，因此用边界内 1e-6 与边界外 1e-6 两个点验证，而不是断言精确零。新增几何回归在修复前共 8 个断言失败（±10° 投影用例与旋转边界用例），证明该回归确实覆盖转置错误。

`ruff` 在宿主机与两个容器内均未安装，未执行；已用 `python3 -m py_compile` 检查修改/新增 Python 文件，并检查行宽不超过 100。实际未运行完整三端测试套件。

### 6.3 只读历史复核

复核对象为 `var/panda_stack/logs/batch-policy/*/recordings/*/task_result.json` 与 Marvin 同结构文件，各 120 局；未运行仿真、未改写 MCAP/task_result/closure audit。

- **B01 重算**：对 240 份原始 sidecar 分别用修复前、修复后的 `_apply_task` 计算。Panda 成功集合前后均为 `closure120-panda-b16-ep-0002`、`closure120-panda-b23-ep-0001`；Marvin 前后均为 0。`closure120-panda-b14-ep-0003` 状态为 FAILED，修复后仍不是成功。
- **B02 重算**：用原始 final `source`/`target` 位姿和现有配置（half extent 0.03 m、margin 0.002 m）调用修复前/后 `_support_xy_error`。120 局中 XY 布尔值仅 `closure120-panda-b14-ep-0003` 由失败变通过（旧 +0.001804 m，新 −0.000552 m）；该局其余记录条件仍失败：`source_on_target_height`、`source_tilt`、`no_finger_contact`、`right_gripper_opening`、`right_tcp_clearance`，不能宣称任务成功。两个原成功局全部条件仍通过；修复后仍有 94/120 局 XY 不通过。
- 复核只重算 XY 项；其他条件的布尔值取自冻结录制，没有把重算当成第二套物理仿真。原 Marvin 均未产生成功事实，B01 不改变其 0/120。

### 6.4 文件、备份与恢复

实际修改：

- `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/evaluation_metrics.py`
- `simulation/mujoco/src/robot_backend_sim/tasks/stack/evaluator.py`
- `tests/unit/test_stack_evaluation.py`、`test_stack_task.py`、`test_handover_task.py`、`test_task_result_sidecar.py`
- 本任务书

未新增仓库文件。用户原有 `docs/experiments/README.md` 修改及未跟踪文档均保留，未覆盖或回退。

备份位于 `/tmp/sim_dual_arm-repair-P1-20261115T043749Z-59699/original/`，按相对路径保存上述被修改文件，另有 `checksums.sha256` 记录两个源文件的基线摘要。精确恢复方法：确认目标文件此后未被用户修改，再用 `cp -p /tmp/sim_dual_arm-repair-P1-20261115T043749Z-59699/original/<相对路径> /home/ljk/work/sim_dual_arm/<相对路径>` 恢复对应文件；只想撤销实现修复时恢复两个源文件即可。备份目录本身不放入仓库。

容器已恢复到开始前的停止状态；容器内 `/tmp/p1view` 已删除。宿主机 `/tmp` 复核脚本与备份保留供审查，可随时删除。

### 6.5 剩余限制

P1 只修正评分与几何正确性，不提高成功率。B01 仍是潜在假阴性的修复：原 Marvin 120 局没有完成任务，重算后仍为 0。B02 不新增成功局，也不解决 Panda 停滞、Marvin 越界、交接顺序验证（B03–B05）、接触证据（B04/B05）或输出准入问题。P1 未验证完整 240 局重放、未做新闭环、未训练、未改权重。

### 6.6 独立复核

已审阅两个源文件与四个测试文件的实际补丁，未发现需要退回 P1 的实质问题。另从当前源码提取原函数体进行独立检查：合法 1 秒事实、RUNNING/条件失败/零窗口/bool 窗口等负例、正负 10° 与零 yaw 下的 9 组几何用例均符合预期；使用修复前备份和当前函数重算原 240 份 sidecar，成功集合仍为 Panda 两局、Marvin 零局。

该复核没有重新运行完整模块导入、pytest 或 MuJoCo 闭环；前述 48/66 项相关回归的执行记录由 P1 实施阶段提供，不能与这次函数体复核混称为重复跑过。Ruff 未执行的限制保留。P1 可以作为 P2 的前置基线。

## 7. P2 实施结果（2026-11-15）

### 7.1 修复与行为变化

- **B04（接触活动性）**：`physics/contact_snapshot.py` 与 `tasks/stack/evaluator.py` 只聚合 `efc_address >= 0` 的有效约束。margin 内、正间隙的 inactive 接触不再计入手指接触数、持物或支撑；有效但瞬时法向力为 0 的接触仍保留。真实 MuJoCo 复现显示：同一正距离带上，`efc_address == -1` 的接触计数为 0，`efc_address >= 0` 且 `N=0` 的接触计数为 1。
- **B05（箱底支撑）**：`build_contact_snapshot` 新增按 geom 的 `SupportGeomContact`，按 geom1/geom2 顺序与接触法向计算支撑体对物块的世界 Z 向力；箱底条件只消费 `box_bottom`，不再使用整个箱体 body 合计。`tasks/handover/observation.py` 的 `SupportContact` 携带 `geom_name`/`upward_force_n`，并新增 `support_upward_force()`；`HandoverBinding` 从 `scene.indices.environment.box_geom_ids` 解析 `box_bottom_geom_names`。侧壁/桌面接触仍参与 `external_support_force()` 的空中借力检查，但不能通过最终箱底条件。
- **B03（空中交接证明）**：`tasks/handover/evaluator.py` 新增 owner-thread 的 `_SequenceFact` 状态机，不依赖 `plan.steps`：左手两指有效接触且物块离桌/无外部支撑并相对位姿稳定持续 `gates.stable_duration_s` → 双手空中稳定 → 左手真实释放（无有效接触、开度与速度门槛）后右手单独空中稳定。任一段中断清空当前计时，重复时间戳不累积时长，新阶段从零计时；`reset`/新 `task_epoch` 清空证明。Policy（`steps=()`）与 Teacher 均要求同一证明；`evaluation_conditions` 新增 `airborne_handover_sequence` 并接入成功窗口。Policy 下的持物丢失、交接窗口外部支撑、合法落箱后解除右手持物要求均已补齐。
- **B07（日志关联）**：`ros/node_services.py` 的 `backend.task.teacher_chunk` 使用 `EventContext(episode_id=context.episode_id, request_id=chunk.request_id)`，不再把 request_id 写入 episode_id。

未修改摩擦、servo、IK、关节限位、Follower、准入、融合、源配置阈值、训练、模型、数据、ROS/protobuf/schema 或 `GetContactSnapshot` 消息结构。`ContactSnapshot` 内部新增 `support_geom_contacts`；`contact_conv.py` 的公开消息仍只由原 body 级行生成。

### 7.2 内部字段与消费者

| 字段 | 生产者 | 消费者 |
| --- | --- | --- |
| `ContactSnapshot.support_geom_contacts` | `physics/contact_snapshot.py::build_contact_snapshot` | `tasks/handover/observation.py::capture_handover_observation` |
| `CompiledScene.geom_names` | `physics/scene_container.py::CompiledScene.__init__` | `build_contact_snapshot` |
| `BOX_BOTTOM_GEOM` | `physics/marvin/environment.py` | `tasks/handover/observation.py`（绑定箱底 geom） |
| `handover.observation.SupportContact.geom_name/upward_force_n` | `capture_handover_observation` | `support_upward_force`、`evaluation_conditions`、`HandoverTaskRuntime._condition/_snapshot/_holder_required` |
| `HandoverBinding.box_bottom_geom_names` | `make_handover_binding`（复用环境绑定） | 最终箱底条件、`released` 门、metrics、Policy 落箱判定 |
| `metrics["handover_sequence"]` | `HandoverTaskRuntime._snapshot` | `TaskState` → runtime sidecar 的 `metrics` 原样冻结；录制复核与诊断；测试 |
| `metrics["conditions"]` 的 `airborne_handover_sequence` | `evaluation_conditions` | `conditions_passed`、最终 `stable` 门、`_update_success` |
| `metrics["box_bottom_support_force_n"]` | `_snapshot` | 录制复核；替换原模糊的 `box_support_force_n`（箱体 body 合计） |

`handover_sequence` 保存阶段完成标志、实际完成仿真时间、当前/完成持稳时长及左右最小接触力；Producer 为任务 runtime，Consumer 为现有 sidecar/诊断读取路径。未新增逐 tick 文件或第二套 JSON 账本。

### 7.3 验证命令与环境

容器使用既有 `sim-dual-arm-runtime-1`、`sim-dual-arm-simulation-1`（`docker start` 原容器，未 build/pull，learning 未启动）。`tests/` 不在容器挂载内，测试副本放入容器 `/tmp/p2view/tests`，`shared`、`simulation/mujoco/src` 通过符号链接解析到 `/shared`、`/app`，因此导入当前源码。执行入口为 `bash /shared/common/tools/scripts/test.sh -q <tests>`，并设置 `PROJECT_ROOT=/tmp/p2view`、`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`（脚本内设置）。

| 环境 | 测试范围 | 修改前基线 | 修改后 |
| --- | --- | --- | --- |
| simulation | `test_stack_task.py`、`test_handover_task.py`、`test_handover_target_geometry.py`、`test_stack_task_execution.py`、`test_stack_teacher.py`、`test_task_config.py`、`test_handover_contact_evidence.py`、`test_stack_contact_evidence.py`、`test_teacher_chunk_events.py` | 4 文件 41 passed（P2 前，无新增测试） | 9 文件 96 passed |
| runtime | `test_stack_evaluation.py`、`test_task_result_sidecar.py`、`test_offline_audit.py`、`test_episode_outcomes.py`、`test_stack_campaign.py` | 2 文件 28 passed | 5 文件 48 passed |

新增真实物理测试覆盖：双指持物、inactive 正间隙、active 正距离、active 零法向力、真实箱底、仅侧壁、桌面支撑、geom1/geom2 反序，以及“接触采样→任务 observation→评价/序列门”贯通。新增序列/状态测试覆盖：左独持→空中共持→左释放→右独持正例（Policy 与 Teacher）；直接入箱、缺左独持、乱序、单指擦碰、桌面共持、开爪仍夹持、持物丢失超容忍、短时丢失在容忍内、计时中断/重复时间戳、交接窗口外部支撑、跨 `task_epoch` 旧证明、机器人与箱体禁止接触、跌落守卫。最终 1 秒窗口仍用 0.999 s 不成功、1.0 s 成功验证，且先建立完整交接前缀。原“只入箱即成功”的测试已改为拒绝负例并补完整交接正例。

**旧行为对照**（证明回归覆盖旧缺陷，不作为新验收）：把 P2 修改前的 `stack/evaluator.py` 放回临时树后，新增 stack 接触证据测试 4 failed / 2 passed（inactive 计数、右指证据、观测计数均复现旧行为）；旧版 handover 测试在原实现上仍接受“直接入箱”成功；用原 `contact_snapshot.py`/`observation.py`/`evaluator.py` 对仅侧壁接触实测，旧 `box_bottom_support` 为 true（2.2226 N），新实现为 false。

**既有失败边界**：较宽的 simulation 单测子集（34 文件）为 459 passed、9 failed；其中 8 项在放入 P2 原文件的临时树上同样失败，属于现有配置/测试漂移（inference 配方预算 10.0 与当前 180.0、dataset `diagnostic` 标志、follower `target_count` 10 与 5、follower 数值断言），另 1 项是 `/tmp` 测试视图的 `ROOT` 路径期望，均不涉及接触、交接或 stack 评价。未使用 importorskip、未删除负例、未放宽断言；ruff 仍不可用，已用 `py_compile` 和 100 列检查。

### 7.4 历史证据边界

未改写、补造或重算原 MCAP/task_result/closure audit；未把旧 Marvin 局重标成功，Panda 仍为两局、Marvin 仍为零局。旧录制没有 geom 级箱底/侧壁或新交接证明字段，P2 不从中回填事实，也不提供旧验收回退路径。一次真实 `GetContactSnapshot` 转换验证确认公开消息结构未变（body 级 support 行仍可生成）。

### 7.5 文件、备份与恢复

实际修改：

- `simulation/mujoco/src/robot_backend_sim/physics/contact_snapshot.py`
- `simulation/mujoco/src/robot_backend_sim/physics/scene_container.py`
- `simulation/mujoco/src/robot_backend_sim/physics/marvin/environment.py`
- `simulation/mujoco/src/robot_backend_sim/tasks/handover/observation.py`
- `simulation/mujoco/src/robot_backend_sim/tasks/handover/evaluator.py`
- `simulation/mujoco/src/robot_backend_sim/tasks/stack/evaluator.py`
- `simulation/mujoco/src/robot_backend_sim/ros/node_services.py`
- `tests/unit/test_handover_task.py`
- `docs/experiments/2026-11-15-inference-repair-taskbook.md`、`docs/experiments/README.md`

新增：

- `tests/unit/test_handover_contact_evidence.py`
- `tests/unit/test_stack_contact_evidence.py`
- `tests/unit/test_teacher_chunk_events.py`

P1 修改的 `runtime/.../offline/evaluation_metrics.py`、`tasks/stack/evaluator.py` 的 `R.T` 投影、四个 P1 测试文件及用户原有文档修改全部保留。也备份了本次未实际改动的 `tests/unit/test_stack_task.py`、`tests/unit/test_stack_task_execution.py` 原始文件以便复核。

备份位于 `/tmp/sim_dual_arm-repair-P2-20261115T051541Z-79408/original/`，按相对路径保存上述被修改原文件（`scene_container.py`、`environment.py` 从 HEAD 只读提取）。精确恢复方法：确认目标文件此后未被用户修改，再 `cp -p /tmp/sim_dual_arm-repair-P2-20261115T051541Z-79408/original/<相对路径> /home/ljk/work/sim_dual_arm/<相对路径>`；撤销新增测试则删除上列三个新文件。备份目录不放入仓库。

### 7.6 剩余限制

P2 只保证成功判定和物理证据可信，不提高成功率。没有运行完整闭环、批量 Policy 推理、训练或新采集；没有在真实 Marvin 闭环中重新验证交接正例，只运行了真实 MuJoCo 最小接触模型与任务状态机/适配测试。真实机器人执行、完整 ROS 集成测试和所有历史 MCAP 的新指标重放未运行，记为未验证。P2 后旧录制不因新证明自动变为成功；真实任务的交接正例仍需在 P3/P7 的完整运行中复核。

### 7.7 独立复核发现项（已补修，见 §7.8）

独立复核时，箱底 geom 专用支撑、inactive 接触区分和 episode/request 日志分离的实现方向与范围一致，但发现新增序列参考的生命周期存在可达问题，因此当时暂缓 P2 验收。以下记录当时发现的两个问题；现已完成补修，最终结果见 §7.8。具体实施范围见 [P2 补修提示词](2026-11-15-stage2-followup-prompt.md)。

**R2-01：未建立阶段证明的候选抓持参考在断触后没有失效。** `_update_sequence` 在首次 `holding` 时保存相对位姿，条件失败时只清空计时，`_remember_sequence_reference` 又因该手已有条目而始终拒绝更新。右手尚未完成共持持稳就短暂松开，再在物块上另一个有效位置重新抓稳，会一直与第一次尝试的旧位姿比较，直到任务超时，无法完成共持证明。

独立复现使用既有 simulation 镜像、当前完整模块和既有 `test_handover_task` fixture：先完成左手空中独持；右手共持 0.1 秒后断触 0.1 秒；随后以沿物块长轴变化 10 mm 的 TCP 相对位置重新双指抓持，并连续保持 1 秒以上。左右 `holding` 均为 true，无外部支撑、无 HOLD_LOST，仍停在 `dual`，`dual_airborne_stable=false`。当前 5 mm 相对位姿容差与旧参考产生 10 mm 差异，导致候选计时始终为零。

该新增复核测试实际结果为 **1 failed in 0.69 s**；它不在原 96 项测试中，因此不否定原 96 passed 的执行记录，但暴露了该组测试未覆盖的恢复路径。应增加“候选断触后异位重抓”正例，并保持持续滑移/抖动不得通过的负例。修复不能逐帧追随当前姿态来消除误差，也不能放宽容差或持物丢失保护。

**R2-02：两个新增必需物理测试模块仍可静默跳过。** `tests/unit/test_handover_contact_evidence.py:18` 与 `test_stack_contact_evidence.py:17` 新增 `pytest.importorskip("mujoco")`，与 P2 提示词要求不符。已有 MuJoCo 环境中的 96 passed 不受影响，但环境缺依赖时不能把本阶段必需物理验证当可选项跳过。移除这两处新增跳过，在正确项目环境运行，缺依赖应显式失败并报告；不要扩大为清理所有历史测试。

本次仅增加文档复核记录，没有修改生产代码或仓库测试。临时回归在容器 `/tmp/p2reference-review` 中运行并已清理；simulation 复用原镜像后恢复停止，runtime/learning 未启动，env.sh 的临时启动方式已复原，无 Git 写操作。未重复跑全套 96/48 项或完整闭环，也未启动 P3。

### 7.8 P2 补修结果（2026-11-15）

**R2-01 已修复。** 物理序列中的左手参考在完成左独持证明后固定，右手参考在完成双手共持证明后固定；在各自证明成立前失去有效抓持，会清除该手的候选相对位姿，重新抓稳时重新建立参考。现有阶段计时中断逻辑继续清零，重新抓持必须从新观测起连续满足原窗口。已经成立的证明不会被清掉，参考也不会逐帧跟随滑移。

Teacher 的步骤门控也存在同类候选参考：`_update_steps` 现在在未建立该手持物时失去有效抓持便清除候选参考。其 `_established` 状态和物理序列的证明时刻分别负责既有步骤门控与独立物理证明，不改变持物丢失容忍时间。已经成立的持物参考继续固定，原 HOLD_LOST/EXTERNAL_SUPPORT/顺序和 epoch 清理规则保留。

**R2-02 已修复。** 两个新增的接触证据测试模块直接导入 `mujoco`，移除了各自新增的 `pytest.importorskip`；缺依赖会显式报错。没有批量修改历史测试中的跳过策略。

新增 8 项参数化回归覆盖：Policy/Teacher 右手候选断触后异位重抓（2 项）、左手候选重抓重新计时、Teacher 左手抓取门控恢复、左右手持续滑移（2 项）、反复重抓不能累积时长，以及已成立左手证明/参考在短暂失去接触后仍保持。重抓测试使用边界前 0.499 s 不通过、满原 0.5 s 才通过；持续滑移和已成立参考漂移仍拒绝。Teacher 边界推进以实际 `step_name` 验证，不把滞后一帧的行为标签当作当前步骤。

| 验证 | 实际结果 | 证据 |
| --- | --- | --- |
| 新回归，修改前 | 4 failed、4 passed、25 deselected | [修改前记录](../../var/_system/diagnostics/p2-followup-red-simulation.log) |
| 新回归，最终状态 | 8 passed、25 deselected | [针对性回归](../../var/_system/diagnostics/p2-followup-focused-simulation.log) |
| P2 九个 simulation 文件（含 P1 几何与真实接触回归） | 104 passed，0 failed、0 skipped | [simulation 结果与命令](../../var/_system/diagnostics/p2-followup-final-simulation.log) |
| P1/P2 五个 runtime 文件 | 48 passed，0 failed、0 skipped | [runtime 结果与命令](../../var/_system/diagnostics/p2-followup-final-runtime.log) |

104 项已包含新增 8 项，不能再次相加计数。仍使用 §7.3 中相同的九个 simulation 和五个 runtime 测试文件；测试运行器 `/tmp/run_p2_followup_checks.py` 在容器 `/tmp/p2followup-*` 建立临时测试视图，输出记录实际导入路径为 `/app/src/.../evaluator.py`、`/app/ros/.../evaluation_metrics.py`，消费本次挂载源码。原镜像通过 env.sh 的临时“启动已有容器”路径复用，未 build/pull，env.sh 随即恢复原文。

本轮修改的实现/测试为 `tasks/handover/evaluator.py`、`tests/unit/test_handover_task.py`、`test_handover_contact_evidence.py`、`test_stack_contact_evidence.py`，并同步本任务书、补修提示词状态和文档索引。无新增生产文件、跨端接口或参数，P1/P2 其他补丁保持。未训练、采集、调整准入/Follower/融合或修改模型和原数据。

备份为 `/tmp/sim_dual_arm-repair-P2-followup-tv17ve6c/original/`，按相对路径保存本轮前的原文件，其中已包含此前 P1/P2 修复。复原时先确认没有后续修改，再把需要撤回的文件从该目录逐个 `cp -p` 回仓库；不要使用更早 P2 备份来撤回本轮，以免同时撤掉原 P2。备份中也保存了本轮文档前态和 env.sh；本轮未新增仓库测试文件，8 项回归均在既有测试模块内。

全部临时测试视图已删除，runtime/simulation 恢复停止，learning 未启动，env.sh 与本轮前一致，无 Git 写操作。已检查 Python 语法编译、100 列行宽和 `git diff --check`。Ruff 未执行；没有重跑旧 459 项宽子集、完整闭环、全量录包重放或真机，不能据本轮测试宣称成功率提高。P2 补修验收通过，P3 保持待执行。

## 8. P3 难度评估与离线定位（2026-11-15）

### 8.1 难度与拆分

P3 整体较难：准入决定、实际选窗、融合、继承参考和夹爪时序互相影响。已继续亲自完成 P3-A 离线定位；下一小步 P3-B 收敛为现有准入中的按轴有界投影，难度中等，提示词已准备。P3-C 夹爪重新定时仍是较难项，需要独立设计与闭环验证。

详细方法与权衡见 [P3 离线分析](../../var/_system/diagnostics/2026-11-15-p3-execution-analysis.md)，逐请求数据见 [P3 重放结果](../../var/_system/diagnostics/p3-execution-replay.json)。两者直接服务于本任务书和 P3-B 的原输出验收；不另存原张量副本，不增加总清单或源码摘要。

### 8.2 实际执行与结论

复用了既有 simulation 镜像（Ruckig 0.15.3、NumPy 1.26.4），只调用当前挂载源码做数学重放，未启动模型、运行物理 step 或新增闭环。输入为 5 个原局和既有 5 个诊断局，以及原 Marvin 118 个被 Policy 拒绝的预测；配置来自各局冻结 runtime 快照。

- **733 个命令与 NPZ 完全一致。** 有融合时与 execution_targets 比较，否则与 admitted 比较；保留了原始输出和实际输出的区别。
- **28 条已接受参考重算一致。** 按各局首/中/末抽取代表点，段数与段时长相同，c0–c3 最大差为 0。这是数学重放，不是新增闭环或单测数量。
- **118 个 Marvin Policy 拒绝预测的实际选窗全部合法。** 实际 selected_indices 为 `20..39` 或 `21..40`；所有越界均在 46 及以后的未执行尾部，越界轴只有 left_joint7，最大所需修正为 0.005338–0.007671 rad。全 chunk 严格范围检查是当前显式契约，不能只删除某层检查冒充修复。
- **两次夹爪 Follower 拒绝正确。** 重放得到原偏差；从继承 q/v 和原 vmax/amax 积分的宽松可达包络表明，Panda window20 第 21 块、Marvin clip002 第 13 块在首选点不可避免的误差下界分别为 55.659/19.716 mm，已经超过原 40/10 mm 预算。当前规划器即便被替换，也不能让这两条原动作在原约束下通过。

原 240 局没有改判或重新计数，没有改写 MCAP、NPZ、task_result、closure audit、模型、数据或权重。

### 8.3 下一小步的具体候选

已比较“仅按实际选窗准入”和“按轴小幅有界投影”。前者保留原值，但影响 PolicyService、PolicyClient、backend 和共享/离线范围验证的含义；后者可复用现有 raw/admitted/correction 证据，并保持整块输出在原硬限位内，因此准备后者作为改动更集中的 P3-B 候选。

候选仅在 Marvin 方案中给 left_joint7 设置 0.01 rad 修正额度，其他臂轴为零，原夹爪额度保持。0.01 rad 是基于本批输出的候选最大修正量，不是新的硬限位或通用安全余量。它显式改变此前臂关节不裁剪的部署实验条件；**该候选已在 §9 中实施，P3-C 仍未完成。**

生产实现应以一份按轴映射替换原仅夹爪额度并同步调用方，不保留双轨参数或添加纯转发包装；零额度严格拒绝，超额拒绝，原输出不变，完整 60 点仍由各端按原 Profile 严格检查。P3-B 即使通过也只解决第一道阻断，不解决上述夹爪不可达和闭环成功。

### 8.4 文件、复原与限制

本轮只更新本任务书及 `docs/experiments/README.md`，新增 P3-B 提示词、上述诊断 Markdown/JSON。未修改生产源码、测试或方案配置。文档修改前态及 env.sh 原文备份为 `/tmp/sim-dual-arm-P3-evidence-aghhdtv6/original/`，按相对路径保存；撤回时确认文件没有后续修改，再逐个 `cp -p` 恢复本轮改过的两个既有文档，新文件仅在确认没有后续修改后删除。不要恢复生产源码或早期 P1/P2 备份。

离线脚本 `replay.py`、`arm_replay.py` 保留在该 `/tmp` 目录，消费原录制并生成当前复核结果。env.sh 的临时 start 变更在启动后立即恢复；没有 build/pull，simulation 已恢复停止，runtime/learning 本轮未启动，容器内临时脚本已删除，无 Git 写操作。

未运行新增单测、全量物理重放、新闭环或真机；P3-B 未实现，P3-C 未设计完成，P3 总体不能记为通过。上述结论只定位了已有失败和下一步范围，没有成功率改善证据。

> 后续更新（2026-11-15）：P3-B 已按同一候选在 §9 实施并通过其限定验证；P3-A 的“P3-B 未实现”仅描述本轮范围。P3-C 与 P3 总体验收仍保持未完成。

## 9. P3-B 实施结果：按轴有界输出准入（2026-11-15）

### 9.1 修复内容与行为变化

保留 ACT 线性输出头、现有权重和全部 RobotProfile 硬限位，只替换学习端部署准入的额度模型：

- `chunk_admission.py::admit_action_chunk` 的第五个参数由标量 `max_correction_m` 改为按 joint name 的 `corrections: Mapping[str, float]`。额度以该轴 Profile 单位（rad 或 m）计量。配置额度大于零的轴用原 lower/upper 投影，并对**完整 60 点**的每轴最大修正量与额度比较；未列出或额度为零的轴必须整块已在硬限位内。超额拒绝；拒绝时仍保留原始输出。
- `pipeline.py` 的准入不再因“未启用证据/融合”而跳过：空映射、全部零额度、关闭时间融合都执行同一严格整块范围检查。准入通过后才计算时间融合预览；关联、缓存和 `commit` 仍在服务端完整校验之后，拒绝不会写入融合历史、不会进入请求成功缓存，也不会返回旧动作。是否保存 NPZ 仍由“有任一非空额度映射或启用融合”控制，与准入判断分离。
- `policy_config.py` 以 `max_prediction_corrections: dict[str, NonNegativeFloat]` 替换 `gripper_max_correction_m`。字段从 Policy 配方的 `model` 段读取；Pydantic 入口拒绝未知轴名、负数、非有限值和非数值。旧的 `0.03 m` 单一上界随字段删除；每轴修正额度由本阶段明确的方案配置给出，投影结果受原 Profile 硬限位约束。Profile 不为修正额度本身提供上界，不能据此宣称可任意增大配置额度。
- 方案配置是唯一参数来源：Marvin `left_joint7: 0.01`（rad）、两夹爪 `0.005`（m），其余臂轴未配置即为零；Panda 两夹爪 `0.03`（m），全部臂轴为零。通用学习代码不包含机器人/任务名或固定索引分支。
- NPZ 仍只使用既有 `raw_act/admitted/correction/accepted/reason` 与可选融合字段；`raw_act` 为未修改的模型输出，`admitted` 是投影后整块，`correction = admitted - raw_act`。未新增张量日志、wire 字段或第二份额度配置。

未修改 Follower、`ActionWindow` 选择、末点规划、Teacher、采集时序、训练、数据、归一化、模型头、RobotProfile、成功阈值或 runtime/backend/ROS 的整块范围检查。

### 9.2 字段与消费者

| 字段 | 生产者 | 消费者 |
| --- | --- | --- |
| `PolicyServiceConfig.max_prediction_corrections` | 方案 `configs/learning/policy*.yaml` 的 `model` 段，经 `load_policy_config`/`parse_policy_config` 校验 | `ChunkedPolicyPipeline._admit` → `admit_action_chunk`；`to_document()`/resolved contract 仅记录生效配置 |
| `Admission.positions/correction/accepted/reason` | `admit_action_chunk` | 融合预览、`PipelineResult`、NPZ、PolicyService 返回路径 |

runtime/backend/ROS 不读取该映射，继续严格检查完整 60 点处于 Profile 范围内。

### 9.3 准入约定

| 情形 | 结果 |
| --- | --- |
| 配置轴在额度内正向/负向越界 | 投影到原边界，`correction` 符号与单位正确，`raw_act` 不变 |
| 配置轴超过额度 | 拒绝，报告轴名、实际修正量与额度；`raw_act` 保留 |
| 未配置轴或显式零额度越界 | 拒绝；最后未选点越界同样拒绝，证明仍是全 chunk 契约 |
| 空映射 | 无任何绕过：仍逐轴执行严格硬限位检查 |
| 全部点合法 | 任意额度设置都不改变输出（`admitted == raw_act`） |
| 有融合 | 单个原始 chunk 先独立准入，不能被历史平均值抵消；拒绝不增长历史 |
| 拒绝 | 不写入 `request_cache`、不推进 `last_observation_id`、不返回旧动作 |

### 9.4 验证命令与环境

复用既有 `sim-dual-arm-learning:current` 镜像，未 build/pull。测试不在容器挂载内，最终测试视图位于容器 `/tmp/p3bview/tests`，`shared → /shared`、`learning/lerobot → /app`、`build → /build`；导入路径实测为 `/app/src/dual_arm_learning/...`。入口为 `PROJECT_ROOT=/tmp/p3bview bash /shared/common/tools/scripts/test.sh -q <tests>`（脚本内设置 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`）。跨端配方测试使用同一 simulation 镜像的一次性 `docker run --rm`，临时挂载 runtime 源码与当前 tests。

| 验证 | 结果 | 证据 |
| --- | --- | --- |
| 修改前基线（同六个 learning 测试文件） | 3 failed、47 passed | [基线](../../var/_system/diagnostics/p3b-baseline-learning.log) |
| 修改后同六文件 | 63 passed | [结果](../../var/_system/diagnostics/p3b-final-learning-focused.log) |
| 相关 learning 协议/服务补充 | 9 passed | [结果](../../var/_system/diagnostics/p3b-final-learning-extra.log) |
| 新回归对旧生产代码 | 18 failed、16 passed | [红测](../../var/_system/diagnostics/p3b-red-against-old-code.log) |
| 跨端 Marvin 学习配方与 runtime 配方 | 4 failed、2 passed；4 项为 P2 §7.3 已记录的既有配置漂移，新增逐轴额度断言通过 | [结果](../../var/_system/diagnostics/p3b-final-cross-end-recipe.log) |
| 原输出只读反事实准入 | 见 §9.5 | [复核](../../var/_system/diagnostics/p3b-original-output-replay.log) |
| 导入源码与 API | `/app/src` 当前源码；旧字段不存在 | [记录](../../var/_system/diagnostics/p3b-import-and-signature.log) |

三个基线失败均为既有问题：提交 `e571c5c` 把 Panda 配方的 `temporal_ensemble_coeff` 改为 `null`，而三个融合测试仍假设配方启用融合。现在这些测试通过 `fused()` 显式启用融合系数，保留原有断言，不依赖配方默认值。新增回归覆盖：额度内正/负投影与边界、额度单位区分、超额、未配置轴、显式零额度、空映射、最后未选点超额、真实配方贯通 Pipeline、PolicyService 返回路径 60 点全在原范围、NPZ 原始/投影/修正一致，以及融合不能掩盖单块越界且历史不增长。未使用 `importorskip`、删除负例或放宽断言。

`ruff` 在环境中不可用，未执行；已用 `python3 -m py_compile` 检查全部修改/新增 Python 文件，并检查行宽不超过 100。未运行完整三端测试套件。

### 9.5 原输出只读复核

复核只读取原 NPZ 与已冻结的重放 JSON，不写回任何 NPZ、sidecar、MCAP 或原审计，也未运行模型或物理仿真。

- **Marvin 118 个原 Policy 拒绝预测**：旧准入 0 个接受；新准入 118 个全部接受。新增修正只有 `left_joint7`，首修正索引分布为 46（1 个）、48（60 个）、49（13 个）、50（44 个），最早为 46；最大幅度 0.007671 rad（小于 0.01 rad 候选额度）。相对旧 `admitted`，差异仅出现在 `left_joint7` 且索引 ≥46；每个预测的 20 个真实 `selected_indices`（含夹爪）逐值不变，选中臂关节与原始输出完全一致。
- 这批原始块中另有少量夹爪投影（左最大 0.000731 m、右最大 0.000323 m），均在原 0.005 m 额度内，旧准入同样会投影，不是 P3-B 新增行为。
- **Panda 既有样本**：`closure120-panda-b11-ep-0000`（原停滞）、`closure120-panda-b16-ep-0005`（夹住后停滞）、原成功 `closure120-panda-b16-ep-0002` 与 `closure120-panda-b23-ep-0001` 共 4 局 463 个预测。新旧准入的接受判定与投影结果逐值一致，差异 0，全部已接受；Panda 方案额度未改变。
- 反事实通过不等于服务端真实返回、物理执行或任务成功；原 240 局成功率与成功集合没有任何改写。

### 9.6 文件、备份与恢复

实际修改：

- `learning/lerobot/src/dual_arm_learning/chunk_admission.py`
- `learning/lerobot/src/dual_arm_learning/pipeline.py`
- `learning/lerobot/src/dual_arm_learning/policy_config.py`
- `shared/scenarios/marvin_handover/configs/learning/policy.yaml`
- `shared/scenarios/panda_stack/configs/learning/policy_temporal.yaml`
- `shared/scenarios/marvin_handover/README.md`
- `tests/unit/test_chunk_admission.py`
- `tests/unit/test_lerobot_act_adapter.py`
- `tests/unit/test_policy_config.py`
- `tests/unit/test_handover_inference_recipe.py`
- 本任务书、[P3-B 提示词](2026-11-15-stage3-admission-prompt.md)、[文档索引](README.md)

新增诊断日志（消费者为本任务书与复核；均在 `var/_system/diagnostics/`）：

- `p3b-baseline-learning.log`、`p3b-final-learning-focused.log`、`p3b-final-learning-extra.log`
- `p3b-red-against-old-code.log`、`p3b-final-cross-end-recipe.log`
- `p3b-original-output-replay.log`、`p3b-import-and-signature.log`

未新增生产文件、跨端消息字段或第二份额度配置。P1/P2 补丁及用户原有未提交修改全部保留。

备份位于 `/tmp/sim_dual_arm-repair-P3B-20261115T081134Z-12420/original/`，按相对路径保存修改前文件（含 `env.sh` 与本轮文档前态），并以 `checksums.sha256` 记录。精确恢复：确认目标文件此后未被用户修改，再对需要撤回的文件执行 `cp -p /tmp/sim_dual_arm-repair-P3B-20261115T081134Z-12420/original/<相对路径> /home/ljk/work/sim_dual_arm/<相对路径>`；本轮新增的七个诊断日志在确认无后续消费者后可删除。不要用更早的 P1/P2 备份恢复本轮文件，以免同时撤销已通过的补丁。

容器复用原 `sim-dual-arm-learning-1`：`env.sh` 仅在启动时临时把 `up -d --build` 改为 `up -d --no-build`，启动后立即按原字节恢复，恢复后 `sha256=2ecc68b55dff3841a51cbd523042842819469568ce8837fe628a55c8fd444876`。未 build/pull；runtime/simulation 未启动；跨端测试使用 simulation 镜像的一次性容器；结束时 learning 恢复停止、容器内 `/tmp/p3bview`、`/tmp/p3b-oldpkg` 与 `/tmp/p3bcheck.py` 删除。无 Git 写操作。

### 9.7 未验证项与下一阶段

- 未运行新闭环、批量推理、物理重放、训练、补采或真机；没有成功率改善证据。Panda 仍为原两局成功、Marvin 仍为零局。
- 两次代表性夹爪 Follower 拒绝仍按 §8.2 的不可达性证据存在；P3-B 不消除它们。
- P3-C 夹爪重新定时与阶段推进未设计完成；P3 总体不能记为通过，P3-B 只能记为“输出准入候选实现完成”。
- 未验证所有历史录制在新准入下的服务端真实返回；§9.5 是离线反事实。
- 本轮不自动进入 P3-C 或 P4–P7。

### 9.8 P3-B 独立复核（2026-11-15）

已复核三个生产模块、两份方案配置、调用方和新增测试。准入始终执行、按轴额度、全 chunk 拒绝、原始输出保存及先准入后融合的实现与候选方案一致；没有发现本阶段必须退回实现的实质问题。P3-B 可按“输出准入候选实现完成”验收，P3 总体仍未通过。

在既有 learning 镜像、当前 `/app/src` 挂载源码上独立运行针对性测试：`test_chunk_admission.py`、`test_policy_config.py`、`test_lerobot_act_adapter.py` 中使用 `quota or correction or in_range or empty_mapping or real_recipe or configured_axes or unlisted or admission_evidence or optional_temporal` 选择的用例，结果 **14 passed、23 deselected**，无失败或跳过。没有重复运行 §9.4 的全部 63/9 项，也未重复跨端配方测试；14 项与原回归有重叠，不能相加为独立覆盖数量。

另用当前完整准入函数和 P3-B 备份中的旧函数读取原 NPZ 重算：Marvin 118 个旧拒绝均通过新准入，每个真实选窗的完整位置与旧 admitted 相同，选中臂轴与 raw 相同；相对旧 admitted 仅 left_joint7 改变，最大 0.007670706939697292 rad。Panda 四局 463 个预测的新旧接受判定和投影结果完全一致。

这次逐预测复核也纠正了 §9.5 的统计措辞：原日志 `axis_first_index` 是跨样本最小值，不能解释为“所有预测均从 46 开始”。实际分布为 46/48/49/50，数量分别 1/60/13/44；这不改变尾部越界、选中点不变或 118 个反事实通过的结论。§9.1 同时明确区分“配置中的最大修正额度”和“投影后动作的 Profile 硬限位”。

证据沿用单个诊断日志：[独立测试与 NPZ 复核](../../var/_system/diagnostics/p3b-independent-review-learning.log)。没有生成新源码 hash、复制原始张量、改写原 NPZ/MCAP/sidecar 或修改生产代码/配置。本次仅修改本任务书并新增该日志；任务书修改前态及 env.sh 备份位于 `/tmp/sim_dual_arm-P3B-review-ejbtigpg/original/`，相对路径保存。确认无后续修改后逐文件恢复，不使用早期 P1/P2/P3-B 实施备份覆盖当前状态。

临时容器测试视图已删除；env.sh 启动后立即恢复原字节，learning 已恢复停止，runtime/simulation 本轮未启动，没有 build/pull 或 Git 写操作。未运行新闭环、物理重放、训练或真机；本次验收不增加成功率证据，也未进入 P3-C。

## 10. P3-C 实施结果：显式夹爪过渡重新定时（2026-11-15）

### 10.1 原因定位与假说判定

用原 NPZ、`/runtime/commands` 中的原始选窗/继承 anchor、`FollowerReference`、`ExecutionFeedback`（含参考/实测开度、速度、接触与 `tracking_quality`）重建了两个代表请求前后的连续窗口。两个原请求的所有时间仍为观测后 50 ms 网格，首个选中点均在交接后 0.05 s；未发现 observation/handoff/预测索引错位。

| 假说 | 结论 | 证据 |
| --- | --- | --- |
| 1. 模型自身开合方向反复/过早切换 | **成立，主因** | 同一物理时刻的相邻预测不一致：Panda 在 21.35 s 处，20.3 s 的预测给 0.0003 m（闭合），21.3 s 的预测给 0.0433 m（开张），差 43 mm；Marvin 在 14.35 s 处，12.35 s 的预测给 0.0625 m（闭合），13.35 s 的预测给 0.0814 m（开张），差 19 mm。执行器忠实执行了较旧、较远视界的预测。 |
| 2. observation/handoff/预测索引对齐错误 | **被推翻** | 60 点均为 `(i+1)×50 ms`；`ActionWindow` 的 selected_indices 与录制 `FollowerReference`/命令逐值一致；失败点均为交接后 0.05 s，Panda 选窗 21..40、Marvin 20..39。 |
| 3. 前序参考滞后累积 | **部分成立** | 非最终曲线在窗口末端被截断：Panda chunk 19 目标 0.0164 m，导出终点 0.013108 m；随后 chunk 20 从该值继续。持物时参考还可低于实测孔径约 18 mm（见假说 6）。 |
| 4. 非零继承速度后的反向动作不可达 | **成立（Marvin）** | Marvin 继承 `q0=0.062496 m, v0=-0.028877 m/s`，首个目标 0.081393 m；宽松可达上界 0.061677 m，不可避免误差 0.019716 m > 0.010 m。 |
| 5. 单末点规划丢失窗口内部时序 | **成立但非充分修复方向** | 参考只规划到最后选中点；内部开合时序不进入曲线。但即使替换规划器，从继承状态到首个目标的宽松可达包络误差仍为 55.659/19.716 mm，超过原预算；末点/终端状态决定下一 anchor，规划器替换不消除该矛盾。 |
| 6. 持物时实测开度与闭合参考的正常差异 | **成立（Panda）** | 右指两指在 20.3–21.35 s 实测 0.03966–0.04044 m、接触力约 0.45–1.63 N，参考约 0–0.0222 m；`CONSTRAINED/CONTACT_UNLOADING` 只豁免跟踪，不改变继承值。下一 anchor 继承参考 0.022205 m，而模型新目标 0.079865 m 更接近实测 0.0404 m。 |

未采用的方案：放大 `max_target_deviations`、提高速度/加速度、关闭跟踪或整块检查、用平滑目标悄悄替换比较目标、把实测开度直接写成继承参考。它们都会改变原契约或掩盖原始偏差。

模型跨预测的相位不一致本身是示教/训练/时间分布问题，留给 P4/P5；本阶段保留原始模型输出，但通过显式改写执行目标尝试解除继承状态下的不可达请求。该改写是**保持时间戳的执行目标整形**：选中轴的时间点和选窗不变，选中位置值被改成从继承状态朝原末点的直线并逐点裁入宽松包络，因此原窗口内的先开后合等内部时序不保留。原始模型目标偏差与适配执行目标偏差必须分别报告；适配目标满足 Follower 预算不等于原始目标满足预算。单点位于包络内只是必要条件，不保证整段曲线存在；真实 Ruckig、连续限位、制动与 Follower 偏差检查继续保留并传播失败。该机制只在 Policy 入口启用，Teacher 保持 P3-C 前的完整原始目标。

### 10.2 修复内容与行为变化

- 新增通用纯数学模块 `shared/common/python/trajectory/src/robot_trajectory/transition_retiming.py`：对每个夹爪轴，用 Profile 硬限位、`max_velocity`、控制 `max_acceleration` 和该轴 Follower 预算计算从 `(q0,v0)` 出发的宽松可达区间。若某个选中目标超出该区间超过预算，则原请求必然被拒绝（必要矛盾）；此时把该轴的选中目标重定时为
  `p'_k = clamp(q0 + (p_last - q0) * t_k / W, lower(t_k), upper(t_k))`，
  时间、选窗和整块尾部不变。未触发的轴保持原始目标逐值不变。臂轴不参与本阶段重定时。
- `PolicyStreamRunner._retime` 在提交前对 `ActionWindow` 的选中夹爪目标执行上述整形；`ContinuousStreamRunner` 基类的 `_retime` 返回 `None`，Teacher 路径不调用该机制，完整 60 点原目标、chunk identity/digest、门控与既有 backend 拒绝行为不变。Policy 提交的是显式适配后的新请求，原模型目标仍保留在 NPZ/策略侧。未改准入、Follower 预算、规划器、融合、窗口长度、物理参数、成功条件或 runtime/backend 的整块范围检查。
- 非最终计划仍精确结束在窗口末端（Panda/Marvin 均为 1.0 s）；导出终点的位置/速度经既有 `compute_active_chunk_anchor` 成为下一 anchor，未新增时间轴或延长曲线。

### 10.3 字段、记录与消费者

| 记录 | 生产者 | 消费者 |
| --- | --- | --- |
| `follower.chunk_retimed`（`axis`、`original_first/last`、`adapted_first/last`、`original_excess`、`deviation_budget`、`selected_points`、`request_id`） | `PolicyStreamRunner._retime` → `chunk_stream.py::_emit_retiming`；Teacher 不发该事件 | 本任务书、闭环诊断与逐轴复核；不新增文件或 wire 字段 |
| 适配后的 `Trajectory.targets` | `_request` | 既有 `/runtime/commands`、`FollowerReference.request/window`、实际参考 |
| 原始模型目标（`raw_act/admitted`） | 既有 P3-B NPZ | 与命令/参考通过 `request_id`、`observation_id`、`chunk_index` 关联 |

### 10.4 验证结果

| 验证 | 结果 | 证据 |
| --- | --- | --- |
| 新增纯回归（静止起点、非零开张速度、反向切换、窗口内开合变化、未选尾部、边界/限位、轴选择、确定性） | 9 passed | `test_transition_retiming.py` |
| 新增 runtime 集成回归（适配目标进入命令、未选点不变、retimed 事件） | 1 passed；旧 `chunk_stream.py` 下 1 failed/13 passed | [红测](../../var/_system/diagnostics/p3c-red-runtime-old-code.log)、[最终 runtime 聚焦](../../var/_system/diagnostics/p3c-final-runtime-focused.log) |
| runtime 相关 11 文件 | 129 passed | [最终 runtime 聚焦](../../var/_system/diagnostics/p3c-final-runtime-focused.log) |
| 新增真实 Ruckig 回归（两条原请求仍拒绝、适配请求接受、两类偏差分别报告、计划时长=窗口、下一 anchor） | 4 passed | [simulation 聚焦](../../var/_system/diagnostics/p3c-final-simulation-focused.log) |
| 新增真实 MuJoCo 持物回归（参考闭合被物块挡住，实测开度高于参考、双指 active 接触） | 1 passed | 同上 |
| simulation 相关 10 文件 | 136 passed、2 failed | 2 项为 P2 §7.3 已记录的既有配置漂移（`target_count 5 != 10`、follower 终端速度数值断言），不涉及本模块 |
| P2 九个 simulation 文件 | 104 passed | [P2 结果](../../var/_system/diagnostics/p3c-final-simulation-p2.log) |
| 已接受块的必要触发反事实（10 局、按真实 `target_count` 选窗） | 所有已接受块最大包络超出 30.174 mm（Panda）/4.206 mm（Marvin），均低于 40/10 mm 预算，不触发重定时；仅两个代表拒绝块触发（55.659/19.716 mm） | [soundness](../../var/_system/diagnostics/p3c-soundness-accepted-commands.log) |
| 两条真实请求的离线生产重放 | 原请求仍 `FOLLOWER_DEVIATION_EXCEEDED`；重定时请求接受。Panda 执行偏差 0、实际参考对原目标 55.659 mm、下一 anchor 0.062205 m/0.04 m/s；Marvin 执行偏差 0.000676 m、对原目标 19.735 mm、下一 anchor 0.061016 m/−0.001480 m/s | [production replay](../../var/_system/diagnostics/p3c-production-replay.log) |

`ruff` 仍不可用；已用 `python3 -m py_compile` 检查全部修改/新增 Python 文件并检查行宽（`chunk_stream.py:441` 的既有超宽行未改）。未使用 importorskip、删除负例或放宽断言。

### 10.5 小批闭环（8 次，独立于原 240 次）

固定 Bundle：Panda `stack-act-refined2`，Marvin `handover-act-refined-final`。Panda 使用诊断 `window20`（临时把 `panda_follower_v1.yaml` 的 `target_count` 由 5 改为 20，全部 6 次 Panda 运行一致，结束后已恢复为 5）；Marvin 使用现有 `target_count: 20`。两组均保留 P3-B 准入、相同布局/种子/评价/Follower 预算；基线使用 P3-C 之前的 `chunk_stream.py`，干预使用本阶段新文件；服务复用既有 backend/policy 镜像，namespace `/robot/diagnosis`、endpoint `learning:50071`。每行是一次启动尝试。

| 场景/seed/组 | 任务推进 | 重定时轴 | 首个失败原因 | 完整任务条件 |
| --- | --- | --- | --- | --- |
| Panda 20013 基线 | 17.666 s | 0 | `right_joint2` 偏差 0.321071 > 0.3 | 未满足 |
| Panda 20060 基线 | 11.636 s | 0 | `FOLLOWER_HORIZON_EXCEEDED`：Ruckig 3.186 s > 3 s | 未满足 |
| Panda 20057 基线 | 22.604 s | 0 | `right_joint2` 偏差 0.384467 > 0.3 | 未满足 |
| Marvin 5103 基线 | 11.586 s | 0 | `left_gripper_width` 偏差 0.020346 > 0.01 | 未满足 |
| Marvin 5103 干预 | 20.864 s | chunk 13、14 | 越过原 13.65 s 夹爪阻断；新首个失败为 `right_joint2` 偏差 0.051593 > 0.05 | 未满足；`handover_sequence.phase=left`，左独持未稳定 |
| Panda 20013 干预 | 25.056 s | 0 | 无失败；`SUCCEEDED` | **满足**（全部 stack 条件通过，稳定 2.0 s） |
| Panda 20060 干预 | 5.610 s | 0 | `right_joint2` 偏差 0.301572 > 0.3 | 未满足 |
| Panda 20057 干预 | 6.626 s | 0 | `right_joint2` 偏差 0.308601 > 0.3 | 未满足 |

Marvin 干预的两个重定时事件：chunk 13 `left_gripper_width` 原首点 0.079721→适配 0.058178 m（原超出 0.021544 > 0.01），chunk 14 原首点 0.049705→适配 0.061121 m（原超出 0.011416 > 0.01）；两者末点与选窗不变。详见 [Marvin 干预证据](../../var/_system/diagnostics/p3c-marvin-intervention-evidence.log) 与 [闭环汇总](../../var/_system/diagnostics/p3c-closed-loop-summary.log)。

解读边界：Panda 20013 干预的完整成功发生在 **0 次重定时**，且同种子基线在另一次运行失败；观测/推理的墙钟时序使同 seed 闭环不完全可复现，因此该成功不归因于 P3-C。P3-C 的闭环正证据只有 Marvin：越过原夹爪阻断并多推进约 9.3 s，但仍未完成任务。Panda 的 6 次运行中，**4 次**因 `right_joint2`、**1 次**因 horizon 被拒绝，**另 1 次成功且零适配**；它们均未复现 window20 诊断中的夹爪阻断。Panda 使用诊断 window20，该结果不能替代默认 `target_count=5` 的闭环验证。

### 10.6 文件、备份与恢复

实际修改：

- `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/chunk_stream.py`
- `tests/unit/test_policy_stream.py`
- `docs/experiments/2026-11-15-inference-repair-taskbook.md`、[文档索引](README.md)

新增：

- `shared/common/python/trajectory/src/robot_trajectory/transition_retiming.py`
- `tests/unit/test_transition_retiming.py`
- `tests/unit/test_follower_transition_retiming.py`
- `tests/unit/test_gripper_hold_aperture.py`

临时配置（已恢复）：`shared/scenarios/panda_stack/profiles/control/panda_follower_v1.yaml` 在闭环期间 `target_count: 5→20`，结束后按备份恢复为 5（sha256 `e22f4b…`）。

新增诊断证据（消费者为本任务书/复核，均位于 `var/`）：`var/_system/diagnostics/p3c-*.log` 共 8 个、`p3c-proto-input.json`（52 KB 的两次原始命令/配置抽取，供 production replay 复现）、`var/<scenario>/diagnostics/p3c-closed-loop/` 与 `var/<scenario>/diagnostics/batch-policy/p3c-*/report.json`。原 240 局、原 MCAP/task_result/closure audit、模型、数据集和 Bundle 未改写。

备份位于 `/tmp/sim_dual_arm-repair-P3C-20261115T085444Z-36072/original/`，按相对路径保存 `chunk_stream.py`、`test_policy_stream.py`、本任务书、`docs/experiments/README.md`、`env.sh` 与 `panda_follower_v1.yaml` 的前态，另有 `git-status-before.txt` 记录开始前用户修改。精确恢复：确认目标文件此后未被用户修改，再执行
`cp -p /tmp/sim_dual_arm-repair-P3C-20261115T085444Z-36072/original/<相对路径> /home/ljk/work/sim_dual_arm/<相对路径>`；
撤回新增文件时删除上述 4 个新源码/测试文件；本轮新增诊断目录在确认无后续复核后删除。不要用更早的 P1/P2/P3-B 备份恢复本轮文件。

容器：复用既有 `simulation:current`、`runtime`、`learning:current` 镜像，未 build/pull；backend/policy 进程已停止，runtime/simulation/learning 容器恢复为开始前的停止状态；`env.sh` 字节未变（sha256 `2ecc68b5…`）；容器内 `/tmp/p3cview`、`/tmp/p3c_proto.py`、`/tmp/p3c_production_replay.py`、`/tmp/test_hold.py` 已删除。无 Git 写操作。

### 10.7 未验证项与下一阶段

- **P3-C 未完成、P3 未通过。** 夹爪过渡的离线/单测/真实 Ruckig/真实 MuJoCo 证据成立，Marvin 闭环越过原阻断；但没有可归因于 P3-C 的完整任务成功，Panda 闭环未复现代表夹爪阻断。
- 新的主要阻断是 **臂轴过渡**：Panda 6 次中 4 次因 `right_joint2` 偏差 0.30–0.38 rad > 0.3 拒绝，1 次因 Ruckig 3.186 s > 3 s horizon 拒绝，另 1 次成功且没有触发适配。它们不属于夹爪重定时范围；是否需要对臂轴做同类重新定时、还是回到示教/训练（P4/P5），需另立范围并冻结配置后再验证，不能在本轮顺手放宽 0.3 rad 预算或删除 horizon 检查。
- 模型跨预测的夹爪相位不一致（同一未来时刻相差 19–43 mm）是 P4/P5 的明确输入：需检查示教中的夹爪过渡时间分布、动作时间对齐和训练选模，不应只靠执行侧重定时。
- 未运行完整 240 局重放、批量推理、训练/补采/真机；未验证所有历史录制在新机制下的服务端真实返回；`ruff` 未执行。
- 本阶段新增的 `follower.chunk_retimed` 事件与 `p3c-*` 诊断日志尚未被其他工具消费；若后续删除，需先确认本任务书不再引用。

### 10.8 P3-C 独立复核：待补修（2026-11-15）

**R3C-01：Policy 目标适配误用于 Teacher 执行路径。** `ContinuousStreamRunner` 同时供 `PolicyStreamRunner` 和 `TeacherStreamRunner` 使用；当前 `retiming_axes` 无条件包含两夹爪，`_request` 也无条件调用适配，所以 Teacher 原始目标也会在触发时被改写。本阶段要求保留 Teacher 行为，该边界尚未实现；Teacher 也没有 Policy NPZ 可作为其原始预测的来源证据。

最小复现使用当前挂载模块、真实 `TeacherTargetSource`/`TeacherStreamRunner`、原 Panda `teacher_collect.yaml` 和现有测试 IO/control。继承右夹爪 `q0=0.022205447806916776 m, v0=0.04 m/s`，原目标为 0.079865 m，第一选中点位于交接后 0.05 s。当前 request 中该点变为 **0.024205447806916774 m**，`retimed=True`，即使配方 `action_source=teacher`、runner kind 和 source 类型也均为 Teacher。

独立新增的范围回归在 `/tmp` 测试视图中失败；连同现有 `test_transition_retiming.py` 与 `test_policy_stream.py` 实际结果为 **1 failed、23 passed in 1.02 s**。[完整红测](../../var/_system/diagnostics/p3c-independent-review-runtime.log)记录了导入路径、来源和改写前后值。这不否定原 129/136 项测试执行记录，但说明其未覆盖该边界；没有证明原正式示教曾触发或旧数据被污染。

应按已有动作来源限定为 Policy 适配，Teacher 保持原始完整目标及原有拒绝行为。不要通过默认开关、重复来源字段、按任务名分支或改 Teacher 计划来掩盖问题。在此范围补修前，不进入依赖 Teacher 行为的 P4 示教诊断。

**R3C-02：反向切换测试使用了错误的交接相对时间。** `test_transition_retiming.py::test_retimes_a_reverse_switch_from_a_closing_velocity` 的循环用 `(index + 21) * 0.05` 计算包络，首点为 1.05 s；实际 `window.targets` 对应交接后 0.05–1.00 s。过宽的包络不能充分验证逐点边界。应使用真正的交接相对时间并保留独立数值断言；生产时间轴在这项复核中没有发现同样错误。

**说明边界。** 当前实现保持时间戳而改变位置值，是显式执行目标整形。它会把选中轴窗口内部路径改成朝末点的直线再逐点裁入宽松包络，因此不能声明原内部开合顺序保持；单点包络合法也不保证整段曲线可行，仍需后续真实 Ruckig、连续限位、制动与偏差守卫。原 8 次闭环只验证了所列配置，Panda 的 window20 不能替代恢复后默认 target_count=5 的闭环证据。

下一步仅实施 [P3-C 补修提示词](2026-11-15-stage3c-followup-prompt.md)，不扩展到臂轴、不增加新的闭环预算。本次独立复核没有改动生产源码或仓库测试；只更新任务书/索引，新增补修提示词和上述诊断日志。修改前文档及 env.sh 在 `/tmp/sim_dual_arm-P3C-review-0yp2jfz7/original/` 按相对路径保存；复原前确认没有后续修改。复现测试与运行器留在该临时目录，容器内测试视图已删除，runtime 恢复停止，simulation/learning 未启动，env.sh 已恢复原字节。无 build/pull、Git 写操作或新闭环。

### 10.9 P3-C 补修结果：Teacher 隔离与时间轴修正（2026-11-15）

**R3C-01 已修复。** 夹爪目标整形从 `ContinuousStreamRunner` 基类移出：基类 `_retime` 返回 `None`，只有 `PolicyStreamRunner._retime` 调用 `retime_action_window`。`PolicyStreamRunner` 在自己的 `__init__` 解析两个 Profile 夹爪轴；Teacher 不经过该入口。未增加来源字段、默认开关、包装层或按任务名/日志字符串判断；使用既有类入口而没有重读 `action_source` 字符串。Teacher 的完整 60 点、chunk identity/digest、门控和 backend 拒绝行为保持 P3-C 前状态。

红测使用真实 `TeacherTargetSource`/`TeacherStreamRunner`、Panda `teacher_collect.yaml` 和现有测试 IO/control，构造与 Policy 触发相同的输入（继承右夹爪 `q0=0.022205447806916776 m, v0=0.04 m/s`，原目标 0.079865 m，首选中点在交接后 0.05 s）。补修前实际为 **1 failed、23 passed**，失败断言显示提交目标被改成 `0.024205447806916774 m`；补修后同一回归 **24 passed**。日志：[红测](../../var/_system/diagnostics/p3c-followup-red-runtime.log)、[绿测](../../var/_system/diagnostics/p3c-followup-green-runtime-focused.log)。红测运行的是补修前工作树，绿测运行的是补修后工作树；两次导入均为容器内当前挂载源码。

回归覆盖：

| 验证 | 结果 |
| --- | --- |
| Teacher 完整 60 点逐值不变、无 `follower.chunk_retimed`、`chunk_digest`/命令身份/observation/anchor 不变 | passed |
| 同类 Policy 输入仍触发原适配，目标与事件数值保持 | passed（既有 `test_gripper_transition_is_retimed_before_submission`） |
| Teacher backend 拒绝传播（fake control 抛 `FOLLOWER_DEVIATION_EXCEEDED`）不被吞掉，且提交的仍是完整原目标 | passed |
| 纯数学 9 项（含修正后的反向切换时间轴） | passed |
| runtime 相关 11 文件 | **130 passed**（较补修前 129 增加 Teacher 回归 1 项） |
| 纯数学/真实 Ruckig/真实 MuJoCo 3 文件 | **14 passed**：纯数学 9、真实 Ruckig 4（两条原请求仍拒绝、显式适配请求接受、原始/执行偏差分离）、真实 MuJoCo 1（持物实测/参考差异） |
| 正常 Policy 请求 | 既有 8-chunk 测试继续通过，未触发时目标逐值不变 |

**R3C-02 已修复。** `test_retimes_a_reverse_switch_from_a_closing_velocity` 改用 `window.targets` 的交接相对时间（`0.05, 0.10, …, 1.00 s`），不再用源索引 `(index+21)*0.05`。新增独立手算边界：首点区间 `[0.060619432610975056, 0.061676860524460864]`、次点区间 `[0.058619432610975054, 0.062108008799460865]`；适配首点等于首点包络上界 `0.061676860524460864`，第二点等于次点上界 `0.062108008799460865`，第三点为直线值 `0.062273755412041734` 且严格位于该点包络内。生产时间轴和选窗规则未改。

**算法含义已明确。** 当前实现保持时间戳、改变选中位置值，并把窗口内部路径替换为朝末点直线再逐点裁入宽松包络；这是显式执行目标整形，不保留原窗口内部先开后合时序。原始模型目标偏差与适配执行目标偏差分别报告；单点包络内只是必要条件，真实 Ruckig、连续限位、制动和 Follower 偏差检查继续保留，失败继续传递。未改成功判据、限位、加速度、速度、偏差预算或继承 anchor。

**未验证项与边界。** 本次补修没有新闭环（预算为零），没有重跑默认 `target_count=5` 的闭环，没有训练/补采/改权重或正式数据，没有扩展臂轴或进入 P4–P7。原 8 次闭环与 Panda `window20` 证据仍按 §10.5 的边界解释；Panda 零适配成功不归因于本机制。补修通过只表示消除 Teacher 误适配和测试时间轴缺口，不代表 P3 总体验收通过，也不代表成功率改善。

**文件、备份与恢复。** 本轮修改：

- `runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/chunk_stream.py`
- `shared/common/python/trajectory/src/robot_trajectory/transition_retiming.py`（仅文档字符串）
- `tests/unit/test_policy_stream.py`
- `tests/unit/test_transition_retiming.py`
- 本任务书、[文档索引](README.md)、[P3-C 补修提示词](2026-11-15-stage3c-followup-prompt.md)（状态行）

未新增生产文件。备份位于 `/tmp/sim_dual_arm-repair-P3C-followup-20261115T102528Z-81257/original/`，按相对路径保存上述前态、`env.sh`、两份文档，另有 `git-status-before.txt` 记录开始前用户修改。恢复方法：确认目标文件此后未被用户修改，再逐文件
`cp -p /tmp/sim_dual_arm-repair-P3C-followup-20261115T102528Z-81257/original/<相对路径> /home/ljk/work/sim_dual_arm/<相对路径>`；
不要用更早的 P3-C/P3-B 备份恢复本轮，以免同时撤销已通过的补丁。本轮新增诊断日志为 `var/_system/diagnostics/p3c-followup-*.log`（红测 1 个、绿测 3 个），消费者为本任务书；原 P3-C 日志与闭环证据保留。

容器：复用既有 runtime/simulation 镜像，未 build/pull；测试视图位于容器 `/tmp/p3cview`，导入 `/app/ros/.../chunk_stream.py` 与 `/shared/.../transition_retiming.py` 当前挂载源码；结束后已删除视图并停止容器，`env.sh` 字节未变（sha256 `2ecc68b5…`）；无 Git 写操作。

### 10.10 补修后的独立复核（2026-11-15）

已检查当前 `chunk_stream.py`：基类 `_retime` 返回 None，Teacher 保留完整原目标；仅 Policy 子类执行既有整形，拒绝传播仍保留。反向切换回归已改为真正的交接相对时间，并保留首点/次点/第三点的独立数值断言。没有发现本次补修必须退回的实质问题。

在既有 runtime 镜像中独立执行 `test_transition_retiming.py` 和 `test_policy_stream.py`，导入当前挂载源码，结果 **24 passed in 0.98 s**，无失败或跳过。日志为 [独立补修复核](../../var/_system/diagnostics/p3c-followup-independent-review-runtime.log)。24 项与 §10.9 的回归有重叠，不相加为新的独立覆盖数量；本次没有重新执行全部 130 项、真实 Ruckig/MuJoCo 14 项或闭环。

补修可按限定范围验收，P3 总体仍未通过。对恢复记录保留来源区别：实施方说明两个测试文件的补修前态是首次编辑后重建的版本，属于重建备份；diff/校验值能核对当前内容与差异，不能独立证明最初编辑前字节来源。后续修改应在首次编辑前保存当前文件，不重复这种操作。

本次只更新本任务书/索引，新增 P3-D 提示词和上述日志，未修改生产源码、配置或仓库测试。修改前文档和 env.sh 保存在 `/tmp/sim_dual_arm-P3C-followup-review-q2gc1k7z/original/`，按相对路径恢复前须确认无后续修改。runtime 临时视图已删除、容器恢复停止；simulation/learning 未启动，env.sh 已按原字节恢复，无 build/pull、Git 写操作或新闭环。

## 11. P3-D 实施结果：臂轴与 horizon 阻断的离线归因（2026-11-15）

执行提示词见 [P3-D 提示词](2026-11-15-stage3d-blocker-analysis-prompt.md)。本节记录实际重放与归因；P3 总体仍未通过。下一阶段提示词见 [P3-E 提示词](2026-11-15-stage3e-anchor-replan-prompt.md)。

### 11.1 输入、身份与重放

从原 MCAP 的 `/runtime/commands`、`follower_references`、`observation_index` 和 `execution_feedback` 提取六条最后提交但未被接受的请求，使用各局 `resolved/runtime.json` 的冻结 Profile、控制约束、Follower 预算、`target_count=20`（Panda 原试验为 window20，不是当前默认 5）和真实 anchor q/v。每条拒绝取前序至少两块已接受请求作对照，另取 Panda 20013 干预成功局的 8 个接受块作正对照；Marvin clip002 的左夹爪拒绝只作既有机制校准，不计入六条。

按 session/request/observation 身份把 NPZ 的 `raw_act/admitted` 与原命令关联：六条阻断的 `admitted_vs_command` 差异为空，限制轴的 `raw_act` 与命令逐值相同（差 0），说明臂轴目标就是模型输出，P3-B/P3-C 没有改写它；P3-C 只可能在夹爪轴产生差异。

重放脚本 `/tmp/p3d_replay.py` 在既有 simulation 镜像中调用当前源码与 Ruckig 0.15.3；输入 JSON 在 `var/_system/diagnostics/p3d-inputs/`。六条拒绝的原因、轴和数值与 backend 记录逐字符一致：

| 样本 | 原拒绝 | 重放 |
| --- | --- | --- |
| Panda 基线 20013 | `right_joint2` 0.3210710970375711 > 0.3 | 完全一致 |
| Panda 基线 20060 | `FOLLOWER_HORIZON_EXCEEDED` 3186000000 ns | 完全一致 |
| Panda 基线 20057 | `right_joint2` 0.38446654679412956 > 0.3 | 完全一致 |
| Panda 干预 20060 | `right_joint2` 0.301572351328516 > 0.3 | 完全一致 |
| Panda 干预 20057 | `right_joint2` 0.30860054528156367 > 0.3 | 完全一致 |
| Marvin 干预 5103 | `right_joint2` 0.0515928782035361 > 0.05 | 完全一致 |

每局前序接受块（有录制参考的 6 块）和成功局 8 个接受块全部在预算内；每局有录制参考的 6 块及成功局有参考的 6 块，重放计划与录制 `FollowerReference` 的段数、段时长和 c0–c3 系数最大差均为 **0**。前序参考 anchor 与被拒绝命令 anchor 的位置/速度最大差 **0**；限制轴参考滞后约 1e-10。没有 anchor 数值不连续或继承速度错误。

### 11.2 逐请求证据

完整数据见 [逐请求摘要](../../var/_system/diagnostics/p3d-summary.log)、[重放 JSON](../../var/_system/diagnostics/p3d-replay-results.json) 和 [重放日志](../../var/_system/diagnostics/p3d-replay.log)。表中“包络超出”是在原继承 anchor、原预算下逐选中点的宽松可达包络超出最大值；“单轴偏差/时长”是保持同 anchor、同目标、同终端速度约束的 1-DOF Ruckig；“同步时长”是 16 轴完整曲线；“前缀”是固定 1.0 s 交接窗口的导出参考；“实测 lead”是从提交时刻最后 `CHUNK` 实测状态到原目标绝对时间的宽松包络诊断。

| 样本 | 限制轴 | anchor q / v | 包络超出 / 预算 | 单轴偏差 | 单轴/同步完整时长 | 前缀限位/制动 | 前缀超预算轴 | 前序 lag | 同刻预测变化 | 实测/接触 | 实测 lead 超出 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Panda 基线 20013 | `right_joint2` | 1.083238 / 0.511625 | 0.320768 / 0.3 | 0.321071 | 1.298 / 1.298 s | ok / ok | `right_joint2` 0.321071 | 2.3e-10 | −0.206785 | OUTSIDE_TOLERANCE；右夹爪 NONE（接触候选 source） | 0.001258，无轴超 |
| Panda 基线 20057 | `right_joint2` | 0.982635 / −0.648524 | 0.384256 / 0.3 | 0.384467 | 2.554 / 2.554 s | ok / ok | `right_joint2` 0.384467 | 6.7e-11 | +0.251885 | FOLLOWING；右夹爪 NONE | 0，无轴超 |
| Panda 基线 20060 | `right_joint2` | 0.995781 / −0.8 | 0.563099 / 0.3 | 0.563099 | 3.186 / 3.186 s | ok / ok | `right_joint2` 0.563099、`right_joint4` 0.462584 | −7.4e-11 | +0.405263 | CONSTRAINED；右夹爪 CONTACT source | 0.000791，无轴超 |
| Panda 干预 20060 | `right_joint2` | 1.059406 / 0.007582 | 0.301572 / 0.3 | 0.301572 | 1.092 / 1.092 s | ok / ok | `right_joint2` 0.301572 | −1.0e-10 | +0.303204 | FOLLOWING；右夹爪 POSITION | 0，无轴超 |
| Panda 干预 20057 | `right_joint2` | 1.261037 / 0.531147 | 0.308601 / 0.3 | 0.308601 | 1.162 / 1.162 s | ok / ok | `right_joint2` 0.308601 | −3.0e-10 | −0.242506 | FOLLOWING；右夹爪 POSITION | 0，无轴超 |
| Marvin 干预 5103 | `right_joint2` | −0.020646 / −0.144624 | 0.051367 / 0.05 | 0.051593 | 1.000 / 1.000 s | ok / ok | `right_joint2` 0.051593、`right_joint7` 0.054216 | −1.8e-10 | −0.056300 | OUTSIDE_TOLERANCE；左夹爪 NONE（接触候选 block） | 0，无轴超 |

所有样本的预测时域为 3.0 s（60×50 ms），非最终导出窗口为 1.0 s；表中“完整时长”是 Ruckig 曲线时长，不是导出/执行窗口。限制轴连续偏差峰值时刻（交接后）：Panda 20013 0.500 s、20057 0.350 s、horizon 0.400 s、干预 20060 0.050 s、干预 20057 0.250 s；Marvin 0.071 s。表中“实测/接触”取提交时刻最后一条 `CHUNK` 反馈（不是失败后的 `BRAKE` 行），只用于判断当时的进度/接触，不作为继承参考。

Marvin clip002 校准（左夹爪）：原拒绝 0.019727525391288196 > 0.01；包络超出 0.019716，单轴偏差 0.019727，限位/制动 ok；与原 P3-C 机制一致，不计入六条。

### 11.3 归因结论

1. **原继承 anchor 下六条均为 A。** 选中目标超出原预算下的宽松可达包络；同 anchor 的单轴 1-DOF Ruckig 也得到相同超预算偏差。B（存在可行参考但末点/同步规划未找到）不被支持：单轴完整时长与 16 轴同步完整时长在六条上分别完全相等（1.298/2.554/3.186/1.092/1.162/1.000 s），没有其他轴拖慢限制轴。
2. **3.186 s horizon 不是单纯契约误拒绝。** 完整曲线 3.186 s、预测时域 3.0 s、实际导出窗口 1.0 s；固定交接 1.0 s 前缀的位置/速度/加速度连续限位和制动均通过，但 `right_joint2` 0.563099、`right_joint4` 0.462584 超过 0.3。只删除/放宽 3 s 检查会先把拒绝改成偏差拒绝，不会使请求可执行；C（实际执行参考可行但仅受完整曲线时域限制）不被支持。若未来仍要改该契约，受影响者至少包括 `PreparedFollowerPlan`/`FollowerReference.curve_duration_ns`、ROS `FollowerReference` 消息、runtime/backend 校验、离线 audit 与相关测试；本轮不实施。
3. **anchor 数值继承不是原因。** 前序计划终点与拒绝 anchor 逐值一致，限制轴参考滞后约 1e-10；问题不是上一块没到达目标或速度继承算错。
4. **跨预测目标变化的归因保留不确定性。** 同一未来时刻的新目标相对上一预测改变 0.056–0.405 rad；四条（20013、20057、horizon、干预 20057）的首选中点更接近提交时刻实测状态而非旧计划终点（如 20057：目标 1.201158、实测 1.229476、旧 anchor 0.982635），另两条（干预 20060：目标 1.362607、实测 1.064235；Marvin：目标 −0.080494、实测 −0.001656）不接近。较新观测/接触可能支持合理反馈修正；没有固定输入对照和额外模型推理，不能单独判定为训练或数据错误，也不能排除模型时序误差。
5. **实测 lead 的决策解释已撤回。** P3-E 确认该组最后 CHUNK 行均处于原拒绝后的 STOPPING；其包络数值与时间表仅为后处理诊断，不能证明决策时可用状态可达或原承诺契约错误。原 anchor 的 A 证明和原拒绝重放不受影响；活动参考提前切换的独立运动学反事实见 §12，跨端方案仍待确认。
6. **没有证据支持直接放宽预算或改规划器。** 单轴可行不等于整条曲线可行，3 s 上限也不能单独删除；六条的固定前缀在真实 Ruckig 下仍违反偏差。生产 Profile、预算、限位、成功条件和夹爪整形范围均未改变。

### 11.4 最小离线原型

`/tmp/p3d_replay.py` 是离线原型：按原 anchor/目标/终端速度构造 Ruckig 曲线；为检查固定 1.0 s 前缀，只在原型进程内临时放宽 exporter 的模块级 3 s 上限（导出计划仍是合法 ≤3 s 的 `ReferencePlan`），随后用真实连续限位、Follower 偏差和 `plan_braking` 检查。结果如上：前缀限位/制动通过、偏差失败；3 s 完整曲线只被记录，不进入生产。

该原型没有构造“从实测状态真实规划”的候选（P3-D 要求候选保持原继承 q/v）；实测 lead 包络只是必要条件的诊断。候选是否通过真实 Ruckig/连续限位/制动/Follower 检查，留待 P3-E。

### 11.5 未通过条件与下一阶段

- 六条原请求在原契约下仍全部拒绝；P3-D 没有提高成功率，也没有闭环或新推理。
- 已确认的阻断是**原继承状态下目标超过原预算可达范围**。固定单块承诺与较新预测冲突是下一项假说，尚未证明原承诺错误；六条中四条的新目标更接近实测状态，两条不接近，也不足以直接认定模型或数据错误。
- P3-E 提示词：[P3-E 提示词](2026-11-15-stage3e-anchor-replan-prompt.md)。它只做离线候选和契约方案：优先验证从活动参考连续提前切换，以及从参考有界制动后再规划；实测 q/v 直接起步仅为理想化对照，缺少连续桥接不能算部署候选。分别检查额外前缀、切换可生效时间、原曲线偏差和选窗/承诺影响。原型多数通过不等于原契约错误，几个原型失败也不能自动移交训练。需要变更跨端含义时，在完整原型和消费者方案后等待确认，不实施。不得把夹爪直线整形扩展到臂轴。
- 未验证：默认 `target_count=5` 闭环（本批全部 window20）、真实实测-state Ruckig 候选、模型目标正确性的固定输入对照、horizon 契约在 anchor 修复后的重新评估、全部历史录制重放、真机。
- P3 总体仍未通过；P3-D 的离线归因完成不等于成功率改善或 P3 验收通过。

### 11.6 文件、备份与复原

本轮修改：

- `docs/experiments/2026-11-15-inference-repair-taskbook.md`、[文档索引](README.md)
- [P3-D 提示词](2026-11-15-stage3d-blocker-analysis-prompt.md) 状态行

新增：

- [P3-E 提示词](2026-11-15-stage3e-anchor-replan-prompt.md)
- `var/_system/diagnostics/p3d-inputs/*.json`（8 局、每局最后 8 命令/6 参考/600 反馈行；约 13 MB，非 MCAP 复制）
- `var/_system/diagnostics/p3d-replay.log`、`p3d-replay-results.json`、`p3d-summary.log`

原型与抽取脚本保留在 `/tmp/p3d_replay.py`、`/tmp/p3d_extract.py`，供复核；未新增生产源码、协议、配置或测试。

备份位于 `/tmp/sim_dual_arm-repair-P3D-20261115T104813Z-92523/original/`，按相对路径保存本任务书、索引、P3-D 提示词、`env.sh` 前态，另有 `git-status-before.txt` 记录开始前用户修改。恢复：确认目标文件此后未被用户修改，再逐文件
`cp -p /tmp/sim_dual_arm-repair-P3D-20261115T104813Z-92523/original/<相对路径> /home/ljk/work/sim_dual_arm/<相对路径>`；
新增的 P3-E 提示词和 `p3d-*` 诊断在确认无后续消费者后删除。不要用更早的 P3-C/P3-B 备份恢复本轮。

容器：复用既有 runtime/simulation 镜像，未 build/pull；测试/重放使用容器内当前挂载源码（`/app/src/...`、`/shared/...`），临时视图与脚本已删除；runtime/simulation 已恢复停止，learning 未启动；`env.sh` 字节未变（sha256 `2ecc68b5…`）。无 Git 写操作。

### 11.7 P3-D 证据与 P3-E 候选条件的独立复核（2026-11-15）

已检查逐请求摘要、重放 JSON 和现有 P3-E 提示词。原 anchor 下六条为 A、horizon 前缀仍超预算的数值记录保留；本次没有独立重跑真实 Ruckig，不能当作又一次精确重放通过。

从重放 JSON 的实际测量时间、handoff 和目标时间独立计算，得到下表。原首点相对原 handoff 均为 0.05 s，原选窗执行时长均为 1.0 s：

| 样本 | 测量至原 handoff 的额外前缀 | 测量至原首点 | 测量至原末点 |
| --- | --- | --- | --- |
| Panda 基线 20013 | 0.938 s | 0.988 s | 1.938 s |
| Panda 基线 20057 | 0.952 s | 1.002 s | 1.952 s |
| Panda 基线 20060（horizon） | 0.958 s | 1.008 s | 1.958 s |
| Panda 干预 20057 | 0.940 s | 0.990 s | 1.940 s |
| Panda 干预 20060 | 0.948 s | 0.998 s | 1.948 s |
| Marvin 干预 5103 | 0.942 s | 0.992 s | 1.942 s |

因此实测 lead 包络通过不能表述成“目标已证明物理可达”或“更接近 C”。它改变了起点和可用运动时间，还涉及原本执行中的前序计划。实测采样时刻不等于新计划可生效时刻；不能倒用采样时间或拒绝后 BRAKE 状态。所有测量字段（包括 `measured_at_handoff`）都须以其实际 time_ns 核验，不能按字段名推定未来状态已经观测到。

P3-E 提示词已收紧为三个离线对照：活动参考在可执行切换时刻的连续提前替换、从参考出发的有界制动/保持、实测 q/v 的理想化起点对照。后者若没有连续桥接，不得当作部署候选。位置/速度接缝、规划/接收延迟、已接受剩余计划的撤销与生效确认、旧 anchor/迟到响应、Teacher 隔离都需要具体说明。

提前切换还会改变 ActionWindow 的选窗；保留原20个绝对目标可用于隔离变量，但不能伪称现有60点请求的语义未变。候选必须按原 handoff/原 anchor 定义的期望曲线，在原窗口内计算连续偏差；若另使用新起点重新定义期望曲线，结果必须另列，不能通过改变首段插值降低比较标准。

删除了原提示词中“多数候选通过就认定契约修复、失败就移交训练”的预设结论。逐请求的可行、失败、未验证和被释放的约束分别报告，再推荐一个有证据的最小步骤。运动学通过也不等于碰撞/接触/跟踪及完整任务通过。

本次只修改本任务书、索引和 P3-E 提示词，未改生产源码、配置、测试或原重放数据，也未运行新模型推理、闭环或容器任务。修改前态按相对路径保存在 `/tmp/sim_dual_arm-P3D-review-ubsr93gg/original/`；确认无后续修改后逐文件恢复。没有 Git 写操作，P3-E 仍待执行。

## 12. P3-E 实施结果：提前切换、制动保持与实测起点候选的离线验证（2026-11-15）

执行提示词见 [P3-E 提示词](2026-11-15-stage3e-anchor-replan-prompt.md)。本节记录实际候选验证与契约方案；P3 总体仍未通过，离线候选可用不等于成功率提高。

### 12.1 输入、复现与两套偏差定义

输入为 `var/_system/diagnostics/p3d-inputs/*.json` 的冻结 Profile、控制约束、Follower 预算、真实 `selected_indices`、原目标、原 anchor、原交接与目标绝对时间。未启动闭环、模型推理、训练或补采，未改生产执行策略、配置、Profile、预算或协议。

原型 `/tmp/p3e_candidates.py` 在既有 simulation 镜像内用 Ruckig 0.15.3、2 ms `duration_discretization` 与当前源码重放；输出 [p3e-candidates.json](../../var/_system/diagnostics/p3e-candidates.json) 与 [p3e-candidates.log](../../var/_system/diagnostics/p3e-candidates.log)。六条原拒绝均由生产 `prepare_follower_plan` 复现，轴与数值逐字符一致；前序活动计划终点与拒绝 anchor 的位置/速度最大差均为 **0**。

两套偏差分别命名、分别报告：

- **own 偏差**：以候选 anchor 和候选窗口目标定义的分段线性期望曲线。这是生产实际检查的量。
- **原曲线偏差**：在原交接 H 至原窗口末端，用原 anchor q_H 与原 20 个选中目标定义的分段线性期望曲线，对候选参考计算连续最大偏差（含交接处和点间极值），按原预算逐轴比较。**仅通过 own 不等于通过原曲线。**

偏差实现自检：令切换时刻 S=H，原型输出的连续偏差与生产 `compute_follower_deviations` 逐轴一致（最大差 1.4e-16），三例限制轴 0.321071、0.563099、0.051593 与 P3-D 完全相同。因此下文所有原曲线数值与生产偏差同源同精度。

### 12.2 时间、可用信息与额外前缀

`S_min = ceil_2ms(提交估计 + 50 ms min_lead)`。提交/拒绝仿真时间由各局已接受块的 `accepted_sim_time_ns` 与 runtime `elapsed_ns` 线性拟合反推；实测行只有仿真时间戳，接收延迟未记录。

| 样本 | 观测 O | 提交估计 | 拒绝估计 | S_min | 撤销 H−S_min | 原交接 H | 首/末目标绝对时间 | 拟合残差 | RPC |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Panda 基线 20013 | 19.300 | 19.359 | 19.390 | 19.410 | 0.940 s | 20.350 | 20.400 / 21.350 | 7.3 ms | 25.9 ms |
| Panda 基线 20057 | 24.300 | 24.348 | 24.375 | 24.400 | 0.950 s | 25.350 | 25.400 / 26.350 | 19.6 ms | 27.7 ms |
| Panda 基线 20060 | 13.300 | 13.373 | 13.394 | 13.424 | 0.926 s | 14.350 | 14.400 / 15.350 | 8.2 ms | 29.0 ms |
| Panda 干预 20060 | 7.300 | 7.347 | 7.376 | 7.398 | 0.952 s | 8.350 | 8.400 / 9.350 | 7.5 ms | 24.9 ms |
| Panda 干预 20057 | 8.300 | 8.364 | 8.393 | 8.416 | 0.934 s | 9.350 | 9.400 / 10.350 | 6.4 ms | 26.4 ms |
| Marvin 干预 5103 | 21.300 | 21.373 | 21.405 | 21.424 | 0.926 s | 22.350 | 22.400 / 23.350 | 7.0 ms | 27.3 ms |

- 额外前缀 = H−S_min = **0.926–0.952 s**；首目标在 S_min 之后 0.976–1.002 s，末目标之后 1.926–1.952 s。原窗口只有 1.0 s，因此这段额外时间不是"同一时间起点只换 anchor"：它撤销一条已接受、正在执行的前序计划的大部分（见 12.3）。
- 不采用历史采样时刻零延迟接管：S_min 由提交时间 + `min_lead` 得到，规划耗时按记录的 RPC 实测值计入。按拒绝响应后再提交估计，S 再后移约 0.06–0.11 s；六条在整个提前区间内仍通过 own 检查，结论不依赖这 0.1 s。
- 每条拒绝前最后一条 `CHUNK` 实测行的 phase 均为 **STOPPING**（19.412 / 24.398 / 13.392 / 7.402 / 8.410 / 21.408），即拒绝触发的停止之后；P3-D 的"实测 lead"状态即来自这一行。本阶段不把它当决策输入，候选三另取拒绝前的 STARTED 行。

### 12.3 候选一：从活动参考在可执行切换时刻提前重规划

在 S 处对前序已接受计划采样 q/v 作为新 anchor，保留已执行前缀；用真实 Ruckig（同一 2 ms 网格、相同终端速度规则）规划到目标。两种窗口变体：

- **变体 a**：保留原 20 个绝对目标与绝对时间。它需要显式的选窗原点，**不是现有 60 点 StreamRequest 的合法原样返回**；首段插值为 S→原首目标（0.976–1.002 s）。
- **变体 b**：沿用现有"交接后前 20 点"选窗。窗口末端前移 950–1000 ms（选窗 1/2..20/21），原窗口只覆盖 0–50 ms，不能声称保留原曲线。

| 样本 | S_min 下生产接受 | 完整 Ruckig 时长 | own 最差（比值） | S_min 原曲线最差（比值） | own 与原曲线同时通过的 S 区间 |
| --- | --- | --- | --- | --- | --- |
| Panda 基线 20013 | 是 | 1940 ms | `right_joint7` 0.0896 / 0.4（0.224） | `right_joint2` 0.1945 / 0.3（0.648） | [19.410, 20.310] |
| Panda 基线 20057 | 是 | 1950 ms | `right_joint3` 0.0703 / 0.3（0.234） | `right_joint2` 0.1927 / 0.3（0.642） | [24.400, 25.220] |
| Panda 基线 20060 | 是 | 1926 ms | `right_joint3` 0.0715 / 0.3（0.238） | `right_joint2` 0.3191 / 0.3（**1.064**）、`right_joint4` 0.3106 / 0.3（**1.035**） | [13.720, 13.944] |
| Panda 干预 20060 | 是 | 1952 ms | `right_joint2` 0.1488 / 0.3（0.496） | `right_joint2` 0.1488 / 0.3（0.496） | [7.398, 8.318] |
| Panda 干预 20057 | 是 | 1934 ms | `right_joint2` 0.1675 / 0.3（0.558） | `right_joint2` 0.1675 / 0.3（0.558） | [8.416, 9.316] |
| Marvin 干预 5103 | 是 | 1926 ms | `right_joint2` 0.0472 / 0.05（0.945） | `right_joint7` 0.1024 / 0.05（**2.049**）、`right_joint2` 0.1002 / 0.05（**2.005**）、`right_joint3` 0.0966 / 0.05（**1.932**）、`right_joint4` 0.0651 / 0.05（**1.302**）、`right_joint5` 0.1194 / 0.1（**1.194**）、`right_joint1` 0.0584 / 0.05（**1.168**） | [21.948, 22.164] |

逐项结论：

1. **own 检查（生产口径）**：六条在 S_min 下全部被真实 Ruckig + 连续位置/速度/加速度限位 + 制动 + C1 接缝 + Follower 偏差接受，own 最差比值 0.224–0.945；最紧的是 Marvin 的 `right_joint2`（0.945），其余为 0.224–0.558。20060 的 3.186 s 全曲线在提前起点下变为 1.926 s：原 anchor 在 `right_joint2` 的 −0.8 rad/s 在 S_min 处只有 −0.074 rad/s，反向制动段消失；3 s 检查仍保留并由重试重新计算，未被绕过。
2. **原曲线检查（公平口径）**：四条在 S_min 下原曲线也在预算内（0.496–0.648）；**Panda 基线 20060 超预算 6.4%/3.5%**，**Marvin 六轴超预算 1.17–2.05 倍**。两者峰值都在原交接 H 附近：这是提前起点使参考在 H 时已比旧承诺 q_H 前进 0.319/0.311 rad（Panda）和 0.058–0.119 rad（Marvin）所致，H 之后对原目标折线的跟踪本身在预算内。
3. **两套标准同时可行的 S 存在**：20060 首次同时通过为 **S=13.720（2 ms 细化，own 0.2745、原曲线 0.9999）**，Marvin 为 **S=21.948（own 0.305、原曲线 0.995）**；两者都比 S_min 晚 0.30/0.52 s，但仍比 H 早 0.63/0.40 s。其余四条从 S_min 起即同时通过。也就是说，**"同时满足两套预算"需要按请求选择 S，而不是固定提前量或无条件最早切换**；20 ms 粗扫 + 2 ms 细化可覆盖（≤约 35 次 Ruckig/请求）。
4. **被撤销的前序计划**：活动块为 20013/20057/20060/干预 20060/干预 20057/Marvin 的第 18/23/12/6/7/20 块，起点为 H−1.0 s，已执行前缀仅 **0.048–0.074 s**，撤销 **0.926–0.952 s（92.6–95.2%）**。切换处 q/v 取自该活动计划在 S 的采样，位置/速度按构造与已执行参考完全一致；候选内部段接缝 C1 检查全部通过（最大 dq/dv ≤ 1e-9）。
5. **选窗**：变体 b 的尾部前移 0.95–1.0 s，且原曲线比较只覆盖 0–50 ms；变体 a 保留原 20 个绝对目标但需要显式选窗原点。两者都不是"现有 60 点请求语义未变"。
6. **跟踪/接触/碰撞/任务**：切换处记录跟踪质量为 CONSTRAINED（20013、20060）、FOLLOWING（干预 20060、干预 20057）、OUTSIDE_TOLERANCE（20057、Marvin）；20013 与 20060 的右夹爪为 CONTACT/source，Marvin 左夹爪为 NONE/block、右夹爪 POSITION。跟踪超时守卫与接触卸载门控未在原型中运行，只记录切换处的状态。**新参考下的真实跟踪、碰撞、持物与任务结果均未验证**；运动学通过不等于物理可执行。
7. **生产默认窗口补充检查**：同一六条预测若改用当前生产默认 `target_count=5`（选窗 21..25，窗口 0.25 s），仍有 **5/6 被拒绝**：20057 0.367777、20060 0.530692、干预 20060 0.301572、干预 20057 0.308601、Marvin 0.051593；只有 20013 被接受。拒绝不是 window20 特有，前 5 个选中点已含超预算目标。用候选一在 S_min 与 S_min+0.1 重规划时六条全部被生产检查接受（own 比值 0.019–0.188）；原曲线比值依次为 0.68/0.78/1.18–1.21/0.90/0.78/1.22–1.23，仍是 20060 与 Marvin 超预算，且短窗口使该比值比 20 点窗口更紧。S 选择必须按实际 `target_count` 重算。结果见 [p3e-default-window.json](../../var/_system/diagnostics/p3e-default-window.json) 与 [p3e-default-window-retry.json](../../var/_system/diagnostics/p3e-default-window-retry.json)。

### 12.4 候选二：有界制动、保持后再规划

从同一活动参考状态做真实恒定减速度制动，得到停车时刻与保持位置，再按原绝对目标规划。S_min 下活动块刚起步，参考速度只有 0.01–0.08 rad/s，制动仅 10.5–109.2 ms，保持 ≤2 ms，新计划起点为 S+0.012–0.110 s；六条新计划全部被生产检查接受，原曲线结论与候选一相同（20060 1.067、Marvin 2.021，其余 0.516–0.656）。制动段、保持段与新计划的接缝均为位置/速度连续，制动段本身通过连续限位。

随着 S 后移，参考速度增大，各样本最长制动升至 0.18–0.77 s（干预 20060 的参考速度始终很小，保持约 8 ms），部分样本的停车点越过首个目标（missed 1–14），或新计划 own 偏差超预算：20013 12/19、20057 13/19、20060 12/19、干预 20060 19/19、干预 20057 14/19、Marvin 15/19 通过。**候选二严格劣于候选一**：起点更晚、可用时间更少、额外引入一次停车，且对 20060 与 Marvin 的原曲线问题没有改善。它只在"必须显式停车"的语义下才有独立价值。

### 12.5 候选三：实测 q/v 理想化起点

取拒绝前 STARTED 实测行（不取 STOPPING 行）用真实 Ruckig 起步：

| 样本 | 实测时刻 M / phase | 生产接受 | own 最差（比值） | 与活动参考的最大不连续 dq / dv | 原曲线最差（比值） |
| --- | --- | --- | --- | --- | --- |
| Panda 基线 20013 | 19.400 STARTED | 是 | `right_gripper_width` 0.0189 / 0.04（0.472） | 0.0397 rad / 0.0044 rad·s⁻¹ | 0.639 |
| Panda 基线 20057 | 24.350 STARTED | 是 | `right_joint3` 0.0700 / 0.3（0.233） | 0.0095 / 0.0071 | 0.651 |
| Panda 基线 20060 | 13.350 STARTED | 是 | `right_gripper_width` 0.0189 / 0.04（0.473） | 0.0398 / 0.0031 | 1.071 |
| Panda 干预 20060 | 7.400 STARTED | 是 | `right_joint2` 0.1446 / 0.3（0.482） | 0.0093 / 0.0015 | 0.501 |
| Panda 干预 20057 | 8.400 STARTED | 是 | `right_joint2` 0.1710 / 0.3（0.570） | 0.0087 / 0.0044 | 0.570 |
| Marvin 干预 5103 | 21.400 STARTED | 是 | `right_joint2` 0.0486 / 0.05（0.971） | 0.0050 / 0.0062 | 2.078 |

- 实测速度全部在关节限位内，无需裁剪，未做裁剪。
- **参考连续性未通过**：实测状态与活动参考在 M 处相差 0.0050–0.0398 rad 和 0.0015–0.0071 rad·s⁻¹；直接用实测 q/v 起步会让参考在 M 跳变。本阶段没有构造"有界时长、满足限位"的连续桥接，也没有给出其可生效时间，因此**候选三只是理想化上界，不是部署候选**。
- 更早的拒绝前实测行也不可用：干预 20060 在 7.300 的 `right_gripper_width` 为 0.08000023（硬上限 0.080，超出容差），Marvin 在 21.350 的实测起点 own 偏差 0.050615 > 0.05；均按原值报告，未裁剪。
- 最后一条 `CHUNK`（STOPPING）行在六条上只会得到相近数值；它属于拒绝后的停止状态，不能作为决策输入。

### 12.6 逐样本结论、释放的约束与未验证项

- 六条原请求在**原继承 anchor** 下仍全部拒绝，P3-D 的 A 归类保留；候选一/二/三的通过都是"改变切换时刻与 anchor 来源"后的离线局部结果，**不构成原契约错误的证明**，也不能按"多数通过"改写原归类。
- 候选一在 S_min 下六条全部通过生产检查；按同时满足 own 与原曲线的标准，四条立即通过，Panda 基线 20060 与 Marvin 干预 5103 分别需要 S≥13.720 与 S≥21.948。候选二被候选一支配。候选三因缺少连续桥接不可部署。
- **本批离线候选中候选一增加的运动段最少**：候选二额外停车，当前指标未显示收益；候选三缺少连续桥接。尚未证明候选一可部署或在碰撞、接触、跟踪等未验证指标上全局更优。
- **被释放的约束**（仅在这三个原型中）：`successor.start == active.end`；`anchor.reference == active plan endpoint`；交接后前 20 点的自动选窗（变体 a）；"每个活动块只有一个固定后继、拒绝即终止"。**未被释放**：0.3/0.05 rad 预算、0.04/0.01 m 夹爪预算、0.8 rad/s 与 1.0 rad/s² 限位、3 s `curve_duration_ns`/`MAX_HORIZON_NS`、Follower 配置、成功判据、P2 任务/接触门控。
- **未验证**：新参考下的 MuJoCo 跟踪质量、碰撞、持物/接触维持、任务成功率；撤销 92.6–95.2% 活动块对真实轨迹与接触的实际影响；真实接收/规划延迟（未记录）；默认 `target_count=5` 的闭环（本轮只离线重放了六条预测的 5 点窗口，5/6 仍拒绝，未跑闭环）；真机。R2 拟保持原窗口末端，但这只说明末端绝对时刻相同，不能证明后继观测、预测节奏和可用 horizon 不变；这些仍需验证。

### 12.7 契约方案（提议，不实施）

推荐继续准备失败路径候选 R2：在有效活动 Policy 流的数学规划拒绝处提供一次替换机会，从活动参考在 S 的采样连续接管，保留原绝对目标与原窗口末端。它尚不是已实现或已批准的生产协议，数值候选通过也不证明碰撞、接触和跟踪可行。

最新推荐语义、字段消费者和红测规格集中在 §13：backend 原子切换及可查询确认；同 token 同内容在排期/生效阶段幂等；选窗原点与实际切换时刻分开；own 与原曲线预算均作硬门。没有合法 S 或超时则停止，不退化到允许原曲线超预算的 R1。

S 必须按实际窗口和实际决策/规划时钟重算。20 ms 粗扫、2 ms 细化是一种搜索候选，不保证最早解或所有窄区间都被找到，也没有固定35次调用的普遍上界；搜索耗时和接受截止必须实测、受限并在受理前再次检查。150 ms、5% 裕量和单独的 1e-9 容差均不是本轮已冻结的参数，不能擅自作为第二套阈值；物理/预算边界使用既有权威来源。

四项跨端语义仍待用户确认；可先准备可审查草案及红测规格，不修改生产源码、协议、配置或 SDK。

### 12.8 验收与唯一推荐的下一小步

R2 的验收至少包括：六条冻结请求的红测（原拒绝复现、S 扫描选点、own/原曲线数值、anchor 来源与终止状态）；anchor 不匹配、`now > S − min_lead`、重复 token、替换后旧 anchor 后继、已有 pending 冲突、找不到 S 的失败路径；`selection_origin_ns` 默认值下普通请求的选窗逐值不变；3 s 检查在重试中仍生效；现有 runtime/backend/contracts 绿测不变；真实 Ruckig 六条重放与一次 MuJoCo 跟踪回归；六条 MCAP 只读重放复核拒绝→暂定→重试时序。任何一项都不提高预算或扩大限位。

**唯一推荐的下一小步：P3-F = 契约语义确认与离线红测冻结（不实施生产）。** 先向用户呈现 §13 的具体草案与红测规格，一次确认四项语义后再冻结并执行离线红测；不把未确认的草案写成既定契约；不新增闭环、推理、训练、补采，不改生产源码/配置/协议。通过后才进入实现阶段。P3 总体仍未通过。

P3-F 提示词：[P3-F 提示词](2026-11-15-stage3f-contract-freeze-prompt.md)。

### 12.9 文件、备份与复原

本轮修改：`docs/experiments/2026-11-15-inference-repair-taskbook.md`（本节）、[文档索引](README.md)、[P3-E 提示词](2026-11-15-stage3e-anchor-replan-prompt.md) 状态行。

新增：[P3-F 提示词](2026-11-15-stage3f-contract-freeze-prompt.md)、`var/_system/diagnostics/p3e-candidates.json`、`p3e-candidates.log`、`p3e-default-window.json`、`p3e-default-window-retry.json`（逐请求候选、时间假设、检查明细与生产默认窗口补充检查，非 MCAP 复制）。

原型保留在 `/tmp/p3e_candidates.py`、`/tmp/p3e_selftest.py`、`/tmp/p3e_refine.py`、`/tmp/p3e_default_window.py`、`/tmp/p3e_default_retry.py`，供复核；未新增生产源码、协议、配置或测试。

备份位于 `/tmp/sim_dual_arm-repair-P3E-20261115T115416Z-121320/original/`，按相对路径保存本任务书、索引与 P3-E 提示词前态。恢复：确认目标文件此后未被用户修改，再逐文件
`cp -p /tmp/sim_dual_arm-repair-P3E-20261115T115416Z-121320/original/<相对路径> /home/ljk/work/sim_dual_arm/<相对路径>`；
新增的 P3-F 提示词与 `p3e-*` 诊断在确认无后续消费者后删除。不要用更早的 P3-D/P3-C 备份恢复本轮。

容器：复用既有 simulation 镜像，未 build/pull；以 `docker start` 启动已存在容器做只读重放，未改挂载；结束后恢复为停止状态（`sim-dual-arm-simulation-1` Exited；runtime/learning 未启动）。临时脚本通过 `docker cp` 放入容器 `/tmp`，未改容器外状态；宿主脚本保留在 `/tmp`。无 Git 写操作。

### 12.10 P3-E 独立文档与结果复核（2026-11-15）

本次检查既有候选 JSON、默认窗口对照、日志与控制端查询/重发实现，没有重新执行 Ruckig 或 MuJoCo。JSON 中六条原请求均 `accepted=false` 且 `reproduces_recorded=true`，六个最早候选的 variant_a 均通过其规划检查；默认窗口对照为 20点 6拒绝、5点 5拒绝/1接受。这里只复核结果记录，不算独立重跑通过。

所谓候选“生产接受”是离线规划/校验函数接受，不是现有跨端 SubmitFollower 替换请求已经被受理或生效。

**撤回 P3-D 实测 lead 的决策解释。** 六条 `timing.last_chunk_feedback.phase` 均为 STOPPING，属于原拒绝触发停止之后；§11 中该组实测 lead 数值、时间表只能保留为后处理诊断，不能作为决策时刻可用状态或“原目标在提前量内可达”的证据。它不影响原 anchor 的 A 证明和拒绝重放。P3-E 从活动参考提前切换的数值候选是另一组证据，不能借旧 STOPPING 行补充在线可用信息。

**R2 仍需补齐协议约束。** 事件送达不能决定 backend 是否已经切换；必须有唯一、可幂等查询的 backend 状态。同 token 同内容在排期后也应返回已有阶段，不能把 ACK 漏收后的正常重发变成终止性顺序错误。扫描中的时钟推进、窄可行区间、默认5点的原曲线硬门、任务/安全停止优先等均已纳入 §13 草案。原 S_min 是拟合提交时间+50ms 的离线假设，不等于拒绝响应和实际搜索完成后的生效下界。

尚未验证真实时延、切换时跟踪、碰撞/接触或任务结果，不能称为可部署方案。已有记录中的 `docker start` 绕过根 env.sh，启动方式与 RULES §2.1 不一致；后续按既有入口管理环境，本次不展开 P6 环境改造。

本次只修订任务书、索引和 P3-F 提示词，不改生产/协议/配置/测试或既有 JSON。修改前态在 `/tmp/sim_dual_arm-P3E-review-bimynoo_/original/` 按相对路径保存；确认无后续修改后逐文件恢复。未启动容器、推理、闭环、训练或采集，无 Git 写操作。

## 13. P3-F 已冻结：R2 替换契约与离线验证（2026-11-15）

**状态：R2-F1 离线规范已冻结；生产实现未开始，P3 总体未通过。** 用户对四项语义的实际答复为：**“按推荐 R2 确认四项”**。此次授权只覆盖离线规范、状态模型、现有接口红测和冻结输入数学重放，不包含生产源码/配置/ROS/SDK 变更，也不增加闭环、模型推理、训练或采集预算。本节是唯一规范正文，提示词和 diagnostics 引用本节，不维护另一份参数或可编辑契约。

### 13.1 已确认的四项语义

| 项 | 冻结选择 |
| --- | --- |
| 一次受限恢复 | 仅有效活动 Policy 流的 `FOLLOWER_DEVIATION_EXCEEDED` / `FOLLOWER_HORIZON_EXCEEDED` 可以产生一次 `CHUNK_REPLAN_REQUIRED`，保留原 reason。每个原失败请求最多一次替换尝试，失败不递归恢复；其他错误、停止、安全、租约与上下文约束照旧，不恢复 STOPPING 流。 |
| 原子切换与确认 | 旧参考执行至 S；backend 在 S 原子撤销旧尾部并启用新参考。backend 查询结果为权威，`CHUNK_REVOKED` 为通知。相同 token/内容在已排期和已生效阶段均幂等，不重复排期；ACK/事件漏收时查询，不猜测轨迹。 |
| 选窗原点 | `selection_origin_ns`：普通请求等于 handoff；替换请求等于原 H，实际 handoff 为 S。原选中点的位置、绝对时间、target_count 和末端不变。同步更新全部消费者，不留旧接口兼容分支或第二份可编辑目标数组。 |
| 双预算硬门 | own 与原 H/原 anchor 定义的原曲线连续偏差均满足原逐轴预算；3 s、硬限位、速度、加速度、制动、成功条件与 P2/Teacher 隔离保持。找不到合法 S 或超时就走原停止路径，不退回 R1，不提高额度。 |

本轮最小实现规格限定为**已有活动块、没有已接受 pending、非最终的 STREAM_NEXT**。STREAM_FIRST、最终块和 Teacher 仍沿原路径，后续不得借此扩大恢复范围。Policy 资格在 runtime 使用既有 `PolicyStreamRunner` 入口，在 backend 使用 owner 持有的、当前任务准备事实 `prepare_kind=evaluation_only` 与运行状态共同判定；不根据任务名、command_id 文本或事件日志识别来源，不新增请求来源开关。任务 epoch 与控制 generation/lease 必须同时有效。

### 13.2 时间、目标与两套预算的定义

记原观测时刻为 O、原 handoff 为 H、原窗口末端为 T，活动块序号为 n−1，失败请求拟用序号为 n。

- 普通窗口选择 `O + source_time > handoff` 后的前 k 点。替换窗口选择 `O + source_time > selection_origin` 后的前 k 点，其中 origin=H、handoff=S，且 O≤S<H<T。执行相对时间为 `O + source_time − S`。T 由选中末点推导，不新增可编辑末端字段。
- 替换复用**首次失败的已提交请求**中的完整目标数组、O、observation_id、k、joint order、context/profile、lease、原 accept_before 和非最终标志。它不重新推理、融合或准入，也不再次调用 P3-C 整形。首次请求可能已经包含既有 P3-C 的夹爪整形；“不改目标”是相对该提交请求，不是把执行目标冒称为 NPZ raw 输出。
- 起点为该活动参考在 S 的 q/v，不能用实测开度或 STOPPING 后的反馈代替。按现有 SSOT 容差验证 C1；未承诺跨块加速度/有限 jerk 连续。
- **own 门**：新参考在 [S,T] 对从 q(S) 到原选中目标定义的期望折线计算生产连续偏差。
- **原曲线门**：同一新参考在 [H,T] 对原 q(H)/原选中目标定义的期望折线计算生产连续偏差。它不是两条 Ruckig 参考的差，也不从 S 偷换原曲线起点。[S,H) 仍须通过 own、连续运动限位和制动检查。
- 两者逐轴使用同一原预算和 `_DEVIATION_ATOL`。完整 Ruckig 时长仍须≤3 s；不能只验证导出前缀后绕过该检查。
- 在 S 生效后，序号 n 的新参考实际有效到 T；下一 anchor 为该新参考在 T 的 q/v，下一块为 n+1。原 H anchor 及被撤销的旧尾部不再能作为后继依据。

### 13.3 规划归属、字段与全部消费者

**只在 backend 既有规划线程搜索并缓存一个候选，物理 owner 负责最终受理与切换。** runtime 不增加第二个 Ruckig，不在物理 tick 扫描。流程为原失败→backend 有界搜索→返回 offer→runtime 确认同一候选→owner 排期→S 原子生效。原失败请求未被接受；`CHUNK_REPLAN_REQUIRED` 不是任务成功或块接受。

冻结的新增信息如下。Python 为严格类型；纳秒使用现有非负 int64 范围，身份与 token 使用现有标识长度约束。ROS 使用当前接口直接演进；optional 用**零或一个元素的有界序列**表达，不新增 `has_*` 与对象内容互相校验的双重表示。此处只是规格，本轮未编辑 IDL。

| 权威信息及类型 | 生产者 | 消费者与约束 |
| --- | --- | --- |
| `StreamRequest.selection_origin_ns: int`，流模式必填，MOTION 明确为0 | runtime 请求构造 | `StreamRequest.window`、`ActionWindow`、ROS转换、backend规划/受理、FollowerReference及离线审计。普通流显式填handoff，替换填原H；不静默推导默认值兼容旧请求。 |
| `StreamRequest.replaces: str \| None`，只携带 offer token | runtime 确认请求 | backend恢复记录绑定原失败请求、活动参考与epoch；StreamCommand、FollowerReference.request及录制同步。普通/Teacher为None。身份从token绑定记录和既有anchor获取，不再复制一套source/seq/S。 |
| `ReplanStatus.token: str`、`original_observation_id: int`、`anchor: ChunkAnchor`、`expires_at_ns: int`、`original_reason: str`、`phase` | backend owner/规划线程 | SubmitFollower 的可选 replan 返回值、QueryStreamCommand 的可选 replan 结果、runtime及录制。phase仅为OFFERED/SCHEDULED/EFFECTIVE/ABANDONED。anchor已经包含S、stream/predecessor/profile和q/v。 |
| `ReplanStatus.abandon_reason: str \| None` | backend owner | 仅ABANDONED有值；原reason不被覆盖。EFFECTIVE的实际生效时刻严格为anchor.S，可由阶段推导，不再维护另一份可编辑时间。 |
| `QueryStreamCommand.replan_token: str \| None` | runtime | 复用现有查询，None保留普通查询；有token时在同context/command下返回该恢复状态和现有execution。未知/过期结果不得推测为未切换。 |
| `CHUNK_REVOKED` 撤销事件 | backend owner，S tick | 既有事件/录制/离线审计；指向旧chunk，时刻S。新chunk STARTED与撤销由同一次owner操作产生，查询状态独立于通知。 |

`ReplanStatus` 的外层响应沿用现有 context/command_id，旧请求身份由该外层、原 observation_id、anchor.predecessor_seq+1 和 backend 保存的不可变请求确定；token 在当前控制上下文内唯一即可，**不使用新增哈希、签名或防篡改清单**。原请求、选中索引、T、双预算向量均从唯一输入推导，不再保存第二份可编辑配置。

**沿用既有流身份规则。** 当前一个流的 command_id/command_seq 固定，chunk_seq由anchor派生。替换保持该身份及原拟用chunk_seq=n；原请求从未接受，不消耗n。不能擅自增加command_seq或换command_id启动新流。token只允许对这一未接受槽位确认一次；已经接受的普通块不能通过该入口改写。相同原请求在OFFERED/SCHEDULED/EFFECTIVE阶段重发，应返回该请求现有恢复状态，不能在已经排期后先走普通payload冲突检查；相同token重发确认也不得产生新副作用。

需一次同步的读取端：共享契约/校验/ROS编码、SubmitFollower/QueryStreamCommand消息、backend planner与owner/control/history、runtime控制客户端及Policy runner、FollowerReference请求校验、录制命令与事件解码、离线follower时间线/闭环审计、对应测试和生成SDK。Teacher只随通用编码显式提供普通origin，不获得恢复或整形策略。学习RPC/模型张量/数据/Bundle不扩展。

### 13.4 生命周期、时限与失败优先级

1. **资格与原错误。** 在抛出原错误并停止之前，由owner确认有效的活动Policy、当前任务、lease/context、无pending和上述失败允许集合。数学失败保留原reason，原请求不写成ACCEPTED。不存在资格或候选时，维持原停止/制动行为。
2. **有界搜索。** owner采集不可变活动参考/请求与时钟快照，规划线程在仍可生效区间内搜索；每次计算前后重新读取时钟和取消条件。S取实际2 ms物理网格，严格早于H，不早于当前now+已有min_lead；obs freshness、原accept_before、episode/task/墙钟deadline均约束结果。
3. **搜索策略。** 20 ms粗扫→命中附近2 ms细化；粗扫未命中且仍有预算时覆盖剩余2 ms点，避免把窄可行区间漏检写成无解。耗时截止由已有请求/规划超时、accept_before、观测有效期和S提前量推导；不另设150 ms、固定35次调用或5%阈值。数值成功但计算结束时已迟到的候选必须丢弃。
4. **OFFERED。** backend仅保留一个有界候选。offer有效上界为 `min(原accept_before, O+max_observation_age, S−min_lead)`，仍受既有墙钟/租约/任务期限约束。runtime只在期限内确认同一候选，不改S、不重新整形、不申请第二次搜索。offer超时由owner按原失败收束，不能无限继续旧块。
5. **SCHEDULED。** owner在确认时重读now、活动身份/epoch、任务状态、lease、pending、目标/原点/anchor和双门结果。该候选与所有约束未变才占用pending；缓存只避免重复Ruckig，不免除实时校验。相同token/完整内容返回已有阶段，不再次排期。不同内容或未知token拒绝，不覆盖合法pending；runtime自己的确认失败按既有停止路径收束。
6. **EFFECTIVE。** 在S的owner tick，先处理安全/任务/lease停止，再一次性撤销旧尾部、安装新活动参考、释放pending、冻结状态。错过S不能回填成已切换。旧FollowerReference和已执行段不改写，审计按旧有效区间`[old_start,S)`、新有效区间`[S,T]`重建唯一参考。正常撤销不是任务FAILED。
7. **结果不明。** ACK丢失或事件丢失时，用相同token查询/原内容幂等提交，返回已有OFFERED/SCHEDULED/EFFECTIVE/ABANDONED。没有ACK不意味着旧参考仍执行。不能确认则使用既有未知结果/停止处理，绝不新建token或盲发下一块。生效后的停止不抹掉历史EFFECTIVE事实。
8. **ABANDONED。** 无候选、搜索预算耗尽、迟到、有效状态改变或合法确认的硬门失败，均只走原停止/制动路径，保留原reason和放弃原因；不递归恢复。不同token/内容的错误包不得覆盖已经接受的pending。历史状态按现有有界命令保留策略清理，不新建永久回执数据库。

**后继观测规则也需实现验证。** runtime先以查询确认序号n已生效，再获得以T为起点、predecessor=n的后继anchor，然后等待observation_id递增的新观测。不得多推进chunk计数/观测序号或沿用旧anchor。仍要求`O_next<T`、60点中按普通origin=T有足够k点、观测未过期且受理提前量足够；否则如实停止。本轮只证明T不变，没有新观测/推理证据，**不宣称预测节奏与可用horizon已经验证**。

### 13.5 实际离线测试：生产红测、模型自测分开记录

| 证据 | 本轮实际结果 | 能说明什么 |
| --- | --- | --- |
| 现有生产接口红测与生产数学对照 | **1 failed / 13 passed**，预期exit=1 | 新selection_origin字段被当前ActionWindow以`Extra inputs are not permitted`拒绝。其中2项普通行为对照：boundary仍返回INVALID_CHUNK_START、原3.186 s horizon拒绝逐字符保留；另11项重算候选并调用真实plan_braking，通过连续制动检查。没有把普通入口提前放行当作修复。 |
| 离线R2状态模型 | **29 passed** | 原请求/确认重复、排期ACK漏收、S事件漏收后查询、token内容冲突、迟到、双门、pending冲突、停止优先、context变化、旧anchor/原T后继、漏过S、窄区间和预算耗尽；另对真实输入5/20点普通选窗作对照。 |
| 真实Ruckig数学重放 | **12组：20点6条原拒绝；5点5拒绝/1接受** | 原拒绝保持；对11条失败路径分别搜索，均找到双门候选。原已接受的5点20013不产生offer。没有构造服务请求接受或历史替换事件。 |
| 尚未实现的跨端测试 | **未执行，不算绿测** | 真实RPC/owner调度、ID/上下文/lease绑定、真实丢包重发、S原子切换、引用撤销审计、下一观测/horizon、消息生成及SDK两端type support。状态模型不替代这些测试。 |

状态模型只是可执行规格，不被生产导入；`task_proof=unchanged`是模型断言，不是本轮重新跑过P2真实接触测试。真实Ruckig调用生产`prepare_follower_plan`，连续限位/制动/own/3 s门由生产执行；原曲线门对精确裁切到[H,T]的三次多项式调用生产`compute_follower_deviations`，没有镜像包、放宽导出上限或第二份连续极值实现。

### 13.6 按窗口分别重算的候选与计算时间

六个样本均来自原window20活动参考。5点是**保留同一活动参考/anchor、只替换本次新预测的选点数**的反事实，不是从头按默认5点运行的证据。表中比值为每轴偏差/原预算的最大值，≤1为通过；原单位完整保存在JSON（臂rad、夹爪m）。S及耗时受本机运行和估计时钟影响，不是唯一或最早解。

| 原样本 | 20点S(s) | own/原曲线最大比值 | 搜索耗时(ms) | 5点S(s) | own/原曲线最大比值 | 搜索耗时(ms) |
| --- | --- | --- | --- | --- | --- | --- |
| Panda基线20013 | 19.520 | 0.223136 / 0.631609 | 42.11 | 普通接受，不offer | — | — |
| Panda基线20057 | 24.638 | 0.253605 / 0.632713 | 283.27 | 24.538 | 0.041539 / 0.778067 | 82.39 |
| Panda基线20060 | 13.720 | 0.274492 / 0.999936 | 282.58 | 13.800 | 0.245055 / 0.999189 | 229.12 |
| Panda干预20060 | 7.506 | 0.527295 / 0.527295 | 43.71 | 7.506 | 0.191415 / 0.897960 | 43.71 |
| Panda干预20057 | 8.582 | 0.562127 / 0.562127 | 163.02 | 8.542 | 0.075021 / 0.778120 | 81.35 |
| Marvin干预5103 | 21.948 | 0.305303 / 0.994861 | 499.33 | 21.948 | 0.270678 / 0.996820 | 457.17 |

11个候选完整曲线时长0.652–1.844 s，位置接缝最大误差约4.01e-21（逐轴单位rad或m），速度约6.33e-11（rad/s或m/s），均在现有1e-9容差内，导出末端严格为各自原T。20060等原曲线比值接近1，不能把小样本通过推成有余量的普遍保证。

**时钟证据限制。** 每次Ruckig/连续检查的真实墙钟时间被计入搜索；候选区间由原失败事件的sim/wall拟合及最大残差估计当前时刻，再叠加本轮计算耗时，并受MCAP内原accept_before限制。本次额外以现有min_lead=50 ms留作确认往返的诊断预留；它是离线假设，**没有证明真实ROS往返耗时≤50 ms**，不是新生产参数。本次所有结果满足该估计模型，offer剩余约50.25–231.43 ms；导入、录包解码、配置加载不计入搜索耗时。不能据此宣称在线时延已验收。实际实现必须使用真实时钟和原配置截止，超时允许停止。

### 13.7 本轮发现与未实现边界

- 当前`ActionWindow`将选点与handoff绑定；把H改成S会改变选中点。新字段缺失已由生产接口红测确认。
- 普通`boundary_rejection`只允许活动末端交接，拒绝提前S是当前契约的正确行为。实现时需要token绑定的专门替换路径，不能删除普通边界守卫。
- 当前查询只返回整体execution，不能确认某个替换token的排期/生效状态；现有客户端把非accepted的确定reason抛出。两端都需同步处理暂定状态，不能只改backend回包。
- 当前流command_id/command_seq固定；擅自“重试换身份”会破坏现有流一致性。原请求重发在SCHEDULED/EFFECTIVE时须先命中恢复记录，再走普通已接受块的payload比较。
- P3-C若对替换重试再次整形，会违反原目标不变；本契约禁止该路径。没有将此潜在集成错误写成当前生产已发生的bug。
- 未发现本阶段必须新增的哈希、签名、证明清单、通用恢复框架或长期中间产物。新增脚本/JSON/日志只有离线复核和后续回归消费者，归diagnostics；不复制MCAP/NPZ，不修改历史240局。

### 13.8 文件、复原与下一阶段

修改：本任务书、文档索引、P3-F提示词状态。新增：

- [P3-G 实施提示词](2026-11-15-stage3g-replan-implementation-prompt.md)；仅供下一次明确实施指令使用，本轮未执行。
- `var/_system/diagnostics/p3f_contract_model.py`、`test_p3f_contract_model.py`、`p3f_probe.py`、`test_p3f_production_red.py`。
- `var/_system/diagnostics/p3f-candidates.json`、`p3f-probe.log`、`p3f-model.log`、`p3f-red.log`。

修改前态在 `/tmp/sim_dual_arm-repair-P3F-bxdi0ya5/original/` 按相对路径保存。恢复前三个既有文档时，先确认没有后续修改，再逐文件`cp -p <original>/<相对路径> /home/ljk/work/sim_dual_arm/<相对路径>`；新增文件确认无后续消费者后删除。不要用P3-E或更早备份覆盖本轮前已有补丁。临时启动脚本也保存在该/tmp目录；不生成新哈希/证明清单。

仅通过env.sh使用既有simulation镜像运行离线测试；env.sh中up命令为避免build曾临时改用单端start，调用后立即按字节恢复，最终与备份逐字节相同。runtime/learning未启动，simulation已恢复停止；未build/pull、未执行Git写操作。Python语法与100列检查、文档链接及`git diff --check`已检查；未安装Ruff时不声称执行Ruff。

**P3-F交付时的后续方案为P3-G；现已被§14用户最新指令调整为数据优先排查，R2暂缓实施。** 四项语义已确认，不再为同一事项重复询问；P3-F授权不延伸为本轮生产实施授权。真实MuJoCo跟踪、碰撞、持物/接触维持、真实通信延迟、从头默认5点闭环和完整任务成功仍未验证。P3总体未通过。

## 14. 按数据→模型→执行收敛：随机抽查与真实物理回放（2026-11-15）

用户最新优先级：首先修复几乎没有成功的问题；代码清理靠后。停止继续扩展R2，先用少量随机数据回放与模型对照排除容易排除的部分。此前P3-F的零推理/零闭环限制只限定旧阶段；本节的小样本物理回放、离线模型查询及真实推理对照依最新指令执行，未启动训练、正式补采或修改旧权重/数据。

### 14.1 固定随机抽查：6局、4190帧

固定抽样seed=20261115，每项任务2条训练、1条验证。Panda：10027/10007/10045；Marvin：4157/4155/4158。当前数据分别为`var/panda_stack/datasets/panda_stack_learning_v1`、`var/marvin_handover/datasets/dataset`，均45train+5validation；没有按结果重新挑样本。

- 实际Parquet的action、observation.state/velocity、raw_action_chunk与原export按float32逐值相同，最大差0；两套时间列完全相同。20Hz连续、action lead=50ms、执行区间、NaN/Inf均无异常，动作目标没有硬限位外值。
- 54个训练MP4帧与对应原PNG的定位时间最大差4.55e-13s，无相机错序、黑帧或明显错位。最差PSNR：Panda36.49dB，Marvin28.83dB；有损压缩会改变像素，不宣称两种输入完全相同。
- 动作近静止比例Panda31.6%–33.3%、Marvin约41.5%（阈值：每50ms臂<0.001rad、夹爪<0.0001m）。Panda最长1.8–1.85s主要最终稳定；Marvin最长1.55s含物理反馈未过门及连续稳定窗口。`WAITING_FOR_FEEDBACK`不是网络等待，不直接删帧、压缩时间戳或删掉物理门。
- 发现近同观测对应不同相对未来：Marvin帧310/337状态臂差6.07e-5rad、夹爪差6.63e-7m，右腕图完全相同，global图MAE仅0.015/255，未来60点最大差0.7267rad；Panda相似例约0.351rad。这提示等待/阶段时序敏感，但后续实际模型仍能利用细小图像变化区分相位，不能写成严格不可学习。

证据：`var/_system/diagnostics/triage-data-audit.json`、`triage-data-contact-sheet.png`；复现脚本`triage_data_audit.py`、`triage_data_visuals.py`。这是抽样通过，不是穷尽证明全部数据无问题。

### 14.2 动作回放：确认真实可完成性，并修复B09

`data_first_replay.py`只按seed初始化场景一次，随后每2ms调用现有RobotIO与真实`mj_step`，不覆盖物块位姿、不调用Teacher重新规划。比较原MCAP三次多项式Reference与训练用20Hz action路径两种模式，后者为位置线性插值及匹配速度前馈；任务评价使用当前物理判据。

| 样本 | 原Reference物理回放 | 20Hz标签物理回放 | 原Reference对历史实测最大差 |
| --- | --- | --- | --- |
| Panda10027/10007/10045 | 3/3 SUCCEEDED | 3/3 SUCCEEDED | 0 |
| Marvin4157/4155/4158，补修前 | 3/3在合法左开爪时误报HOLD_LOST | 同样3/3误报 | 0；继续物理至原终局仍逐值复现 |
| Marvin相同三局，补修后 | 3/3 SUCCEEDED | 3/3 SUCCEEDED | 0 |

**B09：Policy交接证明路径把正常左手打开过程当成丢物。** Marvin4157在31.870s已有稳定空中双持证明；左手32.90s失去双指持物，33.00s完全无接触，但右手继续稳定持物。33.154s左开度0.066177m、手指仍以约0.0138m/s正常打开，尚未达到“开足且慢速”的释放证明；旧`_holder_required(left)`因此继续要求左持物，超0.25s后误判HOLD_LOST。其余两局复现相同错误。

修复：空步骤Policy路径在**空中稳定双持证明成立后**将持物丢失职责交给右手，左手不再被要求一直夹到完全打开。完整左开足/慢速/无接触、右独持稳定以及最终箱底/1s稳定成功证明全部保持；Teacher步骤分支不变。没有放宽接触力、持物丢失容忍、摩擦或成功阈值，也没有新增消息/速度字段。

- 新回归修复前1 failed，修复后`test_handover_task.py`和真实接触证据测试共41 passed；左独持证明后但双持前失持、右接管后失持、开爪仍接触等负例保持。
- 修复后三条Marvin均在46.906–47.110s完成，真实左独持/空中共持/释放后右独持各通过原0.5s窗口，最终真实箱底及1s稳定通过。
- 20Hz路径相对历史最大臂误差Panda0.000194–0.000214rad、Marvin0.000555–0.000558rad，抽中12种动作路径均可完成任务。

证据：`data-first-replay.jsonl`（原基线，首个4157早停后另有完整续跑，以完整行复核）、`data-first-replay-fixed.jsonl`、`data-first-handover-red.log`、`data-first-handover-green.log`。这些是新回放，不改写原240局或原采集结果。

### 14.3 正确数据经过当前Follower：2次成功对照

为了区分“动作能直接执行”与“当前规划器能执行这些动作”，使用原数据给出O+50ms..3s的完整60点，走生产`prepare_follower_plan`、`MotionRuntime`及真实物理；保持固定继承anchor、原连续限位、Follower偏差/跟踪/接触守卫和当前成功判据。

| 场景/seed | 当前正式窗口 | 接受块 | 结果 |
| --- | --- | --- | --- |
| Panda10027 | k=5 | 93 | 24.018s SUCCEEDED |
| Marvin4157 | k=20 | 47 | 46.932s SUCCEEDED |

这是**完美数据预测、即时计算、零RPC延迟**的执行对照，未运行学习模型，未经过runtime的P3-C整形，不是实际Policy成功。它证明原固定anchor/窗口的执行器具备执行正确动作的能力，不能继续把R2替换协议当成当前首要路线。

Teacher与Policy实际binding配方的scene路径、control路径及velocity_feedforward=True均相同。因此可优先排除抽中示教动作不可执行、20Hz降采样必败、基本摩擦/servo使正确动作无法完成。没有据此排除模型偏差下的接触敏感性、真实推理/RPC时延或闭环观测漂移。

证据：`var/_system/diagnostics/data-first-oracle-follower.jsonl`；`data_first_replay.py --mode oracle_follower`。

### 14.4 模型对数据：学到了动作，局部误差与输入域仍待处理

使用同6局、实际部署Bundle：Panda`stack-act-refined2`、Marvin`handover-act-refined-final`。44次真实训练视频输入预测（Panda20、Marvin24）及12次同刻原PNG对照，共56份输出；没有训练或修改权重。

监督定义确认：state为实测，action是O+50ms的执行参考。训练delta=0..2.95s对应部署50ms..3s，没有发现天然错一帧。

| 视频输入验证样本，按窗口平均 | 活动臂MAE / 保持当前状态基线 | 活动夹爪MAE / 基线 |
| --- | --- | --- |
| Panda | 0.01322 / 0.16204 rad | 2.275 / 34.883 mm |
| Marvin | 0.01049 / 0.22812 rad | 1.467 / 17.859 mm |

不支持“训练整体失效”，暂不通过改LR或追加训练来碰运气。但也不能因为平均误差小就排除模型：Panda原PNG闭爪窗口夹爪MAE5.797mm、峰值12.303mm；Marvin原PNG窗口活动臂MAE0.048735rad、峰值0.127037rad，其中前5点MAE0.002995rad，21–40点0.062359rad。视频换同刻原RGB可使Marvin该窗输出最大变化约0.069rad。

因此接下来的小对照针对**实际选中视界、在线RGB、连续预测一致性和延迟**，不先改物理参数，也不把模型局部误差藏进一个全轴平均数。

证据：`collection-model-probe.json`、`collection-model-probe.npz`；NPZ保存raw/truth/state/O/observation_id/frame/episode及公共50ms..3s时间轴，用于执行对照，不复制图像或模型。

### 14.5 启动环境的最小修复与复原

`env.sh up/enter`改为`up -d --no-build --pull never <service>`，普通启动复用本地镜像；新增显式`env.sh build <env>`，README同步。真实启动三端成功，并用临时docker命令替身核对up与build分派；没有build/pull镜像。B08只完成“普通启动不隐式构建/拉取”子项，基础Dockerfile来源固定仍未处理。

本轮既有文件修改前态统一在`/tmp/sim_dual_arm-data-first-7ra_tv68/original/`按相对路径保存，包括env.sh、README、handover evaluator、其回归及任务书/索引等实际修改文档。确认目标没有后续修改后逐文件`cp -p`恢复；不使用旧阶段备份。新增诊断脚本/报告仅本节分析及后续执行对照消费，正式数据/权重/原MCAP未写回；未执行Git写操作。
