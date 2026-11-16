# 第二阶段执行提示词

本提示词准备 P2 的实施范围，当前尚未执行。完整复制下方代码块给项目中的编码 AI，或让其读取本文件并执行。P1 补丁必须保留。

```text
在 /home/ljk/work/sim_dual_arm 执行双任务修复任务书 P2，完成物理接触证据与真实交接顺序验证后停止，不自动进入 P3。

先读：
1. /home/ljk/work/sim_dual_arm/RULES.md
2. /home/ljk/work/sim_dual_arm/docs/experiments/2026-11-15-inference-repair-taskbook.md（尤其 P2、P1 实施结果及独立复核）
3. /home/ljk/work/sim_dual_arm/var/_system/diagnostics/2026-11-15-code-and-artifact-audit.md
4. /home/ljk/work/sim_dual_arm/var/_system/diagnostics/2026-11-15-inference-failure-analysis.md

基线：P1 已修复离线稳定时长硬编码与 Stack XY 投影，原成功集合仍为 Panda 两局、Marvin 零局；P1 和文档均尚未提交。不要清理、回退或覆盖已有补丁。此阶段验收物理评价可信度，不以成功率提升为目标。

本阶段只实施：
- B03：Marvin Policy/evaluation_only 的 steps=() 下，当前可仅凭最终入箱成功，缺少独立的空中交接证明，步骤相关 HOLD_LOST/交接窗口保护也有空缺。
- B04：Stack 将 inactive gap 接触计入手指接触数，可能把近接触当作未释放。
- B05：Marvin box_bottom_support 按整个箱体 body 合计，侧壁可冒充箱底。
- B07：backend.task.teacher_chunk 日志把 chunk.request_id 写成 episode_id。

先核实调用链，定位入口：
simulation/mujoco/src/robot_backend_sim/tasks/handover/{plugin,evaluator,observation,gate,teacher}.py
simulation/mujoco/src/robot_backend_sim/physics/contact_snapshot.py
simulation/mujoco/src/robot_backend_sim/physics/marvin/environment.py
simulation/mujoco/src/robot_backend_sim/tasks/stack/evaluator.py
simulation/mujoco/src/robot_backend_sim/ros/node_services.py
tests/unit/test_handover_task.py、test_stack_task.py、test_stack_task_execution.py、test_handover_target_geometry.py，以及现有接触/ROS 适配测试。

约束：
1. 不执行 Git 写操作；只读检查使用 GIT_OPTIONAL_LOCKS=0。记录已有修改，将本阶段要改的原文件按相对路径备份到 /tmp/sim_dual_arm-repair-P2-<唯一标识>/original/，列出新文件。保留通过的修复，提供精确复原办法，不覆盖用户后来修改。
2. 不训练、不补采正式数据、不改既有数据/权重，不运行批量 Policy 推理。允许最小真实 MuJoCo 接触用例、任务状态机测试、ROS 适配验证及只读使用既有录制。
3. 不改摩擦、servo、IK、关节硬限位、Follower、准入、融合，以及源配置中的力/高度/速度/持稳/持物丢失/最终稳定门槛。复用现有配置，不另建一套 P2 常量。
4. 物理证明归 simulation 的对应任务插件；不把具体任务名、阶段表或轴组放进通用 runtime/learning/shared，不造通用工作流引擎或兼容模式，不加“跳过交接”的开关。
5. 优先在任务内部 observation/state 和现有 TaskState.metrics_json 中表达任务专有事实，列出字段、生产者/消费者。保持 ROS/protobuf/schema、GetContactSnapshot 公开结构不变。确需跨端类型变化时先给出具体必要性与接口方案，按 RULES 确认，不能静默改 SDK。
6. 原 MCAP/task_result/closure audit 不改写、不补造新字段，不为缺少证据的旧记录默认填“已完成交接”。正式运行日志归 logs，离线和诊断验证归 diagnostics，临时测试视图归 /tmp；不生成无人消费的全树摘要或证据包。
7. 复用 runtime/simulation 既有镜像，不 build/pull，不启动 learning。核实 tests 真实可见且导入本次源码；记录容器初态，结束后清理临时视图并恢复初态。不把容器改造混入 P2。

实施顺序：

A. 先使接触事实正确（B04/B05）
- 先写能暴露旧行为的最小真实 MjModel/MjData 用例，使用 mj_contactForce；不要只手写 contact_count/force 假对象。按本地版本核实 margin/gap、efc_address 和求解时机。
- inactive 的正间隙接触不能阻止“已释放”，也不能证明持物或支撑。不要把所有真实 active 的零法向力触碰一概忽略来放宽安全判断；按原任务意图明确 active 触碰语义并测试，不随意增加 epsilon。
- 修改共享接触聚合前查消费者，保持力、计数、单位含义一致；不批量改变 robot-robot/robot-box 碰撞守卫。
- 区分箱底 geom 与四面箱壁；现有 marvin/environment.py 已提供 box_bottom / box_wall_* 和 box_geom_ids，优先复用绑定。不得在共享接触聚合器里硬编码 Marvin 的箱体几何。
- 箱底支撑须来自真实箱底对物块的有效支撑，正确处理 geom1/geom2 顺序和作用方向；仅侧壁接触/摩擦支撑、箱外桌面支撑均不能替代。不能用“box 总力减 table 总力”代替箱底实测。
- 为最终放置新增箱底专用量时，侧壁等全部外部接触仍参与空中借力检测。更新内部 observation 的全部构造和测试，不留旧字段别名或默认通过值。

B. 再使空中交接成为成功必需证据（B03）
- 物理证明与 Teacher 目标步骤分离；无论 steps 是否为空，都由 owner 线程实际观测判定：左手单独空中持稳 -> 空中双手共持 -> 左手真实释放后右手单独空中持稳，严格有序。
- 使用已有两指力、相对位姿/运动稳定、离桌/外部支撑、释放和持续时长门槛；不能由 Teacher 名称、接受命令、关爪目标或最终入箱反推。右手单指擦碰不算共持，桌上双手夹住不算空中共持，左爪目标为“开”但仍夹持不算已释放。
- 条件中断按原语义重新计时；重复时间戳不得累积时长，新阶段不得借用前一阶段计时；复位/新 task_epoch 清空证明，成功按既有规则冻结。
- 复用现有相对位姿、接触与保护机制，明确状态所有者。Teacher 仍可按 steps 生成目标，不能成为 Policy 证明来源；不要让 Teacher/Policy 维护不同阈值的两套物理判据。
- 补上 Policy 下持物丢失、交接中外部支撑检查；初始桌上抓取不误报，建立持物后按原丢失容忍时长判断，合法落箱不继续被算作空中借力。
- 真正成功必须同时具备完整序列证明和原入箱/真实箱底支撑/双手释放/速度漂移/最终稳定窗口。新 metrics 字段必须接入实际成功条件，不能仅输出诊断值却保留绕过路径。
- 在现有 metrics_json 保存录包复核需要的阶段完成状态、实际完成时间及必要实测量，明确消费者；不增加逐 tick 独立文件或第二套 JSON 账本。

C. 修复日志关联（B07）
- 使用已按现有链校验的真实 request/context episode_id，request_id 独立保留；不截断 request_id 猜 episode，不增加回退分支。
- 用适配测试证明 episode_id 与 request_id 不会再混用。

D. 验证矩阵与 P1 测试衔接
- 序列正例：三段实测空中持稳按序成立，随后真实落底、释放、达到原最终稳定窗口才成功；Teacher 与 Policy/evaluation_only 都覆盖。
- 序列负例：直接入箱静置、缺一段、乱序、单指/力不足、桌面共持、时间不足、中断后累计、左未实际释放、持物丢失超原容忍、空中借助桌/箱、跨 episode 继承旧证明。
- 接触正反例：真实双指持物、真实箱底、inactive gap、active 触碰、仅侧壁、桌面；现有碰撞/掉落守卫继续有效。至少用真实 MuJoCo 最小例子贯通“接触采样 -> 任务 observation -> 评价”。合成观测只能证明状态机逻辑，不能替代接触物理验证。
- P1 的 test_success_needs_the_configured_one_second_window 当前从最终放置直接起步；P2 后先建立完整交接前缀，再测最终 0.999 s 不成功、1.0 s 才成功，不删除稳定边界覆盖。
- test_evaluation_only_plan_uses_the_same_physical_success 当前接受“没交接、只入箱”，应改为拒绝负例，并补完整交接正例。必要 Teacher fixture 同步真实化，不为保旧测试增加旁路。
- 保留 P1 的通用 1 秒成功事实识别、R.T、margin 边界和 upright 负例；通用离线端继续消费冻结事实，不复制任务阶段表。
- 先记录相关现有测试基线，再跑修改点及调用方回归、必要真实物理/ROS 适配验证；不新增 importorskip、删负例或降低断言来通过。准确区分既有失败、本次回归和未验证项。
- 既有 Teacher 录制可只读验证记录到的测量顺序，不能当 Policy 成功；旧录包缺 geom/法向或新证明时不能补造事实或回退旧验收。旧 Marvin 120 局都未完成任务，P2 不应把它们重标成功；Panda 历史成功如涉及接触语义变化，按证据说明，不为维持数字调规则。

交付：
更新任务书 P2 状态及一份实施摘要、必要索引。先报告行为变化和验证结论，列实际修改/新文件、内部字段及消费者、命令/环境/通过失败跳过数、物理验证和历史证据边界、备份及恢复办法、P1 保留和容器清理情况。
不把逻辑测试通过写成闭环成功，不声称提高成功率。保留可审查补丁，不提交 Git，不自动做 P3 裁剪/窗口/融合调参。
不要只交计划，要完成本阶段允许范围的实现和验证。确有不可避免的跨端变更时，先完成不依赖它的部分，再按 RULES 说明具体阻塞和接口方案。
```
