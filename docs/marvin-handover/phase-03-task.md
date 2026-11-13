# P03 交接任务与 Teacher

**交付状态：已完成（2026-11-13），见 [deliveries/phase-03.md](deliveries/phase-03.md)。** runtime 任务预算上限为有界 300 s；本任务使用 180 s 仿真预算，实测完整示教 56.1 s。

前置：P02B 的接触持物、候选路径和标准控制通过。目标：新增 `handover` 生命周期、门禁与评价，通过真实 runtime Teacher 控制链完成[任务行为](task-contract.md)。

任务实现复用 P02B 的目标/路径生成和 P02A 的机器人/TCP/接触能力，不重复定义原始关节、执行器或诊断轨迹。Teacher 与门禁可以依赖 MuJoCo 任务观测，通用 worker、控制调度和 runtime 采集流程只依赖其能力与不可变输出。

## 标准示教链的准备顺序

本阶段必须完成 runtime Teacher 适配和 SDK，不能将其推迟到 P04。按以下顺序建立实际依赖：

1. 实现 handover 任务插件、Teacher IDL 和 simulation/runtime 两端具体扩展适配，明确 capability 与阶段码。
2. 建立真实 `simulation_teacher`、`runtime_teacher` 和诊断 `campaign` 配方，以及三份 profile 与必要接口组成的最小 `binding.yaml`。campaign 先填写 P04 规定的独立预算/种子/路径，`required_paths` 只列已经存在的文件；模型为 null，尚未实现的 learning/policy 引用不声明。
3. 在 runtime 容器执行 `build-ros-sdk --scenario marvin_handover --shared-root /shared`，取得实际扩展包和匹配标记，再运行 collection 校验与选择预检。预检不能回退 Panda 的任务配置、Teacher 或台账路径。
4. simulation/runtime 分别通过 `prepare --scenario marvin_handover --role teacher` 生成新目录配置，启动 backend 与 runtime，通过标准控制链验收示教。运行配置使用 prepared 结果，不省略身份和约束校验。

当前 simulation scenario teacher prepare 校验 collection，必须先有上述真实引用；runtime direct `--template` prepare 当前只支持 policy，不作为 Teacher 入口。P03 创建采集侧引用是启动依赖，P04 再完成实测数据质量、导出与采集验收。

构建包集前关闭本次相关进程。切换 Panda 回归时按 Panda 方案重建活动 SDK 并检查标记，不能让旧进程继续使用被替换的包集。

## 任务插件与物理评价

1. 在 simulation 新增 handover 的配置、任务编排、Teacher、门禁和评价模块，并在现有 registry 显式注册。保留 stack 插件，通用 worker 不按任务名拼接抓取逻辑。
2. 复用 `PrepareTask / StartTask / GetTaskState` 生命周期及现有 lease/context/task_epoch/capture 约束。准备在不可变快照上规划，不执行运动；启动后 owner 根据实测事实推进，复位清理旧计划与门禁。
3. 按 [task-contract.md](task-contract.md)实现全部行为阶段。`dual_hold` 必须证实右手已稳定接住同一物块；只有通过门禁才允许左爪打开。左臂退出完成前，右臂保持物块，不开始穿越左臂所在空间。
4. 物理评价分别检查离桌、交接悬空、左右持物、外部支撑、入箱完整包含、双手释放与最终稳定。评价读取同一 owner 捕获的仿真事实，不让 ROS 回调并发读取 MjData。
5. 独立校验终态，防止 Teacher 阶段名或计划完成直接触发成功。推理模式仍启用同一个物理评价器，但不启动 Teacher 输出。

## TaskState 与诊断

公共 TaskState 保持现有结构。`phase` 表示任务生命周期，行为阶段与物理事实进入 `metrics`，统一由 ROS 转换器写为有限值 JSON。至少明确这些指标的名称、单位与含义：

| 指标类别 | 必需事实 |
| --- | --- |
| 阶段 | 当前行为阶段、门禁状态/失败原因、进入时间、等待时长 |
| 持物 | 目标物块身份，左右手双指接触、持续时长、物块相对夹爪运动 |
| 悬空/支撑 | 离桌高度、桌/箱/其他外物接触与支撑、交接窗口是否违规 |
| 交接 | 右手接稳时间、左手实际释放时间、左手退开是否完成 |
| 最终结果 | 完整入箱、双手释放、物块线/角速度、连续稳定时长、失败/停止原因 |

指标 schema 在本任务说明中维护即可，公共 TaskState 不添加 Marvin 专用字段。运行端可原样记录；Policy 输入不加入物块位姿、阶段或成功判定真值。

## Teacher 扩展与执行

- 在新方案下建立 `marvin_handover_api_msgs`，capability 拟为 `marvin_handover_api_msgs.teacher_chunk.v1`。提供现有采集选择器需要的 `TeacherCapability.msg`、`GetTeacherChunk.srv`、包清单和构建文件。
- 本次保留 60×16、50 ms 的目标块规格及显式 profile、epoch、capture、观测时间/源时间、plan/chunk 身份。阶段码按本任务定义，不复用 Panda stack 的九阶段语义。
- simulation 服务注册与 runtime 类型解析都按明确 capability 选择已实现适配。具体扩展的导入、请求/响应转换放专用适配模块，通用任务客户端、Teacher 流与采集编排消费稳定能力，不直接依赖 Panda/Marvin 扩展类型。IDL 可随方案选择构建。
- 任务插件生成不可变目标和门禁事实，ROS 响应填充放所属端转换器。需要调整现有插件协议时，同时迁移 stack 调用者与测试，避免长期保留两套同义调用路径。
- Teacher 输出经现有动作流与 SubmitFollower 执行。闭爪/换手等离散意图使用明确的阶段保持和实测门禁，避免未来动作块越过尚未通过的门禁而提前松左手。
- 处理捕获与阶段切换的对应关系：返回的门禁和目标属于被请求的快照，过期 epoch/capture 拒绝；不能用稍后接稳的事实为更早观测生成“已可松手”的目标。

runtime 的新扩展适配、本阶段真实配方和 SDK 是完整示教的必需交付。录制/导出质量完善和诊断数据集采集在 P04 完成。

## 时限与窗口覆盖

P02B 的连续物理诊断采用 72 个有限 follower 窗口，跨度 122.290 s 仿真时间、128.59 s 测试时间。规划报告的 34.781 s 为多个候选分支段时长累计，不能替代真实调度预算。P03 可用 180 s 仿真预算作为首轮有界起点，再按 Teacher 实际调度和门禁复测；不能沿用 Panda 的短时默认值。

完整示教预算覆盖候选路径、夹爪动作、门禁等待和最终稳定，并与任务插件、Teacher/runtime、停止余量及墙钟时限一致。当前 runtime 任务预算上限为 300 s（见 [runtime_config.py](../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/runtime_config.py)）；本方案为 180 s 仿真、256 块、420 s 墙钟，实测为 56.1 s 仿真、57 块。

若本任务有实测依据需要更长总时限，先在所属实现集中调整该任务时限约束及相关消费者/测试，保持租约、跟踪、轨迹和停止保护有效。`max_chunks × follower.target_count × action_period_ns` 必须覆盖 `task_sim_budget_ns`，墙钟预算覆盖实际仿真速度、RPC 与收尾。准备阶段就验证参数一致，不靠运行时无限延长等待。

## 验证与完成条件

完成至少一次全程物理示教，保存可观察双手接稳/释放先后顺序的图像、阶段事件、接触/支撑事实、控制反馈与最终结果。轨迹目标到位不足以验收。

新增 handover 集中在任务环境、Teacher、门禁/评价及具体扩展适配；沿用 P02 的机器人执行映射与现有调度机制。stack 与 handover 走各自任务/扩展选择，通用机制不依据方案名实现两套行为。

针对实际失败入口验证：右手未接触/未接稳时左手不松；左手释放后右手失持、阶段超时和运行中中断能终止并收尾。至少右手未接稳与操作者中断两项通过实际仿真运行；其余判断使用针对性状态机/物理测试，避免为故障演示引入永久调试开关。

最终同一模型/配方条件下需取得连续 5 次复位后的完整成功，可与 P04 合格采集共用证据。此处衡量固定场景重复性。P03 首次完整成功且必要失败路径有效后进入 P04；后续修改门禁、模型、场景或执行参数时，受影响的连续成功证据重新取得。

`deliveries/phase-03.md` 记录指标/阶段码定义、门禁参数、接口 capability、全程结果、失败与停止证据，以及任务/机器人/ROS 职责调整。

目录导航：[任务书入口](README.md) · [前一阶段 P02](phase-02-motion.md) · [下一阶段 P04](phase-04-runtime-data.md)
