# 架构边界与当前接入缺口

依据 2026-11-13 的工作树与阶段记录核对。P00～P06 已交付；Teacher 完整物理任务、6 条诊断数据与 4/2 划分、128 step 新模型执行链路和职责修复均已验收，见 [P06 交付](deliveries/phase-06.md)。P06 的静态导入检查确认 `shared/common/` 不反向导入四端、learning 不导入 ROS/MuJoCo、runtime 不导入 MuJoCo 或方案 Teacher 消息、simulation 不导入 runtime/learning；修复后的边界按下表以真实调用关系核对。推理专用轨迹保真容差和模型效果限制保留，正式策略可用性另行验证。职责提取以真实调用关系为依据。

## 端与方案的职责

| 归属 | 持有内容 | 对外边界 |
| --- | --- | --- |
| `shared/common/` | 跨端接口、档案 schema、公共轨迹/反馈结构、通用工具 | 公共生产包不反向导入四端；保持原有身份与约束验证 |
| `shared/scenarios/marvin_handover/` | Marvin 档案/标定、各端配方、方案绑定及专用 IDL | 声明选择与参数，不持有机器人模型或运行端业务实现 |
| `simulation/mujoco/` | `assets/` 下机器人模型资产、模型装配、机器人 I/O、碰撞/接触、动力学、IK、任务插件、Teacher 与物理评价 | 持有 `MjModel/MjData` 与物理步进；对外发布不可变状态、服务结果和反馈 |
| `runtime/ros2/` | 能力适配、同步观测、租约、动作流调度、停止、录制、导出与质量审计 | 通过 API 使用 Teacher/TaskState/控制反馈，不导入 MuJoCo、求解交接位姿或猜测物块物理状态 |
| `learning/lerobot/` | 数据转换、采样、训练、处理器、bundle 和 Policy 服务 | 保持 ROS-free；观测/动作绑定来自 Marvin 档案，使用本次训练统计 |
| `hardware/` | 真机驱动与设备能力 | 本次无真机实现交付 |

## 跨端公共与端内复用

跨端公共放在 `shared/common/`，包括协议、数据契约与确实被多端使用的数学/轨迹表示。端内复用留在所属端：MuJoCo 步进和渲染依赖具体后端，ACT 训练依赖具体算法；它们可服务多个任务，不因此成为跨端公共代码。

四端内部按实际变化区分可复用机制、机器人/算法适配和任务实现。下表定义职责，不要求为四端建立对称目录或统一的 `common/` 层。

| 端 | 端内可复用机制 | 机器人或算法适配 | 任务专用内容 |
| --- | --- | --- | --- |
| simulation | MuJoCo 步进、渲染、命令调度、反馈发布、场景编译与复位机制 | 机器人模型装配、关节/执行器映射、夹爪驱动、TCP、接触语义与运动学绑定 | 桌面/物块/箱体等任务环境构造、Teacher、阶段门禁、允许接触和成功判定 |
| runtime | 观测同步、租约、动作发送、录制、通用导出/审计流程、进程收尾 | Robot/Policy 协议与能力适配，优先复用现有客户端 | 专用 Teacher 接口适配、任务相关导出内容、阶段定义和质量规则 |
| learning | 数据/训练运行、预算、模型管理与服务会话中已被实际消费者复用的部分 | ACT 等算法实现、处理器、模型格式与相应输入输出约束 | 任务相关样本筛选；任务文本、数据划分和训练参数优先由配方表达 |
| hardware | 实际驱动出现后按真实共性提取的通信、状态与控制机制 | 厂商驱动、夹爪、传感器和标定适配 | 消费上层目标与约束，通常不承接任务编排；本次保留边界说明 |

“跨任务复用”不等于“算法无关”：ACT 特有的模型构建、处理器与动作块约束继续归 ACT 适配，不能把整套训练代码标为通用算法机制。learning 已有 [adapters/](../../learning/lerobot/src/dual_arm_learning/adapters/)，先验证新档案/配方能驱动现有实现，再依据实际缺口调整边界。

simulation 内还应分清三种场景职责：通用场景容器负责编译/复位，机器人装配负责本体及其绑定，任务环境负责桌面/物块/箱体及布局。接触层先从具名几何产生接触事实，机器人适配解释左右爪、指面和开度，任务再判断“同一物块已接稳”“交接时无外物支撑”。Teacher 规划从任务快照生成目标，经现有控制通道执行，不能在规划线程直接修改物理状态。

## 变化如何落到实现

| 变化 | 所属修改 | 应继续复用的内容 |
| --- | --- | --- |
| 已有实现支持的机器人/任务/模型组合或数值变化 | 新增/调整方案的选择、档案与配方；checkpoint 通过真实 bundle 引用替换 | 各端已有机制与适配实现 |
| 新机器人能力 | simulation 的模型/执行/接触/运动学适配与新档案；真机驱动出现时归 hardware | 任务逻辑及端内机制，在其所需能力和公开契约相容时复用 |
| 新任务能力 | 所属端的任务环境、Teacher、门禁/评价、专用接口适配和数据质量规则 | 机器人执行映射、通用动作流、录制和学习运行机制 |
| 新算法能力 | learning 的算法/处理器/模型格式适配及配方；契约变化时同步必要生产者和消费者 | 契约相容的 Robot/Policy 调度、控制与录制机制 |

`shared/scenarios/<id>/` 保存选择与参数，通过档案和各端配方选择机器人、任务实现、算法与训练 bundle。方案专用 IDL 随方案保存；机器人原始 URDF/网格归 simulation 的机器人资产，Teacher、驱动、模型装配和任务评价归所属端。binding 现有字段能表达的选择直接复用，机器人装配/资产由 simulation 解析，不向 binding 填尚未支持的字段。

## 机器人资产与路径

当前 Panda 位于 `simulation/mujoco/assets/franka_emika_panda/`，Marvin 位于 `simulation/mujoco/assets/marvin/`。两者都由 simulation 的 [assets.py](../../simulation/mujoco/src/robot_backend_sim/physics/assets.py) 按所属端定位；Marvin 通过 `robot.assembly: marvin` 选择固定版本模型。资产说明随模型保存，Panda 位置和内容保持不变。

宿主机的 simulation 目录在容器内挂为 `/app`，shared 独立挂为 `/shared`。场景配方只表达机器人选择，不写模型文件路径；runtime/learning 不解析 MuJoCo 资产。跨仓相对路径、容器固定 `/app` 字符串和新增挂载均不作为解析方案。

## 准备入口与阶段依赖

| 阶段 | 可执行入口与必须先具备的内容 |
| --- | --- |
| P01/P01A | 离线模型检查；P01A 补齐资产定位、有效碰撞与联动约束 |
| P02A | 已交付：真实 backend 配方，direct `prepare --template`；`task/task_mode` 同时省略；SDK 为 robot_api_msgs＋sim_api_msgs |
| P02B | 已交付：标准控制下的左右独立持稳、共持全过程、带物停止及连续空中交接、退手、运输和入箱释放均通过；任务目标/路径模块供 P03 复用，证据见[交付记录](deliveries/phase-02b.md) |
| P03 | 已交付：任务插件/门禁/评价、Teacher IDL、两端能力适配、180 s 有界预算与真实完整示教；证据见[交付记录](deliveries/phase-03.md) |
| P04 | 数据已交付：通用导出按配方阶段/必需阶段生成区间，质量门槛由真实渲染测量；1 条 pilot、6 条诊断数据与 4/2 SELECTED 清单通过；中断录制已关闭、终态已按真实产物补录，见[交付记录](deliveries/phase-04.md) |
| P05 | 诊断链路已交付：learning/policy 配方、新 bundle、真实预测/执行/收尾；左臂轨迹保真容差最高放宽 30.2 倍，仅作诊断条件，不证明原容差下可用，见[交付记录](deliveries/phase-05.md) |
| P06 | 已交付：证据与职责核对、命令核对、campaign 改动后的针对性回归；边界检查与应用说明见[交付记录](deliveries/phase-06.md) |

当前 simulation 的 scenario teacher prepare 校验 collection，要求三份 profile 与 simulation_teacher/runtime_teacher/campaign；runtime direct `--template` prepare 仅支持有真实 Policy binding 的 policy 路径。P02 不构造假 collection 配方，P03 则提前建立实际可用的采集侧引用；P05 已将 learning/policy 与 simulation/runtime policy 引用加入 binding，模型由真实新 bundle 提供，policy 准备仍要求真实 Policy binding。

现有活动 SDK 默认共用 `build/ros-dev` 并挂载至 simulation 的 `/opt/ros-dev`。构建不同包集前结束本次相关进程，按所选包集/方案重建并校验标记，再启动后端；Panda 回归明确切回 Panda SDK。P02A 的当前活动包集是 `robot_api_msgs`＋`sim_api_msgs`（不含 `panda_stack_api_msgs`），Panda 示教前必须按 Panda 方案重建并核对标记。新任务可在显式适配选择点增加注册，通用执行循环不增加任务行为分支。

适配创建与任务选择在所属端的装配入口完成，通用执行循环消费已选择的能力和事实。换任务不能要求重写机器人 I/O，换机器人不能通过复制整套任务/控制链或在同步、步进、录制主循环中增加方案名分支完成。职责已经清楚的模块保留原位，目录调整服务于真实依赖修复。

## 当前源码依据与已落地修复

| 当前事实与读取入口 | 对 Marvin 的影响 | 最小修复方向 / 阶段 |
| --- | --- | --- |
| P01 已有 [scene_container.py](../../simulation/mujoco/src/robot_backend_sim/physics/scene_container.py)、[Marvin 场景组合](../../simulation/mujoco/src/robot_backend_sim/physics/marvin/scene.py)和独立装配/环境函数 | 已能离线运行；P02A 已完成标准 Worker 的场景选择与 I/O 装配 | P01A 已整理模型与资产定位；P02A 在 [scene_loader.py](../../simulation/mujoco/src/robot_backend_sim/config/scene_loader.py) 按 `robot.assembly` 选择配置，在 [robot_adapters.py](../../simulation/mujoco/src/robot_backend_sim/physics/robot_adapters.py) 注册场景与 I/O；Panda 场景配方同样显式声明 `assembly: panda` |
| [Marvin 装配](../../simulation/mujoco/src/robot_backend_sim/physics/marvin/assembly.py)只保留 `Arm_*5`/`Arm_*7` 两项有几何依据的同臂排除；源 URDF mimic 已导入为每侧一条 joint equality，编译模型 `neq=2` | 同臂其余配对、跨臂、手指和环境碰撞保持有效；P01A 已用实际可达位姿验证 | P02B 候选路径继续使用；后续若运动区域进入底座范围再补代理 |
| [panda_io.py](../../simulation/mujoco/src/robot_backend_sim/physics/panda_io.py)、[worker_owner.py](../../simulation/mujoco/src/robot_backend_sim/runtime/worker_owner.py) 直接构造 PandaIO；[control.py](../../simulation/mujoco/src/robot_backend_sim/control/control.py) 检查具体 PandaIO 类型 | P02A 已解除：`RobotIO` 能力契约、[MarvinIO](../../simulation/mujoco/src/robot_backend_sim/physics/marvin/io.py) 具名绑定和装配选择均由所选机器人提供 | controller/motion 只消费 `CompiledScene`/`RobotIO`；继续保留同一场景、线程归属和 profile/配置验证 |
| [motion_config.py](../../simulation/mujoco/src/robot_backend_sim/config/motion_config.py)、[follower_config.py](../../simulation/mujoco/src/robot_backend_sim/config/follower_config.py) 强制 Panda 16 维名称/顺序 | P02A 已解除：配方只验证维数/唯一性/数值，启动时按所选 profile 的具名顺序绑定 | 新增机器人只需提供 profile 与相应配方；不得恢复 Panda 硬编码顺序 |
| [kinematics.py](../../simulation/mujoco/src/robot_backend_sim/kinematics/kinematics.py) 保留 Panda 右臂 key-pose 检查；[kinematics/dual_arm.py](../../simulation/mujoco/src/robot_backend_sim/kinematics/dual_arm.py) 使用机器人适配提供的具名关节、TCP 和碰撞分组 | P02B 已解除左臂/双臂目标限制，规划净空、共持全过程与连续交接入箱物理诊断通过 | 新机器人通过 `RobotGeometry` 提供关节/TCP/指面/分组；目标与路径归 `tasks/handover`，P03 Teacher 复用该模块 |
| [tasks/handover/plan.py](../../simulation/mujoco/src/robot_backend_sim/tasks/handover/plan.py) 提供连续任务目标/路径；[cohold.py](../../simulation/mujoco/src/robot_backend_sim/tasks/handover/cohold.py) 从固定实测抓持基准生成共持步骤 | 规划、协调误差与带物净空使用同一命令路径；五类诊断只经标准控制提交目标，不修改执行物块状态 | 数值求解/净空机制留在 `kinematics`，抓持关系与任务路径归 `tasks/handover`；P03 复用已有实现 |
| [contacts.py](../../simulation/mujoco/src/robot_backend_sim/physics/contacts.py) 接收机器人适配提供的两指具名绑定；[physics/contact_snapshot.py](../../simulation/mujoco/src/robot_backend_sim/physics/contact_snapshot.py) 产生每指/物块/支撑/夹爪一致快照 | P02A 已验证空爪反馈与停止；P02B 已验证左右独立持稳、共持全过程、带物停止及最终箱内稳定 | 快照由 owner 线程捕获；P02B 完成诊断的接触/相对运动验收，P03 增加任务生命周期门禁与阶段判定 |
| [registry.py](../../simulation/mujoco/src/robot_backend_sim/tasks/registry.py) 显式注册 stack 与 handover；任务插件实现 Teacher 响应填充 | 机器人/场景不随任务 registry 自动接入 | P03 已增加 handover 生命周期、门禁/评价和 Teacher 能力声明；60×16 约束保留在契约边界 |
| [teacher_extension.py](../../simulation/mujoco/src/robot_backend_sim/ros/teacher_extension.py)、[teacher_client.py](../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/teacher_client.py) 按 `<package>.teacher_chunk.v<N>` 解析生成类型并校验能力 | 新 capability 由方案扩展包声明；通用客户端不导入具体扩展 | P03 已完成两端能力适配；请求/响应转换仍在所属任务模块，handover 交接时刻为可选显式字段 |
| [export_quality.py](../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/export_quality.py) 的阶段顺序、必需阶段与质量规则来自 campaign 配方，区间取实际阶段事件与观测时间；无阶段或必需阶段缺失显式不合格 | 新任务按自己的阶段、对象和相机评价；P04 已用真实渲染测量填写 Marvin 规则 | 保留现有预算/划分机制与 `stack/*/export` 约定；P05 未启用静态样本筛选，现有筛选器仍固定右臂七轴与 stack 阶段，留待确有静态冗余时再修订 |
| [config.py](../../learning/lerobot/src/dual_arm_learning/config.py)、[policy_config.py](../../learning/lerobot/src/dual_arm_learning/policy_config.py) 固定当前 ACT 规格；[sample_filter_rules.py](../../learning/lerobot/src/dual_arm_learning/sample_filter_rules.py) 强制右臂七轴与堆叠阶段 | 不能仅复制 YAML 就把双臂动作筛选正确；短训练默认也不满足正式服务准入 | P05 保留 ACT 规格，换用 Marvin 档案/配方与本次训练统计；不启用静态帧筛选，通过现有 `--no-require-formal` 建立诊断服务准入；正式准入默认值未改 |

这些读取入口不是必须逐文件改写的清单。实现者沿实际调用链修改生产者、消费者及测试，不靠大范围重命名消除表面上的 Panda/stack 字符串。

## 选择、契约与数值来源

- `scenario_id` 使用 `marvin_handover`，仿真 `task_type` 使用 `handover`。它们是方案选择和任务身份；机器人装配由 simulation 配方明确选择，不能用任务名推断机器人型号。
- binding 继续使用 schema v1；Robot Profile 使用现行 schema v2。schema 文件位于 `common/interfaces/schemas/v1/`，该目录名不代表 Robot Profile 可以使用 schema_version 1。
- Robot Profile 声明公开关节与相机，每路相机使用自己的标定及由 64 个十六进制字符组成的 SHA-256 摘要。关节/执行器/指面/TCP 等 MuJoCo 内部绑定由仿真配方与适配持有。
- 现有 binding 不包含机器人构造器或动态 Python 插件字段。P01 已提供 `robot.assembly` 与 `task.environment`；P02A 已由 [`scene_loader.py`](../../simulation/mujoco/src/robot_backend_sim/config/scene_loader.py) 按 `robot.assembly` 选择场景配置，并由 [`robot_adapters.py`](../../simulation/mujoco/src/robot_backend_sim/physics/robot_adapters.py) 装配场景与 I/O。不在 binding 填未知字段或用任意 import 路径加载代码。
- TaskState 复用 `task_type`、生命周期和 `metrics_json`。本任务行为阶段、接触/支撑和稳定指标在任务内定义；runtime 原样记录并解释终态，学习推理不读取这些特权状态。
- Teacher 扩展使用 `marvin_handover_api_msgs.teacher_chunk.v1`，IDL 位于方案 `extensions/ros/`，两端实现放所属端。请求保留 60×16、50 ms、profile、epoch、capture、观测/源时间与 plan/chunk 身份，并新增可选 `handoff_time` 供交接任务对齐调度交接时刻；公共 robot/sim API 与 Policy API 保持现有协议。
- 相同维数不代表相同契约；Panda 的 profile、相机标定、模型、训练统计和接触参数都不作为 Marvin 的默认来源。

## 修复如何算完成

每个实际修复在对应阶段交付中写明：原调用为什么阻碍本次扩展、修改后的职责/数据来源、受影响的 Panda/Marvin 证据。公共路径变化至少覆盖两个方案；专用实现验证本方案即可。

阶段记录区分机器人、任务、算法/协议和机制的实际变更，给出关键调用与数值来源即可。验收使用本次 Panda stack 和 Marvin handover 两个实现组合，以及必要的边界测试；不为证明分层额外实现交叉任务组合或第二算法。

本次调用链中确认的职责越界、重复数值源和任务/机器人分支耦合，在所属阶段完成修正及相关回归。临时接通可以用于诊断，阶段验收同时要求功能和边界修复成立，不能仅以新任务跑通结束阶段。

保留业务参数、约束验证与真实停止原因。首次消除 Panda 硬编码时可以调整机制的真实依赖边界；完成后，已有能力范围内的新增方案应只需选择和参数变化。新增能力的代码集中在对应适配/实现，机制通过已明确的能力边界继续工作。

若出现跨层问题，在所属层修复后继续任务；新增抽象承接当前真实消费者所需能力。hardware 保留现有边界说明，已有清晰职责的模块保留原位。已移除的历史兼容层、旧 schema 或目录不因本任务恢复。

目录导航：[任务书入口](README.md) · [任务判定](task-contract.md) · [P00](phase-00-baseline.md)
