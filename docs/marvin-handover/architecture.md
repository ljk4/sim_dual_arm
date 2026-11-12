# 架构边界与当前接入缺口

依据 2026-11-12 工作树中的源码核对。本文件列出当前限制及实施要求；现有 Panda 专用实现对原方案有效，是否提取共享部分由本次真实调用关系决定。

## 端与方案的职责

| 归属 | 持有内容 | 对外边界 |
| --- | --- | --- |
| `shared/common/` | 跨端接口、档案 schema、公共轨迹/反馈结构、通用工具 | 公共生产包不反向导入四端；保持原有身份与约束验证 |
| `shared/scenarios/marvin_handover/` | 源机器人资产、Marvin 档案/标定、四端配方、方案绑定及专用 IDL | 以声明与资产为主，数值由所属档案/配方维护，不放运行端业务实现 |
| `simulation/mujoco/` | 模型装配、机器人 I/O、碰撞/接触、动力学、IK、任务插件、Teacher 与物理评价 | 持有 `MjModel/MjData` 与物理步进；对外发布不可变状态、服务结果和反馈 |
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

`shared/scenarios/<id>/` 保存选择与参数，通过档案和各端配方选择机器人、任务实现、算法与模型。源 URDF/网格和方案专用 IDL 可随方案保存，它们是声明性资源；Teacher、驱动、模型装配和任务评价等业务代码仍归各端。binding 现有字段能表达的选择直接复用；机器人构造选择由 simulation 配方承接，不向 binding 填入尚未支持的字段。

适配创建与任务选择在所属端的装配入口完成，通用执行循环消费已选择的能力和事实。换任务不能要求重写机器人 I/O，换机器人不能通过复制整套任务/控制链或在同步、步进、录制主循环中增加方案名分支完成。职责已经清楚的模块保留原位，目录调整服务于真实依赖修复。

## 当前源码依据与修复任务

| 当前事实与读取入口 | 对 Marvin 的影响 | 最小修复方向 / 阶段 |
| --- | --- | --- |
| [scene_config.py](../../simulation/mujoco/src/robot_backend_sim/config/scene_config.py)、[scene.py](../../simulation/mujoco/src/robot_backend_sim/physics/scene.py) 固定 Panda 限位、双独立基座、两个物块和模型尺寸检查 | Marvin 是一个共享根的双臂 URDF，场景还需要长方块与开口箱体 | 分清通用编译/复位、机器人装配与任务环境构造；增加独立 Marvin 装配和 handover 环境，通过配方选择；P01 |
| [panda_io.py](../../simulation/mujoco/src/robot_backend_sim/physics/panda_io.py)、[worker_owner.py](../../simulation/mujoco/src/robot_backend_sim/runtime/worker_owner.py) 直接构造 PandaIO；[control.py](../../simulation/mujoco/src/robot_backend_sim/control/control.py) 检查具体 PandaIO 类型 | 改 profile 不能换掉执行器、指关节或总开度语义 | 在使用边界提取两种机器人实际需要的具名 I/O 能力，创建独立适配；owner/controller 接收所选适配；P02 |
| [motion_config.py](../../simulation/mujoco/src/robot_backend_sim/config/motion_config.py)、[follower_config.py](../../simulation/mujoco/src/robot_backend_sim/config/follower_config.py) 强制 Panda 16 维名称/顺序 | 维数相同仍不能套用 Panda 限位与索引 | 配置按所选 profile 的名称、单位、限位和执行角色绑定；保留维数与数值验证；P02 |
| [kinematics.py](../../simulation/mujoco/src/robot_backend_sim/kinematics/kinematics.py) 按 `right_joint1..7` 求解并使用 Panda 限位；[kinematics_clearance.py](../../simulation/mujoco/src/robot_backend_sim/kinematics/kinematics_clearance.py) 按名称前缀及 source/target 分类 | 左臂抓取和双臂交接不能只替换 IK 目标；原名 Marvin 几何可能漏检 | 由机器人/场景显式提供活动关节、TCP、机器人和物体几何分组；任务决定两臂目标及允许接触；P02 |
| [contacts.py](../../simulation/mujoco/src/robot_backend_sim/physics/contacts.py) 绑定 Panda 手指；[gripper_contact.py](../../simulation/mujoco/src/robot_backend_sim/physics/gripper_contact.py) 固定夹爪索引 7/15 | 同一接触算法需要新的几何与关节绑定；全局“某手持物”不足以判断交接 | 机器人提供指面/关节绑定，通用代码测量接触事实，任务确认两手持有同一物块与支撑关系；P02/P03 |
| [registry.py](../../simulation/mujoco/src/robot_backend_sim/tasks/registry.py) 只支持 stack；[base.py](../../simulation/mujoco/src/robot_backend_sim/tasks/base.py) 的 TaskChunk 仍描述 60×16，插件还含响应填充方法 | 已有任务协议能复用，但任务抽象仍带固定 Teacher 与传输细节 | 新增 handover 插件；本次保留 60×16，ROS 填充放转换/适配模块，核心插件产出不可变事实；P03 |
| [teacher_extension.py](../../simulation/mujoco/src/robot_backend_sim/ros/teacher_extension.py)、[teacher_client.py](../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/teacher_client.py) 实际只加载 panda_stack_api_msgs | binding 声明新 capability 不会自动创建可用客户端或服务 | 将具体扩展类型加载/转换与通用采集流程分开，两端按显式 capability 选择适配；P03/P04 |
| [export_quality.py](../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/export_quality.py) 固定堆叠 PHASES；[collect_attempt.py](../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/collect_attempt.py) 输出含 stack 路径；[campaign.py](../../runtime/ros2/ros/dual_arm_runtime/dual_arm_runtime/offline/campaign.py) 筛选说明仍写 50 条 | 新阶段可能丢失或按 global 规则误判；路径与说明可能带入旧任务语义 | 阶段集合/质量要求随配方与任务事实传入，输出位置由生产消费双方一致选择，报告使用实际数量；P04 |
| [config.py](../../learning/lerobot/src/dual_arm_learning/config.py)、[policy_config.py](../../learning/lerobot/src/dual_arm_learning/policy_config.py) 固定当前 ACT 规格；[sample_filter_rules.py](../../learning/lerobot/src/dual_arm_learning/sample_filter_rules.py) 强制右臂七轴与堆叠阶段 | 不能仅复制 YAML 就把双臂动作筛选正确；短训练默认也不满足正式服务准入 | 首版保留 ACT 规格，换新档案/统计；不启用静态帧筛选，采用现有诊断数据与非正式服务准入；P05 |

这些读取入口不是必须逐文件改写的清单。实现者沿实际调用链修改生产者、消费者及测试，不靠大范围重命名消除表面上的 Panda/stack 字符串。

## 选择、契约与数值来源

- `scenario_id` 使用 `marvin_handover`，仿真 `task_type` 使用 `handover`。它们是方案选择和任务身份；机器人装配由 simulation 配方明确选择，不能用任务名推断机器人型号。
- binding 继续使用 schema v1；Robot Profile 使用现行 schema v2。schema 文件位于 `common/interfaces/schemas/v1/`，该目录名不代表 Robot Profile 可以使用 schema_version 1。
- Robot Profile 声明公开关节与相机，每路相机使用自己的标定及由 64 个十六进制字符组成的 SHA-256 摘要。关节/执行器/指面/TCP 等 MuJoCo 内部绑定由仿真配方与适配持有。
- 现有 binding 不包含机器人构造器或动态 Python 插件字段。需要增加的仿真配置由 P01 在所属端实现后写入配方；不在 binding 填未知字段或用任意 import 路径加载代码。
- TaskState 复用 `task_type`、生命周期和 `metrics_json`。本任务行为阶段、接触/支撑和稳定指标在任务内定义；runtime 原样记录并解释终态，学习推理不读取这些特权状态。
- Teacher 扩展拟使用 `marvin_handover_api_msgs.teacher_chunk.v1`，IDL 放方案 `extensions/ros/`，两端实现放所属端。公共 robot/sim API 与 Policy API 优先保持现有协议。
- 相同维数不代表相同契约；Panda 的 profile、相机标定、模型、训练统计和接触参数都不作为 Marvin 的默认来源。

## 修复如何算完成

每个实际修复在对应阶段交付中写明：原调用为什么阻碍本次扩展、修改后的职责/数据来源、受影响的 Panda/Marvin 证据。公共路径变化至少覆盖两个方案；专用实现验证本方案即可。

阶段记录区分机器人、任务、算法/协议和机制的实际变更，给出关键调用与数值来源即可。验收使用本次 Panda stack 和 Marvin handover 两个实现组合，以及必要的边界测试；不为证明分层额外实现交叉任务组合或第二算法。

本次调用链中确认的职责越界、重复数值源和任务/机器人分支耦合，在所属阶段完成修正及相关回归。临时接通可以用于诊断，阶段验收同时要求功能和边界修复成立，不能仅以新任务跑通结束阶段。

保留业务参数、约束验证与真实停止原因。首次消除 Panda 硬编码时可以调整机制的真实依赖边界；完成后，已有能力范围内的新增方案应只需选择和参数变化。新增能力的代码集中在对应适配/实现，机制通过已明确的能力边界继续工作。

若出现跨层问题，在所属层修复后继续任务；新增抽象承接当前真实消费者所需能力。hardware 保留现有边界说明，已有清晰职责的模块保留原位。已移除的历史兼容层、旧 schema 或目录不因本任务恢复。

目录导航：[任务书入口](README.md) · [任务判定](task-contract.md) · [P00](phase-00-baseline.md)
