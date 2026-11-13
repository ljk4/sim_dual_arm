# P01 模型、场景与档案

P01 基础模型、场景、档案和离线结果已交付，见[实际记录](deliveries/phase-01.md)。本文件规定模型与场景职责；当前资产位置、碰撞排除与联动约束已由 [P01A](phase-01-readiness.md) 补齐，见[交付记录](deliveries/phase-01a.md)。

## 本阶段的职责拆分

通用编译/复位机制留在 simulation；Marvin 本体、执行器、手指联动和 TCP 属于机器人装配适配；桌面、长方块、箱体和布局属于 handover 任务环境。装配入口组合两者，不以 `task_type=handover` 推断机器人型号，也不在机器人装配器中写任务物块和交接位姿。

任务环境构造与 P03 的 Teacher/任务生命周期分开，P01 可先离线编译、步进和渲染基础场景。保持已有职责清楚的模块，不要求先迁移所有 Panda 文件或建设通用场景插件系统。

## 文件与数值组织

模型资产位于 `simulation/mujoco/assets/marvin/`，由 simulation 的 `physics/assets.py` 按 `robot.assembly` 定位；P01A 已完成整理。方案布局如下，文件按实际消费者需要建立：

```text
shared/scenarios/marvin_handover/
├── README.md
├── binding.yaml
├── profiles/
│   ├── robots/                     # schema v2 profile、相机标定
│   ├── observations/               # RGB + measured joint state
│   ├── actions/                    # joint_position
│   └── control/                    # 本模型 control/stream/follower 数值
├── configs/
│   ├── simulation/                 # scene、backend、task、kinematics
│   ├── runtime/                    # teacher、policy、diagnostic campaign
│   └── learning/                   # diagnostic dataset/training、policy
└── extensions/ros/                 # P03 实现的 Teacher IDL
```

P01/P01A 建立可用模型与档案；P02A 使用无任务插件 direct backend。P03 为首次标准 Teacher 示教补齐采集侧配方、binding、IDL、两端适配和 SDK，并通过 collection 预检；P04/P05 再加入实际数据/学习/推理引用。所有声明的引用必须已经存在。

## 模型与场景任务

1. 将[源资产](model-notes.md)作为 simulation 的机器人资产管理，保留原始几何、关节与惯性语义。处理 `package://` 引用及 fixed 关节无效 limit 等导入适配；输入来源和必要变换写资产说明。
2. 建立一个固定根 Marvin 装配器，使用本机实际 MuJoCo 版本支持的导入/构造方式。先检查导入后联动，再补所需执行器/约束，避免源 mimic 与手工约束重复；不复制两份完整机器人来拼出双臂。
3. 在 simulation 配置中引入足以选择 Panda/Marvin 装配的明确字段或等价的现有选择点，接通配置加载与构造；独立选择 handover 任务环境。通用场景容器接收构造结果，场景的物块、桌面、箱体参数由任务环境配方提供。
4. 为必要碰撞建立可用的代理几何或经验证的网格。保留必要同臂自碰、双臂互碰和手指接触，排除项须有几何依据；箱体按底板与四壁构造，验证开口没有被碰撞凸包封闭。记录与源碰撞网格的实际差异。
5. 通过模型测量确定物块/箱体初版尺寸、桌高、基座姿态、TCP 与指面。初选抓取区与双臂重叠区，P02 再确认完整路径可达。
6. 实现确定性复位：机器人、物块、控制目标及任务相关状态回到一致初态。显式绑定 `qpos/dof/actuator` 地址，移除新路径对 Panda `nq/nv/nu` 和地址邻接的依赖。

## 档案与相机任务

首版复用现有 ACT 规格：16 维状态/位置动作，20 Hz，动作相对观测领先 50 ms，60 点动作块；三路 RGB 为 `global / left_wrist / right_wrist`，均为 320×240。这是现有学习实现的范围，不代表任意相机或维数已被支持。

- 公开顺序固定为左臂 7 关节、左夹爪总开度、右臂 7 关节、右夹爪总开度。Marvin 原始关节名到公开名称的映射写明，名称可保持现有语义，但 robot/profile 身份使用 Marvin 自身标识。
- 臂关节使用 rad，夹爪总开度使用 m，速度与限位来自新模型及保守执行配置。建立总开度到指位移/执行器命令的正反变换，测量开端、中点与闭端；不沿用 Panda 的 ctrl 范围。
- Robot Profile 使用 schema_version 2、`cameras` 与每路标定摘要；不恢复旧版相机字段/兼容转换。Robot/observation/action 三份档案对关节顺序、相机和身份一致。
- 安装全局和双腕相机，定义 RGB 通道、光学 frame、内参、安装变换与标定文件。全局相机覆盖桌面、交接区和箱口；腕相机在相关抓持阶段能提供实际物块图像，P04 再定像素门槛。
- 仅在源资产与初版场景验证后定稿数值；公开 profile 的有效限位和相机声明须与编译模型/渲染输出一致。

## 验证与完成条件

模型可编译并按配置重置，离线静置步进无 NaN、发散或明显装配错误；单关节低幅运动方向、范围与手指联动正确。记录初态和夹爪开/中/闭三态的测量及双臂装配视图。

三路图像尺寸、内容与相机声明相符，标定被档案校验接受。渲染成功与碰撞有效分别报告；本阶段不将离线模型检查写成完整控制或交接验证。

机器人装配不依赖 handover 阶段/评价器，任务环境通过声明的机器人挂载/工作空间事实组合场景，不读取具体 I/O 内部。通用编译/复位/渲染机制继续服务 Panda 与 Marvin，新增方案的选择和参数来自配方。

Panda 原模型构造与复位相关回归应成立。已有 `deliveries/phase-01.md` 保留当时模型和结果；P01A 另存整理后的实际证据，P02B 再确定完整候选路径和真实持物能力。

目录导航：[任务书入口](README.md) · [P00](phase-00-baseline.md) · [下一阶段 P01A](phase-01-readiness.md) · [P02 总览](phase-02-motion.md)
