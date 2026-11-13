# P01A 模型资产整理与碰撞复核

状态：2026-11-12 已交付，实际结果见[交付记录](deliveries/phase-01a.md)。本文件保留阶段要求；完成后进入 P02A。

前置：保留 P00/P01 已有成果。目标：得到路径归属清楚、碰撞和联动约束可用于后续运动验证的模型。完成本阶段后进入 P02A。

## 当前输入与目标位置

P01 的原始资产原位于 `shared/scenarios/marvin_handover/assets/marvin/`，包含 1 个 URDF 和 23 个 STL。P01A 已整体移至所属端目标位置：

```text
simulation/mujoco/assets/
├── franka_emika_panda/          # 保持现有位置与内容
└── marvin/
    ├── README.md               # 来源、原始语义与导入差异
    ├── urdf/Marvin_Stand_M6_2026.4.8.urdf
    └── meshes/*.STL
```

机器人本体资产归 simulation，任务方案继续持有档案/标定、场景与运行配方、接口声明。原始 URDF 的来源和格式不要求与 Panda MJCF 相同，但当前唯一消费端和机器人资产归属应一致。

## 资产与路径任务

1. 将项目内现有 Marvin 资产整体移入目标位置，保持原始 URDF/STL 内容，不重复从 Downloads 导入，不保留两份运行时资产。保留源资产说明；P01 历史报告仍描述当时实际路径。
2. 按现有 [assets.py](../../simulation/mujoco/src/robot_backend_sim/physics/assets.py) 的模块定位方式，由 simulation 根据 `robot.assembly: marvin` 定位本端固定版本资产。配置加载与机器人装配共用这一结果；当前 `robot.urdf` 的方案相对路径随迁移移除，不引入另一套资产选择字段。
3. 资产根在宿主机为 `simulation/mujoco/assets/`，在 simulation 容器为 `/app/assets/`，由所属端解析。`/shared` 与 `/app` 是独立挂载，不能靠在 YAML 中增加 `../` 重建宿主机目录关系，也不能硬编码 Downloads 或宿主机绝对路径。
4. 同步场景配置、配置加载器、装配/检查入口、相关测试和现行 README。模型路径只由 simulation 消费，runtime/learning 继续通过公开档案和 API 工作；本次布局不需要新增 Docker 挂载或资产 URI 框架。

## 碰撞与联动任务

1. 检查 [assembly.py](../../simulation/mujoco/src/robot_backend_sim/physics/marvin/assembly.py) 的 `_add_self_exclusions`。P01 时 88 项排除按链路编号批量生成，覆盖大量同臂非相邻连杆及连杆与手指配对。逐项收敛到有装配/运动依据的必要排除；原碰撞凸包不可用时补适合本任务的代理几何。
2. 验证同臂自碰、双臂互碰、腕部/手指、机器人与桌/箱/物块的有效碰撞集合。至少用实际可达的碰撞姿态或已知重叠几何证明必要配对会被检测，不能仅统计“跨臂排除数为 0”。底座目前无碰撞；依据 P02 候选运动区域决定是否补代理，并记录适用边界。
3. 枚举编译模型全部 equality 的类型、对象和参数，解释 P01 的 `neq=4` 与源 mimic、手工联动的关系。确认冗余后保留一套有效联动约束，不先假定四条都必要，也不把固定数量 4 当成正确性条件。
4. 检查两指内侧面、TCP 与总开度映射。P01 开端实测约 `0.08143/0.08142 m`，profile 为 `0.0814 m`；明确测量、命令范围和数值容差的区别，保留实测反馈，不通过裁剪状态制造一致。

## 验证与完成条件

- 宿主机能解析目标资产路径；在有 MuJoCo 依赖的宿主机环境进行编译检查，否则宿主机只验证定位，完整编译/步进/渲染在现有 simulation 容器执行。
- 容器内仍从 `/shared` 读取方案配方，从 `/app/assets/marvin` 读取模型；不存在旧方案资产依赖和重复副本。
- 重新取得模型编译、静置、低幅运动、夹爪三态及三路图像证据。原始质量/惯性语义对照保留；若编译约束或代理几何改变，报告实际差异，不沿用旧固定模型计数作断言。
- 必要碰撞配对检测有效，联动约束来源清楚；Panda 场景/复位及本次涉及的共享机制回归通过。

报告存入新目录，不覆盖 `p01-final7-20261112`。交付 `deliveries/phase-01a.md`，记录移动前后位置、实际路径解析、有效碰撞集合、联动结论、模型/渲染结果和受影响回归。P01 的离线成果保留，P01A 完成后才将模型交给标准控制接入。

目录导航：[任务书入口](README.md) · [P01 基础模型](phase-01-model.md) · [下一阶段 P02A](phase-02-control.md)
