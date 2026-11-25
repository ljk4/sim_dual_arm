# Marvin 机器人公共资产（`shared/robots/marvin/`）

本目录是 Marvin 双臂机器人的跨后端与跨任务公共资产根目录，包含原始 URDF、STL 网格、共享观测/动作/控制档案（`profiles/`）及双臂共享 Teacher ROS 2 扩展包（`extensions/ros/marvin_teacher_api_msgs/`），供 `simulation/`（如 MuJoCo）与未来其他后端/真机（如 Isaac Sim、天机 `hardware/tianji`）统一按 `robot.assembly: marvin` 引用。

## 来源

```text
/home/ljk/Downloads/Marvin-Pro-M6_URDF_wGripper&FTSensor/Marvin_Stand_M6_2026.4.8
├── urdf/Marvin_Stand_M6_2026.4.8.urdf
└── meshes/*.STL
```

源包保留原样作为单一可信源。

## 目录结构

- `urdf/Marvin_Stand_M6_2026.4.8.urdf`：23 个 link、22 个 joint；每臂 7 个旋转关节、2 个滑动手指关节。
- `meshes/`：23 个二进制 STL，均被 URDF 的 visual 和 collision 引用。
- `profiles/`：跨任务、跨后端共享的观测（`observations/`）、动作（`actions/`）与控制/跟随参数档案（`control/`）。
- `extensions/ros/marvin_teacher_api_msgs/`：Marvin 共享 60×16 Teacher chunk ROS 2 接口扩展包。

## 导入适配（MuJoCo 后端实现位于 `simulation/mujoco`）

1. **固定根**：`ZJ_Robot_link` 不添加自由关节，保持固定底座。
2. **网格路径**：URDF 的 `package://.../meshes/<name>.STL` 相对路径在装配时由 `simulation/mujoco/src/robot_backend_sim/common/physics/assets.py` 解析到 `shared/robots/marvin/meshes/`；源文件不改写。
3. **底座网格拆分**：`ZJ_Robot_link.STL` 有 216,087 个三角面，超过 MuJoCo 单文件 200,000 面上限。装配器在内存中将其拆分为两个 `uservert/userface` 网格，源 STL 保持原样。
4. **固定关节限位**：`Arm_R8_Joint` 为固定关节，编译模型包含 14 个旋转与 4 个滑动自由度。
5. **手指联动**：每侧第 11 关节 mimic 第 10 关节，由 URDF 导入器生成对应 joint equality 约束（`neq=2`）。
6. **碰撞与桌面范围**：同臂排除仅保留 `Arm_*5_Link`/`Arm_*7_Link`，底座网格仅用于渲染；若动作进入底座中轴 0.15 m 内须先补代理碰撞。

## 仿真模型与实验室天机真机的已知差异（Sim2Real 注意事项）

当前 `urdf/Marvin_Stand_M6_2026.4.8.urdf` 与实验室正在接入的天机真机之间存在以下尚未对齐的物理差异（各方案通过独立 `RobotProfile` 隔离，详情见 [架构扩展与演进指南](../../docs/features/架构扩展与演进指南.md)）：

- **夹爪与腕部相机设备不同**：实验室天机真机实际搭载大寰 90C 夹爪、手腕 RealSense 相机与 4 路视触觉传感器，末端载荷、几何包络与腕部相机视场不同于当前 URDF 默认夹爪与仿真相机。
- **物理尺寸与外参尚未标定**：双臂基座相对安装位姿、桌面实际高度及手眼标定矩阵尚未完成实测校准，跨后端策略迁移需待标定与对齐后再行开展。

## 未改动内容

- 惯性质量与惯量矩阵按 URDF 导入；离线检查逐 link 对照质量与主惯量。
- 关节原点、轴向、限位、几何单位均来自源文件；底座（`ZJ_Robot_link`）及 1–7 关节壳体在 URDF 与 `assembly.py` 中统一设为白色 `[0.92, 0.92, 0.92, 1.0]`，8–11 腕末端与指爪设为黑色 `[0.12, 0.12, 0.12, 1.0]`。
- `Arm_R11_Link` 的 `y=1.793` 关节原点和大网格偏移保留；手指内侧面位置由源网格包围盒测量后写入仿真 site。
