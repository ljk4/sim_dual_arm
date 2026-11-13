# Marvin 源资产

## 来源

```text
/home/ljk/Downloads/Marvin-Pro-M6_URDF_wGripper&FTSensor/Marvin_Stand_M6_2026.4.8
├── urdf/Marvin_Stand_M6_2026.4.8.urdf
└── meshes/*.STL
```

2026-11-12 原样复制到项目；P01A 移至本目录 `simulation/mujoco/assets/marvin/`，作为唯一的运行时资产。P01 历史报告中的 `shared/scenarios/marvin_handover/assets/marvin/` 描述的是当时路径。源包没有附带许可证或使用条款文件；本仓库只保存原始文件和转换说明，不重新分发为独立产品。

## 文件

- `urdf/Marvin_Stand_M6_2026.4.8.urdf`：23 个 link、22 个 joint；每臂 7 个旋转关节、2 个滑动手指关节。
- `meshes/`：23 个二进制 STL，均被 URDF 的 visual 和 collision 引用。

## 导入适配（实现位于 `simulation`）

1. **固定根**：`ZJ_Robot_link` 不添加自由关节，保持固定底座。
2. **网格路径**：URDF 的 `package://.../meshes/<name>.STL` 相对路径在装配时解析到本目录的 `meshes/`；源文件不改写。资产由 simulation 的 `physics/assets.py` 按 `robot.assembly: marvin` 定位，不依赖方案文件相对路径或宿主机绝对路径。
3. **底座网格拆分**：`ZJ_Robot_link.STL` 有 216,087 个三角面，超过 MuJoCo 单文件 200,000 面上限。装配器在内存中把它拆成两个 `uservert/userface` 网格，几何不简化；源 STL 保持原样。编译模型有 24 个网格（底座 2 个，其余 22 个）。
4. **固定关节限位**：`Arm_R8_Joint` 在源 URDF 中带 `lower=1.5708, upper=-1.5708, velocity=0` 的无效 limit。它是固定关节，导入后不产生自由度；编译模型只有 14 个旋转和 4 个滑动关节。
5. **手指联动**：源 URDF 每侧第 11 关节 mimic 第 10 关节（倍率 1、偏移 0）。MuJoCo 3.17 的 URDF 导入器已为每侧生成一条 joint equality；装配器只命名该约束并写入经验证的 `solref`/`solimp`，不再添加第二条。P01 的手工重复约束在 P01A 移除，编译模型 `neq=2`。
6. **碰撞**：机器人 link 的 collision 网格保留并参与碰撞。同臂排除仅保留 `Arm_*5_Link`/`Arm_*7_Link`：两者源凸包在复位位姿和抽样可达位姿中永久重叠约 0.061 m，中间的 `Arm_*6` 又使原始零件实际分离；其余同臂配对、左右臂配对和四指碰撞均保留。P01 的 88 项批量排除在 P01A 收敛为这 2 项。
7. **底座碰撞边界**：底座网格仅用于渲染，本阶段不补碰撞代理。任务桌面位于 x≥0.40 m，P01A 的复位、低幅运动与碰撞探针不要求底座参与碰撞。若 P02B 候选路径进入底座中轴约 0.15 m 范围，应先补一个经验证的代理几何，而不是启用 216k 面的粉碎凸包。
8. **视觉/碰撞差异**：URDF 对 visual 和 collision 使用同一 STL，MuJoCo 导入为每 link 一个网格 geom，因此本方案没有单独的视觉简化网格；底座是唯一显式视觉 geom。

## 未改动内容

- 惯性质量与惯量矩阵按 URDF 导入；离线检查逐 link 对照质量与主惯量。
- 关节原点、轴向、限位、几何单位均来自源文件。
- `Arm_R11_Link` 的 `y=1.793` 关节原点和大网格偏移保留；手指内侧面位置由源网格包围盒测量后写入仿真 site。
