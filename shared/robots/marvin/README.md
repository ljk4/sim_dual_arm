# Marvin 机器人公共资产（`shared/robots/marvin/`）

本目录是 Marvin / 天机双臂机器人的跨后端与跨任务公共资产根目录，包含双臂 URDF、STL 网格、共享观测/动作/控制档案（`profiles/`）及双臂共享 Teacher ROS 2 扩展包（`extensions/ros/marvin_teacher_api_msgs/`），供 `simulation/mujoco` 与真机 `hardware/tianji` 统一按 `robot.assembly: marvin` 引用。

## 目录结构

- `urdf/Marvin_Stand_M6_2026.4.8.urdf`：底座 `ZJ_Robot_link`、双侧 7 自由度机械臂（`Arm_{L,R}1_Link`..`Arm_{L,R}7_Link`），以及与真机末端硬件对齐的 `Arm_{L,R}_gripper_base`、`Arm_{L,R}_finger1`、`Arm_{L,R}_finger2`（行程 `0 .. 0.045 m`，对应知行 `CTAG2F90C` `90 mm` 满行程）。
- `meshes/`：底座与 1–7 轴机械臂二进制 STL 网格；可选末端可视化 STL 可放于 `meshes/gripper/`。
- `profiles/`：跨任务、跨后端共享的观测（`observations/marvin_rgb_joint.json`：`cam_high`、`cam_left_wrist`、`cam_right_wrist`）、动作（`actions/marvin_joint_position.json`）与控制/跟随参数档案（`control/`）。
- `extensions/ros/marvin_teacher_api_msgs/`：Marvin 共享 60×16 Teacher chunk ROS 2 接口扩展包。

## MuJoCo 装配语义（`simulation/mujoco`）

1. **固定根与底座网格拆分**：`ZJ_Robot_link` 固定在世界坐标系；`ZJ_Robot_link.STL`（216,087 三角面）在内存中拆分为两个视觉网格以适配 MuJoCo 200k 面限制，底座网格仅作渲染。
2. **知行 `CTAG2F90C` 55 mm 平行四连杆与柔顺指垫**：
   - `assembly.py` 为每根手指注入侧向闭合自由度（`Arm_{L,R}_finger{1,2}_Joint` ∈ `[0, 0.045] m`）与轴向伸长自由度（`Arm_{L,R}_axial{1,2}_Joint`），并通过四次多项式 `mjEQ_JOINT` 约束精确复现 `55.00 mm` 连杆闭合前伸弧线（共 `neq=6` 约束，`nq=29, nv=28`）。
   - 末端装配 `6 mm` 腕部相机板、`CTAG2F90C` 本体、3D 打印指座、DeepTouch 散斑视触觉壳体及内凸 `24×24×4 mm` 柔顺硅胶指垫（`condim=4`、`solref=(0.006, 1.0)`、`solimp=(0.90, 0.99, 0.0015, 0.5, 2.0)`）。
   - 支持配置 `pad_inward_offset_m` 以描述实测空夹触碰零位偏置，`gripper_opening()` 返回与 `CTAG2F90C` `0.090 m` 满行程一致的开口读数（`0.0 .. 0.090 m`），`pad_aperture_m()` 返回指垫物理内间距。
3. **碰撞与外观**：同侧排除保留 `Arm_*5_Link`/`Arm_*7_Link` 与 `Arm_*_finger1`/`Arm_*_finger2`（共 4 组）。底座及 1–7 关节为白色 `[0.92, 0.92, 0.92, 1.0]`，夹爪本体与指垫为深色，指座区分蓝/白两色便于视觉辨识。
