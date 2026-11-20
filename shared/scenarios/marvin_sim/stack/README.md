# marvin_sim/stack

Marvin 固定基座双臂平台上的单臂堆叠方案（右臂将红色 4 cm `source` 方块抓起并堆放到蓝色 6 cm `target` 方块上，左臂保持安全初始姿态；支持 `±0.03 m` 平面随机化与 `±10°` 偏航随机化）。本目录保存方案绑定、机器人/观测/动作档案、仿真/运行/学习配方与方案专用 Teacher 扩展接口。

## 公开契约

- 控制轴顺序（16 维）：`left_joint1..left_joint7`、`left_gripper_width`、`right_joint1..right_joint7`、`right_gripper_width`。
- Teacher 动作块网格：`60 点 × 16 维`（50 ms 步长），能力标识 `marvin_stack_api_msgs.teacher_chunk.v1`。
- 相机名称：`global`、`left_wrist`、`right_wrist`（均为 `320×240` RGB，摘要随 `configs/simulation/scene/marvin_stack.yaml` 同步哈希）。
- 臂关节限位来自源 URDF；夹爪总开度范围为 `0.0014 .. 0.0814 m`。

## 主要文件

| 路径 | 说明 |
| --- | --- |
| `binding.yaml` | 方案绑定与三端配方选择（`--robot marvin_sim --task stack`） |
| `extensions/ros/marvin_stack_api_msgs/` | `marvin_stack_api_msgs` ROS 2 IDL 扩展 |
| `profiles/` | `marvin_stack_v1` 机器人档案、三路相机标定与 16 维控制参数 |
| `configs/simulation/scene/marvin_stack.yaml` | 物理、Marvin 装配参数、桌块与三路相机声明 |
| `configs/simulation/scene/marvin_stack_task.yaml` | 14 项堆叠成功判据、阶段门禁与 Teacher 参数 |
| `configs/simulation/checks/` | 67 布局静态可见性与运动学/净空检查配置 |
| `configs/simulation/backend/` | direct / teacher / policy 三类后端启动配方 |
| `configs/runtime/` | Teacher 采集、Policy 推理、阶段评估与 50 条正式 campaign 预算 |
| `configs/learning/` | ACT 45/5 数据转换、采样过滤、训练与推理服务配置 |
