# marvin_sim/stack

右臂将红色 4 cm `source` 方块堆放到蓝色 6 cm `target` 方块上，左臂保持安全初始姿态。每局复位到场景配方声明的固定方块位置和朝向，`seed` 用于标识运行。采集与评估衡量固定布局中的表现。

## 公开契约

- 控制轴顺序（16 维）：`left_joint1..left_joint7`、`left_gripper_width`、`right_joint1..right_joint7`、`right_gripper_width`。
- Teacher 动作块网格：`60 点 × 16 维`（50 ms 步长），能力标识 `marvin_teacher_api_msgs.teacher_chunk`。
- 相机名称：`global`、`left_wrist`、`right_wrist`（均为 `320×240` RGB，摘要随 `configs/simulation/scene/marvin_stack.yaml` 同步哈希）。
- 臂关节限位来自源 URDF；夹爪总开度范围为 `0.0014 .. 0.0814 m`。

## 主要文件

| 路径 | 说明 |
| --- | --- |
| `binding.yaml` | 方案绑定与三端配方选择（`--robot marvin_sim --task stack`） |
| `../extensions/ros/marvin_teacher_api_msgs/` | `marvin_sim` 共享 `marvin_teacher_api_msgs` ROS 2 IDL 扩展 |
| `profiles/` | `profiles/robot/profile.json` 与本任务的连续流超时 |
| `configs/simulation/scene/marvin_stack.yaml` | 物理、Marvin 装配参数、桌块与三路相机声明 |
| `configs/simulation/scene/marvin_stack_task.yaml` | 14 项堆叠成功判据、阶段门禁与 Teacher 参数 |
| `configs/simulation/scene/stack_paths.yaml` | 右臂抓取、搬运与堆叠路径规划参数 |
| `configs/simulation/backend/` | teacher / policy 两类后端启动配方 |
| `configs/runtime/` | Teacher 采集、Policy 推理、阶段评估与 50 条正式 campaign 预算 |
| `configs/learning/` | ACT 45/5 数据转换、采样过滤、训练与推理服务配置 |

堆叠与交接共用 Marvin 的 [观测档案](../profiles/observations/marvin_rgb_joint.json)、[动作档案](../profiles/actions/marvin_joint_position.json)、[运动限值](../profiles/control/marvin_control.yaml)、[Teacher 跟随参数](../profiles/control/marvin_follower.yaml)和 [Policy 跟随参数](../profiles/control/marvin_follower_inference.yaml)。方案绑定与启动配方直接引用这五份文件。
