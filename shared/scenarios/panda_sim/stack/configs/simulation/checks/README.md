# 仿真检查配置（Checks）

- **`scene_check.yaml`**：定义三路相机（`global`、`left_wrist`、`right_wrist`）静态可见性检查的布局网格与像素覆盖率门限。
- **`kinematics_check.yaml`**：定义堆叠任务各阶段关键位姿的 IK 求解容差与碰撞净空门限。

检查命令可通过 `simulation` 容器内对应工具模块执行（`python3 -m robot_backend_sim.tools.check_scene` 与 `python3 -m robot_backend_sim.tools.check_kinematics`）。

目录导航：[方案说明](../../../README.md)
