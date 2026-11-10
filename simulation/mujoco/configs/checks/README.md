# 仿真检查配置（Checks）

- **`scene_check.yaml`**：定义三路相机（`global`、`left_wrist`、`right_wrist`）静态可见性检查的布局网格与像素覆盖率门限。
- **`kinematics_check.yaml`**：定义堆叠任务各阶段关键位姿的 IK 求解容差与碰撞净空门限。

检查命令通过容器内 [`scripts/run.sh`](../../scripts/README.md) 的 `scene` 与 `kinematics` 子命令执行。

目录导航：[上级目录](../README.md)
