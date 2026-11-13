# kinematics

逆运动学、路径采样和间隙检查。

模块：

- `common.py`：机器人无关的位姿提取与关节路径采样（`ObjectPose`、`extract_object_pose`、`sample_joint_path`、`rotation_down`）。
- `dual_arm.py`：机器人无关的双臂顺序 IK 与载物路径间隙检查。
- `kinematics_clearance.py`：碰撞对枚举与间隙测量。
- `kinematics.py`：Panda 右臂关键位姿 IK 与间隙诊断，供 `tasks/stack/` 与 `tools/check_kinematics.py` 使用。

本子包不依赖 ROS。

目录导航：[上级目录](../README.md)
