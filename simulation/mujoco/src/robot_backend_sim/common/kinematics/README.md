# kinematics

逆运动学、路径采样和间隙检查。

模块：

- `common.py`：机器人无关的位姿提取与关节路径采样（`ObjectPose`、`extract_object_pose`、`sample_joint_path`、`rotation_down`）。
- `kinematics_clearance.py`：机器人无关的碰撞对枚举与间隙测量。

本子包不依赖 ROS。

目录导航：[上级目录](../../README.md)
