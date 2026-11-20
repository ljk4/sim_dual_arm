# 仿真端（Simulation）

本目录按物理仿真引擎组织机器人后端实现。MuJoCo 后端负责 Marvin 双臂动力学步进、三路 EGL 相机渲染、提交轨迹的安全校验与执行、接触判定和程序示教。当前方案见 [`marvin_sim/stack`](../shared/scenarios/marvin_sim/stack/README.md) 与 [`marvin_sim/handover`](../shared/scenarios/marvin_sim/handover/README.md)，公共 ROS 接口见 [`shared/common/interfaces/ros/`](../shared/common/interfaces/ros/README.md)。

目录导航：[上级目录](../README.md)
