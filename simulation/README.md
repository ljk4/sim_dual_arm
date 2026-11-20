# 仿真端（Simulation）

本目录按物理仿真引擎组织机器人后端实现。当前唯一实现为 MuJoCo（[`mujoco/`](mujoco/README.md)），负责 Marvin 双臂动力学步进、三路 EGL 相机渲染、Ruckig 有界跟随规划、接触判定与程序示教。当前方案的档案、配方与选择见 [`marvin_sim/stack`](../shared/scenarios/marvin_sim/stack/README.md) 与 [`marvin_sim/handover`](../shared/scenarios/marvin_sim/handover/README.md)，公共 ROS 接口见 [`shared/common/interfaces/ros/`](../shared/common/interfaces/ros/README.md)。

目录导航：[上级目录](../README.md)
