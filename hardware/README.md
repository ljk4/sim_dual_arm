# 真机硬件接口边界（Hardware）

本目录预留用于真机硬件驱动接入。当前阶段仅确立与仿真端共享的统一 Robot API 边界，尚未包含具体真机驱动实现。

## 统一 Robot API 约束

真机后端与 MuJoCo 仿真后端（[`simulation/mujoco/`](../simulation/mujoco/README.md)）必须实现完全一致的公共契约：

- **通信接口**：ROS 2 接口包 [`contracts/ros/robot_api_msgs/`](../contracts/ros/robot_api_msgs/README.md)。
- **机器人语义与限位**：[`profiles/robots/`](../profiles/robots/README.md)、[`profiles/observations/`](../profiles/observations/README.md)、[`profiles/actions/`](../profiles/actions/README.md) 与 [`profiles/control/`](../profiles/control/README.md)。
- **运行端解耦**：运行端（[`runtime/ros2/`](../runtime/ros2/README.md)）仅通过标准 ROS 2 服务与话题访问后端，不导入任何具体物理引擎或硬件驱动代码。

## 接入准则

- 真机的设备 IP、序列号、账号凭证与单机标定参数属于本地私有配置，严禁提交至代码仓库。
- 在确认具体机械臂/夹爪硬件驱动、开源许可与安全急停边界前，不在本目录创建占位空模块或假驱动。

目录导航：[上级目录](../README.md)
