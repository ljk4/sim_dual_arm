# 真机硬件接口边界（Hardware）

本目录预留用于真机硬件驱动接入。当前阶段仅确立与仿真端共享的统一 Robot API 边界，尚未包含具体真机驱动实现；[`shared/scenarios/panda_stack/binding.yaml`](../shared/scenarios/panda_stack/binding.yaml) 中的 `backend.kind=hardware` 只是结构取值，runtime 采集仍拒绝非 simulation 后端。

## 统一 Robot API 约束

真机后端与 MuJoCo 仿真后端（[`simulation/mujoco/`](../simulation/mujoco/README.md)）必须实现完全一致的公共契约：

- **通信接口**：ROS 2 接口包 [`shared/common/interfaces/ros/robot_api_msgs/`](../shared/common/interfaces/ros/robot_api_msgs/README.md)。
- **机器人语义与限位**：[`shared/scenarios/panda_stack/profiles/robots/`](../shared/scenarios/panda_stack/profiles/robots/README.md)、[`observations/`](../shared/scenarios/panda_stack/profiles/observations/README.md)、[`actions/`](../shared/scenarios/panda_stack/profiles/actions/README.md) 与 [`control/`](../shared/scenarios/panda_stack/profiles/control/README.md)。
- **运行端解耦**：运行端（[`runtime/ros2/`](../runtime/ros2/README.md)）仅通过标准 ROS 2 服务与话题访问后端，不导入任何具体物理引擎或硬件驱动代码。

`sim_api_msgs` 中的仿真复位和通用任务真值接口不属于真机必需接口；固定堆叠 Teacher 已迁至方案专用 `panda_stack_api_msgs`。通用控制无需加载任一可选包；现有以仿真任务结束为条件的采集配方仍需要仿真后端及其所选接口包。

## 接入准则

- 真机的设备 IP、序列号、账号凭证与单机标定参数属于本地私有配置，严禁提交至代码仓库。
- 在确认具体机械臂/夹爪硬件驱动、开源许可与安全急停边界前，不在本目录创建占位空模块或假驱动。

目录导航：[上级目录](../README.md)
