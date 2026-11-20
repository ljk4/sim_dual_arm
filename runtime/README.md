# 运行端（Runtime）

本目录按通信中间件组织机器人运行端实现。当前唯一实现为 ROS 2 Jazzy（[`ros2/`](ros2/README.md)），负责多路观测同步组装、控制会话与租约管理、流式动作块接续、MCAP 录制、离线审计与评估。当前方案的运行配方与选择见 [`marvin_sim/stack`](../shared/scenarios/marvin_sim/stack/README.md) 与 [`marvin_sim/handover`](../shared/scenarios/marvin_sim/handover/README.md)，公共接口约束见 [`shared/common/`](../shared/common/README.md)。

目录导航：[上级目录](../README.md)
