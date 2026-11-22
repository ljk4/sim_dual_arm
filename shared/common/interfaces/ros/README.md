# ROS 2 接口层

- [`robot_api_msgs/`](robot_api_msgs/README.md)：真机与仿真共用的控制、观测、反馈与 Episode 接口。
- [`sim_api_msgs/`](sim_api_msgs/README.md)：可选的仿真复位与物体真值任务生命周期接口。
- 机器人/方案专用 Teacher：`shared/scenarios/<robot>/extensions/ros/`（或 `shared/scenarios/<robot>/<task>/extensions/ros/`），由 binding 的 `interfaces.extension_ros` 与 `interfaces.teacher_api` 选择；`marvin_sim/stack` 与 `marvin_sim/handover` 共用 `shared/scenarios/marvin_sim/extensions/ros/marvin_teacher_api_msgs/`（能力标识 `marvin_teacher_api_msgs.teacher_chunk`）。

消息与服务由各包的 `CMakeLists.txt` 生成。`runtime` 容器内的 `scripts/build_sdk.sh` 一次编译本目录全部公共包与所有方案扩展包；SDK 完成标记记录实际包集与各包源码摘要。

目录导航：[上级目录](../../README.md)
