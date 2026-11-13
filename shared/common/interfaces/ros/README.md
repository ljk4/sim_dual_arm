# ROS 2 接口层

- [`robot_api_msgs/`](robot_api_msgs/README.md)：真机与仿真共用的控制、观测、反馈与 Episode 接口。
- [`sim_api_msgs/`](sim_api_msgs/README.md)：可选的仿真复位与物体真值任务生命周期接口；真机无需实现或安装。
- 方案专用 Teacher：`shared/scenarios/<scenario>/extensions/ros/`，由 binding 的 `interfaces.extension_ros` 选择；`panda_stack` 使用 `panda_stack_api_msgs`，`marvin_handover` 使用 `marvin_handover_api_msgs`。

消息与服务由各包的 `CMakeLists.txt` 生成。`build-ros-sdk` 一次编译本目录全部公共包与所有方案扩展包，切换方案无需重编；SDK 完成标记记录实际包集与各包源码摘要。

目录导航：[上级目录](../../README.md)
