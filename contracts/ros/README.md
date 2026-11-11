# ROS 2 接口层

- [`robot_api_msgs/`](robot_api_msgs/README.md)：真机与仿真共用的控制、观测、反馈与 Episode 接口。
- [`sim_api_msgs/`](sim_api_msgs/README.md)：仿真复位、程序示教及物体真值任务接口；真机无需实现或安装。

消息与服务由各包的 `CMakeLists.txt` 生成。runtime 容器内执行 `runtime/ros2/scripts/run.sh build-ros-sdk` 同时构建两个包，SDK 完成标记校验每个包的模块及契约源码摘要。服务名称不变；迁移后的类型名使用 `sim_api_msgs`。

目录导航：[上级目录](../README.md)
