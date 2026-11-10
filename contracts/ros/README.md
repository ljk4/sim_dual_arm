# ROS 2 接口层

[`robot_api_msgs/`](robot_api_msgs/README.md) 是跨 `simulation`、`runtime` 与 `hardware` 共享的唯一 ROS 2 接口包。

- 消息与服务通过 `CMakeLists.txt` 编译生成至 `.build/ros-dev/`（容器内挂载为 `/opt/ros-dev`），各端直接导入已安装的接口模块，不在业务包中复制 `.msg` 或 `.srv` 文件。

目录导航：[上级目录](../README.md)
