# robot_api_msgs

双臂机器人公共 ROS 2 接口包：

- **[`msg/`](msg/README.md)**：机器人状态、相机帧、观测索引、流式命令、跟随参考、执行反馈与任务状态消息。
- **[`srv/`](srv/README.md)**：控制租约、Episode 生命周期、流式命令查询、受限跟随提交与任务/示教服务。

[`CMakeLists.txt`](CMakeLists.txt) 是接口编译与安装的唯一入口。修改接口定义后，须在 `runtime` 容器内重新运行 `runtime/ros2/scripts/run.sh build-ros-sdk`。

目录导航：[上级目录](../README.md)
