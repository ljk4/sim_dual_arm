# dual_arm_runtime

ament_python 包。`package.xml` 与 `setup.py` 声明依赖和三个入口：
`runtime`、`native_runtime`、`inspect_episode`。Python 实现在
`dual_arm_runtime/`，`resource/dual_arm_runtime` 只是 ament 索引标记。

配置由 `runtime/ros2/configs/` 的 YAML 选择；ROS 消息来自
`contracts/ros/robot_api_msgs`。模块分组见
[dual_arm_runtime 包](dual_arm_runtime/README.md)。

目录导航：[上级目录](../README.md)。
