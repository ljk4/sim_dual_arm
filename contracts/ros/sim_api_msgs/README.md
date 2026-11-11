# sim_api_msgs

仿真专用 ROS 接口，依赖 `robot_api_msgs/RunContext` 与 `builtin_interfaces`。

- [`msg/`](msg/README.md)：带物体真值和任务指标的 `TaskState`。
- [`srv/`](srv/README.md)：`ResetSimulation`、`PrepareTask`、`StartTask`、`GetTaskState`、`GetTeacherChunk`。

只由 simulation 后端实现。runtime 在任务或显式复位路径延迟加载这些类型，通用控制客户端导入不要求安装本包。

目录导航：[上级目录](../README.md)
