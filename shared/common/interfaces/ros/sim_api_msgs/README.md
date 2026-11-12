# sim_api_msgs

可选仿真任务接口，依赖 `robot_api_msgs/RunContext` 与 `builtin_interfaces`。
本包不属于真机控制的强制依赖，普通控制客户端未使用时不会加载它。

- [`msg/`](msg/README.md)：带物体真值和任务指标的 `TaskState`。
- [`srv/`](srv/README.md)：`ResetSimulation`、`PrepareTask`、`StartTask`、`GetTaskState`。

只由 simulation 后端实现。runtime 在复位或任务路径延迟加载这些类型。固定
60×16 堆叠 Teacher 已迁至方案专用 `panda_stack_api_msgs`，本包不再包含
`GetTeacherChunk`。

目录导航：[上级目录](../README.md)
