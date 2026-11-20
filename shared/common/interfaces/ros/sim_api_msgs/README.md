# sim_api_msgs

可选仿真任务接口，依赖 `robot_api_msgs/RunContext` 与 `builtin_interfaces`。
本包不属于真机控制的强制依赖，普通控制客户端未使用时不会加载它。

- [`msg/`](msg/README.md)：带物体真值和任务指标的 `TaskState`；P02B 增加 owner 线程一致接触/物块快照 `ContactSnapshot`。
- [`srv/`](srv/README.md)：`ResetSimulation`、`PrepareTask`、`StartTask`、`GetTaskState`；P02B 增加 `GetContactSnapshot`，只用于仿真端诊断，不承载 Teacher 动作块。

只由 simulation 后端实现。runtime 在复位或任务路径延迟加载这些类型。固定
60×16 堆叠与交接 Teacher 已迁至方案专用扩展包（`marvin_stack_api_msgs`、
`marvin_handover_api_msgs`），本包不再包含 `GetTeacherChunk`。

目录导航：[上级目录](../README.md)
