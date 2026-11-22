# sim_api_msgs

可选仿真任务接口，依赖 `robot_api_msgs/RunContext` 与 `builtin_interfaces`。
本包不属于真机控制的强制依赖，普通控制客户端未使用时不会加载它。

- [`msg/`](msg/README.md)：带物体真值和任务指标的 `TaskState`，以及 owner 线程一致接触/物块快照 `ContactSnapshot`。
- [`srv/`](srv/README.md)：`ResetSimulation`、`PrepareTask`、`StartTask`、`GetTaskState`、`GetContactSnapshot`，仅用于仿真任务管理与诊断，不承载 Teacher 动作块。

只由 simulation 后端实现。runtime 在复位或任务路径延迟加载这些类型。60×16 Teacher 动作块接口位于方案扩展包（如 `marvin_teacher_api_msgs`）。

目录导航：[上级目录](../README.md)
