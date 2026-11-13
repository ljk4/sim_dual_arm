# 仿真任务服务

`ResetSimulation` 复位物理；`PrepareTask`、`StartTask` 管理仿真任务；`GetTaskState`
查询真值；`GetContactSnapshot` 返回 owner 线程捕获的一致接触/物块快照。这些接口服务仿真任务和诊断，不包含方案专用 Teacher 动作块。
`GetTeacherChunk` 已迁至 `shared/scenarios/panda_stack/extensions/ros/panda_stack_api_msgs`。
服务名称保持不变。

目录导航：[上级目录](../README.md)
