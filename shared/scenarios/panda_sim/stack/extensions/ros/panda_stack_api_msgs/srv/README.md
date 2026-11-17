# 堆叠 Teacher 服务

`GetTeacherChunk.srv` 保持原有 60×16 载荷、50 ms 时间网格和 0–8 堆叠阶段码。
服务名仍为 `<namespace>/get_task_chunk`；消息类型迁移到
`panda_stack_api_msgs/srv/GetTeacherChunk`，能力标识为
`panda_stack_api_msgs.teacher_chunk.v1`。旧 `sim_api_msgs` 同名类型不再提供。

目录导航：[上级目录](../README.md)
