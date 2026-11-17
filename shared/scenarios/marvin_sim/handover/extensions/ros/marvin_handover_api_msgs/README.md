# Marvin 交接 Teacher 扩展

`GetTeacherChunk.srv` 提供 60×16 原始目标、50 ms 时间网格和交接阶段码
`0–10`（左接近、左抓取、左抬起、交接接近、双手接稳、左释放、左退开、
右运输、右放置、右释放、稳定）。服务名为 `<namespace>/get_task_chunk`，
能力标识 `marvin_handover_api_msgs.teacher_chunk.v1` 同时由
`msg/TeacherCapability.msg` 和方案 binding 的 `interfaces.teacher_api` 声明；
运行端只按该标识选择适配，不按方案名推断。

阶段码是交接任务自身约定，与 `panda_stack_api_msgs` 的九阶段码不通用。

目录导航：[方案说明](../../../README.md)
