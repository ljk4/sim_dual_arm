# Marvin 堆叠 Teacher 扩展

`GetTeacherChunk.srv` 提供 60×16 原始目标、50 ms 时间网格与九阶段堆叠码
`0–8`（`pregrasp`、`grasp`、`close`、`lift`、`transport`、`lower`、`open`、
`retreat`、`stable`）。服务名为 `<namespace>/get_task_chunk`，
能力标识 `marvin_stack_api_msgs.teacher_chunk.v1` 同时由
`msg/TeacherCapability.msg` 与方案 binding 的 `interfaces.teacher_api` 声明；
运行端按该标识动态加载适配，不按方案名硬编码。

目录导航：[方案说明](../../../README.md)
