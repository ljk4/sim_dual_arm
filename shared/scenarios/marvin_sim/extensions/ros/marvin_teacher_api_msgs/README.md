# Marvin Teacher 共享 ROS 扩展

`GetTeacherChunk.srv` 为 `marvin_sim` 各任务提供统一的 `60 × 16` 原始 Teacher 动作块、50 ms 相对时间网格及阶段码接口。服务名为 `<namespace>/get_task_chunk`，能力标识 `marvin_teacher_api_msgs.teacher_chunk` 同时由 `msg/TeacherCapability.msg` 与各任务 `binding.yaml` 的 `interfaces.teacher_api` 声明；阶段码顺序由具体任务约定（如 `stack` 为 `0–8`，`handover` 为 `0–10`）。

目录导航：[方案目录](../../../../README.md)
