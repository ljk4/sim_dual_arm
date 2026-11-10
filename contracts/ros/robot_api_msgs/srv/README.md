# ROS 2 服务定义（srv）

按职责划分为三组：

- **控制租约与生命周期**：`AcquireControl`、`RenewControl`、`ReleaseControl`、`StartEpisode`、`EndEpisode`、`Stop`、`ResetSimulation`。
- **流式与受限跟随**：`SubmitFollower`、`GetStreamAnchor`、`QueryStreamCommand`。
- **任务管理与程序示教**：`PrepareTask`、`StartTask`、`GetTaskState`、`GetTeacherChunk`。

目录导航：[上级目录](../README.md)
