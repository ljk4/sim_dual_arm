# 方案目录（Scenarios）

本目录按 `<robot-id>/<task-id>/` 两级结构组织可运行方案，通过各自目录下的 `binding.yaml` 选择档案、三端配方与专用接口：

- **[`_template/`](_template/README.md)**：新增方案说明与不可直接运行的 `binding.example.yaml` 模板。
- **`marvin_sim/`**：Marvin 固定基座双臂机器人方案集合：
  - `profiles/`：共享的观测档案、动作档案与控制/跟随参数（`observations/`、`actions/`、`control/`）。
  - `extensions/ros/marvin_teacher_api_msgs/`：共享 Teacher 扩展接口（能力标识 `marvin_teacher_api_msgs.teacher_chunk`）。
  - **[`marvin_sim/stack/`](marvin_sim/stack/README.md)**：单臂方块堆叠任务。
  - **[`marvin_sim/handover/`](marvin_sim/handover/README.md)**：抓取—空中交接—入箱任务。

目录导航：[上级目录](../README.md) · [跨端架构分工与解耦审查](../../docs/features/跨端架构分工与解耦审查.md)
