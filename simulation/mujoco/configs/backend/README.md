# 后端启动配方模板（Backend）

- **`backend.yaml`**：基础双 Panda Robot API 后端配置（不加载特定任务插件）。
- **`backend_stack_teacher.yaml`**：启用堆叠任务评估器与程序示教（Teacher）的数据采集后端模板。
- **`backend_stack_act_chunked.yaml`**：分块 ACT 策略推理专用的后端模板（无 Teacher，引用学习场景变体与 `panda_follower_v1.yaml` 跟随预算）。
- **`backend_stack_inference.yaml`**：通用堆叠推理后端模板。

每份模板通过相对路径引用 `scene/`、`profiles/control/`、`profiles/robots/` 与 `contracts/schemas/v1/`。运行时须通过 `scripts/run.sh prepare` 生成带随机种子与运行清单的 `backend.yaml`。

目录导航：[上级目录](../README.md)
