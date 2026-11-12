# 后端启动配方模板（Backend）

- **`backend.yaml`**：基础双 Panda Robot API 后端配置（不加载特定任务插件）。
- **`backend_stack_teacher.yaml`**：启用堆叠任务评估器与程序示教（Teacher）的数据采集后端模板。
- **`backend_stack_act_chunked.yaml`**：方案与直接配置入口共用的 Policy 推理模板（无 Teacher，引用学习场景与 `panda_follower_v1.yaml` 跟随预算）。

每份模板通过相对路径引用 `scene/`、`shared/scenarios/panda_stack/profiles/{control,robots}/` 与 `shared/common/interfaces/schemas/v1/`。运行时须通过 `scripts/run.sh prepare` 生成带随机种子与运行清单的 `backend.yaml`。

目录导航：[方案说明](../../../README.md)
