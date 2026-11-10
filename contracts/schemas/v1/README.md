# Schema v1

包含以下严格 JSON Schema 定义：

- **机器人与数据语义**：`robot_profile.schema.json`（单相机 v1 格式）、`robot_profile_v2.schema.json`（当前具名多相机 `schema_version=2` 格式）、`observation.schema.json`、`action.schema.json`。
- **运行配置与产物**：`runtime_config.schema.json`、`episode_manifest.schema.json`（`schema_version=1`）。
- **结构化日志**：`run_event.schema.json`、`run_status.schema.json`。

各文档类型的 `schema_version` 独立演进，均开启 `additionalProperties: false` 严格校验。

目录导航：[上级目录](../README.md)
