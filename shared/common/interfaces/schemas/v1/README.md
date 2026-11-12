# Schema v1

包含以下严格 JSON Schema 定义：

- **机器人与数据语义**：`robot_profile.schema.json`（唯一 Robot Profile 契约，`schema_version=2`，具名相机数组）、`observation.schema.json`、`action.schema.json`。
- **运行产物**：`episode_manifest.schema.json`（`schema_version=1`）。Runtime 私有配置由 runtime 的 `RuntimeRecipe` 严格校验，公共目录不再维护重复定义。
- **结构化日志**：`run_event.schema.json`、`run_status.schema.json`。

各文档类型的 `schema_version` 独立演进，均开启 `additionalProperties: false` 严格校验。

Robot Profile 只接受 `schema_version=2`：使用 `cameras` 数组（1～8 路），每路都必须提供标定摘要。旧单个 `camera` 字段与 v1 Profile 不再支持。目录 `v1` 表示本组契约的集合版本，与各文件内部的格式版本独立；观测、动作和 Episode 等契约继续使用各自既有版本。

目录导航：[上级目录](../README.md)
