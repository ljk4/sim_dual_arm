# Schema v1

包括 `robot_profile`、`observation`、`action`、`runtime_config`、`run_event`、
`run_status` 和 `episode_manifest`。`robot_profile` 同时提供历史单相机
`schema_version=1` 与当前具名相机 `schema_version=2` 两种严格格式
（`robot_profile_v2.schema.json`）。版本按具体类型分别演进；旧记录仍按原字段
严格读取。

目录导航：[上级目录](../README.md)。
