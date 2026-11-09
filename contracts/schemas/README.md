# Schema

`v1/` is the single current schema directory; unknown versions are rejected.

- `v1/`：当前机器人 profile、观测、动作、配置、运行日志和 episode manifest。
- `v1/robot_profile_v2.schema.json`：仍在使用中的 schema_version=2 机器人 profile
  格式；profile 文档版本与 episode manifest 版本彼此独立。

schema 只描述结构；关节/相机顺序和单位由 `profiles/` 与
`robot_contracts` 校验。

目录导航：[上级目录](../README.md)。
