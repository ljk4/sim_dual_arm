# Schema

按版本保存机器读取的 JSON Schema，加载器按 `schema_version` 选择，未知版本拒绝。

- `v1/`：历史 mock profile、观测、动作、配置和 episode manifest。
- `v2/`：具名相机 profile 与 v2 episode manifest。
- `v3/`：原生 MOTION episode manifest。
- `v4/`：连续 Policy/受限跟随 episode manifest。

schema 只描述结构；关节/相机顺序和单位由 `profiles/` 与
`robot_contracts` 校验。

目录导航：[上级目录](../README.md)。
