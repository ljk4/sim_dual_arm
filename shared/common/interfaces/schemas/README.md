# JSON Schema 契约

本目录直接存放机器人 Profile、观测、动作、Episode Manifest、采集选择清单（`formal-select.json`）、导出结构（`labels.json`/`quality.json`）与运行事件的 JSON Schema。校验拒绝未声明字段。

- Schema 仅定义文档结构与基础类型约束；关节顺序、相机命名、物理单位与跨字段语义由方案 `profiles` 与 [`robot_contracts`](../../python/contracts/src/robot_contracts/README.md) 联合校验。

目录导航：[上级目录](../../README.md)
