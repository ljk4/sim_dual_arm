# Policy API v1

`policy.proto` 定义 `PolicyService`（`GetPolicyInfo`、`OpenSession`、`Predict`、`CloseSession`）、`PolicyInfo` 元数据、多路相机与关节状态观测输入以及关节动作块输出。

- **动态元数据校验**：动作块长度（`chunk_size`）、步长周期（`action_period_ns`）、关节顺序与 `test_only` 标志由 `PolicyInfo` 声明（当前 Marvin ACT 配方为 60 点、50 ms、16 维），客户端与服务端在会话建立及每次预测时严格校验一致性。
- **时间基准**：`JointTarget.time_from_observation_ns` 表示相对于输入 `Observation` 机器人采样时刻的未来正向偏移量。
- **兼容性原则**：向后兼容变更仅新增字段，不兼容变更须升级协议版本，严禁静默改变既有字段语义。

目录导航：[上级目录](../README.md)
