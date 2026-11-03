# Policy API v1

`policy.proto` 定义 `PolicyService`、`PolicyInfo`、观测/相机输入和关节动作块。
块长、周期、关节顺序和 `test_only` 由元数据声明；本轮 ACT 配方使用 60 点、
50 ms 和 16 维，客户端与服务端需校验其一致性，数组大小并非 Protobuf 固定长度。
`JointTarget.time_from_observation_ns` 相对原 Observation 机器人时间。
兼容变更保留旧字段，不兼容时提升版本，不静默改变历史字段语义。

目录导航：[上级目录](../README.md)。
