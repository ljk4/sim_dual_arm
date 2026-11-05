# 机器人 profile

保存可版本化的机器人语义实例；部署地址、容器和设备绑定在 `deploy/`，schema 在
`contracts/schemas/`，加载/哈希在 `contracts/python/src/robot_contracts/`。

- `robots/mock_dual_arm_v1/`：历史单相机 CPU mock，`test_only=true`。
- `robots/panda_stack_v1/`：双 Panda、16 维关节、三相机基线。
- `robots/panda_stack_learning_v1/`：学习任务相机变体，右腕 FOV 68°。
- `observations/`、`actions/`：观测和动作绑定，顺序必须与 profile 完全一致。

profile 不包含主机路径或后端类型；改变关节/相机语义必须形成新摘要，不能沿用旧
hash。历史细节见 [DSH-018](../docs/history/README.md#dsh-018)。

目录导航：[上级目录](../README.md)。
