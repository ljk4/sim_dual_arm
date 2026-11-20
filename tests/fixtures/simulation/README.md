# 仿真配置测试夹具（Simulation Fixtures）

存放专供单元测试与历史回归对照使用的配置文件：

- **`backend_follower.yaml`**：验证基础仿真后端受限跟随接口的最小配置。
- **`backend_stack_act.yaml`**：保存早期 ACT 时序参数，用于与当前分块配方（`backend_stack_policy.yaml`）进行回归对照测试。
- **`policy_unit.yaml`**：Policy 服务配置解析单元测试夹具。

这些夹具引用的场景、控制限值与 Robot Profile 均指向仓库当前有效源码。

目录导航：[上级目录](../../README.md)
