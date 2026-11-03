# Runtime

ROS 2 运行层：订阅 backend 状态与相机、组装观测、执行控制会话、写入 MCAP、
导出标签并处理停止和收尾。ROS 包在 `ros/dual_arm_runtime/`。

Runtime 不导入 backend 物理实现，也不训练模型。安装后的入口为 `runtime`、
`native_runtime` 和 `inspect_episode`；接口边界见
[DSH-000 接口材料](../../docs/history/dsh-000.md#src-docs-architecture-and-interfaces-md)。

目录导航：[上级目录](../README.md)。
