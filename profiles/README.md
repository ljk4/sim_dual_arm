# 机器人配置档案（Profiles）

本目录集中维护版本化的机器人物理语义档案、观测/动作绑定以及跨端共享的控制配置，不包含任何主机路径、IP 地址或容器绑定信息。

## 目录结构

- **[`robots/`](robots/README.md)**：机器人物理参数、关节限位与相机标定档案：
  - [`panda_stack_v1/`](robots/panda_stack_v1/README.md)：双 Panda 16 维关节 + 三路相机基线 Profile。
  - [`panda_stack_learning_v1/`](robots/panda_stack_learning_v1/README.md)：学习任务专用相机变体（右腕垂直视场角 68°）。
- **[`observations/`](observations/README.md)** 与 **[`actions/`](actions/README.md)**：观测与动作字段绑定（顺序与单位必须与对应 Robot Profile 严格一致）。
- **[`control/`](control/README.md)**：跨 `simulation` 与 `runtime` 共享的控制限值、受限跟随与流式超时配置单一数据源（SSOT）。

Robot Profile 统一使用 `schema_version=2`，对应唯一的 `robot_profile.schema.json`；旧单个 `camera` 格式已移除。现有档案目录中的 `_v1` 是档案标识，目录名和正式 JSON 内容保持不变。

结构约束由 [`contracts/schemas/v1/`](../contracts/schemas/v1/README.md) 定义，加载与语义哈希计算由 [`robot_contracts`](../contracts/python/src/robot_contracts/README.md) 实现。任何关节顺序、限位或相机标定的改动均会产生新的语义哈希，严禁新旧数据混用。

目录导航：[上级目录](../README.md)
